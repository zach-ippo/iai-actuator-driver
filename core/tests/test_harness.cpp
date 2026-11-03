// Self-test of the harness and of the header-only pieces every unit relies on (no implementation unit needed).
#include <exception>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"

TEST(harness_checks_pass) {
    CHECK(1 + 1 == 2);
    CHECK_EQ(2u, 2);
    CHECK_EQ(std::string("a"), "a");
    CHECK_NEAR(0.1 + 0.2, 0.3, 1e-9);
    CHECK_THROWS(throw std::runtime_error("x"));
    CHECK_THROWS(throw iai_actuator_driver::SoftLimitError("x"), iai_actuator_driver::EcError);
    REQUIRE(true);
}

TEST(harness_header_only_helpers) {
    // STOP broadcast frame quoted by the Python edition: 00 05 04 2C FF 00 4D 12
    const std::vector<std::uint8_t> frame = iai_actuator_driver::build_frame(0, {0x05, 0x04, 0x2C, 0xFF, 0x00});
    REQUIRE(frame.size() == 8);
    CHECK_EQ(frame[6], 0x4D);
    CHECK_EQ(frame[7], 0x12);
    CHECK(iai_actuator_driver::check_crc(frame));
    CHECK_EQ(iai_actuator_driver::reg::from_i32_words(0xFFFF, 0xFFFF), -1);
    CHECK_EQ(iai_actuator_driver::reg::push_power(70), 0xB2);
    iai_actuator_driver::ModbusError e("no answer");
    e.set_attempts(4);
    e.add_note("decel stop sent");
    CHECK_EQ(std::string(e.what()), "no answer (gave up after 4 attempts) [decel stop sent]");
}

TEST(harness_json_and_inline_bus_helpers) {
    iai_actuator_driver::Json j = iai_actuator_driver::Json::object();
    j.set("name", "COM3:1").set("position_mm", 50.25).set("alarm", 0).set("velocity_mm_s", std::optional<double>());
    j.set("rows", std::vector<int>{0, 1, 2});
    const std::string text = j.dump();
    CHECK_EQ(text, std::string(R"({"name":"COM3:1","position_mm":50.25,"alarm":0,"velocity_mm_s":null,"rows":[0,1,2]})"));
    const iai_actuator_driver::Json back = iai_actuator_driver::Json::parse(j.dump(2));
    CHECK(back == j);
    CHECK_EQ(back.find("rows")->size(), 3u);
    CHECK_THROWS(iai_actuator_driver::Json::parse("{\"a\":}"), std::invalid_argument);
    CHECK_EQ(iai_actuator_driver::axis_name("COM3", 1), "COM3:1");
    CHECK_EQ(iai_actuator_driver::default_scan_slaves().size(), 16u);
    iai_actuator_driver::SerialPortInfo info;
    info.port = "COM3";
    info.vid = 0x10C4;
    info.pid = 0x81D7;
    CHECK(info.is_iai());
    info.vid.reset();
    info.description = "iai USB to UART Bridge Controller";
    CHECK(info.is_iai());
}

TEST(harness_each_run_has_its_own_temp_directory) {
    // test_main.cpp: several ec_tests at once (parallel builds of one checkout) never share the fixed file names the
    // tests write there
    const std::filesystem::path temp = std::filesystem::temp_directory_path();
    const std::string name = (temp.has_filename() ? temp : temp.parent_path()).filename().string();
    CHECK_EQ(name.rfind("ec_tests_", 0), 0u);
    CHECK(std::filesystem::is_directory(temp));
}

TEST(harness_freed_memory_quarantine_takes_any_mix_of_block_sizes) {
    // test_main.cpp (EC_TESTS_POISON): past its byte cap (large blocks) and its block cap (many small ones) the oldest
    // blocks leave the quarantine in order: none is lost, freed twice or read from an empty slot
    std::vector<char*> large;
    for (int round = 0; round < 2; ++round) {
        for (int i = 0; i < 160; ++i) large.push_back(new char[std::size_t(1) << 20]);
        for (char* p : large) delete[] p;
        large.clear();
        std::vector<int*> small(4096);
        for (int k = 0; k < 300; ++k) {  // 1.2 million blocks
            for (std::size_t i = 0; i < small.size(); ++i) small[i] = new int(k);
            for (int* p : small) delete p;
        }
    }
    CHECK(large.empty());
}

TEST(harness_thread_reports_what_its_body_threw) {
    // ec_test::Thread: what the body throws is a failure of the running test with its what() text (at the join or in
    // the destructor, once), or the test's data when taken with error(); never std::terminate
    struct CapturedErrors {  // the expected failure lines, out of the test's own output
        std::ostringstream text;
        std::streambuf* const saved = std::cerr.rdbuf(text.rdbuf());
        ~CapturedErrors() { std::cerr.rdbuf(saved); }
    };
    const int before = ec_test::current().failures;
    std::string said;
    {
        const CapturedErrors captured;
        {
            ec_test::Thread lost([] { throw std::runtime_error("thrown and never joined"); });
        }  // joined and reported by the destructor
        ec_test::Thread joined([] { throw std::runtime_error("thrown before the join"); });
        joined.join();
        joined.join();  // reported once
        said = captured.text.str();
    }
    const int reported = ec_test::current().failures - before;
    ec_test::current().failures = before;  // expected here: not failures of this test
    CHECK_EQ(reported, 2);
    CHECK(said.find("exception in a test thread: thrown and never joined") != std::string::npos);
    CHECK(said.find("exception in a test thread: thrown before the join") != std::string::npos);
    ec_test::Thread fine([] {});
    CHECK(fine.error() == nullptr);
    ec_test::Thread taken([] { throw std::logic_error("taken as data"); });
    const std::exception_ptr error = taken.error();
    REQUIRE(error != nullptr);
    CHECK_EQ(ec_test::describe(error), std::string("taken as data"));
    taken.join();  // taken: nothing left to report
    CHECK_EQ(ec_test::current().failures, before);
}
