// iad_call_json (src/c_api_tables.cpp): the core's stateless functions by name for the bindings (the alarm and CON
// parameter tables and rules, the info decoders), with Python types kept in the JSON. The bindings' conformance tests
// (bindings/python/tests/test_tables_conformance.py) compare every code and parameter with the Python edition; these
// check the entry point itself: names, batches, errors, number types. No I/O.
#include <string>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/json.hpp"
#include "harness.hpp"

extern "C" iad_result IAD_CALL iad_call_json(const char* function, const char* args_json, char* buffer,
                                             size_t buffer_size, size_t* required);

using namespace iai_actuator_driver;

namespace {

/// The output of one call, or "error <code>: <message>".
std::string call(const char* function, const char* args) {
    size_t required = 0;
    std::string buffer(64, '\0');
    iad_result r = iad_call_json(function, args, &buffer[0], buffer.size(), &required);
    if (r == IAD_ERR_BUFFER_TOO_SMALL) {
        buffer.assign(required, '\0');
        r = iad_call_json(function, args, &buffer[0], buffer.size(), &required);
    }
    if (r != IAD_OK) return "error " + std::to_string(r) + ": " + iad_last_error_message();
    return buffer.c_str();
}

}  // namespace

TEST(capi_tables_unknown_function_is_unsupported) {
    CHECK(call("alarms.nope", nullptr).find("error -11") == 0);
    CHECK(call("functions", nullptr).find("\"con_params.parse\"") != std::string::npos);
}

TEST(capi_tables_alarm_rules_with_the_family) {
    CHECK_EQ(call("alarms.alarm_level", R"({"code": 44})"), std::string("\"unknown\""));
    CHECK_EQ(call("alarms.alarm_level", R"({"code": 44, "family": "pcon"})"), std::string("\"message\""));
    CHECK_EQ(call("alarms.alarm_level", R"({"code": 44, "system_code": 172})"), std::string("\"message\""));
    CHECK_EQ(call("alarms.alarm_family", R"({"system_code": 174, "family": "EC"})"), std::string("null"));
    const Json pcon = Json::parse(call("alarms.alarm_info", R"({"code": 206, "family": "PCON"})"));
    CHECK(pcon.find("remedy")->as_string().find("On a PCON-CB an alarm reset is enough") != std::string::npos);
    CHECK_EQ(pcon.find("family")->as_string(), std::string("PCON"));
    const Json any = Json::parse(call("alarms.alarm_info", R"({"code": 206})"));
    CHECK(any.find("remedy")->as_string().find("SCON-CB") != std::string::npos);
    CHECK(any.find("family")->is_null());
    // the EC's maintenance-warning clearing only on an EC
    CHECK(alarm_info(0x04E).reset.find("maintenance warning 1") == 0);
    CHECK(alarm_info(0x04E, std::nullopt, std::nullopt, std::nullopt, 0xAE).reset.find("alarm reset") == 0);
}

TEST(capi_tables_batches_and_python_number_types) {
    CHECK_EQ(call("alarms.alarm_name", R"([{"code": 0}, {"code": 4095}])"),
             std::string("[\"No alarm (in normal condition)\",\"PowerUP (no error)\"]"));
    const std::string tables = call("con_params.tables", nullptr);
    // No.22 Home offset: lo 0.0 (a float in the Python edition), hi 9999.99; No.8: lo 1, hi 9999 (ints)
    CHECK(tables.find("\"no\":22,") != std::string::npos);
    CHECK(tables.find("\"lo\":0.0,\"hi\":9999.99") != std::string::npos);
    CHECK(tables.find("\"lo\":1,\"hi\":9999,") != std::string::npos);
    const std::string p8 = R"({"no": 8, "name": "Default velocity", "symbol": "VCMD", "class_id": 8, "item_id": 10,
        "kind": "scaled", "scale": 0.01, "unit": "mm/s", "lo": 1, "hi": 9999})";
    CHECK_EQ(call("con_params.decode", (R"({"raw": 25000, "p": )" + p8 + "}").c_str()), std::string("250.0"));
    CHECK_EQ(call("con_params.parse", (R"({"value": 10000, "p": )" + p8 + "}").c_str()),
             std::string("error -10: No.8: 10000.0 outside 1..9999 mm/s"));
    CHECK_EQ(call("con_params.parse", (R"({"value": true, "p": )" + p8 + "}").c_str()),
             std::string("error -10: No.8: pass a number or a label, not True"));
    CHECK_EQ(call("con_params.parse", (R"({"value": {"float": "inf"}, "p": )" + p8 + "}").c_str()),
             std::string("error -10: No.8: inf is not a finite number"));
}
