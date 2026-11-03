// Axis unit: open (baud detection, the no-answer hint), connect / identify on every simulator profile, the rotary and
// unknown units, the status fallbacks and the read-only reads. Simulator and FakeTransport only (no COM port).
#include <memory>
#include <string>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::shared_ptr<SimulatedUnit> make_sim(SimProfile profile, double scale = 20.0) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_sim(const std::shared_ptr<SimulatedUnit>& sim, AxisOptions axis = {}) {
    OpenOptions o;
    o.slave = sim->slave();
    o.axis = axis;
    return Axis::open(sim, o);
}

/// The 22 status words of a unit at rest: servo on, homed, PEND, PMSS, PARW idle, drive power on.
std::vector<std::uint16_t> rest_words() {
    std::vector<std::uint16_t> w(22, 0);
    w[5] = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Pend | reg::kDss1Pwr;  // DSS1
    w[6] = reg::kDss2Enbs;                                                    // DSS2: enable switch pressed
    w[7] = reg::kDssePmss | reg::kDsseParw | reg::kDssePsns;                  // DSSE
    w[9] = 0x0001;                                                            // STAT low word: MPOW
    return w;
}

/// FC03 answer of `count` words from `words` at 0x9000.
Bytes read_answer(int slave, const std::vector<std::uint16_t>& words, int count) {
    Bytes pdu{0x03, static_cast<std::uint8_t>(2 * count)};
    for (int i = 0; i < count; ++i) {
        const std::uint16_t v = i < static_cast<int>(words.size()) ? words[i] : 0;
        pdu.push_back(static_cast<std::uint8_t>(v >> 8));
        pdu.push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
    return build_frame(slave, pdu);
}

}  // namespace

TEST(axis_connect_identify_every_profile) {
    for (SimProfile p : sim_profiles()) {
        auto sim = make_sim(p);
        Axis ax = open_sim(sim);
        const DeviceInfo info = ax.connect();
        CHECK(ax.device_info().has_value());
        CHECK(info.system_code.has_value());
        CHECK(info.soft_limits_mm.has_value());
        CHECK(info.max_speed_mm_s.has_value());
        const AxisRanges r = ax.limits();
        REQUIRE(r.position.min && r.position.max);
        CHECK(*r.position.max > *r.position.min);
        CHECK(r.speed.max.value_or(0) > 0);
        CHECK(ax.max_positions() >= (ax.is_ec() ? 3 : 1));  // an EC always keeps its rows 0-2
        const Status st = ax.status();
        CHECK(st.modbus_enabled);
        CHECK_NEAR(st.position_mm, sim->position_mm(), 0.011);
        CHECK(!ax.to_string().empty());
        if (p == SimProfile::Rotary) {
            CHECK_EQ(ax.units(), "deg");
            CHECK_EQ(info.units, "deg");
        } else {
            CHECK_EQ(ax.units(), "mm");
        }
        if (p == SimProfile::Pcon) {
            CHECK_EQ(ax.product_family(), "PCON");
            CHECK(!ax.is_ec());
            CHECK_EQ(ax.max_positions(), sim->position_count());
        } else if (p != SimProfile::Unknown) {
            CHECK(ax.is_ec());
        }
    }
}

TEST(axis_connect_rotary_unit_moves_in_degrees) {
    auto sim = make_sim(SimProfile::Rotary);
    Axis ax = open_sim(sim, AxisOptions{});
    ax.connect();
    CHECK_EQ(ax.kind(), "rotary");
    const AxisRanges r = ax.limits();
    CHECK_EQ(r.units, "deg");
    CHECK_EQ(r.position.unit, "deg");
    MoveOptions o;
    o.speed = 90.0;
    const auto st = ax.move_to(90.0, o);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 90.0, 0.1);
}

TEST(axis_connect_unknown_unit_is_driven_not_refused) {
    auto sim = make_sim(SimProfile::Unknown);
    Axis ax = open_sim(sim);
    const DeviceInfo info = ax.connect();
    CHECK_EQ(*info.system_code, 0x99);  // reported as what it says it is
    CHECK_EQ(*info.unit_code, 0x7E);
    bool unverified = false;
    for (const auto& e : info.errors)
        if (e.find("UNVERIFIED") != std::string::npos || e.find("tried on hardware") != std::string::npos)
            unverified = true;
    CHECK(!unverified);  // no label on the unit, and never a refusal
    MoveOptions o;
    o.speed = 20.0;
    const auto st = ax.move_to(5.0, o);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 5.0, 0.1);
}

TEST(axis_connect_takes_modbus_control_at_rest) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Gripper;
    so.time_scale = 20.0;
    so.pmsl = false;  // as after every power-up
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = open_sim(sim);
    CHECK(!ax.status().modbus_enabled);
    ax.connect(false);
    CHECK(!sim->pmsl());  // take_control = false: nothing written
    ax.connect(true);
    CHECK(sim->pmsl());
    CHECK(ax.status().modbus_enabled);
    ax.disable_modbus_control();
    CHECK(!sim->pmsl());
    const auto st = ax.enable_modbus_control();
    REQUIRE(st.has_value());
    CHECK(st->modbus_enabled);
}

TEST(axis_connect_pcon_pc_control_is_taken_back_while_paused_at_rest_in_pio_mode) {
    // The PCON with nothing on its PIO connector: in PIO mode the servo goes off and DSS1 STP (the unwired *STP input)
    // shows at rest (VERIFIED on the PCON-CB, 2026-11-02). Taking PC control back must not be refused for that pause.
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    Status st = ax.set_action_mode(false);
    CHECK(!st.modbus_enabled);
    st = ax.status();
    CHECK(st.paused());
    CHECK(!st.servo_on);
    CHECK(!st.moving);
    CHECK(!st.idle());  // the pause still counts for idle(): only the control switch looks at motion instead
    st = ax.set_action_mode(true);
    CHECK(st.modbus_enabled);
    CHECK(!st.paused());  // Modbus control back clears the PIO pause; the servo stays off
    CHECK(!st.servo_on);
    ax.disable_modbus_control();
    CHECK(ax.status().paused());
    const auto on = ax.enable_modbus_control();
    REQUIRE(on.has_value());
    CHECK(on->modbus_enabled);
    ax.disable_modbus_control();
    ax.connect(true);  // the session start takes it too
    CHECK(sim->pmsl());
    ax.disable_modbus_control();
    ax.servo(true);  // a command takes control first (auto_modbus_control), then the servo comes on
    st = ax.status();
    CHECK(st.modbus_enabled);
    CHECK(st.servo_on);
    CHECK(st.idle());
}

namespace {
template <class F>
std::string refusal_of(F&& f) {
    try {
        f();
    } catch (const EcNotReadyError& e) {
        return e.what();
    }
    return std::string();
}
bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }
}  // namespace

TEST(axis_connect_pcon_refusals_name_the_pio_mode_or_the_pause_not_a_move) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.set_action_mode(false);  // PIO mode, nothing on the PIO connector: paused at rest, servo off
    sim->clear_requests();
    for (const std::string& text : {refusal_of([&] { ax.disable_modbus_control(); }),
                                    refusal_of([&] { ax.set_action_mode(false); }),
                                    refusal_of([&] { ax.software_reset(); })}) {
        CHECK(has(text, "PIO mode (PMSS 0)"));
        CHECK(has(text, "*STP"));
        CHECK(!has(text, "is moving"));
        CHECK(!has(text, "stop() it"));
    }
    CHECK(has(refusal_of([&] { ax.software_reset(); }), "take PC control"));
    // paused by the driver in PC mode: the pause is named, with resume()
    ax.set_action_mode(true);
    ax.pause();
    for (const std::string& text : {refusal_of([&] { ax.disable_modbus_control(); }),
                                    refusal_of([&] { ax.set_action_mode(false); }),
                                    refusal_of([&] { ax.software_reset(); })}) {
        CHECK(has(text, "paused (DSS1 STP on)"));
        CHECK(has(text, "resume()"));
        CHECK(!has(text, "is moving"));
    }
    const std::string changed = refusal_of([&] { ax.change_speed(20.0); });  // not "no move is in progress"
    CHECK(has(changed, "paused (DSS1 STP on)"));
    CHECK(has(changed, "nothing sent"));
    ax.resume();
}

TEST(axis_connect_pcon_servo_on_after_taking_pc_control_with_the_action_mode) {
    // On the PCON-CB every 0x45 cmd 0x0C with op1 = 1 cleared DSSE b12, and servo ON was then ignored (no alarm, no cause
    // shown) until a PMSL coil OFF (VERIFIED, 2026-11-02; the simulator's PCON models it). set_action_mode(PC) takes
    // control with the PMSL coil and changes the safety speed with the SFTY coil instead, so the servo comes on.
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.set_action_mode(false);  // PIO with nothing on the PIO connector: paused, servo off
    CHECK(!ax.status().servo_on);
    sim->clear_requests();
    Status st = ax.set_action_mode(true);
    CHECK(st.modbus_enabled);
    int action_pc = 0;
    for (const Bytes& f : sim->requests())
        if (f.size() >= 8 && f[1] == 0x45 && f[3] == reg::kIaiCmdActionMode && f[5] == 1) ++action_pc;
    CHECK_EQ(action_pc, 0);  // no op1 = 1
    std::optional<Status> on = ax.servo(true);
    REQUIRE(on.has_value());
    CHECK(on->servo_on);
    ax.servo(false);
    st = ax.set_action_mode(true, true);  // already in PC: nothing for control, the safety speed by its coil
    CHECK(st.modbus_enabled);
    CHECK(st.safety_speed());
    st = ax.set_action_mode(true, false);
    CHECK(!st.safety_speed());
    on = ax.servo(true);
    REQUIRE(on.has_value());
    CHECK(on->servo_on);
    ax.servo(false);
    // the model itself: the action mode PC by its frame blocks servo ON until a PMSL coil OFF
    send_action_mode(*ax.bus(), 1, true, false);
    bool threw = false;
    try {
        ax.servo(true, true, 1.0);
    } catch (const ServoBlockedError& e) {  // PMSS 1 and b12 0: the cause and the remedy, not "no known cause"
        threw = true;
        const std::string what = e.what();
        CHECK(what.find("DSSE b12 reads 0") != std::string::npos);
        CHECK(what.find("PMSL OFF, then ON") != std::string::npos);
        CHECK(what.find("no known cause") == std::string::npos);
        REQUIRE(e.reasons().size() == 1u);
    }
    CHECK(threw);
    CHECK(!sim->servo_on());
    send_modbus_control(*ax.bus(), 1, false);
    send_modbus_control(*ax.bus(), 1, true);
    on = ax.servo(true);
    REQUIRE(on.has_value());
    CHECK(on->servo_on);
}

TEST(axis_connect_pc_control_is_still_refused_while_a_paused_axis_moves) {
    // PMSS 0 and STP set: the pause alone is no refusal when the servo is off, but the servo on (a PLC holding *STP), a
    // MOVE bit, a home return, a push or a VNOW above 0 still is, and handing control to PIO while paused stays refused.
    // Nothing is written when refused.
    struct Case {
        const char* name;
        bool servo;
        std::uint16_t dsse;
        std::uint16_t vnow;
        bool refused;
    };
    const Case cases[] = {
        {"at a standstill, servo off", false, 0, 0, false},
        {"at a standstill, servo on", true, 0, 0, true},
        {"MOVE", false, reg::kDsseMove, 0, true},
        {"homing", false, reg::kDsseGhms, 0, true},
        {"VNOW", false, 0, 300, true},
        {"push", false, reg::kDssePush, 0, true},
        {"push, servo on", true, reg::kDssePush, 0, true},
    };
    for (const Case& c : cases) {
        auto words = rest_words();
        words[5] = static_cast<std::uint16_t>((c.servo ? reg::kDss1Sv : 0) | reg::kDss1Hend | reg::kDss1Stp | reg::kDss1Pwr);
        words[7] = static_cast<std::uint16_t>(reg::kDsseParw | reg::kDssePsns | c.dsse);  // PMSS 0: PIO mode
        words[11] = c.vnow;
        auto fake = std::make_shared<FakeTransport>([&words](const Bytes& f) -> std::optional<Bytes> {
            if (f.size() >= 6 && f[1] == 0x03 && f[2] == 0x90 && f[3] == 0x00) return read_answer(f[0], words, f[5]);
            if (f.size() >= 6 && f[1] == 0x05) return f;  // FC05 echo
            return std::nullopt;
        });
        Axis ax(std::make_shared<ModbusRtu>(fake), 1);
        CHECK(ax.status().paused());
        fake->clear_writes();
        if (c.refused) {
            CHECK_THROWS(ax.enable_modbus_control(false), EcNotReadyError);
            CHECK_THROWS(ax.set_action_mode(true), EcNotReadyError);
            CHECK_THROWS(ax.servo(true), EcNotReadyError);  // the automatic take-over looks at the same rule
        } else {
            ax.enable_modbus_control(false);
        }
        int coils = 0;
        for (const Bytes& f : fake->writes())
            if (f.size() >= 6 && (f[1] == 0x05 || f[1] == 0x45)) ++coils;
        CHECK_EQ(coils, c.refused ? 0 : 1);  // one PMSL write, none when refused
        if (!c.refused) {
            // in PC mode, paused at a standstill: handing control to PIO keeps the old rule (idle())
            words[7] = static_cast<std::uint16_t>(words[7] | reg::kDssePmss);
            fake->clear_writes();
            CHECK_THROWS(ax.disable_modbus_control(), EcNotReadyError);
            CHECK_THROWS(ax.set_action_mode(false), EcNotReadyError);
            CHECK(fake->writes().size() <= 2);  // status reads only
            for (const Bytes& f : fake->writes()) CHECK_EQ(f[1], 0x03);
        }
    }
}

TEST(axis_connect_open_detects_the_baud_rate_in_place) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Rod;
    so.time_scale = 20.0;
    so.baud_rate = 115200;       // the PC side starts here
    so.unit_baud_rate = 38400;   // the unit is at its power-up rate
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    CHECK(ax.baud_detected());
    CHECK_EQ(ax.baud_rate().value_or(0), 38400);
    CHECK(ax.owns_bus());
    CHECK_NEAR(ax.status().position_mm, 0.0, 0.01);
    ax.close();
    ax.close();  // safe twice
}

TEST(axis_connect_slave_58_is_refused_before_anything_is_sent) {
    // 0x3A = ':', the Modbus ASCII start: every IAI unit of the line would raise alarm 0x05E (ON-WIRE on a PCON-CB)
    auto fake = std::make_shared<FakeTransport>(nullptr, 115200);
    OpenOptions o;
    o.slave = 58;
    std::string text;
    try {
        Axis::open(fake, o);
    } catch (const std::invalid_argument& e) {
        text = e.what();
    }
    CHECK(text.find("slave 58 (0x3A) is never used") != std::string::npos);
    CHECK(text.find("nothing sent") != std::string::npos);
    CHECK(fake->writes().empty());
    auto bus = std::make_shared<ModbusRtu>(fake);
    CHECK_THROWS(Axis(bus, 58), std::invalid_argument);
    CHECK(fake->writes().empty());
}

TEST(axis_connect_a_direct_line_refuses_another_position_unit) {
    // ON-WIRE on a PCON-CB: a stated 0.001 on a direct line read P0 94.99 as 9.499 (and targets would go x 10)
    CHECK_THROWS(Line(LineKind::Direct, nullptr, false, 0.001), std::invalid_argument);
    Line direct(LineKind::Direct);
    CHECK_THROWS(direct.set_position_unit(0.001), std::invalid_argument);
    CHECK_THROWS(direct.set_position_unit(0.001, 1, kUnitRead), std::invalid_argument);
    direct.set_position_unit(0.01);  // the unit's own: accepted
    CHECK_EQ(direct.require_motion(1), 0.01);
    LineOptions stated;
    stated.kind = "direct";
    stated.position_unit_mm = 0.001;
    auto fake = std::make_shared<FakeTransport>(nullptr, 115200);
    CHECK_THROWS(Port(fake, "P", BusOptions{}, stated), std::invalid_argument);
    CHECK(fake->writes().empty());
    // detected: the line probe finds no gateway, so the line is direct and the open is refused before any write
    for (const char* kind : {"auto", "direct"}) {
        auto sim = make_sim(SimProfile::Pcon);
        OpenOptions o;
        o.slave = sim->slave();
        o.line.kind = kind;
        o.line.position_unit_mm = 0.001;
        std::string text;
        try {
            Axis::open(sim, o);
        } catch (const std::invalid_argument& e) {
            text = e.what();
        }
        CHECK(text.find("position unit 0.001 mm refused on a direct line") != std::string::npos);
        for (const auto& f : sim->requests()) CHECK(f.size() > 1 && (f[1] == 0x03 || f[1] == 0x11));  // reads only
    }
    auto sim = make_sim(SimProfile::Pcon);
    OpenOptions o;
    o.slave = sim->slave();
    o.line.position_unit_mm = 0.01;
    Axis ax = Axis::open(sim, o);
    CHECK_NEAR(ax.status().position_mm, sim->position_mm(), 0.01);
}

TEST(axis_connect_open_without_answer_raises_the_hint) {
    auto fake = std::make_shared<FakeTransport>(nullptr, 115200);
    bool threw = false;
    try {
        Axis::open(fake);
    } catch (const ModbusTimeout& e) {
        threw = true;
        CHECK(std::string(e.what()).find("24 V supply") != std::string::npos);
    }
    CHECK(threw);
    CHECK(!fake->is_open());  // the port is closed again
}

TEST(axis_connect_status_falls_back_to_shorter_reads) {
    const auto words = rest_words();
    auto fake = std::make_shared<FakeTransport>([&words](const Bytes& f) -> std::optional<Bytes> {
        if (f.size() >= 6 && f[1] == 0x03 && f[2] == 0x90 && f[3] == 0x00) {
            const int count = f[5];
            if (count > reg::kStatusMinWords) return build_frame(f[0], {0x83, 0x02});  // refused
            return read_answer(f[0], words, count);
        }
        return std::nullopt;
    });
    Axis ax(std::make_shared<ModbusRtu>(fake), 1);
    const Status st = ax.status();
    CHECK(st.servo_on);
    CHECK(st.homed);
    CHECK_EQ(st.raw.words, 8);
    const auto writes = fake->writes();
    REQUIRE(writes.size() >= 3);
    CHECK_EQ(writes[0][5], 22);
    CHECK_EQ(writes[1][5], 20);
    CHECK_EQ(writes[2][5], 8);
    fake->clear_writes();
    ax.status();  // remembered: one read of 8 words
    REQUIRE(fake->writes().size() == 1);
    CHECK_EQ(fake->writes()[0][5], 8);
}

TEST(axis_connect_velocity_is_derived_from_position_samples) {
    auto sim = make_sim(SimProfile::Slider, 1.0);
    Axis ax = open_sim(sim, AxisOptions{});
    MoveOptions o;
    o.speed = 100.0;
    o.wait = false;
    ax.move_to(100.0, o);
    sleep_at_least(0.1);
    ax.status();
    sleep_at_least(0.05);
    const Status st = ax.status();
    CHECK(st.moving);
    REQUIRE(st.velocity_mm_s.has_value());
    CHECK(*st.velocity_mm_s > 20.0);
    ax.stop();
}

TEST(axis_connect_alarm_history_and_rows) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    const auto history = ax.read_alarm_history();
    const auto all = ax.read_alarm_history(true);
    CHECK(all.size() >= history.size());
    CHECK_EQ(all.size(), static_cast<std::size_t>(reg::kAlarmHistoryEntries));
    for (std::size_t i = 1; i < history.size(); ++i) CHECK(history[i].index > history[i - 1].index);
    const PositionEntry row0 = ax.read_position_table(0);
    CHECK_NEAR(row0.position_mm, 3.0, 0.001);
    CHECK(row0.checksum_ok().value_or(false));
    const auto words = ax.read_registers(reg::kMonitorBase, 2);
    CHECK_EQ(words.size(), 2u);
    CHECK(ax.read_boot_baud_rate().value_or(0) == 38400);
    const auto eff = ax.effective_stroke();
    REQUIRE(eff.has_value());
    CHECK_NEAR(eff->max, 9.5, 0.01);
}

TEST(axis_connect_configured_stroke_only_narrows) {
    auto sim = make_sim(SimProfile::Slider);
    AxisOptions o;
    o.stroke = Span{10.0, 100.0};
    Axis ax = open_sim(sim, o);
    const auto eff = ax.effective_stroke();
    REQUIRE(eff.has_value());
    CHECK_NEAR(eff->min, 10.0, 1e-9);
    CHECK_NEAR(eff->max, 100.0, 1e-9);
    o.stroke = Span{-50.0, 500.0};  // wider than the unit's: the unit's own range applies
    ax.set_options(o);
    const auto wide = ax.effective_stroke();
    REQUIRE(wide.has_value());
    CHECK_NEAR(wide->min, 0.0, 0.01);
    CHECK_NEAR(wide->max, 150.0, 0.01);
    AxisOptions bad;
    bad.stroke = Span{5.0, 1.0};
    CHECK_THROWS(ax.set_options(bad), std::invalid_argument);
    CHECK_THROWS(Axis(ax.bus(), 0), std::invalid_argument);
}

TEST(axis_connect_error_type_names) {
    CHECK_EQ(detail::error_type_name(SoftLimitError("x")), "SoftLimitError");
    CHECK_EQ(detail::error_type_name(ModbusTimeout("x")), "ModbusTimeout");
    CHECK_EQ(detail::error_type_name(ServoBlockedError("x", {})), "ServoBlockedError");
    CHECK_EQ(detail::link_fault_kind(TransportError("x")), "port_vanished");
    CHECK_EQ(detail::link_fault_kind(ModbusTimeout("x")), "unit_silent");
    CHECK_EQ(detail::link_fault_kind(EcAlarmError(0x90)), "");
}
