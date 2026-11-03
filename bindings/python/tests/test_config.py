"""Parameter write + commit, software reset, baud change, stored-position moves, push power and unit codes.

Evidence: VERIFIED = confirmed on hardware, ON-WIRE = observed on the serial line (a USB capture of IA-OS
talking to the EC-R6S rod), UNVERIFIED = expected behaviour, not yet confirmed on hardware.

* Parameter commit = FC10 item write, then coil 0x0428 (DRGE b7 PARW) ON; no OFF (ON-WIRE: IA-OS's Transfer;
  the same frames from this driver VERIFIED permanent on the EC-R6S rod). DRGE 0x0180 at +55 ms (commit
  running), 0x0100 at +235 ms (ON-WIRE). Waiting for DRGE b7 = 0 AND DSSE b7 = 1 before and after the coil is
  UNVERIFIED. Without PARW a write is RAM only (VERIFIED on the EC-GRBP8M gripper: lost at power-off).
* Software reset = ``01 45 00 09 00 00 00 00`` (servo OFF) then ``01 45 00 05 00 01 00 00`` -> ``01 45 00 00``
  (ON-WIRE, then VERIFIED). Afterwards PMSS 0, SFTY 0; the uptime keeps counting; the EC-R6S rod (incremental)
  lost HEND. The SFRS coil 0x0420 raised alarm 0x090 with the servo on (VERIFIED on the EC-GRBP8M gripper) and
  is never used.
* Baud change = broadcast ``00 45 00 04 00 14 00 05`` (115200; ON-WIRE); the "break" method sends it after a
  200 ms break at 9600 (UNVERIFIED); volatile (VERIFIED). The EC-R6S rod switched 115200 <-> 230400 (0x14 /
  0x15) sent direct (VERIFIED), but IGNORED 0x06 ("38400").
* 0x9800 is ignored while PMSS = 0 (VERIFIED on the EC-GRBP8M gripper). The EC-R6S rod's push row has PPOW 26
  (~10 %), and the rod answers FC11 after 0.2-0.4 s. The per-model parameter lists IA-OS shows: gripper P1-P7,
  P9, P10, P12; rod P1-P6, P8, P9, P12 (VERIFIED on those two units).

Everything runs against the simulator (no serial port is opened).
"""

from __future__ import annotations

import contextlib
import io
import json
import struct
import time
import unittest
from dataclasses import replace

from ec_testutil import FrameRecorder, ScriptedTransport, SteppedClockSim

from iai_actuator_driver import cli
from iai_actuator_driver import info as ec_info
from iai_actuator_driver import registers as R
from iai_actuator_driver import testing
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.errors import (
    EcNotReadyError,
    ModbusTimeout,
    MotionIncompleteError,
    SoftLimitError,
    UnsafeCommandError,
    VerificationError,
)
from iai_actuator_driver.modbus import (
    BAUD_DETECT_ORDER,
    BAUD_PROBE_ALL,
    IAI_BAUD_CODES,
    SLAVE_ID_TIMEOUT_S,
    VERIFIED_BAUD_RATES,
    ModbusRTU,
    build_frame,
)
from iai_actuator_driver.sim import SimulatedEC


def hexbytes(text: str) -> bytes:
    return bytes.fromhex(text.replace(" ", ""))


def open_sim(
    profile: str = "slider",
    *,
    sim: SimulatedEC | None = None,
    time_scale: float = 50.0,
    max_speed: float = 100.0,
    max_accel: float = 0.3,
    stroke: tuple[float, float] | None = None,
    auto: bool = True,
    **sim_kwargs: object,
) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = sim or SimulatedEC(profile=profile, time_scale=time_scale, **sim_kwargs)  # type: ignore[arg-type]
    rec = FrameRecorder()
    bus = ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0)
    ec = Elecylinder(
        bus, stroke=stroke, max_speed_mm_s=max_speed, max_accel_g=max_accel, poll_interval_s=0.005,
        owns_bus=True, auto_modbus_control=auto,
    )
    return ec, sim, rec


def writes(rec: FrameRecorder, function: int) -> list[bytes]:
    return [f for f in rec.tx if f[1] == function]


def coil_writes(rec: FrameRecorder, address: int) -> list[bool]:
    return [f[4] == 0xFF for f in writes(rec, 0x05) if struct.unpack(">H", f[2:4])[0] == address]


# a unit whose CTCL (class 3 item 1, 0x0300) reads 0: with FC11 silent its unit code cannot be read
NO_CTCL = {0x0300: 0, 0x0301: 0}


def param_value(ec: Elecylinder, no: int) -> object:
    return next(p for p in ec.user_parameters() if p["no"] == no)["value"]


# =========================================================================== link layer


class LinkLayerTest(unittest.TestCase):
    def test_fc11_waits_for_the_slow_rod(self) -> None:
        self.assertGreaterEqual(SLAVE_ID_TIMEOUT_S, 0.5)
        sim = SimulatedEC(profile="rod")
        bus = ModbusRTU(sim, min_gap_s=0.0)
        started = time.perf_counter()
        self.assertEqual(bus.report_slave_id(1, retries=0), bytes.fromhex("ecff000002"))
        self.assertGreaterEqual(time.perf_counter() - started, 0.25)  # answered after ~0.3 s, not lost
        with self.assertRaises(ModbusTimeout):
            bus.report_slave_id(1, retries=0, timeout=None)  # the transport's own (short) timeout only
        self.assertEqual(len(bus.read_registers(1, R.PNOW, 2)), 2)  # a delayed answer never leaks into the next one

    def test_rod_identity_and_unit_label(self) -> None:
        ec, _, _ = open_sim("rod")
        with ec:
            info = ec.identify()
        self.assertEqual((info.unit_code, info.firmware, info.model), (0x00, "0002", "EC-R6S-150-1-PN"))
        self.assertEqual(info.controller, "EC 24 V, unit 0x00 (EC standard)")  # the unit code's family name
        self.assertEqual((info.actuator_kind, info.actuator_kind_name), (3, "rod"))  # the kind comes from AKID 7.33
        self.assertEqual((info.user_name, info.axis_number), (None, 0))  # the rod: blank unit name, AXNO 0
        self.assertEqual((len(info.categories), info.soft_limits_mm, info.max_speed_mm_s), (22, (-0.3, 150.3), 800.0))
        self.assertEqual((info.max_accel_g, info.max_decel_g), (1.0, 1.0))  # MAXA / MAXD (VERIFIED on the EC-R6S rod)
        self.assertEqual(info.errors, ())
        self.assertEqual(ec.unit_code, 0x00)

    def test_broadcast_baud_frame_matches_the_capture(self) -> None:
        transport = ScriptedTransport()
        bus = ModbusRTU(transport, min_gap_s=0.0)
        frame = bus.broadcast_iai_command(
            0, R.IAI_CMD_BAUD, 0x14, R.IAI_BAUD_OP2, allow_unsafe=True, allow_never_send=True, settle_s=0
        )
        self.assertEqual(frame, hexbytes("00 45 00 04 00 14 00 05 25 08"))  # the captured frame, CRC included
        self.assertEqual(transport.writes, [frame])
        with self.assertRaises(UnsafeCommandError):
            bus.broadcast_iai_command(0, R.IAI_CMD_BAUD, 0x14, 5)  # on the never-send list
        with self.assertRaisesRegex(UnsafeCommandError, "never-send"):
            bus.broadcast_iai_command(0, R.IAI_CMD_BAUD, 0x14, 5, allow_unsafe=True)  # allow_unsafe is not enough
        self.assertEqual(transport.writes, [frame])
        self.assertEqual(IAI_BAUD_CODES[115200], 0x14)
        self.assertEqual({IAI_BAUD_CODES[r] for r in (9600, 19200, 38400, 57600, 230400)}, {4, 5, 6, 0x13, 0x15})
        self.assertEqual(VERIFIED_BAUD_RATES, (115200, 230400))  # the codes the EC-R6S rod took (0x06 was ignored)
        self.assertEqual(BAUD_PROBE_ALL[: len(BAUD_DETECT_ORDER)], BAUD_DETECT_ORDER)
        self.assertEqual(set(BAUD_PROBE_ALL), set(IAI_BAUD_CODES))  # the failure path probes every coded rate

    def test_iai_command_without_retries(self) -> None:
        transport = ScriptedTransport([None, None, None])
        bus = ModbusRTU(transport, min_gap_s=0.0, retries=2)
        with self.assertRaises(ModbusTimeout):
            bus.iai_command(1, 0, R.IAI_CMD_RESET, 1, allow_unsafe=True, retries=0)
        self.assertEqual(len(transport.writes), 1)

    def test_break_and_baud_need_a_capable_transport(self) -> None:
        bus = ModbusRTU(ScriptedTransport(), min_gap_s=0.0)
        self.assertFalse(bus.can_send_break)
        with self.assertRaises(TypeError):
            bus.send_break(0.2)
        with self.assertRaises(TypeError):
            bus.set_baudrate(9600)


# =========================================================================== user parameters


class ParameterListTest(unittest.TestCase):
    def test_gripper_list(self) -> None:
        ec, _, _ = open_sim("gripper")
        with ec:
            params = ec.user_parameters()
        self.assertEqual([p["no"] for p in params], [1, 2, 3, 4, 5, 6, 7, 9, 10, 12])
        self.assertTrue(all(p["found"] and p["listed"] and p["verified_for_model"] for p in params))
        self.assertEqual({p["no"] for p in params if p["home_parameter"]}, {3, 4})
        self.assertEqual({p["no"]: p["text"] for p in params}[3], "1:Forward")

    def test_rod_list_shows_p8_read_only(self) -> None:
        ec, _, _ = open_sim("rod")
        with ec:
            params = ec.user_parameters()
        self.assertEqual([p["no"] for p in params], [1, 2, 3, 4, 5, 6, 8, 9, 12])  # = IA-OS's list on the EC-R6S rod
        p8 = next(p for p in params if p["no"] == 8)
        # P8 = ATYP 7.7 bit 18 (location UNVERIFIED): the EC-R6S rod's 0x05350000 -> "1:Disable", the value IA-OS
        # shows; read-only here
        self.assertEqual((p8["located"], p8["found"], p8["writable"], p8["read_only"]), (True, True, False, True))
        self.assertEqual((p8["value"], p8["text"], p8["key"], p8["address"]), (1, "1:Disable", "7.7", "0x070C"))
        self.assertIn("STN1", p8["model_note"])
        values = {p["no"]: p["text"] for p in params}
        self.assertEqual(values, {1: "150.00 mm", 2: "0.10 mm", 3: "1:Forward", 4: "3.00 mm", 5: "0:Disable",
                                  6: "0:Disable", 8: "1:Disable", 9: "0:Double", 12: "0:Continuous energization"})

    def test_slider_family_list_is_used_without_a_model_label(self) -> None:
        ec, _, _ = open_sim("slider")
        with ec:
            params = ec.user_parameters()
        # unit 0x02 (EC dust/drip-proof), IA-OS's list: candidates 1-9, 11-18, minus the visibility rules: P8 (no
        # STN1 tuning data), P11 / P13-P18 (ACCL extensibility level 1, EXFN 0), P16 (no SORS); no P10 candidate
        self.assertEqual([p["no"] for p in params], [1, 2, 3, 4, 5, 6, 7, 9, 12])
        self.assertTrue(all(p["writable"] and p["verified_for_model"] for p in params))  # verified_for_model = known
        # every parameter is offered, with no note about the model: there is no per-model opt-in or label
        self.assertFalse(any(p["needs_model_opt_in"] for p in params))
        self.assertTrue(all(p["model_note"] == "" for p in params))
        model = ec_info.model_parameter_list(0x02)
        self.assertTrue(model["known"] and model["verified"])
        self.assertEqual(model["seen_on"], [])
        self.assertEqual(model["note"], "the parameters IA-OS lists for unit 0x02 (EC dust/drip-proof)")
        self.assertEqual(model["family"], "EC dust/drip-proof")
        # without the runtime rules: the family's candidate list
        self.assertEqual(ec_info.model_parameter_list(0x00)["listed"], [1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15,
                                                                        16, 17, 18])
        unknown = ec_info.model_parameter_list(0x0C)  # an unknown unit code (e.g. a later model)
        self.assertFalse(unknown["known"] or unknown["verified"])
        self.assertFalse(any(ec_info.needs_model_opt_in(p, unknown) for p in ec_info.USER_PARAMETERS))

    def test_visibility_rules_reproduce_the_lists_seen_on_hardware(self) -> None:
        for profile, seen in (("gripper", R.MODEL_PARAMETERS[0x0A]), ("rod", R.MODEL_PARAMETERS[0x00])):
            with self.subTest(profile=profile):
                ec, _, _ = open_sim(profile)
                with ec:
                    model = ec.parameter_model(visibility=True)
                self.assertEqual(tuple(model["listed"]), seen)
                hidden = {h["no"]: h["reason"] for h in model["hidden"]}
                self.assertTrue(set(hidden) >= {11, 13, 14, 15, 16, 17, 18})
                if profile == "rod":
                    self.assertIn("CNWL", hidden[7])  # no wireless module on the -PN rod

    def test_the_unit_code_list_is_used_whatever_the_model_string(self) -> None:
        for code, model_name in ((0x00, "EC-R6S-150-1-PN"), (0x0A, "EC-GRBP8M-20-1-1-PN-WA-WL2"),
                                 (0x00, "EC-S6S-200-1-PN"), (0x00, None), (0x0A, "EC-GRST7M-10")):
            with self.subTest(code=code, model_name=model_name):
                model = ec_info.model_parameter_list(code, model_name)
                self.assertTrue(model["known"])  # the recorded list is used (unlisted ones stay refused)
                self.assertTrue(model["verified"])  # kept for compatibility: equals known
                self.assertEqual(model["seen_on"], [])
                self.assertNotIn("UNVERIFIED", model["note"].upper())
                self.assertEqual(model["note"], f"the parameters IA-OS lists for unit 0x{code:02X} ({model['family']})")
        unknown = ec_info.model_parameter_list(None)
        self.assertFalse(unknown["known"] or unknown["verified"])
        self.assertEqual(unknown["note"], "unit code not readable (FC11 / CTCL): every mapped item found on the unit "
                                          "is shown")
        self.assertEqual(ec_info.model_parameter_list(0x7E)["note"],
                         "unit code 0x7E: every mapped item found on the unit is shown")
        self.assertFalse(any(ec_info.needs_model_opt_in(p, unknown) for p in ec_info.USER_PARAMETERS))

    def test_another_unit_with_the_rod_code_is_written_the_same_way(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50.0, model_string="EC-S6S-200-1-PN")  # same unit code, other model
        ec, _, rec = open_sim(sim=sim)
        with ec:
            params = {p["no"]: p for p in ec.user_parameters()}
            self.assertEqual(sorted(params), [1, 2, 3, 4, 5, 6, 8, 9, 12])  # the 0x00 list, as a guide
            self.assertTrue(params[5]["verified_for_model"])  # the 0x00 list is recorded
            self.assertEqual(params[5]["model_note"], "")
            self.assertFalse(params[5]["needs_model_opt_in"] or params[2]["needs_model_opt_in"])
            rec.clear()
            bit = ec.write_user_parameter(5, 1)  # a bit parameter: written, no model warning (no per-model opt-in)
            self.assertEqual(bit.after, "1:Enable")
            unlisted = ec.write_user_parameter(7, 1)  # not in the 0x00 list: written, with a warning
            self.assertFalse(unlisted.listed_for_model)
            self.assertTrue(any("not in the parameter list for unit 0x00" in w for w in unlisted.warnings))
            result = ec.write_user_parameter(2, 0.2)  # an mm item: written the same way
            self.assertTrue(result.verified_for_model)
        self.assertFalse(any("UNVERIFIED" in w for w in result.warnings + bit.warnings), result.warnings + bit.warnings)

    def test_unit_info_carries_the_model_list(self) -> None:
        ec, _, _ = open_sim("rod")
        with ec:
            data = ec.unit_info(sections=["identity", "parameters"])
        params = data["parameters"]
        self.assertEqual(params["model"]["listed"], [1, 2, 3, 4, 5, 6, 8, 9, 12])
        named = {p["no"]: p for p in params["named"]}
        self.assertFalse(named[7]["listed"])  # found by ID, but hidden on the rod (no CNWL)
        self.assertTrue(named[7]["writable"])  # unlisted is no gate: the write goes through with a warning
        self.assertEqual(named[7]["model_note"],
                         "not listed by IA-OS for unit 0x00 (shown only with a wireless module (SUP2 CNWL))")
        self.assertTrue(named[2]["writable"])
        self.assertTrue(params["model"]["verified"])  # kept for compatibility: the unit code has an IA-OS list
        self.assertFalse(any(p["needs_model_opt_in"] for p in params["named"]))
        self.assertEqual(params["not_located"], [])  # P8 / P11 are located (read-only)
        self.assertEqual((named[8]["text"], named[8]["listed"], named[8]["writable"]), ("1:Disable", True, False))
        self.assertEqual(sorted(named), list(range(1, 19)))  # P1-P18 decoded
        self.assertEqual(named[14]["text"], "150.30 mm")  # zone 1 + (0xC.1 entry 0), not on the list, read-only
        self.assertIn("PARW", params["write_note"])
        json.dumps(data)


class ParameterWriteTest(unittest.TestCase):
    def test_commit_matches_the_captured_frames(self) -> None:
        ec, sim, rec = open_sim("gripper")
        with ec:
            rec.clear()
            result = ec.write_user_parameter(2, 0.20)
            self.assertEqual((result.before, result.after, result.value), ("0.10 mm", "0.20 mm", 0.2))
            self.assertEqual((result.key, result.address, result.raw_after), ("8.9", "0x0810", "00000014"))
            self.assertTrue(result.verified and result.committed and result.saved and result.needs_reset)
            self.assertTrue(result.commit_confirmed)  # DRGE b7 seen set after the coil, then cleared
            self.assertEqual(result.warnings, ())
            self.assertIn("software reset", result.takes_effect)
            fc10 = writes(rec, 0x10)
            self.assertIn(build_frame(1, hexbytes("10 08 10 00 02 04 00 00 00 14")), fc10)  # = the captured frame
            self.assertEqual(coil_writes(rec, R.COIL_PARW), [True])  # ON only: it clears itself
            self.assertIn(hexbytes("01 05 04 28 FF 00 0D 02"), rec.tx)  # the captured PARW frame, CRC included
            # the handshake: DRGE (0x0D02) and DSSE (0x9007) read right before the coil and after it
            coil = rec.tx.index(hexbytes("01 05 04 28 FF 00 0D 02"))
            reads = [struct.unpack(">H", f[2:4])[0] for f in rec.tx if f[1] == 0x03]
            before = [struct.unpack(">H", f[2:4])[0] for f in rec.tx[:coil] if f[1] == 0x03]
            self.assertEqual(before[-2:], [R.DRGE, R.DSSE])
            self.assertGreaterEqual(reads.count(R.DRGE) - before.count(R.DRGE), 2)  # b7 set, then clear
            self.assertEqual(ec.read_registers(R.DRGE, 1)[0] & R.DRGE_PARW, 0)
            self.assertEqual(sim.parameter_commits, 1)
            sim.power_cycle()
            self.assertEqual(param_value(ec, 2), 0.2)  # committed: survives the power cycle

    def test_commit_with_dsse_b7_dropping(self) -> None:
        ec, sim, _ = open_sim("gripper", parw_dsse_busy=True, parw_busy_s=0.15)
        with ec:
            result = ec.write_user_parameter(2, 0.20)
        self.assertTrue(result.saved and result.commit_confirmed)
        self.assertEqual(sim.parameter_commits, 1)

    def test_coil_that_never_latches_is_reported_unconfirmed(self) -> None:
        # the coil is acknowledged and never acted on (like PMSS = 0 in the sim), or the commit is over at once
        sims = (
            SimulatedEC(profile="gripper", time_scale=50.0, parw_latches=False),
            SimulatedEC(profile="gripper", parw_busy_s=0),
        )
        for sim in sims:
            with self.subTest(parw_latches=sim.parw_latches, parw_busy_s=sim.parw_busy_s):
                ec, _, rec = open_sim(sim=sim)
                with ec:
                    result = ec.write_user_parameter(2, 0.20)
                self.assertEqual(coil_writes(rec, R.COIL_PARW), [True])
                self.assertTrue(result.committed)  # coil acknowledged and the handshake idle ...
                self.assertFalse(result.commit_confirmed or result.saved)  # ... but b7 never seen: not confirmed
                self.assertTrue(any(w.startswith("commit NOT confirmed") for w in result.warnings))
                self.assertIn("NOT confirmed", result.takes_effect)
                self.assertFalse(result.to_dict()["saved"])

    def test_busy_commit_blocks_a_new_write(self) -> None:
        ec, sim, rec = open_sim("gripper", parw_busy_s=5.0)
        with ec, testing.timings(parw_ready_s=0.2):
            ec.bus.write_coil(1, R.COIL_PARW, True)  # another master's commit is still running
            self.assertTrue(sim.parw_busy)
            rec.clear()
            with self.assertRaises(EcNotReadyError) as caught:
                ec.write_user_parameter(2, 0.20)
            self.assertIn("still in progress", str(caught.exception))
            self.assertIn("nothing written", str(caught.exception))
            self.assertEqual(writes(rec, 0x10) + writes(rec, 0x05), [])

    def test_ram_only_reverts_at_power_up(self) -> None:
        ec, sim, rec = open_sim("gripper")
        with ec:
            rec.clear()
            result = ec.write_user_parameter(2, "0.30", commit=False)
            self.assertFalse(result.committed or result.saved)
            self.assertIn("RAM", result.takes_effect)
            self.assertEqual(coil_writes(rec, R.COIL_PARW), [])
            self.assertEqual(param_value(ec, 2), 0.3)
            sim.power_cycle()
            self.assertEqual(param_value(ec, 2), 0.1)  # RAM only: lost at power-off (VERIFIED on the gripper)

    def test_shared_items_are_read_modify_written(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            self.assertEqual(ec.write_user_parameter(5, "1:Enable").raw_after, "00000440")  # P6 bit 10 kept
            self.assertEqual(ec.write_user_parameter(6, "disable").raw_after, "00000040")  # P5 bit 6 kept
            self.assertEqual(ec.write_user_parameter(7, 1).raw_after, "00808008")  # 1 = Disable (inverted sense)
            self.assertEqual(ec.write_user_parameter(10, "Enable").raw_after, "00848008")
            self.assertEqual(ec.write_user_parameter(12, "1:Instantaneous energization").raw_after, "20410000")
            self.assertEqual(ec.write_user_parameter(9, 3).after, "3:Single")
            self.assertEqual(sim.parameter_commits, 6)

    def test_unchanged_value_is_neither_written_nor_committed(self) -> None:
        ec, sim, rec = open_sim("gripper")
        with ec:
            rec.clear()
            result = ec.write_user_parameter(2, 0.10)
            self.assertTrue(result.skipped)
            self.assertFalse(result.changed)
            self.assertFalse(result.committed)
            self.assertEqual(writes(rec, 0x10), [])
            self.assertEqual(sim.parameter_commits, 0)
            forced = ec.write_user_parameter(2, 0.10, force=True)  # force: committed anyway, still no item write
        self.assertFalse(forced.skipped)
        self.assertTrue(forced.committed)
        self.assertEqual(writes(rec, 0x10), [])
        self.assertEqual(sim.parameter_commits, 1)

    def test_refusals_write_nothing(self) -> None:
        cases = (
            ("rod", 8, 0, {}, UnsafeCommandError, "read-only"),  # P8 = 7.7 b18: located, read-only (STN1 side effects)
            ("gripper", 11, 0, {}, UnsafeCommandError, "read-only"),
            ("gripper", 13, 0, {}, UnsafeCommandError, "read-only"),
            ("gripper", 19, 0, {}, UnsafeCommandError, "not a confirmed"),
            # ORG patched to 0 below: P3 is then not settable (UNVERIFIED)
            ("gripper", 3, 0, {"commit": False}, SoftLimitError, "not settable"),
            ("gripper", 2, 2.0, {}, SoftLimitError, "0.01..1.00"),
            ("gripper", 2, "abc", {}, SoftLimitError, "needs a number"),
            ("gripper", 4, 9.0, {}, SoftLimitError, "0.00..5.00"),
            ("gripper", 5, True, {}, SoftLimitError, "not True"),
            ("gripper", 5, 2, {}, SoftLimitError, "is not one of"),
            ("gripper", 1, 9.6, {}, SoftLimitError, "narrow the stroke"),
            ("gripper", 9, 4, {}, SoftLimitError, "this unit accepts only 0, 3"),  # level 0, MXPT 3
            ("rod", 9, 3, {}, SoftLimitError, "this unit accepts only 0"),  # the EC-R6S rod: MXPT (3.16) = 2
            ("gripper", 12, 1, {}, SoftLimitError, "refuses P12 != 0 while P9 is 3"),  # P9 patched to 3 below
        )
        for profile, no, value, kwargs, error, text in cases:
            with self.subTest(profile=profile, no=no, value=value):
                ec, sim, rec = open_sim(profile)
                if no == 3:
                    sim.set_parameter_word(0x0918, 0, nv=True)  # gripper ORG (9.16) = 0
                    sim.set_parameter_word(0x0919, 0, nv=True)
                if no == 12:
                    sim.set_parameter_word(0x0802, 0, nv=True)  # gripper P9 (8.2) = 3 Single
                    sim.set_parameter_word(0x0803, 3, nv=True)
                with ec:
                    rec.clear()
                    with self.assertRaises(error) as caught:
                        ec.write_user_parameter(no, value, **kwargs)  # type: ignore[arg-type]
                    self.assertIn(text, str(caught.exception))
                    self.assertEqual(writes(rec, 0x10), [])
                    self.assertEqual(coil_writes(rec, R.COIL_PARW), [])
                    self.assertEqual(sim.parameter_ram, {})

    def test_home_and_unlisted_parameters_are_written_with_a_warning(self) -> None:
        for no, value, text in (
            (4, 3.1, "home parameters"),
            (3, 0, "home parameters"),
            (7, "1:Disable", "not in the parameter list for unit 0x00"),
        ):
            with self.subTest(no=no):
                ec, sim, _ = open_sim("rod")
                with ec:
                    result = ec.write_user_parameter(no, value)  # allow_home_params / allow_unlisted: not needed
                self.assertTrue(result.committed)
                self.assertTrue(any(text in w for w in result.warnings), result.warnings)
                self.assertEqual(sim.parameter_commits, 1)
        self.assertFalse(result.listed_for_model)
        self.assertTrue(any("written anyway" in w for w in result.warnings))

    def test_any_model_write_has_no_model_warning(self) -> None:
        ec, _, _ = open_sim("slider")
        with ec:
            result = ec.write_user_parameter(2, 0.2)
        self.assertEqual(result.after, "0.20 mm")
        self.assertTrue(result.verified_for_model)  # kept for compatibility: unit 0x02 has an IA-OS list
        self.assertEqual(result.warnings, ())

    def test_bit_parameters_of_any_model_are_written_the_same_way(self) -> None:
        ec, sim, rec = open_sim("slider")  # unit 0x02: IA-OS's parameter list is recorded
        with ec:
            unlisted = ec.write_user_parameter(10, 1, commit=False)  # no P10 on the dust-proof family's list
            self.assertTrue(any("not in the parameter list for unit 0x02" in w for w in unlisted.warnings))
            for no, value, text in ((5, 1, "1:Enable"), (6, 1, "1:Enable"), (9, 3, "3:Single")):
                with self.subTest(no=no):
                    result = ec.write_user_parameter(no, value, commit=False)
                    self.assertEqual(result.after, text)
                    self.assertFalse(any("UNVERIFIED" in w for w in result.warnings), result.warnings)
            # the legacy keyword is accepted and changes nothing
            result = ec.write_user_parameter(5, 0, allow_unverified_model=True)
        self.assertEqual(result.after, "0:Disable")

    def test_unknown_unit_code_writes_the_same_way_and_retries_fc11(self) -> None:
        # FC11 timing out (the rod answers later than the timeout) must not leave unit_code None for the life of
        # the object: P7 would then write 8.50 bit 23 with the gripper's layout. (CTCL 0x0300, the fallback
        # identity, is blanked here too, so the unit code cannot be read at all.)
        sim = SimulatedEC(profile="rod", time_scale=50.0, slave_id_delay_s=2.0, word_overrides=NO_CTCL)
        rec = FrameRecorder()
        ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0, retries=0), poll_interval_s=0.005,
                         owns_bus=True)
        with ec:
            # no per-model refusal and no model warning: an unreadable unit code is written the same way
            result = ec.write_user_parameter(2, 0.2, commit=False)
            self.assertEqual(result.after, "0.20 mm")
            self.assertFalse(any("UNVERIFIED" in w for w in result.warnings), result.warnings)
            self.assertIsNone(ec.unit_code)
            sim.slave_id_delay_s = 0.0  # FC11 answers now: the next call reads the unit code (not cached as unknown)
            unlisted = ec.write_user_parameter(7, 1, commit=False)
            self.assertTrue(any("not in the parameter list for unit 0x00" in w for w in unlisted.warnings))
            self.assertEqual(ec.unit_code, 0x00)

    def test_fc11_silent_unit_code_from_ctcl(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50.0, slave_id_delay_s=2.0)  # FC11 never in time
        ec = Elecylinder(ModbusRTU(sim, min_gap_s=0.0, retries=0), poll_interval_s=0.005, owns_bus=True)
        with ec:
            self.assertEqual(ec.parameter_model()["unit_code"], 0x00)  # from CTCL (class 3 item 1)
            self.assertEqual(ec.write_user_parameter(2, 0.2).after, "0.20 mm")

    def test_unknown_unit_code_works_like_any_other(self) -> None:
        # a unit code without a family name in the driver (e.g. a later model): motion and parameter writes work the
        # same way, with no note or warning about the model (no opt-in)
        sim = SimulatedEC(profile="rod", time_scale=50.0, unit_code=0x0C)
        ec, _, rec = open_sim(sim=sim)
        with ec:
            info = ec.identify()
            self.assertFalse(info.unit_known)
            self.assertIsNone(info.unit_name)
            self.assertFalse(any("UNVERIFIED" in e or "family table" in e for e in info.errors), info.errors)
            params = {p["no"]: p for p in ec.user_parameters()}
            self.assertFalse(any(p["needs_model_opt_in"] for p in params.values()))
            self.assertFalse(any("UNVERIFIED" in p["model_note"] for p in params.values()))
            self.assertEqual(ec.move_to(5.0, speed_mm_s=20.0).position_mm, 5.0)  # motion is not refused
            for no, value, text in ((2, 0.2, "0.20 mm"), (5, 1, "1:Enable")):
                with self.subTest(no=no):
                    result = ec.write_user_parameter(no, value)
                    self.assertEqual(result.after, text)
                    self.assertFalse(result.verified_for_model)  # no IA-OS list for 0x0C
                    self.assertFalse(any("UNVERIFIED" in w for w in result.warnings), result.warnings)

    def test_unreadable_unit_code_still_writes(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50.0, slave_id_delay_s=2.0, word_overrides=NO_CTCL)
        ec = Elecylinder(ModbusRTU(sim, min_gap_s=0.0, retries=0), poll_interval_s=0.005, owns_bus=True)
        with ec:
            params = {p["no"]: p for p in ec.user_parameters()}
            self.assertFalse(any(p["needs_model_opt_in"] for p in params.values()))
            self.assertTrue(all(not p["writable"] for p in params.values() if p["read_only"]))
            result = ec.write_user_parameter(2, 0.2)
        self.assertEqual(result.after, "0.20 mm")
        self.assertFalse(result.verified_for_model)

    def test_p1_narrows_the_stroke_at_once(self) -> None:
        ec, _, _ = open_sim("gripper")
        with ec:
            ec.connect()
            self.assertEqual(ec.effective_stroke, (0.0, 9.5))
            with self.assertRaises(SoftLimitError):
                ec.write_user_parameter(1, 9.0)  # the axis stands at 9.50 mm: it would be outside the new range
            ec.move_to(5.0, speed_mm_s=20)
            result = ec.write_user_parameter(1, 9.0)
            self.assertEqual((result.raw_after, ec.effective_stroke), ("000003A2", (0.0, 9.0)))  # (9.00 + 0.30) x 100 = 930
            with self.assertRaises(SoftLimitError):
                ec.move_to(9.2, speed_mm_s=10)
            ec.write_user_parameter(1, 9.5)  # back to the value read in this session (the ceiling)

    def test_refused_while_moving(self) -> None:
        ec, _, rec = open_sim("slider", time_scale=1.0)
        with ec:
            ec.move_to(120.0, speed_mm_s=20.0, wait=False)
            rec.clear()
            with self.assertRaises(EcNotReadyError):
                ec.write_user_parameter(2, 0.2)
            self.assertEqual(writes(rec, 0x10), [])
            ec.stop(wait=True)

    def test_commit_takes_modbus_control_at_rest_first(self) -> None:
        ec, sim, rec = open_sim("gripper", pmsl=False)
        with ec:
            ec.write_user_parameter(2, 0.2)
        order = [(f[1], struct.unpack(">H", f[2:4])[0]) for f in rec.tx if f[1] in (0x05, 0x10)]
        self.assertEqual(order, [(0x05, R.COIL_PMSL), (0x10, 0x0810), (0x05, R.COIL_PARW)])
        self.assertEqual(sim.parameter_commits, 1)

    def test_parw_that_does_not_clear_is_reported(self) -> None:
        ec, sim, _ = open_sim("gripper", parw_busy_s=60.0)  # the commit never finishes
        with ec, testing.timings(parw_clear_s=0.3), self.assertRaises(VerificationError) as caught:
            ec.write_user_parameter(2, 0.2)
        self.assertIn("commit is not confirmed", str(caught.exception))
        self.assertIn("Do not reset or power off", str(caught.exception))
        self.assertTrue(sim.parw_busy)

    def test_read_back_mismatch_is_not_committed(self) -> None:
        class IgnoresParams(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1] in (0x06, 0x10) and 0x0800 <= struct.unpack(">H", frame[2:4])[0] < 0x0900:
                    # accepted, not stored: the normal reply to FC06 (the request echoed) and FC10 (address, count)
                    return build_frame(self.slave, frame[1:6])
                return NotImplemented

        ec, sim, rec = open_sim(sim=IgnoresParams(profile="gripper", time_scale=50.0))
        with ec, self.assertRaises(VerificationError):
            ec.write_user_parameter(2, 0.2)
        self.assertEqual(coil_writes(rec, R.COIL_PARW), [])
        self.assertEqual(sim.parameter_commits, 0)


# =========================================================================== software reset


class SoftwareResetTest(unittest.TestCase):
    def test_captured_frames_and_incremental_unit_loses_home(self) -> None:
        ec, sim, rec = open_sim("rod")
        with ec:
            before = ec.status()
            rec.clear()
            result = ec.software_reset(reset_wait_s=0.0)
        iai = writes(rec, 0x45)
        self.assertEqual(iai[:2], [hexbytes("01 45 00 09 00 00 00 00 49 02"), hexbytes("01 45 00 05 00 01 00 00 08 C3")])
        self.assertEqual(coil_writes(rec, R.COIL_SFRS), [])  # never the SFRS coil (alarm 0x090 on the gripper)
        self.assertTrue(result.accepted and not result.reply_lost)
        self.assertEqual((result.modbus_enabled, result.safety_speed, result.servo_on), (False, False, True))
        self.assertEqual((result.homed, result.needs_home, result.alarm_code), (False, True, 0))
        self.assertGreaterEqual(result.uptime_after_s or 0, before.raw["STIM"] / 1000)  # uptime not reset
        self.assertTrue(any("not reset by a software reset" in n for n in result.notes))
        self.assertTrue(any("home lost" in n for n in result.notes))
        self.assertEqual(sim.software_resets, 1)
        json.dumps(result.to_dict())

    def test_absolute_unit_keeps_home(self) -> None:
        ec, _, _ = open_sim("slider")
        with ec:
            result = ec.software_reset(reset_wait_s=0.0)
            self.assertFalse(result.needs_home)
            self.assertTrue(result.homed and result.servo_on)
            ec.move_to(80.0, speed_mm_s=50.0)  # takes Modbus control again at rest and moves
            self.assertTrue(ec.status().modbus_enabled)

    def test_committed_parameter_applies_after_the_reset(self) -> None:
        ec, sim, _ = open_sim("rod")
        with ec:
            ec.write_user_parameter(2, 0.20)
            self.assertEqual(sim.ls_range_mm, 0.10)  # in the register at once, applied only after a reset
            ec.software_reset(reset_wait_s=0.0)
            self.assertEqual(sim.ls_range_mm, 0.20)
            self.assertEqual(param_value(ec, 2), 0.2)

    def test_reset_waits_for_a_running_commit(self) -> None:
        ec, sim, rec = open_sim("rod", parw_busy_s=0.3)
        with ec:
            ec.bus.write_registers(1, 0x0810, [0, 20])  # P2 = 0.20 (RAM) ...
            ec.bus.write_coil(1, R.COIL_PARW, True)  # ... and its commit, still running
            self.assertTrue(sim.parw_busy)
            rec.clear()
            started = time.perf_counter()
            ec.software_reset(reset_wait_s=0.0)
            self.assertGreaterEqual(time.perf_counter() - started, 0.2)  # waited for DRGE b7 = 0 / DSSE b7 = 1
            first_iai = next(i for i, f in enumerate(rec.tx) if f[1] == 0x45)
            last_drge = max(i for i, f in enumerate(rec.tx[:first_iai]) if f[1] == 0x03 and f[2:4] == b"\x0d\x02")
            self.assertLess(last_drge, first_iai)
            self.assertNotIn("parameter commit (PARW) interrupted by a restart: lost", sim.events)
            self.assertEqual(param_value(ec, 2), 0.2)  # the commit survived the reset

    def test_reset_refused_while_a_commit_does_not_finish(self) -> None:
        ec, sim, rec = open_sim("rod", parw_busy_s=60.0, parw_dsse_busy=True)
        with ec, testing.timings(parw_ready_s=0.2):
            ec.bus.write_coil(1, R.COIL_PARW, True)
            rec.clear()
            with self.assertRaises(EcNotReadyError) as caught:
                ec.software_reset(reset_wait_s=0.0)
            self.assertIn("software reset not sent", str(caught.exception))
            self.assertIn("DSSE 0x", str(caught.exception))
            self.assertEqual(writes(rec, 0x45) + writes(rec, 0x05), [])
        self.assertEqual(sim.software_resets, 0)

    def test_uncommitted_write_is_lost_at_the_reset(self) -> None:
        ec, _, _ = open_sim("rod")
        with ec:
            ec.write_user_parameter(2, 0.30, commit=False)
            ec.software_reset(reset_wait_s=0.0)
            self.assertEqual(param_value(ec, 2), 0.1)

    def test_home_parameter_on_absolute_unit_gives_abs_error(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            ec.write_user_parameter(4, 0.60, allow_home_params=True)
            result = ec.software_reset(reset_wait_s=0.0)
            self.assertEqual((result.alarm_code, result.needs_home, result.servo_on), (0x0EE, True, False))
            st = ec.reset_alarm()  # gripper recovery: the servo came back on by itself
            self.assertTrue(st.servo_on and not st.homed and not st.alarm)
            st = ec.home()
            self.assertTrue(st.homed)
            self.assertEqual(sim.alarm_code, 0)
            ec.write_user_parameter(4, 0.50, allow_home_params=True)  # restore

    def test_refused_while_moving_or_pushing(self) -> None:
        ec, sim, rec = open_sim("slider", time_scale=1.0)
        with ec:
            ec.move_to(120.0, speed_mm_s=20.0, wait=False)
            rec.clear()
            with self.assertRaises(EcNotReadyError):
                ec.software_reset()
            self.assertEqual(writes(rec, 0x45), [])
            ec.stop(wait=True)
        ec, sim, rec = open_sim("slider")
        sim.obstacle_mm = 85.0
        with ec:
            self.assertTrue(ec.push(80.0, 10.0, 30.0, 1).contact)
            rec.clear()
            with self.assertRaises(EcNotReadyError):
                ec.software_reset()
            self.assertEqual(writes(rec, 0x45), [])

    def test_ec_answers_at_another_rate_after_the_reset(self) -> None:
        ec, sim, _ = open_sim("rod", baudrate=115200, reset_baudrate=38400)
        with ec:
            result = ec.software_reset(reset_wait_s=0.0)
            self.assertEqual((result.baudrate, result.baud_changed, ec.baudrate), (38400, True, 38400))
            self.assertEqual(sim.ec_baudrate, 38400)
            self.assertIsInstance(ec.status().position_mm, float)

    def test_lost_reply_is_confirmed_from_the_status(self) -> None:
        class LostReply(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1:4] == bytes([0x45, 0x00, 0x05]):  # the software reset happens, its reply never arrives
                    self.handle(frame)
                    return None
                return NotImplemented

        ec, sim, _ = open_sim(sim=LostReply(profile="rod", time_scale=50.0))
        with ec:
            result = ec.software_reset(reset_wait_s=0.0)
        self.assertTrue(result.reply_lost and not result.accepted)
        self.assertEqual(sim.software_resets, 1)

    def test_reset_that_did_not_happen_is_reported(self) -> None:
        class NoReset(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1:4] == bytes([0x45, 0x00, 0x05]) and frame[4:6] == bytes([0x00, 0x01]):  # the software reset
                    self.events.append("reset ignored")
                    return build_frame(self.slave, bytes([0x45, 0x00, 0x00]))  # accepted, not acted on
                return NotImplemented

        ec, _, _ = open_sim(sim=NoReset(profile="rod", time_scale=50.0))
        with ec:
            result = ec.software_reset(reset_wait_s=0.0)  # reported, never raised: a CON unit restarts without PMSS 0
        self.assertFalse(result.confirmed)
        self.assertTrue(any("NOT confirmed" in n for n in result.notes))

    def test_silent_ec_times_out(self) -> None:
        ec, _, _ = open_sim("rod", reset_s=1000.0)
        with ec, self.assertRaises(ModbusTimeout) as caught:
            ec.software_reset(reset_wait_s=0.0, timeout=0.3)
        self.assertIn("did not answer", " ".join(caught.exception.__notes__))


# =========================================================================== baud rate


class BaudChangeTest(unittest.TestCase):
    def test_direct(self) -> None:
        # VERIFIED on the EC-R6S rod: 115200 -> 230400 (0x15) and 230400 -> 115200 (0x14), sent at the current rate.
        ec, sim, rec = open_sim("rod", baudrate=115200)
        with ec:
            result = ec.set_baudrate(230400, settle_s=0.0)
            self.assertEqual((result.previous, result.baudrate, result.code), (115200, 230400, 0x15))
            self.assertTrue(result.verified and not result.probed and result.code_verified)
            self.assertEqual((ec.baudrate, sim.ec_baudrate), (230400, 230400))
            self.assertIn(hexbytes("00 45 00 04 00 15 00 05"), [f[:-2] for f in rec.tx])
            self.assertIn("volatile", result.note)
            ec.move_to(80.0, speed_mm_s=50.0)  # the link works at the new rate
            result = ec.set_baudrate(115200, settle_s=0.0)
            self.assertEqual((result.previous, result.baudrate, result.code), (230400, 115200, 0x14))
            self.assertEqual((ec.baudrate, sim.ec_baudrate), (115200, 115200))
            sim.power_cycle()  # volatile: 38400 after power-up (VERIFIED on the EC-DS4H slider)
            self.assertEqual(sim.ec_baudrate, 38400)

    def test_38400_is_sent_without_the_opt_in_and_taken_by_the_gripper(self) -> None:
        # The EC-GRBP8M gripper takes code 0x06 (VERIFIED); the check after the switch confirms it.
        ec, sim, _ = open_sim("gripper", baudrate=230400, ignored_baud_codes=())
        with ec:
            result = ec.set_baudrate(38400, settle_s=0.0)
            self.assertTrue(result.verified)
            self.assertTrue(result.code_verified)
            self.assertEqual((ec.bus.baudrate, sim.ec_baudrate), (38400, 38400))

    def test_direct_38400_is_ignored_like_on_the_rod(self) -> None:
        # The EC-R6S rod IGNORED code 0x06 ("38400") sent direct; it stayed at its rate.
        ec, sim, rec = open_sim("rod", baudrate=230400)
        with ec:
            with self.assertRaises(VerificationError) as caught:
                ec.set_baudrate(38400, settle_s=0.0, allow_unverified_code=True)
            self.assertIn("still answers at 230400", str(caught.exception))
            self.assertEqual((ec.bus.baudrate, sim.ec_baudrate), (230400, 230400))
            self.assertIn(hexbytes("00 45 00 04 00 06 00 05"), [f[:-2] for f in rec.tx])
            self.assertIn("baud code 0x06 (38400) ignored", sim.events)
            self.assertIsInstance(ec.status().position_mm, float)  # still talking at 230400

    def test_unverified_code_is_sent_with_a_warning(self) -> None:
        for rate in (9600, 19200, 57600, 14400, 28800, 76800):  # 38400 is sent and checked (see above)
            with self.subTest(rate=rate):
                ec, sim, _ = open_sim("rod", baudrate=115200)
                with ec:
                    with self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                        result = ec.set_baudrate(rate, settle_s=0.0)  # allow_unverified_code: no longer needed
                    self.assertEqual((result.baudrate, result.code_verified), (rate, False))
                    self.assertEqual(sim.ec_baudrate, rate)
                text = "\n".join(logs.output)
                self.assertIn("its code is UNVERIFIED", text)
                self.assertIn("Sent anyway", text)
                self.assertEqual("never probes it" in text, rate not in BAUD_DETECT_ORDER)

    def test_a_rate_without_a_code_is_refused(self) -> None:
        ec, _, rec = open_sim("rod", baudrate=115200)
        with ec:
            with self.assertRaisesRegex(SoftLimitError, "baud rate 12345 has no IAI code"):
                ec.set_baudrate(12345, settle_s=0.0, allow_unverified_code=True)
            self.assertEqual([f for f in rec.tx if f[0] == 0], [])  # nothing broadcast

    def test_failure_probes_every_coded_rate(self) -> None:
        class OtherTable(SimulatedEC):  # an EC whose code table differs from the driver's: 0x15 lands at 76800
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1:4] == bytes([0x45, 0x00, 0x04]) and frame[4:6] == bytes([0x00, 0x15]):
                    code = IAI_BAUD_CODES[76800]
                    return self.handle(build_frame(frame[0], frame[1:4] + struct.pack(">H", code) + frame[6:8]))
                return NotImplemented

        ec, sim, _ = open_sim(sim=OtherTable(profile="rod", baudrate=115200, time_scale=50.0))
        with ec:
            with self.assertRaises(VerificationError) as caught:
                ec.set_baudrate(230400, settle_s=0.0)
            self.assertIn("answers at 76800", str(caught.exception))
            self.assertEqual(ec.bus.baudrate, 76800)  # found although auto-baud at connect never tries 76800
            self.assertIsInstance(ec.status().position_mm, float)
            self.assertNotIn(76800, BAUD_DETECT_ORDER)

    def test_break_method(self) -> None:
        ec, sim, rec = open_sim("slider", baudrate=57600)
        with ec:
            result = ec.set_baudrate(115200, method="break", settle_s=0.0)
            self.assertTrue(result.verified)
            self.assertEqual((ec.baudrate, sim.ec_baudrate), (115200, 115200))
        self.assertIn("break 0.200 s", sim.events)
        self.assertIn("EC at 9600 baud (break)", sim.events)
        self.assertEqual(result.frame, "00 45 00 04 00 14 00 05 25 08")  # the captured frame

    def test_not_taken_is_detected_by_probing(self) -> None:
        # every baud command is accepted and ignored
        sim = SimulatedEC(baudrate=115200, time_scale=50.0, ignored_baud_codes=IAI_BAUD_CODES.values())
        ec, sim, _ = open_sim(sim=sim)
        with ec, self.assertRaises(VerificationError) as caught:
            ec.set_baudrate(230400, settle_s=0.0)
        self.assertIn("still answers at 115200", str(caught.exception))
        self.assertEqual(ec.bus.baudrate, 115200)

    def test_refusals(self) -> None:
        ec, _, rec = open_sim("slider", baudrate=115200)
        with ec:
            for rate in (1234, True, 460800):
                with self.assertRaises(SoftLimitError):
                    ec.set_baudrate(rate)  # type: ignore[arg-type]
            with self.assertRaises(ValueError):
                ec.set_baudrate(230400, method="bogus")
            self.assertEqual([f for f in rec.tx if f[0] == 0], [])
        ec, _, _ = open_sim("slider")  # no line model: nothing to change
        with ec, self.assertRaises(EcNotReadyError):
            ec.set_baudrate(230400)
        ec, sim, rec = open_sim("slider", baudrate=115200, time_scale=1.0)
        with ec:
            ec.move_to(120.0, speed_mm_s=20.0, wait=False)
            with self.assertRaises(EcNotReadyError):
                ec.set_baudrate(230400)
            self.assertEqual([f for f in rec.tx if f[0] == 0], [])
            ec.stop(wait=True)


# =========================================================================== stored-position moves


class MoveToPositionTest(unittest.TestCase):
    def test_slider_rows_need_allow_above_caps(self) -> None:
        ec, sim, rec = open_sim("slider")
        with ec:
            rec.clear()
            with self.assertRaises(SoftLimitError) as caught:
                ec.move_to_position(2)  # stored 248.50 mm/s > the 100 mm/s cap
            self.assertIn("allow_above_caps", str(caught.exception))
            self.assertEqual(writes(rec, 0x06), [])
            st = ec.move_to_position(2, allow_above_caps=True)
            self.assertEqual(st.position_mm, 75.0)
            self.assertTrue(st.in_position)
            self.assertFalse(sim.moving)
            self.assertIn(build_frame(1, hexbytes("06 98 00 00 02")), rec.tx)

    def test_takes_modbus_control_first(self) -> None:
        ec, sim, rec = open_sim("gripper", pmsl=False)
        with ec:
            st = ec.move_to_position(0)
        self.assertEqual(st.position_mm, 3.0)
        order = [f[1] for f in rec.tx if f[1] in (0x05, 0x06)]
        self.assertEqual(order[:2], [0x05, 0x06])  # PMSL before 0x9800 (the EC ignores 0x9800 with PMSS = 0, VERIFIED)
        self.assertTrue(sim.pmsl)

    def test_without_auto_control_nothing_is_sent(self) -> None:
        ec, _, rec = open_sim("gripper", pmsl=False, auto=False)
        with ec:
            rec.clear()
            with self.assertRaises(EcNotReadyError) as caught:
                ec.move_to_position(0)
        self.assertIn("silently ignores 0x9800", str(caught.exception))
        # reads only: FC03, and the FC11 identity the row check (P9 position mode, EC only) reads once
        self.assertEqual([f for f in rec.tx if f[1] not in (0x03, 0x11)], [])

    def test_ignored_command_raises_instead_of_hanging(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            ec.bus.write_coil(1, R.COIL_STOP, True)  # another master holds STOP: the EC ignores moves
            started = time.perf_counter()
            with self.assertRaises(MotionIncompleteError) as caught:
                ec.move_to_position(0)
            self.assertLess(time.perf_counter() - started, 3.0)
        self.assertIn("ignored", str(caught.exception))
        self.assertIn("move ignored (STOP held)", sim.events)

    def test_push_row_settles_past_a_transient_contact(self) -> None:
        sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0, push_blip_mm=6.76)
        ec, _, _ = open_sim(sim=sim)
        ec.poll_interval_s = 0.002
        with ec:
            st = ec.move_to_position(1)  # grip: approach 4.10, push 50 % to 9.50, no work -> miss
        self.assertTrue(st.push_missed)
        self.assertFalse(st.push_contact)
        self.assertEqual(st.position_mm, 9.5)

    def test_push_row_inside_its_zone_is_refused(self) -> None:
        ec, _, rec = open_sim("gripper")  # starts at 9.50, inside the grip push zone 4.10..9.50
        with ec:
            rec.clear()
            with self.assertRaises(EcNotReadyError) as caught:
                ec.move_to_position(1)
        self.assertIn("skip the approach", str(caught.exception))
        self.assertEqual(writes(rec, 0x06), [])

    def test_rod_rows(self) -> None:
        ec, sim, _ = open_sim("rod")
        with ec:
            with self.assertRaises(SoftLimitError):
                ec.move_to_position(0)  # 240 mm/s, decel 0.47 G: above the caps
            st = ec.move_to_position(1, allow_above_caps=True)  # push row: PPOW 26 (~10 %) is the EC's own, accepted
            self.assertTrue(st.push_missed)
            self.assertEqual(st.position_mm, 150.0)
            st = ec.move_to_position(0, allow_above_caps=True)  # back to B.End at its stored 240 mm/s
            self.assertEqual(st.position_mm, 0.0)

    def test_row_outside_the_stroke_and_bad_rows(self) -> None:
        ec, _, rec = open_sim("slider", stroke=(0.0, 100.0))
        with ec:
            rec.clear()
            with self.assertRaises(SoftLimitError) as caught:
                ec.move_to_position(1, allow_above_caps=True)  # 150.00 mm
            self.assertIn("outside the effective stroke", str(caught.exception))
            for bad in (3, -1, True, 1.0):
                with self.assertRaises(SoftLimitError):
                    ec.move_to_position(bad)  # type: ignore[arg-type]
            self.assertEqual(writes(rec, 0x06), [])

    def test_no_wait_returns_once_started(self) -> None:
        ec, sim, _ = open_sim("slider", time_scale=5.0)
        with ec:
            self.assertIsNone(ec.move_to_position(0, wait=False, allow_above_caps=True))
            self.assertTrue(sim.moving)
            ec.stop(wait=True)

    def test_already_there(self) -> None:
        ec, _, _ = open_sim("slider", position_mm=75.0)
        with ec:
            st = ec.move_to_position(2, allow_above_caps=True)
        self.assertEqual(st.position_mm, 75.0)


# =========================================================================== push power and unit codes


class PushPowerAndUnitCodeTest(unittest.TestCase):
    def test_rod_push_row_keeps_its_own_push_power(self) -> None:
        ec, sim, _ = open_sim("rod")
        with ec:
            row = ec.read_position_table(1)
            self.assertEqual((row.push_power, row.control_flags), (26, 0x0016))  # 26 / 255 = ~10 %: below 20 %
            entry = replace(row, position_mm=40.0, speed_mm_s=30.0, accel_g=0.3, decel_g=0.3, band_mm=110.0)
            written = ec.write_position_table(1, entry, True)  # the row values VERIFIED on the EC-R6S rod
            self.assertEqual((written.position_mm, written.push_power, written.checksum_ok), (40.0, 26, True))
            with self.assertRaises(SoftLimitError) as caught:
                ec.write_position_table(1, replace(entry, push_power=200), True)  # 78 % > MXPP 70 %: refused
            self.assertIn("different from the EC's own value (26)", str(caught.exception))
            self.assertIn("1..70 %", str(caught.exception))
            # below the recommended 20 % (no MNPP on the rod) is only a recommendation (the EC-R6S rod ran its own
            # 10 % push row, VERIFIED): accepted
            self.assertEqual(ec.write_position_table(1, replace(entry, push_power=30), True).push_power, 30)

    def test_unit_codes(self) -> None:
        # family names per unit code; the actuator kind comes from AKID, not the unit code
        self.assertEqual(R.UNIT_CODES[0x00], "EC standard")
        self.assertEqual(R.UNIT_CODES[0x0A], 'EC(MS) "2-size EC"')
        self.assertEqual(R.ec_family(0xED, 0x00), "EC(200V)")
        self.assertIsNone(R.ec_family(0xEC, 0x10))  # CCM conveyor motor: not an EC actuator
        # kept for compatibility, no longer used by the driver: no table of tested units remains
        self.assertEqual((R.UNIT_CODES_CONFIRMED, R.UNITS_SEEN_ON_HARDWARE, R.MODEL_PARAMETERS_SEEN_ON),
                         (frozenset(), frozenset(), {}))
        self.assertIsNone(R.ec_family(0xEC, 0x0C))  # unknown unit code: no family name
        self.assertEqual(R.ACTUATOR_KINDS[4], "gripper")


# =========================================================================== CLI


def run_cli(*argv: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stderr(err):
        code = cli.main(list(argv), out=out)
    return code, out.getvalue(), err.getvalue()


class CliTest(unittest.TestCase):
    def test_param_list_and_set(self) -> None:
        code, out, _ = run_cli("--sim", "--sim-profile", "rod", "param", "list")
        self.assertEqual(code, 0)
        self.assertIn("P8", out)
        p8 = next(line for line in out.splitlines() if line.startswith("  P8 "))
        self.assertIn("1:Disable", p8)
        self.assertIn("READ-ONLY", p8)
        self.assertNotIn("P7 ", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.2")
        self.assertEqual(code, 0)
        self.assertIn("0.10 mm -> 0.20 mm", out)
        self.assertIn("committed to non-volatile memory", out)
        self.assertIn("swreset", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.2", "--no-commit")
        self.assertIn("RAM ONLY", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.1", "--no-commit")
        self.assertEqual(code, 0)  # the value the unit holds: nothing written, not RAM ONLY
        self.assertIn("unchanged, the unit already holds this value; nothing written (--no-commit)", out)
        self.assertNotIn("RAM ONLY", out)
        code, out, err = run_cli("--sim", "--sim-profile", "gripper", "--json", "param", "set", "4", "0.6")
        self.assertEqual(code, 0, err)  # a home parameter: written with the home warning
        data = json.loads(out)
        self.assertEqual((data["after"], data["committed"], data["home_parameter"]), ("0.60 mm", True, True))
        self.assertTrue(any("home parameters" in w for w in data["warnings"]), data["warnings"])
        code, out, err = run_cli("--sim", "--sim-profile", "rod", "param", "set", "7", "1")
        self.assertEqual(code, 0, err)
        self.assertIn("not in the parameter list for unit 0x00", out)
        code, _, err = run_cli("--sim", "--sim-profile", "rod", "param", "set", "7", "0", "--allow-unlisted")
        self.assertEqual(code, 0, err)  # the old options are still accepted (and ignored)
        code, out, _ = run_cli("--sim", "--sim-profile", "pcon", "param", "list")
        self.assertIn("DANGEROUS", out)
        self.assertNotIn("--allow-", out)

    def test_dangerous_parameters_need_no_option(self) -> None:
        for key, value in (("22", "1.0"), ("78", "1"), ("7.7", "0")):  # home offset, a dangerous one, a raw item
            with self.subTest(key=key):
                code, out, err = run_cli("--sim", "--sim-profile", "pcon", "--json", "param", "set", key, value,
                                         "--no-commit")  # fmt: skip
                self.assertEqual(code, 0, err)
                self.assertTrue(json.loads(out)["warnings"])
        code, _, err = run_cli("--sim", "--sim-profile", "pcon", "param", "set", "78", "1", "--allow-dangerous",
                               "--allow-home-params", "--no-commit")
        self.assertEqual(code, 0, err)  # the old options are still accepted (and ignored)

    def test_any_model_is_listed_and_written_without_a_model_label(self) -> None:
        code, out, _ = run_cli("--sim", "param", "list")  # the slider: unit 0x02
        self.assertEqual(code, 0)
        lines = {line.split()[0]: line for line in out.splitlines() if line.startswith("  P")}
        self.assertNotIn("UNVERIFIED", lines["P5"])
        self.assertNotIn("--allow-unverified-model", lines["P5"])  # no per-model opt-in
        code, out, _ = run_cli("--sim", "param", "set", "5", "1")
        self.assertEqual(code, 0)
        self.assertIn("committed to non-volatile memory", out)
        self.assertNotIn("UNVERIFIED", out)
        for profile in ("gripper", "unknown"):
            code, out, _ = run_cli("--sim", "--sim-profile", profile, "param", "list")
            self.assertEqual(code, 0)
            self.assertNotIn("UNVERIFIED", out)

    def test_swreset_baud_goto_and_reset_names(self) -> None:
        code, out, _ = run_cli("--sim", "--sim-profile", "rod", "--sim-time-scale", "50", "swreset")
        self.assertEqual(code, 0)
        self.assertIn("software reset accepted (01 45 00 00)", out)
        self.assertIn("NEEDS HOME", out)
        code, out, _ = run_cli("--sim", "--sim-baud", "115200", "baud", "230400")
        self.assertEqual(code, 0)
        self.assertIn("baud 115200 -> 230400 (direct; broadcast 00 45 00 04 00 15 00 05", out)
        self.assertNotIn("not verified", out)
        code, out, _ = run_cli("--sim", "--sim-baud", "38400", "--json", "baud", "115200", "--method", "break")
        self.assertEqual(json.loads(out)["baudrate"], 115200)
        code, _, err = run_cli("--sim", "baud", "230400")  # the plain simulator has no baud rate
        self.assertEqual(code, 2)
        code, _, err = run_cli("--sim", "--sim-baud", "115200", "baud", "38400")  # sent, then checked: this unit ignores it
        self.assertEqual(code, 2)
        self.assertIn("not taken", err)
        self.assertIn("a power cycle brings it back to 38400", err)
        code, _, err = run_cli("--sim", "--sim-baud", "115200", "baud", "38400", "--allow-unverified-code")
        self.assertEqual(code, 2)
        self.assertIn("not taken", err)  # sent, then ignored by the simulator (as by the EC-R6S rod)
        code, out, _ = run_cli("--sim", "--sim-baud", "115200", "baud", "57600")
        self.assertEqual(code, 0)
        self.assertIn("warning: code 0x13 is not verified on hardware (sent anyway)", out)
        code, out, _ = run_cli("--sim", "--sim-baud", "115200", "baud", "57600", "--allow-unverified-code")
        self.assertEqual(code, 0)  # the old option is still accepted (and ignored)
        code, out, _ = run_cli("--sim", "--sim-profile", "gripper", "--sim-time-scale", "50", "goto", "0")
        self.assertEqual(code, 0)
        self.assertIn("position        3.00 mm", out)
        code, _, err = run_cli("--sim", "--sim-time-scale", "50", "--max-speed", "100", "goto", "2")  # above the cap
        self.assertEqual(code, 2)
        self.assertIn("allow_above_caps", err)
        code, _, _ = run_cli("--sim", "--sim-time-scale", "50", "--max-speed", "100", "goto", "2", "--allow-fast")
        self.assertEqual(code, 0)
        code, _, err = run_cli("--sim", "alarm-reset")
        self.assertEqual((code, err), (0, ""))
        code, _, err = run_cli("--sim", "reset")  # deprecated alias: still the ALARM reset
        self.assertEqual(code, 0)
        self.assertIn("deprecated and means ALARM reset", err)


# =========================================================================== simulator models


class SimulatorModelTest(unittest.TestCase):
    def test_line_baud_model(self) -> None:
        sim = SimulatedEC(baudrate=115200)
        bus = ModbusRTU(sim, min_gap_s=0.0, retries=0)
        self.assertTrue(bus.can_change_baudrate)
        bus.read_registers(1, R.PNOW, 1)
        bus.set_baudrate(38400)
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, R.PNOW, 1)  # the EC is still at 115200
        self.assertEqual(bus.detect_baudrate(1), 115200)
        sim.send_break(0.1)
        self.assertEqual(sim.ec_baudrate, 115200)  # < 150 ms: no effect
        sim.send_break(0.2)
        self.assertEqual(sim.ec_baudrate, 9600)
        sim.power_cycle()
        self.assertEqual(sim.ec_baudrate, 38400)
        self.assertIsNone(SimulatedEC().baudrate)  # default: no line model at all

    def test_parw_commit_window(self) -> None:
        # ON-WIRE: DRGE 0x0180 at +55 ms after the coil, 0x0100 at +235 ms; waiting for DSSE b7 = 1 as well is
        # UNVERIFIED.
        for dsse_busy in (False, True):
            with self.subTest(parw_dsse_busy=dsse_busy):
                sim = SimulatedEC(profile="gripper", parw_busy_s=0.3, parw_dsse_busy=dsse_busy)
                bus = ModbusRTU(sim, min_gap_s=0.0)
                self.assertEqual(bus.read_registers(1, R.DRGE, 1), [0x0100])
                self.assertTrue(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_PARW)  # idle 0x33C2: b7 = 1
                bus.write_registers(1, 0x0810, [0, 20])
                bus.write_coil(1, R.COIL_PARW, True)
                self.assertEqual(bus.read_registers(1, R.DRGE, 1), [0x0180])
                dsse_b7 = bool(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_PARW)
                self.assertEqual(dsse_b7, not dsse_busy)
                self.assertEqual(bus.read_registers(1, 0x0202, 1)[0] & R.DSSE_PARW, 0x80 if not dsse_busy else 0)
                self.assertEqual(sim.parameter_commits, 0)  # still writing
                time.sleep(0.35)
                self.assertEqual(bus.read_registers(1, R.DRGE, 1), [0x0100])
                self.assertTrue(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_PARW)
                self.assertEqual(sim.parameter_commits, 1)
                sim.power_cycle()
                self.assertEqual(bus.read_registers(1, 0x0810, 2), [0, 20])

    def test_restart_inside_the_commit_window_loses_it(self) -> None:
        sim = SimulatedEC(profile="gripper", parw_busy_s=5.0)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_registers(1, 0x0810, [0, 20])
        bus.write_coil(1, R.COIL_PARW, True)
        sim.power_cycle()  # worst case (unknown on the EC): the cut-off commit is lost
        self.assertIn("parameter commit (PARW) interrupted by a restart: lost", sim.events)
        self.assertEqual(bus.read_registers(1, 0x0810, 2), [0, 10])
        self.assertEqual(bus.read_registers(1, R.DRGE, 1), [0x0000])  # PMSS 0 after power-up, b7 clear
        self.assertEqual(sim.parameter_commits, 0)

    def test_baud_code_0x06_is_ignored(self) -> None:
        sim = SimulatedEC(baudrate=115200)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        flags = {"allow_unsafe": True, "allow_never_send": True, "settle_s": 0}
        bus.broadcast_iai_command(0, R.IAI_CMD_BAUD, 0x06, R.IAI_BAUD_OP2, **flags)
        self.assertEqual(sim.ec_baudrate, 115200)  # the EC-R6S rod ignored 0x06 sent direct
        bus.broadcast_iai_command(0, R.IAI_CMD_BAUD, 0x15, R.IAI_BAUD_OP2, **flags)
        self.assertEqual(sim.ec_baudrate, 230400)
        sim = SimulatedEC(baudrate=115200, ignored_baud_codes=())  # opt out: every code taken as listed
        ModbusRTU(sim, min_gap_s=0.0).broadcast_iai_command(0, R.IAI_CMD_BAUD, 0x06, 5, **flags)
        self.assertEqual(sim.ec_baudrate, 38400)

    def test_parw_ignored_without_modbus_control(self) -> None:
        sim = SimulatedEC(profile="gripper", pmsl=False)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_registers(1, 0x0810, [0, 20])
        bus.write_coil(1, R.COIL_PARW, True)
        self.assertEqual(bus.read_registers(1, R.DRGE, 1), [0x0000])  # b7 never set: the driver's "unconfirmed" case
        self.assertEqual(sim.parameter_commits, 0)
        sim.power_cycle()
        self.assertEqual(bus.read_registers(1, 0x0810, 2), [0, 10])

    def test_sfrs_coil_with_servo_on_raises_0x090(self) -> None:
        sim = SimulatedEC()
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_coil(1, R.COIL_SFRS, True)
        self.assertEqual((sim.alarm_code, sim.servo_on, sim.software_resets), (0x090, False, 0))  # as on the gripper


if __name__ == "__main__":
    unittest.main()
