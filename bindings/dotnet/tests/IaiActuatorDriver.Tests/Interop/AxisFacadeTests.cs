using System;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// <see cref="Axis"/> on the native core (switch stage S3): the constants against the core's (iad_constants_json), a
    /// status record against the raw words the core kept, the core's API identifiers in this library's spelling and the
    /// non-volatile bookkeeping shared per unit. On the simulator; no port opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class AxisFacadeTests
    {
        [NativeFact]
        public void The_axis_constants_are_the_cores()
        {
            var core = (JsonObject)MiniJson.Parse(NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_constants_json(b, s, out r)))!;
            Assert.Equal(core.Number("SETTLE_S"), Axis.SettleSDefault);
            Assert.Equal(core.Number("PUSH_SETTLE_S"), Axis.PushSettleSDefault);
            Assert.Equal(core.Number("STORED_START_S"), Axis.StoredStartS);
            Assert.Equal(core.Number("FALLBACK_SPEED_CAP"), Axis.FallbackSpeedCap);
            Assert.Equal(core.Number("FALLBACK_ACCEL_CAP_G"), Axis.FallbackAccelCapG);
            Assert.Equal(core.Number("FALLBACK_PUSH_CAP_PCT"), Axis.FallbackPushCapPct);
            Assert.Equal(core.Number("MOVE_BAND_MAX"), Axis.MoveBandMax);
            Assert.Equal(core.Number("P1_MIN_MM"), Axis.P1MinMm);
            Assert.Equal(new double?[] { Axis.P2MinMm, Axis.P2MaxMm }, core.Array("P2_RANGE_MM")!.Select(v => (double?)v));
            Assert.Equal(new double?[] { Axis.P4MinMm, Axis.P4MaxMm }, core.Array("P4_RANGE_MM")!.Select(v => (double?)v));
            Assert.Equal(core.Number("PARW_READY_S"), Axis.ParwReadyS);
            Assert.Equal(core.Number("PARW_CLEAR_S"), Axis.ParwClearS);
            Assert.Equal(core.Number("PARW_OBSERVE_S"), Axis.ParwObserveS);
            Assert.Equal(core.Int("NV_WRITE_CYCLES"), Axis.NvWriteCycles);
            Assert.Equal(core.Int("NV_WARN_PER_HOUR"), Axis.NvWarnPerHour);
            Assert.Equal(core.Number("NV_WINDOW_S"), Axis.NvWindowS);
            Assert.Equal(Axis.NvSkipNote, core.String("NV_SKIP_NOTE"));
            Assert.Equal(core.Number("RESET_WAIT_S"), Axis.ResetWaitS);
            Assert.Equal(core.Number("RESET_TIMEOUT_S"), Axis.ResetTimeoutS);
            Assert.Equal(core.Number("BREAK_S"), Axis.BreakS);
            Assert.Equal(core.Number("BAUD_SETTLE_S"), Axis.BaudSettleS);
        }

        [NativeFact]
        public void A_status_is_decoded_from_the_words_the_core_read()
        {
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Gripper));
            var (axis, _) = SimKit.Open(sim);
            using (axis)
            {
                var st = axis.Status();
                var read = Status.FromRegisters(axis.ReadRegisters(Registers.PNOW, 20)).Raw;
                Assert.True(st.Raw.Count >= 8);
                foreach (var name in new[] { "DSS1", "DSS2", "DSSE", "ALMC" }.Where(st.Raw.ContainsKey))
                {
                    Assert.Equal(read[name], st.Raw[name]);
                }

                Assert.Equal(sim.PositionMm, st.PositionMm, 2);
                Assert.Equal(sim.Homed, st.Homed);
            }
        }

        [NativeFact]
        public void Messages_name_the_api_as_this_library_does()
        {
            Assert.Equal("call EnableModbusControl() while the axis is at rest", NameMap.Managed("call enable_modbus_control() while the axis is at rest"));
            Assert.Equal("pass allowNonvolatile: true to write it", NameMap.Managed("pass allow_nonvolatile to write it"));
            Assert.Equal("(MaxSpeedMmS); nothing sent", NameMap.Managed("(max_speed_mm_s); nothing sent"));
            Assert.Equal("wait or Stop() first", NameMap.Managed("wait or stop() first"));
            Assert.Equal("finish_home() and configured_max_speed_mm_s", NameMap.Managed("finish_home() and configured_max_speed_mm_s"));
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                // row 2 runs at 248.50 mm/s, above this axis's speed cap: the core names max_speed_mm_s
                var row = axis.ReadPositionTable(2) with { PositionMm = 61.0 };
                var refused = Assert.Throws<SoftLimitException>(() => axis.WritePositionTable(2, row));
                Assert.Contains("MaxSpeedMmS", refused.Message);
                Assert.DoesNotContain("max_speed_mm_s", refused.Message);
            }
        }

        [NativeFact]
        public void The_non_volatile_count_is_kept_per_unit_in_the_core()
        {
            var (axis, sim, _) = SimKit.MakeAxis(maxSpeed: 700.0);
            using (axis)
            {
                var stored = axis.ReadPositionTable(2);
                axis.WritePositionTable(2, stored with { PositionMm = 60.0 }, allowNonvolatile: true);
                var again = new Axis(SimKit.Bus(sim), 1, SimKit.MakeOptions(maxSpeed: 700.0)); // a new object on the same unit
                Assert.Equal(1, again.NonVolatileWrites);
                Assert.Equal(1, again.NonVolatileWritesLastHour);
                NativeMethods.iad_axis_nonvolatile_writes(again.Core.Handle, out var total, out _, out _);
                Assert.Equal(1, total);
            }
        }
    }
}
