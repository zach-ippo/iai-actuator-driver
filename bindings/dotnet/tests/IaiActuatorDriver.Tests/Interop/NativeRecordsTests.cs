using System;
using System.Linq;
using IaiActuatorDriver.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The core's outputs as the public records (stage 1 of the switch), checked against the managed decoders on the
    /// same simulated unit: the status flags of every profile, the ranges, a stored row both ways, the identity.
    /// </summary>
    public class NativeRecordsTests
    {
        [NativeTheory]
        [InlineData("slider")]
        [InlineData("gripper")]
        [InlineData("rod")]
        [InlineData("pcon")]
        [InlineData("rotary")]
        [InlineData("unknown")]
        public void The_core_status_and_the_managed_decoder_agree_on_every_profile(string profile)
        {
            using var sim = NativeSim.Open("{\"profile\": \"" + profile + "\", \"time_scale\": 20}");
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            using var axis = NativeAxis.OpenOnBus(bus, 1);
            axis.Connect();

            var core = axis.Status().ToRecord();
            var managed = Status.FromRegisters(bus.ReadRegisters(1, 0x9000, Registers.STATUS_BLOCK_WORDS));
            Assert.Equal(managed.PositionMm, core.PositionMm, 2);
            Assert.Equal(managed.AlarmCode, core.AlarmCode);
            Assert.Equal(managed.ServoOn, core.ServoOn);
            Assert.Equal(managed.Homed, core.Homed);
            Assert.Equal(managed.InPosition, core.InPosition);
            Assert.Equal(managed.Alarm, core.Alarm);
            Assert.Equal(managed.Warning, core.Warning);
            Assert.Equal(managed.ModbusEnabled, core.ModbusEnabled);
            Assert.Equal(managed.EmergencyStop, core.EmergencyStop);
            Assert.Equal(managed.Paused, core.Paused);
            Assert.Equal(managed.SafetySpeed, core.SafetySpeed);
            Assert.Equal(managed.Ready, core.Ready);
            foreach (var word in new[] { "DSS1", "DSS2", "DSSE", "DIPM", "DOPM", "ALMC", "PNOW" })
            {
                Assert.Equal(managed.Raw[word], core.Raw[word]);
            }

            Assert.Null(core.AtBackwardEnd);  // ZONS is not in the ABI's status (raw status words: a later ABI)
        }

        [NativeFact]
        public void Ranges_rows_identity_and_alarms_fill_the_public_records()
        {
            using var axis = NativeAxis.OpenSim("slider", new NativeSimOptions { TimeScale = 20 });
            var identity = axis.Connect().ToRecord();
            Assert.Equal(1, identity.Slave);
            Assert.Equal("mm", identity.Units);
            Assert.NotNull(identity.SystemCode);

            var limits = axis.Limits();
            var ranges = limits.ToRecord(axis.LimitsJson());
            Assert.Equal(limits.Position.Max, ranges.Position.Max);
            Assert.Equal(limits.Speed.Max, ranges.Speed.Max);
            Assert.Equal("mm", ranges.Units);
            Assert.NotEqual(string.Empty, ranges.Position.Source);

            var row = axis.ReadPositionTable(2).ToRecord();
            Assert.Equal(75.0, row.PositionMm, 1);
            var back = row.ToNative();
            Assert.Equal(row.SpeedMmS, back.SpeedMmS);
            Assert.Equal(row.ControlFlags, back.ControlFlags);
            Assert.Equal(row with { Checksum = null }, back.ToRecord());

            var alarms = axis.AlarmHistory().ToRecords();
            Assert.All(alarms, a => Assert.Equal(Alarms.AlarmName(a.Code), a.Name));
        }
    }
}
