using System;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text.Json;
using System.Threading;
using IaiActuatorDriver.Cli;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The non-volatile write guard (simulator only). The units' non-volatile memory lasts about 100,000 write cycles (IAI).
    /// Parameter commits and stored-row writes skip a write that would change nothing (<c>force: true</c> writes anyway), every
    /// axis counts its non-volatile writes, and more than <see cref="Axis.NvWarnPerHour"/> in the last hour logs one WARNING per
    /// hour (never a refusal).
    /// </summary>
    [Collection(SimCollection.Name)]
    public class NvGuardTests
    {
        private static (Axis Axis, SimulatedUnit Sim, FrameRecorder Rec) Gripper() => SimKit.MakeAxis(SimKit.Sim(SimProfile.Gripper));

        /// <summary>A new axis object on <paramref name="sim"/> (a reconnect, or a new object per job, in this process).</summary>
        private static Axis Reconnect(SimulatedUnit sim, double maxSpeed = 100.0) => new Axis(SimKit.Bus(sim), 1, SimKit.MakeOptions(maxSpeed: maxSpeed));

        private static bool IsParwFrame(byte[] data) =>
            data.Length >= 6 && data[1] == 0x05 && ((data[2] << 8) | data[3]) == Registers.COIL_PARW && data[4] == 0xFF;

        /// <summary>The PARW coil is answered but, while <see cref="IgnoreParw"/>, never latched: the commit is not confirmed.</summary>
        private sealed class NoLatchSim : SimulatedUnit
        {
            public NoLatchSim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            public bool IgnoreParw { get; set; } = true;

            protected override void WriteCoil(int address, bool on)
            {
                if (address == Registers.COIL_PARW && IgnoreParw)
                {
                    return;
                }

                base.WriteCoil(address, on);
            }
        }

        /// <summary>While <see cref="LoseParw"/>, the PARW coil frame never reaches the unit (nothing runs, nothing is answered).</summary>
        private sealed class LostParwSim : SimulatedUnit
        {
            public LostParwSim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            public bool LoseParw { get; set; } = true;

            protected override void WriteFrame(byte[] data)
            {
                if (LoseParw && IsParwFrame(data))
                {
                    RecordRequest(data);
                    return;
                }

                base.WriteFrame(data);
            }
        }

        // ------------------------------------------------------------------ parameters

        [Fact]
        public void An_unchanged_value_writes_and_commits_nothing()
        {
            var (axis, sim, rec) = Gripper();
            using (axis)
            {
                rec.Clear();
                var result = axis.WriteParameter(2, 0.10); // P2 is 0.10 on the simulated gripper
                Assert.True(result.Skipped);
                Assert.False(result.Changed);
                Assert.False(result.Committed);
                Assert.Equal(Axis.NvSkipNote, result.Note);
                Assert.True(result.Saved); // the unit already holds it
                Assert.Contains("already holds", result.TakesEffect);
                Assert.Empty(SimKit.Sent(rec, 0x10));
                Assert.Equal(0, sim.ParameterCommits);
                Assert.Equal(0, axis.NonVolatileWrites);
                Assert.Equal(true, result.ToDictionary()["skipped"]);
            }
        }

        [Fact]
        public void Force_commits_an_unchanged_value()
        {
            var (axis, sim, _) = Gripper();
            using (axis)
            {
                var result = axis.WriteParameter(2, 0.10, force: true);
                Assert.False(result.Skipped);
                Assert.True(result.Forced);
                Assert.True(result.Committed);
                Assert.True(result.CommitConfirmed);
                Assert.Equal(1, sim.ParameterCommits);
                Assert.Equal(1, axis.NonVolatileWrites);
            }
        }

        [Fact]
        public void A_changed_value_is_committed_and_counted()
        {
            var (axis, sim, _) = Gripper();
            using (axis)
            {
                var result = axis.WriteParameter(2, 0.20);
                Assert.False(result.Skipped);
                Assert.True(result.Changed);
                Assert.True(result.Committed);
                Assert.Equal(1, sim.ParameterCommits);
                Assert.Equal(1, axis.NonVolatileWrites);
                Assert.Equal(1, axis.NonVolatileWritesLastHour);
            }
        }

        [Fact]
        public void Earlier_RAM_only_writes_are_still_committed()
        {
            var (axis, sim, _) = Gripper();
            using (axis)
            {
                var ram = axis.WriteParameter(5, "1:Enable", commit: false); // RAM only: waits for a commit
                Assert.True(ram.Changed);
                Assert.False(ram.Committed);
                Assert.False(ram.Skipped);
                var result = axis.WriteParameter(2, 0.10); // unchanged, but the commit saves P5
                Assert.False(result.Skipped);
                Assert.False(result.Changed);
                Assert.True(result.Committed);
                Assert.Equal(1, sim.ParameterCommits);
                Assert.True(axis.WriteParameter(2, 0.10).Skipped); // nothing left uncommitted now
                Assert.Equal(1, sim.ParameterCommits);
                Assert.Equal(1, axis.NonVolatileWrites);
            }
        }

        [Fact]
        public void An_unchanged_RAM_only_write_is_not_pending()
        {
            var (axis, sim, _) = Gripper();
            using (axis)
            {
                Assert.False(axis.WriteParameter(2, 0.10, commit: false).Skipped); // nothing asked of the memory
                Assert.True(axis.WriteParameter(2, 0.10).Skipped); // nothing was written to RAM either
                Assert.Equal(0, sim.ParameterCommits);
            }
        }

        [Fact]
        public void A_software_reset_drops_the_pending_RAM_only_writes()
        {
            var (axis, sim, _) = Gripper();
            using (axis)
            {
                axis.WriteParameter(5, "1:Enable", commit: false);
                Assert.True(axis.SoftwareReset().Confirmed); // the restart reloaded the parameters: P5 is gone
                Assert.True(axis.WriteParameter(2, 0.10).Skipped);
                Assert.Equal(0, sim.ParameterCommits);
            }
        }

        [Fact]
        public void An_unconfirmed_commit_is_committed_again()
        {
            var sim = new NoLatchSim(SimKit.Sim(SimProfile.Gripper));
            var axis = Reconnect(sim);
            var first = axis.WriteParameter(2, 0.20);
            Assert.True(first.Committed);
            Assert.False(first.CommitConfirmed);
            Assert.False(first.Saved);
            sim.IgnoreParw = false;
            var retry = axis.WriteParameter(2, 0.20); // the same value: the commit runs, it is not skipped
            Assert.False(retry.Skipped);
            Assert.False(retry.Changed);
            Assert.True(retry.Committed);
            Assert.True(retry.Saved);
            Assert.Equal(1, sim.ParameterCommits);
            Assert.True(axis.WriteParameter(2, 0.20).Skipped); // confirmed now: nothing pending
            sim.PowerCycle();
            Assert.StartsWith("0.20", Reconnect(sim).ReadParameter(2).Text);
        }

        [Fact]
        public void A_lost_PARW_coil_is_committed_again()
        {
            var sim = new LostParwSim(SimKit.Sim(SimProfile.Gripper));
            var axis = Reconnect(sim);
            Assert.Throws<ModbusTimeoutException>(() => axis.WriteParameter(2, 0.20));
            Assert.Equal(0, sim.ParameterCommits);
            sim.LoseParw = false;
            var retry = axis.WriteParameter(2, 0.20);
            Assert.False(retry.Skipped);
            Assert.True(retry.Committed);
            Assert.True(retry.Saved);
            Assert.Equal(1, sim.ParameterCommits);
        }

        [Fact]
        public void A_RAM_only_write_is_seen_by_a_new_axis_object()
        {
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Gripper));
            Assert.False(Reconnect(sim).WriteParameter(2, 0.20, commit: false).Committed);
            var again = Reconnect(sim); // a reconnect in the same process
            var result = again.WriteParameter(2, 0.20);
            Assert.False(result.Skipped);
            Assert.True(result.Committed);
            Assert.True(result.Saved);
            Assert.Equal(1, sim.ParameterCommits);
        }

        [Fact]
        public void A_serial_port_is_known_by_its_name()
        {
            // No port is opened: axes on simulators get the bookkeeping a serial port's name gives (the core's, shared per key).
            var forget = Utf8.Encode(Axis.NvPortKey("COM4", 1).Substring(0, 9)); // "port:com4" / "port:COM4"
            NativeErrors.Check(NativeMethods.iad_nv_forget(forget));
            var made = new System.Collections.Generic.List<Axis>();
            Axis On(string port, int slave)
            {
                var (axis, _, _) = SimKit.MakeAxis();
                made.Add(axis);
                NativeErrors.Check(NativeMethods.iad_axis_set_nv_unit(axis.Core.Handle, Utf8.Encode(Axis.NvPortKey(port, slave)), Utf8.Encode(port + " slave " + slave)));
                return axis;
            }

            try
            {
                var first = On("COM42", 1);
                first.NoteNonVolatileWrite("test", Timing.Now);
                Assert.Equal(1, On("COM42", 1).NonVolatileWrites); // a new handle, the same unit
                Assert.Equal(0, On("COM42", 2).NonVolatileWrites);
                Assert.Equal(0, On("COM43", 1).NonVolatileWrites);
                Assert.Equal(RuntimeInformation.IsOSPlatform(OSPlatform.Windows) ? 1 : 0, On("com42", 1).NonVolatileWrites);
            }
            finally
            {
                foreach (var axis in made)
                {
                    axis.Dispose();
                }

                NativeErrors.Check(NativeMethods.iad_nv_forget(forget));
            }
        }

        [Fact]
        public void A_raw_item_and_a_CON_parameter_are_guarded_too()
        {
            var (axis, sim, rec) = Gripper();
            using (axis)
            {
                var raw = Convert.ToInt64(axis.ReadParameter("8.9").Unsigned);
                rec.Clear();
                var result = axis.WriteParameter("8.9", raw, allowDangerous: true);
                Assert.True(result.Skipped);
                Assert.Empty(SimKit.Sent(rec, 0x10));
                Assert.Equal(0, sim.ParameterCommits);
            }

            var (con, conSim, _) = SimKit.MakeAxis(SimKit.Sim(SimProfile.Pcon));
            using (con)
            {
                var current = con.Parameters().Single(p => p.Number == 34).Value!; // PSHV, written in the PCON test of the suite
                var result = con.WriteParameter(34, current);
                Assert.True(result.Skipped);
                Assert.False(result.Committed);
                Assert.Equal(0, conSim.ParameterCommits);
                Assert.DoesNotContain(result.Warnings, w => w.Contains("software reset"));
                var forced = con.WriteParameter(34, current, force: true);
                Assert.True(forced.Committed);
                Assert.Equal(1, conSim.ParameterCommits);
                Assert.Equal(1, con.NonVolatileWrites);
            }
        }

        // ------------------------------------------------------------------ position table

        [Fact]
        public void An_unchanged_row_is_not_written()
        {
            var (axis, sim, rec) = SimKit.MakeAxis(maxSpeed: 700.0); // row 2 runs at 248.50 mm/s
            using (axis)
            {
                var stored = axis.ReadPositionTable(2);
                rec.Clear();
                var back = axis.WritePositionTable(2, stored, allowNonvolatile: true);
                Assert.True(back.Skipped);
                Assert.Equal(stored, back); // the stored row (Skipped does not take part in equality)
                Assert.Equal(stored.GetHashCode(), back.GetHashCode());
                Assert.Empty(SimKit.Sent(rec, 0x10));
                Assert.Equal(0, sim.NonvolatileWrites);
                Assert.Equal(0, axis.NonVolatileWrites);
                Assert.False(axis.ReadPositionTable(2).Skipped);
                Assert.Equal(true, back.ToDictionary()["skipped"]);
            }
        }

        [Fact]
        public void A_changed_or_forced_row_is_written_and_counted()
        {
            var (axis, sim, _) = SimKit.MakeAxis(maxSpeed: 700.0);
            using (axis)
            {
                var stored = axis.ReadPositionTable(2);
                var back = axis.WritePositionTable(2, stored with { PositionMm = 60.0 }, allowNonvolatile: true);
                Assert.False(back.Skipped);
                Assert.Equal(1, sim.NonvolatileWrites);
                Assert.Equal(1, axis.NonVolatileWrites);
                var again = axis.WritePositionTable(2, back, allowNonvolatile: true, force: true);
                Assert.False(again.Skipped);
                Assert.Equal(2, sim.NonvolatileWrites);
                Assert.Equal(2, axis.NonVolatileWrites);
            }
        }

        [Fact]
        public void The_guards_still_come_first()
        {
            var (axis, sim, _) = SimKit.MakeAxis(maxSpeed: 700.0);
            using (axis)
            {
                var stored = axis.ReadPositionTable(2);
                // simple-data row: refused, not "skipped"
                Assert.Throws<NonVolatileWriteException>(() => axis.WritePositionTable(260, stored));
                // no opt-in needed: the unchanged row is skipped
                Assert.True(axis.WritePositionTable(2, stored).Skipped);
                Assert.Equal(0, sim.NonvolatileWrites);
            }
        }

        // ------------------------------------------------------------------ rate warning

        [Fact]
        public void One_warning_per_hour_and_never_a_refusal()
        {
            var (axis, sim, _) = SimKit.MakeAxis(maxSpeed: 700.0);
            using (axis)
            {
                var t0 = Timing.Now;
                using (var quiet = new LogCapture())
                {
                    for (var i = 0; i < Axis.NvWarnPerHour; i++)
                    {
                        axis.NoteNonVolatileWrite("test", t0);
                    }

                    Assert.Empty(quiet.Messages);
                }

                string message;
                using (var loud = new LogCapture())
                {
                    axis.NoteNonVolatileWrite("test", t0);
                    message = Assert.Single(loud.Messages);
                }

                Assert.Contains($"{Axis.NvWarnPerHour + 1} non-volatile writes in the last hour on slave 1", message);
                Assert.Contains("about 100,000 write cycles (about 68 days at this rate)", message);
                Assert.Contains("use MoveTo (direct moves are RAM only)", message);
                using (var once = new LogCapture())
                {
                    // one per hour, and the writes still go through
                    var stored = axis.ReadPositionTable(2);
                    var written = axis.WritePositionTable(2, stored with { PositionMm = 61.0 }, allowNonvolatile: true);
                    Assert.False(written.Skipped);
                    Assert.Empty(once.Messages);
                }

                Assert.Equal(1, sim.NonvolatileWrites);
                Assert.Equal(Axis.NvWarnPerHour + 2, axis.NonVolatileWrites);

                // An hour later the old writes have left the window and the warning may come again.
                var later = t0 + Axis.NvWindowS + 10.0;
                Assert.Equal(0, axis.NonVolatileWritesLastHourAt(later));
                using (var quiet = new LogCapture())
                {
                    axis.NoteNonVolatileWrite("test", later);
                    Assert.Empty(quiet.Messages);
                }

                Assert.Equal(1, axis.NonVolatileWritesLastHourAt(later));
                using (var again = new LogCapture())
                {
                    for (var i = 0; i < Axis.NvWarnPerHour; i++)
                    {
                        axis.NoteNonVolatileWrite("test", later);
                    }

                    Assert.Single(again.Messages);
                }
            }
        }

        [Fact]
        public void Counted_per_unit_across_axis_objects()
        {
            var (axis, sim, _) = SimKit.MakeAxis(maxSpeed: 700.0); // row 2 runs at 248.50 mm/s
            using (var quiet = new LogCapture())
            {
                for (var i = 0; i < Axis.NvWarnPerHour - 1; i++)
                {
                    axis.NoteNonVolatileWrite("test", Timing.Now);
                }

                Assert.Empty(quiet.Messages);
            }

            for (var i = 0; i < 2; i++)
            {
                var job = Reconnect(sim, 700.0); // open, write a row, drop it: one object per job
                var stored = job.ReadPositionTable(2);
                using (var capture = new LogCapture())
                {
                    job.WritePositionTable(2, stored with { PositionMm = 60.0 + i }, allowNonvolatile: true);
                    if (i == 0)
                    {
                        Assert.Empty(capture.Messages);
                    }
                    else
                    {
                        Assert.Contains($"{Axis.NvWarnPerHour + 1} non-volatile writes in the last hour", Assert.Single(capture.Messages));
                    }
                }

                Assert.Equal(Axis.NvWarnPerHour + i, job.NonVolatileWrites);
                Assert.Equal(Axis.NvWarnPerHour + i, job.NonVolatileWritesLastHour);
            }

            Assert.Equal(2, sim.NonvolatileWrites);
            var (other, _, _) = SimKit.MakeAxis(); // another unit: its own count
            using (other)
            {
                Assert.Equal(0, other.NonVolatileWrites);
            }
        }

        [Fact]
        public void The_counters_are_read_only()
        {
            Assert.Null(typeof(Axis).GetProperty(nameof(Axis.NonVolatileWrites))!.SetMethod);
            Assert.Null(typeof(Axis).GetProperty(nameof(Axis.NonVolatileWritesLastHour))!.SetMethod);
            Assert.Equal(100_000, Axis.NvWriteCycles);
            Assert.Equal(60, Axis.NvWarnPerHour);
        }

        // ------------------------------------------------------------------ command line

        private static readonly CliEnvironment NoHardware = new CliEnvironment
        {
            ListPorts = () => Array.Empty<SerialPortInfo>(),
            OpenSerial = (name, baud, timeout) => throw new InvalidOperationException("a test must never open a real serial port: " + name),
        };

        private static (int Code, string Out, string Err) Run(params string[] args)
        {
            var output = new StringWriter();
            var error = new StringWriter();
            var code = CliApp.Run(args, output, error, NoHardware, CancellationToken.None);
            return (code, output.ToString(), error.ToString());
        }

        [Fact]
        public void Param_set_prints_the_note_and_takes_force()
        {
            var skipped = Run("--sim", "gripper", "param", "set", "2", "0.1");
            Assert.Equal(0, skipped.Code);
            Assert.Contains(Axis.NvSkipNote, skipped.Out);
            Assert.Contains("--force", skipped.Out);
            Assert.DoesNotContain("committed to non-volatile memory", skipped.Out);
            var forced = Run("--sim", "gripper", "param", "set", "2", "0.1", "--force");
            Assert.Equal(0, forced.Code);
            Assert.Contains("committed to non-volatile memory", forced.Out);
            var json = Run("--sim", "gripper", "--json", "param", "set", "2", "0.1");
            Assert.Equal(0, json.Code);
            Assert.True(JsonDocument.Parse(json.Out).RootElement.GetProperty("skipped").GetBoolean());
            Assert.Equal(2, Run("--sim", "gripper", "param", "list", "--force").Code); // --force belongs to 'param set'
        }

        [Fact]
        public void No_commit_and_skip_lines_say_how_to_commit_a_RAM_only_value()
        {
            var ram = Run("--sim", "gripper", "param", "set", "2", "0.20", "--no-commit");
            Assert.Equal(0, ram.Code);
            Assert.Contains("'param set 2 0.20 --force'", ram.Out);
            var skipped = Run("--sim", "gripper", "param", "set", "2", "0.1"); // a new run: it cannot see an earlier RAM-only write
            Assert.Equal(0, skipped.Code);
            Assert.Contains("earlier --no-commit run left this value in RAM only", skipped.Out);
        }
    }
}
