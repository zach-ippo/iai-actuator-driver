using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using IaiActuatorDriver.Cli;
using IaiActuatorDriver.Simulation;
using IaiActuatorDriver.Tests.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The commands the Python command line has and the C# one gained with the public API parity: <c>check</c> (the self-test and its JSON
    /// report), <c>backup</c> (a read-only JSON backup) and <c>recover</c> (the recovery report, never a move). Run on the built-in
    /// simulator; no serial port is opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class CliParityTests
    {
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

        private static string TempFile(string name) => Path.Combine(Path.GetTempPath(), "iad_cli_" + Guid.NewGuid().ToString("N") + "_" + name);

        [NativeFact]
        public void Check_prints_each_line_writes_the_report_and_exits_zero_when_nothing_failed()
        {
            var report = TempFile("check.json");
            try
            {
                var (code, output, error) = Run("--sim", "gripper", "check", "--report", report);
                Assert.Equal(0, code);
                Assert.Empty(error);
                var lines = output.Split('\n').Select(l => l.TrimEnd('\r')).ToList();
                Assert.Equal(CliApp.Banner, lines[0]);
                Assert.Equal("compatibility check on sim, slave 1, strategy coil", lines[1]);
                Assert.StartsWith("PASS  link               answered FC03", lines[2]);
                Assert.Contains(lines, l => l.StartsWith("PASS  slave_id ", StringComparison.Ordinal));
                Assert.Contains(lines, l => l.StartsWith("SKIP  motion ", StringComparison.Ordinal) && l.Contains("not requested"));
                Assert.Contains("RESULT PASS - report written to " + report, output);

                using var doc = JsonDocument.Parse(File.ReadAllText(report));
                Assert.Equal("PASS", doc.RootElement.GetProperty("result").GetString());
                Assert.Equal("sim", doc.RootElement.GetProperty("port").GetString());
                Assert.False(doc.RootElement.GetProperty("motion_requested").GetBoolean());
                Assert.Equal("link", doc.RootElement.GetProperty("items")[0].GetProperty("name").GetString());
            }
            finally
            {
                File.Delete(report);
            }
        }

        [NativeFact]
        public void Check_with_motion_moves_and_returns_and_a_bad_report_path_falls_back_to_the_temp_folder()
        {
            var name = "iad_cli_" + Guid.NewGuid().ToString("N") + "_motion.json";
            var fallback = Path.Combine(Path.GetTempPath(), name);
            try
            {
                var missing = Path.Combine(Path.GetTempPath(), "iad_no_such_folder_" + Guid.NewGuid().ToString("N"), name);
                var (code, output, error) = Run("--sim", "slider", "check", "--motion", "--report", missing);
                Assert.Equal(0, code);
                Assert.Contains("warning: could not write the report to " + missing, error);
                Assert.Contains("(fallback location)", output);
                Assert.True(File.Exists(fallback));
                using var doc = JsonDocument.Parse(File.ReadAllText(fallback));
                Assert.True(doc.RootElement.GetProperty("motion_requested").GetBoolean());
                Assert.Contains(doc.RootElement.GetProperty("items").EnumerateArray(), i => i.GetProperty("name").GetString()!.Contains("motion") && i.GetProperty("result").GetString() == "PASS");
            }
            finally
            {
                File.Delete(fallback);
            }
        }

        [NativeFact]
        public void Check_with_motion_stops_on_Ctrl_C_and_exits_130()
        {
            // ON-WIRE on a PCON-CB (driver 1.1.0): Ctrl+C did not reach check --motion: it moved out and back and exited 0
            using var cts = new CancellationTokenSource();
            CancelAtDirectMove? line = null;
            var env = new CliEnvironment
            {
                ListPorts = () => Array.Empty<SerialPortInfo>(),
                OpenSerial = (name, baud, timeout) => throw new InvalidOperationException("a test must never open a real serial port: " + name),
                CreateSim = (profile, slaves, baud) => line = new CancelAtDirectMove(SimSupport.CreateLine(profile, slaves, baud), cts),
            };
            var report = TempFile("check_cancelled.json");
            try
            {
                var output = new StringWriter();
                var error = new StringWriter();
                var code = CliApp.Run(new[] { "--sim", "slider", "check", "--motion", "--report", report }, output, error, env, cts.Token);
                Assert.Equal(130, code);
                Assert.Contains("interrupted", error.ToString());
                Assert.NotNull(line);
                Assert.Equal(1, line!.DirectMoves);  // motion_out only: no move back after Ctrl+C
                Assert.Contains(line.StopCoilWrites, on => on);  // the decel STOP
            }
            finally
            {
                File.Delete(report);
            }
        }

        /// <summary>The simulated line, with Ctrl+C (the token cancelled, on another thread) as the first 0x9900 move frame goes out.</summary>
        private sealed class CancelAtDirectMove : ITransport
        {
            private readonly ITransport _inner;
            private readonly CancellationTokenSource _cts;

            public CancelAtDirectMove(ITransport inner, CancellationTokenSource cts)
            {
                _inner = inner;
                _cts = cts;
            }

            public int DirectMoves { get; private set; }

            public System.Collections.Generic.List<bool> StopCoilWrites { get; } = new System.Collections.Generic.List<bool>();

            public int BaudRate { get => _inner.BaudRate; set => _inner.BaudRate = value; }

            public TimeSpan ReadTimeout { get => _inner.ReadTimeout; set => _inner.ReadTimeout = value; }

            public void Write(byte[] buffer, int offset, int count)
            {
                if (count > 5 && buffer[offset + 1] == 0x10 && buffer[offset + 2] == 0x99 && buffer[offset + 3] == 0x00 && ++DirectMoves == 1)
                {
                    ThreadPool.QueueUserWorkItem(_ => _cts.Cancel());
                }

                if (count > 5 && buffer[offset + 1] == 0x05 && ((buffer[offset + 2] << 8) | buffer[offset + 3]) == Registers.COIL_STOP)
                {
                    StopCoilWrites.Add(buffer[offset + 4] == 0xFF);
                }

                _inner.Write(buffer, offset, count);
            }

            public int Read(byte[] buffer, int offset, int count) => _inner.Read(buffer, offset, count);

            public void ResetInputBuffer() => _inner.ResetInputBuffer();

            public void SendBreak(TimeSpan duration) => _inner.SendBreak(duration);

            public void Close() => _inner.Close();

            public void Dispose() => _inner.Dispose();
        }

        [NativeFact]
        public void Backup_writes_the_file_and_summarises_what_it_holds()
        {
            var file = TempFile("backup.json");
            try
            {
                var (code, output, _) = Run("--sim", "gripper", "backup", file, "--rows", "0,1");
                Assert.Equal(0, code);
                Assert.StartsWith("backup written to " + file + " (read-only: FC03 / FC11 only; ", output);
                Assert.Contains("contents   identity, directory, ", output);
                Assert.Contains(" parameter items (", output);
                Assert.Contains(", 2 position rows, ", output);
                Assert.Contains(" alarm entries, maintenance counters", output);
                using var doc = JsonDocument.Parse(File.ReadAllText(file));
                Assert.Equal("ec_driver.backup/1", doc.RootElement.GetProperty("meta").GetProperty("schema").GetString());
                Assert.Equal(2, doc.RootElement.GetProperty("positions").GetProperty("rows").GetArrayLength());

                var json = Run("--sim", "gripper", "--json", "backup", file);
                Assert.Equal(0, json.Code);
                Assert.Equal("ec_driver.backup/1", JsonDocument.Parse(json.Out).RootElement.GetProperty("meta").GetProperty("schema").GetString());
            }
            finally
            {
                File.Delete(file);
            }
        }

        [NativeFact]
        public void Backup_refuses_a_bad_rows_option_before_opening_anything()
        {
            foreach (var bad in new[] { "x", "1,a", "-1", string.Empty, "," })
            {
                var (code, output, error) = Run("--sim", "gripper", "backup", "--rows", bad);
                Assert.Equal(2, code);
                Assert.Equal($"error: --rows must be used, all or row numbers such as 0,1,2 (got '{bad}'){Environment.NewLine}", error);
                Assert.Empty(output);
            }

            Assert.Equal(2, Run("--sim", "gripper", "backup", "a.json", "b.json").Code);
        }

        [NativeFact]
        public void Recover_reports_the_state_and_never_moves_the_axis()
        {
            var (code, output, error) = Run("--sim", "gripper", "recover", "--wait", "5");
            Assert.Equal(0, code);
            Assert.Empty(error);
            Assert.StartsWith("sim: recovered from ok", output);
            Assert.Contains("PC control yes", output);
            Assert.Contains("servo      ON", output);
            Assert.Contains("homed      yes", output);
            Assert.Contains("position        9.50 mm", output);
            Assert.EndsWith("ready to move: yes; needs attention: no (nothing was moved)" + Environment.NewLine, output);

            var json = Run("--sim", "gripper", "--json", "recover", "--wait", "5");
            Assert.Equal(0, json.Code);
            var root = JsonDocument.Parse(json.Out).RootElement;
            Assert.True(root.GetProperty("same_unit").GetBoolean());
            Assert.True(root.GetProperty("ready_to_move").GetBoolean());
        }

        [NativeFact]
        public void Recover_takes_a_target_and_a_backup_and_refuses_another_units_backup()
        {
            var file = TempFile("backup.json");
            try
            {
                Assert.Equal(0, Run("--sim", "gripper", "backup", file, "--rows", "0,1").Code);
                var (code, output, _) = Run("--sim", "gripper", "recover", "--backup", file, "--target", "5", "--no-control", "--wait", "5");
                Assert.Equal(0, code);
                Assert.Contains("position        9.50 mm (target 5.00, +4.50)", output);
                Assert.Contains("note:      position 9.50 is +4.50 from the last commanded target 5.00", output);
                Assert.EndsWith("needs attention: YES (nothing was moved)" + Environment.NewLine, output);

                var other = Run("--sim", "slider", "recover", "--backup", file, "--wait", "3");
                Assert.Equal(2, other.Code);
                Assert.Contains("UnitReplacedException", other.Err);

                var missing = Run("--sim", "gripper", "recover", "--backup", file + ".missing", "--wait", "3");
                Assert.Equal(2, missing.Code);
                Assert.Contains("error: backup " + file + ".missing: ", missing.Err);

                var rate = Run("--sim", "gripper", "recover", "--restore-baud", "9600");
                Assert.Equal(2, rate.Code);
                Assert.Contains("--restore-baud: choose from 115200, 230400", rate.Err);
            }
            finally
            {
                File.Delete(file);
            }
        }

        [NativeFact]
        public void Monitor_watch_prints_the_first_status_and_a_summary_and_json_lines()
        {
            var (code, output, error) = Run("--sim", "slider", "monitor", "--watch", "--duration", "0.6", "--interval", "0.1");
            Assert.Equal(0, code);
            Assert.Empty(error);
            var lines = output.Split('\n', StringSplitOptions.RemoveEmptyEntries).Select(l => l.TrimEnd('\r')).ToList();
            Assert.Matches(@"^\s+\d+\.\d{3} s  position\s+73\.95 mm  velocity\s+0\.00 mm/s  SERVO HOMED IN-POS MODBUS$", lines[0]);
            Assert.Matches(@"^\s+\d+\.\d{3} s  \d+ polls \([\d.]+ /s\), 0 errors, \d+ link checks, link ok$", lines[^1]);

            var json = Run("--sim", "slider", "--json", "monitor", "--watch", "--duration", "0.6");
            Assert.Equal(0, json.Code);
            var rows = json.Out.Split('\n', StringSplitOptions.RemoveEmptyEntries).Select(l => JsonDocument.Parse(l).RootElement).ToList();
            Assert.Equal("status", rows[0].GetProperty("type").GetString());
            Assert.Equal(73.95, rows[0].GetProperty("status").GetProperty("position_mm").GetDouble());
            Assert.Equal("summary", rows[^1].GetProperty("type").GetString());
            Assert.True(rows[^1].GetProperty("polls").GetInt64() > 0);
            Assert.Equal("ok", rows[^1].GetProperty("link_state").GetString());
        }

        /// <summary>
        /// A unit that holds the first request of the link check until it is released: the check is then still running when the watch ends.
        /// The frame is chosen by its content, not by time: the check begins with a full status read (FC03 0x9000 x 22), while the monitor
        /// polls 8 words (answered at once); the watch makes no other 22-word read after the monitor's first poll.
        /// </summary>
        private sealed class HoldingLinkCheckUnit : SimulatedUnit
        {
            private readonly ManualResetEventSlim _release = new ManualResetEventSlim();
            private volatile bool _polled;  // the monitor's first poll came

            public HoldingLinkCheckUnit(SimulatedUnitOptions options)
                : base(options)
            {
            }

            /// <summary>Set when the link check's first request arrived.</summary>
            public ManualResetEventSlim Held { get; } = new ManualResetEventSlim();

            public void Release() => _release.Set();

            protected override byte[]? HandleFrame(byte[] frame)
            {
                if (frame.Length >= 6 && frame[1] == 0x03 && frame[2] == 0x90 && frame[3] == 0x00 && frame[5] == 8)
                {
                    _polled = true;
                }
                else if (_polled && frame.Length >= 6 && frame[1] == 0x03 && frame[2] == 0x90 && frame[3] == 0x00 && frame[5] == 22 && !Held.IsSet)
                {
                    Held.Set();
                    _release.Wait(TimeSpan.FromSeconds(5));
                }

                return base.HandleFrame(frame);
            }
        }

        [NativeFact]
        public void Monitor_watch_counts_a_link_check_that_is_still_running_when_it_ends()
        {
            var unit = new HoldingLinkCheckUnit(new SimulatedUnitOptions { Profile = SimProfile.Slider });
            var env = new CliEnvironment
            {
                ListPorts = NoHardware.ListPorts,
                OpenSerial = NoHardware.OpenSerial,
                CreateSim = (profile, slaves, baud) => new SimLine(new[] { unit }, "sim"),
            };
            using var cancel = new CancellationTokenSource();
            var output = new StringWriter();
            var ender = System.Threading.Tasks.Task.Run(() =>
            {
                unit.Held.Wait(TimeSpan.FromSeconds(10));
                cancel.Cancel();  // the watch ends (as with Ctrl+C) while the link check waits for the unit ...
                Thread.Sleep(300);
                unit.Release();   // ... which answers a moment later
            });
            var code = CliApp.Run(new[] { "--sim", "slider", "monitor", "--watch", "--interval", "0.1" }, output, new StringWriter(), env, cancel.Token);
            ender.Wait();
            Assert.Equal(0, code);
            var lines = output.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries).Select(l => l.TrimEnd('\r')).ToList();
            Assert.True(unit.Held.IsSet, "the link check's full status read never came");
            Assert.Matches(@"^\s+\d+\.\d{3} s  \d+ polls \([\d.]+ /s\), 0 errors, 1 link checks, link ok$", lines[^1]);  // 0 before the fix
        }

        [NativeFact]
        public void A_move_can_change_its_speed_or_acceleration_while_it_runs_and_may_be_fast()
        {
            var speed = Run("--sim", "slider", "move", "100", "--speed", "40", "--change-speed", "80", "--change-after", "0.3");
            Assert.Equal(0, speed.Code);
            Assert.Contains("changing the speed of the running move to 80 mm/s", speed.Err);
            Assert.Contains("position      100.00 mm", speed.Out);
            var accel = Run("--sim", "slider", "move", "100", "--speed", "40", "--change-accel", "0.2", "--change-after", "0.3");
            Assert.Equal(0, accel.Code);
            Assert.Contains("changing the acceleration of the running move to 0.2 G", accel.Err);
            Assert.Contains("position      100.00 mm", accel.Out);

            var refused = Run("--sim", "slider", "move", "100", "--no-wait", "--change-speed", "80");
            Assert.Equal(2, refused.Code);
            Assert.Equal("error: --change-speed / --change-accel need a plain move that waits (no --no-wait, no --decel)" + Environment.NewLine, refused.Err);
            Assert.Equal(2, Run("--sim", "slider", "move", "100", "--decel", "0.2", "--change-accel", "0.2").Code);

            var fast = Run("--sim", "slider", "move", "90", "--speed", "100", "--fast");
            Assert.Equal(0, fast.Code);
            Assert.Matches(@"position\s+(89\.9\d|90\.0\d) mm", fast.Out);  // the fast rule accepts the first sample inside the band
            Assert.Equal(0, Run("--sim", "slider", "jog", "-2", "--fast").Code);
            Assert.Equal(2, Run("--sim", "slider", "status", "--fast").Code);
        }

        [NativeFact]
        public void Teach_stores_the_position_without_an_opt_in_and_pause_commands_run()
        {
            var taught = Run("--sim", "slider", "--unit-max", "teach", "0");
            Assert.Equal(0, taught.Code);
            Assert.StartsWith("row 0 taught from the current position: 73.95 mm, speed ", taught.Out);
            Assert.EndsWith("(non-volatile, read back)" + Environment.NewLine, taught.Out);
            // still accepted
            var json = Run("--sim", "slider", "--unit-max", "--json", "teach", "0", "--allow-nonvolatile");
            Assert.Equal(0, json.Code);
            Assert.Equal(73.95, JsonDocument.Parse(json.Out).RootElement.GetProperty("position_mm").GetDouble());
            Assert.Equal(2, Run("--sim", "slider", "teach", "0", "--method", "bogus").Code);

            var pause = Run("--sim", "slider", "pause");
            Assert.Equal(0, pause.Code);
            Assert.StartsWith("paused: the move is held and keeps its remaining travel", pause.Out);
            var resume = Run("--sim", "slider", "resume");
            Assert.Equal(0, resume.Code);
            Assert.Equal("pause released: a paused move continues by itself" + Environment.NewLine, resume.Out);
            var cancel = Run("--sim", "slider", "cancel-pause");  // a new simulated unit holds no pause
            Assert.Equal(2, cancel.Code);
            Assert.Contains("EcNotReadyException", cancel.Err);
        }

        [NativeFact]
        public void The_new_commands_are_in_the_help_and_refuse_other_commands_options()
        {
            var help = Run("--help");
            Assert.Equal(0, help.Code);
            Assert.Contains("  check [--motion] [--report FILE]", help.Out);
            Assert.Contains("  backup [FILE] [--rows used|all|0,1,2]", help.Out);
            Assert.Contains("  recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]", help.Out);
            Assert.Contains("  teach N [--force] [--method table|coil]", help.Out);
            var hiddenOptions = new[]
            {
                "--allow-nonvolatile", "--allow-dangerous", "--allow-home-params", "--allow-unlisted",
                "--allow-unverified-code",
            };
            foreach (var hidden in hiddenOptions)
            {
                Assert.DoesNotContain(hidden, help.Out); // accepted and ignored, not offered
            }

            Assert.Contains("  monitor [--count N] [--interval S] [--watch [--duration S]]", help.Out);
            Assert.Equal(2, Run("--sim", "gripper", "status", "--motion").Code);
            Assert.Equal(2, Run("--sim", "gripper", "check", "--rows", "all").Code);
            Assert.Equal(2, Run("--sim", "gripper", "check", "extra").Code);
        }
    }
}
