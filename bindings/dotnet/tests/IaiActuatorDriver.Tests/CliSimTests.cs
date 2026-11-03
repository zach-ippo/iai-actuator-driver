using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using IaiActuatorDriver.Cli;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The command-line tool on the built-in simulator (<c>--sim PROFILE</c>), exactly as a customer without hardware runs it.
    /// No test opens a real serial port: the environment refuses to.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class CliSimTests
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

        [Fact]
        public void Json_output_stays_one_document_when_Modbus_control_is_enabled_first()
        {
            // a unit in PIO mode (PMSS 0) at rest: the note about enabling Modbus control goes to stderr, stdout is only the JSON
            var pio = new CliEnvironment
            {
                ListPorts = NoHardware.ListPorts,
                OpenSerial = NoHardware.OpenSerial,
                CreateSim = (profile, slaves, baud) => SimLine.WithSlaves(slaves, new SimulatedUnitOptions { Profile = SimProfile.Gripper, Pmsl = false, BaudRate = baud }, "sim"),
            };
            const string note = "enabling Modbus control (PMSL) while the axis is at rest";
            var output = new StringWriter();
            var error = new StringWriter();
            var code = CliApp.Run(new[] { "--sim", "gripper", "--json", "mode", "pc" }, output, error, pio, CancellationToken.None);
            Assert.Equal(0, code);
            Assert.DoesNotContain(note, output.ToString());
            Assert.Contains(note, error.ToString());
            Assert.True(JsonDocument.Parse(output.ToString()).RootElement.GetProperty("modbus_enabled").GetBoolean());

            // without --json the note stays on stdout, as before
            output = new StringWriter();
            error = new StringWriter();
            code = CliApp.Run(new[] { "--sim", "gripper", "mode", "pc" }, output, error, pio, CancellationToken.None);
            Assert.Equal(0, code);
            Assert.Contains(note, output.ToString());
            Assert.DoesNotContain(note, error.ToString());
        }

        [Fact]
        public void The_simulator_profiles_are_the_six_simulated_units()
        {
            Assert.Equal(new[] { "slider", "gripper", "rod", "pcon", "rotary", "unknown" }, NoHardware.SimProfiles);
            Assert.Equal(new[] { "slider", "gripper", "rod", "pcon", "rotary", "unknown" }, SimSupport.Profiles);
            Assert.Throws<ArgumentException>(() => SimSupport.CreateLine("teapot", new[] { 1 }, null));
            Assert.Throws<ArgumentException>(() => SimSupport.CreateLine("3", new[] { 1 }, null)); // a number is not a profile
        }

        [Theory]
        [InlineData("slider", "model      MWB-EC-DSRR4-WA-MF", "units      mm", 73.95)]
        [InlineData("gripper", "model      EC-GRBP8M-20-1-1-PN-WA-WL2", "units      mm", 9.5)]
        [InlineData("rod", "model      EC-R6S-150-1-PN", "units      mm", 0.0)]
        [InlineData("pcon", "family     PCON", "units      mm", 60.0)]
        [InlineData("rotary", "model      SIM-EC-ROTARY-360", "units      deg", 0.0)]
        [InlineData("unknown", "family     IAI controller", "units      mm", 0.0)]
        public void Every_profile_identifies_itself_and_reports_its_status(string profile, string model, string units, double position)
        {
            var identify = Run("--sim", profile, "identify");
            Assert.Equal(0, identify.Code);
            Assert.Contains(model, identify.Out);
            Assert.Contains(units, identify.Out);
            Assert.DoesNotContain("note:", identify.Err); // no serial note: there is no port
            var status = Run("--sim", profile, "--json", "status");
            Assert.Equal(0, status.Code);
            Assert.Equal(position, JsonDocument.Parse(status.Out).RootElement.GetProperty("position_mm").GetDouble());
            var limits = Run("--sim", profile, "limits");
            Assert.Equal(0, limits.Code);
            Assert.Contains("position       0 ..", limits.Out);
        }

        [Fact]
        public void Read_only_commands_work_on_a_CON_unit_too()
        {
            var info = Run("--sim", "pcon", "--json", "info", "--section", "identity");
            Assert.Equal(0, info.Code);
            Assert.True(JsonDocument.Parse(info.Out).RootElement.GetProperty("meta").GetProperty("read_only").GetBoolean());
            var alarms = Run("--sim", "pcon", "alarms");
            Assert.Equal(0, alarms.Code);
            var list = Run("--sim", "pcon", "param", "list");
            Assert.Equal(0, list.Code);
            Assert.Contains("PCON parameters", list.Out);
            var monitor = Run("--sim", "pcon", "monitor", "--count", "2", "--interval", "0");
            Assert.Equal(0, monitor.Code);
        }

        [Fact]
        public void An_EC_profile_lists_its_user_parameters()
        {
            var list = Run("--sim", "rod", "param", "list");
            Assert.Equal(0, list.Code);
            Assert.Contains("user parameters (", list.Out);
            Assert.Contains("P1 ", list.Out);
            Assert.Contains("HOME, DANGEROUS", list.Out);
            Assert.DoesNotContain("--allow-", list.Out); // no opt-in hints
            var home = Run("--sim", "rod", "param", "set", "P3", "1"); // home direction: written with the home warning
            Assert.Equal(0, home.Code);
            Assert.Contains("P3 / P4 are home parameters", home.Out + home.Err);
            // still accepted
            Assert.Equal(0, Run("--sim", "rod", "param", "set", "P3", "0", "--allow-home-params").Code);
        }

        [Fact]
        public void Scan_finds_every_simulated_axis_with_its_identity()
        {
            var (code, output, _) = Run("scan", "--sim", "slider", "--sim-slaves", "1,3", "--slaves", "1-4", "--scan-timeout", "0.01");
            Assert.Equal(0, code);
            Assert.Contains("sim: 2 axis/axes at no baud rate (simulated)", output);
            Assert.Contains("sim:3", output);
            var json = Run("scan", "--sim", "gripper", "--sim-slaves", "2", "--slaves", "1-4", "--scan-timeout", "0.01", "--json");
            var axes = JsonDocument.Parse(json.Out).RootElement.GetProperty("sim").GetProperty("axes");
            Assert.Equal(1, axes.GetArrayLength());
            Assert.Equal("sim:2", axes[0].GetProperty("name").GetString());
            Assert.Equal(1, Run("scan", "--sim", "slider", "--slaves", "5-6", "--scan-timeout", "0.01").Code); // nothing at 5 or 6
        }

        [Fact]
        public void The_slave_option_picks_the_simulated_address()
        {
            var (code, output, _) = Run("--sim", "slider", "--axis", "4", "identify");
            Assert.Equal(0, code);
            Assert.Contains("slave 5", output);
        }

        [Fact]
        public void A_simulated_move_goto_and_stop_run_in_real_time()
        {
            var move = Run("--sim", "slider", "move", "100", "--speed", "80");
            Assert.Equal(0, move.Code);
            Assert.Contains("position      100.00 mm", move.Out);
            var refused = Run("--sim", "slider", "move", "500");
            Assert.Equal(2, refused.Code);
            Assert.Contains("SoftLimitException", refused.Err);
            var jog = Run("--sim", "slider", "jog", "-5");
            Assert.Equal(0, jog.Code);
            Assert.Contains("position       68.95 mm", jog.Out); // a fresh simulated unit each run: 73.95 - 5
            var stop = Run("--sim", "slider", "stop");
            Assert.Equal(0, stop.Code);
            Assert.Equal(0, Run("--sim", "slider", "alarm-reset").Code);
            Assert.Equal(0, Run("--sim", "slider", "servo", "on").Code);
            var rotary = Run("--sim", "rotary", "move", "90", "--speed", "90");
            Assert.Equal(0, rotary.Code);
            Assert.Contains("position       90.00 deg", rotary.Out);
        }

        [Fact]
        public void A_stored_row_runs_with_a_speed_override()
        {
            var (code, output, _) = Run("--sim", "slider", "goto", "0", "--speed-pct", "50", "--allow-fast");
            Assert.Equal(0, code);
            Assert.Contains("mm", output);
            Assert.Equal(2, Run("--sim", "slider", "goto", "9").Code); // the slider has rows 0..2
        }

        [Fact]
        public void The_simulated_line_can_model_a_baud_rate()
        {
            var (code, output, _) = Run("--sim", "rod", "--baud", "38400", "limits");
            Assert.Equal(0, code);
            Assert.Contains("units      mm", output);
        }

        [Fact]
        public void A_unit_without_a_family_name_is_reported_without_a_label()
        {
            var identify = Run("--sim", "unknown", "identify");
            Assert.Equal(0, identify.Code);
            Assert.DoesNotContain("UNVERIFIED", identify.Out + identify.Err);
            Assert.DoesNotContain("note       unit code", identify.Out);
            Assert.Contains("AKID kind 42: commanded as a linear axis in mm", identify.Out); // what the driver does
            Assert.DoesNotContain("driver's table", identify.Out + identify.Err);
            var move = Run("--sim", "unknown", "move", "50", "--speed", "20");
            Assert.Equal(0, move.Code);
            Assert.Contains("position       50.00 mm", move.Out);

            // an EC unit code without a family name: scan shows the product family, identify adds no note
            var scan = new StringWriter();
            var hit = new ScanHit("SIM", 1) { SystemCode = 0xEC, UnitCode = 0x7E, Firmware = "0002" };
            CliOutput.PrintScan("SIM", null, new[] { hit }, null, scan);
            Assert.Contains("EC 24 V, unit 0x7E, firmware 0002: EC" + Environment.NewLine, scan.ToString());
            Assert.DoesNotContain("UNVERIFIED", scan.ToString());
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Rod, timeScale: 50.0) with { UnitCode = 0x7E });
            var (axis, _) = SimKit.Open(sim, new AxisOptions { PollIntervalS = 0.005 });
            using (axis)
            {
                var info = new StringWriter();
                CliOutput.PrintInfo(axis.Identify(), axis, info);
                Assert.Contains("controller EC 24 V, unit 0x7E", info.ToString());
                Assert.DoesNotContain("note       ", info.ToString());
                Assert.DoesNotContain("UNVERIFIED", info.ToString());
            }
        }

        [Fact]
        public void An_unknown_profile_is_a_usage_error()
        {
            var (code, _, err) = Run("--sim", "teapot", "status");
            Assert.Equal(2, code);
            Assert.Contains("--sim: choose a profile", err);
        }
    }
}
