using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Json;
using System.Threading;
using IaiActuatorDriver.Cli;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The command-line tool against test units. Every port is a <see cref="FakeTransport"/> handed out by the test
    /// environment: no test opens a real serial port.
    /// </summary>
    public class CliTests
    {
        private static readonly SerialPortInfo[] Adapters =
        {
            new SerialPortInfo("COM1", "Communications Port (COM1)"),
            new SerialPortInfo("COM3", "IAI USB to UART Bridge Controller (COM3)", 0x10C4, 0x81D7, "IAI Corporation"),
        };

        private sealed class Setup
        {
            public Dictionary<string, UnitLine> Ports { get; } = new Dictionary<string, UnitLine>();

            public List<(string Profile, IReadOnlyList<int> Slaves)> SimCalls { get; } = new List<(string, IReadOnlyList<int>)>();

            public IReadOnlyList<SerialPortInfo> Listed { get; set; } = Adapters;

            public UnitLine? Sim { get; set; }

            public CliEnvironment Env => new CliEnvironment
            {
                ListPorts = () => PortManager.Sort(Listed),
                OpenSerial = (name, baud, timeout) => Ports.TryGetValue(name, out var line) ? Reopened(line) : throw new IOException($"{name}: no such port (test)"),
                CreateSim = (profile, slaves, baud) =>
                {
                    SimCalls.Add((profile, slaves));
                    return Sim is null ? throw new NotSupportedException("no simulator in this test") : Reopened(Sim);
                },
            };

            // Every command line is a new program: it opens the port again (the previous one closed it).
            private static ITransport Reopened(UnitLine line)
            {
                line.Transport.Reopen();
                return line.Transport;
            }

            public (int Code, string Out, string Err) Run(params string[] args) => RunWith(CancellationToken.None, args);

            public (int Code, string Out, string Err) RunWith(CancellationToken ct, params string[] args)
            {
                var output = new StringWriter();
                var error = new StringWriter();
                var code = CliApp.Run(args, output, error, Env, ct);
                return (code, output.ToString(), error.ToString());
            }
        }

        private static Setup Com3(double position = 0.0, int slave = 1)
        {
            var setup = new Setup();
            setup.Ports["COM3"] = new UnitLine("COM3", new[] { new SimUnit("rod", slave).Ready(position) });
            return setup;
        }

        [Fact]
        public void Help_version_and_usage_errors()
        {
            var s = new Setup();
            var help = s.Run("--help");
            Assert.Equal(0, help.Code);
            Assert.Contains("usage: iai-actuator-driver-cs", help.Out);
            Assert.Contains("goto N [--speed-pct P]", help.Out);
            Assert.Equal(0, s.Run("--version").Code);
            Assert.Equal(2, s.Run().Code);
            Assert.Equal(2, s.Run("--port", "COM3", "--bogus", "status").Code);
            Assert.Equal(2, s.Run("--port", "COM3", "dance").Code);
            Assert.Equal(2, s.Run("status").Code); // neither --port nor --sim
            Assert.Equal(2, s.Run("--port", "COM3", "--sim", "rod", "status").Code);
            Assert.Equal(2, s.Run("--port", "COM3", "--slave", "2", "--axis", "1", "status").Code);
            Assert.Equal(2, s.Run("--port", "COM3", "move").Code);
            Assert.Equal(2, s.Run("--port", "COM3", "status", "--speed", "5").Code); // status takes no --speed
            Assert.Equal(2, s.Run("--port", "COM3", "servo", "maybe").Code);
            Assert.Equal(2, s.Run("--port", "COM3", "--slave", "300", "status").Code);
            var bad = s.Run("--port", "COM3", "move", "abc");
            Assert.Equal(2, bad.Code);
            Assert.Contains("not a number", bad.Err);
        }

        [Fact]
        public void Version_prints_the_full_informational_version()
        {
            Assert.Equal("0.12.0-beta.1", CliApp.VersionText("0.12.0-beta.1+6b26852", new Version(0, 12, 0, 0)));
            Assert.Equal("0.12.0-beta.1", CliApp.VersionText("0.12.0-beta.1", new Version(0, 12, 0, 0)));
            Assert.Equal("1.0.0", CliApp.VersionText("1.0.0", new Version(1, 0, 0, 0)));
            Assert.Equal("1.0.0", CliApp.VersionText(null, new Version(1, 0, 0, 0)));
            Assert.Equal("0.0.0", CliApp.VersionText(null, null));

            var informational = typeof(CliApp).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>();
            Assert.NotNull(informational);
            var expected = informational!.InformationalVersion.Split('+')[0];
            var library = typeof(Axis).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>();
            Assert.Equal(library!.InformationalVersion.Split('+')[0], expected);  // the tool's and the library's
            var (code, output, _) = new Setup().Run("--version");
            Assert.Equal(0, code);
            Assert.Equal("iai-actuator-driver-cs " + expected, output.TrimEnd('\r', '\n'));
        }

        [Fact]
        public void Ports_lists_IAI_adapters_first()
        {
            var s = new Setup();
            var (code, output, _) = s.Run("ports");
            Assert.Equal(0, code);
            var lines = output.Split('\n').Select(l => l.TrimEnd('\r')).ToList();
            Assert.StartsWith("PORT", lines[0]);
            Assert.StartsWith("COM3", lines[1]);
            Assert.Contains("10C4:81D7", lines[1]);
            var json = s.Run("ports", "--json");
            Assert.Equal(0, json.Code);
            var doc = JsonDocument.Parse(json.Out);
            Assert.True(doc.RootElement[0].GetProperty("is_iai").GetBoolean());
            Assert.Equal("COM3", doc.RootElement[0].GetProperty("port").GetString());
        }

        [Fact]
        public void Scan_lists_the_axes_of_each_port()
        {
            var s = new Setup();
            s.Ports["COM3"] = UnitLine.Rods("COM3", 1, 3);
            var (code, output, _) = s.Run("scan", "--ports", "COM3,COM9", "--slaves", "1-4", "--scan-timeout", "0.005");
            Assert.Equal(0, code);
            Assert.Contains("COM3: 2 axis/axes", output);
            Assert.Contains("COM3:3", output);
            Assert.Contains("axis 2", output);
            Assert.Contains("COM9: error: IOException", output);

            var json = s.Run("scan", "--ports", "COM3", "--slaves", "1-4", "--scan-timeout", "0.005", "--json");
            var axes = JsonDocument.Parse(json.Out).RootElement.GetProperty("COM3").GetProperty("axes");
            Assert.Equal(2, axes.GetArrayLength());
            Assert.Equal("COM3:1", axes[0].GetProperty("name").GetString());
        }

        [Fact]
        public void Scan_exit_codes()
        {
            var s = new Setup();
            s.Ports["COM3"] = UnitLine.Rods("COM3", 5);
            var none = s.Run("scan", "--slaves", "1-2", "--scan-timeout", "0.005"); // the IAI adapter COM3 by default
            Assert.Equal(1, none.Code);
            Assert.Contains("COM3: no axis answered", none.Out);
            s.Listed = Adapters.Take(1).ToList();
            var noAdapter = s.Run("scan");
            Assert.Equal(2, noAdapter.Code);
            Assert.Contains("no IAI USB adapter", noAdapter.Err);
            Assert.Equal(2, s.Run("scan", "--ports", "COM8").Code); // every port failed
            Assert.Equal(2, s.Run("scan", "--slaves", "0-3").Code);
        }

        [Fact]
        public void Scan_on_the_simulator()
        {
            var s = new Setup { Sim = UnitLine.Rods("sim", 1, 3) };
            var (code, output, _) = s.Run("scan", "--sim", "rod", "--sim-slaves", "1,3", "--slaves", "1-4", "--scan-timeout", "0.005");
            Assert.Equal(0, code);
            Assert.Contains("sim: 2 axis/axes at no baud rate (simulated)", output);
            Assert.Equal("rod", s.SimCalls[0].Profile);
            Assert.Equal(new[] { 1, 3 }, s.SimCalls[0].Slaves);
            Assert.Equal(2, s.Run("scan", "--sim", "teapot").Code);
        }

        [Fact]
        public void Status_text_and_json()
        {
            var s = Com3(position: 12.34);
            var (code, output, err) = s.Run("--port", "COM3", "status");
            Assert.Equal(0, code);
            Assert.Contains("position       12.34 mm", output);
            Assert.Contains("SERVO HOMED", output);
            Assert.Contains("note: COM3 at", err);
            var json = s.Run("--port", "COM3", "--json", "status");
            Assert.Equal(12.34, JsonDocument.Parse(json.Out).RootElement.GetProperty("position_mm").GetDouble());
            Assert.DoesNotContain(s.Ports["COM3"].Transport.Writes, f => f[1] == 0x05 || f[1] == 0x06 || f[1] == 0x10); // read-only
        }

        [Fact]
        public void The_axis_option_picks_the_slave()
        {
            var s = Com3(slave: 3);
            var (code, output, _) = s.Run("--port", "COM3", "--axis", "2", "identify");
            Assert.Equal(0, code);
            Assert.Contains("model      EC-R6S-150-1-PN", output);
            Assert.Contains("slave 3", output);
        }

        [Fact]
        public void The_simulator_option_opens_a_simulated_line()
        {
            var s = new Setup { Sim = UnitLine.Rods("sim", 2) };
            var (code, output, err) = s.Run("--sim", "rod", "--slave", "2", "limits");
            Assert.Equal(0, code);
            Assert.Contains("units      mm (kind rod", output);
            Assert.Contains("speed", output);
            Assert.DoesNotContain("note:", err);
            Assert.Equal(new[] { 2 }, s.SimCalls.Single().Slaves);
        }

        [Fact]
        public void Move_checks_the_range_before_sending_anything()
        {
            var s = Com3();
            var refused = s.Run("--port", "COM3", "move", "500");
            Assert.Equal(2, refused.Code);
            Assert.Contains("SoftLimitException", refused.Err);
            Assert.DoesNotContain(s.Ports["COM3"].Transport.Writes, f => f[1] == 0x10);

            var fast = s.Run("--port", "COM3", "move", "50", "--speed", "200");
            Assert.Equal(2, fast.Code); // above the tool's default 100 units/s cap
            Assert.Equal(0, s.Run("--port", "COM3", "--unit-max", "move", "50", "--speed", "200").Code);

            var (code, output, _) = s.Run("--port", "COM3", "move", "80", "--speed", "40");
            Assert.Equal(0, code);
            Assert.Contains("position       80.00 mm", output);
        }

        [Fact]
        public void Jog_takes_a_negative_step()
        {
            var s = Com3(position: 50.0);
            var (code, output, _) = s.Run("--port", "COM3", "jog", "-5");
            Assert.Equal(0, code);
            Assert.Contains("position       45.00 mm", output);
        }

        [Fact]
        public void Goto_with_a_speed_override_sends_0x45_cmd_6()
        {
            var s = Com3(position: 50.0);
            var capped = s.Run("--port", "COM3", "goto", "0");
            Assert.Equal(2, capped.Code); // stored 240 mm/s is above the tool's default cap
            var (code, output, _) = s.Run("--port", "COM3", "goto", "0", "--speed-pct", "25", "--allow-fast");
            Assert.Equal(0, code);
            Assert.Contains("position        0.00 mm", output);
            Assert.Equal("45 00 06 00 00 00 3F", Frames.Pdu(Frames.Iai(s.Ports["COM3"].Transport).Single()));
        }

        [Fact]
        public void Goto_notes_a_rounded_speed_override()
        {
            var s = Com3(position: 50.0);
            var (code, _, error) = s.Run("--port", "COM3", "goto", "0", "--speed-pct", "0.1", "--allow-fast");
            Assert.Equal(0, code);
            Assert.Contains("note: --speed-pct 0.1: the row runs at 0.78 % of its stored speed (override code 1;", error);
            Assert.Equal("45 00 06 00 00 00 01", Frames.Pdu(Frames.Iai(s.Ports["COM3"].Transport).Single()));
        }

        [Fact]
        public void Push_reports_the_outcome()
        {
            var s = Com3(position: 0.0);
            s.Ports["COM3"][1].ContactAt = 85.0;
            var (code, output, _) = s.Run("--port", "COM3", "push", "80", "10", "30");
            Assert.Equal(0, code);
            Assert.Contains("push contact at 85.00 mm", output);
        }

        [Fact]
        public void Stop_releases_the_STOP_coil()
        {
            var s = Com3();
            Assert.Equal(0, s.Run("--port", "COM3", "stop").Code);
            var stops = s.Ports["COM3"].Coils().Where(c => c.Address == Registers.COIL_STOP).Select(c => c.On).ToList();
            Assert.Equal(new[] { true, false }, stops);
        }

        [Fact]
        public void A_cancelled_move_is_stopped_and_exits_130()
        {
            var s = Com3();
            s.Ports["COM3"][1].StepMm = 0.1;
            using var cts = new CancellationTokenSource();
            cts.Cancel();
            var (code, _, err) = s.RunWith(cts.Token, "--port", "COM3", "move", "100");
            Assert.Equal(130, code);
            Assert.Contains("interrupted", err);
            var stops = s.Ports["COM3"].Coils().Where(c => c.Address == Registers.COIL_STOP).Select(c => c.On).ToList();
            Assert.Equal(new[] { true, false }, stops);
        }

        [Fact]
        public void Reset_is_the_deprecated_alias_of_alarm_reset()
        {
            var s = Com3();
            var (code, _, err) = s.Run("--port", "COM3", "reset");
            Assert.Equal(0, code);
            Assert.Contains("deprecated", err);
            Assert.Empty(Frames.Iai(s.Ports["COM3"].Transport)); // never a software reset
            Assert.Contains((1, Registers.COIL_ALRS, true), s.Ports["COM3"].Coils());
        }

        [Fact]
        public void Read_only_commands_info_alarms_monitor()
        {
            var s = Com3(position: 7.0);
            var info = s.Run("--port", "COM3", "--json", "info", "--section", "identity", "--section", "alarms");
            Assert.Equal(0, info.Code);
            var root = JsonDocument.Parse(info.Out).RootElement;
            Assert.True(root.GetProperty("meta").GetProperty("read_only").GetBoolean());
            Assert.Equal(JsonValueKind.Null, root.GetProperty("parameters").ValueKind);
            var text = s.Run("--port", "COM3", "info");
            Assert.Equal(0, text.Code);
            Assert.Contains("[identity]", text.Out);
            var alarms = s.Run("--port", "COM3", "alarms");
            Assert.Equal(0, alarms.Code);
            Assert.Contains("code 0x00c1", alarms.Out);
            var monitor = s.Run("--port", "COM3", "monitor", "--count", "2", "--interval", "0");
            Assert.Equal(0, monitor.Code);
            Assert.Equal(2, monitor.Out.Split('\n').Count(l => l.StartsWith("position", StringComparison.Ordinal)));
            Assert.All(s.Ports["COM3"].Transport.Writes, f => Assert.Contains(f[1], new byte[] { 0x03, 0x11 }));
        }

        [Fact]
        public void Param_list_and_a_home_parameter_set_without_an_opt_in()
        {
            var s = Com3();
            var list = s.Run("--port", "COM3", "param", "list");
            Assert.Equal(0, list.Code);
            Assert.Contains("user parameters (", list.Out);
            var get = s.Run("--port", "COM3", "param", "get", "P2");
            Assert.Equal(0, get.Code);
            Assert.Equal(2, JsonDocument.Parse(get.Out).RootElement.GetProperty("number").GetInt32());
            var home = s.Run("--port", "COM3", "param", "set", "P3", "1");
            Assert.Equal(0, home.Code); // the home direction is written with the home warning
            Assert.Contains("P3 / P4 are home parameters", home.Out + home.Err);
            Assert.Equal(2, s.Run("--port", "COM3", "param", "list", "--no-commit").Code);
            Assert.Equal(2, s.Run("--port", "COM3", "param", "delete", "P3").Code);
        }

        [Fact]
        public void A_port_that_cannot_be_opened_is_reported()
        {
            var s = new Setup();
            var (code, _, err) = s.Run("--port", "COM42", "status");
            Assert.Equal(2, code);
            Assert.Contains("error: IOException", err);
        }
    }
}
