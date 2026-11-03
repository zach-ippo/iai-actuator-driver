using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// Stage 2 of the C# switch: <see cref="ModbusRtu"/> is a facade over the native core's bus (the frame-level
    /// behaviour is ModbusRtuTests'); these check the seams: the core's log lines, the lifetime of a bridged transport,
    /// the managed requests the ABI cannot express yet, the serial port and the port list. No port opened.
    /// </summary>
    public class BusFacadeTests
    {
        [NativeFact]
        public void The_core_runs_the_bus_and_its_log_lines_reach_DriverLog()
        {
            using var capture = new LogCapture(DriverLogLevel.Debug);
            var t = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 38400 };
            t.Enqueue(T.ReadResponse(1, 0x1CE3));
            var bus = new ModbusRtu(t, 2, TimeSpan.Zero);
            Assert.Equal(38400, bus.DetectBaudRate(1));
            Assert.Contains("slave 1 answers at 38400 baud", capture.Messages);  // written by the core (iai_actuator_driver.modbus)
        }

        [NativeFact]
        public void A_bus_the_collector_finalises_never_closes_the_transport()
        {
            var t = new FakeTransport();
            Abandon(t);
            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();
            Assert.False(t.Closed);
            t.Enqueue(T.ReadResponse(1, 7));
            using var bus = new ModbusRtu(t, minGap: TimeSpan.Zero);
            Assert.Equal(new ushort[] { 7 }, bus.ReadRegisters(1, 0x9002, 1));
            bus.Close();
            Assert.True(t.Closed);  // an explicit close does
        }

        [NativeFact]
        public void Managed_requests_and_core_requests_keep_the_gap_between_them()
        {
            var t = new FakeTransport();
            t.Enqueue(T.Response(1, "11 05 EC FF 00 00 20"), T.ReadResponse(1, 0), T.Response(1, "11 05 EC FF 00 00 20"));
            var bus = new ModbusRtu(t, minGap: TimeSpan.FromMilliseconds(20));
            bus.ReportSlaveId(1, timeout: TimeSpan.FromSeconds(1));  // FC11 with another wait: the managed path
            bus.ReadRegisters(1, 0x9002, 1);                          // the core
            bus.ReportSlaveId(1, timeout: TimeSpan.FromSeconds(1));
            var times = t.WriteTimes;
            Assert.Equal(3, times.Count);
            for (var i = 1; i < times.Count; i++)
            {
                Assert.True(times[i] - times[i - 1] >= 0.0195, $"gap {times[i] - times[i - 1]:F4} s");
            }
        }

        [NativeFact]
        public void A_serial_port_that_does_not_exist_is_an_IOException()
        {
            // a device name no PC has: nothing is opened
            var e = Assert.Throws<IOException>(() => new SerialTransport("IAI-TEST-NO-SUCH-PORT"));
            Assert.Contains("IAI-TEST-NO-SUCH-PORT", e.Message);
            Assert.IsAssignableFrom<ModbusException>(e.InnerException);
        }

        [NativeFact]
        public void The_port_list_is_the_cores()
        {
            var json = NativeDriver.ListPortsJson();
            var listed = PortManager.ListPorts();
            foreach (var port in listed)
            {
                Assert.Contains("\"" + port.Port + "\"", json);
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void Abandon(FakeTransport t)
        {
            var bus = new ModbusRtu(t);
            Assert.False(bus.IsClosed);
        }
    }

    /// <summary>Loading the native library: the override variable and the load errors (no managed fallback).</summary>
    public class NativeLoaderTests
    {
        [Fact]
        public void The_load_error_names_the_runtime_the_places_searched_and_the_abi()
        {
            var text = NativeLoader.NotFoundMessage();
            Assert.Contains(NativeLoader.Rid, text);
            Assert.Contains(NativeLoader.FileName, text);
            Assert.Contains($"ABI {NativeDriver.AbiMajor}.{NativeDriver.AbiMinor}", text);
            Assert.Contains(NativeDriver.OverrideVariable, text);
            Assert.Contains(AppContext.BaseDirectory, text);
            Assert.Contains(Path.Combine("runtimes", NativeLoader.Rid, "native"), text);
            Assert.Equal("IAI_ACTUATOR_DRIVER_NATIVE", NativeDriver.OverrideVariable);
        }

        [Fact]
        public void A_library_of_another_abi_is_refused_with_both_versions()
        {
            var text = NativeDriver.AbiMismatchMessage(0, 3);
            Assert.Contains("ABI 0.3", text);
            Assert.Contains($"ABI {NativeDriver.AbiMajor}.{NativeDriver.AbiMinor}", text);
        }

        [Fact]
        public void An_override_that_names_a_missing_file_is_a_DllNotFoundException()
        {
            var path = Path.Combine(Path.GetTempPath(), "iad-no-such-dir", NativeLoader.FileName);
            var e = Assert.Throws<DllNotFoundException>(() => NativeLoader.LoadOverride(path));
            Assert.Contains(path, e.Message);
            Assert.Contains(NativeDriver.OverrideVariable, e.Message);
        }

        [NativeFact]
        public void An_override_loads_the_file_it_names()
        {
            Assert.NotEqual(IntPtr.Zero, NativeLoader.LoadOverride(NativeLib.Path));
        }

        [NativeFact]
        public void A_program_started_with_the_override_uses_that_file_or_fails_naming_it()
        {
            var cli = Path.Combine(AppContext.BaseDirectory, "iai-actuator-driver-cs.dll");
            Assert.True(File.Exists(cli), cli);

            var copy = Path.Combine(Path.GetTempPath(), "iad-override-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(copy);
            try
            {
                var library = Path.Combine(copy, NativeLoader.FileName);
                File.Copy(NativeLib.Path, library);
                var ok = RunCli(cli, library, "--sim", "slider", "--json", "status");
                Assert.True(ok.Code == 0, ok.Err);
                Assert.Contains("position_mm", ok.Out);

                var missing = Path.Combine(copy, "missing", NativeLoader.FileName);
                var bad = RunCli(cli, missing, "--sim", "slider", "status");
                Assert.NotEqual(0, bad.Code);
                Assert.Contains(NativeDriver.OverrideVariable, bad.Out + bad.Err);
            }
            finally
            {
                try
                {
                    Directory.Delete(copy, recursive: true);
                }
                catch (IOException)
                {
                    // the child may still hold the copy for a moment; the temp folder is cleaned up later
                }
                catch (UnauthorizedAccessException)
                {
                }
            }
        }

        private static (int Code, string Out, string Err) RunCli(string cli, string library, params string[] args)
        {
            var host = Environment.GetEnvironmentVariable("DOTNET_HOST_PATH");
            if (string.IsNullOrEmpty(host) || !File.Exists(host))
            {
                host = "dotnet";
            }

            var start = new ProcessStartInfo(host)
            {
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true,
            };
            start.ArgumentList.Add(cli);
            foreach (var a in args)
            {
                start.ArgumentList.Add(a);
            }

            start.Environment[NativeDriver.OverrideVariable] = library;
            using var process = Process.Start(start)!;
            var output = process.StandardOutput.ReadToEndAsync();
            var error = process.StandardError.ReadToEndAsync();
            if (!process.WaitForExit(60000))
            {
                process.Kill();
                throw new TimeoutException("the CLI did not finish");
            }

            return (process.ExitCode, output.Result, error.Result);
        }
    }
}
