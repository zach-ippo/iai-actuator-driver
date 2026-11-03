using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The fixes of the round-5 review (the "polish" commit) in the C# binding: a transport exception that is not the port's ends a
    /// link check and a recovery with itself, JSON numbers keep their float / integer token, a stopped monitor reads as empty, an
    /// invalid line kind is refused before a port is opened, and a call gives its own result while another thread checks the link.
    /// No port opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class PolishReviewTests
    {
        // ------------------------------------------------------------------ a transport's own exception (IAD_CALLBACK_ABORT)

        [NativeFact]
        public void A_transport_exception_that_is_not_the_ports_ends_a_link_check_and_a_recovery_with_itself()
        {
            var transport = new ThrowingTransport(new SimulatedUnit(SimKit.Sim()));
            using var port = new Port(transport, "P", minGap: TimeSpan.Zero);
            var axis = port.Axis(1);
            axis.Connect();
            axis.Track("A");
            Assert.Equal("ok", axis.CheckLink().State);

            var odd = new NotImplementedException("no frames today");
            transport.Failure = odd;
            Assert.Same(odd, Assert.Throws<NotImplementedException>(() => axis.Status()));  // as before
            Assert.Same(odd, Assert.Throws<NotImplementedException>(() => axis.CheckLink()));  // not a "port_vanished" link state
            Assert.NotEqual("port_vanished", axis.LinkState);
            var clock = System.Diagnostics.Stopwatch.StartNew();
            Assert.Same(odd, Assert.Throws<NotImplementedException>(() => axis.Recover(timeout: 5.0)));  // not RecoveryException after its timeout
            Assert.True(clock.Elapsed < TimeSpan.FromSeconds(3.0), $"the recovery went on for {clock.Elapsed.TotalSeconds:0.0} s");

            // the port's own failure stays a link fault
            transport.Failure = new IOException("cable pulled");
            var lost = axis.CheckLink();
            Assert.Equal("port_vanished", lost.State);
        }

        [NativeFact]
        public void A_cell_group_command_keeps_a_transport_exception_that_is_not_the_ports()
        {
            var transport = new ThrowingTransport(new SimulatedUnit(SimKit.Sim()));
            using var cell = Cell.Open(new Dictionary<string, ITransport> { ["P"] = transport }, new[] { 1 }, new CellOptions { Identify = false });
            var odd = new NotImplementedException("no frames today");
            transport.Failure = odd;
            Assert.Same(odd, cell.StatusAll(returnExceptions: true).Errors["P:1"]);
            Assert.Same(odd, Assert.Throws<CellException>(() => cell.StatusAll()).Errors["P:1"]);
            transport.Failure = null;
            Assert.True(cell.StatusAll().Succeeded);
        }

        // ------------------------------------------------------------------ JSON numbers keep their token

        [Fact]
        public void A_number_written_as_a_float_stays_a_float_and_an_integer_a_long()
        {
            var plain = (Dictionary<string, object?>)JsonPlain.Plain(MiniJson.Parse("{\"x\":3.0,\"n\":3,\"e\":1e2,\"big\":9007199254740993,\"neg\":-0.0,\"l\":[1,2.0,{\"k\":4.0,\"j\":4}]}"))!;
            Assert.Equal(3.0, Assert.IsType<double>(plain["x"]));
            Assert.Equal(3L, Assert.IsType<long>(plain["n"]));
            Assert.Equal(100.0, Assert.IsType<double>(plain["e"]));
            Assert.IsType<double>(plain["big"]);  // beyond 9e15: not a count
            Assert.IsType<double>(plain["neg"]);
            var list = (List<object?>)plain["l"]!;
            Assert.IsType<long>(list[0]);
            Assert.IsType<double>(list[1]);
            var inner = (Dictionary<string, object?>)list[2]!;
            Assert.IsType<double>(inner["k"]);
            Assert.IsType<long>(inner["j"]);

            // text: a whole float keeps its decimal point, so the next reader keeps it a float
            Assert.Equal("{\"x\":3.0,\"n\":3,\"y\":0.25,\"z\":1E+20}", JsonPlain.Write(new Dictionary<string, object?> { ["x"] = 3.0, ["n"] = 3L, ["y"] = 0.25, ["z"] = 1e20 }, indented: false));
            var back = JsonPlain.DictOf(JsonPlain.Write(plain));
            Assert.IsType<double>(back["x"]);
            Assert.IsType<long>(back["n"]);

            // what only the document knew: the array of the report layer
            var dict = JsonPlain.DictOf("{\"position_mm\":100.0,\"speed_mm_s\":[100.0,50.5],\"rows\":[1,2]}");
            Assert.Equal(100.0, (double)dict["position_mm"]!);  // a boxed long here threw InvalidCastException
            Assert.All((List<object?>)dict["speed_mm_s"]!, v => Assert.IsType<double>(v));
            Assert.All((List<object?>)dict["rows"]!, v => Assert.IsType<long>(v));
        }

        [NativeFact]
        public void A_report_of_an_axis_parked_at_a_whole_position_gives_a_float()
        {
            var sim = new SimulatedUnit(SimKit.Sim());
            var (axis, _) = SimKit.Open(sim);
            using (axis)
            {
                axis.MoveTo(100.0, speed: 100.0);
                axis.Track("A");
                var report = axis.Recover(10.0, new RecoveryPolicy { PollS = 0.02 });
                var position = Assert.IsType<double>(report.ToDictionary()["position_mm"]);  // a long before: (double)d["position_mm"] threw
                Assert.InRange(position, 99.9, 100.1);
                Assert.Matches(@"""position_mm"": -?[0-9]+\.[0-9]+", report.ToJson());  // a float keeps its decimal point (100.0)
                var check = axis.CheckLink();
                Assert.True(check.Ok);
            }
        }

        // ------------------------------------------------------------------ a stopped monitor

        [NativeFact]
        public void A_monitor_that_was_stopped_reads_as_empty()
        {
            var sim = new SimulatedUnit(SimKit.Sim());
            var (axis, _) = SimKit.Open(sim);
            using (axis)
            {
                var monitor = axis.StartMonitor();
                Assert.NotNull(monitor.WaitUpdate(timeout: 5.0));
                Assert.NotNull(monitor.Latest);
                axis.StopMonitor();
                Assert.Null(monitor.Latest);  // "null before the first poll": an EcNotReadyException before
                Assert.Null(monitor.Status);
                Assert.Null(monitor.Fresh());
                Assert.Null(monitor.WaitUpdate(timeout: 0.2));
                Assert.Null(monitor.WaitFor(_ => true, timeout: 1.0));
                Assert.Null(monitor.WaitDone(0.2));
                Assert.Null(monitor.WaitDone(0.2, new CancellationTokenSource().Token));
                Assert.False(monitor.Running);
            }
        }

        [NativeFact]
        public void A_wait_ends_with_null_when_another_thread_stops_the_monitor()
        {
            var sim = new SimulatedUnit(SimKit.Sim());
            var (axis, _) = SimKit.Open(sim);
            using (axis)
            {
                var monitor = axis.StartMonitor();
                Assert.NotNull(monitor.WaitUpdate(timeout: 5.0));
                var waiter = Task.Run(() => monitor.WaitFor(_ => false, timeout: null));
                Thread.Sleep(150);
                axis.StopMonitor();
                Assert.True(waiter.Wait(TimeSpan.FromSeconds(10)));
                Assert.Null(waiter.Result);
            }
        }

        // ------------------------------------------------------------------ the line kind

        [Fact]
        public void An_invalid_line_kind_is_refused_before_a_port_is_opened()
        {
            var opened = 0;
            var options = new CellOptions
            {
                Line = (LineKind)9,
                SerialOpener = (name, rate, timeout) =>
                {
                    Interlocked.Increment(ref opened);
                    return new SimulatedUnit(SimKit.Sim());
                },
            };
            Assert.Throws<ArgumentOutOfRangeException>(() => Cell.Open(new[] { "COM98" }, options: options));  // not "no axis found"
            Assert.Equal(0, opened);
            Assert.Throws<ArgumentOutOfRangeException>(() => PortManager.Scan("COM98", line: (LineKind)9));  // not "port in use" or "not found"
        }

        // ------------------------------------------------------------------ a call's own result

        [NativeFact]
        public void Calls_give_their_own_result_while_another_thread_checks_the_link()
        {
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Gripper));
            var (axis, _) = SimKit.Open(sim);
            using (axis)
            {
                axis.Connect();
                axis.Track("G");
                using var stop = new CancellationTokenSource();
                var watchdog = Task.Run(() =>
                {
                    while (!stop.IsCancellationRequested)
                    {
                        axis.CheckLink();
                    }
                });
                try
                {
                    for (var i = 0; i < 150; i++)
                    {
                        var report = axis.Recover(5.0, new RecoveryPolicy { PollS = 0.02 });
                        Assert.True(report.SameUnit, $"recovery {i}: {report.ToJson()}");
                        Assert.True(report.ToDictionary().ContainsKey("actions"), $"recovery {i}: {report.ToJson()}");
                        var summary = axis.EnableFast(null, monitor: false);
                        Assert.Equal(CompletionRule.Fast, summary.Completion);
                        axis.DisableFast();
                    }
                }
                finally
                {
                    stop.Cancel();
                    watchdog.Wait(TimeSpan.FromSeconds(10));
                }
            }
        }

        // ------------------------------------------------------------------ helpers

        /// <summary>A transport that throws what <see cref="Failure"/> holds from every write.</summary>
        private sealed class ThrowingTransport : ITransport
        {
            private readonly SimulatedUnit _unit;

            public ThrowingTransport(SimulatedUnit unit) => _unit = unit;

            public Exception? Failure { get; set; }

            public int BaudRate
            {
                get => _unit.BaudRate;
                set => _unit.BaudRate = value;
            }

            public TimeSpan ReadTimeout { get; set; } = TimeSpan.FromSeconds(0.2);

            public void Write(byte[] buffer, int offset, int count)
            {
                if (Failure is { } failure)
                {
                    throw failure;
                }

                _unit.Write(buffer, offset, count);
            }

            public int Read(byte[] buffer, int offset, int count) => _unit.Read(buffer, offset, count);

            public void ResetInputBuffer() => _unit.ResetInputBuffer();

            public void SendBreak(TimeSpan duration) => _unit.SendBreak(duration);

            public void Close() => _unit.Close();

            public void Dispose() => Close();
        }
    }
}
