// MultiAxisCell - several axes as one cell through the C# library (IaiActuatorDriver.Cell): open, read, servo, home,
// move together, stop during a move, and the per-axis results of a group command.
//
// Shows Cell.Open over several ports (one axis per COM port is the normal setup, VERIFIED; several axes on one
// port, by address, are UNVERIFIED on hardware), StatusAll, ServoAll, HomeAll, MoveAll (started together, with
// wait: false and WaitAll), StopAll during a move, and CellException: a group command handles every axis, then reports
// the axes that failed. Every position comes from the unit's own range, so the same program runs on any mix of units.
//
// Run it from the repository folder (close IA-OS first):
//
//     dotnet run --project examples/csharp/MultiAxisCell -- --sim                          slider, rod, gripper
//     dotnet run --project examples/csharp/MultiAxisCell -- --sim --sim-profiles slider,pcon
//     dotnet run --project examples/csharp/MultiAxisCell -- --port COM3 --port COM4        each port scanned
//     dotnet run --project examples/csharp/MultiAxisCell -- --port COM3:1 --port COM3:2    exactly these addresses
//
// Options: --speed UNITS_S (one speed for every axis; default 20, raised to the highest documented minimum speed the
// driver reports for the units), --step UNITS (default 10, at most a quarter of each stroke), --home (home every axis, not only those
// that report no home).
//
// Motion: around the middle of each unit's stroke, --step apart: values for a first run on unloaded units. Homing
// runs each axis to its mechanical end: keep the travel clear. At the end every servo is switched off. On Ctrl+C or an
// error every axis gets a decel STOP and the servos are left as they are (a vertical axis without a brake can drop
// when its servo goes off). Exit codes: 0 done, 2 error, 130 interrupted.
//
// Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver;
using IaiActuatorDriver.Simulation;

namespace IaiActuatorDriver.Examples
{
    internal static class Program
    {
        private const string Banner = "iai-actuator-driver example: multi-axis cell, C# (UNOFFICIAL - not an IAI product)";
        private const double DefaultSpeed = 20.0; // units/s for a first run
        private const double DefaultStep = 10.0;  // units between the low and the high position

        private sealed class Args
        {
            public List<string> Ports = new List<string>(); // --port, repeated; empty = --sim
            public Dictionary<string, List<int>> Slaves = new Dictionary<string, List<int>>(); // --port PORT:SLAVE
            public SimProfile[] SimProfiles = { SimProfile.Slider, SimProfile.Rod, SimProfile.Gripper };
            public double? Speed;
            public double Step = DefaultStep;
            public bool Home;
        }

        /// <summary>Where one axis works: low, middle and high, around the middle of its own range.</summary>
        private sealed class Plan
        {
            public double Low;
            public double Middle;
            public double High;
            public string Units = "mm";
        }

        private static int Main(string[] argv)
        {
            if (Array.Exists(argv, a => a == "-h" || a == "--help"))
            {
                Usage(false);
                return 0;
            }

            Args args;
            try
            {
                args = ParseArgs(argv);
            }
            catch (ArgumentException e)
            {
                Console.Error.WriteLine("error: " + e.Message);
                Usage(true);
                return 2;
            }

            Console.WriteLine(Banner);
            Cell? cell = null;
            Task? stopping = null; // the decel STOP Ctrl+C started
            var gate = new object();

            // Ctrl+C: keep the process alive and stop every axis on a worker thread (StopAll may be called while another
            // thread waits in a group command). A second Ctrl+C ends the program at once.
            Console.CancelKeyPress += (sender, e) =>
            {
                lock (gate)
                {
                    if (stopping != null)
                    {
                        return;
                    }

                    e.Cancel = true;
                    var target = cell;
                    stopping = Task.Run(() => target?.StopAll(returnExceptions: true));
                }
            };

            var code = 0;
            try
            {
                var opened = OpenCell(args);
                lock (gate)
                {
                    cell = opened;
                }

                Run(opened, args, () => Volatile.Read(ref stopping) != null);
            }
            catch (Exception e) when (Volatile.Read(ref stopping) == null)
            {
                code = Report(e);
            }
            catch (Exception)
            {
                // interrupted: the group command ended because of the STOP
            }

            if (Volatile.Read(ref stopping) is Task stop)
            {
                stop.Wait(); // StopAll with returnExceptions does not throw for an axis; it holds until standstill
                Console.Error.WriteLine("interrupted (Ctrl+C): a decel STOP was sent to every axis; the servos are left as they are");
                code = 130;
            }
            else if (code != 0 && cell != null)
            {
                var results = cell.StopAll(returnExceptions: true); // try every axis, even when one fails
                Console.Error.WriteLine("       decel STOP sent to every axis; the servos are left as they are");
                foreach (var failed in results.Errors)
                {
                    Console.Error.WriteLine($"       STOP not confirmed on {failed.Key}: {failed.Value.Message}");
                }
            }

            cell?.Dispose(); // closes every port
            if (code == 0)
            {
                Console.WriteLine("done");
            }

            return code;
        }

        /// <summary>One simulated line per profile ("SIM1".."SIMn", one unit at address 1 each), or the given ports.</summary>
        private static Cell OpenCell(Args args)
        {
            if (args.Ports.Count == 0)
            {
                var lines = new Dictionary<string, SimLine>();
                foreach (var profile in args.SimProfiles)
                {
                    // Pmsl false: PIO in control, as after a power-up; the driver takes Modbus control at rest.
                    var name = "SIM" + (lines.Count + 1).ToString(CultureInfo.InvariantCulture);
                    lines[name] = new SimLine(new[] { new SimulatedUnit(new SimulatedUnitOptions { Profile = profile, Pmsl = false }) }, name);
                }

                return Cell.Open(lines, slaves: new[] { 1 }); // the cell owns the lines and closes them
            }

            // "COM3" scans the port (addresses 1..16); "COM3:1" names one address. Either every entry names one or none does.
            if (args.Slaves.Count == 0)
            {
                return Cell.Open(args.Ports);
            }

            return Cell.Open(args.Slaves.ToDictionary(kv => kv.Key, kv => kv.Value.ToArray()));
        }

        private static void Run(Cell cell, Args args, Func<bool> interrupted)
        {
            void StepDone()
            {
                if (interrupted())
                {
                    throw new OperationCanceledException();
                }
            }

            Console.WriteLine($"cell: {cell.Count} {(cell.Count == 1 ? "axis" : "axes")}");
            var plan = new Dictionary<string, Plan>();
            var speed = DefaultSpeed;
            foreach (var name in cell.Names)
            {
                var axis = cell[name];
                var limits = axis.Limits(); // the unit's own ranges, units and kind
                if (limits.Position.Min is not double low || limits.Position.Max is not double high)
                {
                    throw new EcNotReadyException(name + ": the unit's soft limits are not readable: set CellOptions.AxisOptions with a Stroke");
                }

                var half = Math.Floor(Math.Min(args.Step, (high - low) / 4.0) / 2.0 * 100.0) / 100.0;
                var middle = Math.Round((low + high) / 2.0, 2);
                plan[name] = new Plan { Low = middle - half, Middle = middle, High = middle + half, Units = limits.Units };
                // one speed for every axis: at least each unit's documented minimum (below it a unit may run unsteadily)
                speed = Math.Max(speed, limits.Speed.RecommendedMin ?? 0.0);
                var info = axis.Identify(); // READ-ONLY
                Console.WriteLine($"  {name,-8} {info.Model} ({limits.Family}, {limits.Kind}); low {F(plan[name].Low)}, "
                                  + $"middle {F(middle)}, high {F(plan[name].High)} {limits.Units}");
            }

            speed = args.Speed ?? speed;
            Console.WriteLine($"speed {speed.ToString("G", CultureInfo.InvariantCulture)} units/s for every move (checked against every unit before anything is sent)");
            PrintStatus(cell, "StatusAll", plan);
            StepDone();

            Console.WriteLine("ServoAll(true)");
            cell.ServoAll(true);
            StepDone();
            var homes = cell.StatusAll().Where(kv => args.Home || !kv.Value.Homed).Select(kv => kv.Key).ToList();
            if (homes.Count > 0)
            {
                Console.WriteLine($"HomeAll: {string.Join(", ", homes)} (each runs to its mechanical end)");
                cell.HomeAll(homes);
                StepDone();
            }
            else
            {
                Console.WriteLine("HomeAll: not needed, every axis reports its home (--home homes them anyway)");
            }

            foreach (var (label, pick) in new (string, Func<Plan, double>)[] { ("middle", p => p.Middle), ("low", p => p.Low), ("high", p => p.High) })
            {
                var watch = Stopwatch.StartNew();
                // checked first on every axis: one bad value refuses the whole move (CellException), nothing sent
                var done = cell.MoveAll(plan.ToDictionary(kv => kv.Key, kv => pick(kv.Value)), speed)!;
                Console.WriteLine($"MoveAll to the {label} positions: {Positions(done)} ({watch.Elapsed.TotalSeconds:0.00} s)");
                StepDone();
            }

            // Start the moves without waiting, then wait for all: the program could do other work in between.
            cell.MoveAll(plan.ToDictionary(kv => kv.Key, kv => kv.Value.Low), speed, wait: false);
            var arrived = cell.WaitAll();
            Console.WriteLine($"MoveAll (wait: false) + WaitAll to the low positions: {Positions(arrived)}");
            StepDone();

            // StopAll during a move: a decel STOP on every axis as fast as possible.
            cell.MoveAll(plan.ToDictionary(kv => kv.Key, kv => kv.Value.High), speed, wait: false);
            Thread.Sleep(200);
            var stopped = cell.StopAll();
            Console.WriteLine($"StopAll during a move to the high positions: stopped at {Positions(stopped)}");
            StepDone();

            PrintStatus(cell, "StatusAll", plan);
            Console.WriteLine("ServoAll(false)");
            cell.ServoAll(false);
        }

        private static void PrintStatus(Cell cell, string title, IReadOnlyDictionary<string, Plan> plan)
        {
            Console.WriteLine(title);
            foreach (var kv in cell.StatusAll())
            {
                var st = kv.Value;
                var flags = new List<string>();
                if (st.ServoOn) flags.Add("servo ON");
                if (st.Homed) flags.Add("homed");
                if (st.InPosition) flags.Add("in position");
                if (st.Moving) flags.Add("moving");
                if (st.ModbusEnabled) flags.Add("PC control");
                var text = string.Join(", ", flags);
                if (st.Alarm || st.Warning) // the bit can be set with no code (code 0)
                {
                    text += $"; {(st.Alarm ? "alarm" : "warning")}";
                    if (st.AlarmCode != 0)
                    {
                        var alarm = Alarms.AlarmInfoOf(st.AlarmCode);
                        text += $" {alarm.Hex} {alarm.Name}";
                    }
                    else
                    {
                        text += " (no code)";
                    }
                }

                var units = plan.TryGetValue(kv.Key, out var p) ? p.Units : string.Empty;
                Console.WriteLine($"  {kv.Key,-8} {F(st.PositionMm),8} {units,-3} {text}");
            }
        }

        /// <summary>"SIM1:1 65.00, SIM2:1 65.00": where each axis of a group result is.</summary>
        private static string Positions(CellResults<Status> results) =>
            string.Join(", ", results.Names.Select(n => n + " " + (results.ContainsKey(n) ? F(results[n].PositionMm) : "failed")));

        /// <summary>Print an error; a CellException lists every axis that failed. Returns the exit code 2.</summary>
        private static int Report(Exception e)
        {
            Console.Out.Flush();
            if (e is CellException cell)
            {
                Console.Error.WriteLine($"error: {cell.Operation} failed on {cell.Errors.Count} axis(es)");
                foreach (var failed in cell.Errors)
                {
                    Console.Error.WriteLine($"       {failed.Key}: {failed.Value.GetType().Name}: {failed.Value.Message}");
                }
            }
            else
            {
                Console.Error.WriteLine($"error: {e.GetType().Name}: {e.Message}");
            }

            return 2;
        }

        private static string F(double value) => value.ToString("0.00", CultureInfo.InvariantCulture);

        private static void Usage(bool toError)
        {
            var o = toError ? Console.Error : Console.Out;
            o.WriteLine("usage: MultiAxisCell (--sim [--sim-profiles A,B,...] | --port PORT[:SLAVE] ...) [--speed UNITS_S]");
            o.WriteLine("                     [--step UNITS] [--home]");
        }

        private static Args ParseArgs(string[] argv)
        {
            var a = new Args();
            var sim = false;
            for (var i = 0; i < argv.Length; ++i)
            {
                var opt = argv[i];
                string Value() => i + 1 < argv.Length ? argv[++i] : throw new ArgumentException(opt + " needs a value");
                double Positive()
                {
                    var text = Value();
                    if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) || !(v > 0) || double.IsInfinity(v))
                    {
                        throw new ArgumentException($"{opt} takes a number above 0, got '{text}'");
                    }

                    return v;
                }

                switch (opt)
                {
                    case "--sim": sim = true; break;
                    case "--home": a.Home = true; break;
                    case "--port":
                        var port = Value();
                        a.Ports.Add(port);
                        var colon = port.LastIndexOf(':');
                        if (colon > 0)
                        {
                            if (!int.TryParse(port.Substring(colon + 1), NumberStyles.Integer, CultureInfo.InvariantCulture, out var slave) || slave < 1 || slave > 247)
                            {
                                throw new ArgumentException($"--port {port}: the address after ':' takes 1..247");
                            }

                            var portName = port.Substring(0, colon);
                            if (!a.Slaves.TryGetValue(portName, out var slaves))
                            {
                                a.Slaves[portName] = slaves = new List<int>();
                            }

                            slaves.Add(slave);
                        }

                        break;
                    case "--sim-profiles":
                        a.SimProfiles = Value().Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries).Select(ParseProfile).ToArray();
                        break;
                    case "--speed": a.Speed = Positive(); break;
                    case "--step": a.Step = Positive(); break;
                    default: throw new ArgumentException("unknown option " + opt);
                }
            }

            if (sim == (a.Ports.Count > 0))
            {
                throw new ArgumentException("give --sim or one or more --port PORT");
            }

            if (a.Slaves.Count > 0 && a.Slaves.Values.Sum(s => s.Count) != a.Ports.Count)
            {
                throw new ArgumentException("give every --port as PORT (scanned) or every one as PORT:SLAVE, not a mix");
            }

            return a;
        }

        private static SimProfile ParseProfile(string name)
        {
            if (!Enum.TryParse(name.Trim(), ignoreCase: true, out SimProfile profile) || !Enum.IsDefined(typeof(SimProfile), profile)
                || char.IsDigit(name.Trim(), 0))
            {
                throw new ArgumentException($"--sim-profiles: '{name}' is not one of slider, gripper, rod, pcon, rotary, unknown");
            }

            return profile;
        }
    }
}
