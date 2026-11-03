// FirstMove - one axis through the C# library (IaiActuatorDriver.Axis): open, connect, servo ON, home when needed, an
// absolute move and two relative moves around mid-stroke, the status, servo OFF.
//
// Run it from the repository folder (close IA-OS first: only one program can use a serial port):
//
//     dotnet run --project examples/csharp/FirstMove -- --sim                     the simulator (an ELECYLINDER slider)
//     dotnet run --project examples/csharp/FirstMove -- --sim --sim-profile rod   slider, gripper, rod, pcon, rotary, unknown
//     dotnet run --project examples/csharp/FirstMove -- --port COM3               the unit on COM3, address 1
//     dotnet run --project examples/csharp/FirstMove -- --port COM3 --slave 2     the unit at address 2 on a shared port
//
// Options: --speed UNITS_S (default 20, raised to the unit's documented minimum speed where the driver reports one),
// --distance UNITS (the step out and back, default 10, at most a quarter of the stroke), --home (a home return even
// when the status says homed). The library project also copies the native core (iai_actuator_driver.dll from
// core/build/, core/README.md) next to the program when it is built.
//
// Every target and speed is checked against the unit's own ranges before anything is sent: a value outside them
// throws SoftLimitException and nothing moves. Every error the driver raises on purpose is an EcDriverException
// (or a subclass).
//
// Motion: to the middle of the stroke, then --distance further and back. The home return runs only when the status
// says the axis is not homed, or with --home: the axis runs to its mechanical end, so keep the travel clear. Test a
// new setup unloaded, and keep a hand on the emergency stop.
//
// End: the servo is switched OFF (a vertical axis without a brake can fall when the servo goes off: support it first)
// and Modbus control is handed back to PIO when PIO had it at the start. Ctrl+C sends a decel STOP (Stop may be called
// while another thread waits for a move); the waiting move then throws and the program closes the port with the servo
// left as it is. Exit codes: 0 done, 2 error, 130 interrupted.
//
// Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
using System;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver;
using IaiActuatorDriver.Simulation;

namespace IaiActuatorDriver.Examples
{
    internal static class Program
    {
        private const string Banner = "iai-actuator-driver example: first move, C# (UNOFFICIAL - not an IAI product)";
        private const double DefaultSpeed = 20.0;    // units/s: a first test (unloaded, mid-stroke, 20 mm/s or less)
        private const double DefaultDistance = 10.0; // units: the step out and back

        private sealed class Args
        {
            public string? Port;               // --port COM3; null = --sim
            public SimProfile SimProfile = SimProfile.Slider;
            public int Slave = 1;
            public double? Speed;
            public double Distance = DefaultDistance;
            public bool Home;
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
            Axis? axis = null;
            Task? stopping = null; // the decel STOP Ctrl+C started
            var gate = new object();

            // Ctrl+C: keep the process alive, send a decel STOP on a worker thread (it blocks until standstill); the move
            // waiting on the main thread then throws. A second Ctrl+C ends the program at once.
            Console.CancelKeyPress += (sender, e) =>
            {
                lock (gate)
                {
                    if (stopping != null)
                    {
                        return; // second Ctrl+C: e.Cancel stays false, the process ends
                    }

                    e.Cancel = true;
                    var target = axis;
                    stopping = Task.Run(() => target?.Stop());
                }
            };

            var motion = false; // a motion command ran: an error afterwards sends a decel STOP before the port closes
            var code = 0;
            try
            {
                var opened = OpenAxis(args);
                lock (gate)
                {
                    axis = opened;
                }

                FirstMove(opened, args, ref motion, () => Volatile.Read(ref stopping) != null);
            }
            catch (Exception e) when (Volatile.Read(ref stopping) == null)
            {
                code = Report(e);
            }
            catch (Exception)
            {
                // interrupted: the move ended because of the STOP
            }

            if (Volatile.Read(ref stopping) is Task stop)
            {
                try
                {
                    stop.Wait(); // the STOP holds until standstill: close the port after it
                }
                catch (AggregateException e)
                {
                    Console.Error.WriteLine("STOP after Ctrl+C failed: " + e.InnerException?.Message);
                }

                Console.Error.WriteLine("interrupted (Ctrl+C): a decel STOP was sent; the servo is left as it is");
                code = 130;
            }
            else if (code != 0 && motion && axis != null)
            {
                try
                {
                    axis.Stop(); // an error after a motion command: decel STOP before the port closes
                    Console.Error.WriteLine("       a decel STOP was sent; the servo is left as it is");
                }
                catch (EcDriverException e)
                {
                    Console.Error.WriteLine("       decel STOP failed too: " + e.Message);
                }
            }

            axis?.Dispose(); // closes the port
            if (code == 0)
            {
                Console.WriteLine("done");
            }

            return code;
        }

        /// <summary>The axis on --port (baud rate found automatically), or a simulated unit with --sim.</summary>
        private static Axis OpenAxis(Args args)
        {
            if (args.Port != null)
            {
                return Axis.Open(args.Port, slave: args.Slave);
            }

            // Pmsl false: PIO in control, as after a power-up, so Connect takes Modbus control as it does on hardware.
            var unit = new SimulatedUnit(new SimulatedUnitOptions { Profile = args.SimProfile, Slave = args.Slave, Pmsl = false });
            return Axis.Open(unit, slave: args.Slave); // the axis owns the simulated unit and closes it
        }

        private static void FirstMove(Axis ax, Args args, ref bool motion, Func<bool> interrupted)
        {
            void StepDone()
            {
                if (interrupted())
                {
                    throw new OperationCanceledException();
                }
            }

            var found = ax.Status();         // read-only: the state before this program sent anything
            var info = ax.Connect();         // identity and ranges; Modbus control is taken here, at rest only
            var limits = ax.Limits();        // every range the driver enforces, read from the unit itself
            var units = limits.Units;        // "mm", or "deg" on a rotary axis
            Show("unit", $"{info.Model} ({info.ProductFamily}; actuator: {info.Kind}), firmware {info.Firmware}, address {info.Slave}"
                         + (ax.BaudRate is int baud ? $", {baud} baud" : string.Empty));
            if (limits.Position.Min is not double low || limits.Position.Max is not double high)
            {
                throw new EcNotReadyException("the unit's soft limits are not readable here: set AxisOptions.Stroke");
            }

            // below the documented minimum speed a unit may run noisily or unsteadily (the driver only warns)
            var speed = args.Speed ?? Math.Max(DefaultSpeed, limits.Speed.RecommendedMin ?? 0.0);
            Show("range", $"{F(low)} .. {F(high)} {units}; speed up to {limits.Speed.Max:G} {units}/s; {speed:G} {units}/s for every move here");
            Show("status", Describe(found, units));
            StepDone();

            // A servo ON the status shows cannot work (EMG, drive power cut) is refused before anything is sent.
            var st = ax.Servo(true)!;
            Show("servo", "ON");
            StepDone();
            if (args.Home || !st.Homed)
            {
                Show("home", "home return: the axis runs to its mechanical end, keep the travel clear");
                st = ax.Home()!;
                motion = true;
                Show(string.Empty, $"homed at {F(st.PositionMm)} {units}");
                StepDone();
            }
            else
            {
                Show("home", "not needed: the status says homed (--home runs a home return anyway)");
            }

            var middle = Math.Round((low + high) / 2.0, 2);
            var step = Math.Floor(Math.Min(args.Distance, (high - low) / 4.0) * 100.0) / 100.0;
            st = ax.MoveTo(middle, speed)!; // accel 0.1 G, band 0.1, waits until in position and settled
            motion = true;
            Show("move", $"to the middle of the stroke: at {F(st.PositionMm)} {units}");
            StepDone();
            foreach (var delta in new[] { step, -step })
            {
                st = ax.MoveBy(delta, speed)!; // relative to the measured position
                Show("move", $"by {delta.ToString("+0.00;-0.00", CultureInfo.InvariantCulture)} {units}: at {F(st.PositionMm)} {units}");
                StepDone();
            }

            motion = false;
            Show("status", Describe(ax.Status(), units));
            ax.Servo(false);
            Show("servo", "OFF");
            if (!found.ModbusEnabled)
            {
                ax.DisableModbusControl(); // PIO had control when this program started: hand it back
                Show("control", "handed back to PIO");
            }
        }

        /// <summary>The status in one line: position and the flags that matter around a move.</summary>
        private static string Describe(Status st, string units)
        {
            var text = $"position {F(st.PositionMm)} {units}";
            if (st.ServoOn) text += ", servo ON";
            if (st.Homed) text += ", homed";
            if (st.InPosition) text += ", in position";
            if (st.Moving) text += ", moving";
            if (st.ModbusEnabled) text += ", PC control";
            if (st.EmergencyStop) text += ", emergency stop";
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

            return text;
        }

        /// <summary>Print an error; an alarm with its name and how to clear it. Returns the exit code 2.</summary>
        private static int Report(Exception e)
        {
            Console.Out.Flush();
            Console.Error.WriteLine($"error: {e.GetType().Name}: {e.Message}");
            if (e is EcAlarmException alarm)
            {
                var a = Alarms.AlarmInfoOf(alarm.AlarmCode);
                Console.Error.WriteLine($"       alarm {a.Hex} {a.Name}: {a.Reset}");
            }

            return 2;
        }

        private static void Show(string label, string text) => Console.WriteLine($"{label,-10} {text}");

        private static string F(double value) => value.ToString("0.00", CultureInfo.InvariantCulture);

        private static void Usage(bool toError)
        {
            var o = toError ? Console.Error : Console.Out;
            o.WriteLine("usage: FirstMove (--sim [--sim-profile NAME] | --port PORT) [--slave N] [--speed UNITS_S]");
            o.WriteLine("                 [--distance UNITS] [--home]");
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
                    case "--port": a.Port = Value(); break;
                    case "--sim-profile":
                        var profile = Value();
                        if (!Enum.TryParse(profile, ignoreCase: true, out a.SimProfile) || !Enum.IsDefined(typeof(SimProfile), a.SimProfile)
                            || char.IsDigit(profile, 0))
                        {
                            throw new ArgumentException("--sim-profile takes slider, gripper, rod, pcon, rotary or unknown");
                        }

                        break;
                    case "--slave":
                        if (!int.TryParse(Value(), NumberStyles.Integer, CultureInfo.InvariantCulture, out a.Slave) || a.Slave < 1 || a.Slave > 247)
                        {
                            throw new ArgumentException("--slave takes 1..247");
                        }

                        break;
                    case "--speed": a.Speed = Positive(); break;
                    case "--distance": a.Distance = Positive(); break;
                    default: throw new ArgumentException("unknown option " + opt);
                }
            }

            if (sim == (a.Port != null))
            {
                throw new ArgumentException("give --sim or --port PORT (one of them)");
            }

            return a;
        }
    }
}
