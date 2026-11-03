using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Simulation;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Helpers shared by the tests that run the driver against <see cref="SimulatedUnit"/> (no port is opened).</summary>
    internal static class SimKit
    {
        /// <summary>The options of the Python suite's <c>make_ec</c>: stroke 0..150, 100 mm/s, 0.3 G, 5 ms polls.</summary>
        public static AxisOptions MakeOptions(CommandMethod strategy = CommandMethod.Coil, double? maxSpeed = 100.0, double? maxAccel = 0.3) => new AxisOptions
        {
            Stroke = (0.0, 150.0),
            MaxSpeedMmS = maxSpeed,
            MaxAccelG = maxAccel,
            Strategy = strategy,
            PollIntervalS = 0.005,
        };

        /// <summary>An axis owning its bus on a fresh simulated unit (time scale 50 unless given) with a frame recorder.</summary>
        public static (Axis Axis, SimulatedUnit Sim, FrameRecorder Rec) MakeAxis(
            SimulatedUnitOptions? sim = null, CommandMethod strategy = CommandMethod.Coil, double? maxSpeed = 100.0, double? maxAccel = 0.3, SimulatedUnit? unit = null)
        {
            unit ??= new SimulatedUnit(sim ?? Sim());
            var rec = new FrameRecorder();
            var bus = new ModbusRtu(unit, minGap: TimeSpan.Zero, frameLogger: rec.Log);
            return (new Axis(bus, 1, MakeOptions(strategy, maxSpeed, maxAccel), ownsBus: true), unit, rec);
        }

        /// <summary>Options for <see cref="MakeAxis"/>: time scale 50 and the given changes.</summary>
        public static SimulatedUnitOptions Sim(SimProfile profile = SimProfile.Slider, double timeScale = 50.0) =>
            new SimulatedUnitOptions { Profile = profile, TimeScale = timeScale };

        /// <summary>An axis owning its bus on <paramref name="sim"/>, stroke from the unit unless given, 5 ms polls.</summary>
        public static (Axis Axis, FrameRecorder Rec) Open(SimulatedUnit sim, AxisOptions? options = null, int retries = ModbusRtu.DefaultRetries)
        {
            var rec = new FrameRecorder();
            var bus = new ModbusRtu(sim, retries, TimeSpan.Zero, rec.Log);
            return (new Axis(bus, 1, options ?? new AxisOptions { PollIntervalS = 0.005 }, ownsBus: true), rec);
        }

        /// <summary>A bus on <paramref name="transport"/> without the inter-frame gap.</summary>
        public static ModbusRtu Bus(ITransport transport, FrameRecorder? rec = null) => new ModbusRtu(transport, minGap: TimeSpan.Zero, frameLogger: rec is null ? null : rec.Log);

        /// <summary>Every recorded request with function <paramref name="function"/>.</summary>
        public static List<byte[]> Sent(FrameRecorder rec, int function) => rec.Tx.Where(f => f.Length > 1 && f[1] == function).ToList();

        /// <summary>Every request in <paramref name="frames"/> with function <paramref name="function"/>.</summary>
        public static List<byte[]> Of(IEnumerable<byte[]> frames, int function) => frames.Where(f => f.Length > 1 && f[1] == function).ToList();

        /// <summary>The ON / OFF state of every FC05 write to <paramref name="coil"/>.</summary>
        public static List<bool> CoilWrites(IEnumerable<byte[]> frames, int coil) =>
            Of(frames, 0x05).Where(f => ((f[2] << 8) | f[3]) == coil).Select(f => f[4] == 0xFF).ToList();

        /// <summary>(index in <paramref name="frames"/>, on) of every FC05 write to <paramref name="coil"/>.</summary>
        public static List<(int Index, bool On)> CoilFrames(IReadOnlyList<byte[]> frames, int coil) =>
            frames.Select((f, i) => (f, i)).Where(x => x.f.Length > 5 && x.f[1] == 0x05 && ((x.f[2] << 8) | x.f[3]) == coil).Select(x => (x.i, x.f[4] == 0xFF)).ToList();

        /// <summary>The 9-word 0x9900 writes as (pcmd, inp, vcmd, acmd, ppow, ctlf).</summary>
        public static List<(int Pcmd, uint Inp, uint Vcmd, int Acmd, int Ppow, int Ctlf)> Moves(IEnumerable<byte[]> frames)
        {
            var result = new List<(int, uint, uint, int, int, int)>();
            foreach (var f in frames)
            {
                if (f.Length >= 25 && f[1] == 0x10 && ((f[2] << 8) | f[3]) == Registers.DIRECT_MOVE_BASE && ((f[4] << 8) | f[5]) == 9)
                {
                    var w = Enumerable.Range(0, 9).Select(i => (f[7 + (2 * i)] << 8) | f[8 + (2 * i)]).ToArray();
                    result.Add((Registers.FromI32Words(w[0], w[1]), Registers.FromU32Words(w[2], w[3]), Registers.FromU32Words(w[4], w[5]), w[6], w[7], w[8]));
                }
            }

            return result;
        }

        /// <summary>The (address, count) of every FC03 request in <paramref name="frames"/>.</summary>
        public static List<(int Start, int Count)> Fc03Ranges(IEnumerable<byte[]> frames) =>
            Of(frames, 0x03).Select(f => (((f[2] << 8) | f[3]), ((f[4] << 8) | f[5]))).ToList();

        /// <summary>A full frame (slave + PDU + CRC) from hex.</summary>
        public static byte[] Frame(int slave, string pdu) => Crc16.BuildFrame(slave, T.Hex(pdu));

        /// <summary>Send one raw request and return whatever the unit answers.</summary>
        public static byte[] Exchange(SimulatedUnit sim, string pdu, int slave = 1)
        {
            sim.ResetInputBuffer();
            sim.Write(Frame(slave, pdu));
            return sim.Read(512);
        }

        /// <summary>The 9-word 0x9900 block of a move as written by another master (bypassing the driver).</summary>
        public static ushort[] MoveBlock(double mm, double band, double speed, double accelG, int ppow = 0, int ctlf = 0)
        {
            var (ph, pl) = Registers.ToI32Words((long)Math.Round(mm * 100));
            var (ih, il) = Registers.ToU32Words((long)Math.Round(band * 100));
            var (vh, vl) = Registers.ToU32Words((long)Math.Round(speed * 100));
            return new[] { ph, pl, ih, il, vh, vl, (ushort)Math.Round(accelG * 100), (ushort)ppow, (ushort)ctlf };
        }

        /// <summary>A move sent by another master (e.g. a second tool), bypassing the driver's checks: band 0.10, 0.30 G.</summary>
        public static void RawMove(ModbusRtu bus, double targetMm, double speedMmS) =>
            bus.WriteRegisters(1, Registers.DIRECT_MOVE_BASE, MoveBlock(targetMm, 0.10, speedMmS, 0.30));

        /// <summary>
        /// Every status the driver read, decoded from the recorded answers to FC03 reads of the monitor block 0x9000 (the
        /// tests cannot hook <see cref="Axis.Status"/> itself, so they watch the line).
        /// </summary>
        public static List<Status> Observed(FrameRecorder rec)
        {
            var result = new List<Status>();
            byte[]? lastTx = null;
            foreach (var (direction, f) in rec.Frames)
            {
                if (direction == FrameDirection.Tx)
                {
                    lastTx = f;
                    continue;
                }

                if (direction != FrameDirection.Rx || lastTx is null || f.Length < 5 || lastTx.Length < 6 || f[1] != 0x03 || lastTx[1] != 0x03)
                {
                    continue;
                }

                if (((lastTx[2] << 8) | lastTx[3]) != Registers.MONITOR_BASE || f[2] / 2 < Registers.STATUS_BLOCK_MIN_WORDS)
                {
                    continue;
                }

                var words = Enumerable.Range(0, f[2] / 2).Select(i => (ushort)((f[3 + (2 * i)] << 8) | f[4 + (2 * i)])).ToArray();
                result.Add(Status.FromRegisters(words));
            }

            return result;
        }

        /// <summary>Position from a monitor block read.</summary>
        public static double Pnow(IReadOnlyList<ushort> words) => Registers.FromI32Words(words[0], words[1]) / 100.0;

        /// <summary>Wait until <paramref name="condition"/> holds (or fail after <paramref name="timeoutS"/>).</summary>
        public static void WaitUntil(Func<bool> condition, double timeoutS = 5.0)
        {
            var deadline = DateTime.UtcNow.AddSeconds(timeoutS);
            while (!condition())
            {
                if (DateTime.UtcNow > deadline)
                {
                    throw new TimeoutException("condition not met in time");
                }

                Thread.Sleep(2);
            }
        }
    }
}
