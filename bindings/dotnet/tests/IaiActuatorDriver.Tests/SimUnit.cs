using System;
using System.Collections.Generic;
using System.Linq;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// A small simulated unit behind a <see cref="FakeTransport"/> responder, for the axis tests: the rod's (or the PCON's)
    /// real register blocks plus a simple motion model. Every status read reports the state first and then advances the
    /// motion by <see cref="StepMm"/>, so the first read after a command always shows the axis moving.
    /// </summary>
    internal sealed class SimUnit
    {
        private readonly object _sync = new object();
        private bool _homeCoil;
        private bool _alrsCoil;
        private int _parwLeft;
        private int _silentLeft;

        private enum Phase
        {
            Idle,
            Move,
            Home,
            PushApproach,
            Push,
        }

        private Phase _phase = Phase.Idle;
        private double _target;
        private double _pushEnd;
        private int _pushDirection;

        public SimUnit(string profile = "rod", int address = 1)
        {
            Address = address;
            Profile = profile;
            if (profile == "pcon")
            {
                Load(UnitFixtures.PconBlocks, UnitFixtures.PconIndexTable, UnitFixtures.PconIdentification, UnitFixtures.PconPositions, UnitFixtures.PconAlarmHistory);
                SlaveId = T.Hex("AE FF 44 00 00");
                Position = 60.0;
                Homed = true;
            }
            else
            {
                Load(UnitFixtures.RodBlocks, UnitFixtures.RodIndexTable, UnitFixtures.RodIdentification, UnitFixtures.RodPositions, UnitFixtures.RodAlarmHistory);
                Put(0x0100, T.Words(UnitFixtures.RodSupport));
                SlaveId = T.Hex("EC FF 00 00 02");
            }
        }

        public int Address { get; }

        public string Profile { get; }

        public bool IsCon => Profile == "pcon";

        public Dictionary<int, ushort> Memory { get; } = new Dictionary<int, ushort>();

        public byte[]? SlaveId { get; set; }

        /// <summary>The transport to switch when a broadcast baud change arrives (null: ignored).</summary>
        public FakeTransport? Transport { get; set; }

        // ---- state
        public double Position { get; set; }

        public bool Servo { get; set; }

        public bool Homed { get; set; }

        public bool Pmss { get; set; }

        public bool Sfty { get; set; }

        public bool Stp { get; set; }

        public bool StopCoil { get; set; }

        public bool Pend { get; set; } = true;

        public bool Psfl { get; set; }

        public bool PushContact { get; set; }

        public bool Pushing { get; set; }

        public bool AlarmBit { get; set; }

        public int AlarmCode { get; set; }

        public bool Emergency { get; set; }

        public uint UptimeMs { get; set; } = 100_000;

        /// <summary>Distance per status read while moving.</summary>
        public double StepMm { get; set; } = 25.0;

        /// <summary>Where a push meets the work (null: the push misses).</summary>
        public double? ContactAt { get; set; }

        /// <summary>Reads of DRGE that show PARW busy after the PARW coil.</summary>
        public int ParwBusyReads { get; set; } = 2;

        /// <summary>0x9800 / 0x45 cmd 6 are accepted but never run (e.g. a STOP held by another master).</summary>
        public bool IgnoreStoredMoves { get; set; }

        /// <summary>Answer nothing to the next 0x45 home command (it still runs).</summary>
        public bool DropHomeReply { get; set; }

        /// <summary>Reject the next 0x45 home command with this code (and do not run it).</summary>
        public int? RejectHome { get; set; }

        /// <summary>Requests left unanswered after a software reset (the reply gap of a CON controller).</summary>
        public int SilentAfterReset { get; set; } = 3;

        /// <summary>Raise this alarm once the axis passes <see cref="AlarmAtMm"/> during a move.</summary>
        public int? AlarmDuringMove { get; set; }

        public double AlarmAtMm { get; set; }

        public List<string> Log { get; } = new List<string>();

        public void Put(int address, IReadOnlyList<ushort> words)
        {
            for (var i = 0; i < words.Count; i++)
            {
                Memory[address + i] = words[i];
            }
        }

        public ushort Word(int address) => Memory.TryGetValue(address, out var v) ? v : (ushort)0;

        public void PutU32(int address, uint value) => Put(address, new[] { (ushort)(value >> 16), (ushort)value });

        /// <summary>Ready for motion: servo on, homed, PMSS 1, at <paramref name="position"/>.</summary>
        public SimUnit Ready(double position = 0.0)
        {
            Servo = true;
            Homed = true;
            Pmss = true;
            Position = position;
            Pend = true;
            return this;
        }

        public (ModbusRtu Bus, FakeTransport Transport) Bus(int baudRate = 0)
        {
            var transport = new FakeTransport(Respond, baudRate);
            Transport = transport;
            if (baudRate > 0)
            {
                transport.AnswerAtBaudRate = baudRate;
            }

            return (new ModbusRtu(transport, 2, TimeSpan.Zero), transport);
        }

        public Axis Axis(AxisOptions? options = null, int baudRate = 0)
        {
            var (bus, _) = Bus(baudRate);
            return new Axis(bus, Address, options ?? Fast);
        }

        public static AxisOptions Fast { get; } = new AxisOptions { PollIntervalS = 0.002, SettleS = 0.02, PushSettleS = 0.03 };

        public bool IsMoving => _phase != Phase.Idle;

        public byte[]? Respond(byte[] request)
        {
            lock (_sync)
            {
                if (!Crc16.Check(request))
                {
                    return null;
                }

                if (request[0] == 0)
                {
                    Broadcast(request);
                    return null;
                }

                if (request[0] != Address)
                {
                    return null;
                }

                if (_silentLeft > 0)
                {
                    _silentLeft--;
                    return null;
                }

                switch (request[1])
                {
                    case 0x03:
                        return Read(U16(request, 2), U16(request, 4));
                    case 0x05:
                        Coil(U16(request, 2), U16(request, 4) == 0xFF00);
                        return Crc16.BuildFrame(Address, request.Skip(1).Take(5).ToArray());
                    case 0x06:
                        WriteRegister(U16(request, 2), U16(request, 4));
                        return Crc16.BuildFrame(Address, request.Skip(1).Take(5).ToArray());
                    case 0x10:
                    {
                        var address = U16(request, 2);
                        var count = U16(request, 4);
                        var words = Enumerable.Range(0, count).Select(i => (ushort)U16(request, 7 + (2 * i))).ToArray();
                        WriteRegisters(address, words);
                        return Crc16.BuildFrame(Address, request.Skip(1).Take(5).ToArray());
                    }

                    case 0x11:
                        if (SlaveId is null)
                        {
                            return null;
                        }

                        var pdu = new List<byte> { 0x11, (byte)SlaveId.Length };
                        pdu.AddRange(SlaveId);
                        return Crc16.BuildFrame(Address, pdu);
                    case 0x17:
                    {
                        var readAddress = U16(request, 2);
                        var readCount = U16(request, 4);
                        var writeAddress = U16(request, 6);
                        var writeCount = U16(request, 8);
                        var words = Enumerable.Range(0, writeCount).Select(i => (ushort)U16(request, 11 + (2 * i))).ToArray();
                        Put(writeAddress, words);
                        Log.Add($"FC17 0x{writeAddress:X4}");
                        var values = ReadWords(readAddress, readCount);
                        var p = new List<byte> { 0x17, (byte)(2 * values.Length) };
                        foreach (var v in values)
                        {
                            p.Add((byte)(v >> 8));
                            p.Add((byte)v);
                        }

                        return Crc16.BuildFrame(Address, p);
                    }

                    case 0x45:
                        return Iai(request[3], U16(request, 4), U16(request, 6));
                    default:
                        return Crc16.BuildFrame(Address, new byte[] { (byte)(request[1] | 0x80), 0x01 });
                }
            }
        }

        // ------------------------------------------------------------------ requests

        private byte[] Read(int address, int count) => T.ReadResponse(Address, ReadWords(address, count));

        private ushort[] ReadWords(int address, int count)
        {
            var advance = address <= Registers.DSS1 && address + count > Registers.DSS1;
            var monitor = address < Registers.MONITOR_BASE + 34 && address + count > Registers.MONITOR_BASE ? Monitor() : Array.Empty<ushort>();
            var values = new ushort[count];
            for (var i = 0; i < count; i++)
            {
                var a = address + i;
                if (a >= Registers.MONITOR_BASE && a < Registers.MONITOR_BASE + monitor.Length)
                {
                    values[i] = monitor[a - Registers.MONITOR_BASE];
                }
                else if (a == Registers.DRGE)
                {
                    values[i] = (ushort)((Pmss ? 0x0100 : 0) | (_parwLeft > 0 ? Registers.DRGE_PARW : 0));
                    if (_parwLeft > 0)
                    {
                        _parwLeft--;
                    }
                }
                else
                {
                    values[i] = Word(a);
                }
            }

            if (advance)
            {
                Advance();
            }

            return values;
        }

        private ushort[] Monitor()
        {
            var m = new ushort[34];
            var (ph, pl) = Registers.ToI32Words((long)Math.Round(Position * 100));
            m[0] = ph;
            m[1] = pl;
            m[2] = (ushort)AlarmCode;
            var dss1 = (Emergency ? Registers.DSS1_EMGS : 0) | (Sfty ? Registers.DSS1_SFTY : 0) | (Servo ? Registers.DSS1_SV : 0)
                | (Psfl ? Registers.DSS1_PSFL : 0) | (AlarmBit ? Registers.DSS1_ALMH : 0) | (Stp ? Registers.DSS1_STP : 0)
                | (Homed ? Registers.DSS1_HEND : 0) | (Pend ? Registers.DSS1_PEND : 0);
            var dsse = (Pmss ? Registers.DSSE_PMSS : 0) | (_phase != Phase.Idle && !Stp ? Registers.DSSE_MOVE : 0)
                | (_phase == Phase.Home ? Registers.DSSE_GHMS : 0) | (Pushing ? Registers.DSSE_PUSH : 0)
                | (PushContact ? Registers.DSSE_PUSH_CONTACT : 0) | (_parwLeft > 0 ? 0 : Registers.DSSE_PARW);
            m[5] = (ushort)dss1;
            m[6] = 0x8000; // DSS2 ENBS: a real unit reports its enable input on (no teaching tool's switch released)
            m[7] = (ushort)dsse;
            var (sh, sl) = Registers.ToU32Words(UptimeMs);
            m[16] = sh;
            m[17] = sl;
            UptimeMs += 10;
            return m;
        }

        private void Advance()
        {
            if (_phase == Phase.Idle || Stp || StopCoil)
            {
                return;
            }

            var goal = _phase == Phase.Push ? _pushEnd : _target;
            if (_phase == Phase.Push && ContactAt.HasValue)
            {
                var c = ContactAt.Value;
                if ((_pushDirection > 0 && c >= Math.Min(_target, _pushEnd) && c <= _pushEnd) || (_pushDirection < 0 && c <= Math.Max(_target, _pushEnd) && c >= _pushEnd))
                {
                    goal = c;
                }
            }

            var delta = goal - Position;
            Position = Math.Abs(delta) <= StepMm ? goal : Position + (Math.Sign(delta) * StepMm);
            if (AlarmDuringMove.HasValue && ((delta > 0 && Position >= AlarmAtMm) || (delta < 0 && Position <= AlarmAtMm)))
            {
                AlarmCode = AlarmDuringMove.Value;
                AlarmBit = true;
                Servo = false;
                _phase = Phase.Idle;
                Pend = false;
                return;
            }

            if (Position != goal)
            {
                return;
            }

            switch (_phase)
            {
                case Phase.Move:
                    _phase = Phase.Idle;
                    Pend = true;
                    break;
                case Phase.Home:
                    _phase = Phase.Idle;
                    Homed = true;
                    Pend = true;
                    break;
                case Phase.PushApproach:
                    _phase = Phase.Push;
                    Pushing = true;
                    break;
                case Phase.Push:
                    _phase = Phase.Idle;
                    if (ContactAt.HasValue && goal == ContactAt.Value)
                    {
                        Pend = true;
                        PushContact = true;
                    }
                    else
                    {
                        Psfl = true;
                        Pushing = false;
                    }

                    break;
            }
        }

        private void StartMove(double target)
        {
            if (!Servo || !Homed)
            {
                return;
            }

            ClearMotionFlags();
            _target = target;
            _phase = Phase.Move;
        }

        private void StartPush(double target, double band, int direction)
        {
            if (!Servo || !Homed)
            {
                return;
            }

            ClearMotionFlags();
            _target = target;
            _pushEnd = target + (direction * band);
            _pushDirection = direction;
            _phase = Phase.PushApproach;
        }

        private void StartHome()
        {
            if (!Servo)
            {
                return;
            }

            ClearMotionFlags();
            Homed = false;
            _target = 0.0;
            _phase = Phase.Home;
        }

        private void ClearMotionFlags()
        {
            Pend = false;
            Psfl = false;
            PushContact = false;
            Pushing = false;
        }

        private void Halt()
        {
            if (_phase != Phase.Idle)
            {
                _phase = Phase.Idle;
                Pend = true;
                Pushing = false;
            }
        }

        private void StartRow(int n, double speedScale)
        {
            if (IgnoreStoredMoves || !Pmss)
            {
                return; // the EC silently ignores 0x9800 while PMSS = 0 (VERIFIED)
            }

            var w = Enumerable.Range(0, 16).Select(i => Word(Registers.PositionAddress(n) + i)).ToArray();
            var entry = PositionEntry.FromRegisters(w);
            Log.Add($"row {n} at {speedScale:F2}");
            if (entry.IsPush)
            {
                StartPush(entry.PositionMm, entry.BandMm, (entry.ControlFlags & Registers.CTLF_PUSH_AWAY) != 0 ? 1 : -1);
            }
            else
            {
                StartMove(entry.PositionMm);
            }
        }

        private void Coil(int address, bool on)
        {
            Log.Add($"coil 0x{address:X4} {(on ? "ON" : "OFF")}");
            switch (address)
            {
                case Registers.COIL_SON:
                    Servo = on;
                    if (!on)
                    {
                        Halt();
                    }

                    break;
                case Registers.COIL_HOME:
                    if (on && !_homeCoil)
                    {
                        StartHome();
                    }

                    _homeCoil = on;
                    break;
                case Registers.COIL_ALRS:
                    if (on && !_alrsCoil)
                    {
                        AlarmCode = 0;
                        AlarmBit = false;
                    }

                    _alrsCoil = on;
                    break;
                case Registers.COIL_STP:
                    Stp = on;
                    break;
                case Registers.COIL_PMSL:
                    if (on && !Pmss)
                    {
                        Halt(); // switching PMSL on mid-move stops the move (VERIFIED)
                    }

                    Pmss = on;
                    break;
                case Registers.COIL_STOP:
                    StopCoil = on;
                    if (on)
                    {
                        Halt();
                    }

                    break;
                case Registers.COIL_SFTY:
                    Sfty = on;
                    break;
                case Registers.COIL_PARW:
                    if (on)
                    {
                        _parwLeft = ParwBusyReads;
                    }

                    break;
                case Registers.COIL_PCRF:
                    if (on)
                    {
                        var pcmd = Registers.FromI32Words(Word(Registers.PROFILE_BASE), Word(Registers.PROFILE_BASE + 1));
                        StartMove(pcmd * 0.01);
                    }

                    break;
            }
        }

        private void WriteRegister(int address, int value)
        {
            Memory[address] = (ushort)value;
            if (address == Registers.COMMAND1_BASE)
            {
                StartRow(value, 1.0);
            }
        }

        private void WriteRegisters(int address, ushort[] words)
        {
            if (address == Registers.DIRECT_MOVE_BASE && words.Length == Registers.DIRECT_MOVE_WORDS)
            {
                Put(address, words);
                var target = Registers.FromI32Words(words[0], words[1]) * 0.01;
                var band = Registers.FromU32Words(words[2], words[3]) * 0.01;
                var ctlf = words[8];
                if ((ctlf & Registers.CTLF_PUSH) != 0)
                {
                    StartPush(target, band, (ctlf & Registers.CTLF_PUSH_AWAY) != 0 ? 1 : -1);
                }
                else
                {
                    StartMove(target);
                }

                return;
            }

            Put(address, words);
            if (address >= Registers.POSITION_TABLE_BASE && address < Registers.POSITION_TABLE_BASE + (16 * 768) && words.Length == Registers.POSITION_WORDS_WRITABLE)
            {
                Memory[address + 15] = (ushort)Registers.PositionChecksum(words); // the unit updates the checksum itself
            }
        }

        private byte[]? Iai(int command, int op1, int op2)
        {
            Log.Add($"0x45 cmd 0x{command:X2} {op1} {op2}");
            var rejection = 0;
            switch (command)
            {
                case Registers.IAI_CMD_SERVO:
                    Servo = op1 != 0;
                    if (!Servo)
                    {
                        Halt();
                    }

                    break;
                case Registers.IAI_CMD_HOME:
                    if (RejectHome.HasValue)
                    {
                        rejection = RejectHome.Value;
                        RejectHome = null;
                        break;
                    }

                    if (op1 == 0)
                    {
                        rejection = 0x11;
                        break;
                    }

                    StartHome();
                    if (DropHomeReply)
                    {
                        DropHomeReply = false;
                        return null;
                    }

                    break;
                case Registers.IAI_CMD_RESET:
                    if (op1 == Registers.IAI_RESET_ALARM)
                    {
                        AlarmCode = 0;
                        AlarmBit = false;
                    }
                    else
                    {
                        SoftwareReset();
                    }

                    break;
                case Registers.IAI_CMD_ACTION_MODE:
                    Pmss = op1 != 0;
                    Sfty = op2 != 0;
                    break;
                case Registers.IAI_CMD_MOVE_POSITION:
                    StartRow(op1, (op2 + 1) / 256.0); // the PCON-CB runs (op2 + 1) / 256 (ON-WIRE)
                    break;
                default:
                    rejection = 0x01;
                    break;
            }

            return Crc16.BuildFrame(Address, new byte[] { 0x45, (byte)rejection, 0x00 });
        }

        private void SoftwareReset()
        {
            _phase = Phase.Idle;
            StopCoil = false;
            Stp = false;
            Servo = true; // the unit switches its servo on again by itself
            Sfty = false;
            if (IsCon)
            {
                _silentLeft = SilentAfterReset; // keeps PMSS 1 and its home; silent for a moment
            }
            else
            {
                Pmss = false; // like a power-up
                Homed = false; // incremental: home lost
            }
        }

        private void Broadcast(byte[] request)
        {
            if (request[1] == 0x45 && request[3] == Registers.IAI_CMD_BAUD)
            {
                var code = U16(request, 4);
                Log.Add($"broadcast baud code 0x{code:X2}");
                if (Transport != null && ModbusRtu.BaudByCode.TryGetValue(code, out var rate))
                {
                    Transport.AnswerAtBaudRate = rate;
                }
            }
        }

        private static int U16(byte[] frame, int offset) => (frame[offset] << 8) | frame[offset + 1];

        private void Load(IReadOnlyDictionary<int, string> blocks, string index, string identification, IReadOnlyDictionary<int, ushort[]> rows, string history)
        {
            foreach (var kv in blocks)
            {
                Put(kv.Key, T.Words(kv.Value));
            }

            Put(0x0000, T.Words(index));
            Put(0x0600, T.Words(identification));
            Put(0x0500, T.Words(history));
            foreach (var kv in rows)
            {
                Put(Registers.PositionAddress(kv.Key), kv.Value);
            }
        }
    }
}
