using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver.Simulation
{
    /// <summary>
    /// Byte-level simulated unit on an RS-485 line, usable wherever an <see cref="ITransport"/> is: it parses Modbus RTU
    /// request frames and answers exactly like a slave would (FC03 / 05 / 06 / 10 / 11 / 17 and IAI 0x45 / 0x48), so the
    /// whole driver stack runs without hardware: <c>new Axis(new ModbusRtu(new SimulatedUnit()))</c>. It never opens a port.
    /// </summary>
    /// <remarks>
    /// <para>Each <see cref="SimProfile"/> is seeded with the values read live from that unit (PNOW, DSS1, DSSE, the
    /// position rows, the 0x9900 block, the FC11 answer, the alarm history, the identification block with a placeholder
    /// serial, the soft limits, the parameter classes, the directory). Motion is a real-time trapezoid (speed VCMD,
    /// acceleration ACMD, deceleration ACMD or DCMD). Behaviour follows the hardware tests:</para>
    /// <list type="bullet">
    /// <item><description>DSS1 b0 = commanded target reached; b7 = 1 only when stopped; DSSE b4 / b5 while moving. The
    /// MOVE bit clears during the final deceleration.</description></item>
    /// <item><description>PEND and MEND rise as soon as the axis is inside the band of a normal move, while it is still
    /// moving, and the last <see cref="SimulatedUnitOptions.SettleTailMm"/> is covered slowly (the settling tail): a driver
    /// must also wait for PNOW to stop changing (VERIFIED behaviour, values chosen to match the readings).</description></item>
    /// <item><description>STOP (0x042C) latches until written OFF (worst case; UNVERIFIED): while ON, moves and home
    /// returns are ignored. It also stops a move with PMSS = 0 (VERIFIED, test 19); 0x9900 moves run with PMSS = 0 (test
    /// 18); switching PMSL ON mid-move stops the move. 0x9800 position-number moves are silently ignored with PMSS = 0
    /// (test N4); other coils, DRG writes and IAI commands are ignored / rejected 0x3A then (DOC; UNVERIFIED on the EC).</description></item>
    /// <item><description>Parameter writes (classes 7 / 8 / 9 / 0C) land in RAM: they read back at once and are lost by a
    /// restart (test N3) unless the PARW coil 0x0428 committed them (ON-WIRE: DRGE b7 reads 1 for about 0.2 s). A restart
    /// applies the committed soft limits and LS range.</description></item>
    /// <item><description>STP (0x040A) is a level input: it holds a move, and a move commanded while it is ON waits.</description></item>
    /// <item><description>Home: HEND 0, GHMS 1, about 30 mm/s to the mechanical end (-2.00 mm), a creep back to 0.00 with
    /// MOVE 0 but GHMS 1, then HEND 1 + PEND 1 (VERIFIED DSSE values).</description></item>
    /// <item><description>Push (CTLF bit 1, bit 2 = away from home): approach PCMD, then push over the band at
    /// min(VCMD, 20 mm/s); a workpiece at <see cref="ObstacleMm"/> gives contact, none gives PSFL at the band end. A start
    /// inside / past the push zone skips the approach. <see cref="SimulatedUnitOptions.PushBlipMm"/> reproduces hardware
    /// test G5 (a contact fingerprint for a moment, then a miss).</description></item>
    /// <item><description>IAI 0x45: servo cmd 9, home cmd 0x0A with op1 = the home pattern (op1 = 0 rejected 0x11), alarm
    /// reset cmd 5 op1 0, software reset cmd 5 op1 1 (after servo off; with the servo on it raises alarm 0x090), action
    /// mode cmd 0x0C, stored row cmd 6 at op2 / 255 of its speed (VERIFIED on the PCON), broadcast baud change cmd 4.
    /// Unknown commands are rejected 0x21. Alarms follow <see cref="Alarms.AlarmLevelOf"/>: a message-level code keeps
    /// the axis running, any other switches the servo off.</description></item>
    /// <item><description>Command profile 0x0F00 (20 words) is writable (read back as written, no motion) and runs with the
    /// PCRF coil 0x042E (VERIFIED on the PCON; modelled the same way on the EC, UNVERIFIED there).</description></item>
    /// <item><description>ZONS b12 / b13 / b14 (LS0-LS2) and the DOPM LS bits follow the position alone: lit within the
    /// LS range of row 0 / 1 / 2 (a push row's marker sits at its push end).</description></item>
    /// <item><description>The baud rate is modelled only with <see cref="SimulatedUnitOptions.BaudRate"/>: the unit then
    /// answers only when the PC side matches its rate; a BREAK of 150 ms or more drops it to 9600 (DOC).</description></item>
    /// <item><description>Reads are bounded by the directory: 0 inside a block without recorded data, exception 02 outside
    /// every block and ID map (stricter than the EC, so tests catch reads that stray past a block).</description></item>
    /// </list>
    /// <para>What it is NOT: proof of a unit's behaviour. Where the hardware tests and manuals are silent the simulator
    /// makes a simple choice. Passing tests against it shows the driver is self-consistent with what was observed.</para>
    /// <para>Thread-safe. Override the protected members to model a unit that differs (a refused function, a lost frame).</para>
    /// <para>The engine is the native core's simulator (iad_sim, the one the Python edition and the C++ core use): this class
    /// resolves the settings, opens the core's unit with them and reads its state; a clock of the unit's own
    /// (<see cref="SimulatedUnitOptions.Clock"/>) drives the core's unit. An override of the protected members runs in the
    /// core's request hook, on the thread doing the bus I/O: it may use <see cref="InjectFault"/> and
    /// <see cref="RecordRequest"/> and the base members, no other member of the unit.</para>
    /// </remarks>
    public partial class SimulatedUnit : ITransport
    {
        /// <summary>"Motion command while in SERVO-OFF condition".</summary>
        public const int AlarmServoOffMove = 0x080;

        /// <summary>"Absolute positional movement command when Homing is not completed".</summary>
        public const int AlarmNotHomedMove = 0x083;

        /// <summary>"Motion command during Homing".</summary>
        public const int AlarmMoveWhileHoming = 0x084;

        /// <summary>"Position No. error when moving".</summary>
        public const int AlarmPositionNo = 0x085;

        /// <summary>"Positional instruction information error when executing".</summary>
        public const int AlarmBadBand = 0x0A3;

        /// <summary>"The instructed deceleration is abnormal."</summary>
        public const int AlarmBadDecel = 0x0A7;

        /// <summary>"The instructed speed is abnormal."</summary>
        public const int AlarmBadSpeed = 0x0AB;

        /// <summary>"Software stroke limit over error".</summary>
        public const int AlarmSoftLimit = 0x0D9;

        /// <summary>"Software reset command while in SERVO-ON condition" (test R1: servo off, no reset).</summary>
        public const int AlarmResetServoOn = 0x090;

        /// <summary>"ABS error (2)": home lost (gripper after a committed P4 change + software reset).</summary>
        public const int AlarmAbsError = 0x0EE;

        /// <summary>0x45 rejection "Command code error".</summary>
        public const int RejectCommandCode = 0x21;

        /// <summary>0x45 rejection "Prohibition of command in monitor mode" (PMSS = 0).</summary>
        public const int RejectMonitorMode = 0x3A;

        /// <summary>0x45 rejection "1st operand error" (home with op1 = 0, VERIFIED).</summary>
        public const int RejectOperand1 = 0x11;

        /// <summary>0x45 rejection "Motion command while servo OFF".</summary>
        public const int RejectServoOff = 0x23;

        /// <summary>0x45 rejection "Motion command during homing".</summary>
        public const int RejectHoming = 0x24;

        /// <summary>The writable parameter classes (RAM until PARW commits them).</summary>
        public static readonly IReadOnlyList<int> ParameterRamClasses = new[] { 0x0007, 0x0008, 0x0009, 0x000C };

        /// <summary>P3 home direction (item 9.16) and P4 home position adjustment (item 9.21).</summary>
        public static readonly IReadOnlyList<(int ClassId, int ItemId)> HomeParameterItems = new[] { (0x0009, 16), (0x0009, 21) };

        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
        private static readonly IReadOnlyDictionary<SimProfile, ProfileDefaults> Profiles = BuildProfiles();

        private readonly object _sync = new object();
        private readonly Queue<SimFault> _faults = new Queue<SimFault>();
        private readonly (double Min, double Max) _softLimits;
        private readonly double _lsRange;
        private double? _pushBlipMm;
        private TimeSpan _readTimeout = TimeSpan.FromSeconds(0.2);

        /// <summary>A simulated unit with the given settings (<c>null</c>: a slider at slave 1).</summary>
        /// <exception cref="ArgumentException">An invalid profile, DOPM layout or time scale.</exception>
        /// <exception cref="FileNotFoundException"><see cref="SimulatedUnitOptions.BkecPath"/> does not exist.</exception>
        public SimulatedUnit(SimulatedUnitOptions? options = null)
        {
            var o = options ?? new SimulatedUnitOptions();
            if (!(o.TimeScale > 0) || double.IsInfinity(o.TimeScale))
            {
                throw new ArgumentOutOfRangeException(nameof(options), o.TimeScale, "TimeScale must be > 0");
            }

            if (!Profiles.TryGetValue(o.Profile, out var d))
            {
                throw new ArgumentException($"profile must be one of {string.Join(", ", Profiles.Keys)}", nameof(options));
            }

            var layout = o.Dopm ?? d.Dopm;
            if (layout != DopmLayout.ThreePosition && layout != DopmLayout.TwoPosition)
            {
                throw new ArgumentException("DOPM layout must be ThreePosition or TwoPosition", nameof(options));
            }

            if (o.Slave < 0 || o.Slave > 247)
            {
                throw new ArgumentOutOfRangeException(nameof(options), o.Slave, "slave address must be 0..247");
            }

            // the settings, the profile's values where the options leave them open (the core gets them resolved)
            var stroke = o.StrokeMm ?? d.StrokeMm;
            Profile = o.Profile;
            Slave = o.Slave;
            StrokeMm = stroke;
            TimeScale = o.TimeScale;
            MaxSpeedMmS = o.MaxSpeedMmS ?? d.MaxSpeedMmS;
            MaxAccelG = o.MaxAccelG;
            SystemCode = o.SystemCode ?? d.SystemCode;
            UnitCode = o.UnitCode ?? d.UnitCode;
            Firmware = o.Firmware ?? d.Firmware;
            Dopm = layout;
            RecomputePositionChecksum = o.RecomputePositionChecksum;
            DropReplyAfterRetarget = o.DropReplyAfterRetarget;
            StopLatches = o.StopLatches;
            MoveBitClearsInDecel = o.MoveBitClearsInDecel;
            _softLimits = (-Registers.SOFT_LIMIT_MARGIN_MM, stroke + Registers.SOFT_LIMIT_MARGIN_MM);
            _lsRange = o.LsRangeMm;
            EarlyInPosition = o.EarlyInPosition;
            SettleTailMm = o.SettleTailMm;
            SettleSpeedMmS = o.SettleSpeedMmS;
            _pushBlipMm = o.PushBlipMm;
            PushBlipLenMm = o.PushBlipLenMm;
            Absolute = o.Absolute ?? d.Absolute;
            BootBaudRate = o.BootBaudRate;
            ResetBaudRate = o.ResetBaudRate;
            ResetS = o.ResetS;
            SlaveIdDelayS = o.SlaveIdDelayS ?? d.SlaveIdDelayS;
            ParwBusyS = o.ParwBusyS;
            ParwDsseBusy = o.ParwDsseBusy;
            IgnoredBaudCodes = new HashSet<int>(o.IgnoredBaudCodes ?? Array.Empty<int>());

            // the directory (index table): the profile's own, or the first 0x100 words of a PC-tool backup
            IReadOnlyDictionary<int, ushort>? index = null;
            if (o.BkecPath != null)
            {
                if (!File.Exists(o.BkecPath))
                {
                    throw new FileNotFoundException("backup not found", o.BkecPath);
                }

                index = LoadBkecRegisters(o.BkecPath).Where(kv => kv.Key < 0x0100).ToDictionary(kv => kv.Key, kv => kv.Value);
                IndexSource = "bkec:" + o.BkecPath;
            }
            else
            {
                IndexSource = o.Profile == SimProfile.Slider ? "embedded" : "embedded:" + ProfileName;
            }

            (_core, _port, _coreHooks) = OpenCore(o, o.PositionMm ?? d.PositionMm, index); // SimulatedUnit.Native.cs
        }

        /// <summary>A simulated unit of <paramref name="profile"/> at <paramref name="slave"/>.</summary>
        public SimulatedUnit(SimProfile profile, int slave = 1, double timeScale = 1.0)
            : this(new SimulatedUnitOptions { Profile = profile, Slave = slave, TimeScale = timeScale })
        {
        }

        // ================================================================== settings (read-only)

        /// <summary>The profile the unit was built from.</summary>
        public SimProfile Profile { get; }

        /// <summary>The profile's name in lower case ("slider", "gripper", "rod", "pcon", "rotary", "unknown").</summary>
        public string ProfileName => Profile.ToString().ToLowerInvariant();

        /// <summary>Slave address the unit answers to.</summary>
        public int Slave { get; }

        /// <summary>Stroke (the soft limits are -0.30 .. stroke + 0.30 until a committed parameter changes them).</summary>
        public double StrokeMm { get; }

        /// <summary>Simulated seconds per real second.</summary>
        public double TimeScale { get; }

        /// <summary>The maximum speed (MAXV, the 0x0812 default speed); above it a move raises alarm 0x0AB.</summary>
        public double MaxSpeedMmS { get; }

        /// <summary>The maximum acceleration; above it a move raises alarm 0x0A7.</summary>
        public double MaxAccelG { get; }

        /// <summary>FC11 system code.</summary>
        public int SystemCode { get; }

        /// <summary>FC11 unit code.</summary>
        public int UnitCode { get; }

        /// <summary>FC11 firmware bytes.</summary>
        public (int High, int Low) Firmware { get; }

        /// <summary>DOPM layout.</summary>
        public DopmLayout Dopm { get; }

        /// <summary>Stored positions in use: 3 in 3-position mode (slider), 2 in 2-position mode.</summary>
        public int PositionCount => Dopm == DopmLayout.ThreePosition ? 3 : 2;

        /// <summary>Where the directory came from ("embedded", "embedded:gripper", "bkec:path").</summary>
        public string IndexSource { get; }

        public bool RecomputePositionChecksum { get; }

        public bool DropReplyAfterRetarget { get; }

        public bool StopLatches { get; }

        public bool MoveBitClearsInDecel { get; }

        public bool EarlyInPosition { get; }

        public double SettleTailMm { get; }

        public double SettleSpeedMmS { get; }

        public double PushBlipLenMm { get; }

        /// <summary>Absolute encoder (keeps HEND at a software reset).</summary>
        public bool Absolute { get; }

        public int BootBaudRate { get; }

        public int? ResetBaudRate { get; }

        public double ResetS { get; }

        public double SlaveIdDelayS { get; }

        public double ParwBusyS { get; }

        public bool ParwDsseBusy { get; }

        public IReadOnlyCollection<int> IgnoredBaudCodes { get; }

        // ================================================================== state (test hooks)

        /// <summary>Servo state. Setting it changes the state only (no event, no stop): a test hook.</summary>
        public bool ServoOn
        {
            get => CoreBool("servo_on");
            set => CoreSetState("servo_on", value, "setting ServoOn");
        }

        /// <summary>HEND. Setting it is a test hook.</summary>
        public bool Homed
        {
            get => CoreBool("homed");
            set => CoreSetState("homed", value, "setting Homed");
        }

        /// <summary>Modbus control (PMSL / DSSE PMSS). Setting it is a test hook (no stop).</summary>
        public bool Pmsl
        {
            get => CoreBool("pmsl");
            set => CoreSetState("pmsl", value, "setting Pmsl");
        }

        /// <summary>SFTY.</summary>
        public bool SafetySpeed => CoreBool("safety_speed");

        /// <summary>The present alarm code (0: none).</summary>
        public int AlarmCode => CoreInt("alarm_code");

        /// <summary>Test hook: a workpiece a push makes contact with (<c>null</c>: none, the push misses).</summary>
        public double? ObstacleMm
        {
            get => CoreObstacle();
            set => CoreSetObstacle(value);
        }

        /// <summary>Test hook: the G5 contact blip position during a push stroke (<c>null</c>: none).</summary>
        public double? PushBlipMm
        {
            get => CorePushBlip();
            set => CoreSetPushBlip(value);
        }

        /// <summary>The soft limits in force (changed only by a restart with committed parameters).</summary>
        public (double Min, double Max) SoftLimitsMm => CoreSoftLimits();

        /// <summary>The LS range in force (EC parameter 2).</summary>
        public double LsRangeMm => CoreState().Number("ls_range_mm") ?? _lsRange;

        /// <summary>Current simulated position.</summary>
        public double PositionMm => CoreNumber("position_mm");

        /// <summary>True while a move, push or home return is unfinished (whatever the MOVE bit says).</summary>
        public bool Moving => CoreBool("moving");

        /// <summary>True while a latched STOP blocks new commands.</summary>
        public bool StopHeld => CoreBool("stop_held");

        /// <summary>Position-table writes so far.</summary>
        public int NonvolatileWrites => CoreInt("nonvolatile_writes");

        /// <summary>Completed PARW commits.</summary>
        public int ParameterCommits => CoreInt("parameter_commits");

        /// <summary>Software resets executed.</summary>
        public int SoftwareResets => CoreInt("software_resets");

        /// <summary>Parameter RAM writes so far.</summary>
        public int ParameterRamWrites => CoreInt("parameter_ram_writes");

        /// <summary>Moves started with the PCRF coil.</summary>
        public int ProfileMoves => CoreInt("profile_moves");

        /// <summary>Moves started with 0x45 cmd 6.</summary>
        public int OverrideMoves => CoreInt("override_moves");

        /// <summary>True once <see cref="Close"/> was called.</summary>
        public bool Closed => !CoreBool("open");

        /// <summary>Every request frame received (the last 2000), oldest first.</summary>
        public IReadOnlyList<byte[]> Requests => CoreRequests();

        /// <summary>What the unit did (the last 2000 events), oldest first, e.g. "stop", "home complete".</summary>
        public IReadOnlyList<string> Events => CoreEvents();

        /// <summary>Parameter words written since the last commit / restart (address -> value): lost by a restart.</summary>
        public IReadOnlyDictionary<int, ushort> ParameterRam => CoreWords("parameter_ram");

        /// <summary>Parameter words committed with PARW (address -> value): kept across restarts.</summary>
        public IReadOnlyDictionary<int, ushort> ParameterNv => CoreWords("parameter_nv");

        /// <summary>A PARW commit is running (DRGE b7 reads 1).</summary>
        public bool ParwBusy => CoreBool("parw_busy");

        /// <summary>The rate the simulated unit listens at (<c>null</c>: the line is not modelled).</summary>
        public int? EcBaudRate => CoreUnitBaudRate();

        /// <summary>Forget the recorded requests.</summary>
        public void ClearRequests() => CoreControl("clear_requests");

        /// <summary>Forget the recorded events.</summary>
        public void ClearEvents() => CoreControl("clear_events");

        /// <summary>The latched state of coil <paramref name="address"/> (false when never written).</summary>
        public bool CoilState(int address) => CoreCoilState(address);

        /// <summary>Test hook: set the latched state of a coil without acting on it (e.g. a STOP left ON by another master).</summary>
        public void SetCoilState(int address, bool on) => CoreSetCoilState(address, on);

        /// <summary>Raw 16 words of position-table row <paramref name="n"/>.</summary>
        public ushort[] PositionRow(int n) => CorePositionRow(n);

        /// <summary>Corrupt the next <paramref name="count"/> responses.</summary>
        public void InjectFault(SimFault kind, int count = 1)
        {
            if (!Enum.IsDefined(typeof(SimFault), kind))
            {
                throw new ArgumentException("unknown fault kind", nameof(kind));
            }

            CoreInjectFault(kind, count);
        }

        /// <summary>
        /// Raise an alarm as if the actuator detected it: a message-level code (0x011-0x07F, e.g. the maintenance warning
        /// 0x04E) keeps the axis running; any other code sets ALMH and switches the servo off.
        /// </summary>
        public void TriggerAlarm(int code) => CoreTriggerAlarm(code);

        /// <summary>
        /// Test hook: a PLC starts a move over PIO. Ignored (returns false) while Modbus has control. With
        /// <paramref name="holdStart"/> the PLC keeps its start input on and the unit starts the move again as soon as a Modbus
        /// STOP is released - a WORST-CASE model (a Modbus STOP on a PIO-commanded move is untested). <see cref="PioRelease"/>
        /// drops the input.
        /// </summary>
        public bool PioMove(double targetMm, double speedMmS = 20.0, double accelG = 0.3, bool holdStart = false) => CorePioMove(targetMm, speedMmS, accelG, holdStart);

        /// <summary>Test hook: the PLC drops a start input held with <see cref="PioMove"/>.</summary>
        public void PioRelease() => CoreControl("pio_release");

        /// <summary>
        /// Test hook: a 24 V power cycle (tests 17, N3, N4). The axis stops where it is. After power-up: servo ON, homed,
        /// position kept, PMSS = 0, safety speed off, every coil released, no alarm, a 0x0FFF power-up marker in the alarm
        /// history, uptime restarted, the line at <see cref="BootBaudRate"/>. The position table is kept; parameter writes not
        /// committed revert (N3), committed ones apply now. Pending replies are lost.
        /// </summary>
        public void PowerCycle() => _core.PowerCycle(loseHome: false);

        /// <summary>Parse the first <paramref name="maxLines"/> <c>AAAA,VVVV</c> data lines of a PC-tool backup (read-only).</summary>
        public static Dictionary<int, ushort> LoadBkecRegisters(string path, int maxLines = 0x100)
        {
            var regs = new Dictionary<int, ushort>();
            foreach (var raw in File.ReadLines(path))
            {
                var line = raw.Trim();
                if (line.Length == 0 || line.StartsWith(";", StringComparison.Ordinal))
                {
                    continue;
                }

                var comma = line.IndexOf(',');
                var addr = comma < 0 ? line : line.Substring(0, comma);
                var value = comma < 0 ? string.Empty : line.Substring(comma + 1);
                regs[int.Parse(addr, NumberStyles.HexNumber, Inv)] = (ushort)int.Parse(value, NumberStyles.HexNumber, Inv);
                if (regs.Count >= maxLines)
                {
                    break;
                }
            }

            return regs;
        }

        // ================================================================== ITransport

        /// <inheritdoc />
        public void Write(byte[] buffer, int offset, int count)
        {
            if (buffer is null)
            {
                throw new ArgumentNullException(nameof(buffer));
            }

            var frame = new byte[count];
            Array.Copy(buffer, offset, frame, 0, count);
            CoreWrite(frame); // the core's request hook runs an override of WriteFrame
        }

        /// <summary>Send one whole frame (the bytes of one request).</summary>
        public void Write(byte[] frame) => Write(frame, 0, frame.Length);

        /// <inheritdoc />
        public int Read(byte[] buffer, int offset, int count) => CoreRead(buffer, offset, count);

        /// <summary>Read up to <paramref name="count"/> bytes of the pending answer (empty: none).</summary>
        public byte[] Read(int count)
        {
            var buffer = new byte[count];
            var n = Read(buffer, 0, count);
            Array.Resize(ref buffer, n);
            return buffer;
        }

        /// <inheritdoc />
        public void ResetInputBuffer() => CoreIo("clear", () => NativeMethods.iad_transport_reset_input(_port!));

        /// <summary>
        /// The PC side's baud rate, 0 when the line is not modelled (the default). Setting it changes the PC side in place;
        /// throws <see cref="NotSupportedException"/> on a unit without a line model.
        /// </summary>
        public virtual int BaudRate
        {
            get => CoreBaudRate();
            set => CoreSetBaudRate(value);
        }

        /// <inheritdoc />
        public TimeSpan ReadTimeout
        {
            get { lock (_sync) { return _readTimeout; } }
            set { lock (_sync) { _readTimeout = value; } }
        }

        /// <summary>A line BREAK: 150 ms or more drops the unit to 9600 baud (DOC; the driver's break path uses 200 ms).</summary>
        public void SendBreak(TimeSpan duration) => CoreIo("send a break on", () => NativeMethods.iad_transport_send_break(_port!, duration.TotalSeconds));

        /// <inheritdoc />
        public void Close() => CoreClose();

        /// <inheritdoc />
        public void Dispose() => Close();

        /// <inheritdoc />
        public override string ToString() => F($"SimulatedUnit({ProfileName}, slave {Slave})");

        // ================================================================== frame handling (overridable)

        /// <summary>
        /// One request frame arrives on the line: the core records it, models the line (baud rate, reset silence), handles it
        /// and queues the answer (with any injected fault). Override to lose or alter frames.
        /// </summary>
        protected virtual void WriteFrame(byte[] data) => CoreWriteFrame(data);

        /// <summary>Record a request frame (for an override of <see cref="WriteFrame"/> that does not call the base).</summary>
        protected void RecordRequest(byte[] data)
        {
            return; // the core records every frame written to the unit
        }

        /// <summary>Handle one frame; the answer frame, or <c>null</c> for silence (bad CRC, another slave, broadcast).</summary>
        protected virtual byte[]? HandleFrame(byte[] frame) => CoreHandleFrame(frame);

        /// <summary>FC11 report slave id: <c>[system] FF [unit] [firmware hi] [firmware lo]</c>.</summary>
        protected virtual byte[] Fc11(byte[] body) => CorePdu(0x11, body);

        /// <summary>IAI 0x45: <c>[group u8][cmd u8][op1 u16][op2 u16]</c>, answer <c>45 [rejection] 00</c>.</summary>
        protected virtual byte[] Fc45(byte[] body) => CorePdu(0x45, body);

        /// <summary>IAI 0x48: <c>[group u8][cmd u16][op1..op4 u32]</c>, answer <c>48 [rejection u16] [status1..4 u32]</c>.</summary>
        protected virtual byte[] Fc48(byte[] body) => CorePdu(0x48, body);

        /// <summary>FC05 write coil (the coil map of class 0x0D). Unknown coils raise exception 02.</summary>
        protected virtual void WriteCoil(int address, bool on) => CoreWriteCoil(address, on);

        /// <summary>Read one register (the whole register map; exception 02 outside every directory block).</summary>
        protected virtual int ReadWord(int address) => CoreReadWord(address);

        /// <summary>A Modbus exception the simulated unit answers with (<c>[function | 0x80] [code]</c>).</summary>
        protected sealed class SimException : Exception
        {
            /// <summary>Raise exception <paramref name="code"/> (01 illegal function, 02 illegal address, 03 illegal value).</summary>
            public SimException(int code)
                : base(F($"Modbus exception 0x{code:X2}"))
            {
                Code = code;
            }

            /// <summary>The exception code.</summary>
            public int Code { get; }
        }

        private static string F(FormattableString text) => text.ToString(Inv);

        /// <summary>The settings each profile gives where the options leave them open (the core's fixtures hold the rest).</summary>
        private static IReadOnlyDictionary<SimProfile, ProfileDefaults> BuildProfiles()
        {
            var rodBased = new ProfileDefaults
            {
                StrokeMm = 150.0,
                PositionMm = 0.0,
                MaxSpeedMmS = 800.0,
                UnitCode = 0x00,
                Firmware = (0x00, 0x02),
                Dopm = DopmLayout.TwoPosition, // live DOPM 0x0005 at 0.00 mm: b0 LS0, b2 *ALM
                Absolute = false, // no WA option: loses HEND at a software reset
                SlaveIdDelayS = 0.3, // FC11 answered after 0.2-0.4 s, not within 0.2 s
            };
            return new Dictionary<SimProfile, ProfileDefaults>
            {
                [SimProfile.Slider] = new ProfileDefaults
                {
                    StrokeMm = 150.0,
                    PositionMm = 73.95,
                    MaxSpeedMmS = 700.0,
                    UnitCode = 0x02,
                    Firmware = (0x00, 0x13),
                    Dopm = DopmLayout.ThreePosition,
                    Absolute = true, // WA: battery-less absolute encoder
                },
                [SimProfile.Gripper] = new ProfileDefaults
                {
                    StrokeMm = 9.5,
                    PositionMm = 9.5,
                    MaxSpeedMmS = 53.0,
                    UnitCode = 0x0A,
                    Firmware = (0x00, 0x02),
                    Dopm = DopmLayout.TwoPosition,
                    Absolute = true, // WA
                },
                [SimProfile.Rod] = rodBased,
                [SimProfile.Pcon] = new ProfileDefaults
                {
                    StrokeMm = 150.0,
                    PositionMm = 60.0,
                    MaxSpeedMmS = 840.0,
                    UnitCode = 0x44,
                    Firmware = (0x00, 0x00),
                    Dopm = DopmLayout.TwoPosition,
                    Absolute = true, // HEND 1 after power-up
                    SystemCode = 0xAE,
                },
                [SimProfile.Rotary] = rodBased with
                {
                    StrokeMm = 360.0,
                    MaxSpeedMmS = 600.0,
                    Absolute = true,
                    SlaveIdDelayS = 0.0,
                },
                [SimProfile.Unknown] = rodBased with
                {
                    StrokeMm = 100.0,
                    MaxSpeedMmS = 400.0,
                    UnitCode = 0x7E,
                    Firmware = (0x01, 0x00),
                    Absolute = true,
                    SlaveIdDelayS = 0.0,
                    SystemCode = 0x99,
                },
            };
        }

        private static int U16(byte[] data, int offset) => (data[offset] << 8) | data[offset + 1];

        private static byte[] Prepend(byte first, byte[] rest)
        {
            var result = new byte[rest.Length + 1];
            result[0] = first;
            Array.Copy(rest, 0, result, 1, rest.Length);
            return result;
        }

        /// <summary>Per-unit defaults of one profile.</summary>
        private sealed record ProfileDefaults
        {
            public double StrokeMm { get; init; }

            public double PositionMm { get; init; }

            public double MaxSpeedMmS { get; init; }

            public int UnitCode { get; init; }

            public (int High, int Low) Firmware { get; init; }

            public DopmLayout Dopm { get; init; }

            public bool Absolute { get; init; }

            public double SlaveIdDelayS { get; init; }

            public int SystemCode { get; init; } = 0xEC;
        }
    }
}
