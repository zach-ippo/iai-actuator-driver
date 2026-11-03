using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver
{
    /// <summary>
    /// One IAI axis on a Modbus RTU bus: an ELECYLINDER, or an actuator on a CON-family controller (PCON, ACON, DCON, SCON
    /// ...), whatever the actuator kind (slider, rod, gripper, rotary ...). The same commands everywhere; only the unit label
    /// (<see cref="Units"/>: "mm", or "deg" on a rotary axis, AKID item 7.33 = 1) and the unit's own ranges differ.
    /// </summary>
    /// <remarks>
    /// <para>Ranges and capabilities are read from the unit itself (directory and ID maps, soft limits 7.1 / 7.2, MAXV 7.4,
    /// MAXA 7.5, MAXD 7.6, MXPP 7.9, MNPP 7.58, AKID 7.33, MAXP 3.2, the support words), never from a table of tested
    /// models: <see cref="Limits"/> reports them. Every product is driven the same way.</para>
    /// <para>Safety defaults (the same guards as the Python edition):</para>
    /// <list type="bullet">
    /// <item><description>Every motion input is checked against <see cref="Limits"/> before anything is sent
    /// (<see cref="SoftLimitException"/>, nothing written). Configured caps can only narrow the unit's own.</description></item>
    /// <item><description>Every move writes the full 9-word 0x9900 block, CTLF included, so a stale CTLF bit left by another
    /// tool can never turn an absolute move into a relative move or a push.</description></item>
    /// <item><description>Moves require Modbus control, servo on, homed, no alarm / emergency stop, not paused, not homing.
    /// Modbus control (PMSL) is taken automatically, but only at rest: switching PMSL on mid-move stops the move on the EC
    /// (VERIFIED, hardware test 18).</description></item>
    /// <item><description>Completion is PEND (or PSFL) and the position within the band and PNOW unchanged over
    /// <see cref="SettleS"/>: on hardware PEND rose inside the band while the axis was still moving (VERIFIED). A push
    /// outcome must hold for <see cref="PushSettleS"/> (hardware test G5).</description></item>
    /// <item><description>Every stop - <see cref="Stop"/> and the automatic stop after an error, timeout, alarm or
    /// cancellation - uses the VERIFIED STOP coil sequence: 0x042C ON, wait until the axis stopped, 0x042C OFF.</description></item>
    /// <item><description>Position-table writes and dangerous parameter writes go straight through (a risky one with a
    /// warning); an unchanged value is not written and every non-volatile write is counted.</description></item>
    /// </list>
    /// <para>The axis is the native core's (iad_axis on the core bus of <see cref="Bus"/>): the reads, guards, waits and the
    /// non-volatile accounting run there, the same as in the Python edition.</para>
    /// <para>Thread safety: every transaction holds the port's lock (<see cref="ModbusRtu"/>), the axis's cached state is
    /// guarded by the core axis, and configuration sequences (parameter / position-table writes, software reset, baud
    /// change) are serialised per port. <see cref="Stop"/> and <see cref="Pause"/> may be called from another thread while a
    /// move waits. There is no watchdog: if the PC goes silent, a move in progress is completed (VERIFIED on the EC).</para>
    /// </remarks>
    public sealed partial class Axis : IDisposable
    {
        /// <summary>Default <see cref="SettleS"/>: "done" needs PNOW unchanged over two polls this far apart.</summary>
        public const double SettleSDefault = 0.1;

        /// <summary>Default <see cref="PushSettleS"/>: a push contact / miss must hold this long, position unchanged (G5).</summary>
        public const double PushSettleSDefault = 0.15;

        /// <summary>A holding contact may creep this much over <see cref="PushSettleS"/> (hardware test 14: 0.01 mm in 0.26 s).</summary>
        public const double PushCreepMm = 0.05;

        /// <summary>A 0x9800 position-number move must visibly start within this (hardware: about 50 ms).</summary>
        public const double StoredStartS = 0.5;

        /// <summary>Speed cap (units/s) only while neither the unit's maximum nor a configured cap is known.</summary>
        public const double FallbackSpeedCap = 100.0;

        /// <summary>Accel / decel cap (G) only while neither the unit's maximum nor a configured cap is known.</summary>
        public const double FallbackAccelCapG = 0.30;

        /// <summary>Push cap (%) only while neither MXPP nor a configured cap is known.</summary>
        public const double FallbackPushCapPct = 70.0;

        /// <summary>Driver sanity limit of a positioning band (units); a push band may span the stroke.</summary>
        public const double MoveBandMax = 10.0;

        /// <summary>Driver sanity limits of the EC parameter writes (the unit's own valid ranges are not documented).</summary>
        public const double P1MinMm = 1.00;

        public const double P2MinMm = 0.01;

        public const double P2MaxMm = 1.00;

        public const double P4MinMm = 0.00;

        public const double P4MaxMm = 5.00;

        /// <summary>Before a parameter write / the PARW coil: wait at most this for DRGE b7 = 0 and DSSE b7 = 1.</summary>
        public const double ParwReadyS = 2.0;

        /// <summary>After the PARW coil: idle again within this (ON-WIRE: 0.24 s).</summary>
        public const double ParwClearS = 2.0;

        /// <summary>With no busy sign seen, keep watching this long before "unconfirmed".</summary>
        public const double ParwObserveS = 0.3;

        /// <summary>The units' non-volatile memory lasts about this many write cycles (IAI).</summary>
        public const int NvWriteCycles = 100_000;

        /// <summary>More non-volatile writes than this in the last hour on one axis: one WARNING per hour (never a refusal).</summary>
        public const int NvWarnPerHour = 60;

        /// <summary>The sliding window of the rate warning, in seconds.</summary>
        public const double NvWindowS = 3600.0;

        /// <summary>The note of a write that was skipped because the unit already holds the value (see <see cref="ParameterWriteResult.Skipped"/>).</summary>
        public const string NvSkipNote = "unchanged: nothing written (one non-volatile write cycle saved)";

        /// <summary>After the software reset: wait this long before probing (ON-WIRE: IA-OS reopened after about 1.1 s).</summary>
        public const double ResetWaitS = 1.0;

        /// <summary>Give up when the unit has not answered this long after the software reset.</summary>
        public const double ResetTimeoutS = 10.0;

        /// <summary>The line break of <see cref="BaudChangeMethod.Break"/> (DOC: 150 ms or more drops the unit to 9600 baud).</summary>
        public const double BreakS = 0.2;

        /// <summary>After the broadcast baud change, before the port switches (ON-WIRE: IA-OS waited about 0.2 s).</summary>
        public const double BaudSettleS = 0.2;

        private const string LogSource = "IaiActuatorDriver.Axis";

        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        private readonly object _sync = new object();
        private readonly bool _ownsBus;
        private (double Min, double Max)? _stroke;
        private double? _maxSpeed;
        private double? _maxAccel;
        private double? _maxPush;
        private CommandStrategy _strategy;
        private double _pollInterval;
        private double _settle;
        private double _pushSettle;
        private bool _autoModbusControl;
        private CompletionRule? _completion;

        /// <summary>One axis on an open bus (the caller keeps owning <paramref name="bus"/>).</summary>
        /// <param name="bus">An open <see cref="ModbusRtu"/>.</param>
        /// <param name="slave">Modbus address (axis number + 1).</param>
        /// <param name="options">Caps, strategy and timing (<c>null</c>: defaults; every cap = the unit's own maximum).</param>
        public Axis(ModbusRtu bus, int slave = 1, AxisOptions? options = null)
            : this(bus, slave, options, ownsBus: false)
        {
        }

        /// <summary>One axis at <paramref name="slave"/> on a shared <see cref="Port"/> (gateway or daisy chain; UNVERIFIED).</summary>
        public Axis(Port port, int slave = 1, AxisOptions? options = null)
            : this((port ?? throw new ArgumentNullException(nameof(port))).Bus, slave, options, ownsBus: false)
        {
        }

        internal Axis(ModbusRtu bus, int slave, AxisOptions? options, bool ownsBus)
            : this(bus, slave, options, ownsBus, null)
        {
        }

        /// <summary>
        /// An axis on <paramref name="bus"/> over a core axis that exists already (<paramref name="adopted"/>: a cell's own
        /// axis, iad_cell_axis_ref), given these settings; null opens one on the bus.
        /// </summary>
        internal Axis(ModbusRtu bus, int slave, AxisOptions? options, bool ownsBus, NativeAxis? adopted)
        {
            Bus = bus ?? throw new ArgumentNullException(nameof(bus));
            if (slave < 1 || slave > 247)
            {
                throw new ArgumentOutOfRangeException(nameof(slave), slave, "slave address must be 1..247");
            }

            var o = options ?? new AxisOptions();
            Slave = slave;
            _ownsBus = ownsBus;
            Stroke = o.Stroke;
            MaxSpeedMmS = o.MaxSpeedMmS;
            MaxAccelG = o.MaxAccelG;
            MaxPushPct = o.MaxPushPct;
            _strategy = CommandStrategy.For(o.Strategy);
            PollIntervalS = o.PollIntervalS;
            AutoModbusControl = o.AutoModbusControl;
            SettleS = o.SettleS;
            PushSettleS = o.PushSettleS;
            _completion = o.Completion == CompletionRule.Fast ? CompletionRule.Fast : (CompletionRule?)null;
            _core = adopted is null ? OpenCore() : AdoptCore(adopted);
            _completion = null;
            HookEvents();
        }

        // ------------------------------------------------------------------ construction / session

        /// <summary>
        /// Open serial port <paramref name="portName"/> once and return an axis that owns it. With
        /// <paramref name="baudRate"/> <c>null</c> the rate is found by probing in place (115200 first, then 38400 - the EC's
        /// power-up rate, VERIFIED - 9600, 230400, 57600, 19200); the port is never closed and reopened. Throws
        /// <see cref="ModbusTimeoutException"/> (port closed again) when no rate answers.
        /// </summary>
        public static Axis Open(
            string portName,
            int? baudRate = null,
            int slave = 1,
            AxisOptions? options = null,
            TimeSpan? readTimeout = null,
            int retries = ModbusRtu.DefaultRetries,
            Action<FrameDirection, byte[]>? frameLogger = null)
        {
            var transport = new SerialTransport(portName, baudRate ?? ModbusRtu.DefaultBaudRate, readTimeout);
            return Open(transport, slave, options, autoBaud: baudRate is null, retries, frameLogger);
        }

        /// <summary>
        /// Wrap <paramref name="transport"/> in a bus the returned axis owns. With <paramref name="autoBaud"/> and a transport
        /// that has a baud rate, the rate the unit answers at is found in place first; a transport without one (an
        /// in-memory fake) is used as it is.
        /// </summary>
        public static Axis Open(
            ITransport transport,
            int slave = 1,
            AxisOptions? options = null,
            bool autoBaud = true,
            int retries = ModbusRtu.DefaultRetries,
            Action<FrameDirection, byte[]>? frameLogger = null)
        {
            var bus = new ModbusRtu(transport ?? throw new ArgumentNullException(nameof(transport)), retries, null, frameLogger);
            try
            {
                var detected = false;
                if (autoBaud && bus.CanChangeBaudRate)
                {
                    try
                    {
                        bus.DetectBaudRate(slave, ModbusRtu.BaudDetectOrder);
                    }
                    catch (ModbusTimeoutException exc)
                    {
                        exc.AddNote("check the 24 V supply, the cable, the slave address (axis number + 1) and that IA-OS (or any other program) has disconnected");
                        throw;
                    }

                    detected = true;
                }

                return new Axis(bus, slave, options, ownsBus: true) { BaudDetected = detected };
            }
            catch
            {
                bus.Close();
                throw;
            }
        }

        /// <summary>The bus this axis talks on.</summary>
        public ModbusRtu Bus { get; }

        /// <summary>Modbus address (axis number + 1).</summary>
        public int Slave { get; }

        /// <summary>The serial baud rate in use (<c>null</c> for a transport without one).</summary>
        public int? BaudRate => Bus.BaudRate;

        /// <summary><see cref="Open(ITransport, int, AxisOptions?, bool, int, Action{FrameDirection, byte[]}?)"/> found the baud rate by probing.</summary>
        public bool BaudDetected { get; private set; }

        /// <summary>The configured stroke (min, max), or <c>null</c> to use the unit's soft limits. Settable.</summary>
        public (double Min, double Max)? Stroke
        {
            get
            {
                lock (_sync)
                {
                    return _stroke;
                }
            }

            set
            {
                if (value.HasValue)
                {
                    var (lo, hi) = value.Value;
                    if (!IsFinite(lo) || !IsFinite(hi) || !(lo < hi))
                    {
                        throw new ArgumentException($"invalid stroke ({lo}, {hi})", nameof(value));
                    }
                }

                lock (_sync)
                {
                    _stroke = value;
                }

                ApplyOptions();
            }
        }

        /// <summary>Optional speed cap (units/s); <c>null</c> = the unit's MAXV 7.4. The cap enforced is <see cref="SpeedCapMmS"/>.</summary>
        public double? MaxSpeedMmS
        {
            get
            {
                lock (_sync)
                {
                    return _maxSpeed;
                }
            }

            set
            {
                if (value.HasValue && !(value.Value > 0 && value.Value <= 100_000))
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "MaxSpeedMmS must be > 0 (or null: the unit's own maximum)");
                }

                lock (_sync)
                {
                    _maxSpeed = value;
                }

                ApplyOptions();
            }
        }

        /// <summary>Optional accel / decel cap (G); <c>null</c> = the unit's MAXA / MAXD 7.5 / 7.6.</summary>
        public double? MaxAccelG
        {
            get
            {
                lock (_sync)
                {
                    return _maxAccel;
                }
            }

            set
            {
                if (value.HasValue && !(value.Value > 0 && value.Value <= Registers.ACMD_MAX_RAW * Registers.ACCEL_SCALE + 1e-9))
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "MaxAccelG must be in (0, 3.0] (or null: the unit's own maximum)");
                }

                lock (_sync)
                {
                    _maxAccel = value;
                }

                ApplyOptions();
            }
        }

        /// <summary>Optional push cap (%); <c>null</c> = the unit's MXPP 7.9.</summary>
        public double? MaxPushPct
        {
            get
            {
                lock (_sync)
                {
                    return _maxPush;
                }
            }

            set
            {
                if (value.HasValue && !(value.Value > 0 && value.Value <= 100))
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "MaxPushPct must be in (0, 100] (or null: the unit's own maximum)");
                }

                lock (_sync)
                {
                    _maxPush = value;
                }

                ApplyOptions();
            }
        }

        /// <summary>
        /// How servo / home / alarm reset are sent (moves always use 0x9900, stops the STOP coil). The core sends them by
        /// the strategy's <see cref="CommandStrategy.Method"/>: a strategy class of the caller's own is not supported
        /// (<see cref="NotSupportedException"/>).
        /// </summary>
        public CommandStrategy Strategy
        {
            get
            {
                lock (_sync)
                {
                    return _strategy;
                }
            }

            set
            {
                if (value is null)
                {
                    throw new ArgumentNullException(nameof(value));
                }

                if (!(value is StandardCoilStrategy || value is IaiCommandStrategy || value is IaiCommand2Strategy))
                {
                    throw new NotSupportedException(
                        $"the native core sends servo / home / alarm reset by method only (Coil, Iai, Iai2); a strategy class of its own ({value.GetType().Name}) is not supported");
                }

                Run(() => NativeMethods.iad_axis_set_strategy(H, NativeAxisOptions.MethodCode(value.Method)));
                lock (_sync)
                {
                    _strategy = value;
                }
            }
        }

        /// <summary>Status polling period while waiting (s).</summary>
        public double PollIntervalS
        {
            get => Volatile.Read(ref _pollInterval);
            set
            {
                Volatile.Write(ref _pollInterval, value >= 0 && IsFinite(value) ? value : throw new ArgumentOutOfRangeException(nameof(value)));
                ApplyOptions();
            }
        }

        /// <summary>A move, home or STOP counts as done only once PNOW stayed unchanged this long (s).</summary>
        public double SettleS
        {
            get => Volatile.Read(ref _settle);
            set
            {
                Volatile.Write(ref _settle, value >= 0 && IsFinite(value) ? value : throw new ArgumentOutOfRangeException(nameof(value)));
                ApplyOptions();
            }
        }

        /// <summary>A push outcome must hold this long, position unchanged, before it is reported (s).</summary>
        public double PushSettleS
        {
            get => Volatile.Read(ref _pushSettle);
            set
            {
                Volatile.Write(ref _pushSettle, value >= 0 && IsFinite(value) ? value : throw new ArgumentOutOfRangeException(nameof(value)));
                ApplyOptions();
            }
        }

        /// <summary>
        /// Take Modbus control (PMSL) automatically before a command when PMSS = 0 and the axis is at rest (default). With
        /// false such commands raise <see cref="EcNotReadyException"/> until <see cref="EnableModbusControl"/> is called.
        /// </summary>
        public bool AutoModbusControl
        {
            get
            {
                lock (_sync)
                {
                    return _autoModbusControl;
                }
            }

            set
            {
                lock (_sync)
                {
                    _autoModbusControl = value;
                }

                ApplyOptions();
            }
        }

        /// <summary>The unit's own soft limits (min, max), once read (<see cref="ReadSoftLimits"/>).</summary>
        public (double Min, double Max)? SoftLimitsMm => State().Pair("soft_limits_mm");

        /// <summary>Where the soft limits came from, or why they are not known.</summary>
        public string SoftLimitsNote => State().String("soft_limits_note") ?? "not read yet";

        /// <summary>The unit's maximum speed (MAXV 7.4, else the 0x0812 default speed), once read.</summary>
        public double? UnitMaxSpeedMmS => State().Number("ec_max_speed_mm_s");

        /// <summary>Where <see cref="UnitMaxSpeedMmS"/> came from.</summary>
        public string UnitMaxSpeedNote => State().String("ec_max_speed_note") ?? "not read yet";

        /// <summary>The unit's class-7 limits (<see cref="ReadAxisLimits"/>), once read.</summary>
        public AxisLimits? AxisLimits => AxisRecords.ToAxisLimits(State().Object("axis_limits"));

        /// <summary>FC11 / CTCL system code, once read.</summary>
        public int? SystemCode => State().Int("system_code");

        /// <summary>FC11 / CTCL unit code, once read.</summary>
        public int? UnitCode => State().Int("unit_code");

        /// <summary>The unit's power-up baud rate from BRSL (item 8.3), once read.</summary>
        public int? BootBaudRate => State().Int("boot_baud_rate");

        /// <summary>The unit's directory records as read so far (empty before the first read).</summary>
        public IReadOnlyList<IndexCategory> Categories => AxisRecords.ToCategories(State().Array("categories"));

        /// <summary>
        /// Start a session: read identity, directory, soft limits and speed limit, then take control. Modbus control (PMSL)
        /// is taken only when <paramref name="takeControl"/> and the axis is at rest; while it moves (e.g. under PIO) control
        /// is left as it is and a warning is logged, because switching PMSL on mid-move stops the move.
        /// </summary>
        public DeviceInfo Connect(bool takeControl = true)
        {
            Run(() => NativeMethods.iad_axis_connect(H, takeControl ? 1 : 0, out _));
            return DeviceInfoRead();
        }

        /// <summary>
        /// Release a STOP coil left ON or a pause this axis holds (best effort: a failure is logged), then close the bus if
        /// this object opened it. What the axis read stays readable.
        /// </summary>
        public void Close()
        {
            try
            {
                ShutdownEvents();
                if (!_core.IsClosed && !Bus.IsClosed)
                {
                    try
                    {
                        Run(() => NativeMethods.iad_axis_release_held(H));
                    }
                    catch (Exception exc) when (IsLinkError(exc))
                    {
                        DriverLog.Warning(LogSource, "could not release the STOP coil on close: " + exc.Message);
                    }
                }
            }
            finally
            {
                if (_ownsBus)
                {
                    Bus.Close();
                }
            }
        }

        /// <inheritdoc />
        public void Dispose() => Close();

        /// <inheritdoc />
        public override string ToString() =>
            $"Axis(slave={Slave}, stroke={FmtRange(Stroke, "mm")}, soft_limits={FmtRange(SoftLimitsMm, "mm")}, strategy={Strategy.Method})";

        // ------------------------------------------------------------------ stroke / caps

        /// <summary>The unit's soft limits minus its 0.30 margin (its nominal stroke), once read.</summary>
        public (double Min, double Max)? UnitStroke
        {
            get
            {
                var soft = SoftLimitsMm;
                if (!soft.HasValue)
                {
                    return null;
                }

                var (lo, hi) = soft.Value;
                var m = Registers.SOFT_LIMIT_MARGIN_MM;
                return hi - lo > (2 * m) + 0.01 ? (Math.Round(lo + m, 2), Math.Round(hi - m, 2)) : (lo, hi);
            }
        }

        /// <summary>
        /// Limits every target is checked against: configured stroke and unit stroke intersected; <c>null</c> while nothing is
        /// known or when the two do not overlap. No I/O.
        /// </summary>
        public (double Min, double Max)? EffectiveStroke
        {
            get
            {
                var cfg = Stroke;
                var unit = UnitStroke;
                if (!cfg.HasValue || !unit.HasValue)
                {
                    return cfg ?? unit;
                }

                var lo = Math.Max(cfg.Value.Min, unit.Value.Min);
                var hi = Math.Min(cfg.Value.Max, unit.Value.Max);
                return lo < hi ? (lo, hi) : ((double, double)?)null;
            }
        }

        /// <summary>Highest speed the driver commands: min(<see cref="MaxSpeedMmS"/>, MAXV) - each when known; 100 while neither is.</summary>
        public double SpeedCapMmS => State().Number("speed_cap_mm_s") ?? FallbackSpeedCap;

        /// <summary>The unit's MAXA (item 7.5), once read.</summary>
        public double? UnitMaxAccelG => AxisLimits?.MaxAccelG;

        /// <summary>The unit's MAXD (item 7.6), once read.</summary>
        public double? UnitMaxDecelG => AxisLimits?.MaxDecelG;

        /// <summary>Highest accel (= decel on a 0x9900 move): min(<see cref="MaxAccelG"/>, MAXA, MAXD); 0.30 while none is known.</summary>
        public double AccelCapG => State().Number("accel_cap_g") ?? FallbackAccelCapG;

        /// <summary>Highest acceleration on a move with its own deceleration: min(<see cref="MaxAccelG"/>, MAXA).</summary>
        public double AccelOnlyCapG => State().Number("accel_only_cap_g") ?? FallbackAccelCapG;

        /// <summary>Highest deceleration on a move with its own deceleration: min(<see cref="MaxAccelG"/>, MAXD).</summary>
        public double DecelCapG => State().Number("decel_cap_g") ?? FallbackAccelCapG;

        /// <summary>Highest push force: min(<see cref="MaxPushPct"/>, MXPP 7.9); 70 while neither is known.</summary>
        public double PushCapPct => State().Number("push_cap_pct") ?? FallbackPushCapPct;

        // ------------------------------------------------------------------ units, kind, family

        /// <summary>"deg" on a rotary axis (AKID 7.33 = 1; without AKID, ATYP 7.7 bit 8), else "mm". Reads the unit's limits once.</summary>
        public string Units => AxisLimits?.Units ?? KindOf("units", "mm");

        /// <summary>Actuator kind in words from AKID ("slider", "rod", "gripper", "rotary" ...; "unknown" when not readable).</summary>
        public string Kind => AxisLimits?.Kind ?? KindOf("kind", "unknown");

        /// <summary>Family of the FC11 system code: "EC", "PCON", "ACON/DCON", "SCON" or "IAI controller".</summary>
        public string ProductFamily => SystemCode is int code ? Registers.ProductFamily(code) : KindJson().String("product_family") ?? Registers.ProductFamily(null);

        /// <summary>The unit is an ELECYLINDER (system 0xEC / 0xED; also when unknown); otherwise a CON-family controller.</summary>
        public bool IsEc => SystemCode is int code ? Registers.IsEcFamily(code) : KindJson().Bool("is_ec") ?? true;

        private string UnitsLabel => AxisLimits?.Units ?? "mm";

        /// <summary>What kind of unit this is (iad_axis_kind_json: reads the limits and the FC11 system code once).</summary>
        private JsonObject KindJson() => JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_kind_json(H, b, s, out r));

        /// <summary><paramref name="name"/> of the unit's kind, read once; <paramref name="fallback"/> when not readable.</summary>
        private string KindOf(string name, string fallback)
        {
            try
            {
                return KindJson().String(name) ?? fallback;
            }
            catch (Exception exc) when (IsLinkError(exc))
            {
                DriverLog.Info(LogSource, "axis limits not readable: " + exc.Message);
                return fallback;
            }
        }

        // ------------------------------------------------------------------ read-only: status, identity, limits

        /// <summary>Read and decode the monitor block (velocity derived from the previous position sample while moving).</summary>
        public Status Status()
        {
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_status(H, out st));
            return Required(st);
        }

        /// <summary>
        /// Read the identity (<see cref="DeviceIdentity.Read"/>: FC11 with the CTCL fallback, directory, identification block,
        /// CON controller name, AXNO) plus the soft limits, the axis limits (MAXV / MAXA / MAXD, AKID, units) and the stored
        /// row count. Parts the unit rejects or does not answer are reported in <see cref="DeviceInfo.Errors"/>; this never
        /// throws on them. READ-ONLY.
        /// </summary>
        public DeviceInfo Identify()
        {
            Run(() => NativeMethods.iad_axis_identify(H, out _));
            return DeviceInfoRead();
        }

        /// <summary>The identity record of the last connect / identify (no I/O).</summary>
        private DeviceInfo DeviceInfoRead() =>
            AxisRecords.ToDeviceInfo(JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_identity_json(H, b, s, out r)));

        /// <summary>
        /// Read the unit's own soft limits (min, max; mm or deg) from items 7.1 / 7.2 by item ID (or the first 4 words of class
        /// 7). <c>null</c> (and the reason in <see cref="SoftLimitsNote"/>) when the unit does not provide plausible limits.
        /// Communication errors other than an exception response propagate.
        /// </summary>
        public (double Min, double Max)? ReadSoftLimits() =>
            JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_soft_limits_json(H, b, s, out r)).Pair("soft_limits_mm");

        /// <summary>
        /// Read the unit's own limits from class 7 by item ID (and SUPT from class 1). READ-ONLY (FC03). MAXV 7.4 (speed
        /// cap), MAXA / MAXD 7.5 / 7.6 (accel / decel caps), MXPH 7.8, MXPP 7.9 (hard push cap), MNPP 7.58 (recommended
        /// minimum push, 20 % when absent), TCPC / TCOA 7.34 / 7.35 (MXPP = 0 or TCPC = 0 means no push - UNVERIFIED), AKID
        /// 7.33 (actuator kind), ATYP 7.7. Also sets <see cref="UnitMaxSpeedMmS"/>: MAXV, or the 0x0812 default speed (item
        /// 8.10) when MAXV is not readable. A class missing or answered with an exception leaves its fields <c>null</c>; a
        /// timeout propagates.
        /// </summary>
        public AxisLimits ReadAxisLimits() =>
            AxisRecords.ToAxisLimits(JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_axis_limits_json(H, b, s, out r)))!;

        /// <summary>
        /// The unit's maximum speed: MAXV, item 7.4 by item ID (<see cref="ReadAxisLimits"/>; VERIFIED 800.00 mm/s on EC #3).
        /// 0x0812 (item 8.10 VCMD) is only the default command speed, read as the fallback. <c>null</c> when neither is plausible.
        /// </summary>
        public double? ReadSpeedLimit()
        {
            var speed = double.NaN;
            Run(() => NativeMethods.iad_axis_read_speed_limit(H, out speed));
            return Known.Double(speed);
        }

        /// <summary>Class 1 support-function words by name (SUPT, DIAG, MVPT, ... SUP3), READ-ONLY; <c>null</c> when not readable.</summary>
        public IReadOnlyDictionary<string, uint>? ReadSupportWords()
        {
            var words = Json((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_support_words_json(H, b, s, out r)) as JsonObject;
            return words?.Keys.ToDictionary(k => k, k => (uint)(words.Long(k) ?? 0));
        }

        /// <summary>
        /// Number of stored rows the unit uses (rows 0 .. n - 1): MAXP (item 3.2: 64 on the PCON) capped by the directory's
        /// row count of the position table; the directory alone when MAXP is not readable; 3 when neither is. An EC always
        /// keeps its three rows 0-2 (backward end, forward end, middle), so an EC reports at least 3. Read once. READ-ONLY.
        /// </summary>
        public int MaxPositions()
        {
            var count = 0;
            Run(() => NativeMethods.iad_axis_max_positions(H, out count));
            return count;
        }

        /// <summary>
        /// Every range the driver enforces on this axis, read from the unit itself. READ-ONLY. Position: the effective
        /// stroke (soft limits 7.1 / 7.2 minus the 0.30 margin, intersected with a configured stroke); speed 0.01 ..
        /// min(MAXV 7.4, configured cap); accel / decel 0.01 .. min(MAXA 7.5 / MAXD 7.6, configured cap); push 1 ..
        /// min(MXPP 7.9, configured cap) % with the MNPP 7.58 recommendation; band; push band; stored rows; jog step; speed
        /// override.
        /// </summary>
        public AxisRanges Limits() =>
            AxisRecords.ToRanges(JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_limits_json(H, b, s, out r)));

        /// <summary>Raw FC03 read from this axis (diagnostics).</summary>
        public ushort[] ReadRegisters(int address, int count) => Bus.ReadRegisters(Slave, address, count);

        /// <summary>Read position-table row <paramref name="n"/> (0 .. <see cref="MaxPositions"/> - 1; on an EC 0 = backward end, 1 = forward end, 2 = middle).</summary>
        public PositionEntry ReadPositionTable(int n)
        {
            var entry = SizedStruct.Fresh<IadPositionEntry>();
            Run(() => NativeMethods.iad_axis_read_position_table(H, n, ref entry));
            return new NativePositionEntry(entry).ToRecord();
        }

        /// <summary>
        /// The unit's alarm history (0x0500 + 6 x k, k = 0..31; index 0 is the newest, consistent with the time stamps read on
        /// hardware). Empty slots (code 0) are skipped unless <paramref name="includeEmpty"/>. Code 0x0FFF is the power-up
        /// marker. READ-ONLY.
        /// </summary>
        public IReadOnlyList<AlarmRecord> ReadAlarmHistory(bool includeEmpty = false) =>
            AxisRecords.ToAlarmRecords(Json((byte[]? b, UIntPtr s, out UIntPtr r) =>
                NativeMethods.iad_axis_read_alarm_history_json(H, includeEmpty ? 1 : 0, b, s, out r)) as List<object?>);

        /// <summary>
        /// One read of the whole monitor block (0x9000, up to 34 words), decoded. READ-ONLY (FC03; class 4 once for the rated
        /// current): position (PNOW), velocity (VNOW, the unit's own coarse measurement), current (CNOW) and current % (CNOW /
        /// MCUR 4.5 x 100), deviation, alarm, uptime, every word by name.
        /// </summary>
        public MonitorReading ReadMonitor() =>
            AxisRecords.ToMonitor(JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_monitor_json(H, b, s, out r)));

        // ------------------------------------------------------------------ helpers

        private static bool IsFinite(double v) => !double.IsNaN(v) && !double.IsInfinity(v);

        private static bool IsLinkError(Exception exc) => exc is ModbusException || exc is System.IO.IOException;

        private static string F2(double v) => v.ToString("F2", Inv);

        private static string FmtRange((double Min, double Max)? r, string units) =>
            r.HasValue ? $"{F2(r.Value.Min)}..{F2(r.Value.Max)} {units}" : "unknown";
    }
}
