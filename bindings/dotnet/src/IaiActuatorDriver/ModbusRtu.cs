using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    /// <summary>Direction of a frame passed to the frame logger.</summary>
    public enum FrameDirection
    {
        /// <summary>A request sent by the driver.</summary>
        Tx,

        /// <summary>A complete response.</summary>
        Rx,

        /// <summary>Bytes received before a timeout or a mismatch (incomplete or foreign frame).</summary>
        RxPartial,
    }

    /// <summary>Response to function 0x45: rejection code (0 = accepted) and status byte.</summary>
    public readonly record struct IaiResponse(int Rejection, int Status);

    /// <summary>Response to function 0x48: 16-bit rejection code (0 = accepted) and four 32-bit status words.</summary>
    public sealed record IaiResponse2(int Rejection, IReadOnlyList<uint> Status);

    /// <summary>Something that reads holding registers of one unit (FC03): <see cref="ModbusRtu.ForSlave"/>.</summary>
    public interface IRegisterReader
    {
        /// <summary>Read <paramref name="count"/> holding registers from <paramref name="address"/>.</summary>
        ushort[] ReadRegisters(int address, int count);
    }

    /// <summary>
    /// Thread-safe Modbus RTU master for IAI units over an <see cref="ITransport"/> (IAI manual ME0162: CRC-16/MODBUS low
    /// byte first, big-endian data). Besides the standard function codes 0x03 / 0x05 / 0x06 / 0x10 / 0x11 / 0x17 it speaks
    /// IAI's control-command functions 0x45 (ON-WIRE and VERIFIED on an EC) and 0x48 (UNVERIFIED: no unit tested uses it).
    /// </summary>
    /// <remarks>
    /// <list type="bullet">
    /// <item><description>The port is opened once and kept open; the baud rate is changed in place.</description></item>
    /// <item><description>One transaction at a time (a lock), at least <see cref="MinGap"/> (default 5 ms) of silence
    /// between the end of one transaction and the next request. On hardware the read sent right after a mid-move write
    /// once got no reply, so the gap and the retries are required, not optional.</description></item>
    /// <item><description>Timeouts, CRC errors and mismatched frames are retried (<see cref="Retries"/>, default 3);
    /// exception responses and IAI rejections are valid answers and never retried. Pass <c>retries: 0</c> for a command
    /// that must not be sent twice.</description></item>
    /// <item><description>Several axes can share one line (slave = axis number + 1; RCON gateway / SIO daisy chain,
    /// UNVERIFIED on hardware): one <see cref="ModbusRtu"/> per port, its lock serialises the axes' transactions.</description></item>
    /// <item><description>Broadcast (slave 0) is only used by <see cref="BroadcastIaiCommand"/> (the IAI baud change,
    /// ON-WIRE: <c>00 45 00 04 00 &lt;code&gt; 00 05</c>); a broadcast is never answered.</description></item>
    /// <item><description>The requests run on the native core's bus (iad_bus, the same master the Python edition and the
    /// C++ core use); a transport other than <see cref="SerialTransport"/> reaches the core through callbacks and must not
    /// call the driver from them. The core's log lines reach <see cref="DriverLog"/> with its own sources
    /// ("iai_actuator_driver.modbus").</description></item>
    /// </list>
    /// </remarks>
    public sealed class ModbusRtu : IDisposable
    {
        /// <summary>FC03 read holding registers.</summary>
        public const byte FcReadHoldingRegisters = 0x03;

        /// <summary>FC05 write single coil.</summary>
        public const byte FcWriteCoil = 0x05;

        /// <summary>FC06 write single register.</summary>
        public const byte FcWriteRegister = 0x06;

        /// <summary>FC10 write multiple registers.</summary>
        public const byte FcWriteRegisters = 0x10;

        /// <summary>FC11 report slave ID.</summary>
        public const byte FcReportSlaveId = 0x11;

        /// <summary>FC17 read / write multiple registers.</summary>
        public const byte FcReadWriteRegisters = 0x17;

        /// <summary>IAI control command (0x45): <c>[group u8][cmd u8][op1 u16][op2 u16]</c>.</summary>
        public const byte FcIaiCommand = 0x45;

        /// <summary>IAI control command 2 (0x48): <c>[group u8][cmd u16][op1..op4 u32]</c>. UNVERIFIED.</summary>
        public const byte FcIaiCommand2 = 0x48;

        /// <summary>Largest single FC03 read this driver issues; bigger reads are split (a conservative size).</summary>
        public const int MaxReadRegisters = 60;

        /// <summary>An FC10 frame must fit the controller's 256-byte receive buffer (DOC, ME0162 p.5-4).</summary>
        public const int MaxWriteRegisters = 123;

        /// <summary>Largest write part of an FC17 frame.</summary>
        public const int MaxReadWriteWriteRegisters = 60;

        /// <summary>FC05 value for ON.</summary>
        public const ushort CoilOn = 0xFF00;

        /// <summary>FC05 value for OFF.</summary>
        public const ushort CoilOff = 0x0000;

        /// <summary>Rate a port is opened at when none is given (IA-OS switches ECs to it; volatile).</summary>
        public const int DefaultBaudRate = 115200;

        /// <summary>Default retries after a timeout / CRC error / mismatched frame (IAI recommends 3).</summary>
        public const int DefaultRetries = 3;

        private const string LogSource = "IaiActuatorDriver.ModbusRtu";

        /// <summary>Most retries the core's bus takes per request (iad_bus_set_retries 0..100).</summary>
        private const int MaxCoreRetries = 100;

        private readonly object _sync = new object();
        private readonly NativeBus _bus = null!;
        private readonly TransportBridge? _bridge;
        private int _retries;
        private TimeSpan _minGap;
        private bool _closed;

        /// <summary>Create a master on <paramref name="transport"/> (which it then owns: <see cref="Close"/> closes it).</summary>
        /// <param name="transport">The byte pipe (e.g. <see cref="SerialTransport"/>).</param>
        /// <param name="retries">Extra attempts after a timeout / CRC error / mismatched frame (default 3).</param>
        /// <param name="minGap">Minimum silence between transactions (default 5 ms).</param>
        /// <param name="frameLogger">Optional (direction, frame) callback for debugging and captures.</param>
        /// <exception cref="DllNotFoundException">The native core is missing (the message says where it was looked for).</exception>
        public ModbusRtu(ITransport transport, int retries = DefaultRetries, TimeSpan? minGap = null, Action<FrameDirection, byte[]>? frameLogger = null)
        {
            Transport = transport ?? throw new ArgumentNullException(nameof(transport));
            Retries = retries;
            MinGap = minGap ?? DefaultMinGap;
            FrameLogger = frameLogger;

            // The bus is the core's (iad_bus): a serial port hands over its native transport, any other transport reaches
            // the core through callbacks (TransportBridge). The bus shares the transport, so our handle is released here.
            NativeDriver.EnsureLoaded();
            NativeSinks.ForwardToDriverLog();
            var serial = transport as SerialTransport;
            var handle = serial?.Handle ?? TransportBridge.Open(transport, null, forwardClose: false, out _bridge);
            try
            {
                _bus = NativeBus.Open(handle, Math.Min(_retries, MaxCoreRetries), _minGap.TotalSeconds);
            }
            finally
            {
                if (serial is null)
                {
                    handle.Dispose();
                }
            }

            var self = new WeakReference<ModbusRtu>(this);
            _bus.SetFrameSink((direction, frame) =>
            {
                if (self.TryGetTarget(out var bus))
                {
                    bus.Log(direction, frame);
                }
            });
        }

        /// <summary>Default minimum silence between transactions.</summary>
        public static TimeSpan DefaultMinGap { get; } = TimeSpan.FromMilliseconds(5);

        /// <summary>
        /// How long <see cref="ReportSlaveId"/> waits for the answer by default. VERIFIED: the EC-R6S-150 rod answered FC11
        /// only after 0.2-0.4 s (no reply within 0.2 s), the other units at once.
        /// </summary>
        public static TimeSpan SlaveIdTimeout { get; } = TimeSpan.FromSeconds(0.65);

        /// <summary>
        /// How long <see cref="Probe"/> waits for an answer (one FC03 read, no retries). An answering unit replies within a
        /// few ms at 9600..230400 baud, so 50 ms per address keeps a 16-address scan at about 1 s.
        /// </summary>
        public static TimeSpan ScanTimeout { get; } = TimeSpan.FromMilliseconds(50);

        /// <summary>How long the bus stays silent after a broadcast by default (ON-WIRE: IA-OS waited about 0.2 s).</summary>
        public static TimeSpan DefaultBroadcastSettle { get; } = TimeSpan.FromSeconds(0.2);

        /// <summary>
        /// IAI 0x45 commands allowed without <c>allowUnsafe</c> (IAI's control-command numbering). 4 / 5 / 9 / 0x0C are
        /// ON-WIRE; 5 / 9 / 0x0A / 0x0B / 0x0C are VERIFIED on an EC (6 only on a PCON controller); the others UNVERIFIED.
        /// </summary>
        public static IReadOnlyDictionary<int, string> IaiSafeCommands { get; } = Ro(new Dictionary<int, string>
        {
            [0x05] = "reset (alarm reset only: op1 must be 0)",
            [0x06] = "move to position index with override",
            [0x07] = "move to position index with speed",
            [0x09] = "servo on/off",
            [0x0A] = "return home",
            [0x0B] = "decel stop",
            [0x0C] = "change action mode",
            [0x0D] = "change speed while moving",
            [0x0E] = "change acceleration while moving",
        });

        /// <summary>IAI 0x48 commands allowed without <c>allowUnsafe</c> (16-bit codes; UNVERIFIED).</summary>
        public static IReadOnlyDictionary<int, string> IaiSafeCommands2 { get; } = Ro(new Dictionary<int, string>
        {
            [0x22] = "change action mode",
            [0x23] = "alarm reset",
            [0x30] = "servo on/off",
            [0x31] = "return home",
            [0x32] = "move to position index with override",
            [0x33] = "move to position index with speed",
            [0x34] = "decel stop",
            [0x35] = "change speed while moving",
            [0x36] = "change acceleration while moving",
        });

        private static readonly int[] BaudCodeOrder = { 9600, 19200, 38400, 57600, 115200, 230400, 14400, 28800, 76800 };

        /// <summary>
        /// op1 of the IAI baud-rate change (0x45 cmd 4) per rate. Only 0x14 (115200, ON-WIRE + VERIFIED) and 0x15 (230400,
        /// VERIFIED) are confirmed on every unit; <b>0x06 ("38400") depends on the unit</b> (the EC-GRBP8M gripper and the
        /// PCON-CB take it, the EC-R6S rod ignores it), so this table is not every unit's for every code (see
        /// <see cref="VerifiedBaudRates"/> and <see cref="UnitDependentBaudRates"/>). 14400 / 28800 / 76800 are never probed at connect.
        /// </summary>
        public static IReadOnlyDictionary<int, int> IaiBaudCodes { get; } = Ro(new Dictionary<int, int>
        {
            [9600] = 0x04, // UNVERIFIED
            [19200] = 0x05, // UNVERIFIED
            [38400] = 0x06, // unit-dependent: VERIFIED on the EC-GRBP8M gripper and the PCON-CB, IGNORED by the EC-R6S rod
            [57600] = 0x13, // UNVERIFIED
            [115200] = 0x14, // ON-WIRE + VERIFIED: EC #3 direct, both ways
            [230400] = 0x15, // VERIFIED: EC #3 direct
            [14400] = 0x11, // UNVERIFIED
            [28800] = 0x12, // UNVERIFIED
            [76800] = 0x07, // UNVERIFIED
        });

        /// <summary>IAI baud code -> rate (e.g. BRSL, item 8.3: 6 -> 38400).</summary>
        public static IReadOnlyDictionary<int, int> BaudByCode { get; } =
            Ro(BaudCodeOrder.ToDictionary(rate => IaiBaudCodes[rate], rate => rate));

        /// <summary>
        /// Rates whose baud-change code was verified on hardware (the EC-R6S rod, sent direct at the current rate).
        /// <see cref="Axis.SetBaudRate"/> sends any other rate with an IAI code with a warning (UNVERIFIED code).
        /// </summary>
        public static IReadOnlyList<int> VerifiedBaudRates { get; } = Array.AsReadOnly(new[] { 115200, 230400 });

        /// <summary>
        /// Rates whose code some units take and others ignore: 0x06 (38400) switched the EC-GRBP8M gripper (VERIFIED, sent
        /// direct from 230400 and from 9600) and the PCON-CB (VERIFIED, from 230400) and was IGNORED by the EC-R6S rod.
        /// <see cref="Axis.SetBaudRate"/> sends them without the opt-in because it checks the result: a unit that ignores the
        /// code keeps answering at its rate, and the port is left there (<see cref="VerificationException"/>).
        /// </summary>
        public static IReadOnlyList<int> UnitDependentBaudRates { get; } = Array.AsReadOnly(new[] { 38400 });

        /// <summary>
        /// Rates <see cref="DetectBaudRate(int, IEnumerable{int}?, int, bool, TimeSpan?)"/> tries after the current one.
        /// VERIFIED: all three test ECs boot at 38400 (BRSL, item 8.3 = code 6) and IA-OS switches them to 115200
        /// (volatile, ON-WIRE).
        /// </summary>
        public static IReadOnlyList<int> BaudDetectOrder { get; } = Array.AsReadOnly(new[] { 115200, 38400, 9600, 230400, 57600, 19200 });

        /// <summary>
        /// Every rate an IAI baud code names, <see cref="BaudDetectOrder"/> first: what to probe when a unit does not
        /// answer at a new rate (a unit whose code table differs may have gone to any of them).
        /// </summary>
        public static IReadOnlyList<int> BaudProbeAll { get; } =
            Array.AsReadOnly(BaudDetectOrder.Concat(BaudCodeOrder.Where(r => !BaudDetectOrder.Contains(r))).ToArray());

        /// <summary>The transport this master owns.</summary>
        public ITransport Transport { get; }

        /// <summary>The core's bus under this master (an <see cref="Axis"/> opens the core's axis on it).</summary>
        internal NativeBus Native => _bus;

        /// <summary>The callbacks of a transport the core does not know (null for a serial port): what it threw, its timeout.</summary>
        internal TransportBridge? Bridge => _bridge;

        /// <summary>
        /// A call of the core's port on this master's bus (a scan, the baud detection, the line-wide baud change; <see cref="Port"/>):
        /// under this master's lock, refused when closed, the core's rate brought in line with the transport's first.
        /// </summary>
        internal T OnPort<T>(Func<NativeBus, T> call)
        {
            lock (_sync)
            {
                ThrowIfClosed();
                SyncRate();
                SyncReadTimeout();
                return call(_bus);
            }
        }

        /// <summary>
        /// Before the core's cell scans this master's bus (<see cref="Cell"/>): under its lock, refused when closed, the
        /// core's read timeout brought in line with the transport's (a probe's wait caps it).
        /// </summary>
        internal void PrepareScan()
        {
            lock (_sync)
            {
                ThrowIfClosed();
                SyncReadTimeout();
            }
        }

        /// <summary>Extra attempts after a timeout / CRC error / mismatched frame.</summary>
        public int Retries
        {
            get => _retries;
            set
            {
                if (value < 0)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "retries must be >= 0");
                }

                lock (_sync)
                {
                    _retries = value;
                    if (_bus != null && !_closed)
                    {
                        _bus.SetRetries(Math.Min(value, MaxCoreRetries));
                    }
                }
            }
        }

        /// <summary>Minimum silence between the end of one transaction and the next request.</summary>
        public TimeSpan MinGap
        {
            get => _minGap;
            set
            {
                if (value < TimeSpan.Zero)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "min gap must be >= 0");
                }

                lock (_sync)
                {
                    _minGap = value;
                    if (_bus != null && !_closed)
                    {
                        _bus.SetMinGap(value.TotalSeconds);
                    }
                }
            }
        }

        /// <summary>Optional (direction, frame) callback. An exception it throws is logged and ignored.</summary>
        public Action<FrameDirection, byte[]>? FrameLogger { get; set; }

        /// <summary>The transport's baud rate, or <c>null</c> if it has none (an in-memory fake).</summary>
        public int? BaudRate
        {
            get
            {
                var rate = Transport.BaudRate;
                return rate > 0 ? rate : (int?)null;
            }
        }

        /// <summary>True if the transport has a baud rate that can be changed in place.</summary>
        public bool CanChangeBaudRate => BaudRate.HasValue;

        /// <summary>True once <see cref="Close"/> was called.</summary>
        public bool IsClosed
        {
            get
            {
                lock (_sync)
                {
                    return _closed;
                }
            }
        }

        // ------------------------------------------------------------------ lifecycle

        /// <summary>Close the transport. Safe to call twice.</summary>
        public void Close()
        {
            lock (_sync)
            {
                if (_closed)
                {
                    return;
                }

                _closed = true;
                try
                {
                    _bus.Dispose();  // the core closes its bus (the frame sink first); a bridged transport is closed below
                }
                finally
                {
                    Transport.Close();
                }
            }
        }

        /// <inheritdoc />
        public void Dispose() => Close();

        /// <summary>A register reader bound to one slave address (for the directory reader and other read-only helpers).</summary>
        public IRegisterReader ForSlave(int slave)
        {
            CheckSlave(slave);
            return new SlaveReader(this, slave);
        }

        // ------------------------------------------------------------------ baud rate

        /// <summary>Change the transport's baud rate in place (the port is never closed).</summary>
        /// <exception cref="NotSupportedException">The transport has no baud rate.</exception>
        public void SetBaudRate(int baudRate)
        {
            if (baudRate <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(baudRate));
            }

            lock (_sync)
            {
                ThrowIfClosed();
                RequireBaudRate("change");
                SyncRate();
                Core(() => _bus.SetBaudRate(baudRate));
            }
        }

        /// <summary>
        /// Hold the line in BREAK for <paramref name="duration"/> (0 &lt; duration &lt;= 2 s; 150 ms or more drops a unit to
        /// 9600 baud, DOC). Nothing else is sent meanwhile (the bus lock is held).
        /// </summary>
        public void SendBreak(TimeSpan duration)
        {
            if (duration <= TimeSpan.Zero || duration > TimeSpan.FromSeconds(2))
            {
                throw new ArgumentOutOfRangeException(nameof(duration), "break duration must be in (0, 2] s");
            }

            lock (_sync)
            {
                ThrowIfClosed();
                Core(() => _bus.SendBreak(duration.TotalSeconds));
            }
        }

        /// <summary>Find the baud rate <paramref name="slave"/> answers at and leave the transport set to it.</summary>
        /// <inheritdoc cref="DetectBaudRate(IEnumerable{int}, IEnumerable{int}?, int, bool, TimeSpan?)"/>
        public int DetectBaudRate(int slave, IEnumerable<int>? candidates = null, int probeAddress = 0x9000, bool currentFirst = true, TimeSpan? probeTimeout = null) =>
            DetectBaudRate(new[] { slave }, candidates, probeAddress, currentFirst, probeTimeout);

        /// <summary>
        /// Find the baud rate one of <paramref name="slaves"/> answers at and leave the transport set to it.
        /// </summary>
        /// <remarks>
        /// Tries the current rate first (unless <paramref name="currentFirst"/> is false: then <paramref name="candidates"/>
        /// in their order, the current rate where it appears or last), with one FC03 read of
        /// <paramref name="probeAddress"/> per address and rate and no retries. At each rate the addresses are asked in
        /// order and the first answer settles the rate; an exception response counts as an answer (a valid frame proves
        /// the rate). The rate is changed in place: the port is never closed or reopened (that, or a long line break,
        /// would drop an EC back to its power-up rate). <paramref name="probeTimeout"/> (default: the transport's timeout
        /// for one address, <see cref="ScanTimeout"/> for several) caps the wait per address. The bus lock is held
        /// throughout.
        /// </remarks>
        /// <exception cref="ModbusTimeoutException">No answer at any rate (the starting rate is restored).</exception>
        /// <exception cref="NotSupportedException">The transport has no baud rate.</exception>
        public int DetectBaudRate(IEnumerable<int> slaves, IEnumerable<int>? candidates = null, int probeAddress = 0x9000, bool currentFirst = true, TimeSpan? probeTimeout = null)
        {
            var addresses = (slaves ?? throw new ArgumentNullException(nameof(slaves))).ToList();
            if (addresses.Count == 0)
            {
                throw new ArgumentException("no slave address to probe", nameof(slaves));
            }

            foreach (var address in addresses)
            {
                CheckSlave(address);
            }

            CheckU16(nameof(probeAddress), probeAddress);
            if (probeTimeout.HasValue && probeTimeout.Value <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(probeTimeout), "probe timeout must be > 0");
            }

            var rates = candidates?.ToList();
            lock (_sync)
            {
                ThrowIfClosed();
                RequireBaudRate("detect");
                SyncRate();
                SyncReadTimeout();

                // the core's detection over targets (the same rule: one read per address and rate, the bus lock held throughout)
                return Core(() => _bus.DetectBaudRate(addresses, rates, probeAddress, currentFirst, probeTimeout?.TotalSeconds));
            }
        }

        /// <summary>
        /// Ask <paramref name="slave"/> once (FC03, one register at <paramref name="address"/>) and return true if a valid
        /// frame came back. No retries and at most <paramref name="timeout"/> (default <see cref="ScanTimeout"/>) of
        /// waiting (the transport's read timeout is shortened for this one request), so a missing address costs about
        /// <paramref name="timeout"/>. An exception response counts as an answer; a reply from another address, a bad
        /// CRC or silence is false. READ-ONLY.
        /// </summary>
        public bool Probe(int slave, TimeSpan? timeout = null, int address = 0x9000)
        {
            CheckSlave(slave);
            CheckU16(nameof(address), address);
            var wait = timeout ?? ScanTimeout;
            if (wait <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(timeout), "probe timeout must be > 0");
            }

            lock (_sync)
            {
                ThrowIfClosed();
                SyncReadTimeout();
                return Core(() => _bus.Probe(slave, wait.TotalSeconds, address));  // the core caps the wait (a managed transport's timeout too)
            }
        }

        // ------------------------------------------------------------------ public requests

        /// <summary>
        /// FC03: read <paramref name="count"/> holding registers starting at <paramref name="address"/>. Reads larger than
        /// <see cref="MaxReadRegisters"/> are split. Do not span IAI register categories (e.g. 0x9000 and 0x9800) in one
        /// call. <paramref name="retries"/> overrides <see cref="Retries"/> for this call.
        /// </summary>
        public ushort[] ReadRegisters(int slave, int address, int count, int? retries = null)
        {
            CheckSlave(slave);
            CheckU16(nameof(address), address);
            if (count < 1 || address + count > 0x10000)
            {
                throw new ArgumentOutOfRangeException(nameof(count), count, $"invalid register count {count} at 0x{address:X4}");
            }

            var extra = CheckRetries(retries);
            lock (_sync)
            {
                ThrowIfClosed();
                return Core(() => _bus.ReadRegisters(slave, address, count, extra));
            }
        }

        /// <summary>FC05: force a single coil ON (FF00h) or OFF (0000h).</summary>
        public void WriteCoil(int slave, int address, bool on)
        {
            CheckSlave(slave);
            CheckU16(nameof(address), address);
            lock (_sync)
            {
                ThrowIfClosed();
                Core(() => _bus.WriteCoil(slave, address, on, Math.Min(_retries, MaxCoreRetries)));
            }
        }

        /// <summary>FC06: write one holding register.</summary>
        public void WriteRegister(int slave, int address, ushort value)
        {
            CheckSlave(slave);
            CheckU16(nameof(address), address);
            lock (_sync)
            {
                ThrowIfClosed();
                Core(() => _bus.WriteRegister(slave, address, value));
            }
        }

        /// <summary>FC10: write 1..<see cref="MaxWriteRegisters"/> consecutive holding registers in one frame.</summary>
        public void WriteRegisters(int slave, int address, IReadOnlyList<ushort> values)
        {
            CheckSlave(slave);
            CheckU16(nameof(address), address);
            if (values is null)
            {
                throw new ArgumentNullException(nameof(values));
            }

            var n = values.Count;
            if (n < 1 || n > MaxWriteRegisters)
            {
                throw new ArgumentOutOfRangeException(nameof(values), $"can write 1..{MaxWriteRegisters} registers per frame, got {n}");
            }

            var words = values.ToArray();
            lock (_sync)
            {
                ThrowIfClosed();
                Core(() => _bus.WriteRegisters(slave, address, words));
            }
        }

        /// <summary>FC17: write <paramref name="values"/> at <paramref name="writeAddress"/>, then read <paramref name="readCount"/> registers, in one frame.</summary>
        public ushort[] ReadWriteRegisters(int slave, int readAddress, int readCount, int writeAddress, IReadOnlyList<ushort> values)
        {
            CheckSlave(slave);
            CheckU16(nameof(readAddress), readAddress);
            CheckU16(nameof(writeAddress), writeAddress);
            if (readCount < 1 || readCount > MaxReadRegisters)
            {
                throw new ArgumentOutOfRangeException(nameof(readCount), $"read count must be 1..{MaxReadRegisters}");
            }

            if (values is null)
            {
                throw new ArgumentNullException(nameof(values));
            }

            var n = values.Count;
            if (n < 1 || n > MaxReadWriteWriteRegisters)
            {
                throw new ArgumentOutOfRangeException(nameof(values), $"can write 1..{MaxReadWriteWriteRegisters} registers with FC17");
            }

            var words = values.ToArray();
            lock (_sync)
            {
                ThrowIfClosed();
                return Core(() => _bus.ReadWriteRegisters(slave, readAddress, readCount, writeAddress, words));
            }
        }

        /// <summary>
        /// FC11: return the data bytes after the byte count. For an EC (VERIFIED): <c>[system code, run status, unit code,
        /// firmware hi, firmware lo]</c>; system code 0xEC = EC 24 V. <paramref name="timeout"/> (default
        /// <see cref="SlaveIdTimeout"/>, 0.65 s) is how long each attempt waits for the answer when that is longer than the
        /// transport's own timeout (the EC-R6S rod answers after 0.2-0.4 s); <see cref="TimeSpan.Zero"/> = the transport's
        /// timeout only.
        /// </summary>
        public byte[] ReportSlaveId(int slave, int? retries = null, TimeSpan? timeout = null)
        {
            CheckSlave(slave);
            var extra = CheckRetries(retries);
            var wait = timeout ?? SlaveIdTimeout;
            if (wait < TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(timeout), "timeout must be >= 0");
            }

            lock (_sync)
            {
                ThrowIfClosed();
                return Core(() => _bus.ReportSlaveId(slave, extra, wait > TimeSpan.Zero ? wait.TotalSeconds : (double?)null));
            }
        }

        /// <summary>
        /// IAI function 0x45 "ControlCommand": <c>[group u8][cmd u8][op1 u16][op2 u16]</c>, reply
        /// <c>[slave] 45 [rejection] [status]</c> (ON-WIRE and VERIFIED).
        /// </summary>
        /// <remarks>
        /// Only commands in <see cref="IaiSafeCommands"/>, addressed to ID group 0, are sent unless
        /// <paramref name="allowUnsafe"/>; commands on <see cref="Registers.IAI_NEVER_SEND"/> only with
        /// <paramref name="allowNeverSend"/> (<see cref="UnsafeCommandException"/>, nothing sent). With
        /// <paramref name="check"/> a non-zero rejection raises <see cref="IaiCommandRejectedException"/> naming it; a
        /// rejection is a valid answer and is never retried. <paramref name="retries"/> (timeouts / CRC errors only): pass
        /// 0 for a command that must not be sent twice.
        /// </remarks>
        public IaiResponse IaiCommand(int slave, int group, int command, int op1 = 0, int op2 = 0, bool allowUnsafe = false, bool allowNeverSend = false, bool check = true, int? retries = null)
        {
            CheckSlave(slave);
            IaiPdu(group, command, op1, op2, allowUnsafe, allowNeverSend);  // the refusals in this library's words, nothing sent
            var extra = CheckRetries(retries);
            IaiResponse result;
            lock (_sync)
            {
                ThrowIfClosed();
                var (rejection, status) = Core(() => _bus.IaiCommand(slave, group, command, op1, op2, IaiFlags(allowUnsafe, allowNeverSend), extra));
                result = new IaiResponse(rejection, status);
            }

            if (check && result.Rejection != 0)
            {
                throw new IaiCommandRejectedException(FcIaiCommand, command, result.Rejection, statusByte: result.Status);
            }

            return result;
        }

        /// <summary>
        /// Send IAI function 0x45 to the broadcast address 0 (every slave on the line acts, none answers). Used only for
        /// the baud-rate change, in the frame IA-OS sends (ON-WIRE: <c>00 45 00 04 00 14 00 05</c> = 115200, then IA-OS
        /// waited about 0.2 s before switching its port). The bus lock is held for <paramref name="settle"/> (default
        /// 0.2 s) after the frame. The same safe-list / never-send rules as <see cref="IaiCommand"/> apply (the baud change,
        /// cmd 4, is on the never-send list). Returns the frame sent.
        /// </summary>
        public byte[] BroadcastIaiCommand(int group, int command, int op1 = 0, int op2 = 0, bool allowUnsafe = false, bool allowNeverSend = false, TimeSpan? settle = null)
        {
            var wait = settle ?? DefaultBroadcastSettle;
            if (wait < TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(settle), "settle must be >= 0");
            }

            IaiPdu(group, command, op1, op2, allowUnsafe, allowNeverSend);  // the refusals in this library's words, nothing sent
            lock (_sync)
            {
                ThrowIfClosed();
                return Core(() => _bus.BroadcastIaiCommand(group, command, op1, op2, IaiFlags(allowUnsafe, allowNeverSend), wait.TotalSeconds));
            }
        }

        /// <summary>
        /// IAI function 0x48 "ControlCommand2": <c>[group u8][cmd u16][op1..op4 u32]</c> (19 data bytes; missing operands
        /// are sent as 0), e.g. <c>01 48 00 00 40</c> + 16 x <c>00</c> + <c>D5 98</c>. Reply <c>[slave] 48 [rejection u16]
        /// [status1..4 u32]</c>. UNVERIFIED: 0x48 is the command set of units whose SUP3 IVC1 bit is set, and none of the
        /// ECs tested has it (they use 0x45). The 16-bit code must be in <see cref="IaiSafeCommands2"/> and
        /// <paramref name="group"/> 0 unless <paramref name="allowUnsafe"/>; codes on <see cref="Registers.IAI2_NEVER_SEND"/>
        /// need <paramref name="allowNeverSend"/>. A rejection (0x1000-0x13FF) raises
        /// <see cref="IaiCommandRejectedException"/> and is never retried.
        /// </summary>
        public IaiResponse2 IaiCommand2(int slave, int command, IReadOnlyList<uint>? operands = null, bool allowUnsafe = false, bool allowNeverSend = false, bool check = true, int group = 0)
        {
            CheckU16(nameof(command), command);
            if (group < 0 || group > 0xFF)
            {
                throw new ArgumentOutOfRangeException(nameof(group), group, "group must be 0..255");
            }

            var ops = operands ?? Array.Empty<uint>();
            if (ops.Count > 4)
            {
                throw new ArgumentException("at most 4 operands", nameof(operands));
            }

            if (Registers.IAI2_NEVER_SEND.TryGetValue(command, out var what) && !allowNeverSend)
            {
                throw new UnsafeCommandException(
                    $"IAI command 0x48/0x{command:x4} ({what}) is on the never-send list; nothing sent (pass allowNeverSend only on the user's explicit request)");
            }

            if (!allowUnsafe)
            {
                if (group != 0)
                {
                    throw new UnsafeCommandException($"IAI command 0x48 to ID group 0x{group:x2}: only group 0 is allowed without allowUnsafe");
                }

                if (!IaiSafeCommands2.ContainsKey(command))
                {
                    throw new UnsafeCommandException(
                        $"IAI command 0x48/0x{command:x4} is not on the safe list; pass allowUnsafe only if you know exactly what it does");
                }
            }

            CheckSlave(slave);
            var words = ops.ToArray();
            int rejection;
            uint[] status;
            lock (_sync)
            {
                ThrowIfClosed();
                (rejection, status) = Core(() => _bus.IaiCommand2(slave, group, command, words, IaiFlags(allowUnsafe, allowNeverSend), Math.Min(_retries, MaxCoreRetries)));
            }

            var result = new IaiResponse2(rejection, Array.AsReadOnly(status));
            if (check && rejection != 0)
            {
                throw new IaiCommandRejectedException(FcIaiCommand2, command, rejection, statusWords: result.Status);
            }

            return result;
        }

        // ------------------------------------------------------------------ internals

        internal static byte[] IaiPdu(int group, int command, int op1, int op2, bool allowUnsafe, bool allowNeverSend)
        {
            if (group < 0 || group > 0xFF || command < 0 || command > 0xFF)
            {
                throw new ArgumentOutOfRangeException(nameof(command), "group and command must be 0..255");
            }

            CheckU16(nameof(op1), op1);
            CheckU16(nameof(op2), op2);
            if (Registers.IAI_NEVER_SEND.TryGetValue(command, out var what) && !allowNeverSend)
            {
                throw new UnsafeCommandException(
                    $"IAI command 0x45/0x{command:x2} ({what}) is on the never-send list; nothing sent (pass allowNeverSend only on the user's explicit request)");
            }

            if (!allowUnsafe)
            {
                if (group != 0)
                {
                    throw new UnsafeCommandException($"IAI command 0x45 to ID group 0x{group:x2}: only group 0 is allowed without allowUnsafe");
                }

                if (!IaiSafeCommands.ContainsKey(command))
                {
                    throw new UnsafeCommandException(
                        $"IAI command 0x45/0x{command:x2} is not on the safe list; pass allowUnsafe only if you know exactly what it does");
                }

                if (command == 0x05 && op1 != 0)
                {
                    throw new UnsafeCommandException("0x45/0x05 with op1 != 0 is a software reset; pass allowUnsafe");
                }
            }

            return new[] { FcIaiCommand, (byte)group, (byte)command, (byte)(op1 >> 8), (byte)op1, (byte)(op2 >> 8), (byte)op2 };
        }

        /// <summary>The IAD_IAI_* opt-ins; a rejection always comes back (this class raises it with its own tables).</summary>
        private static uint IaiFlags(bool allowUnsafe, bool allowNeverSend) =>
            NativeMethods.IaiNoCheck
            | (allowUnsafe ? NativeMethods.IaiAllowUnsafe : 0u)
            | (allowNeverSend ? NativeMethods.IaiAllowNeverSend : 0u);

        /// <summary>
        /// One call into the core's bus, under this master's lock (the seam of every request; <see cref="NativeBus"/> raises
        /// the core's error or what a managed transport threw, <see cref="TransportBridge.Raise"/>).
        /// </summary>
        private static T Core<T>(Func<T> call) => call();

        private static void Core(Action call) => call();

        /// <summary>A frame of the core's bus to <see cref="FrameLogger"/> (a copy); an exception it throws is logged and ignored.</summary>
        private void Log(FrameDirection direction, byte[] frame)
        {
            var logger = FrameLogger;
            if (logger is null)
            {
                return;
            }

            try
            {
                logger(direction, (byte[])frame.Clone());
            }
            catch (Exception exc)
            {
                DriverLog.Error(LogSource, "frame logger raised", exc); // a broken logger must never break communication
            }
        }

        private void ThrowIfClosed()
        {
            if (_closed)
            {
                throw new ModbusException("bus is closed");
            }
        }

        private int RequireBaudRate(string what)
        {
            var rate = Transport.BaudRate;
            if (rate <= 0)
            {
                throw new NotSupportedException($"{Transport} has no baud rate to {what}");
            }

            return rate;
        }

        /// <summary>
        /// Bring the core's idea of the rate in line with the transport's, when the rate was changed outside this master
        /// (directly on the transport): the core starts detection and the slow-rate waits from its own value.
        /// </summary>
        private void SyncRate()
        {
            var rate = Transport.BaudRate;
            if (rate > 0 && rate != CoreRate())
            {
                Core(() => _bus.SetBaudRate(rate));
            }
        }

        /// <summary>
        /// A managed transport's read timeout changed since the core was given it (a caller raised it): hand the core the
        /// one it has now, so that a probe's or a scan's shorter wait caps it (the core shortens only below the timeout it
        /// holds). No I/O.
        /// </summary>
        private void SyncReadTimeout()
        {
            if (_bridge?.TimeoutToSync() is double seconds)
            {
                using var transport = _bus.Transport();
                NativeErrors.Check(NativeMethods.iad_transport_set_read_timeout(transport, seconds));
            }
        }

        /// <summary>The rate the core's transport holds (no I/O).</summary>
        private int CoreRate() => (MiniJson.TryParse(_bus.InfoJson()) as JsonObject)?.Int("baud_rate") ?? 0;

        private int CheckRetries(int? retries)
        {
            if (retries.HasValue && retries.Value < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(retries), "retries must be >= 0");
            }

            return Math.Min(retries ?? _retries, MaxCoreRetries);
        }

        private static void CheckSlave(int slave)
        {
            if (slave < 1 || slave > 247)
            {
                throw new ArgumentOutOfRangeException(nameof(slave), slave, "slave address must be 1..247 (broadcast not supported)");
            }
        }

        internal static void CheckU16(string name, int value)
        {
            if (value < 0 || value > 0xFFFF)
            {
                throw new ArgumentOutOfRangeException(name, value, $"{name} must be 0..65535");
            }
        }

        internal static string HexText(IEnumerable<byte> bytes) =>
            string.Join(" ", bytes.Select(b => b.ToString("x2", CultureInfo.InvariantCulture)));

        private static IReadOnlyDictionary<TKey, TValue> Ro<TKey, TValue>(IDictionary<TKey, TValue> d)
            where TKey : notnull => new ReadOnlyDictionary<TKey, TValue>(d);

        private sealed class SlaveReader : IRegisterReader
        {
            private readonly ModbusRtu _bus;
            private readonly int _slave;

            public SlaveReader(ModbusRtu bus, int slave)
            {
                _bus = bus;
                _slave = slave;
            }

            public ushort[] ReadRegisters(int address, int count) => _bus.ReadRegisters(_slave, address, count);
        }
    }
}
