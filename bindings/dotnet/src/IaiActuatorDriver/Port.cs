using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    /// <summary>
    /// What is on a port's line (the Python edition's <c>line="auto"</c>, "direct", "rcon", "robonet", "robonet-gateway").
    /// Gateway lines are DOC (ME0413, ME0208), UNVERIFIED on hardware.
    /// </summary>
    public enum LineKind
    {
        /// <summary>
        /// Found once (the default): READ-ONLY FC03 probes of an RCON gateway (D0h) and of a ROBONET gateway (3Fh), together
        /// with the baud detection when the rate is not fixed (about 0.1 s more on a line without a gateway).
        /// </summary>
        Auto = 0,

        /// <summary>The units straight on the line (one per port, or a daisy chain): nothing probed.</summary>
        Direct = 1,

        /// <summary>An RCON gateway (its status read at D0h; the axes are the linked ones).</summary>
        Rcon = 2,

        /// <summary>A ROBONET in SIO through mode (cannot be probed).</summary>
        Robonet = 3,

        /// <summary>A ROBONET in gateway mode: the gateway answers, no axis can be reached.</summary>
        RobonetGateway = 4,
    }

    /// <summary>
    /// One serial port: owns its <see cref="ITransport"/> and the <see cref="ModbusRtu"/> master on it.
    /// </summary>
    /// <remarks>
    /// Multi-axis layouts: normally one axis per COM port (open one <see cref="Port"/> per port; any number of ports
    /// can run in parallel, each with its own lock and thread-safe master), and up to 16 axes on one port by slave
    /// address for an RCON gateway or an SIO daisy chain (slave = axis number + 1; UNVERIFIED on hardware).
    /// <see cref="Scan"/> / <see cref="ScanUnits"/> find the addresses that answer; <see cref="Axis(int, AxisOptions?)"/> returns the axis at one
    /// address (created once per address, on the shared bus). A baud change on a port is a broadcast: every unit on the
    /// line switches (<see cref="SetBaudRate"/>). Several ports are driven together by a <see cref="Cell"/>.
    /// <para>The port is the native core's (a C++ Port over the core bus of <see cref="Bus"/>, iad_bus_set_port): the baud
    /// rate found once, the scan, the identities and the line-wide baud change run there, the same as in the Python
    /// edition, the line too: <see cref="LineKind.Auto"/> (the default) probes once for a gateway, read-only.</para>
    /// </remarks>
    public sealed class Port : IDisposable
    {
        private const string LogSource = "IaiActuatorDriver.Port";

        private readonly object _sync = new object();
        private readonly Dictionary<int, Axis> _axes = new Dictionary<int, Axis>();
        private bool _baudDetected;

        /// <summary>Slave addresses <see cref="Scan"/> asks by default (axes 0..15).</summary>
        public static IReadOnlyList<int> DefaultScanAddresses { get; } = Array.AsReadOnly(Enumerable.Range(1, 16).ToArray());

        /// <summary>Wrap an already-open transport (which the port then owns).</summary>
        /// <param name="transport">The byte pipe.</param>
        /// <param name="name">Display name (default: the transport's text).</param>
        /// <param name="retries">Retries of the master (default 3).</param>
        /// <param name="minGap">Minimum silence between transactions (default 5 ms).</param>
        /// <param name="frameLogger">Optional (direction, frame) callback.</param>
        /// <param name="autoBaud">The transport's current rate is a first guess: <see cref="EnsureBaudRate"/> (and
        /// <see cref="ScanUnits"/>) find the rate the units answer at once, in place. <c>false</c> = the rate is fixed. Ignored
        /// for a transport without a baud rate.</param>
        /// <param name="line">What is on the line: <see cref="LineKind.Auto"/> (default) finds it once, read-only, when the
        /// port first needs it (a scan, a broadcast); <see cref="LineKind.Direct"/> probes nothing.</param>
        public Port(ITransport transport, string? name = null, int retries = ModbusRtu.DefaultRetries, TimeSpan? minGap = null, Action<FrameDirection, byte[]>? frameLogger = null, bool autoBaud = true, LineKind line = LineKind.Auto)
        {
            if (!Enum.IsDefined(typeof(LineKind), line))
            {
                throw new ArgumentOutOfRangeException(nameof(line), line, "line must be a LineKind");
            }

            if (transport is null)
            {
                throw new ArgumentNullException(nameof(transport));
            }

            Bus = new ModbusRtu(transport, retries, minGap, frameLogger);
            Name = name ?? (transport is SerialTransport serial ? serial.PortName : transport.ToString() ?? "transport");
            try
            {
                // the core's port on this bus: the rate and the line found there, once (iad_bus_set_port)
                Bus.Native.SetPort(Name, (NativeLineKind)(int)line, fixedRate: !autoBaud || !Bus.CanChangeBaudRate);
                Line = line;
            }
            catch
            {
                Bus.Close();
                throw;
            }
        }

        /// <summary>The port name, e.g. "COM3".</summary>
        public string Name { get; }

        /// <summary>The line kind this port was given (<see cref="LineKind.Auto"/>: found once by the port).</summary>
        public LineKind Line { get; }

        /// <summary>The thread-safe Modbus master on this port.</summary>
        public ModbusRtu Bus { get; }

        /// <summary>The transport the master uses.</summary>
        public ITransport Transport => Bus.Transport;

        /// <summary>Current baud rate (<c>null</c> for a transport without one).</summary>
        public int? BaudRate => Bus.BaudRate;

        /// <summary>True when the baud rate was found by probing (<see cref="Open"/>, <see cref="EnsureBaudRate"/>).</summary>
        public bool BaudDetected
        {
            get
            {
                lock (_sync)
                {
                    if (!Bus.IsClosed)
                    {
                        try
                        {
                            _baudDetected = Bus.Native.PortInfo().Bool("baud_detected") ?? _baudDetected;
                        }
                        catch (ObjectDisposedException)
                        {
                            // closed meanwhile: the last value
                        }
                    }

                    return _baudDetected;
                }
            }
        }

        /// <summary>The axes created on this port (<see cref="Axis(int, AxisOptions?)"/>) by slave address, in address order (a copy).</summary>
        public IReadOnlyDictionary<int, Axis> Axes
        {
            get
            {
                lock (_sync)
                {
                    return _axes.OrderBy(kv => kv.Key).ToDictionary(kv => kv.Key, kv => kv.Value);
                }
            }
        }

        /// <summary>
        /// Open serial port <paramref name="portName"/> once and keep it open.
        /// </summary>
        /// <remarks>
        /// With <paramref name="baudRate"/> <c>null</c> (the default) the rate is found by probing in place:
        /// <see cref="ModbusRtu.DefaultBaudRate"/> (115200) first, then 38400 (the EC's power-up rate, VERIFIED), 9600,
        /// 230400, 57600, 19200 (<see cref="ModbusRtu.BaudDetectOrder"/>), asking <paramref name="detectSlaves"/> (default:
        /// slave 1) at each rate. The rate is changed on the open port: it is never closed and reopened in between. A
        /// fixed <paramref name="baudRate"/> is used as given (no probing). Throws <see cref="ModbusTimeoutException"/>
        /// (port closed again) when no rate answers.
        /// </remarks>
        public static Port Open(
            string portName,
            int? baudRate = null,
            IEnumerable<int>? detectSlaves = null,
            TimeSpan? readTimeout = null,
            int retries = ModbusRtu.DefaultRetries,
            Action<FrameDirection, byte[]>? frameLogger = null,
            LineKind line = LineKind.Auto)
        {
            var transport = new SerialTransport(portName, baudRate ?? ModbusRtu.DefaultBaudRate, readTimeout);
            Port? port = null;
            try
            {
                port = new Port(transport, portName, retries, null, frameLogger, autoBaud: baudRate is null, line);
                if (baudRate is null)
                {
                    port.DetectBaudRate(detectSlaves ?? new[] { 1 });
                }

                return port;
            }
            catch
            {
                if (port != null)
                {
                    port.Dispose();
                }
                else
                {
                    transport.Dispose();
                }

                throw;
            }
        }

        /// <summary>
        /// Find the baud rate one of <paramref name="slaves"/> answers at, in place (see
        /// <see cref="ModbusRtu.DetectBaudRate(IEnumerable{int}, IEnumerable{int}?, int, bool, TimeSpan?)"/>; the core port's
        /// detection, Port::detect_baud_rate). <see cref="EnsureBaudRate"/> then sends nothing more. When nothing answers at any
        /// rate, <see cref="ModbusTimeoutException"/> with the checklist (supply, cable, address, another program).
        /// </summary>
        public int DetectBaudRate(IEnumerable<int> slaves, IEnumerable<int>? candidates = null, TimeSpan? probeTimeout = null)
        {
            var addresses = (slaves ?? throw new ArgumentNullException(nameof(slaves))).ToList();
            if (addresses.Count == 0)
            {
                throw new ArgumentException("no slave address to probe", nameof(slaves));
            }

            foreach (var s in addresses)
            {
                CheckSlave(s);
            }

            if (probeTimeout.HasValue && probeTimeout.Value <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(probeTimeout), "probe timeout must be > 0");
            }

            if (!Bus.CanChangeBaudRate)
            {
                throw new NotSupportedException($"{Transport} has no baud rate to detect");
            }

            var list = candidates?.ToList();
            return Bus.OnPort(native => native.PortDetectBaudRate(addresses, list, probeTimeout?.TotalSeconds));
        }

        /// <summary>
        /// Detect the baud rate once (in place, the port is never reopened) by asking <paramref name="slaves"/> (default
        /// 1..16) at each rate of <see cref="ModbusRtu.BaudDetectOrder"/>. Does nothing for a fixed rate or a transport without
        /// one; later calls return at once. Throws <see cref="ModbusTimeoutException"/> when no address answers at any rate.
        /// </summary>
        public int? EnsureBaudRate(IEnumerable<int>? slaves = null)
        {
            var addresses = slaves?.Distinct().ToList();
            foreach (var s in addresses ?? new List<int>())
            {
                CheckSlave(s);
            }

            if (addresses != null && addresses.Count == 0)
            {
                addresses = null;  // 1..16
            }

            Bus.OnPort(native => native.EnsureBaudRate(addresses));
            return BaudRate;
        }

        /// <summary>
        /// Ask each address once (FC03 of one register at <paramref name="address"/>, no retries, at most
        /// <paramref name="timeout"/> each, default <see cref="ModbusRtu.ScanTimeout"/> = 50 ms) and return the addresses
        /// that answered, in order. An exception response counts as an answer. READ-ONLY; a scan of 16 addresses takes
        /// about 1 s when nothing answers. The baud rate is used as it is (see <see cref="ScanUnits"/>).
        /// </summary>
        public IReadOnlyList<int> Scan(IEnumerable<int>? slaves = null, TimeSpan? timeout = null, int address = 0x9000)
        {
            var found = new List<int>();
            foreach (var slave in slaves ?? DefaultScanAddresses)
            {
                if (Bus.Probe(slave, timeout ?? ModbusRtu.ScanTimeout, address))
                {
                    found.Add(slave);
                }
            }

            return found.AsReadOnly();
        }

        /// <summary>
        /// The addresses in <paramref name="slaves"/> (default 1..16) that answer, with what Report Slave ID (FC11) says about
        /// each. READ-ONLY. The baud rate is detected first when needed (<see cref="EnsureBaudRate"/>); when nothing answers
        /// at any rate the result is empty. Each address gets one FC03 probe with a <paramref name="timeout"/> wait (default
        /// 50 ms) and no retries; answering addresses then get one FC11 (waited for up to 0.65 s: some units answer it late)
        /// unless <paramref name="identify"/> is false.
        /// </summary>
        public IReadOnlyList<ScanHit> ScanUnits(IEnumerable<int>? slaves = null, TimeSpan? timeout = null, bool identify = true)
        {
            var addresses = (slaves ?? DefaultScanAddresses).Distinct().ToList();
            foreach (var s in addresses)
            {
                CheckSlave(s);
            }

            if (addresses.Count == 0)
            {
                return Array.Empty<ScanHit>();
            }

            if (timeout.HasValue && timeout.Value <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(timeout), "scan timeout must be > 0");
            }

            // the core's scan: the rate settled first (none answering: an empty scan, logged), one probe per address, FC11
            var hits = Bus.OnPort(native => native.ScanUnits(addresses, timeout?.TotalSeconds, identify));
            return hits.OfType<JsonObject>().Select(ScanHit.FromCore).ToList().AsReadOnly();
        }

        /// <summary>A register reader bound to <paramref name="slave"/> (for <see cref="UnitDirectory"/> and other read-only helpers).</summary>
        public IRegisterReader Slave(int slave) => Bus.ForSlave(slave);

        /// <summary>What <paramref name="slave"/> tells about itself (<see cref="DeviceIdentity.Read"/>). READ-ONLY.</summary>
        public DeviceInfo Identify(int slave) => DeviceIdentity.Read(Bus, slave);

        /// <summary>
        /// The axis at <paramref name="slave"/> on this port, created once per address on the shared bus (several axes on one
        /// port by slave address are UNVERIFIED on hardware). <paramref name="options"/> apply when it is created; later calls
        /// return the same object. The port keeps owning the bus.
        /// </summary>
        public Axis Axis(int slave = 1, AxisOptions? options = null) => AxisOf(slave, options, null);

        /// <summary>
        /// <see cref="Axis(int, AxisOptions?)"/>; <paramref name="adopted"/>: a cell's own core axis at that address
        /// (iad_cell_axis_ref), taken over when the axis is new, released otherwise.
        /// </summary>
        internal Axis AxisOf(int slave, AxisOptions? options, NativeAxis? adopted)
        {
            try
            {
                CheckSlave(slave);
                lock (_sync)
                {
                    if (!_axes.TryGetValue(slave, out var axis))
                    {
                        axis = new Axis(Bus, slave, options, ownsBus: false, adopted);
                        adopted = null;  // the axis owns it from here on (a constructor that throws leaves it to the finally)
                        _axes[slave] = axis;
                    }

                    return axis;
                }
            }
            finally
            {
                adopted?.Close();
            }
        }

        /// <summary>The directory of <paramref name="slave"/> (<see cref="UnitDirectory.Read"/>). READ-ONLY.</summary>
        public UnitDirectory ReadDirectory(int slave) => UnitDirectory.Read(Bus.ForSlave(slave));

        /// <summary>
        /// Change the line's baud rate with the broadcast baud change: <b>every unit on this port switches</b>. The switch
        /// itself is <see cref="IaiActuatorDriver.Axis.SetBaudRate"/> on the first axis (same rules; volatile - the
        /// units are back at
        /// their power-up rate after a power cycle). Every axis checked must be at rest (<see cref="EcNotReadyException"/>,
        /// nothing sent). The other addresses (<paramref name="slaves"/>, default the port's axes) are then probed at the new
        /// rate; the result says which answered. Several units switching at once is UNVERIFIED on hardware.
        /// </summary>
        public PortBaudResult SetBaudRate(
            int rate,
            IEnumerable<int>? slaves = null,
            BaudChangeMethod method = BaudChangeMethod.Direct,
            double settleS = IaiActuatorDriver.Axis.BaudSettleS,
            bool allowUnverifiedCode = false)
        {
            var addresses = (slaves ?? Axes.Keys).Distinct().OrderBy(s => s).ToList();
            if (addresses.Count == 0)
            {
                throw new EcNotReadyException($"{Name}: no axis on this port to verify a baud change with");
            }

            foreach (var s in addresses)
            {
                CheckSlave(s);
            }

            // the core's line-wide change (Port::set_baud_rate): the at-rest check of every address under the bus's
            // sequence lock (nothing sent when one is not at rest), the broadcast verified on the first, the others probed
            var o = SizedStruct.Fresh<IadBaudOptions>();
            NativeErrors.Check(NativeMethods.iad_baud_options_init(ref o));
            o.Method = method == BaudChangeMethod.Break ? 1 : 0;
            o.SettleS = settleS;
            o.AllowUnverifiedCode = Known.Flag(allowUnverifiedCode);
            var result = Bus.OnPort(native => native.SetBaudRateAll(rate, addresses, o));
            var answered = new SortedDictionary<int, bool>();
            var map = result.Object("answered") ?? new JsonObject();
            foreach (var key in map.Keys)
            {
                answered[int.Parse(key, NumberStyles.Integer, CultureInfo.InvariantCulture)] = map.Bool(key) ?? false;
            }

            var change = AxisRecords.ToBaudChange(result.Object("change") ?? new JsonObject());
            return new PortBaudResult(Name, rate, change, answered.ToDictionary(kv => kv.Key, kv => kv.Value));
        }

        /// <summary>Release any STOP coil left ON by an axis of this port (best effort), then close the port. Safe to call twice.</summary>
        public void Close()
        {
            foreach (var axis in Axes.Values)
            {
                try
                {
                    axis.Close();
                }
                catch (Exception exc)
                {
                    // never keep the port open because one axis failed to close
                    DriverLog.Warning(LogSource, $"{Name}: closing slave {axis.Slave}: {exc.Message}");
                }
            }

            Bus.Close();
        }

        /// <inheritdoc />
        public void Dispose() => Close();

        /// <inheritdoc />
        public override string ToString() => BaudRate.HasValue ? $"Port({Name}, {BaudRate} baud)" : $"Port({Name})";

        private static void CheckSlave(int slave)
        {
            if (slave < 1 || slave > 247)
            {
                throw new ArgumentOutOfRangeException(nameof(slave), slave, "slave address must be 1..247");
            }
        }
    }

    /// <summary>One address that answered a scan (FC03 probe), with what Report Slave ID (FC11) says about it.</summary>
    /// <param name="Port">The port name.</param>
    /// <param name="Slave">The slave address (axis number + 1).</param>
    public sealed record ScanHit(string Port, int Slave)
    {
        /// <summary>The FC11 data bytes (<c>null</c> when FC11 was not asked or not answered).</summary>
        public IReadOnlyList<byte>? SlaveIdRaw { get; init; }

        /// <summary>FC11 system code (0xEC = EC 24 V; others: CON-family controllers).</summary>
        public int? SystemCode { get; init; }

        /// <summary>FC11 unit code.</summary>
        public int? UnitCode { get; init; }

        /// <summary>Firmware version, 4 hex digits.</summary>
        public string? Firmware { get; init; }

        /// <summary>FC11 not answered / rejected (the address still answered the probe).</summary>
        public string? Error { get; init; }

        /// <summary>The line kind of the port ("direct", "rcon", "robonet", "robonet-gateway").</summary>
        public string Line { get; init; } = "direct";

        /// <summary>The 0x9400 serial code: the identity on an RCON line, which has no FC11 (<c>null</c> elsewhere).</summary>
        public string? SerialCode { get; init; }

        /// <summary>"&lt;port&gt;:&lt;slave&gt;", the axis name a <see cref="Cell"/> gives it.</summary>
        public string Name => PortManager.AxisName(Port, Slave);

        /// <summary>Axis number = slave address - 1.</summary>
        public int AxisNumber => Slave - 1;

        /// <summary>Name of the system code (<c>null</c> when FC11 was not answered).</summary>
        public string? SystemName => SystemCode is null
            ? null
            : Registers.SYSTEM_CODES.TryGetValue(SystemCode.Value, out var name) ? name : "system 0x" + SystemCode.Value.ToString("X2", CultureInfo.InvariantCulture);

        /// <summary>Product family ("EC", "PCON", "ACON/DCON", "SCON", else "IAI controller"); <c>null</c> when FC11 was not answered. Every family is driven the same way.</summary>
        public string? ProductFamily => SystemCode is null ? null : Registers.ProductFamily(SystemCode);

        /// <summary>EC family name from the unit code (<c>null</c> for a CON-family unit or an EC unit code the driver has no name for; every unit is driven the same way).</summary>
        public string? Family => Registers.EcFamily(SystemCode, UnitCode);

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["name"] = Name,
            ["port"] = Port,
            ["slave"] = Slave,
            ["axis_number"] = AxisNumber,
            ["slave_id_raw"] = SlaveIdRaw is null ? null : ModbusRtu.HexText(SlaveIdRaw),
            ["system_code"] = SystemCode,
            ["system_name"] = SystemName,
            ["unit_code"] = UnitCode,
            ["firmware"] = Firmware,
            ["product_family"] = ProductFamily,
            ["family"] = Family,
            ["error"] = Error,
            ["line"] = Line,
            ["serial_code"] = SerialCode,
        };

        /// <summary>A hit of the core's scan (iad_bus_scan_units_json: the Python edition's ScanHit.to_dict).</summary>
        internal static ScanHit FromCore(JsonObject o) => new ScanHit(o.String("port") ?? string.Empty, o.Int("slave") ?? 0)
        {
            SlaveIdRaw = o.String("slave_id_raw") is string raw ? AxisRecords.Hex(raw) : null,
            SystemCode = o.Int("system_code"),
            UnitCode = o.Int("unit_code"),
            Firmware = o.String("firmware"),
            Error = o.String("error"),
            Line = o.String("line") ?? "direct",
            SerialCode = o.String("serial_code"),
        };
    }

    /// <summary>Outcome of <see cref="Port.SetBaudRate"/>: the switch (verified on the first axis) and who answers after it.</summary>
    public sealed record PortBaudResult(string Port, int Rate, BaudChangeResult Change, IReadOnlyDictionary<int, bool> Answered)
    {
        /// <summary>Every address checked answered at the new rate.</summary>
        public bool AllAnswered => Answered.Values.All(v => v);

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["port"] = Port,
            ["rate"] = Rate,
            ["change"] = Change.ToDictionary(),
            ["answered"] = Answered.ToDictionary(kv => kv.Key.ToString(CultureInfo.InvariantCulture), kv => (object?)kv.Value),
        };
    }
}
