using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    /// <summary>How <see cref="Cell.Open(CellOptions?)"/> opens its ports and finds its axes.</summary>
    public sealed record CellOptions
    {
        /// <summary>The most axes the cell takes; 0 (default) = no limit, every axis found or listed. Listed slaves above it: <see cref="ArgumentException"/>; a scan that finds more keeps the first ones and logs a warning.</summary>
        public int MaxAxes { get; init; }

        /// <summary>Fixed rate for every port; <c>null</c> (default) detects each port's rate once, in place.</summary>
        public int? BaudRate { get; init; }

        /// <summary>Per-read timeout of the serial ports opened by name (default 0.2 s).</summary>
        public TimeSpan? ReadTimeout { get; init; }

        /// <summary>Retries of each port's master (timeouts / CRC errors).</summary>
        public int Retries { get; init; } = ModbusRtu.DefaultRetries;

        /// <summary>Per-address probe wait while scanning (default <see cref="ModbusRtu.ScanTimeout"/>, 50 ms).</summary>
        public TimeSpan? ScanTimeout { get; init; }

        /// <summary>Addresses asked when the slaves are not listed (default 1..16 = axes 0..15).</summary>
        public IReadOnlyList<int>? ScanSlaves { get; init; }

        /// <summary>Run <see cref="Axis.Identify"/> on every axis (ports in parallel) so each axis knows its unit's ranges; results in <see cref="Cell.Info"/>.</summary>
        public bool Identify { get; init; } = true;

        /// <summary>Optional (port name, direction, frame) callback for every frame of every port the cell opens.</summary>
        public Action<string, FrameDirection, byte[]>? FrameLogger { get; init; }

        /// <summary>Options of every axis (caps, strategy, timing); <c>null</c> = defaults.</summary>
        public AxisOptions? AxisOptions { get; init; }

        /// <summary>
        /// What is on the line of every port the cell opens (default <see cref="LineKind.Auto"/>: found once per port,
        /// READ-ONLY; <see cref="LineKind.Direct"/> probes nothing). A <see cref="Port"/> given to the cell keeps its own.
        /// </summary>
        public LineKind Line { get; init; } = LineKind.Auto;

        /// <summary>Test seam: lists the ports instead of <see cref="PortManager.ListPorts"/>.</summary>
        internal Func<IReadOnlyList<SerialPortInfo>>? PortLister { get; init; }

        /// <summary>Test seam: opens a port by name (name, baud, read timeout) instead of <see cref="SerialTransport"/>.</summary>
        internal Func<string, int, TimeSpan?, ITransport>? SerialOpener { get; init; }
    }

    /// <summary>One absolute move of a <see cref="Cell.MoveAll(IEnumerable{AxisMove}, bool, double?, bool, bool, CancellationToken)"/>, in the axis's own units (mm, or deg on a rotary axis).</summary>
    /// <param name="Axis">The axis name in the cell, e.g. "COM3:1".</param>
    /// <param name="Position">Target position.</param>
    public sealed record AxisMove(string Axis, double Position)
    {
        /// <summary>Speed (units/s).</summary>
        public double Speed { get; init; } = 50.0;

        /// <summary>Acceleration (G); also the deceleration unless <see cref="DecelG"/> differs.</summary>
        public double AccelG { get; init; } = 0.1;

        /// <summary>Positioning band (units).</summary>
        public double Band { get; init; } = 0.1;

        /// <summary>A deceleration different from <see cref="AccelG"/> uses the command-profile move (<see cref="IaiActuatorDriver.Axis.MoveTo"/>).</summary>
        public double? DecelG { get; init; }
    }

    /// <summary>
    /// One or more axes failed in a cell operation. <see cref="Errors"/> / <see cref="Results"/> are keyed by axis name
    /// (cell order); every axis was handled before this was raised.
    /// </summary>
    public sealed class CellException : EcDriverException
    {
        public CellException(string operation, IReadOnlyDictionary<string, Exception> errors, IReadOnlyDictionary<string, object?> results)
            : base(BuildMessage(operation, errors))
        {
            Operation = operation;
            Errors = errors;
            Results = results;
        }

        /// <summary>The operation, e.g. "MoveAll (start)".</summary>
        public string Operation { get; }

        /// <summary>The error of each failed axis.</summary>
        public IReadOnlyDictionary<string, Exception> Errors { get; }

        /// <summary>The result of each axis that succeeded.</summary>
        public IReadOnlyDictionary<string, object?> Results { get; }

        private static string BuildMessage(string operation, IReadOnlyDictionary<string, Exception> errors) =>
            $"{operation} failed on {errors.Count} axis/axes: " + string.Join("; ", errors.Select(kv => $"{kv.Key}: {kv.Value.GetType().Name}: {kv.Value.Message}"));
    }

    /// <summary>
    /// Per-axis results of a cell operation: a read-only dictionary of the axes that succeeded (cell order), plus
    /// <see cref="Errors"/> for the ones that failed (only ever non-empty with <c>returnExceptions: true</c>).
    /// </summary>
    public sealed class CellResults<T> : IReadOnlyDictionary<string, T>
    {
        private readonly Dictionary<string, T> _results;

        internal CellResults(string operation, IReadOnlyList<string> names, Dictionary<string, T> results, Dictionary<string, Exception> errors)
        {
            Operation = operation;
            Names = names;
            _results = results;
            Errors = errors;
        }

        /// <summary>The operation, e.g. "StatusAll".</summary>
        public string Operation { get; }

        /// <summary>Every axis handled, in cell order.</summary>
        public IReadOnlyList<string> Names { get; }

        /// <summary>The error of each failed axis.</summary>
        public IReadOnlyDictionary<string, Exception> Errors { get; }

        /// <summary>No axis failed.</summary>
        public bool Succeeded => Errors.Count == 0;

        /// <inheritdoc />
        public int Count => _results.Count;

        /// <inheritdoc />
        public IEnumerable<string> Keys => _results.Keys;

        /// <inheritdoc />
        public IEnumerable<T> Values => _results.Values;

        /// <summary>The result of <paramref name="name"/>; a failed axis raises <see cref="KeyNotFoundException"/> with its error inside.</summary>
        public T this[string name]
        {
            get
            {
                if (_results.TryGetValue(name, out var value))
                {
                    return value;
                }

                if (Errors.TryGetValue(name, out var error))
                {
                    throw new KeyNotFoundException($"{name} failed in {Operation}: {error.Message}", error);
                }

                throw new KeyNotFoundException($"{name} was not part of {Operation}");
            }
        }

        /// <inheritdoc />
        public bool ContainsKey(string key) => _results.ContainsKey(key);

        /// <inheritdoc />
        public bool TryGetValue(string key, out T value) => _results.TryGetValue(key, out value!);

        /// <inheritdoc />
        public IEnumerator<KeyValuePair<string, T>> GetEnumerator() => _results.GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }

    /// <summary>
    /// Several axes on one or more ports, driven together (the <c>Cell</c> of the Python edition). Build it with
    /// <see cref="Open(CellOptions?)"/> (or from open <see cref="Port"/>s and their axes).
    /// </summary>
    /// <remarks>
    /// <para>The normal setup is one axis per COM port (one IAI USB adapter per actuator, all driven in parallel; the
    /// cell takes every axis it finds unless <see cref="CellOptions.MaxAxes"/> caps it). Several axes can also share
    /// one port by slave address - an RCON gateway or an SIO daisy chain, up to 16 per port, slave = axis number + 1
    /// (UNVERIFIED on hardware) - and the two can be mixed. Axis names are "&lt;port&gt;:&lt;slave&gt;" ("COM3:1");
    /// <see cref="Names"/> / <see cref="Axes"/> are in port order, then slave order.</para>
    /// <para>Rules (as in the Python edition): a port is opened once, with its own lock, and its baud rate detected once, in
    /// place. Ports work in parallel; the axes of one port take turns on its lock. Waiting loops sleep between polls
    /// without holding the lock, so axes sharing a line interleave their frames and move at the same time. Each axis is an
    /// ordinary <see cref="Axis"/> with its own ranges read from its unit; every unit is driven the same way, whatever its model.
    /// <see cref="StopAll"/> writes STOP to every axis first (ports in parallel, axes of a port back to back), then waits
    /// for each to stop. A failure on one axis during <see cref="MoveAll(IEnumerable{AxisMove}, bool, double?, bool, bool, CancellationToken)"/> /
    /// <see cref="HomeAll"/> / <see cref="WaitAll"/> stops the others too (<c>stopOthersOnError</c>). Operations on many axes
    /// collect per-axis results; any failure raises <see cref="CellException"/> (with every result and error) after all
    /// axes were handled, or, with <c>returnExceptions: true</c>, is returned in <see cref="CellResults{T}.Errors"/>.</para>
    /// <para>The cell is the native core's (iad_cell, C++ Cell) over the core ports of its <see cref="Port"/>s: the scan,
    /// the listed slaves, <see cref="CellOptions.MaxAxes"/> and every group command run there, on threads of the core (a
    /// managed transport is called from those threads). Opening the ports, the port errors of ports that cannot be opened
    /// and the .NET argument checks stay here.</para>
    /// <para><see cref="Dispose"/> releases STOP coils left on and closes every port; it cannot stop the axes when an
    /// exception leaves a <c>using</c> block. The waiting calls take a <see cref="CancellationToken"/>: cancelling stops each
    /// axis that is waiting (decel STOP sequence).</para>
    /// <para>Evidence: one axis per port is VERIFIED (each tested unit alone on its own USB adapter). Several axes on one
    /// line, the broadcast baud change reaching several units and the identification of other USB adapters are UNVERIFIED.</para>
    /// </remarks>
    public sealed class Cell : IDisposable, IEnumerable<Axis>
    {
        private const string LogSource = "IaiActuatorDriver.Cell";

        private readonly object _sync = new object();
        private readonly List<Port> _portList;
        private readonly List<string> _names;
        private readonly Dictionary<string, Axis> _axes;
        private readonly Dictionary<string, Port> _portOf = new Dictionary<string, Port>();
        private readonly Dictionary<string, DeviceInfo> _info = new Dictionary<string, DeviceInfo>();
        private readonly NativeCell? _core;
        private IReadOnlyDictionary<string, Exception> _portErrors = new Dictionary<string, Exception>();
        private bool _closed;

        /// <summary>
        /// A cell of every axis already created on <paramref name="ports"/> (<see cref="Port.Axis(int, AxisOptions?)"/>), named
        /// "&lt;port&gt;:&lt;slave&gt;". The cell owns the ports from then on (closed with it).
        /// </summary>
        public Cell(IEnumerable<Port> ports)
            : this(
                (ports ?? throw new ArgumentNullException(nameof(ports))).ToList(),
                null,
                null)
        {
        }

        /// <summary>
        /// The cell over <paramref name="ports"/>: with <paramref name="handle"/> the core's cell that opened
        /// <paramref name="axes"/> (iad_cell_open_buses), else a core cell gathering the axes of the ports
        /// (iad_cell_open_axes; nothing sent).
        /// </summary>
        private Cell(List<Port> ports, IReadOnlyList<(string Name, Axis Axis)>? axes, CellHandle? handle)
        {
            var dup = ports.GroupBy(p => p.Name).FirstOrDefault(g => g.Count() > 1);
            if (dup != null)
            {
                handle?.Dispose();
                throw new ArgumentException($"two ports named {dup.Key}: port names must be unique in a cell", nameof(ports));
            }

            _portList = ports;
            axes ??= ports.SelectMany(p => p.Axes.Select(kv => (PortManager.AxisName(p.Name, kv.Key), kv.Value))).ToList();
            _names = new List<string>();
            _axes = new Dictionary<string, Axis>();
            foreach (var (name, axis) in axes)
            {
                if (_axes.ContainsKey(name))
                {
                    handle?.Dispose();
                    throw new ArgumentException($"two axes named {name}", nameof(axes));
                }

                var port = ports.FirstOrDefault(p => ReferenceEquals(p.Bus, axis.Bus));
                if (port is null)
                {
                    handle?.Dispose();
                    throw new ArgumentException($"axis {name} is not on any port of this cell", nameof(axes));
                }

                _names.Add(name);
                _axes[name] = axis;
                _portOf[name] = port;
            }

            _core = NativeCell.Own(handle ?? Gather(_names, _axes));
        }

        // ------------------------------------------------------------------ construction

        /// <summary>
        /// Open every IAI USB adapter (<see cref="PortManager.ListPorts"/>), find every axis that answers at the addresses of
        /// <see cref="CellOptions.ScanSlaves"/> (default 1..16) and return the cell. READ-ONLY: nothing is written to any unit.
        /// A port that cannot be opened or where nothing answers is skipped (<see cref="PortErrors"/>). Throws
        /// <see cref="EcNotReadyException"/> when no adapter or no axis is found.
        /// </summary>
        public static Cell Open(CellOptions? options = null)
        {
            var o = options ?? new CellOptions();
            var names = (o.PortLister ?? PortManager.ListPorts)().Where(p => p.IsIai).Select(p => p.Port).ToList();
            if (names.Count == 0)
            {
                throw new EcNotReadyException("no IAI USB adapter found (PortManager.ListPorts); name the ports to open");
            }

            return OpenCore(names.Select(n => NamedSpec(n, null, o)).ToList(), strict: false, o);
        }

        /// <summary>
        /// Open the serial ports <paramref name="portNames"/> (e.g. "COM3", "COM4") and return the cell. With
        /// <paramref name="slaves"/> <c>null</c> every answering address of <see cref="CellOptions.ScanSlaves"/> is taken
        /// (ports where nothing answers are skipped); a list is used on every port and each listed slave must answer (any
        /// failure raises, every port closed again). READ-ONLY.
        /// </summary>
        public static Cell Open(IEnumerable<string> portNames, IEnumerable<int>? slaves = null, CellOptions? options = null)
        {
            var o = options ?? new CellOptions();
            var listed = slaves?.ToList();
            var specs = (portNames ?? throw new ArgumentNullException(nameof(portNames))).Select(n => NamedSpec(n, listed, o)).ToList();
            return OpenCore(specs, strict: listed != null, o);
        }

        /// <summary>
        /// Open the ports named by <paramref name="slavesByPort"/> (e.g. { "COM3": [1], "COM5": [2, 3] }) with those slaves;
        /// each listed slave must answer (any failure raises, every port closed again). READ-ONLY.
        /// </summary>
        public static Cell Open(IReadOnlyDictionary<string, int[]> slavesByPort, CellOptions? options = null)
        {
            var o = options ?? new CellOptions();
            var specs = (slavesByPort ?? throw new ArgumentNullException(nameof(slavesByPort)))
                .Select(kv => NamedSpec(kv.Key, (kv.Value ?? Array.Empty<int>()).ToList(), o)).ToList();
            return OpenCore(specs, strict: true, o);
        }

        /// <summary>
        /// A cell on already-open transports by port name (e.g. simulated lines); each is wrapped in a <see cref="Port"/> the
        /// cell owns. Slaves as for <see cref="Open(IEnumerable{string}, IEnumerable{int}?, CellOptions?)"/>. READ-ONLY.
        /// </summary>
        public static Cell Open(IReadOnlyDictionary<string, ITransport> transports, IEnumerable<int>? slaves = null, CellOptions? options = null)
        {
            var o = options ?? new CellOptions();
            var listed = slaves?.ToList();
            var specs = (transports ?? throw new ArgumentNullException(nameof(transports))).Select(kv => new PortSpec(kv.Key, listed, () =>
            {
                var transport = kv.Value ?? throw new ArgumentNullException(nameof(transports), $"no transport for {kv.Key}");
                var fixedRate = o.BaudRate.HasValue && transport.BaudRate != 0;
                if (fixedRate)
                {
                    transport.BaudRate = o.BaudRate!.Value;
                }

                return new Port(transport, kv.Key, o.Retries, null, PortLogger(kv.Key, o), autoBaud: !fixedRate, o.Line);
            })).ToList();
            return OpenCore(specs, strict: listed != null, o);
        }

        /// <summary>
        /// A cell on simulated (or any other) transports by port name, e.g. <c>Cell.Open(SimLine.SimPorts(3))</c>: the same as
        /// <see cref="Open(IReadOnlyDictionary{string, ITransport}, IEnumerable{int}?, CellOptions?)"/> for a dictionary of a
        /// more specific transport type. READ-ONLY.
        /// </summary>
        public static Cell Open<TTransport>(IReadOnlyDictionary<string, TTransport> transports, IEnumerable<int>? slaves = null, CellOptions? options = null)
            where TTransport : class, ITransport
        {
            if (transports is null)
            {
                throw new ArgumentNullException(nameof(transports));
            }

            return Open(transports.ToDictionary(kv => kv.Key, kv => (ITransport)kv.Value), slaves, options);
        }

        /// <summary>
        /// A cell on open <see cref="Port"/>s (which the cell owns from then on). Slaves as for
        /// <see cref="Open(IEnumerable{string}, IEnumerable{int}?, CellOptions?)"/>. READ-ONLY.
        /// </summary>
        public static Cell Open(IEnumerable<Port> ports, IEnumerable<int>? slaves = null, CellOptions? options = null)
        {
            var o = options ?? new CellOptions();
            var listed = slaves?.ToList();
            var specs = (ports ?? throw new ArgumentNullException(nameof(ports))).Select(p => new PortSpec(p.Name, listed, () => p)).ToList();
            return OpenCore(specs, strict: listed != null, o);
        }

        // ------------------------------------------------------------------ access

        /// <summary>The ports of the cell by name, in order.</summary>
        public IReadOnlyDictionary<string, Port> Ports => _portList.ToDictionary(p => p.Name, p => p);

        /// <summary>The axis names ("COM3:1" ...), in port order, then slave order.</summary>
        public IReadOnlyList<string> Names => _names.AsReadOnly();

        /// <summary>The axes, in the order of <see cref="Names"/>.</summary>
        public IReadOnlyList<Axis> Axes => _names.Select(n => _axes[n]).ToList().AsReadOnly();

        /// <summary>Number of axes.</summary>
        public int Count => _names.Count;

        /// <summary><see cref="Axis.Identify"/> of every axis, by name (filled by <see cref="Open(CellOptions?)"/> and <see cref="IdentifyAll"/>).</summary>
        public IReadOnlyDictionary<string, DeviceInfo> Info
        {
            get
            {
                lock (_sync)
                {
                    return _names.Where(_info.ContainsKey).ToDictionary(n => n, n => _info[n]);
                }
            }
        }

        /// <summary>The ports skipped by a scanning <see cref="Open(CellOptions?)"/> and why, by port name.</summary>
        public IReadOnlyDictionary<string, Exception> PortErrors => _portErrors;

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

        /// <summary>The axis named <paramref name="name"/> ("COM3:1").</summary>
        public Axis this[string name] => _axes[CheckName(name)];

        /// <summary>The axis at <paramref name="index"/> of <see cref="Names"/>.</summary>
        public Axis this[int index] => _axes[_names[index]];

        /// <summary>The cell has an axis named <paramref name="name"/>.</summary>
        public bool Contains(string name) => name != null && _axes.ContainsKey(name);

        /// <summary>The cell name of <paramref name="axis"/>.</summary>
        public string NameOf(Axis axis) =>
            _names.FirstOrDefault(n => ReferenceEquals(_axes[n], axis)) ?? throw new KeyNotFoundException($"{axis} is not in this cell");

        /// <summary>The <see cref="Port"/> the axis <paramref name="name"/> is on.</summary>
        public Port PortOf(string name) => _portOf[CheckName(name)];

        /// <summary>Position unit of one axis: "deg" on a rotary axis, else "mm" (read from the unit once).</summary>
        public string UnitsOf(string name) => this[name].Units;

        /// <inheritdoc />
        public IEnumerator<Axis> GetEnumerator() => Axes.GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        /// <inheritdoc />
        public override string ToString() => $"Cell({string.Join(", ", _names)})";

        /// <summary>The core's cell (there is one whenever the cell has an axis).</summary>
        private CellHandle H => _core?.Handle ?? throw new EcNotReadyException("the cell has no axis");

        /// <summary>The core's cell under this object (null without an axis; the interop tests reach the functions this class does not wrap).</summary>
        internal NativeCell? Core => _core;

        // ------------------------------------------------------------------ read-only operations

        /// <summary><see cref="Axis.Identify"/> on every axis (default: all; ports in parallel, the axes of a port one after another). READ-ONLY. Stored in <see cref="Info"/>.</summary>
        public CellResults<DeviceInfo> IdentifyAll(IEnumerable<string>? axes = null, bool returnExceptions = false)
        {
            CheckOpen();
            var names = Select(axes);
            var (found, errors) = ReadAll("identify", names);
            var results = found.ToDictionary(kv => kv.Key, kv => AxisRecords.ToDeviceInfo(kv.Value));
            lock (_sync)
            {
                foreach (var kv in results)
                {
                    _info[kv.Key] = kv.Value;
                }
            }

            return Finish("IdentifyAll", names, results, errors, returnExceptions);
        }

        /// <summary>Every axis's status: ports in parallel, the axes of a port one after another. READ-ONLY.</summary>
        public CellResults<Status> StatusAll(IEnumerable<string>? axes = null, bool returnExceptions = false)
        {
            CheckOpen();
            var names = Select(axes);
            var (results, errors, _) = Group(names, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_status_all(H, a, double.NaN, r, c, out n));
            return Finish("StatusAll", names, Present(results), errors, returnExceptions);
        }

        /// <summary><see cref="Axis.Limits"/> of every axis: each unit's own ranges, units (mm / deg), kind and family. READ-ONLY.</summary>
        public CellResults<AxisRanges> LimitsAll(IEnumerable<string>? axes = null, bool returnExceptions = false)
        {
            CheckOpen();
            var names = Select(axes);
            var (found, errors) = ReadAll("limits", names);
            return Finish("LimitsAll", names, found.ToDictionary(kv => kv.Key, kv => AxisRecords.ToRanges(kv.Value)), errors, returnExceptions);
        }

        /// <summary><see cref="Axis.ReadMonitor"/> of every axis (position, velocity, current, deviation, alarm, uptime). READ-ONLY.</summary>
        public CellResults<MonitorReading> MonitorAll(IEnumerable<string>? axes = null, bool returnExceptions = false)
        {
            CheckOpen();
            var names = Select(axes);
            var (found, errors) = ReadAll("monitor", names);
            return Finish("MonitorAll", names, found.ToDictionary(kv => kv.Key, kv => AxisRecords.ToMonitor(kv.Value)), errors, returnExceptions);
        }

        // ------------------------------------------------------------------ commands

        /// <summary>Servo ON / OFF on every axis and wait for each to confirm (axes in parallel).</summary>
        public CellResults<Status?> ServoAll(bool on, IEnumerable<string>? axes = null, double timeout = 3.0, bool returnExceptions = false)
        {
            CheckOpen();
            var names = Select(axes);
            var (results, errors, _) = Group(names, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_servo_all(H, Known.Flag(on), a, timeout, r, c, out n));
            return Finish("ServoAll", names, results, errors, returnExceptions);
        }

        /// <summary>
        /// Home return on every axis at once and wait for all (each axis polls in its own task). A failure on one axis stops
        /// the others (<paramref name="stopOthersOnError"/>); cancelling stops every axis still homing.
        /// </summary>
        public CellResults<Status?> HomeAll(
            IEnumerable<string>? axes = null,
            double? timeout = null,
            CommandMethod? method = null,
            bool stopOthersOnError = true,
            bool returnExceptions = false,
            CancellationToken cancellationToken = default)
        {
            CheckOpen();
            var names = Select(axes);
            var code = Axis.MethodFor(method);
            var (results, errors, _) = Waiting(names, cancellationToken, () => Group(names, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_home_all(H, a, Known.OrNaN(timeout), code, Known.Flag(stopOthersOnError), r, c, out n)));
            ThrowIfCancelled(errors, cancellationToken);
            return Finish("HomeAll", names, results, errors, returnExceptions);
        }

        /// <summary>
        /// Absolute moves with one speed, acceleration and band for every axis in <paramref name="targets"/>
        /// (axis name -&gt; position, each in its axis's own units); see
        /// <see cref="MoveAll(IEnumerable{AxisMove}, bool, double?, bool, bool, CancellationToken)"/>.
        /// </summary>
        public CellResults<Status>? MoveAll(
            IReadOnlyDictionary<string, double> targets,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? decelG = null,
            bool wait = true,
            double? timeout = null,
            bool stopOthersOnError = true,
            bool returnExceptions = false,
            CancellationToken cancellationToken = default)
        {
            var moves = (targets ?? throw new ArgumentNullException(nameof(targets)))
                .Select(kv => new AxisMove(kv.Key, kv.Value) { Speed = speed, AccelG = accelG, Band = band, DecelG = decelG });
            return MoveAll(moves, wait, timeout, stopOthersOnError, returnExceptions, cancellationToken);
        }

        /// <summary>
        /// Start an absolute move on every axis of <paramref name="moves"/>, then (<paramref name="wait"/>) wait for all.
        /// </summary>
        /// <remarks>
        /// Positions, speeds and bands are in each axis's own units (mm, or deg on a rotary axis: <see cref="UnitsOf"/>).
        /// Every move is first checked against its own unit's ranges and every axis for readiness
        /// (<see cref="Axis.CheckMove"/>, nothing sent): one bad value refuses the whole move with
        /// <see cref="CellException"/> before any axis starts. The moves are then started with as little delay as possible
        /// (ports in parallel, the axes of a port back to back); a start that fails stops the axes already started
        /// (<paramref name="stopOthersOnError"/>) and raises. With <paramref name="wait"/> false this returns <c>null</c> right
        /// after the starts: call <see cref="WaitAll"/> later. <paramref name="timeout"/> is per axis (default: from the move's
        /// distance, speed and acceleration).
        /// </remarks>
        public CellResults<Status>? MoveAll(
            IEnumerable<AxisMove> moves,
            bool wait = true,
            double? timeout = null,
            bool stopOthersOnError = true,
            bool returnExceptions = false,
            CancellationToken cancellationToken = default)
        {
            CheckOpen();
            var plan = new Dictionary<string, AxisMove>();
            var names = new List<string>();
            foreach (var move in moves ?? throw new ArgumentNullException(nameof(moves)))
            {
                var name = CheckName(move.Axis);
                if (plan.ContainsKey(name))
                {
                    throw new ArgumentException($"two moves for axis {name}", nameof(moves));
                }

                plan[name] = move;
                names.Add(name);
            }

            // The .NET check of the inputs, nothing sent: a value that is not a number refuses the whole move as before.
            var refused = new Dictionary<string, Exception>();
            foreach (var name in names)
            {
                var m = plan[name];
                try
                {
                    Axis.CheckFinite(m.Position, m.Speed, m.AccelG, m.Band, m.DecelG);
                }
                catch (SoftLimitException exc)
                {
                    refused[name] = exc;
                }
            }

            if (refused.Count > 0)
            {
                throw new CellException("MoveAll (check; nothing started)", refused, new Dictionary<string, object?>());
            }

            if (names.Count == 0)
            {
                return wait ? WaitAll(names, timeout, stopOthersOnError, returnExceptions, cancellationToken) : null;
            }

            cancellationToken.ThrowIfCancellationRequested();

            // The core checks every move against its unit (nothing sent on a refusal), starts them, then waits for all.
            var native = names.Select(n => new NativeAxisMove(n, plan[n].Position)
            {
                Speed = plan[n].Speed,
                AccelG = plan[n].AccelG,
                Band = plan[n].Band,
                DecelG = Axis.Decel(plan[n].AccelG, plan[n].DecelG),
            }.ToNative()).ToArray();
            var (results, errors, operation) = Waiting(names, cancellationToken, () => Group(names, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_move_each(H, native, NativeText.Size(native.Length), Known.Flag(wait), Known.OrNaN(timeout), Known.Flag(stopOthersOnError), r, c, out n)));
            var phase = OperationName(operation, "WaitAll");
            if (phase.StartsWith("MoveEach", StringComparison.Ordinal))
            {
                phase = "MoveAll" + phase.Substring("MoveEach".Length);  // (0.8) the core's move_each runs this MoveAll
            }

            if (errors.Count > 0 && phase.StartsWith("MoveAll", StringComparison.Ordinal))
            {
                var started = phase.StartsWith("MoveAll (start)", StringComparison.Ordinal)
                    ? names.Where(n => !errors.ContainsKey(n)).ToDictionary(n => n, n => (object?)true)
                    : new Dictionary<string, object?>();
                throw new CellException(phase, errors, started);
            }

            ThrowIfCancelled(errors, cancellationToken);
            return wait ? Finish("WaitAll", names, Present(results), errors, returnExceptions) : null;
        }

        /// <summary>
        /// Wait until every axis (default: all) is in position and settled; each axis polls in its own task. An axis started by
        /// <see cref="MoveAll(IEnumerable{AxisMove}, bool, double?, bool, bool, CancellationToken)"/> is checked against its
        /// target and band; others just have to come to rest in position. <paramref name="timeout"/> is per axis (default: from
        /// the move; 10 s for an axis without a recorded move). A failure stops the others (<paramref name="stopOthersOnError"/>);
        /// cancelling stops every axis still waiting.
        /// </summary>
        public CellResults<Status> WaitAll(
            IEnumerable<string>? axes = null,
            double? timeout = null,
            bool stopOthersOnError = true,
            bool returnExceptions = false,
            CancellationToken cancellationToken = default)
        {
            CheckOpen();
            var names = Select(axes);
            var (results, errors, _) = Waiting(names, cancellationToken, () => Group(names, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_wait_all(H, a, Known.OrNaN(timeout), Known.Flag(stopOthersOnError), r, c, out n)));
            ThrowIfCancelled(errors, cancellationToken);
            return Finish("WaitAll", names, Present(results), errors, returnExceptions);
        }

        /// <summary>
        /// Decel STOP on every axis (default: all), as fast as possible, then wait until each has stopped. First the STOP coil
        /// is written ON to every axis (ports in parallel, the axes of a port back to back, nothing else in between), then each
        /// axis holds STOP until it stands still and releases it (<see cref="Axis.Stop"/>, axes in parallel). Every axis is
        /// attempted even if some fail; the failures are raised together afterwards (<see cref="CellException"/>) unless
        /// <paramref name="returnExceptions"/>. Safe to call from another thread while a move waits. A listed name this cell
        /// does not have never holds back the STOP: every listed axis it has is stopped, then each unknown name is an error
        /// of its own (<see cref="KeyNotFoundException"/>), listed after the axes.
        /// </summary>
        public CellResults<Status> StopAll(IEnumerable<string>? axes = null, double timeout = 5.0, bool returnExceptions = false)
        {
            CheckOpen();
            var (names, unknown) = SelectKnown(axes);
            var (results, errors, _) = Group(names, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_stop_all(H, a, timeout, 0, r, c, out n));
            var listed = names.ToList();
            foreach (var name in unknown)
            {
                errors[name] = new KeyNotFoundException($"no axis '{name}' in this cell (axes: {string.Join(", ", _names)})");
                listed.Add(name);
            }

            return Finish("StopAll", listed, Present(results), errors, returnExceptions);
        }

        // ------------------------------------------------------------------ lifecycle

        /// <summary>Release STOP coils left ON (best effort) and close every port. Safe to call twice.</summary>
        public void Close()
        {
            lock (_sync)
            {
                if (_closed)
                {
                    return;
                }

                _closed = true;
            }

            try
            {
                foreach (var port in _portList)
                {
                    SafeClose(port);
                }
            }
            finally
            {
                _core?.Close();  // iad_cell_release: the ports and axes were closed by their own objects
            }
        }

        /// <summary>Same as <see cref="Close"/>: it does not stop moving axes (see the class remarks).</summary>
        public void Dispose() => Close();

        // ------------------------------------------------------------------ helpers

        internal static double MoveTimeout(double distance, double speed, double accelG)
        {
            var accel = Math.Max(accelG, 0.01) * Registers.G_MM_S2;
            var v = Math.Max(speed, 0.01);
            var d = Math.Max(distance, 0.0);
            var ramp = v * v / accel;
            var t = d >= ramp ? (d / v) + (v / accel) : 2 * Math.Sqrt(d / accel);
            return (2 * t) + 2.0;
        }

        private static PortSpec NamedSpec(string name, IReadOnlyList<int>? listed, CellOptions o)
        {
            if (string.IsNullOrWhiteSpace(name))
            {
                throw new ArgumentException("port name required");
            }

            return new PortSpec(name, listed, () =>
            {
                var rate = o.BaudRate ?? ModbusRtu.DefaultBaudRate;
                var transport = o.SerialOpener != null ? o.SerialOpener(name, rate, o.ReadTimeout) : new SerialTransport(name, rate, o.ReadTimeout);
                try
                {
                    return new Port(transport, name, o.Retries, null, PortLogger(name, o), autoBaud: o.BaudRate is null, o.Line);
                }
                catch
                {
                    transport.Dispose();  // the port that never took it would keep the serial port open
                    throw;
                }
            });
        }

        private static Action<FrameDirection, byte[]>? PortLogger(string name, CellOptions o)
        {
            var logger = o.FrameLogger;
            return logger is null ? null : (Action<FrameDirection, byte[]>)((d, f) => logger(name, d, f));
        }

        private static Cell OpenCore(IReadOnlyList<PortSpec> specs, bool strict, CellOptions o)
        {
            if (o.MaxAxes < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(o.MaxAxes), o.MaxAxes,
                    "MaxAxes must be >= 1, or 0 for no limit");
            }

            if (!Enum.IsDefined(typeof(LineKind), o.Line))  // before any port is opened: a port error would swallow it
            {
                throw new ArgumentOutOfRangeException(nameof(o.Line), o.Line, "Line must be a LineKind");
            }

            if (specs.Count == 0)
            {
                throw new EcNotReadyException("no port given");
            }

            var dup = specs.GroupBy(s => s.Name).FirstOrDefault(g => g.Count() > 1);
            if (dup != null)
            {
                throw new ArgumentException($"port {dup.Key} given twice");
            }

            var scanList = (o.ScanSlaves ?? PortManager.DefaultSlaves).Distinct().ToList();
            foreach (var s in scanList.Concat(specs.SelectMany(sp => sp.Listed ?? Array.Empty<int>())))
            {
                if (s < 1 || s > 247)
                {
                    throw new ArgumentOutOfRangeException(nameof(o.ScanSlaves), s, "slave address must be 1..247");
                }
            }

            if (o.ScanTimeout.HasValue && o.ScanTimeout.Value <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(o.ScanTimeout), "scan timeout must be > 0");
            }

            // Opening the ports stays here (the test seams, a port that cannot be opened is a port error); the core does the rest.
            var opened = new List<(PortSpec Spec, Port Port)>();
            var errors = new Dictionary<string, Exception>();
            foreach (var spec in specs)
            {
                try
                {
                    opened.Add((spec, spec.Create()));
                }
                catch (Exception exc)
                {
                    errors[spec.Name] = exc;
                }
            }

            void CloseOpened()
            {
                foreach (var (_, port) in opened)
                {
                    SafeClose(port);
                }
            }

            var unexpected = errors.Values.FirstOrDefault(e => !IsPortError(e));
            if (unexpected != null)
            {
                CloseOpened();
                ExceptionDispatchInfo.Capture(unexpected).Throw();
            }

            if (errors.Count > 0 && strict)
            {
                CloseOpened();
                var first = errors.Values.First();
                if (errors.Count > 1)
                {
                    Note(first, "other ports failed too: " + string.Join("; ", errors.Select(kv => $"{kv.Key}: {kv.Value.Message}")));
                }

                ExceptionDispatchInfo.Capture(first).Throw();
            }

            foreach (var kv in errors)
            {
                DriverLog.Warning(LogSource, $"port {kv.Key} skipped: {kv.Value.Message}");
            }

            if (opened.Count == 0)
            {
                throw new EcNotReadyException($"no axis found ({string.Join("; ", errors.Select(kv => $"{kv.Key}: {kv.Value.Message}"))})");
            }

            // The core: each port's rate and the scan or the listed slaves (ports in parallel), MaxAxes, the axes.
            var slavesJson = strict
                ? "{" + string.Join(",", opened.Select(op => AxisRecords.Quote(op.Spec.Name) + ":[" + NativeBus.Csv(op.Spec.Listed ?? Array.Empty<int>()) + "]")) + "}"
                : null;
            var cellOptions = CoreOptions(o.MaxAxes, o.ScanTimeout);
            var axisOptions = SizedStruct.Fresh<IadAxisOptions>();
            NativeErrors.Check(NativeMethods.iad_axis_options_init(ref axisOptions));
            try
            {
                foreach (var (_, port) in opened)
                {
                    port.Bus.PrepareScan();  // a managed transport's read timeout as it is now: the probes' wait caps it
                }
            }
            catch
            {
                CloseOpened();
                throw;
            }

            CellHandle handle = null!;
            var result = WithTokens(opened.Select(op => (SafeHandle)op.Port.Bus.Native.Handle).ToList(), tokens =>
                NativeMethods.iad_cell_open_buses(tokens, NativeText.Size(tokens.Length), Utf8.Encode(slavesJson), Utf8.Encode(NativeBus.Csv(scanList)), ref cellOptions, ref axisOptions, out handle));
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                CloseOpened();
                if (errors.Count > 0)
                {
                    Note(error, "ports not opened: " + string.Join("; ", errors.Select(kv => $"{kv.Key}: {kv.Value.Message}")));
                }

                ExceptionDispatchInfo.Capture(error).Throw();
            }

            handle.ReleaseOnly = true;  // the axes and ports close with their own objects (Close)
            var byName = opened.ToDictionary(op => op.Spec.Name, op => op.Port);
            var axes = new List<(string Name, Axis Axis)>();
            try
            {
                var skipped = MiniJson.Parse(NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_port_errors_json(handle, b, s, out r))) as JsonObject;
                foreach (var name in skipped?.Keys ?? Array.Empty<string>())
                {
                    errors[name] = skipped!.Object(name) is JsonObject details ? NativeErrors.FromDetails(details) : new EcDriverException(name + ": skipped");
                }

                NativeErrors.Check(NativeMethods.iad_cell_axis_count(handle, out var count));
                var existing = false;
                for (var i = 0; i < NativeText.ToInt(count); i++)
                {
                    var index = NativeText.Size(i);
                    var name = NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_axis_name(handle, index, b, s, out r), 256);
                    var cut = name.LastIndexOf(':');
                    var slave = int.Parse(name.Substring(cut + 1), NumberStyles.Integer, CultureInfo.InvariantCulture);
                    var key = Utf8.EncodeRequired(name, nameof(name));
                    var core = NativeAxis.Adopt(NativeMethods.iad_cell_axis_ref(handle, key, out var axisHandle), axisHandle, name);
                    var axis = byName[name.Substring(0, cut)].AxisOf(slave, o.AxisOptions, core);
                    existing |= !ReferenceEquals(axis.Core, core);
                    axes.Add((name, axis));
                }

                if (existing)
                {
                    // A Port's axis that existed already keeps its own core axis (its caps, stroke and bookkeeping): the
                    // group commands must run on it, so the cell is gathered from the axes' cores (iad_cell_open_axes).
                    var gathered = Gather(axes.Select(a => a.Name).ToList(), axes.ToDictionary(a => a.Name, a => a.Axis));
                    handle.Dispose();  // released: the scan's own axes the binding did not take go with it
                    handle = gathered!;
                }
            }
            catch
            {
                handle.Dispose();
                CloseOpened();
                throw;
            }

            var used = opened.Select(op => op.Port).Where(p => axes.Any(a => ReferenceEquals(a.Axis.Bus, p.Bus))).ToList();
            foreach (var (_, port) in opened)
            {
                if (!used.Contains(port))
                {
                    SafeClose(port);  // the core closed its bus already
                }
            }

            var ordered = specs.Where(sp => errors.ContainsKey(sp.Name)).ToDictionary(sp => sp.Name, sp => errors[sp.Name]);
            var cell = new Cell(used, axes, handle) { _portErrors = ordered };
            if (o.Identify)
            {
                try
                {
                    cell.IdentifyAll();
                }
                catch
                {
                    cell.Close();
                    throw;
                }
            }

            return cell;
        }

        /// <summary>A core cell gathering axes that exist already (iad_cell_open_axes; nothing sent); null without any.</summary>
        private static CellHandle? Gather(IReadOnlyList<string> names, IReadOnlyDictionary<string, Axis> axes)
        {
            if (names.Count == 0)
            {
                return null;
            }

            var options = CoreOptions(0, null);  // no limit
            CellHandle handle = null!;
            var result = WithTokens(names.Select(n => (SafeHandle)axes[n].Core.Handle).ToList(), tokens =>
                NativeMethods.iad_cell_open_axes(tokens, NativeText.Size(tokens.Length), Utf8.Encode(string.Join(",", names)), ref options, out handle));
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            handle.ReleaseOnly = true;  // the axes and ports are the caller's objects
            return handle;
        }

        /// <summary>The core cell's options: the most axes (0 = no limit) and the probe wait; the binding identifies the axes itself.</summary>
        private static IadCellOptions CoreOptions(int maxAxes, TimeSpan? scanTimeout)
        {
            var o = SizedStruct.Fresh<IadCellOptions>();
            NativeErrors.Check(NativeMethods.iad_cell_options_init(ref o));
            o.MaxAxes = maxAxes;
            if (scanTimeout.HasValue)
            {
                o.ScanTimeoutS = scanTimeout.Value.TotalSeconds;
            }

            o.Identify = 0;
            return o;
        }

        /// <summary>Call <paramref name="call"/> with the tokens of <paramref name="handles"/>, each kept alive for the call.</summary>
        private static int WithTokens(IReadOnlyList<SafeHandle> handles, Func<IntPtr[], int> call)
        {
            var tokens = new IntPtr[handles.Count];
            var added = new bool[handles.Count];
            try
            {
                for (var i = 0; i < handles.Count; i++)
                {
                    handles[i].DangerousAddRef(ref added[i]);
                    tokens[i] = handles[i].DangerousGetHandle();
                }

                TransportBridge.ClearError();
                return call(tokens);
            }
            finally
            {
                for (var i = 0; i < handles.Count; i++)
                {
                    if (added[i])
                    {
                        handles[i].DangerousRelease();
                    }
                }
            }
        }

        /// <summary>The errors a port can fail with (anything else is a bug and is raised).</summary>
        private static bool IsPortError(Exception exc) =>
            exc is EcDriverException || exc is IOException || exc is UnauthorizedAccessException || exc is ArgumentException
            || exc is InvalidOperationException || exc is TimeoutException;

        private static void SafeClose(Port port)
        {
            try
            {
                port.Close();
            }
            catch (Exception exc)
            {
                DriverLog.Warning(LogSource, $"closing {port.Name}: {exc.Message}");
            }
        }

        private static void Note(Exception exc, string note)
        {
            DriverLog.Info(LogSource, note);
            if (exc is EcDriverException driverExc)
            {
                driverExc.AddNote(note);
                return;
            }

            try
            {
                exc.Data["IaiActuatorDriver.Notes"] = exc.Data["IaiActuatorDriver.Notes"] is string earlier ? earlier + "; " + note : note;
            }
            catch (Exception)
            {
                // a read-only Data dictionary: the note is only logged
            }
        }

        private void CheckOpen()
        {
            if (IsClosed)
            {
                throw new EcNotReadyException("cell is closed");
            }
        }

        private string CheckName(string name)
        {
            if (name is null || !_axes.ContainsKey(name))
            {
                throw new KeyNotFoundException($"no axis '{name}' in this cell (axes: {string.Join(", ", _names)})");
            }

            return name;
        }

        private List<string> Select(IEnumerable<string>? axes)
        {
            if (axes is null)
            {
                return _names.ToList();
            }

            var list = new List<string>();
            foreach (var name in axes)
            {
                CheckName(name);
                if (!list.Contains(name))
                {
                    list.Add(name);
                }
            }

            return list;
        }

        /// <summary>
        /// <see cref="Select"/> of the stop path (<see cref="StopAll"/>): the listed axes this cell has, and apart the names it
        /// does not have (an axis that dropped out of a reopened cell), reported once the others were stopped: a STOP never
        /// waits on an unrelated bad name.
        /// </summary>
        private (List<string> Known, List<string> Unknown) SelectKnown(IEnumerable<string>? axes)
        {
            if (axes is null)
            {
                return (_names.ToList(), new List<string>());
            }

            var known = new List<string>();
            var unknown = new List<string>();
            foreach (var name in axes)
            {
                var key = name ?? string.Empty;
                var to = _axes.ContainsKey(key) ? known : unknown;
                if (!to.Contains(key))
                {
                    to.Add(key);
                }
            }

            return (known, unknown);
        }

        /// <summary>One group call of the core: the axes as one string, the results array, its capacity and the count handled.</summary>
        private delegate int GroupCall(byte[]? axes, IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        /// <summary>
        /// One group command of the core over <paramref name="names"/> (ports in parallel on the core's threads): each axis's
        /// status (null when the core kept none, e.g. a move started without waiting) or its error, in the order of
        /// <paramref name="names"/>, and the core's operation when an axis failed ("move_all (start)").
        /// </summary>
        private (Dictionary<string, Status?> Results, Dictionary<string, Exception> Errors, string? Operation) Group(IReadOnlyList<string> names, GroupCall call)
        {
            var results = new Dictionary<string, Status?>();
            var errors = new Dictionary<string, Exception>();
            if (names.Count == 0)
            {
                return (results, errors, null);
            }

            var buffer = new IadCellResult[names.Count];
            var axes = Utf8.Encode(string.Join(",", names));
            var mark = TransportBridge.FailureMark;
            TransportBridge.ClearError();
            var code = call(axes, buffer, NativeText.Size(buffer.Length), out var count);
            if (code != 0 && code != (int)NativeResult.Cell)
            {
                throw TransportBridge.Raise(code);
            }

            // Per-axis errors: the last-error JSON has each one's details, the struct a message cut at 256 bytes.
            var details = code == (int)NativeResult.Cell ? MiniJson.TryParse(NativeErrors.LastErrorJson()) as JsonObject : null;
            var perAxis = details?.Object("axes");
            var handled = new Dictionary<string, IadCellResult>();
            for (var i = 0; i < Math.Min(NativeText.ToInt(count), buffer.Length); i++)
            {
                // (0.8) a name longer than the result's field comes back cut: its index in the request is the key
                handled[SizedStruct.NameOf(AxisOf(buffer[i]), buffer[i].Index, names)] = buffer[i];
            }

            foreach (var name in names)
            {
                if (!handled.TryGetValue(name, out var r))
                {
                    continue;
                }

                if (r.Code == 0)
                {
                    results[name] = _axes[name].StatusOf(r.Status);
                }
                else
                {
                    errors[name] = AxisError(name, perAxis?.Object(name), r.Code, MessageOf(r), mark);
                }
            }

            return (results, errors, details?.String("operation"));
        }

        /// <summary>
        /// One axis's error of a group command: the core's details, or for a failure of a managed transport (on the core's
        /// thread of the port) what the transport threw, as the axis's own call raises it (<see cref="TransportBridge.Raise"/>:
        /// <see cref="ModbusException"/> "transport error: ..." around the original, or the original itself).
        /// </summary>
        private Exception AxisError(string name, JsonObject? details, int code, string message, long mark)
        {
            var error = details != null ? NativeErrors.FromDetails(details) : NativeErrors.Create(code, message, null);
            var result = details?.Int("code") ?? code;
            if ((result == (int)NativeResult.Transport || result == (int)NativeResult.Driver) && _axes[name].Bus.Bridge is TransportBridge bridge)
            {
                return bridge.FailureSince(mark, details?.String("message") ?? message, result, null) ?? error;
            }

            return error;
        }

        /// <summary>A read-only group read of the core (iad_cell_read_all_json): each axis's JSON or its error.</summary>
        private (Dictionary<string, JsonObject> Results, Dictionary<string, Exception> Errors) ReadAll(string what, IReadOnlyList<string> names)
        {
            var results = new Dictionary<string, JsonObject>();
            var errors = new Dictionary<string, Exception>();
            if (names.Count == 0)
            {
                return (results, errors);
            }

            var w = Utf8.EncodeRequired(what, nameof(what));
            var axes = Utf8.Encode(string.Join(",", names));
            var mark = TransportBridge.FailureMark;
            var j = MiniJson.Parse(NativeBus.Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_read_all_json(H, w, axes, b, s, out r))) as JsonObject;
            var found = j?.Object("results");
            var failed = j?.Object("errors");
            foreach (var name in names)
            {
                if (found?.Object(name) is JsonObject value)
                {
                    results[name] = value;
                }
                else if (failed?.Object(name) is JsonObject error)
                {
                    errors[name] = AxisError(name, error, error.Int("code") ?? (int)NativeResult.Driver, error.String("message") ?? string.Empty, mark);
                }
            }

            return (results, errors);
        }

        /// <summary>
        /// A group command that may wait: <paramref name="cancellationToken"/> latches the core's cancel on these axes (each
        /// waiting axis sends its decel stop and ends), cleared again when the call returns. A token cancelled before the call
        /// stops the axes and raises <see cref="OperationCanceledException"/>.
        /// </summary>
        private T Waiting<T>(IReadOnlyList<string> names, CancellationToken cancellationToken, Func<T> call)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                if (names.Count > 0)
                {
                    StopAll(names, returnExceptions: true);
                }

                throw new OperationCanceledException($"cancelled; decel stop sent to {string.Join(", ", names)}", cancellationToken);
            }

            if (!cancellationToken.CanBeCanceled || names.Count == 0)
            {
                return call();
            }

            var axes = Utf8.Encode(string.Join(",", names));
            var latched = 0;
            T result;
            using (cancellationToken.Register(() =>
            {
                Interlocked.Exchange(ref latched, 1);
                try
                {
                    NativeMethods.iad_cell_cancel_latch(H, axes);
                }
                catch (Exception exc) when (exc is ObjectDisposedException || exc is EcNotReadyException)
                {
                    // closed meanwhile: nothing waits any more
                }
            }))
            {
                result = call();
            }

            if (Volatile.Read(ref latched) != 0)
            {
                NativeMethods.iad_cell_cancel_reset(H, axes);
            }

            return result;
        }

        /// <summary>The axes that came back with a status sample.</summary>
        private static Dictionary<string, Status> Present(Dictionary<string, Status?> results) =>
            results.Where(kv => kv.Value != null).ToDictionary(kv => kv.Key, kv => kv.Value!);

        /// <summary>This library's name of a core operation: "move_all (start)" -&gt; "MoveAll (start)"; <paramref name="fallback"/> without one.</summary>
        private static string OperationName(string? core, string fallback)
        {
            if (string.IsNullOrEmpty(core))
            {
                return fallback;
            }

            var cut = core!.IndexOf(' ');
            var head = cut < 0 ? core : core.Substring(0, cut);
            var tail = cut < 0 ? string.Empty : core.Substring(cut);
            return string.Concat(head.Split('_').Where(p => p.Length > 0).Select(p => char.ToUpperInvariant(p[0]) + p.Substring(1))) + tail;
        }

        private static unsafe string AxisOf(IadCellResult r) => Utf8.FromFixed(r.Axis, SizedStruct.AxisNameSize);

        private static unsafe string MessageOf(IadCellResult r) => Utf8.FromFixed(r.Message, 256);

        /// <summary>A cancelled wait raises <see cref="OperationCanceledException"/> (each cancelled axis already sent its decel stop).</summary>
        private static void ThrowIfCancelled(Dictionary<string, Exception> errors, CancellationToken cancellationToken)
        {
            if (!cancellationToken.IsCancellationRequested)
            {
                return;
            }

            var cancelled = errors.Where(kv => kv.Value is OperationCanceledException).Select(kv => kv.Key).ToList();
            if (cancelled.Count > 0)
            {
                throw new OperationCanceledException($"cancelled; decel stop sent to {string.Join(", ", cancelled)}", cancellationToken);
            }
        }

        private static CellResults<T> Finish<T>(string operation, IReadOnlyList<string> names, Dictionary<string, T> results, Dictionary<string, Exception> errors, bool returnExceptions)
        {
            if (errors.Count > 0 && !returnExceptions)
            {
                throw new CellException(operation, errors, results.ToDictionary(kv => kv.Key, kv => (object?)kv.Value));
            }

            return new CellResults<T>(operation, names.ToList().AsReadOnly(), results, errors);
        }

        private sealed class PortSpec
        {
            public PortSpec(string name, IReadOnlyList<int>? listed, Func<Port> create)
            {
                Name = name;
                Listed = listed;
                Create = create;
            }

            public string Name { get; }

            public IReadOnlyList<int>? Listed { get; }

            public Func<Port> Create { get; }
        }
    }
}
