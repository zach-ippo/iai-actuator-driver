using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// Several axes over one or more ports, driven together, on the native core (iad_cell; C++ Cell). Method names follow
    /// the managed edition's <c>IaiActuatorDriver.Cell</c>. Group commands handle every listed axis before returning
    /// (a stop always goes out to all); without <c>returnExceptions</c> a failure on any axis throws
    /// <see cref="CellException"/> with each axis's error, otherwise the per-axis errors are in
    /// <see cref="NativeCellResults.Errors"/>.
    /// </summary>
    /// <remarks>
    /// <para>Axis lists: null = every axis of the cell; names as <see cref="Names"/> gives them ("COM3:1", "SIM1:2").</para>
    /// <para>Threads: <see cref="StopAll"/>, <see cref="PauseAll"/> and <see cref="Cancel"/> may be called from another
    /// thread while a group command waits. The events (<see cref="Fault"/>, <see cref="MotionDone"/>...) are raised on a
    /// thread the binding owns.</para>
    /// <para>Evidence: VERIFIED on the PCON-CB with an RCP6 (steps in docs/NATIVE_ABI.md), tested on the native simulator
    /// besides; everything else UNVERIFIED.</para>
    /// </remarks>
    internal sealed partial class NativeCell : IDisposable, IEnumerable<NativeAxis>
    {
        private readonly CellHandle _handle;
        private readonly List<string> _names;
        private readonly Dictionary<string, NativeAxis> _axes = new Dictionary<string, NativeAxis>(StringComparer.Ordinal);
        private readonly object _gate = new object();

        private NativeCell(CellHandle handle)
        {
            _handle = handle;
            NativeErrors.Check(NativeMethods.iad_cell_axis_count(_handle, out var count));
            var total = NativeText.ToInt(count);
            _names = new List<string>(total);
            for (var i = 0; i < total; i++)
            {
                var index = NativeText.Size(i);
                _names.Add(NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_axis_name(_handle, index, b, s, out r), 256));
            }
        }

        // ------------------------------------------------------------------------------------------ open / close

        /// <summary>
        /// Open a cell on every IAI USB adapter of the PC, each port scanned (iad_cell_open with NULL ports).
        /// <see cref="EcNotReadyException"/> when no axis at all is found.
        /// </summary>
        public static NativeCell Open(NativeCellOptions? options = null) => Open((string?)null, options);

        /// <summary>
        /// Open a cell (Cell::open). <paramref name="ports"/>: "COM3", "COM4" = these ports, scanned (options ScanFirst ..
        /// ScanLast); "COM3:1", "COM3:2", "COM4:1" = exactly these slaves (each must answer). Either every entry names a
        /// slave or none does. Ports that cannot be opened or where nothing answers are skipped when scanning
        /// (<see cref="InfoJson"/> "port_errors"); <see cref="EcNotReadyException"/> when no axis at all is found.
        /// </summary>
        public static NativeCell Open(IEnumerable<string> ports, NativeCellOptions? options = null) =>
            Open(JoinNames(ports ?? throw new ArgumentNullException(nameof(ports))), options);

        /// <summary>As <see cref="Open(IEnumerable{string}, NativeCellOptions?)"/> with one comma-separated string ("COM3,COM4"); null or "" = every IAI USB adapter.</summary>
        public static NativeCell Open(string? ports, NativeCellOptions? options = null)
        {
            NativeDriver.EnsureLoaded();
            var p = Utf8.Encode(string.IsNullOrEmpty(ports) ? null : ports, nameof(ports));
            var cellOptions = NativeCellOptions.ToNative(options);
            var axisOptions = (options?.AxisOptions ?? new NativeAxisOptions()).ToNative();
            var result = NativeMethods.iad_cell_open(p, ref cellOptions, ref axisOptions, out var handle);
            return Adopt(result, handle);
        }

        /// <summary>
        /// A cell on simulated lines of the native core (iad_cell_open_sim): one line "SIM1".."SIMn" per entry of
        /// <paramref name="profiles"/> (null = one "slider" line), each with <paramref name="slavesPerLine"/> units (1..16)
        /// of that profile at addresses 1..n. Axis names "SIM1:1"... No hardware, never opens a port.
        /// </summary>
        public static NativeCell OpenSim(IEnumerable<string>? profiles = null, int slavesPerLine = 1, NativeSimOptions? sim = null, NativeCellOptions? options = null)
        {
            NativeDriver.EnsureLoaded();
            var p = Utf8.Encode(profiles is null ? null : JoinNames(profiles), nameof(profiles));
            var simOptions = NativeSimOptions.ToNative(sim);
            var cellOptions = NativeCellOptions.ToNative(options);
            var axisOptions = (options?.AxisOptions ?? new NativeAxisOptions()).ToNative();
            var result = NativeMethods.iad_cell_open_sim(p, slavesPerLine, ref simOptions, ref cellOptions, ref axisOptions, out var handle);
            return Adopt(result, handle);
        }

        /// <summary>
        /// The owner of a cell the binding opened itself (the public <see cref="IaiActuatorDriver.Cell"/>'s core cell; null
        /// stays null). Closing it closes the borrowed axes and lets go of <paramref name="handle"/> as the handle says
        /// (<see cref="CellHandle.ReleaseOnly"/>); on failure the handle is let go of.
        /// </summary>
        internal static NativeCell? Own(CellHandle? handle) => handle is null ? null : Adopt(0, handle);

        private static NativeCell Adopt(int result, CellHandle handle)
        {
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            try
            {
                return new NativeCell(handle);
            }
            catch
            {
                handle.Dispose();
                throw;
            }
        }

        /// <summary>
        /// Release every STOP / pause the axes hold, close every port and release the handle (Cell::close). The axis
        /// objects of this cell are closed first. Safe to call twice.
        /// </summary>
        public void Close()
        {
            StopEventPump();
            lock (_gate)
            {
                foreach (var axis in _axes.Values)
                {
                    axis.Close();
                }

                _axes.Clear();
            }

            _handle.Dispose();
        }

        /// <inheritdoc cref="Close"/>
        public void Dispose() => Close();

        /// <summary>The cell was closed.</summary>
        public bool IsClosed => _handle.IsClosed;

        /// <summary>The native handle (for calls this class does not wrap).</summary>
        public CellHandle Handle => _handle;

        // ------------------------------------------------------------------------------------------ axes

        /// <summary>Axis names, port order then slave order ("COM3:1", "SIM1:2"...).</summary>
        public IReadOnlyList<string> Names => _names.AsReadOnly();

        /// <summary>Number of axes.</summary>
        public int Count => _names.Count;

        /// <summary>
        /// The axis <paramref name="name"/> (iad_cell_open_axis): every <see cref="NativeAxis"/> method works on it. The cell
        /// owns it: it is closed with the cell (disposing it only releases its handle; the axis stays in the cell).
        /// </summary>
        public NativeAxis this[string name]
        {
            get
            {
                var key = Utf8.EncodeRequired(name, nameof(name));
                lock (_gate)
                {
                    if (_axes.TryGetValue(name, out var cached) && !cached.IsClosed)
                    {
                        return cached;
                    }

                    var result = NativeMethods.iad_cell_open_axis(_handle, key, out var handle);
                    var axis = NativeAxis.Adopt(result, handle, name);
                    _axes[name] = axis;
                    return axis;
                }
            }
        }

        /// <summary>The axis at <paramref name="index"/> of <see cref="Names"/>.</summary>
        public NativeAxis this[int index] => this[_names[index]];

        /// <summary>The cell has an axis of this name.</summary>
        public bool Contains(string name) => name != null && _names.Contains(name);

        /// <inheritdoc />
        public IEnumerator<NativeAxis> GetEnumerator() => _names.Select(n => this[n]).GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        /// <inheritdoc />
        public override string ToString() => $"NativeCell({string.Join(", ", _names)})";

        /// <summary>
        /// The cell as JSON, no I/O: {"axes": [names], "info": {name: identity}, "port_errors": {port: message},
        /// "faults": {name: "port_vanished" | "unit_silent"}}. (0.2) The faults are also raised as <see cref="Fault"/>.
        /// </summary>
        public string InfoJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_info_json(_handle, b, s, out r));

        /// <summary>Forget a fault of one axis (Cell::clear_fault).</summary>
        public void ClearFault(string axis) =>
            NativeErrors.Check(NativeMethods.iad_cell_clear_fault(_handle, Utf8.EncodeRequired(axis, nameof(axis))));

        // ------------------------------------------------------------------------------------------ group commands

        /// <summary>One status read per axis (Cell::status_all; ports in parallel).</summary>
        public NativeCellResults StatusAll(IEnumerable<string>? axes = null, bool returnExceptions = false) =>
            Group("StatusAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_status_all(_handle, a, double.NaN, r, c, out n));

        /// <summary>Servo on / off on every listed axis and wait for each (Cell::servo_all). <paramref name="timeout"/> null = the core's default.</summary>
        public NativeCellResults ServoAll(bool on, IEnumerable<string>? axes = null, double? timeout = null, bool returnExceptions = false) =>
            Group("ServoAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_servo_all(_handle, Known.Flag(on), a, Known.OrNaN(timeout), r, c, out n));

        /// <summary>Home every listed axis at once and wait for all (Cell::home_all). <paramref name="timeout"/> null = the core's default.</summary>
        public NativeCellResults HomeAll(IEnumerable<string>? axes = null, double? timeout = null, bool returnExceptions = false) =>
            Group("HomeAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_home_all(_handle, a, Known.OrNaN(timeout), 0, 1, r, c, out n));  // the axis's method; stop the others on an error

        /// <summary>
        /// Absolute moves with one speed, acceleration and band for every axis in <paramref name="targets"/> (axis name -&gt;
        /// position, each in its axis's own units), started with as little delay as possible (Cell::move_all). Every move
        /// is checked first: one bad value refuses the whole move, nothing sent (<see cref="CellException"/> with the
        /// per-axis errors). A start that fails stops the axes already started. With <paramref name="wait"/> false this
        /// returns null once every move started: call <see cref="WaitAll"/>. (0.2) <paramref name="decelG"/>: a
        /// deceleration other than <paramref name="accelG"/> (the command-profile path).
        /// </summary>
        public NativeCellResults? MoveAll(
            IEnumerable<KeyValuePair<string, double>> targets,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            bool returnExceptions = false,
            double? decelG = null)
        {
            var moves = (targets ?? throw new ArgumentNullException(nameof(targets))).ToList();
            if (moves.Count == 0)
            {
                throw new ArgumentException("no axis to move", nameof(targets));
            }

            var names = moves.Select(m => m.Key).ToList();
            var positions = moves.Select(m => m.Value).ToArray();
            var o = NativeAxis.MoveOptions(speed, accelG, band, wait, timeout, jog: false, decelG);
            var results = Group("MoveAll", names, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_move_all(_handle, a!, positions, NativeText.Size(positions.Length), ref o, null, null, null, null, r, c, out n));
            return wait ? results : null;
        }

        /// <summary>
        /// Wait until every listed axis is in position and settled (Cell::wait_all; an axis started by
        /// <see cref="MoveAll(IEnumerable{KeyValuePair{string, double}}, double, double, double, bool, double?, bool, double?)"/>:
        /// at its target and band). <paramref name="timeout"/> null = per axis from the move's estimate, 10 s otherwise.
        /// </summary>
        public NativeCellResults WaitAll(IEnumerable<string>? axes = null, double? timeout = null, bool returnExceptions = false) =>
            Group("WaitAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_wait_all(_handle, a, Known.OrNaN(timeout), 1, r, c, out n));

        /// <summary>
        /// Decel stop on every listed axis as fast as possible (Cell::stop_all: STOP to all first, then each waits).
        /// <paramref name="timeout"/> s per axis. Safe to call from another thread while a group command waits.
        /// </summary>
        public NativeCellResults StopAll(IEnumerable<string>? axes = null, double timeout = 5.0, bool returnExceptions = false) =>
            Group("StopAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_stop_all(_handle, a, timeout, 0, r, c, out n));

        /// <summary>One group call: the axes as one string, the results array, its capacity and the count handled.</summary>
        private delegate int GroupCall(byte[]? axes, IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        private NativeCellResults Group(string operation, IEnumerable<string>? axes, bool returnExceptions, GroupCall call)
        {
            var list = axes?.ToList();
            var joined = list is null ? null : JoinNames(list);
            var capacity = Math.Max(list?.Count ?? _names.Count, _names.Count);
            var buffer = new IadCellResult[capacity];
            var code = call(Utf8.Encode(joined, nameof(axes)), buffer, NativeText.Size(buffer.Length), out var count);
            if (code != 0 && code != (int)NativeResult.Cell)
            {
                throw NativeErrors.Capture(code);
            }

            // Per-axis messages: the JSON has the full text, the struct a copy cut at 256 bytes.
            string? json = null;
            JsonObject? perAxis = null;
            if (code == (int)NativeResult.Cell)
            {
                json = NativeErrors.LastErrorJson();
                perAxis = (MiniJson.TryParse(json) as JsonObject)?.Object("axes");
            }

            var handled = Math.Min(NativeText.ToInt(count), buffer.Length);
            var requested = list is null ? _names : list.Select(n => n.Trim()).ToList();
            var results = NativeCellResults.From(operation, buffer, handled, perAxis, requested);
            if (code == 0 || returnExceptions)
            {
                return results;
            }

            var errors = results.Errors;
            var succeeded = results.ToDictionary(kv => kv.Key, kv => (object?)kv.Value, StringComparer.Ordinal);
            var exception = new CellException(operation, errors, succeeded);
            NativeErrors.Tag(exception, code, "CellError", json);
            throw exception;
        }

        /// <summary>Names as the ABI's one comma-separated string; a name with a comma is refused.</summary>
        private static string JoinNames(IEnumerable<string> names)
        {
            var list = new List<string>();
            foreach (var name in names)
            {
                if (string.IsNullOrEmpty(name) || name.IndexOf(',') >= 0)
                {
                    throw new ArgumentException($"invalid name '{name}': empty or with a comma");
                }

                list.Add(name.Trim());
            }

            return string.Join(",", list);
        }
    }

    /// <summary>
    /// Per-axis outcome of a group command: a read-only dictionary of the axes that succeeded (in the order handled) with
    /// their status, plus <see cref="Errors"/> for the ones that failed (only ever non-empty with
    /// <c>returnExceptions: true</c>).
    /// </summary>
    internal sealed class NativeCellResults : IReadOnlyDictionary<string, NativeStatus>
    {
        private readonly Dictionary<string, NativeStatus> _results;

        private NativeCellResults(string operation, IReadOnlyList<string> names, Dictionary<string, NativeStatus> results, IReadOnlyDictionary<string, Exception> errors)
        {
            Operation = operation;
            Names = names;
            _results = results;
            Errors = errors;
        }

        /// <summary>The operation, e.g. "MoveAll".</summary>
        public string Operation { get; }

        /// <summary>Every axis handled, in order.</summary>
        public IReadOnlyList<string> Names { get; }

        /// <summary>The error of each failed axis (the managed edition's exception classes).</summary>
        public IReadOnlyDictionary<string, Exception> Errors { get; }

        /// <summary>Every axis succeeded.</summary>
        public bool Succeeded => Errors.Count == 0;

        public int Count => _results.Count;

        public IEnumerable<string> Keys => Names.Where(_results.ContainsKey);

        public IEnumerable<NativeStatus> Values => Keys.Select(k => _results[k]);

        public NativeStatus this[string name] =>
            _results.TryGetValue(name, out var status)
                ? status
                : throw (Errors.TryGetValue(name, out var error) ? error : new KeyNotFoundException($"no result for axis '{name}'"));

        public bool ContainsKey(string key) => _results.ContainsKey(key);

        public bool TryGetValue(string key, out NativeStatus value) => _results.TryGetValue(key, out value!);

        public IEnumerator<KeyValuePair<string, NativeStatus>> GetEnumerator() =>
            Keys.Select(k => new KeyValuePair<string, NativeStatus>(k, _results[k])).GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        /// <summary>
        /// The results of one group call, keyed by axis name. <paramref name="requested"/> is the list the call was given
        /// (all axes: the cell's): a name longer than the result's field comes back cut, and its index (the position in
        /// the request) names the axis (0.8).
        /// </summary>
        internal static unsafe NativeCellResults From(string operation, IadCellResult[] buffer, int count, JsonObject? perAxis, IReadOnlyList<string> requested)
        {
            var names = new List<string>(count);
            var results = new Dictionary<string, NativeStatus>(StringComparer.Ordinal);
            var errors = new Dictionary<string, Exception>(StringComparer.Ordinal);
            for (var i = 0; i < count; i++)
            {
                string name;
                string message;
                fixed (byte* axis = buffer[i].Axis)
                fixed (byte* text = buffer[i].Message)
                {
                    name = Utf8.FromFixed(axis, SizedStruct.AxisNameSize);
                    message = Utf8.FromFixed(text, 256);
                }

                name = SizedStruct.NameOf(name, buffer[i].Index, requested);

                names.Add(name);
                if (buffer[i].Code == 0)
                {
                    results[name] = new NativeStatus(buffer[i].Status);
                }
                else
                {
                    var full = perAxis?.Object(name)?.String("message");
                    errors[name] = NativeErrors.Create(buffer[i].Code, string.IsNullOrEmpty(full) ? message : full, null);
                }
            }

            return new NativeCellResults(operation, names, results, errors);
        }
    }
}
