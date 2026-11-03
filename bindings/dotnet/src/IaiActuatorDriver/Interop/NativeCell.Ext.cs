using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // (ABI 0.2) Per-axis moves, pause / resume / cancel, monitors and events, recovery, lines and gateways, backups and
    // coordinated motion of a cell. Evidence: ported from the Python edition; the C++ code, the ABI and this binding are
    // UNVERIFIED on hardware (simulator only).
    internal sealed partial class NativeCell
    {
        private readonly object _eventGate = new object();
        private NativeEventPump? _pump;

        // ------------------------------------------------------------------------------------------ moves

        /// <summary>
        /// Start an absolute move on every axis of <paramref name="moves"/>, each with its own speed, acceleration,
        /// deceleration, band and pattern (Cell::move_each; a deceleration other than the acceleration runs the
        /// command-profile path), checked first like <see cref="MoveAll(IEnumerable{KeyValuePair{string, double}}, double, double, double, bool, double?, bool, double?)"/>:
        /// one bad value refuses all, nothing sent. With <paramref name="wait"/> the call waits for all
        /// (<paramref name="timeout"/> null = per axis); otherwise it returns null once every move started.
        /// </summary>
        public NativeCellResults? MoveAll(IEnumerable<NativeAxisMove> moves, bool wait = true, double? timeout = null, bool returnExceptions = false)
        {
            var list = (moves ?? throw new ArgumentNullException(nameof(moves))).ToList();
            if (list.Count == 0)
            {
                throw new ArgumentException("no axis to move", nameof(moves));
            }

            var native = list.Select(m => (m ?? throw new ArgumentException("a move is null", nameof(moves))).ToNative()).ToArray();
            var results = Group("MoveAll", list.Select(m => m.Axis).ToList(), returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_move_each(_handle, native, NativeText.Size(native.Length), Known.Flag(wait), Known.OrNaN(timeout), 1, r, c, out n));
            return wait ? results : null;
        }

        /// <summary>
        /// Absolute moves with per-axis values (Cell::move_all with PerAxisMoveValues): <paramref name="speeds"/>,
        /// <paramref name="accelsG"/>, <paramref name="bands"/> and <paramref name="decelsG"/> map an axis name to its own
        /// value in that axis's units; an axis missing from a map takes <paramref name="speed"/> / <paramref name="accelG"/>
        /// / <paramref name="band"/> / <paramref name="decelG"/> (decel: else that axis's acceleration). An axis whose
        /// deceleration differs from its acceleration runs the command-profile path. Checks, refusal, wait and results as
        /// <see cref="MoveAll(IEnumerable{KeyValuePair{string, double}}, double, double, double, bool, double?, bool, double?)"/>.
        /// </summary>
        public NativeCellResults? MoveAllValues(
            IEnumerable<KeyValuePair<string, double>> targets,
            IReadOnlyDictionary<string, double>? speeds = null,
            IReadOnlyDictionary<string, double>? accelsG = null,
            IReadOnlyDictionary<string, double>? bands = null,
            IReadOnlyDictionary<string, double>? decelsG = null,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? decelG = null,
            bool wait = true,
            double? timeout = null,
            bool returnExceptions = false)
        {
            var moves = (targets ?? throw new ArgumentNullException(nameof(targets))).ToList();
            if (moves.Count == 0)
            {
                throw new ArgumentException("no axis to move", nameof(targets));
            }

            var names = moves.Select(m => m.Key).ToList();
            var positions = moves.Select(m => m.Value).ToArray();
            var s = PerAxis(names, speeds);
            var a0 = PerAxis(names, accelsG);
            var b = PerAxis(names, bands);
            var d = PerAxis(names, decelsG);
            var o = NativeAxis.MoveOptions(speed, accelG, band, wait, timeout, jog: false, decelG);
            var results = Group("MoveAll", names, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_move_all(_handle, a!, positions, NativeText.Size(positions.Length), ref o, s, a0, b, d, r, c, out n));
            return wait ? results : null;
        }

        /// <summary>The values of <paramref name="map"/> in the order of <paramref name="names"/> (NaN = the default); null for no map.</summary>
        private static double[]? PerAxis(IReadOnlyList<string> names, IReadOnlyDictionary<string, double>? map) =>
            map is null ? null : names.Select(n => map.TryGetValue(n, out var v) ? v : double.NaN).ToArray();

        /// <summary>
        /// Pause every listed axis, keeping the remaining travel (Cell::pause_all; ports in parallel). With
        /// <paramref name="broadcast"/>: first one broadcast pause on every daisy chain whose axes are all selected (never
        /// behind a gateway; UNVERIFIED), then the per-axis pause. The statuses are not read (all fields zero).
        /// </summary>
        public NativeCellResults PauseAll(IEnumerable<string>? axes = null, bool broadcast = false, bool returnExceptions = false) =>
            Group("PauseAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_pause_all(_handle, a, Known.Flag(broadcast), r, c, out n));

        /// <summary>Release the pause on every listed axis (Cell::resume_all: the paused moves continue); <paramref name="broadcast"/> as <see cref="PauseAll"/>.</summary>
        public NativeCellResults ResumeAll(IEnumerable<string>? axes = null, bool broadcast = false, bool returnExceptions = false) =>
            Group("ResumeAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_resume_all(_handle, a, Known.Flag(broadcast), r, c, out n));

        /// <summary>
        /// <see cref="NativeAxis.Cancel"/> on every listed axis: a group command waiting on them ends with
        /// <see cref="CellException"/> whose axes hold <see cref="OperationCanceledException"/>. Safe from any thread. A
        /// listed name the cell does not have never holds the cancel back: the axes it has are cancelled, then the call
        /// throws a <see cref="CellException"/> with each unknown name an <see cref="ArgumentOutOfRangeException"/> of its own.
        /// </summary>
        public void Cancel(IEnumerable<string>? axes = null) =>
            NativeErrors.Check(NativeMethods.iad_cell_cancel(_handle, Utf8.Encode(axes is null ? null : JoinNames(axes), nameof(axes))));

        // ------------------------------------------------------------------------------------------ monitors, events

        /// <summary>Every queued event of the cell (each names its axis), raised on the binding's event thread.</summary>
        public event EventHandler<NativeEvent>? EventReceived;

        /// <summary>A monitored axis's status changed.</summary>
        public event EventHandler<NativeEvent>? StatusChanged;

        /// <summary>A monitored axis became done by the fast rule against its last target.</summary>
        public event EventHandler<NativeEvent>? MotionDone;

        /// <summary>The cell's fault handling saw a link fault (name = the kind: "port_vanished", "unit_silent", "unit_restarted"...).</summary>
        public event EventHandler<NativeEvent>? Fault;

        /// <summary>A link event or a link monitor's state change (<see cref="MonitorLinks"/>).</summary>
        public event EventHandler<NativeEvent>? LinkChanged;

        /// <summary>The binding's event thread runs.</summary>
        public bool EventPumpRunning
        {
            get
            {
                lock (_eventGate)
                {
                    return _pump?.IsRunning == true;
                }
            }
        }

        /// <summary>
        /// Start the status monitor of every listed axis (Cell::start_monitors: their link events also go to the cell's
        /// fault handling) and queue their events on the cell. With <paramref name="raiseEvents"/> the binding's event
        /// thread raises them (and the cell's faults, queued from the open on).
        /// </summary>
        public void StartMonitors(IEnumerable<string>? axes = null, NativeMonitorOptions? options = null, bool raiseEvents = true)
        {
            var o = NativeMonitorOptions.ToNative(options);
            NativeErrors.Check(NativeMethods.iad_cell_start_monitors(_handle, Utf8.Encode(axes is null ? null : JoinNames(axes), nameof(axes)), ref o));
            if (raiseEvents)
            {
                StartEventPump();
            }
        }

        /// <summary>Stop the status monitors of the listed axes (the event thread keeps raising the cell's faults: <see cref="StopEventPump"/>).</summary>
        public void StopMonitors(IEnumerable<string>? axes = null) =>
            NativeErrors.Check(NativeMethods.iad_cell_stop_monitors(_handle, Utf8.Encode(axes is null ? null : JoinNames(axes), nameof(axes))));

        /// <summary>
        /// The next queued event of the cell, waiting up to <paramref name="timeout"/> s (0: no wait); null when none came.
        /// For a caller that drains the queue itself instead of the C# events (each event is returned once).
        /// </summary>
        public NativeEvent? NextEvent(double timeout = 0.0)
        {
            NativeErrors.Check(NativeMethods.iad_cell_next_event(_handle, timeout, out var ev));
            return ev.Kind == 0 ? null : new NativeEvent(ev, _names);
        }

        /// <summary>Start the binding's event thread for the cell's queue (no-op when it runs).</summary>
        public void StartEventPump()
        {
            lock (_eventGate)
            {
                if (_pump?.IsRunning == true)
                {
                    return;
                }

                _pump = new NativeEventPump("iad-events cell", NextEventCall, Dispatch, ev => new NativeEvent(ev, _names));
            }
        }

        /// <summary>End the event thread after it raised what is still queued (no-op when it does not run).</summary>
        public void StopEventPump()
        {
            NativeEventPump? pump;
            lock (_eventGate)
            {
                pump = _pump;
                _pump = null;
            }

            pump?.Stop(TimeSpan.FromSeconds(5));
        }

        private int NextEventCall(double timeoutS, out IadEvent ev) => NativeMethods.iad_cell_next_event(_handle, timeoutS, out ev);

        private void Dispatch(NativeEvent e)
        {
            NativeEventPump.Raise(EventReceived, this, e);
            switch (e.Kind)
            {
                case NativeEventKind.Status:
                    NativeEventPump.Raise(StatusChanged, this, e);
                    break;
                case NativeEventKind.Done:
                    NativeEventPump.Raise(MotionDone, this, e);
                    break;
                case NativeEventKind.Fault:
                    NativeEventPump.Raise(Fault, this, e);
                    break;
                case NativeEventKind.Link:
                case NativeEventKind.LinkState:
                    NativeEventPump.Raise(LinkChanged, this, e);
                    break;
            }
        }

        // ------------------------------------------------------------------------------------------ recovery

        /// <summary>
        /// recovery::track on every listed axis (Cell::track_all; READ-ONLY): <paramref name="rows"/> null = none, "all",
        /// "0,1". The sessions ({name: session}) are kept for <see cref="LastResultJson"/>. The statuses are not read.
        /// </summary>
        public NativeCellResults TrackAll(
            IEnumerable<string>? axes = null,
            string? rows = null,
            bool alarmHistory = true,
            bool parameters = false,
            bool returnExceptions = false)
        {
            var o = NativeAxis.TrackOptions(alarmHistory, parameters);
            var r0 = Utf8.Encode(rows, nameof(rows));
            return Group("TrackAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_track_all(_handle, a, ref o, r0, r, c, out n));
        }

        /// <summary>
        /// recovery::check_link on every listed axis (Cell::check_links; a fault goes to the cell's fault handling: queued,
        /// and with StopOnFault the other axes stop). One check per axis in list order (state "error" when the check
        /// itself failed). The records are kept for <see cref="LastResultJson"/>.
        /// </summary>
        public IReadOnlyList<KeyValuePair<string, NativeLinkCheck>> CheckLinks(IEnumerable<string>? axes = null)
        {
            var list = axes?.ToList();
            var names = list ?? _names;
            var checks = new IadLinkCheck[Math.Max(names.Count, 1)];
            NativeErrors.Check(NativeMethods.iad_cell_check_links(
                _handle, Utf8.Encode(list is null ? null : JoinNames(list), nameof(axes)), checks, NativeText.Size(checks.Length), out var count));
            var handled = Math.Min(Math.Min(NativeText.ToInt(count), checks.Length), names.Count);
            var result = new List<KeyValuePair<string, NativeLinkCheck>>(handled);
            for (var i = 0; i < handled; i++)
            {
                result.Add(new KeyValuePair<string, NativeLinkCheck>(names[i].Trim(), new NativeLinkCheck(checks[i])));
            }

            return result;
        }

        /// <summary>
        /// Start a background link monitor on the listed axes every <paramref name="intervalS"/> (Cell::monitor_links;
        /// state changes are raised as <see cref="LinkChanged"/>, faults as <see cref="Fault"/> with
        /// <paramref name="raiseEvents"/>). It never recovers or moves anything. Replaces a link monitor started earlier.
        /// </summary>
        public void MonitorLinks(double intervalS = 1.0, IEnumerable<string>? axes = null, bool raiseEvents = true)
        {
            NativeErrors.Check(NativeMethods.iad_cell_monitor_links(_handle, intervalS, Utf8.Encode(axes is null ? null : JoinNames(axes), nameof(axes))));
            if (raiseEvents)
            {
                StartEventPump();
            }
        }

        /// <summary>Stop the link monitor (no-op without one).</summary>
        public void StopLinkMonitor() => NativeErrors.Check(NativeMethods.iad_cell_stop_link_monitor(_handle));

        /// <summary>
        /// (0.6) The link monitor as JSON (no I/O): {"running", "interval_s", "passes", "states": {name: the last link
        /// check}, "errors": [...]}; "null" without one (a stopped monitor is no longer reported).
        /// </summary>
        public string LinkMonitorJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_link_monitor_json(_handle, b, s, out r));

        /// <summary>
        /// recovery::recover on every listed axis (Cell::recover_all; ports in parallel). NEVER starts motion.
        /// <paramref name="timeout"/> null = 30 s. The reports ({"reports": {name: ...}, "errors": {name: message}}) are
        /// kept for <see cref="LastResultJson"/>.
        /// </summary>
        public NativeCellResults RecoverAll(
            IEnumerable<string>? axes = null,
            double? timeout = null,
            NativeRecoveryPolicy? policy = null,
            string? checkRows = null,
            bool returnExceptions = false)
        {
            var p = NativeRecoveryPolicy.ToNative(policy);
            var rows = Utf8.Encode(checkRows, nameof(checkRows));
            return Group("RecoverAll", axes, returnExceptions, (byte[]? a, IadCellResult[] r, UIntPtr c, out UIntPtr n) =>
                NativeMethods.iad_cell_recover_all(_handle, a, Known.OrNaN(timeout), ref p, rows, r, c, out n));
        }

        // ------------------------------------------------------------------------------------------ reading, lines

        /// <summary>monitor_reading of every listed axis as JSON (Cell::monitor_all; READ-ONLY): {"readings": {name: reading}, "errors": {name: message}}.</summary>
        public string MonitorAllJson(IEnumerable<string>? axes = null)
        {
            var a0 = Utf8.Encode(axes is null ? null : JoinNames(axes), nameof(axes));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_monitor_all_json(_handle, a0, b, s, out r));
        }

        /// <summary>Every port's line as JSON ({port: Line::snapshot}; no I/O).</summary>
        public string LinesJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_lines_json(_handle, b, s, out r));

        /// <summary>A fresh read of every gateway's status as JSON ({port: status}, Cell::gateway_status_all; READ-ONLY).</summary>
        public string GatewayStatusJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_gateway_status_json(_handle, b, s, out r));

        /// <summary>State the position unit (0.01 or 0.001 mm) of the listed slaves (null = every axis) on <paramref name="port"/> (Port::set_position_unit).</summary>
        public void SetPositionUnit(string port, double unitMm, IEnumerable<int>? slaves = null) =>
            NativeErrors.Check(NativeMethods.iad_cell_set_position_unit(
                _handle, Utf8.EncodeRequired(port, nameof(port)), unitMm, Utf8.Encode(NativeLists.Slaves(slaves), nameof(slaves))));

        /// <summary>The RCON all-axes monitor of <paramref name="port"/> as JSON (Port::gateway_monitor; READ-ONLY), as <see cref="NativeAxis.GatewayMonitorJson"/>.</summary>
        public string GatewayMonitorJson(string port, IEnumerable<int>? slaves = null)
        {
            var p = Utf8.EncodeRequired(port, nameof(port));
            var s0 = Utf8.Encode(NativeLists.Slaves(slaves), nameof(slaves));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_gateway_monitor_json(_handle, p, s0, b, s, out r));
        }

        /// <summary>
        /// A read-only backup of every listed axis into <paramref name="directory"/> (Cell::backup; one "&lt;name&gt;.json"
        /// each). Returns {name: file path or "error: ..."} as JSON, also kept for <see cref="LastResultJson"/>.
        /// <paramref name="rows"/>: null = "used", "all" or "0,1,2".
        /// </summary>
        public string BackupJson(string directory, IEnumerable<string>? axes = null, string? rows = null)
        {
            var d = Utf8.EncodeRequired(directory, nameof(directory));
            var a0 = Utf8.Encode(axes is null ? null : JoinNames(axes), nameof(axes));
            var r0 = Utf8.Encode(rows, nameof(rows));
            return NativeText.ReadOnce(
                (byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_backup_json(_handle, a0, d, r0, b, s, out r),
                LastResultJson);
        }

        /// <summary>
        /// The JSON kept by the last <see cref="TrackAll"/>, <see cref="CheckLinks"/>, <see cref="RecoverAll"/>,
        /// <see cref="BackupJson"/> or <see cref="CoordRunJson"/> (no I/O). <see cref="EcNotReadyException"/> when there is none.
        /// </summary>
        public string LastResultJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_last_result_json(_handle, b, s, out r));

        // ------------------------------------------------------------------------------------------ coordinated motion

        /// <summary>
        /// Dry run of a coordinated (interpolated) path (CoordinatedGroup::plan): plan the request's path from the axes'
        /// current positions against their ranges and the line timing measured now. Reads only, no motion. The request
        /// and the plan are JSON (the request's form is in iai_actuator_driver.h, "coordinated motion"). An infeasible
        /// path: <see cref="SoftLimitException"/> ("reasons", "max_speed" in <see cref="NativeErrors.ErrorJsonOf"/>).
        /// UNVERIFIED on hardware (simulator only).
        /// </summary>
        public string CoordPlanJson(string request)
        {
            var q = Utf8.EncodeRequired(request, nameof(request));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_coord_plan_json(_handle, q, b, s, out r));
        }

        /// <summary>
        /// Plan and run the request's path (CoordinatedGroup::run): blocks until the path is done and returns the result
        /// as JSON (also kept for <see cref="LastResultJson"/>). An outside stop (<see cref="NativeCell.StopAll"/>, an
        /// axis's Stop or Pause from another thread), a fault alarm, a link fault or a late frame stops every axis of the
        /// group and throws <see cref="EcDriverException"/> (kind "CoordinatedMotionError"; "axis", "cause", "stop" and
        /// "result" in <see cref="NativeErrors.ErrorJsonOf"/>). Runs once. UNVERIFIED on hardware (simulator only).
        /// </summary>
        public string CoordRunJson(string request)
        {
            var q = Utf8.EncodeRequired(request, nameof(request));
            return NativeText.ReadOnce(
                (byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_cell_coord_run_json(_handle, q, b, s, out r),
                LastResultJson);
        }
    }
}
