using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // (ABI 0.2) Push, command-profile moves, sequences, move handles, fast mode, recovery, information, the line and the
    // unit modes. Evidence: the C++ features behind these are ported from the Python edition (VERIFIED there where the C++
    // headers say so); the C++ code, the ABI and this binding are UNVERIFIED on hardware (simulator only).
    internal sealed partial class NativeAxis
    {
        // ------------------------------------------------------------------------------------------ motion

        /// <summary>
        /// Push move (Axis::push): approach <paramref name="target"/> at <paramref name="speed"/>, then push over
        /// <paramref name="band"/> (signed by <paramref name="direction"/>: +1 away from home, -1 toward it) at
        /// <paramref name="forcePct"/> (PPOW = floor(pct x 255 / 100)) up to min(max_push_pct, MXPP 7.9). Refused (nothing
        /// sent) when the unit cannot push, the band end is outside the stroke, the axis is moving, or it stands inside or
        /// past the push zone. With <paramref name="wait"/> the outcome (contact or missed) is returned, else null.
        /// <paramref name="timeout"/> null = 2 x the approach estimate + 2 x band / push speed + 3 s.
        /// </summary>
        public NativePushResult? Push(
            double target,
            double band,
            double forcePct,
            int direction = 1,
            double speed = 10.0,
            double accelG = 0.1,
            bool wait = true,
            double? timeout = null)
        {
            var o = SizedStruct.Fresh<IadPushOptions>();
            NativeErrors.Check(NativeMethods.iad_push_options_init(ref o));
            o.Direction = direction;
            o.Speed = speed;
            o.AccelG = accelG;
            o.Wait = Known.Flag(wait);
            o.TimeoutS = Known.OrNaN(timeout);
            NativeErrors.Check(NativeMethods.iad_axis_push(_handle, target, band, forcePct, ref o, out var result));
            return wait ? new NativePushResult(result, LastResultJsonOrEmpty()) : null;
        }

        /// <summary>
        /// Change the acceleration (= deceleration) of the 0x9900 move in progress (Axis::change_accel; as
        /// <see cref="ChangeSpeed"/>: refused, nothing sent, without a move of this axis running).
        /// </summary>
        public NativeStatus? ChangeAccel(double accelG, bool wait = true, double? timeout = null)
        {
            var o = IadChangeOptions.Of(wait, Known.OrNaN(timeout));
            NativeErrors.Check(NativeMethods.iad_axis_change_accel(_handle, accelG, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// Absolute moves through <paramref name="points"/> in order (Axis::move_sequence); every point is checked before
        /// the first frame (<see cref="SoftLimitException"/> names the point). Returns one status per point.
        /// <paramref name="blendBand"/> null: every point completes before the next is sent; 0.01..10 units: the next
        /// target is sent once within that distance of the current one (UNVERIFIED). <paramref name="timeout"/> is per
        /// point (null = 2 x the move's estimate + 2 s).
        /// </summary>
        public IReadOnlyList<NativeStatus> MoveSequence(
            IEnumerable<NativeSequencePoint> points,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? blendBand = null,
            double? timeout = null,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var native = (points ?? throw new ArgumentNullException(nameof(points)))
                .Select(p => (p ?? throw new ArgumentException("a sequence point is null", nameof(points))).ToNative())
                .ToArray();
            if (native.Length == 0)
            {
                throw new ArgumentException("no point to move to", nameof(points));
            }

            var o = SizedStruct.Fresh<IadSequenceOptions>();
            NativeErrors.Check(NativeMethods.iad_sequence_options_init(ref o));
            o.Speed = speed;
            o.AccelG = accelG;
            o.Band = band;
            o.BlendBand = Known.OrNaN(blendBand);
            o.TimeoutS = Known.OrNaN(timeout);
            o.Completion = (int)completion;
            var statuses = new IadStatus[native.Length];
            NativeErrors.Check(NativeMethods.iad_axis_move_sequence(
                _handle, native, NativeText.Size(native.Length), ref o, statuses, NativeText.Size(statuses.Length), out var count));
            var done = Math.Min(NativeText.ToInt(count), statuses.Length);
            var result = new List<NativeStatus>(done);
            for (var i = 0; i < done; i++)
            {
                result.Add(new NativeStatus(statuses[i]));
            }

            return result;
        }

        /// <summary>The same through plain positions, each with the sequence's speed, acceleration and band.</summary>
        public IReadOnlyList<NativeStatus> MoveSequence(
            IEnumerable<double> positions,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? blendBand = null,
            double? timeout = null,
            NativeCompletion completion = NativeCompletion.Default) =>
            MoveSequence(
                (positions ?? throw new ArgumentNullException(nameof(positions))).Select(p => new NativeSequencePoint(p)),
                speed,
                accelG,
                band,
                blendBand,
                timeout,
                completion);

        // ------------------------------------------------------------------------------------------ move handles

        /// <summary>
        /// Start an absolute move without waiting (Axis::start_move_to): every check of <see cref="MoveTo"/> has passed
        /// and the command is sent when this returns. Wait for it, poll it or cancel it through the handle; dispose the
        /// handle when done (the move itself is not affected).
        /// </summary>
        public NativeMoveHandle StartMoveTo(
            double position,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? timeout = null,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = MoveOptions(speed, accelG, band, false, timeout, jog: false, decelG, accelPattern, completion);
            return NativeMoveHandle.Adopt(this, NativeMethods.iad_axis_start_move_to(_handle, position, ref o, out var move), move);
        }

        /// <summary>Relative to the measured position (Axis::start_move_by); as <see cref="StartMoveTo"/>.</summary>
        public NativeMoveHandle StartMoveBy(
            double delta,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? timeout = null,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = MoveOptions(speed, accelG, band, false, timeout, jog: false, decelG, accelPattern, completion);
            return NativeMoveHandle.Adopt(this, NativeMethods.iad_axis_start_move_by(_handle, delta, ref o, out var move), move);
        }

        /// <summary>A small relative step (Axis::start_jog; limits jog_step); as <see cref="StartMoveTo"/>.</summary>
        public NativeMoveHandle StartJog(
            double step,
            double speed = 10.0,
            double accelG = 0.1,
            double band = 0.1,
            double? timeout = null,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = MoveOptions(speed, accelG, band, false, timeout, jog: true, null, NativeAccelPattern.Default, completion);
            return NativeMoveHandle.Adopt(this, NativeMethods.iad_axis_start_jog(_handle, step, ref o, out var move), move);
        }

        // ------------------------------------------------------------------------------------------ cancel

        /// <summary>
        /// End every blocking call of this axis running now in another thread (a move, jog, home, push, stored row,
        /// <see cref="WaitIdle"/>, a move handle's wait, the servo wait, a cell wait on this axis): it throws
        /// <see cref="OperationCanceledException"/> soon, a motion wait after sending a decel stop (Axis::cancel_waits).
        /// Calls started afterwards are not affected. Sends nothing itself; safe from any thread.
        /// </summary>
        public void Cancel() => NativeErrors.Check(NativeMethods.iad_axis_cancel(_handle));

        // ------------------------------------------------------------------------------------------ fast mode

        /// <summary>
        /// The fast path in one call (Axis::enable_fast): fast completion, the status monitor (with
        /// <paramref name="monitor"/>; its events go to this axis's queue: <see cref="StartEventPump"/> raises them) and the
        /// link raised to <paramref name="baudRate"/> (0 keeps the rate). The baud change is tried only when this axis
        /// owns the port, the transport has a baud rate and the axis is at rest; otherwise the rate is kept and the reason
        /// is in the notes. Safe to call again. 230400 is VERIFIED on the EC-R6S rod in the Python edition.
        /// </summary>
        public NativeFastSummary EnableFast(int baudRate = 230400, bool monitor = true)
        {
            NativeErrors.Check(NativeMethods.iad_axis_enable_fast(_handle, baudRate, Known.Flag(monitor), out var summary));
            return new NativeFastSummary(summary, LastResultJsonOrEmpty());
        }

        /// <summary>Undo <see cref="EnableFast"/> (Axis::disable_fast): safe completion again and the monitor stopped; the baud rate is kept.</summary>
        public NativeFastSummary DisableFast()
        {
            NativeErrors.Check(NativeMethods.iad_axis_disable_fast(_handle, out var summary));
            return new NativeFastSummary(summary, LastResultJsonOrEmpty());
        }

        /// <summary><see cref="EnableFast"/> ran and no <see cref="DisableFast"/> since (Axis::fast_mode).</summary>
        public bool FastMode
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_fast_mode(_handle, out var on));
                return Known.Bool(on);
            }
        }

        // ------------------------------------------------------------------------------------------ recovery

        /// <summary>
        /// Remember what the application has set up while the link is up (recovery::track; READ-ONLY): identity, baud
        /// rate, PC control, safety speed, HEND, uptime, the alarm history (<paramref name="alarmHistory"/>) and optionally
        /// rows (<paramref name="rows"/>: null = none, "all", "0,1,5") and the parameter classes. <paramref name="name"/>
        /// null keeps the earlier session's name; <paramref name="backupJson"/>: a backup to compare against later.
        /// </summary>
        public void Track(string? name = null, string? rows = null, bool alarmHistory = true, bool parameters = false, string? backupJson = null)
        {
            var o = TrackOptions(alarmHistory, parameters);
            NativeErrors.Check(NativeMethods.iad_axis_track(
                _handle, ref o, Utf8.Encode(name, nameof(name)), Utf8.Encode(rows, nameof(rows)), Utf8.Encode(backupJson, nameof(backupJson))));
        }

        internal static IadTrackOptions TrackOptions(bool alarmHistory, bool parameters)
        {
            var o = SizedStruct.Fresh<IadTrackOptions>();
            NativeErrors.Check(NativeMethods.iad_track_options_init(ref o));
            o.AlarmHistory = Known.Flag(alarmHistory);
            o.Parameters = Known.Flag(parameters);
            return o;
        }

        /// <summary>
        /// A session from a backup's JSON (recovery::session_from_backup; no I/O): a recovery then requires the unit the
        /// backup was taken from. <see cref="ArgumentException"/> when the backup holds no identity or is not JSON.
        /// </summary>
        public void SessionFromBackup(string backupJson, string? name = null) =>
            NativeErrors.Check(NativeMethods.iad_axis_session_from_backup(
                _handle, Utf8.EncodeRequired(backupJson, nameof(backupJson)), Utf8.Encode(name, nameof(name))));

        /// <summary>The same from a backup file (UTF-8 JSON); <see cref="System.IO.IOException"/> when it cannot be read.</summary>
        public void SessionFromBackupFile(string path, string? name = null) =>
            NativeErrors.Check(NativeMethods.iad_axis_session_from_backup_file(
                _handle, Utf8.EncodeRequired(path, nameof(path)), Utf8.Encode(name, nameof(name))));

        /// <summary>The unit's session as JSON (Session::to_json; no I/O). <see cref="EcNotReadyException"/> when it has none.</summary>
        public string SessionJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_session_json(_handle, b, s, out r));

        /// <summary>Drop the unit's session, link state and pending target (recovery::forget).</summary>
        public void Forget() => NativeErrors.Check(NativeMethods.iad_axis_forget(_handle));

        /// <summary>
        /// Record the last commanded target (null: none, e.g. after a home return) for the recovery's position check
        /// (recovery::note_target; the axis records its own moves).
        /// </summary>
        public void NoteTarget(double? targetMm) => NativeErrors.Check(NativeMethods.iad_axis_note_target(_handle, Known.OrNaN(targetMm)));

        /// <summary>Tell the session about a deliberate change (recovery::note_control); null = unchanged.</summary>
        public void NoteControl(bool? modbusControl = null, bool? safetySpeed = null, int? baudRate = null, bool? homed = null) =>
            NativeErrors.Check(NativeMethods.iad_axis_note_control(_handle, Tri(modbusControl), Tri(safetySpeed), baudRate ?? 0, Tri(homed)));

        private static int Tri(bool? value) => Known.Tri(value);

        /// <summary>
        /// One cheap link check (recovery::check_link; one status read compared with the session; READ-ONLY). A link
        /// event goes to this axis's queue. A failure that is not a link fault is thrown.
        /// </summary>
        public NativeLinkCheck CheckLink()
        {
            NativeErrors.Check(NativeMethods.iad_axis_check_link(_handle, out var check));
            return new NativeLinkCheck(check, LastResultJsonOrEmpty());
        }

        /// <summary>
        /// The unit's last known link state, no I/O: "ok", "port_vanished", "unit_silent", "unit_restarted",
        /// "unit_replaced", "recovering".
        /// </summary>
        public string LinkState =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_link_state(_handle, b, s, out r), 256);

        /// <summary>
        /// Bring the axis back after a power or link loss and report what changed (recovery::recover_axis: without a
        /// session the identity read by the last <see cref="Connect"/> / <see cref="Identify"/> is the baseline). NEVER
        /// starts motion. <paramref name="timeout"/> null = 30 s; <paramref name="checkRows"/>: the rows to compare
        /// ("all", "0,1"); <paramref name="backupJson"/>: compare with this backup. <see cref="RecoveryException"/> /
        /// <see cref="UnitReplacedException"/> carry the report in <see cref="NativeErrors.ErrorJsonOf"/>.
        /// </summary>
        public NativeRecoverySummary Recover(
            double? timeout = null,
            NativeRecoveryPolicy? policy = null,
            string? checkRows = null,
            string? backupJson = null,
            string? name = null)
        {
            var p = NativeRecoveryPolicy.ToNative(policy);
            NativeErrors.Check(NativeMethods.iad_axis_recover(
                _handle,
                Known.OrNaN(timeout),
                ref p,
                Utf8.Encode(checkRows, nameof(checkRows)),
                Utf8.Encode(backupJson, nameof(backupJson)),
                Utf8.Encode(name, nameof(name)),
                out var summary));
            return new NativeRecoverySummary(summary, LastResultJsonOrEmpty());
        }

        /// <summary>
        /// (0.6) A session from an identity already read (recovery::session_from_info; no I/O): a recovery then checks the
        /// unit is the same one. <paramref name="infoJson"/>: a unit info's identity JSON (serial_number, model_name,
        /// system_code, unit_code, firmware, ...); null = the identity the axis read last (<see cref="EcNotReadyException"/>
        /// when it read none). A session the unit already has is kept.
        /// </summary>
        public void SessionFromInfo(string? infoJson = null, string? name = null) =>
            NativeErrors.Check(NativeMethods.iad_axis_session_from_info(
                _handle, Utf8.Encode(infoJson, nameof(infoJson)), Utf8.Encode(name, nameof(name))));

        /// <summary>
        /// (0.6) Write volatile fields of the unit's session (recovery::update_session; no I/O): one JSON object of "name",
        /// "baudrate", "modbus_control", "safety_speed", "servo_on", "homed", "uptime_ms", "uptime_age_s",
        /// "last_target_mm", "recovering". <see cref="EcNotReadyException"/> without a session; another key or a wrong
        /// value writes nothing (<see cref="ArgumentException"/>).
        /// </summary>
        public void SessionUpdate(string fieldsJson) =>
            NativeErrors.Check(NativeMethods.iad_axis_session_update(_handle, Utf8.EncodeRequired(fieldsJson, nameof(fieldsJson))));

        /// <summary>
        /// (0.6) Record the unit's link state (recovery::set_link_state; no I/O) and queue the matching link event on this
        /// axis when it changed (<see cref="NativeEventSource.SetLinkState"/>). Returns whether it changed.
        /// </summary>
        public bool SetLinkState(string state, string? message = null, string? name = null)
        {
            NativeErrors.Check(NativeMethods.iad_axis_set_link_state(
                _handle, Utf8.EncodeRequired(state, nameof(state)), Utf8.Encode(message, nameof(message)), Utf8.Encode(name, nameof(name)), out var changed));
            return Known.Bool(changed);
        }

        /// <summary>
        /// (0.6) Whether the axis's port is listed by the PC right now (recovery::port_present; nothing opened): null when
        /// it cannot be told (a transport without a name).
        /// </summary>
        public bool? PortPresent()
        {
            NativeErrors.Check(NativeMethods.iad_axis_port_present(_handle, out var present));
            return Known.Tri(present);
        }

        /// <summary>
        /// (0.6) The signs that the unit restarted since its session was refreshed (recovery::restart_signs; no I/O), in
        /// words. <paramref name="words"/>: a status block as read from 0x9000 (8 to 22 words); <paramref name="optionsJson"/>:
        /// {"new_records", "baud_rate_found", "boot_baud_rate", "now_s"} (null: none). <see cref="EcNotReadyException"/>
        /// without a session.
        /// </summary>
        public IReadOnlyList<string> RestartSigns(IReadOnlyList<ushort> words, string? optionsJson = null)
        {
            var w = (words ?? throw new ArgumentNullException(nameof(words))).ToArray();
            var o = Utf8.Encode(optionsJson, nameof(optionsJson));
            var json = NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) =>
                NativeMethods.iad_axis_restart_signs(_handle, w, NativeText.Size(w.Length), o, b, s, out r));
            return (MiniJson.Parse(json) as List<object?> ?? new List<object?>()).OfType<string>().ToList();
        }

        // ------------------------------------------------------------------------------------------ information

        /// <summary>
        /// (0.6) One reader of the unit's directory by name (iad_axis_info_call_json; READ-ONLY, FC03 inside the blocks the
        /// directory declares): "read_directory", "read_block", "read_class", "read_support_words", "read_accl",
        /// "unit_info", "backup", with its arguments as one JSON object (null: none). Its result as JSON (also kept for
        /// <see cref="LastResultJson"/>). A class outside the directory or a read outside a block's extent is an
        /// <see cref="ArgumentException"/> with nothing sent; an unknown function a <see cref="NotSupportedException"/>.
        /// </summary>
        public string InfoCallJson(string function, string? argsJson = null)
        {
            var f = Utf8.EncodeRequired(function, nameof(function));
            var a = Utf8.Encode(argsJson, nameof(argsJson));
            return NativeText.ReadOnce(
                (byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_axis_info_call_json(_handle, f, a, b, n, out r),
                LastResultJson);
        }

        /// <summary>
        /// Everything the unit tells about itself as JSON (info::unit_info; READ-ONLY). <paramref name="sections"/>: null =
        /// all, else of "identity", "parameters", "monitor", "alarms", "maintenance", "positions". The read can be long:
        /// it runs once (a large result is fetched from the copy the library keeps).
        /// </summary>
        public string ReadUnitInfoJson(IEnumerable<string>? sections = null)
        {
            var s = Utf8.Encode(sections is null ? null : string.Join(",", sections), nameof(sections));
            return NativeText.ReadOnce(
                (byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_axis_read_unit_info_json(_handle, s, b, n, out r),
                LastResultJson);
        }

        /// <summary>
        /// A read-only backup in the Python edition's schema (info::backup; FC03 + FC11 only) as JSON.
        /// <paramref name="rows"/>: null = "used", "all" or "0,1,2"; <paramref name="path"/>: also written there (UTF-8).
        /// </summary>
        public string BackupJson(string? rows = null, string? path = null)
        {
            var r0 = Utf8.Encode(rows, nameof(rows));
            var p = Utf8.Encode(path, nameof(path));
            return NativeText.ReadOnce(
                (byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_axis_backup_json(_handle, r0, p, b, n, out r),
                LastResultJson);
        }

        /// <summary>
        /// <see cref="BackupJson"/> of several axes into <paramref name="directory"/> (info::backup_axes; created if
        /// missing), one "&lt;name&gt;.json" each (characters other than letters, digits, '-', '_', '.' become '_').
        /// <paramref name="names"/>: null = each axis's "port:slave" name, else one per axis. An axis that does not answer
        /// is reported and the others are still saved. Returns {name: file path or "error: &lt;Type&gt;: &lt;message&gt;"}.
        /// </summary>
        public static string BackupAxesJson(IEnumerable<NativeAxis> axes, string directory, IEnumerable<string>? names = null, string? rows = null)
        {
            var list = (axes ?? throw new ArgumentNullException(nameof(axes))).ToList();
            var d = Utf8.EncodeRequired(directory, nameof(directory));
            var n = Utf8.Encode(names is null ? null : string.Join(",", names), nameof(names));
            var r0 = Utf8.Encode(rows, nameof(rows));
            var raw = new IntPtr[list.Count];
            var added = new bool[list.Count];
            try
            {
                for (var i = 0; i < list.Count; i++)
                {
                    var handle = (list[i] ?? throw new ArgumentException("an axis is null", nameof(axes)))._handle;
                    handle.DangerousAddRef(ref added[i]);
                    raw[i] = handle.DangerousGetHandle();
                }

                // Written once: a result too large for the buffer is not read again (that would write the files again).
                var buffer = new byte[1024 * 1024];
                NativeErrors.Check(NativeMethods.iad_backup_axes_json(
                    raw, NativeText.Size(raw.Length), n, d, r0, buffer, NativeText.Size(buffer.Length), out _));
                return Utf8.FromBuffer(buffer);
            }
            finally
            {
                for (var i = 0; i < list.Count; i++)
                {
                    if (added[i])
                    {
                        list[i]._handle.DangerousRelease();
                    }
                }
            }
        }

        /// <summary>
        /// The compatibility self-test (run_check; read-only unless <paramref name="motion"/>: then also a short move out
        /// and back of <paramref name="stepMm"/>). Never fails for what the unit answers (a FAIL line); each line is also
        /// queued as a <see cref="CheckItem"/> event. <paramref name="portLabel"/> is recorded as the report's port.
        /// </summary>
        public NativeCheckSummary SelfTest(
            bool motion = false,
            double? stepMm = null,
            double? speedMmS = null,
            double? accelG = null,
            string? portLabel = null)
        {
            var o = SizedStruct.Fresh<IadCheckOptions>();
            NativeErrors.Check(NativeMethods.iad_check_options_init(ref o));
            o.Motion = Known.Flag(motion);
            o.StepMm = stepMm ?? o.StepMm;
            o.SpeedMmS = speedMmS ?? o.SpeedMmS;
            o.AccelG = accelG ?? o.AccelG;
            NativeErrors.Check(NativeMethods.iad_axis_self_test(_handle, ref o, Utf8.Encode(portLabel, nameof(portLabel)), out var summary));
            return new NativeCheckSummary(summary, LastResultJsonOrEmpty());
        }

        /// <summary>
        /// (0.6) The lines the self-test running on this axis has finished so far, from index <paramref name="first"/>, as
        /// JSON [{"name", "result", "detail", "data"}, ...] (no I/O; "[]" when none runs). Called from another thread while
        /// <see cref="SelfTest"/> runs.
        /// </summary>
        public string SelfTestItemsJson(int first = 0) =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_self_test_items_json(_handle, first, b, s, out r));

        /// <summary>
        /// One read of the whole 0x9000 monitor block, decoded, as JSON (monitor_reading: position, velocity, current,
        /// current_pct, deviation, alarm, uptime_s, units, raw; READ-ONLY).
        /// </summary>
        public string ReadMonitorJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_monitor_json(_handle, b, s, out r));

        /// <summary>
        /// The lean status: one read of 0x9000-0x9007 (Axis::read_lean_status; READ-ONLY), the 8 words the status monitor
        /// and the fast completion use (no current or uptime; no velocity while moving). <paramref name="retries"/> null =
        /// the port's.
        /// </summary>
        public NativeStatus ReadLeanStatus(int? retries = null)
        {
            NativeErrors.Check(NativeMethods.iad_axis_read_lean_status(_handle, retries ?? int.MinValue, out var st));
            return new NativeStatus(st);
        }

        /// <summary>The power-up baud rate (BRSL 8.3 on an EC, No.16 on a CON unit; READ-ONLY); null when not readable.</summary>
        public int? ReadBootBaudRate()
        {
            NativeErrors.Check(NativeMethods.iad_axis_read_boot_baud_rate(_handle, out var rate));
            return Known.Int(rate);
        }

        /// <summary>The caps moves are checked against and the documented minimum speed (may read MAXV / MAXA / MAXD / MXPP once; READ-ONLY).</summary>
        public NativeMotionCaps MotionCaps()
        {
            NativeErrors.Check(NativeMethods.iad_axis_motion_caps(_handle, out var caps));
            return new NativeMotionCaps(caps);
        }

        /// <summary>The unit's home pattern ORG (EC item 9.16; READ-ONLY); null when the item is not in the unit's directory.</summary>
        public uint? ReadHomePattern()
        {
            NativeErrors.Check(NativeMethods.iad_axis_read_home_pattern(_handle, out var known, out var pattern));
            return Known.Bool(known) ? pattern : (uint?)null;
        }

        /// <summary>
        /// Release what a home start left set (Axis::finish_home: the HOME coil of the coil strategy, the DRG1 HOME bit
        /// of the register path; nothing with the IAI strategies): call it after <see cref="Home"/> with wait false once
        /// the axis is homed.
        /// </summary>
        public void FinishHome() => NativeErrors.Check(NativeMethods.iad_axis_finish_home(_handle));

        // ------------------------------------------------------------------------------------------ line, unit modes

        /// <summary>The line this axis is on as JSON (Line::snapshot: kind, rules, the gateway's last status, position units, unreachable slaves, explanation; no I/O).</summary>
        public string LineJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_line_json(_handle, b, s, out r));

        /// <summary>A fresh read of the line's gateway status as JSON (RCON / ROBONET gateway; READ-ONLY); "null" on a line without one.</summary>
        public string GatewayStatusJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_gateway_status_json(_handle, b, s, out r));

        /// <summary>
        /// The RCON all-axes monitor of this axis's line as JSON (Port::gateway_monitor; at most two FC03 frames for 16
        /// axes; READ-ONLY): {"&lt;slave&gt;": {"axis", "slave", "position_counts", ...}}. <paramref name="slaves"/>: null =
        /// the slaves the gateway reports linked. <see cref="EcNotReadyException"/> on a line without an RCON gateway. DOC
        /// (ME0413 p.5-2), UNVERIFIED on hardware.
        /// </summary>
        public string GatewayMonitorJson(IEnumerable<int>? slaves = null)
        {
            var s0 = Utf8.Encode(NativeLists.Slaves(slaves), nameof(slaves));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_gateway_monitor_json(_handle, s0, b, s, out r));
        }

        /// <summary>
        /// State this axis's position unit (0.01 or 0.001 mm; RCON parameter No.207) on its line
        /// (Line::set_position_unit). Needed before mm-based motion on an RCON line.
        /// </summary>
        public void SetPositionUnit(double unitMm) => NativeErrors.Check(NativeMethods.iad_axis_set_position_unit(_handle, unitMm));

        /// <summary>
        /// The action mode (Axis::set_action_mode): <paramref name="pc"/> = PC / Modbus control (PMSS 1) or PIO (PMSS 0);
        /// <paramref name="safety"/> null keeps the present safety speed. PIO is 0x45 cmd 0x0C op1 = 0; PC is the PMSL coil
        /// (when PMSS is 0) and the SFTY coil (when the safety speed changes). Refused while the axis moves, pauses, homes or
        /// pushes (<see cref="EcNotReadyException"/>) and on a line without 0x45 (<see cref="UnsafeCommandException"/>),
        /// nothing sent. With <paramref name="verify"/> the result is read back.
        /// </summary>
        public NativeStatus SetActionMode(bool pc = true, bool? safety = null, bool verify = true)
        {
            NativeErrors.Check(NativeMethods.iad_axis_set_action_mode(_handle, Known.Flag(pc), Tri(safety), Known.Flag(verify), out var st));
            return new NativeStatus(st);
        }

        /// <summary>
        /// Switch the safety-speed mode (Axis::set_safety_speed; ~100 mm/s cap when on; volatile): the SFTY coil, or DRG1
        /// b14 on the register path. Takes Modbus control first if needed (at rest only). The driver never changes it by
        /// itself.
        /// </summary>
        public NativeStatus SetSafetySpeed(bool on, bool verify = true)
        {
            NativeErrors.Check(NativeMethods.iad_axis_set_safety_speed(_handle, Known.Flag(on), Known.Flag(verify), out var st));
            return new NativeStatus(st);
        }

        /// <summary>
        /// The command strategy for servo / home / alarm reset (Axis::strategy / set_strategy). The coil strategy adapts
        /// by itself (register behind RCON or after an FC05 exception 01).
        /// </summary>
        public NativeCommandStrategy CommandStrategy
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_strategy(_handle, out var method));
                return (NativeCommandStrategy)method;
            }

            set => NativeErrors.Check(NativeMethods.iad_axis_set_strategy(_handle, (int)value));
        }

        /// <summary>How a stop is sent (Axis::stop_path): "coil" (the STOP coil) or "register" (the coil-free substitute).</summary>
        public string StopPath =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_stop_path(_handle, b, s, out r), 256);

        /// <summary>The line kind (Axis::line_name): "rcon", "robonet", "robonet-gateway"; "" on a direct line.</summary>
        public string LineName =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_line_name(_handle, b, s, out r), 256);

        /// <summary>
        /// <see cref="TeachPosition(int, bool, bool, bool)"/> by <paramref name="method"/> (Axis::teach_position):
        /// <see cref="NativeTeachMethod.Table"/> (the default) or <see cref="NativeTeachMethod.Coil"/> (EXPERIMENTAL; needs
        /// coils). Same refusals and warnings; the flags are accepted and ignored (no longer needed).
        /// </summary>
        public NativePositionWriteResult TeachPosition(int row, NativeTeachMethod method, bool allowNonvolatile = false, bool allowUnsafe = false, bool force = false)
        {
            var o = NvOptions(allowNonvolatile, allowUnsafe, force);
            NativeErrors.Check(NativeMethods.iad_axis_teach_position(_handle, row, (int)method, ref o, out var result));
            return new NativePositionWriteResult(result, LastResultJsonOrEmpty());
        }
    }
}
