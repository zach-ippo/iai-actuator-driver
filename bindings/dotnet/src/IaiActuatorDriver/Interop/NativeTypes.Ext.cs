using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // The ABI 0.2 data: enumerations, inputs and read-only views of the new output structs. Conventions as NativeTypes.cs
    // (units: mm or deg, units/s, G, seconds; a value the library does not know is null).

    // ------------------------------------------------------------------------------------------ enumerations

    /// <summary>
    /// How servo / home / alarm reset are sent (IAD_METHOD_*; C++ CommandMethod / the axis's strategy). (0.8) 0 is
    /// IAD_METHOD_DEFAULT, the axis option in a call and not a strategy to set or read, so these start at 1.
    /// </summary>
    internal enum NativeCommandStrategy
    {
        /// <summary>FC05 coils SON / HOME / ALRS (what the default gives; behind an RCON gateway the coil strategy adapts by itself).</summary>
        Coil = 1,

        /// <summary>IAI 0x45 commands.</summary>
        Iai = 2,

        /// <summary>EXPERIMENTAL IAI 0x48 commands (UNVERIFIED on any unit).</summary>
        Iai2 = 3,

        /// <summary>FC06 writes of DRG1 0x0D00, the coil-free RCON path (DOC, UNVERIFIED on hardware).</summary>
        Register = 4,
    }

    /// <summary>Acceleration pattern of a move (IAD_ACCEL_*; C++ AccelPattern, CTLF b6-b7).</summary>
    internal enum NativeAccelPattern
    {
        /// <summary>Not set: trapezoid (CTLF b6-b7 = 00).</summary>
        Default = 0,

        Trapezoid = 1,

        /// <summary>S-curve (CON No.56 ratio); refused above an acceleration time of 2 s.</summary>
        SMotion = 2,

        /// <summary>First-order filter (CON No.55 time constant).</summary>
        Filter = 3,
    }

    /// <summary>When a move counts as done (IAD_COMPLETION_*; C++ Completion).</summary>
    internal enum NativeCompletion
    {
        /// <summary>A move: the axis option; the axis option: <see cref="Safe"/>.</summary>
        Default = 0,

        /// <summary>PEND, the position within the band and PNOW unchanged over the settle time.</summary>
        Safe = 1,

        /// <summary>One sample: PEND, within the band, DSSE MOVE clear, not homing, not paused.</summary>
        Fast = 2,
    }

    /// <summary>The line a port is on (IAD_LINE_*; C++ LineOptions::kind).</summary>
    internal enum NativeLineKind
    {
        /// <summary>Probe once for an RCON gateway (D0h) and a ROBONET gateway (3Fh).</summary>
        Auto = 0,

        /// <summary>No gateway, no probe.</summary>
        Direct = 1,

        Rcon = 2,

        /// <summary>SIO through mode (cannot be probed).</summary>
        Robonet = 3,

        /// <summary>Gateway mode: no axis access.</summary>
        RobonetGateway = 4,
    }

    /// <summary>How a teach stores the current position (IAD_TEACH_*; C++ TeachMethod).</summary>
    internal enum NativeTeachMethod
    {
        /// <summary>Read the row, replace the position, write it back (the default).</summary>
        Table = 0,

        /// <summary>EXPERIMENTAL MOD / POSR / TEAC coils (DOC for CON units; UNVERIFIED on an EC).</summary>
        Coil = 1,
    }

    /// <summary>Kind of a queued event (IAD_EVENT_*).</summary>
    internal enum NativeEventKind
    {
        /// <summary>No event (only from <see cref="NativeAxis.NextEvent"/> as null).</summary>
        None = 0,

        /// <summary>The monitor saw PNOW / ALMC / DSS1 / DSS2 / DSSE change.</summary>
        Status = 1,

        /// <summary>The axis became done by the fast rule against its last target.</summary>
        Done = 2,

        /// <summary>A link event: name "link_lost", "unit_restarted", "unit_replaced", "recovered".</summary>
        Link = 3,

        /// <summary>A cell's fault: name = the kind ("port_vanished", "unit_silent"...).</summary>
        Fault = 4,

        /// <summary>A link monitor saw an axis's state change: name = the state, message = the reasons.</summary>
        LinkState = 5,

        /// <summary>One line of the self-test: name = the item, cause = "PASS" / "FAIL" / "WARN" / "SKIP".</summary>
        CheckItem = 6,
    }

    /// <summary>Which events a handle queues (IAD_EVENTS_*).</summary>
    [Flags]
    internal enum NativeEventMask
    {
        None = 0,
        Status = 0x01,
        Done = 0x02,

        /// <summary>Link, fault and link-state events.</summary>
        Link = 0x04,
        Check = 0x08,
        All = 0x0F,
    }

    /// <summary>(0.6) What queued an event (IAD_EVENT_SOURCE_*), so a link event reaches the caller that caused it.</summary>
    internal enum NativeEventSource
    {
        /// <summary>Not stated (a library before 0.6, or a kind without a source).</summary>
        None = 0,

        /// <summary>The status monitor (its polls and its link watch).</summary>
        Monitor = 1,

        /// <summary><see cref="NativeAxis.CheckLink"/>.</summary>
        CheckLink = 2,

        /// <summary><see cref="NativeAxis.Recover"/>.</summary>
        Recover = 3,

        /// <summary><see cref="NativeAxis.SetLinkState"/>.</summary>
        SetLinkState = 4,

        /// <summary>A cell's fault handling, link monitor and recovery.</summary>
        Cell = 5,
    }

    // ------------------------------------------------------------------------------------------ events, monitor

    /// <summary>One queued event of an axis or a cell (iad_event), raised on the binding's event thread.</summary>
    internal sealed class NativeEvent : EventArgs
    {
        internal NativeEvent(IadEvent e)
            : this(e, null)
        {
        }

        /// <summary>
        /// <paramref name="cellNames"/>: the names of the cell whose queue the event came from (null for an axis's own
        /// queue). A name longer than the event's axis field is cut there, and the event's index (the axis's position in
        /// the cell, 0.8) gives the whole name.
        /// </summary>
        internal unsafe NativeEvent(IadEvent e, IReadOnlyList<string>? cellNames)
        {
            Kind = Enum.IsDefined(typeof(NativeEventKind), e.Kind) ? (NativeEventKind)e.Kind : NativeEventKind.None;
            RawKind = e.Kind;
            Status = e.Status.RawWords > 0 ? new NativeStatus(e.Status) : null;
            TimeS = e.TimeS;
            WallTime = double.IsNaN(e.WallTimeS) || e.WallTimeS <= 0
                ? (DateTimeOffset?)null
                : DateTimeOffset.FromUnixTimeMilliseconds((long)Math.Round(e.WallTimeS * 1000.0));
            Sequence = e.Sequence;
            Dropped = e.Dropped;
            Axis = Utf8.FromFixed(e.Axis, SizedStruct.AxisNameSize);
            if (cellNames != null)
            {
                Axis = SizedStruct.NameOf(Axis, e.Index, cellNames);
            }

            Name = Utf8.FromFixed(e.Name, 32);
            Cause = Utf8.FromFixed(e.Cause, 32);
            Message = Utf8.FromFixed(e.Message, 256);
            Source = Enum.IsDefined(typeof(NativeEventSource), e.Source) ? (NativeEventSource)e.Source : NativeEventSource.None;
        }

        public NativeEventKind Kind { get; }

        /// <summary>IAD_EVENT_* as the library gave it (a newer library may add kinds; <see cref="Kind"/> is then None).</summary>
        public int RawKind { get; }

        /// <summary>Status / done events: the monitor's sample (the lean 8-word status); null otherwise.</summary>
        public NativeStatus? Status { get; }

        /// <summary>Monotonic seconds when it was queued (the clock of <see cref="NativeMonitorSample"/>).</summary>
        public double TimeS { get; }

        /// <summary>Wall-clock time (a link event: the core's own time stamp).</summary>
        public DateTimeOffset? WallTime { get; }

        /// <summary>Events queued on the handle so far, this one included: 1, 2, ...</summary>
        public long Sequence { get; }

        /// <summary>Events dropped (queue full) since the previous event was returned.</summary>
        public int Dropped { get; }

        /// <summary>The axis ("COM3:1").</summary>
        public string Axis { get; }

        /// <summary>See <see cref="NativeEventKind"/>.</summary>
        public string Name { get; }

        /// <summary>A link event: the link state behind it; a self-test line: its result.</summary>
        public string Cause { get; }

        public string Message { get; }

        /// <summary>(0.6) What queued it (IAD_EVENT_SOURCE_*); None when the library does not say.</summary>
        public NativeEventSource Source { get; }

        /// <inheritdoc />
        public override string ToString() =>
            string.Format(CultureInfo.InvariantCulture, "#{0} {1} {2} {3} {4}", Sequence, Kind, Axis, Name, Cause).TrimEnd();
    }

    /// <summary>Settings of the status monitor and of the handle's event queue (iad_monitor_options; C++ MonitorOptions).</summary>
    internal sealed class NativeMonitorOptions
    {
        /// <summary>Pause between two polls, port lock free: 0..10 s; null = 0.002.</summary>
        public double? GapS { get; set; }

        /// <summary>The link check (with a recovery session) at most this often: 0..3600 s, 0 = never; null = 1 s.</summary>
        public double? LinkCheckS { get; set; }

        /// <summary>Per poll; null = 0 (one attempt, then the lock is released).</summary>
        public int? Retries { get; set; }

        /// <summary>The events the handle queues; null = all.</summary>
        public NativeEventMask? Events { get; set; }

        /// <summary>Events kept, 1..65536; the oldest are dropped beyond it (counted); null = 1024.</summary>
        public int? QueueCapacity { get; set; }

        internal static IadMonitorOptions ToNative(NativeMonitorOptions? options)
        {
            var o = SizedStruct.Fresh<IadMonitorOptions>();
            NativeErrors.Check(NativeMethods.iad_monitor_options_init(ref o));
            if (options is null)
            {
                return o;
            }

            o.GapS = options.GapS ?? o.GapS;
            o.LinkCheckS = options.LinkCheckS ?? o.LinkCheckS;
            o.Retries = options.Retries ?? o.Retries;
            o.Events = options.Events is { } events ? (int)events : o.Events;
            o.QueueCapacity = options.QueueCapacity ?? o.QueueCapacity;
            return o;
        }
    }

    /// <summary>The monitor's last published poll and its counters (iad_monitor_sample; C++ MonitorSample).</summary>
    internal sealed class NativeMonitorSample
    {
        internal NativeMonitorSample(in IadMonitorSample s)
        {
            Status = s.Status.RawWords > 0 ? new NativeStatus(s.Status) : null;
            TStartS = Known.Double(s.TStartS);
            TS = Known.Double(s.TS);
            AgeS = Known.Double(s.AgeS);
            PollRateHz = Known.Double(s.PollRateHz);
            Seq = s.Seq;
            Polls = s.Polls;
            ErrorCount = s.ErrorCount;
            Running = Known.Bool(s.Running);
        }

        /// <summary>The lean status (8 words: no velocity while moving); null before the first poll.</summary>
        public NativeStatus? Status { get; }

        /// <summary>Monotonic seconds just before the request went out.</summary>
        public double? TStartS { get; }

        /// <summary>Monotonic seconds after the answer was decoded.</summary>
        public double? TS { get; }

        /// <summary>Seconds since <see cref="TS"/>; null before the first poll.</summary>
        public double? AgeS { get; }

        /// <summary>Answered polls per second since the start.</summary>
        public double? PollRateHz { get; }

        /// <summary>The sample's sequence number; 0 before the first poll.</summary>
        public long Seq { get; }

        /// <summary>Answered polls.</summary>
        public long Polls { get; }

        /// <summary>Transient errors in total.</summary>
        public long ErrorCount { get; }

        public bool Running { get; }
    }

    // ------------------------------------------------------------------------------------------ motion

    /// <summary>Outcome of a push (iad_push_result; C++ PushResult).</summary>
    internal sealed class NativePushResult
    {
        internal unsafe NativePushResult(IadPushResult r, string json)
        {
            Status = new NativeStatus(r.Status);
            PositionMm = r.PositionMm;
            Outcome = r.Outcome == 2 ? PushOutcome.Missed : PushOutcome.Contact;
            WarningCount = r.WarningCount;
            OutcomeName = Utf8.FromFixed(r.OutcomeName, 16);
            Json = json;
            Warnings = (MiniJson.TryParse(json) as JsonObject)?.Strings("warnings") ?? Array.Empty<string>();
        }

        /// <summary>Contact: holding against the work (the motor keeps pushing until the next move); missed: no contact over the band.</summary>
        public PushOutcome Outcome { get; }

        /// <summary>"contact" / "missed".</summary>
        public string OutcomeName { get; }

        public double PositionMm { get; }

        public NativeStatus Status { get; }

        /// <summary>E.g. a force below the unit's recommended minimum (allowed).</summary>
        public int WarningCount { get; }

        /// <summary>The warnings (from <see cref="Json"/>).</summary>
        public IReadOnlyList<string> Warnings { get; }

        /// <summary>The full record (iad_axis_last_result_json right after the push; "" when not available).</summary>
        public string Json { get; }
    }

    /// <summary>
    /// One point of <see cref="NativeAxis.MoveSequence(IEnumerable{NativeSequencePoint}, double, double, double, double?, double?, NativeCompletion)"/>
    /// (iad_sequence_point; C++ SequencePoint).
    /// </summary>
    internal sealed class NativeSequencePoint
    {
        public NativeSequencePoint(double position, double? speed = null, double? accelG = null, double? band = null)
        {
            Position = position;
            Speed = speed;
            AccelG = accelG;
            Band = band;
        }

        public double Position { get; set; }

        /// <summary>null = the sequence's.</summary>
        public double? Speed { get; set; }

        /// <summary>null = the sequence's.</summary>
        public double? AccelG { get; set; }

        /// <summary>null = the sequence's.</summary>
        public double? Band { get; set; }

        internal IadSequencePoint ToNative() => new IadSequencePoint
        {
            StructSize = SizedStruct.Size<IadSequencePoint>(),
            Position = Position,
            Speed = Known.Slot("speed", Speed),
            AccelG = Known.Slot("accelG", AccelG),
            Band = Known.Slot("band", Band),
        };
    }

    /// <summary>What a move handle knows, no I/O (iad_move_info; C++ MoveHandle).</summary>
    internal sealed class NativeMoveInfo
    {
        internal NativeMoveInfo(in IadMoveInfo i)
        {
            Result = Known.Bool(i.HasResult) ? new NativeStatus(i.Result) : null;
            Target = i.Target;
            Band = i.Band;
            TimeoutS = Known.Double(i.TimeoutS);
            SentAtS = i.SentAtS;
            DeadlineS = Known.Double(i.DeadlineS);
            Completion = i.Completion == 2 ? NativeCompletion.Fast : NativeCompletion.Safe;
            Cancelled = Known.Bool(i.Cancelled);
        }

        /// <summary>The final status once known; null before.</summary>
        public NativeStatus? Result { get; }

        /// <summary>Rounded to 0.001.</summary>
        public double Target { get; }

        public double Band { get; }

        public double? TimeoutS { get; }

        /// <summary>Monotonic seconds right after the command was answered.</summary>
        public double SentAtS { get; }

        /// <summary><see cref="SentAtS"/> + <see cref="TimeoutS"/>.</summary>
        public double? DeadlineS { get; }

        /// <summary><see cref="NativeCompletion.Safe"/> or <see cref="NativeCompletion.Fast"/>.</summary>
        public NativeCompletion Completion { get; }

        public bool Cancelled { get; }
    }

    /// <summary>
    /// One axis's move of <see cref="NativeCell.MoveAll(IEnumerable{NativeAxisMove}, bool, double?, bool)"/>
    /// (iad_axis_move; C++ AxisMove): its own speed, acceleration, deceleration, band and pattern.
    /// </summary>
    internal sealed class NativeAxisMove
    {
        public NativeAxisMove(string axis, double position)
        {
            Axis = axis ?? throw new ArgumentNullException(nameof(axis));
            Position = position;
        }

        /// <summary>The cell's axis name ("COM3:1").</summary>
        public string Axis { get; }

        public double Position { get; }

        public double Speed { get; set; } = 50.0;

        public double AccelG { get; set; } = 0.1;

        public double Band { get; set; } = 0.1;

        /// <summary>A deceleration other than <see cref="AccelG"/> (the command-profile path); null = AccelG.</summary>
        public double? DecelG { get; set; }

        public NativeAccelPattern AccelPattern { get; set; }

        public NativeCompletion Completion { get; set; }

        internal unsafe IadAxisMove ToNative()
        {
            var m = default(IadAxisMove);
            m.StructSize = SizedStruct.Size<IadAxisMove>();  // 0.8: the move's own fields (NaN = the default)
            m.Speed = Known.Slot("speed", Speed);
            m.AccelG = Known.Slot("accelG", AccelG);
            m.Band = Known.Slot("band", Band);
            m.TimeoutS = double.NaN;
            m.DecelG = Known.OrNaN(DecelG);
            m.AccelPattern = (int)AccelPattern;
            m.Completion = (int)Completion;
            m.Position = Position;
            var name = Utf8.EncodeRequired(Axis, nameof(Axis));
            if (name.Length > SizedStruct.AxisNameSize)
            {
                throw new ArgumentException($"axis name '{Axis}' is longer than {SizedStruct.AxisNameSize - 1} bytes");
            }

            for (var i = 0; i < name.Length; i++)
            {
                m.Axis[i] = name[i];
            }

            return m;
        }
    }

    /// <summary>
    /// The caps moves are checked against and the documented minimum speed (iad_motion_caps; Axis::speed_cap,
    /// accel_cap, accel_only_cap, decel_cap, push_cap, recommended_min_speed), in the axis's units.
    /// </summary>
    internal sealed class NativeMotionCaps
    {
        internal unsafe NativeMotionCaps(IadMotionCaps c)
        {
            SpeedCap = Known.Double(c.SpeedCap);
            AccelCap = Known.Double(c.AccelCap);
            AccelOnlyCap = Known.Double(c.AccelOnlyCap);
            DecelCap = Known.Double(c.DecelCap);
            PushCap = Known.Double(c.PushCap);
            MinSpeed = Known.Double(c.MinSpeed);
            MinSpeedSource = Utf8.FromFixed(c.MinSpeedSource, 256);
        }

        /// <summary>units/s: min(max_speed_mm_s, MAXV 7.4), each only when known.</summary>
        public double? SpeedCap { get; }

        /// <summary>G: acceleration = deceleration of a 0x9900 move: min(max_accel_g, MAXA, MAXD).</summary>
        public double? AccelCap { get; }

        /// <summary>G: acceleration of a move with its own deceleration: min(max_accel_g, MAXA).</summary>
        public double? AccelOnlyCap { get; }

        /// <summary>G: deceleration of such a move: min(max_accel_g, MAXD).</summary>
        public double? DecelCap { get; }

        /// <summary>%: min(max_push_pct, MXPP 7.9).</summary>
        public double? PushCap { get; }

        /// <summary>units/s: the documented minimum speed (a warning below it, never a refusal); null when nothing tells.</summary>
        public double? MinSpeed { get; }

        /// <summary>Where <see cref="MinSpeed"/> comes from, or why it is unknown.</summary>
        public string MinSpeedSource { get; }
    }

    // ------------------------------------------------------------------------------------------ fast mode

    /// <summary>What <see cref="NativeAxis.EnableFast"/> / <see cref="NativeAxis.DisableFast"/> did (iad_fast_summary; C++ FastModeSummary).</summary>
    internal sealed class NativeFastSummary
    {
        internal unsafe NativeFastSummary(IadFastSummary s, string json)
        {
            BaudRate = s.BaudRate > 0 ? s.BaudRate : (int?)null;
            Completion = s.Completion == 2 ? NativeCompletion.Fast : NativeCompletion.Safe;
            Monitor = Known.Bool(s.Monitor);
            Json = json;
            var notes = (MiniJson.TryParse(json) as JsonObject)?.Strings("notes");
            Notes = notes is { Count: > 0 } ? notes : NativeLists.Split(Utf8.FromFixed(s.Notes, 256), s.NoteCount);
        }

        /// <summary>The port's rate afterwards; null on a transport without one.</summary>
        public int? BaudRate { get; }

        public NativeCompletion Completion { get; }

        /// <summary>A status monitor runs.</summary>
        public bool Monitor { get; }

        /// <summary>What could not be done (the baud rate kept...).</summary>
        public IReadOnlyList<string> Notes { get; }

        /// <summary>The full record (iad_axis_last_result_json right after the call; "" when not available).</summary>
        public string Json { get; }
    }

    // ------------------------------------------------------------------------------------------ recovery

    /// <summary>Result of a link check (iad_link_check; C++ recovery::LinkCheck).</summary>
    internal sealed class NativeLinkCheck
    {
        internal unsafe NativeLinkCheck(IadLinkCheck c, string? json = null)
        {
            Status = c.Status.RawWords > 0 ? new NativeStatus(c.Status) : null;
            ElapsedS = Known.Double(c.ElapsedS);
            Ok = Known.Bool(c.Ok);
            State = Utf8.FromFixed(c.State, 32);
            Error = Utf8.FromFixed(c.Error, 256);
            Json = json ?? string.Empty;
            var reasons = (MiniJson.TryParse(json) as JsonObject)?.Strings("reasons");
            Reasons = reasons is { Count: > 0 } ? reasons : NativeLists.Split(Utf8.FromFixed(c.Reasons, 256), c.ReasonCount);
        }

        /// <summary>The state is "ok".</summary>
        public bool Ok { get; }

        /// <summary>"ok", "port_vanished", "unit_silent", "unit_restarted", "recovering" ("error": the check itself failed, cell only).</summary>
        public string State { get; }

        /// <summary>The restart signs.</summary>
        public IReadOnlyList<string> Reasons { get; }

        /// <summary>"&lt;Type&gt;: &lt;message&gt;" when the unit did not answer.</summary>
        public string Error { get; }

        /// <summary>The status read; null when the unit did not answer.</summary>
        public NativeStatus? Status { get; }

        public double? ElapsedS { get; }

        /// <summary>The full record (single axis: iad_axis_last_result_json right after the check; "" otherwise).</summary>
        public string Json { get; }

        /// <inheritdoc />
        public override string ToString() => State;
    }

    /// <summary>
    /// What a recovery restores and checks (iad_recovery_policy; C++ recovery::RecoveryPolicy). Every property is
    /// optional: null keeps the library's default. A recovery never starts motion.
    /// </summary>
    internal sealed class NativeRecoveryPolicy
    {
        /// <summary>|position - last target| that still counts as at the target; null = 0.1.</summary>
        public double? PositionToleranceMm { get; set; }

        /// <summary>How often to look for the port / the unit while waiting; null = 0.5 s.</summary>
        public double? PollS { get; set; }

        /// <summary>The power-up rate probed first after a power loss; null = 38400.</summary>
        public int? BootBaudRate { get; set; }

        /// <summary>null = true.</summary>
        public bool? RestoreBaudRate { get; set; }

        /// <summary>Take PC control (PMSL) again when the application held it; null = true.</summary>
        public bool? RestoreControl { get; set; }

        /// <summary>null = true.</summary>
        public bool? RestoreSafetySpeed { get; set; }

        /// <summary>Reset a message-level alarm (others are left for the operator); null = true.</summary>
        public bool? ClearMessageAlarms { get; set; }

        /// <summary>Compare the parameter classes with the session's copy / the backup; null = false.</summary>
        public bool? CheckParameters { get; set; }

        /// <summary>(0.6) Without a session (none tracked) refuse (<see cref="RecoveryException"/>) instead of using the identity read last; null = false.</summary>
        public bool? RequireSession { get; set; }

        internal static IadRecoveryPolicy ToNative(NativeRecoveryPolicy? policy)
        {
            var p = SizedStruct.Fresh<IadRecoveryPolicy>();
            NativeErrors.Check(NativeMethods.iad_recovery_policy_init(ref p));
            if (policy is null)
            {
                return p;
            }

            p.PositionToleranceMm = policy.PositionToleranceMm ?? p.PositionToleranceMm;
            p.PollS = policy.PollS ?? p.PollS;
            p.BootBaudRate = policy.BootBaudRate ?? p.BootBaudRate;
            p.RestoreBaudRate = policy.RestoreBaudRate is { } a ? Known.Flag(a) : p.RestoreBaudRate;
            p.RestoreControl = policy.RestoreControl is { } b ? Known.Flag(b) : p.RestoreControl;
            p.RestoreSafetySpeed = policy.RestoreSafetySpeed is { } c ? Known.Flag(c) : p.RestoreSafetySpeed;
            p.ClearMessageAlarms = policy.ClearMessageAlarms is { } d ? Known.Flag(d) : p.ClearMessageAlarms;
            p.CheckParameters = policy.CheckParameters is { } e ? Known.Flag(e) : p.CheckParameters;
            p.RequireSession = policy.RequireSession is { } f ? Known.Flag(f) : p.RequireSession;
            return p;
        }
    }

    /// <summary>
    /// What a recovery found and did (iad_recovery_summary; C++ recovery::RecoveryReport). The full report (restart
    /// signs, actions, notes, row and parameter differences, new alarm records) is <see cref="Json"/>.
    /// </summary>
    internal sealed class NativeRecoverySummary
    {
        internal unsafe NativeRecoverySummary(IadRecoverySummary r, string json)
        {
            Status = r.Status.RawWords > 0 ? new NativeStatus(r.Status) : null;
            PositionMm = Known.Double(r.PositionMm);
            LastTargetMm = Known.Double(r.LastTargetMm);
            PositionErrorMm = Known.Double(r.PositionErrorMm);
            ElapsedS = Known.Double(r.ElapsedS);
            BaudRate = r.BaudRate > 0 ? r.BaudRate : (int?)null;
            BaudRateFound = r.BaudRateFound > 0 ? r.BaudRateFound : (int?)null;
            AlarmCode = r.AlarmCode;
            Restarted = Known.Bool(r.Restarted);
            SameUnit = Known.Bool(r.SameUnit);
            PortReopened = Known.Bool(r.PortReopened);
            BaudRestored = Known.Bool(r.BaudRestored);
            ControlRestored = Known.Bool(r.ControlRestored);
            SafetySpeedRestored = Known.Bool(r.SafetySpeedRestored);
            ControlKept = Known.Bool(r.ControlKept);
            NeedsHome = Known.Bool(r.NeedsHome);
            NeedsAttention = Known.Bool(r.NeedsAttention);
            ReadyToMove = Known.Bool(r.ReadyToMove);
            FaultAlarm = Known.Bool(r.FaultAlarm);
            AtTarget = Known.Tri(r.AtTarget);
            ServoOn = Known.Tri(r.ServoOn);
            Homed = Known.Tri(r.Homed);
            ModbusControl = Known.Tri(r.ModbusControl);
            SafetySpeed = Known.Tri(r.SafetySpeed);
            NoteCount = r.NoteCount;
            Axis = Utf8.FromFixed(r.Axis, SizedStruct.AxisNameSize);
            Cause = Utf8.FromFixed(r.Cause, 32);
            Summary = Utf8.FromFixed(r.Summary, 256);
            Json = json;
            Notes = (MiniJson.TryParse(json) as JsonObject)?.Strings("notes") ?? Array.Empty<string>();
        }

        /// <summary>The last status read; null when none.</summary>
        public NativeStatus? Status { get; }

        public double? PositionMm { get; }

        public double? LastTargetMm { get; }

        public double? PositionErrorMm { get; }

        public double? ElapsedS { get; }

        /// <summary>The rate in use afterwards; null when not known.</summary>
        public int? BaudRate { get; }

        /// <summary>The rate the unit was found at; null when not searched.</summary>
        public int? BaudRateFound { get; }

        public int AlarmCode { get; }

        public bool Restarted { get; }

        public bool SameUnit { get; }

        public bool PortReopened { get; }

        public bool BaudRestored { get; }

        public bool ControlRestored { get; }

        public bool SafetySpeedRestored { get; }

        /// <summary>(0.6) PC control was held before the fault and still is: nothing written.</summary>
        public bool ControlKept { get; }

        /// <summary>HEND lost: home before the next move.</summary>
        public bool NeedsHome { get; }

        /// <summary>Anything to decide before the next move.</summary>
        public bool NeedsAttention { get; }

        /// <summary>Servo ON, homed, PC control, no alarm, no E-stop.</summary>
        public bool ReadyToMove { get; }

        /// <summary>An alarm that blocks motion is still active.</summary>
        public bool FaultAlarm { get; }

        public bool? AtTarget { get; }

        public bool? ServoOn { get; }

        public bool? Homed { get; }

        public bool? ModbusControl { get; }

        public bool? SafetySpeed { get; }

        public int NoteCount { get; }

        /// <summary>The notes (from <see cref="Json"/>).</summary>
        public IReadOnlyList<string> Notes { get; }

        public string Axis { get; }

        /// <summary>The link state the recovery started from.</summary>
        public string Cause { get; }

        /// <summary>"G: recovered from unit_silent, unit restarted, needs home".</summary>
        public string Summary { get; }

        /// <summary>The full report (iad_axis_last_result_json right after the recovery; "" when not available).</summary>
        public string Json { get; }

        /// <inheritdoc />
        public override string ToString() => Summary;
    }

    // ------------------------------------------------------------------------------------------ self-test

    /// <summary>Summary of a self-test (iad_check_summary; C++ CheckReport). The JSON report is <see cref="Json"/>.</summary>
    internal sealed class NativeCheckSummary
    {
        internal NativeCheckSummary(in IadCheckSummary s, string json)
        {
            Passed = Known.Bool(s.Passed);
            Items = s.Items;
            Failed = s.Failed;
            Warned = s.Warned;
            Skipped = s.Skipped;
            Json = json;
        }

        /// <summary>No line failed (WARN and SKIP do not fail the check).</summary>
        public bool Passed { get; }

        public int Items { get; }

        public int Failed { get; }

        public int Warned { get; }

        public int Skipped { get; }

        /// <summary>The report in the Python edition's layout (iad_axis_last_result_json right after the test).</summary>
        public string Json { get; }
    }

    /// <summary>Lists the ABI joins into one text field.</summary>
    internal static class NativeLists
    {
        /// <summary>The items of a "; "-joined field (the JSON form has them whole: prefer it).</summary>
        internal static IReadOnlyList<string> Split(string joined, int count) =>
            count <= 0 || joined.Length == 0
                ? Array.Empty<string>()
                : joined.Split(new[] { "; " }, StringSplitOptions.RemoveEmptyEntries).ToList();

        /// <summary>A row list of the recovery and backup calls: null = none / the default, else "0,1,5".</summary>
        internal static string? Rows(IEnumerable<int>? rows) =>
            rows is null ? null : string.Join(",", rows.Select(r => r.ToString(CultureInfo.InvariantCulture)));

        /// <summary>Slave numbers as "1,2".</summary>
        internal static string? Slaves(IEnumerable<int>? slaves) => Rows(slaves);
    }
}
