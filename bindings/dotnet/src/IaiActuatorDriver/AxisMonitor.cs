using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    /// <summary>Settings of the status monitor (<see cref="Axis.StartMonitor"/>).</summary>
    public sealed record MonitorOptions
    {
        /// <summary>Pause between two polls with the port lock free (0 .. 10 s; 0 polls back to back).</summary>
        public double GapS { get; init; } = 0.002;

        /// <summary>
        /// How often the link check runs (one full status read compared with the unit's session: 0 .. 3600 s); <c>null</c> or 0
        /// = never (a lost link is still reported from the polls).
        /// </summary>
        public double? LinkCheckS { get; init; } = 1.0;

        /// <summary>Retries per poll (default 0: one attempt, then the lock is released).</summary>
        public int Retries { get; init; }

        /// <summary>Events the axis queues for its events; the oldest are dropped beyond it (1 .. 65536).</summary>
        public int QueueCapacity { get; init; } = 1024;
    }

    /// <summary>One published poll of the status monitor: the lean status with its time stamps and the monitor's counters.</summary>
    /// <param name="Status">The lean status (PNOW, ALMC, DSS1, DSS2, DSSE: no velocity while moving); <c>null</c> before the first poll.</param>
    /// <param name="Seq">The sample's sequence number; 0 before the first poll.</param>
    /// <param name="TStartS">Monotonic seconds just before the request went out.</param>
    /// <param name="TS">Monotonic seconds after the answer was decoded.</param>
    /// <param name="AgeS">Seconds since <paramref name="TS"/>.</param>
    /// <param name="PollRateHz">Answered polls per second since the start.</param>
    /// <param name="Polls">Answered polls.</param>
    /// <param name="ErrorCount">Transient errors in total.</param>
    /// <param name="Running">The monitor runs.</param>
    public sealed record MonitorSample(
        Status? Status, long Seq, double? TStartS, double? TS, double? AgeS, double? PollRateHz, long Polls, long ErrorCount, bool Running);

    /// <summary>The axis's status changed (PNOW, ALMC, DSS1, DSS2 or DSSE): <see cref="Axis.StatusChanged"/>.</summary>
    public sealed class StatusChangedEventArgs : EventArgs
    {
        internal StatusChangedEventArgs(Status status, Status? previous, double timeS)
        {
            Status = status;
            Previous = previous;
            TimeS = timeS;
        }

        /// <summary>The monitor's sample (the lean 8-word status).</summary>
        public Status Status { get; }

        /// <summary>The status of the previous change event; <c>null</c> for the first.</summary>
        public Status? Previous { get; }

        /// <summary>Monotonic seconds when the event was queued.</summary>
        public double TimeS { get; }
    }

    /// <summary>The axis became done by the fast rule against its last target: <see cref="Axis.MotionDone"/> (a rising edge).</summary>
    public sealed class MotionDoneEventArgs : EventArgs
    {
        internal MotionDoneEventArgs(Status status, double timeS)
        {
            Status = status;
            TimeS = timeS;
        }

        /// <summary>The sample that showed the axis done.</summary>
        public Status Status { get; }

        /// <summary>Monotonic seconds when the event was queued.</summary>
        public double TimeS { get; }
    }

    /// <summary>One change of an axis's link state (<see cref="Axis.LinkChanged"/>).</summary>
    /// <param name="Kind">"link_lost", "unit_restarted", "unit_replaced", "recovered" (a fault or link-state change keeps the core's name).</param>
    /// <param name="Axis">The axis ("COM3:1").</param>
    /// <param name="Message">What happened, in words.</param>
    /// <param name="Cause">The link state behind it ("port_vanished", "unit_silent", ...); empty when none.</param>
    /// <param name="Time">The wall-clock time of the event.</param>
    public sealed record RecoveryEvent(string Kind, string Axis, string Message, string Cause, DateTimeOffset Time)
    {
        /// <summary>The event as a JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["kind"] = Kind,
            ["axis"] = Axis,
            ["message"] = Message,
            ["cause"] = Cause.Length == 0 ? null : Cause,
            ["time"] = Time.ToString("o", System.Globalization.CultureInfo.InvariantCulture),
        };
    }

    /// <summary>A link event of the axis: <see cref="Axis.LinkChanged"/>.</summary>
    public sealed class LinkEventArgs : EventArgs
    {
        internal LinkEventArgs(RecoveryEvent @event) => Event = @event;

        /// <summary>The event.</summary>
        public RecoveryEvent Event { get; }

        /// <summary>"link_lost", "unit_restarted", "unit_replaced", "recovered".</summary>
        public string Kind => Event.Kind;

        /// <summary>What happened, in words.</summary>
        public string Message => Event.Message;
    }

    /// <summary>
    /// The background status monitor of an axis (<see cref="Axis.StartMonitor"/>, <see cref="Axis.EnableFast"/>): a thread in the core
    /// that reads a lean status block (PNOW, ALMC, DIPM, DOPM, DSS1, DSS2, DSSE: <c>FC03 0x9000 x8</c>) as fast as the link allows,
    /// with no retries and the port lock free between two polls, so a STOP or another axis of the port gets the line at once. It
    /// only reads. The axis's <see cref="Axis.StatusChanged"/>, <see cref="Axis.MotionDone"/> and <see cref="Axis.LinkChanged"/> events
    /// come from it; the waits here return when a new sample arrives. A transport error (the adapter unplugged) or a closed bus ends
    /// it (<see cref="Running"/> false, <see cref="Error"/> says why); <see cref="Axis.Recover"/> brings the axis back and restarts it.
    /// </summary>
    public sealed class AxisMonitor
    {
        /// <summary>The longest wait of the calling thread between two looks at its cancellation token.</summary>
        private const double SliceS = 0.1;

        internal AxisMonitor(Axis axis) => Axis = axis;

        /// <summary>The axis this monitor watches.</summary>
        public Axis Axis { get; }

        private JsonObject? _last;

        /// <summary>The monitor's state now; <c>null</c> once it was stopped (the core drops it).</summary>
        private JsonObject? Live()
        {
            var now = MiniJson.TryParse(Axis.Core.MonitorInfoJson()) as JsonObject;
            if (now != null)
            {
                _last = now;
            }

            return now;
        }

        /// <summary>The monitor's state now, or what it was when it was last looked at (counters stay readable after a stop).</summary>
        private JsonObject Info() => Live() ?? _last ?? new JsonObject();

        /// <summary>Remember the counters before the monitor is stopped.</summary>
        internal void Snapshot()
        {
            try
            {
                Live();
            }
            catch (Exception exc) when (exc is EcDriverException || exc is ObjectDisposedException)
            {
                // the axis is closed: what was seen last stays
            }
        }

        /// <summary>The monitor thread runs.</summary>
        public bool Running => Live()?.Bool("running") == true;

        /// <summary>The monitor is held (<see cref="Suspend"/>, a baud change, a software reset or a recovery): no polls meanwhile.</summary>
        public bool Suspended => Live()?.Bool("suspended") == true;

        /// <summary>
        /// The core's last sample, or <c>null</c> once the monitor was stopped (<see cref="Axis.StopMonitor"/>, <see cref="Axis.DisableFast"/>,
        /// <see cref="Axis.Close"/>: the core drops it and says "not ready").
        /// </summary>
        private NativeMonitorSample? LatestOrNull()
        {
            try
            {
                return Axis.Core.MonitorLatest();
            }
            catch (EcNotReadyException)
            {
                return null;
            }
        }

        /// <summary>
        /// The last published sample (the lean status and its time stamps), <c>null</c> before the first poll and once the monitor
        /// was stopped.
        /// </summary>
        public MonitorSample? Latest
        {
            get
            {
                var raw = LatestOrNull();
                if (raw is null)
                {
                    return null;
                }

                var sample = SampleOf(raw);
                return sample.Status is null ? null : sample;
            }
        }

        /// <summary>The status of <see cref="Latest"/>.</summary>
        public Status? Status => Latest?.Status;

        /// <summary>Answered polls.</summary>
        public long Polls => (long)(Info().Number("polls") ?? 0);

        /// <summary>Transient errors in total (timeouts, CRC and frame errors).</summary>
        public long ErrorCount => (long)(Info().Number("error_count") ?? 0);

        /// <summary>Answered polls per second since the start.</summary>
        public double? PollRateHz => Info().Number("poll_rate_hz");

        /// <summary>Link checks run so far.</summary>
        public long LinkChecks => (long)(Info().Number("link_checks") ?? 0);

        /// <summary>Seconds since the last poll was answered; <c>null</c> before the first.</summary>
        public double? AgeS => Info().Number("age_s");

        /// <summary>The pause between two polls (s).</summary>
        public double GapS => Info().Number("gap_s") ?? 0.0;

        /// <summary>The axis's link state: "ok", "port_vanished", "unit_silent", "unit_restarted", "unit_replaced", "recovering".</summary>
        public string LinkState => Info().String("link_state") ?? "ok";

        /// <summary>The error that ended the monitor ("Type: message": the port gone, the bus closed); <c>null</c> otherwise.</summary>
        public string? Error => NonEmpty(Info().String("error"));

        /// <summary>The last transient error (a timeout, a CRC error ...); <c>null</c> when none.</summary>
        public string? LastError => NonEmpty(Info().String("last_error"));

        private static string? NonEmpty(string? text) => string.IsNullOrEmpty(text) ? null : text;

        internal static MonitorSample SampleOf(NativeMonitorSample s) =>
            new MonitorSample(s.Status?.ToRecord(), s.Seq, s.TStartS, s.TS, s.AgeS, s.PollRateHz, s.Polls, s.ErrorCount, s.Running);

        /// <summary>
        /// The latest status when the monitor runs and it is younger than <paramref name="maxAgeS"/> seconds, else <c>null</c>
        /// (the axis's own moves use such a sample instead of a fresh read).
        /// </summary>
        public Status? Fresh(double maxAgeS = 0.05)
        {
            var sample = Latest;
            return sample is null || !Running || (sample.AgeS ?? double.MaxValue) > maxAgeS ? null : sample.Status;
        }

        /// <summary>
        /// The completion rule of a fast move on one sample (<see cref="CompletionRule.Fast"/>): PEND, |position - target| within
        /// the band of the axis's last commanded move (no position check without one), DSSE MOVE clear, not homing, not paused.
        /// </summary>
        public bool IsDone(Status status)
        {
            var target = Axis.Core.MonitorTarget();
            return FastDone(status, target?.Target, target?.Band);
        }

        /// <summary>The fast completion rule on one sample (<paramref name="band"/> 0.1 when <c>null</c>).</summary>
        public static bool FastDone(Status status, double? target = null, double? band = null)
        {
            if (status is null)
            {
                throw new ArgumentNullException(nameof(status));
            }

            if (!status.InPosition || status.MoveBit || status.Homing || status.Paused)
            {
                return false;
            }

            return target is null || Math.Abs(status.PositionMm - target.Value) <= (band ?? 0.1) + 1e-6;
        }

        /// <summary>
        /// The first sample newer than <paramref name="afterSeq"/> (<c>null</c> = the next one). <c>null</c> on a timeout
        /// (<paramref name="timeout"/> s, <c>null</c> = 1 s) or when the monitor stopped; raises when it ended on an error.
        /// </summary>
        public MonitorSample? WaitUpdate(long? afterSeq = null, double? timeout = null)
        {
            var after = afterSeq ?? LatestOrNull()?.Seq;
            if (after is null)
            {
                return null;  // the monitor was stopped
            }

            try
            {
                var sample = Axis.Core.MonitorWaitUpdate(after.Value, timeout);
                return sample is null ? null : SampleOf(sample);
            }
            catch (EcNotReadyException)
            {
                return null;  // stopped while waiting
            }
        }

        /// <summary>
        /// Block until a sample (the latest one included) satisfies <paramref name="predicate"/> and return its status; <c>null</c> on
        /// a timeout (<paramref name="timeout"/> s, <c>null</c> = no limit), when the monitor stops or the token is cancelled
        /// (<paramref name="cancellationToken"/>: <see cref="OperationCanceledException"/>); raises when the monitor ended on an
        /// error.
        /// </summary>
        public Status? WaitFor(Func<Status, bool> predicate, double? timeout = null, CancellationToken cancellationToken = default)
        {
            if (predicate is null)
            {
                throw new ArgumentNullException(nameof(predicate));
            }

            var clock = Stopwatch.StartNew();
            long seen = 0;
            while (true)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var latest = Latest;
                if (latest != null && latest.Seq > seen)
                {
                    seen = latest.Seq;
                    if (predicate(latest.Status!))
                    {
                        return latest.Status;
                    }
                }

                var left = timeout is { } limit ? limit - clock.Elapsed.TotalSeconds : double.PositiveInfinity;
                if (left <= 0 || (!Running && Error is null))
                {
                    return null;
                }

                NativeMonitorSample? sample;
                try
                {
                    sample = Axis.Core.MonitorWaitUpdate(seen, Math.Min(SliceS, left));
                }
                catch (EcNotReadyException)
                {
                    return null;  // the monitor was stopped while this waited
                }

                if (sample?.Status != null && sample.Seq > seen)
                {
                    seen = sample.Seq;
                    if (predicate(sample.Status.ToRecord()))
                    {
                        return sample.Status.ToRecord();
                    }
                }
            }
        }

        /// <summary>One wait of the core for the axis done by the fast rule; <c>null</c> on a timeout and once the monitor was stopped.</summary>
        private NativeStatus? DoneOrNull(double? timeout)
        {
            try
            {
                return Axis.Core.MonitorWaitDone(timeout);
            }
            catch (EcNotReadyException)
            {
                return null;
            }
        }

        /// <summary>
        /// Block until a sample shows the axis done by the fast rule (<see cref="IsDone"/>) against its last commanded target;
        /// the status, or <c>null</c> on a timeout (<paramref name="timeout"/> s, <c>null</c> = 10 s) or once the monitor was stopped.
        /// </summary>
        public Status? WaitDone(double? timeout = null, CancellationToken cancellationToken = default)
        {
            if (!cancellationToken.CanBeCanceled)
            {
                return DoneOrNull(timeout)?.ToRecord();
            }

            var clock = Stopwatch.StartNew();
            var limit = timeout ?? 10.0;
            while (true)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var left = limit - clock.Elapsed.TotalSeconds;
                var status = DoneOrNull(Math.Max(0.0, Math.Min(SliceS, left)));
                if (status != null)
                {
                    return status.ToRecord();
                }

                if (left <= SliceS)
                {
                    return null;
                }
            }
        }

        /// <summary>
        /// Hold the monitor (no polls) until the returned object is disposed; a poll in flight is waited for, at most
        /// <paramref name="timeout"/> seconds (<c>null</c> = 1 s). Nests. A baud change, a software reset and
        /// <see cref="Axis.Recover"/> hold the monitor themselves.
        /// </summary>
        public IDisposable Suspend(double? timeout = null)
        {
            Axis.Core.SuspendMonitor(timeout);
            return new Hold(Axis.Core);
        }

        /// <summary>Stop the monitor (<paramref name="timeout"/> s, default 1; same as <see cref="Axis.StopMonitor"/>).</summary>
        public void Stop(double timeout = 1.0) => Axis.StopMonitor(timeout);

        /// <inheritdoc />
        public override string ToString()
        {
            var live = Live();
            var info = live ?? Info();
            return $"AxisMonitor({info.String("name")}, {(live?.Bool("running") == true ? "running" : "stopped")}, {info.Number("polls") ?? 0:0} polls)";
        }

        private sealed class Hold : IDisposable
        {
            private NativeAxis? _axis;

            internal Hold(NativeAxis axis) => _axis = axis;

            public void Dispose()
            {
                var axis = Interlocked.Exchange(ref _axis, null);
                if (axis is null)
                {
                    return;
                }

                try
                {
                    axis.ResumeMonitor();
                }
                catch (ObjectDisposedException)
                {
                    // the axis was closed meanwhile: nothing is held any more
                }
            }
        }
    }
}
