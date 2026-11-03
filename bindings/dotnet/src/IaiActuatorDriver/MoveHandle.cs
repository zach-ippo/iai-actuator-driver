using System;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver
{
    /// <summary>
    /// A move started without waiting (<see cref="Axis.StartMoveTo"/>, <see cref="Axis.StartMoveBy"/>,
    /// <see cref="Axis.StartJog"/>): the command has been sent and every limit check has passed. Wait for it
    /// (<see cref="Wait"/>, <see cref="WaitAsync"/>), poll it (<see cref="Poll"/>, <see cref="IsDone"/>) or stop it
    /// (<see cref="Cancel"/>). One thread uses a handle at a time, except <see cref="Cancel"/>, which may be called from
    /// another thread while a wait runs. <see cref="Dispose"/> releases the handle only: the move itself is not affected
    /// (stop it with <see cref="Cancel"/> or <see cref="Axis.Stop"/>).
    /// </summary>
    public sealed class MoveHandle : IDisposable
    {
        private readonly NativeMoveHandle _inner;
        private readonly object _sync = new object();
        private Status? _result;
        private bool _cancelled;

        internal MoveHandle(Axis axis, NativeMoveHandle inner)
        {
            Axis = axis;
            _inner = inner;
        }

        /// <summary>The axis that runs the move.</summary>
        public Axis Axis { get; }

        /// <summary>The handle was released (<see cref="Dispose"/>).</summary>
        public bool IsClosed => _inner.IsClosed;

        /// <summary>The target (units), rounded to 0.001.</summary>
        public double Target => Snapshot().Target;

        /// <summary>The in-position band (units).</summary>
        public double Band => Snapshot().Band;

        /// <summary>Seconds the move may take from when it was sent; <c>null</c> when not known.</summary>
        public double? TimeoutS => Snapshot().TimeoutS;

        /// <summary>The completion rule the move uses.</summary>
        public CompletionRule Completion => Snapshot().Completion;

        /// <summary>A decel stop was sent through <see cref="Cancel"/>.</summary>
        public bool Cancelled => Snapshot().Cancelled || _cancelled;

        /// <summary>The final status once the move is done (or was cancelled); <c>null</c> before. No I/O.</summary>
        public Status? Result
        {
            get
            {
                lock (_sync)
                {
                    if (_result != null)
                    {
                        return _result;
                    }
                }

                return Snapshot().Result;
            }
        }

        /// <summary>What the handle knows, without any I/O: target, band, time limit, completion rule and the result once known.</summary>
        public MoveInfo Info => Snapshot();

        private MoveInfo Snapshot()
        {
            var info = _inner.Info;
            Status? result;
            lock (_sync)
            {
                result = _result ??= info.Result?.ToRecord();
            }

            return new MoveInfo(info.Target, info.Band, info.TimeoutS, info.Completion.FromNative(), info.Cancelled, result);
        }

        private Status Finish(NativeStatus status)
        {
            var record = status.ToRecord();
            lock (_sync)
            {
                _result ??= record;
                return _result;
            }
        }

        /// <summary>
        /// One check without waiting: <c>true</c> once the move is done (decided on one sample; the default
        /// <see cref="CompletionRule.Safe"/> rule needs its settle window over several calls, so poll in a loop or use
        /// <see cref="Wait"/>). Throws like <see cref="Wait"/> on an alarm or a lost servo (the axis stopped).
        /// </summary>
        public bool IsDone()
        {
            lock (_sync)
            {
                if (_result != null)
                {
                    return true;
                }
            }

            if (!_inner.IsDone)
            {
                return false;
            }

            Snapshot();
            return true;
        }

        /// <summary>
        /// Block until the move is done and return the final status. <paramref name="timeout"/> (seconds) <c>null</c> = the
        /// time left of the move's own estimate (at least the settle time + 1 s). On a timeout or an alarm a decel stop is sent
        /// first and <see cref="MotionTimeoutException"/> / <see cref="EcAlarmException"/> raised. A cancelled
        /// <paramref name="cancellationToken"/> sends a decel stop and raises <see cref="OperationCanceledException"/>.
        /// </summary>
        public Status Wait(double? timeout = null, CancellationToken cancellationToken = default)
        {
            lock (_sync)
            {
                if (_result != null)
                {
                    return _result;
                }
            }

            return Finish(_inner.Wait(timeout, cancellationToken));
        }

        /// <summary><see cref="Wait"/> on a thread-pool thread.</summary>
        public Task<Status> WaitAsync(double? timeout = null, CancellationToken cancellationToken = default) =>
            Task.Run(() => Wait(timeout, cancellationToken), CancellationToken.None);

        /// <summary>
        /// Wait at most <paramref name="maxWaitS"/> seconds for the move to finish, without stopping it when the slice ends:
        /// the final status once done, else <c>null</c>. Once the move's own deadline has passed this is <see cref="Wait"/>
        /// (a decel stop and <see cref="MotionTimeoutException"/> when it still has not finished).
        /// </summary>
        public Status? Poll(double maxWaitS)
        {
            lock (_sync)
            {
                if (_result != null)
                {
                    return _result;
                }
            }

            var status = _inner.Poll(maxWaitS);
            return status is null ? null : Finish(status);
        }

        /// <summary>
        /// Decel stop (the STOP coil sequence of <see cref="Axis.Stop"/>); the status where the axis stopped becomes the
        /// <see cref="Result"/>. With <paramref name="wait"/> a move that still runs after <paramref name="timeout"/>
        /// raises <see cref="MotionTimeoutException"/>. Safe from another thread while a wait runs on this handle.
        /// </summary>
        public Status Cancel(bool wait = false, double timeout = 5.0)
        {
            var status = _inner.Cancel(wait, timeout).ToRecord();
            lock (_sync)
            {
                _cancelled = true;
                _result ??= status;
            }

            return status;
        }

        /// <summary>Release the handle (the move itself is not affected). Safe to call twice.</summary>
        public void Dispose() => _inner.Dispose();

        /// <inheritdoc />
        public override string ToString()
        {
            if (IsClosed)
            {
                return "MoveHandle(closed)";
            }

            var info = Snapshot();
            var state = info.Cancelled ? "cancelled" : info.Result != null ? "done" : "running";
            return $"MoveHandle(target={info.Target:0.###}, band={info.Band:0.###}, {(info.Completion == CompletionRule.Fast ? "fast" : "safe")}, {state})";
        }
    }
}
