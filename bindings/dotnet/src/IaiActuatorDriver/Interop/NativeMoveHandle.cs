using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// (ABI 0.2) A move started without waiting (iad_move_handle; C++ MoveHandle), from <see cref="NativeAxis.StartMoveTo"/>,
    /// <see cref="NativeAxis.StartMoveBy"/> or <see cref="NativeAxis.StartJog"/>. The command is sent and every check has
    /// passed. One thread uses a handle at a time, except <see cref="Cancel"/>, which may be called from another thread
    /// while <see cref="Wait"/> or <see cref="Poll"/> runs. <see cref="Dispose"/> releases the handle only (the move itself
    /// is not affected); the handle keeps its axis object alive in the library, not its port.
    /// </summary>
    internal sealed class NativeMoveHandle : IDisposable
    {
        /// <summary>The slice <see cref="Wait"/> polls in when it can be cancelled.</summary>
        private const double SliceS = 0.1;

        private readonly MoveHandle _handle;

        private NativeMoveHandle(NativeAxis axis, MoveHandle handle)
        {
            Axis = axis;
            _handle = handle;
        }

        /// <summary>The axis that runs the move.</summary>
        public NativeAxis Axis { get; }

        /// <summary>The native handle.</summary>
        public MoveHandle Handle => _handle;

        /// <summary>The handle was released.</summary>
        public bool IsClosed => _handle.IsClosed;

        internal static NativeMoveHandle Adopt(NativeAxis axis, int result, MoveHandle handle)
        {
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return new NativeMoveHandle(axis, handle);
        }

        /// <summary>
        /// One check, no waiting (MoveHandle::done; the safe rule needs its settle window over several calls). Throws like
        /// <see cref="Wait"/> on an alarm or a lost servo (the axis stopped).
        /// </summary>
        public bool IsDone
        {
            get
            {
                var done = 0;
                NativeBus.Run(() => NativeMethods.iad_move_handle_done(_handle, out done, out _));
                return Known.Bool(done);
            }
        }

        /// <summary>
        /// Block until done (MoveHandle::wait) and return the final status. <paramref name="timeout"/> null = the time left
        /// of the move's estimate (at least the settle time + 1 s). On timeout or alarm a decel stop is sent and
        /// <see cref="MotionTimeoutException"/> / <see cref="EcAlarmException"/> thrown. With a
        /// <paramref name="cancellationToken"/> the wait runs in short slices: a cancel sends a decel stop
        /// (<see cref="NativeAxis.Cancel"/>) and throws <see cref="OperationCanceledException"/>.
        /// </summary>
        public NativeStatus Wait(double? timeout = null, CancellationToken cancellationToken = default)
        {
            if (!cancellationToken.CanBeCanceled)
            {
                IadStatus st = default;
                NativeBus.Run(() => NativeMethods.iad_move_handle_wait(_handle, Known.OrNaN(timeout), out st));
                return new NativeStatus(st);
            }

            var clock = Stopwatch.StartNew();
            using (cancellationToken.Register(() => CancelQuietly()))
            {
                try
                {
                    while (true)
                    {
                        if (cancellationToken.IsCancellationRequested)
                        {
                            // Cancelled between two slices: the axis cancel reached no wait, so stop the move here.
                            Cancel();
                            throw new OperationCanceledException(cancellationToken);
                        }

                        var slice = SliceS;
                        if (timeout is { } limit)
                        {
                            var left = limit - clock.Elapsed.TotalSeconds;
                            if (left <= 0)
                            {
                                // The caller's time is up: one last check, then the decel stop and the timeout error.
                                IadStatus last = default;
                                NativeBus.Run(() => NativeMethods.iad_move_handle_wait(_handle, 0.0, out last));
                                return new NativeStatus(last);
                            }

                            slice = Math.Min(slice, left);
                        }

                        var status = Poll(slice);
                        if (status != null)
                        {
                            return status;
                        }
                    }
                }
                catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
                {
                    throw new OperationCanceledException("the wait for the move was cancelled (a decel stop was sent)", cancellationToken);
                }
            }
        }

        /// <summary><see cref="Wait"/> on a thread-pool thread.</summary>
        public Task<NativeStatus> WaitAsync(double? timeout = null, CancellationToken cancellationToken = default) =>
            Task.Run(() => Wait(timeout, cancellationToken), CancellationToken.None);

        /// <summary>
        /// Wait at most <paramref name="maxWaitS"/> for the move to finish without stopping it when that slice ends
        /// (MoveHandle::poll): the final status once done, else null. Once the move's deadline has passed this is
        /// <see cref="Wait"/> (decel stop and <see cref="MotionTimeoutException"/> when it still has not finished).
        /// <see cref="NativeAxis.Cancel"/> ends a slice at once (decel stop, <see cref="OperationCanceledException"/>).
        /// </summary>
        public NativeStatus? Poll(double maxWaitS)
        {
            var done = 0;
            IadStatus st = default;
            NativeBus.Run(() => NativeMethods.iad_move_handle_poll(_handle, maxWaitS, out done, out st));
            return Known.Bool(done) ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// Decel stop (MoveHandle::cancel = Axis::stop); the stop status becomes the result unless one is known.
        /// <paramref name="timeout"/> null = 5 s. Safe from another thread while a wait runs on this handle (that wait
        /// then ends as a move does when the axis is stopped during it).
        /// </summary>
        public NativeStatus Cancel(bool wait = false, double? timeout = null)
        {
            IadStatus st = default;
            NativeBus.Run(() => NativeMethods.iad_move_handle_cancel(_handle, Known.Flag(wait), Known.OrNaN(timeout), out st));
            return new NativeStatus(st);
        }

        /// <summary>What the handle knows, no I/O: target, band, deadline and the result once known.</summary>
        public NativeMoveInfo Info
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_move_handle_result(_handle, out var info));
                return new NativeMoveInfo(info);
            }
        }

        /// <summary>The same as JSON (MoveHandle::to_json: {"target_mm", "band_mm", "completion", "timeout_s", "done", "cancelled", "position_mm"}).</summary>
        public string Json() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_move_handle_json(_handle, b, s, out r), 4096);

        /// <summary>Release the handle (the move itself is not affected). Safe to call twice.</summary>
        public void Dispose() => _handle.Dispose();

        /// <inheritdoc />
        public override string ToString() => $"NativeMoveHandle({Axis.Name})";

        private void CancelQuietly()
        {
            try
            {
                Axis.Cancel();
            }
            catch (Exception)
            {
                // The axis was closed: the wait ends with the closed handle's error.
            }
        }
    }
}
