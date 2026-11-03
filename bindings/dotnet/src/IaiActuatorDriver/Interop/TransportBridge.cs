using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// (ABI 0.3) An <see cref="ITransport"/> the core does not know (<see cref="FakeTransport"/>, the managed simulator, a
    /// customer's own transport, a test double) as a core transport through iad_transport_open_callbacks. The library gets
    /// static thunks that live as long as the process; each transport's state travels in the callbacks' user pointer (a
    /// <see cref="GCHandle"/> freed by the release callback, which the library calls once after its last use).
    /// </summary>
    /// <remarks>
    /// <list type="bullet">
    /// <item><description>Reads: one <see cref="ITransport.Read"/> per callback (real_timeout 0), so the core keeps the
    /// managed edition's rule: a transport answers within its own read timeout, as it is at that moment (a caller may
    /// raise it at any time). Only a wait the core shortens below the timeout it holds (a probe, a capped read; the
    /// set_read_timeout callback, before the request goes out) shortens <see cref="ITransport.ReadTimeout"/>, which is put
    /// back when the core puts its own back. The bus hands the core the transport's timeout as it is now before a probe
    /// or a scan (<see cref="TimeoutToSync"/>), so a timeout raised since the open is capped by the probe's wait too.</description></item>
    /// <item><description>Exceptions never cross into the library: a thunk stores what the transport threw (per thread;
    /// the callbacks run on the thread doing the work), gives the library its text through iad_callback_error and returns
    /// a failure. For an <see cref="IOException"/>, <see cref="UnauthorizedAccessException"/> or
    /// <see cref="InvalidOperationException"/> (the port failed) the failure is the port's: the core raises its
    /// TransportError (never retried; a link check takes it for a link fault) and <see cref="Raise"/> turns it into
    /// <see cref="ModbusException"/> "transport error: ..." with the original as inner exception. Any other exception is
    /// the caller's own error, not the port's: the thunk returns IAD_CALLBACK_ABORT, the core raises its EcDriverError
    /// (never retried, never a link fault: a link check or a recovery ends with it) and <see cref="Raise"/> throws the
    /// original exception itself. The bridge also keeps its last few failures, numbered: a cell's group command runs the
    /// ports on the core's own threads and finds what its axes' transports threw there with
    /// <see cref="FailureSince"/>.</description></item>
    /// <item><description>Closing: the core closes the transport when its bus closes. The close is passed on to
    /// <see cref="ITransport.Close"/> only when <see cref="ForwardClose"/> is set, so a bus the garbage collector finalises
    /// never closes a transport the program still uses (as the managed edition, which had no finalizer).</description></item>
    /// </list>
    /// The callbacks must not call the library (the core holds the bus lock while they run), except iad_callback_error.
    /// </remarks>
    internal sealed class TransportBridge
    {
        private static readonly NativeMethods.TransportWriteFn WriteThunk = OnWrite;
        private static readonly NativeMethods.TransportReadFn ReadThunk = OnRead;
        private static readonly NativeMethods.TransportResetInputFn ResetInputThunk = OnResetInput;
        private static readonly NativeMethods.TransportSetBaudRateFn SetBaudRateThunk = OnSetBaudRate;
        private static readonly NativeMethods.TransportSendBreakFn SendBreakThunk = OnSendBreak;
        private static readonly NativeMethods.TransportUserFn CloseThunk = OnClose;
        private static readonly NativeMethods.TransportUserFn ReleaseThunk = OnRelease;
        private static readonly NativeMethods.TransportSetReadTimeoutFn SetReadTimeoutThunk = OnSetReadTimeout;

        private static readonly IntPtr WritePtr = Marshal.GetFunctionPointerForDelegate(WriteThunk);
        private static readonly IntPtr ReadPtr = Marshal.GetFunctionPointerForDelegate(ReadThunk);
        private static readonly IntPtr ResetInputPtr = Marshal.GetFunctionPointerForDelegate(ResetInputThunk);
        private static readonly IntPtr SetBaudRatePtr = Marshal.GetFunctionPointerForDelegate(SetBaudRateThunk);
        private static readonly IntPtr SendBreakPtr = Marshal.GetFunctionPointerForDelegate(SendBreakThunk);
        private static readonly IntPtr ClosePtr = Marshal.GetFunctionPointerForDelegate(CloseThunk);
        private static readonly IntPtr ReleasePtr = Marshal.GetFunctionPointerForDelegate(ReleaseThunk);
        private static readonly IntPtr SetReadTimeoutPtr = Marshal.GetFunctionPointerForDelegate(SetReadTimeoutThunk);

        /// <summary>How many failures to keep per transport for <see cref="FailureSince"/>.</summary>
        private const int RecentFailures = 16;

        /// <summary>IAD_CALLBACK_ABORT: a callback's result "failed with an error that is not the port's" (the core: IAD_ERR_DRIVER).</summary>
        private const int CallbackAbort = 2;

        /// <summary>What a callback of this thread threw last (cleared by <see cref="ClearError"/> before each library call).</summary>
        [ThreadStatic]
        private static Exception? _error;

        /// <summary>The number of the last failure of any bridge (<see cref="FailureMark"/>).</summary>
        private static long _failureCount;

        /// <summary>This transport's last failures: (number, the text the core got, the exception). Lock it to use it.</summary>
        private readonly List<(long Number, string Text, Exception Error)> _recent = new List<(long, string, Exception)>();

        /// <summary>
        /// The read timeout the core holds for this transport (s; NaN: the transport has none to shorten): the one given
        /// at open, then the one <see cref="TimeoutToSync"/> handed over.
        /// </summary>
        private double _coreTimeoutS;

        /// <summary>The transport's own timeout while the core holds it shortened (set_read_timeout), to put back after.</summary>
        private TimeSpan? _restore;

        private TransportBridge(ITransport transport, bool forwardClose, double? openTimeoutS)
        {
            Transport = transport;
            ForwardClose = forwardClose;
            _coreTimeoutS = openTimeoutS ?? double.NaN;
        }

        /// <summary>The managed transport the callbacks call.</summary>
        internal ITransport Transport { get; }

        /// <summary>Pass the core's close on to <see cref="ITransport.Close"/> (set before an explicit close).</summary>
        internal volatile bool ForwardClose;

        /// <summary>
        /// A core transport on <paramref name="transport"/>. <paramref name="name"/> names it in the core's messages (null:
        /// "CALLBACK"). The baud-rate callback is given only when the transport has a rate now (as the managed edition: a
        /// transport without one never gets one). <paramref name="forwardClose"/>: see <see cref="ForwardClose"/>.
        /// </summary>
        internal static TransportHandle Open(ITransport transport, string? name, bool forwardClose) => Open(transport, name, forwardClose, out _);

        /// <inheritdoc cref="Open(ITransport, string?, bool)"/>
        /// <param name="transport">The managed transport.</param>
        /// <param name="name">Its name in the core's messages.</param>
        /// <param name="forwardClose">See <see cref="ForwardClose"/>.</param>
        /// <param name="bridge">The bridge the callbacks use (its failures and the core's timeout).</param>
        internal static TransportHandle Open(ITransport transport, string? name, bool forwardClose, out TransportBridge bridge)
        {
            if (transport is null)
            {
                throw new ArgumentNullException(nameof(transport));
            }

            NativeDriver.EnsureLoaded();
            var full = default(IadTransportCallbacks05);
            full.Callbacks.StructSize = (uint)Marshal.SizeOf<IadTransportCallbacks05>();  // 0.5: the whole struct, set_read_timeout too
            NativeErrors.Check(NativeMethods.iad_transport_callbacks_init_05(ref full));
            ref var callbacks = ref full.Callbacks;
            var rate = BaudRateOf(transport);
            var timeout = ReadTimeoutOf(transport);
            if (timeout.HasValue)
            {
                callbacks.ReadTimeoutS = timeout.Value;
            }

            callbacks.BaudRate = rate;
            callbacks.RealTimeout = 0;
            callbacks.Write = WritePtr;
            callbacks.Read = ReadPtr;
            callbacks.ResetInput = ResetInputPtr;
            callbacks.SetBaudRate = rate > 0 ? SetBaudRatePtr : IntPtr.Zero;
            callbacks.SendBreak = SendBreakPtr;
            callbacks.Close = ClosePtr;
            callbacks.Release = ReleasePtr;
            full.SetReadTimeout = SetReadTimeoutPtr;

            bridge = new TransportBridge(transport, forwardClose, timeout);
            var state = GCHandle.Alloc(bridge);
            var result = NativeMethods.iad_transport_open_callbacks_05(ref full, Utf8.Encode(name, nameof(name)), GCHandle.ToIntPtr(state), out var handle);
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                state.Free();  // nothing was taken: the release callback does not run
                handle.Dispose();
                throw error;
            }

            return handle;
        }

        /// <summary>Forget what an earlier callback of this thread threw (call before the library call that may fail).</summary>
        internal static void ClearError() => _error = null;

        /// <summary>
        /// The exception of a failed library call that used a bridged transport: the managed edition's exception for what
        /// the transport threw during the call (see the remarks), else the error map's. Call right after the call, on its
        /// thread.
        /// </summary>
        internal static Exception Raise(int result)
        {
            var thrown = _error;
            _error = null;
            if (thrown is null || !IsFailureOfTransport(thrown, result))
            {
                return NativeErrors.Capture(result);
            }

            if (IsTransportError(thrown))
            {
                return Wrapped(thrown, result, NativeErrors.LastErrorJson());
            }

            ExceptionDispatchInfo.Capture(thrown).Throw();
            return thrown;  // not reached
        }

        /// <summary>
        /// Whether a library call that failed with <paramref name="result"/> failed because of what a callback threw
        /// (<paramref name="thrown"/>): a port failure ends as the core's transport error, any other exception as its
        /// driver error (IAD_CALLBACK_ABORT).
        /// </summary>
        internal static bool IsFailureOfTransport(Exception thrown, int result) =>
            IsTransportError(thrown) ? result == (int)NativeResult.Transport : result == (int)NativeResult.Driver || result == (int)NativeResult.Transport;

        /// <summary>The number of the last failure of any bridge: a call's own failures are those numbered after it.</summary>
        internal static long FailureMark => Interlocked.Read(ref _failureCount);

        /// <summary>
        /// The managed edition's exception for a failure of this transport that the core reported as
        /// <paramref name="message"/> (an axis's error in a cell's group command, run on the core's thread) and that was
        /// thrown after <paramref name="mark"/> (<see cref="FailureMark"/> before the call): what <see cref="Raise"/> gives,
        /// the original exception for anything but a port failure. Null when no such failure is kept.
        /// </summary>
        internal Exception? FailureSince(long mark, string message, int result, string? json)
        {
            Exception? thrown = null;
            lock (_recent)
            {
                for (var i = _recent.Count - 1; i >= 0 && thrown is null; i--)
                {
                    var (number, text, error) = _recent[i];
                    if (number > mark && message.IndexOf(text, StringComparison.Ordinal) >= 0 && IsFailureOfTransport(error, result))
                    {
                        thrown = error;
                    }
                }
            }

            return thrown is null ? null : IsTransportError(thrown) ? Wrapped(thrown, result, json) : thrown;
        }

        /// <summary>
        /// The transport's read timeout when the core holds another one (a caller changed it since): the value to hand the
        /// core (iad_transport_set_read_timeout), the core's from then on. Null: nothing to do (the same, none, or the core
        /// holds it shortened for a request right now). Call under the bus's lock.
        /// </summary>
        internal double? TimeoutToSync()
        {
            if (_restore.HasValue)
            {
                return null;
            }

            var now = ReadTimeoutOf(Transport);
            var held = Volatile.Read(ref _coreTimeoutS);
            if (!now.HasValue || double.IsNaN(held) || Math.Abs(now.Value - held) < 1e-9)
            {
                return null;
            }

            Volatile.Write(ref _coreTimeoutS, now.Value);
            return now.Value;
        }

        private static ModbusException Wrapped(Exception thrown, int result, string? json)
        {
            var wrapped = new ModbusException("transport error: " + thrown.Message, thrown);
            NativeErrors.Tag(wrapped, result, "TransportError", json);
            return wrapped;
        }

        /// <summary>The exceptions the managed edition turned into "transport error" (the port failed underneath the bus).</summary>
        internal static bool IsTransportError(Exception exception) =>
            exception is IOException || exception is UnauthorizedAccessException || exception is InvalidOperationException;

        private static int BaudRateOf(ITransport transport)
        {
            try
            {
                var rate = transport.BaudRate;
                return rate > 0 ? rate : 0;
            }
            catch (NotSupportedException)
            {
                return 0;
            }
        }

        private static double? ReadTimeoutOf(ITransport transport)
        {
            try
            {
                var seconds = transport.ReadTimeout.TotalSeconds;
                return seconds >= 0 && !double.IsInfinity(seconds) ? seconds : (double?)null;
            }
            catch (NotSupportedException)
            {
                return null;
            }
        }

        private static TransportBridge Of(IntPtr user) => (TransportBridge)GCHandle.FromIntPtr(user).Target!;

        /// <summary>
        /// Store <paramref name="exception"/> for <see cref="Raise"/> (this thread) and <see cref="FailureSince"/> (the bridge
        /// of <paramref name="user"/>) and hand its text to the library.
        /// </summary>
        private static int Fail(Exception exception, IntPtr user)
        {
            _error = exception;
            try
            {
                var text = (IsTransportError(exception) ? "transport error: " + exception.Message : exception.GetType().Name + ": " + exception.Message).Replace('\0', ' ');
                var number = Interlocked.Increment(ref _failureCount);
                var recent = Of(user)._recent;
                lock (recent)
                {
                    recent.Add((number, text, exception));
                    if (recent.Count > RecentFailures)
                    {
                        recent.RemoveAt(0);
                    }
                }

                NativeMethods.iad_callback_error(Utf8.Encode(text));
            }
            catch (Exception)
            {
                // the text is a courtesy: the failure itself is the result code
            }

            return IsTransportError(exception) ? -1 : CallbackAbort;
        }

        private static int OnWrite(IntPtr user, IntPtr data, UIntPtr size)
        {
            try
            {
                var count = NativeText.ToInt(size);
                var frame = new byte[count];
                if (count > 0)
                {
                    Marshal.Copy(data, frame, 0, count);
                }

                Of(user).Transport.Write(frame, 0, count);
                return 0;
            }
            catch (Exception e)
            {
                return Fail(e, user);
            }
        }

        private static int OnRead(IntPtr user, IntPtr buffer, UIntPtr size, double timeoutS, out UIntPtr got)
        {
            got = UIntPtr.Zero;
            try
            {
                var transport = Of(user).Transport;
                var count = NativeText.ToInt(size);
                if (count == 0)
                {
                    return 0;
                }

                var data = new byte[count];
                var bridge = Of(user);
                var restore = bridge._restore.HasValue || !bridge.IsShortened(timeoutS) ? null : Shorten(transport, timeoutS);
                int n;
                try
                {
                    n = transport.Read(data, 0, count);
                }
                finally
                {
                    if (restore.HasValue)
                    {
                        TrySetReadTimeout(transport, restore.Value);
                    }
                }

                n = Math.Max(0, Math.Min(n, count));
                if (n > 0)
                {
                    Marshal.Copy(data, 0, buffer, n);
                }

                got = NativeText.Size(n);
                return 0;
            }
            catch (Exception e)
            {
                return Fail(e, user);
            }
        }

        /// <summary>
        /// The core asks for a wait shorter than the timeout it holds (given at open, or by <see cref="TimeoutToSync"/>): a
        /// probe, a capped read. Any other wait is the transport's own timeout as it is now: never put back to an older value.
        /// </summary>
        private bool IsShortened(double timeoutS)
        {
            var held = Volatile.Read(ref _coreTimeoutS);
            return !double.IsNaN(held) && timeoutS > 0 && timeoutS < held - 1e-9;
        }

        /// <summary>
        /// The core changes its read timeout (0.5): below the one it holds it shortens the transport's for the
        /// request about to go out (kept until the core puts its own back), anything else puts the transport's own back.
        /// </summary>
        private static int OnSetReadTimeout(IntPtr user, double seconds)
        {
            try
            {
                var bridge = Of(user);
                if (bridge.IsShortened(seconds))
                {
                    var saved = Shorten(bridge.Transport, seconds);
                    if (!bridge._restore.HasValue)
                    {
                        bridge._restore = saved;
                    }
                }
                else if (bridge._restore is TimeSpan saved)
                {
                    bridge._restore = null;
                    TrySetReadTimeout(bridge.Transport, saved);
                }

                return 0;
            }
            catch (Exception e)
            {
                return Fail(e, user);
            }
        }

        /// <summary>Lower the transport's read timeout to the core's shorter wait; the value to restore (null: unchanged).</summary>
        private static TimeSpan? Shorten(ITransport transport, double timeoutS)
        {
            if (!(timeoutS > 0) || double.IsInfinity(timeoutS))
            {
                return null;
            }

            TimeSpan current;
            try
            {
                current = transport.ReadTimeout;
            }
            catch (NotSupportedException)
            {
                return null;
            }

            var wanted = TimeSpan.FromSeconds(timeoutS);
            if (current <= wanted || !TrySetReadTimeout(transport, wanted))
            {
                return null;
            }

            return current;
        }

        private static bool TrySetReadTimeout(ITransport transport, TimeSpan value)
        {
            try
            {
                transport.ReadTimeout = value;
                return true;
            }
            catch (Exception e) when (e is NotSupportedException || e is ArgumentException || e is IOException || e is InvalidOperationException)
            {
                return false;
            }
        }

        private static int OnResetInput(IntPtr user)
        {
            try
            {
                Of(user).Transport.ResetInputBuffer();
                return 0;
            }
            catch (Exception e)
            {
                return Fail(e, user);
            }
        }

        private static int OnSetBaudRate(IntPtr user, int rate)
        {
            try
            {
                Of(user).Transport.BaudRate = rate;
                return 0;
            }
            catch (Exception e)
            {
                return Fail(e, user);
            }
        }

        private static int OnSendBreak(IntPtr user, double seconds)
        {
            try
            {
                Of(user).Transport.SendBreak(TimeSpan.FromSeconds(seconds));
                return 0;
            }
            catch (Exception e)
            {
                return Fail(e, user);
            }
        }

        private static void OnClose(IntPtr user)
        {
            try
            {
                var bridge = Of(user);
                if (bridge.ForwardClose)
                {
                    bridge.Transport.Close();
                }
            }
            catch (Exception e)
            {
                DriverLog.Debug("IaiActuatorDriver.Interop", string.Format(CultureInfo.InvariantCulture, "closing the transport raised: {0}", e.Message));
            }
        }

        private static void OnRelease(IntPtr user)
        {
            try
            {
                GCHandle.FromIntPtr(user).Free();
            }
            catch (Exception)
            {
                // released twice would be a library bug; never let it cross into the library
            }
        }
    }
}
