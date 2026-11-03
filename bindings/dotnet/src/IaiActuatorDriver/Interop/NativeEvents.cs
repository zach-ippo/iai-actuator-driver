using System;
using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// (ABI 0.2) The binding-owned thread that drains a handle's event queue (iad_axis_next_event / iad_cell_next_event,
    /// a short timeout per call, never a callback from the library) and raises the C# events on itself. A handler that
    /// throws is ignored: the pump keeps running. It ends when stopped (after a last drain without waiting) or when the
    /// handle is closed (the library wakes a waiting call).
    /// </summary>
    internal sealed class NativeEventPump
    {
        /// <summary>One wait for the next event: the call's result and the event (kind 0 when none came).</summary>
        internal delegate int NextCall(double timeoutS, out IadEvent ev);

        /// <summary>Longest wait per call: how quickly a stop is seen (the library returns at once when an event comes).</summary>
        private const double SliceS = 0.1;

        private readonly NextCall _next;
        private readonly Action<NativeEvent> _dispatch;
        private readonly Func<IadEvent, NativeEvent> _decode;
        private readonly Thread _thread;
        private volatile bool _stopping;

        /// <param name="name">The thread's name.</param>
        /// <param name="next">One wait for the next event.</param>
        /// <param name="dispatch">Raises the C# events of one event.</param>
        /// <param name="decode">How a queued iad_event becomes a <see cref="NativeEvent"/> (a cell's names resolve a cut axis name); null = no names.</param>
        internal NativeEventPump(string name, NextCall next, Action<NativeEvent> dispatch, Func<IadEvent, NativeEvent>? decode = null)
        {
            _next = next;
            _dispatch = dispatch;
            _decode = decode ?? (ev => new NativeEvent(ev));
            _thread = new Thread(Run) { IsBackground = true, Name = name };
            _thread.Start();
        }

        /// <summary>The thread still runs.</summary>
        internal bool IsRunning => _thread.IsAlive;

        /// <summary>The caller is the pump thread (a handler): it must not wait for itself.</summary>
        internal bool IsPumpThread => Thread.CurrentThread == _thread;

        /// <summary>Raise <paramref name="handler"/>: each subscriber on its own, so one that throws does not keep the others from the event.</summary>
        internal static void Raise(EventHandler<NativeEvent>? handler, object sender, NativeEvent e)
        {
            if (handler is null)
            {
                return;
            }

            foreach (var one in handler.GetInvocationList())
            {
                try
                {
                    ((EventHandler<NativeEvent>)one)(sender, e);
                }
                catch (Exception)
                {
                    // A handler's exception never stops the pump or the other handlers.
                }
            }
        }

        /// <summary>Ask the pump to end after a last drain and wait for it (not when called from a handler).</summary>
        internal void Stop(TimeSpan wait)
        {
            _stopping = true;
            if (!IsPumpThread)
            {
                _thread.Join(wait);
            }
        }

        private void Run()
        {
            while (true)
            {
                int code;
                IadEvent ev;
                try
                {
                    code = _next(_stopping ? 0.0 : SliceS, out ev);
                }
                catch (Exception)
                {
                    return;  // the handle was disposed (ObjectDisposedException) or the library is gone
                }

                if (code != 0)
                {
                    return;  // the handle was closed (IAD_ERR_INVALID_ARGUMENT): nothing more will come
                }

                if (ev.Kind == 0)
                {
                    if (_stopping)
                    {
                        return;
                    }

                    continue;
                }

                var e = _decode(ev);
                try
                {
                    _dispatch(e);
                }
                catch (Exception)
                {
                    // A handler's exception never stops the pump.
                }
            }
        }
    }

    /// <summary>
    /// (ABI 0.2) The log and frame sinks. The library gets static thunks that live as long as the process, so a frame or
    /// log line in flight while a handler is removed never calls a collected delegate; the handlers themselves are looked
    /// up per call (a frame sink by the id passed as the sink's user pointer).
    /// </summary>
    internal static class NativeSinks
    {
        private static readonly NativeMethods.LogSink LogThunk = OnLog;
        private static readonly NativeMethods.FrameSink FrameThunk = OnFrame;
        private static readonly ConcurrentDictionary<long, Action<FrameDirection, byte[]>> Frames =
            new ConcurrentDictionary<long, Action<FrameDirection, byte[]>>();

        private static readonly object LogGate = new object();
        private static Action<DriverLogLevel, string, string>? _log;
        private static long _nextFrameId;
        private static bool _driverLog;
        private static bool _exitHooked;

        internal static void SetLog(Action<DriverLogLevel, string, string>? handler)
        {
            lock (LogGate)
            {
                Volatile.Write(ref _log, handler);
                InstallLocked();
            }
        }

        /// <summary>
        /// (ABI 0.3) Pass the core's log lines on to <see cref="DriverLog"/> from now on (level, the core's source such as
        /// "iai_actuator_driver.modbus", message), besides a handler of <see cref="NativeDriver.SetLogHandler"/>. Called
        /// when the first object of the managed API runs on the core; once on, it stays on. At process exit the sink is
        /// removed first, so no core thread calls into a runtime that is shutting down.
        /// </summary>
        internal static void ForwardToDriverLog()
        {
            if (Volatile.Read(ref _driverLog))
            {
                return;
            }

            lock (LogGate)
            {
                if (_driverLog)
                {
                    return;
                }

                Volatile.Write(ref _driverLog, true);
                InstallLocked();
            }
        }

        private static void InstallLocked()
        {
            var on = _driverLog || Volatile.Read(ref _log) != null;
            NativeErrors.Check(NativeMethods.iad_set_log_sink(on ? LogThunk : null, IntPtr.Zero));
            if (on && !_exitHooked)
            {
                _exitHooked = true;
                AppDomain.CurrentDomain.ProcessExit += (sender, e) =>
                {
                    try
                    {
                        NativeMethods.iad_set_log_sink(null, IntPtr.Zero);
                    }
                    catch (Exception)
                    {
                        // the library may be gone already
                    }
                };
            }
        }

        /// <summary>(ABI 0.3) Install <paramref name="handler"/> as the frame sink of <paramref name="bus"/>; returns its id (0 for none).</summary>
        internal static long SetBusFrame(BusHandle bus, Action<FrameDirection, byte[]>? handler)
        {
            if (handler is null)
            {
                NativeErrors.Check(NativeMethods.iad_bus_set_frame_sink(bus, null, IntPtr.Zero));
                return 0;
            }

            var id = Interlocked.Increment(ref _nextFrameId);
            Frames[id] = handler;
            var result = NativeMethods.iad_bus_set_frame_sink(bus, FrameThunk, new IntPtr(id));
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                Frames.TryRemove(id, out _);
                throw error;
            }

            return id;
        }

        /// <summary>Install <paramref name="handler"/> as the frame sink of <paramref name="axis"/>'s port; returns its id (0 for none).</summary>
        internal static long SetFrame(AxisHandle axis, Action<FrameDirection, byte[]>? handler)
        {
            if (handler is null)
            {
                NativeErrors.Check(NativeMethods.iad_axis_set_frame_sink(axis, null, IntPtr.Zero));
                return 0;
            }

            var id = Interlocked.Increment(ref _nextFrameId);
            Frames[id] = handler;
            var result = NativeMethods.iad_axis_set_frame_sink(axis, FrameThunk, new IntPtr(id));
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                Frames.TryRemove(id, out _);
                throw error;
            }

            return id;
        }

        /// <summary>Forget a frame handler (after the sink was replaced or removed): later frames for it are dropped.</summary>
        internal static void ForgetFrame(long id)
        {
            if (id != 0)
            {
                Frames.TryRemove(id, out _);
            }
        }

        private static void OnLog(int level, IntPtr source, IntPtr message, IntPtr user)
        {
            try
            {
                var handler = Volatile.Read(ref _log);
                var toDriverLog = Volatile.Read(ref _driverLog) && DriverLog.HasListeners;
                if (handler is null && !toDriverLog)
                {
                    return;  // nobody listens: the line is not decoded
                }

                var lvl = (DriverLogLevel)Math.Max(0, Math.Min(level, (int)DriverLogLevel.Error));
                var src = Utf8.FromPointer(source);
                var text = Utf8.FromPointer(message);
                if (toDriverLog)
                {
                    DriverLog.Write(lvl, src, NameMap.Managed(text));
                }

                handler?.Invoke(lvl, src, text);
            }
            catch (Exception)
            {
                // Logging must never break communication (and no exception may cross into the library).
            }
        }

        private static void OnFrame(int direction, IntPtr bytes, UIntPtr length, IntPtr user)
        {
            try
            {
                if (!Frames.TryGetValue(user.ToInt64(), out var handler))
                {
                    return;
                }

                var count = (int)Math.Min(length.ToUInt64(), int.MaxValue);
                var frame = new byte[count];
                if (count > 0 && bytes != IntPtr.Zero)
                {
                    Marshal.Copy(bytes, frame, 0, count);
                }

                handler((FrameDirection)direction, frame);
            }
            catch (Exception)
            {
                // As the log sink.
            }
        }
    }
}
