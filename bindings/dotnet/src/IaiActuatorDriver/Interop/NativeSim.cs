using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>What a simulator request hook does with one frame (<see cref="NativeSim.SetRequestHook"/>).</summary>
    internal enum NativeSimAction
    {
        /// <summary>The simulated unit answers as usual (IAD_SIM_PASS).</summary>
        Pass = 0,

        /// <summary>No answer; the frame does not reach the unit (IAD_SIM_DROP).</summary>
        Drop = 1,

        /// <summary>Answer with the given bytes; the frame does not reach the unit (IAD_SIM_REPLY).</summary>
        Reply = 2,

        /// <summary>Answer with a Modbus exception code; the frame does not reach the unit (IAD_SIM_EXCEPTION).</summary>
        Exception = 3,

        /// <summary>(0.6) The request fails: the write raises the transport error with the message (IAD_SIM_FAIL).</summary>
        Fail = 4,
    }

    /// <summary>A request hook's answer: the action, the reply bytes (Reply) or the exception code (Exception).</summary>
    internal readonly struct NativeSimAnswer
    {
        internal NativeSimAnswer(NativeSimAction action, byte[]? reply = null, int exceptionCode = 0)
        {
            Action = action;
            Reply = reply;
            ExceptionCode = exceptionCode;
        }

        internal NativeSimAction Action { get; }

        internal byte[]? Reply { get; }

        internal int ExceptionCode { get; }

        internal static NativeSimAnswer Pass => new NativeSimAnswer(NativeSimAction.Pass);

        internal static NativeSimAnswer Drop => new NativeSimAnswer(NativeSimAction.Drop);

        /// <summary>(0.6) The write fails with <paramref name="message"/> (UTF-8 in <see cref="Reply"/>).</summary>
        internal static NativeSimAnswer Fail(string message) => new NativeSimAnswer(NativeSimAction.Fail, Utf8.Encode(message ?? string.Empty));
    }

    /// <summary>
    /// (ABI 0.3) A simulated unit of the native core with the test hooks of the Python edition's SimulatedEC (iad_sim; C++
    /// HookedSimUnit). Options are one JSON object (iai_actuator_driver.h, "simulators"). Use it through
    /// <see cref="Transport"/> (a bus, an axis) or on a <see cref="NativeSimLine"/>. Evidence: test tooling.
    /// </summary>
    internal sealed class NativeSim : IDisposable
    {
        private static readonly NativeTestingMethods.SimRequestHook HookThunk = OnRequest;
        private static readonly ConcurrentDictionary<long, Func<byte[], NativeSimAnswer>> Hooks =
            new ConcurrentDictionary<long, Func<byte[], NativeSimAnswer>>();

        private static readonly NativeTestingMethods.ClockNowFn ClockThunk = OnClock;
        private static readonly ConcurrentDictionary<long, Func<double>> Clocks = new ConcurrentDictionary<long, Func<double>>();

        private static long _nextHookId;

        private readonly SimHandle _handle;
        private long _hook;
        private long _clock;

        private NativeSim(SimHandle handle)
        {
            _handle = handle;
        }

        /// <summary>The native handle.</summary>
        internal SimHandle Handle => _handle;

        /// <summary>A simulated unit (iad_sim_open; <paramref name="optionsJson"/> null or "" = the defaults: a slider at slave 1).</summary>
        internal static NativeSim Open(string? optionsJson = null)
        {
            NativeDriver.EnsureLoaded();
            var result = NativeMethods.iad_sim_open(Utf8.Encode(optionsJson, nameof(optionsJson)), out var handle);
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return new NativeSim(handle);
        }

        /// <summary>Release the handle (the unit lives on while a transport, bus, axis or line uses it); the hook is removed first.</summary>
        public void Dispose()
        {
            if (_handle.IsClosed)
            {
                return;
            }

            if (_hook != 0)
            {
                try
                {
                    NativeTestingMethods.iad_sim_set_request_hook(_handle, null, IntPtr.Zero);
                }
                finally
                {
                    Hooks.TryRemove(_hook, out _);
                    _hook = 0;
                }
            }

            if (_clock != 0)
            {
                try
                {
                    NativeTestingMethods.iad_sim_set_clock(_handle, null, IntPtr.Zero);
                }
                finally
                {
                    Clocks.TryRemove(_clock, out _);
                    _clock = 0;
                }
            }

            _handle.Dispose();
        }

        /// <summary>A transport handle on the unit (the simulated port; dispose it when done).</summary>
        internal TransportHandle Transport() => TransportOf(NativeMethods.iad_sim_transport(_handle, out var t), t);

        /// <summary>
        /// (0.6) The simulated port for a bus (iad_sim_port): as <see cref="Transport"/>, and it holds the 1 ms timer
        /// resolution while open, as a serial port does. Dispose it once the bus has it.
        /// </summary>
        internal TransportHandle Port() => TransportOf(NativeMethods.iad_sim_port(_handle, out var t), t);

        /// <summary>The unit's state as JSON (no I/O).</summary>
        internal string StateJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeTestingMethods.iad_sim_state_json(_handle, b, s, out r));

        /// <summary>Every frame written to the unit, oldest first: [{"t", "frame"}].</summary>
        internal string RequestsJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeTestingMethods.iad_sim_requests_json(_handle, b, s, out r));

        /// <summary>What the unit did, oldest first (a JSON array of strings).</summary>
        internal string EventsJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeTestingMethods.iad_sim_events_json(_handle, b, s, out r));

        /// <summary>A 24 V power cycle; <paramref name="loseHome"/> null = lost on an incremental unit only.</summary>
        internal void PowerCycle(bool? loseHome = null) => NativeErrors.Check(NativeTestingMethods.iad_sim_power_cycle(_handle, Known.Tri(loseHome)));

        internal void PowerOff() => NativeErrors.Check(NativeTestingMethods.iad_sim_power_off(_handle));

        internal void PowerOn(bool? loseHome = null) => NativeErrors.Check(NativeTestingMethods.iad_sim_power_on(_handle, Known.Tri(loseHome)));

        /// <summary>The USB adapter pulled: the port vanishes until <see cref="Replug"/> and a reopen.</summary>
        internal void Unplug() => NativeErrors.Check(NativeTestingMethods.iad_sim_unplug(_handle));

        internal void Replug() => NativeErrors.Check(NativeTestingMethods.iad_sim_replug(_handle));

        /// <summary>Serve <paramref name="value"/> for register <paramref name="address"/> (null: the modelled value again).</summary>
        internal void SetWordOverride(int address, int? value) =>
            NativeErrors.Check(NativeTestingMethods.iad_sim_set_word_override(_handle, address, value ?? NativeMethods.IntUnknown));

        internal void SetUnreadable(int start, int count, bool unreadable = true) =>
            NativeErrors.Check(NativeTestingMethods.iad_sim_set_unreadable(_handle, start, count, unreadable ? 1 : 0));

        internal void SetStaleStatusPolls(int polls) => NativeErrors.Check(NativeTestingMethods.iad_sim_set_stale_status_polls(_handle, polls));

        /// <summary>The value a one-register read returns now, the hooks applied.</summary>
        internal ushort RegisterValue(int address)
        {
            NativeErrors.Check(NativeTestingMethods.iad_sim_register_value(_handle, address, out var value));
            return value;
        }

        /// <summary>The unit's own answer to one request frame (it acts on it; empty = no answer).</summary>
        internal byte[] HandleFrame(byte[] request)
        {
            if (request is null)
            {
                throw new ArgumentNullException(nameof(request));
            }

            var buffer = new byte[512];
            NativeErrors.Check(NativeTestingMethods.iad_sim_handle_frame(_handle, request, NativeText.Size(request.Length), buffer, NativeText.Size(buffer.Length), out var length));
            var reply = new byte[NativeText.ToInt(length)];
            Array.Copy(buffer, reply, reply.Length);
            return reply;
        }

        /// <summary>
        /// Install (not null) or remove (null) the request hook: it sees every frame written to the unit and returns what
        /// to do. It runs on the thread doing the I/O with the bus lock held; it may call <see cref="HandleFrame"/> on this
        /// unit and nothing else of the library. An exception it throws counts as <see cref="NativeSimAction.Pass"/>.
        /// </summary>
        internal void SetRequestHook(Func<byte[], NativeSimAnswer>? hook)
        {
            var old = _hook;
            if (hook is null)
            {
                NativeErrors.Check(NativeTestingMethods.iad_sim_set_request_hook(_handle, null, IntPtr.Zero));
                _hook = 0;
            }
            else
            {
                var id = Interlocked.Increment(ref _nextHookId);
                Hooks[id] = hook;
                var result = NativeTestingMethods.iad_sim_set_request_hook(_handle, HookThunk, new IntPtr(id));
                if (result != 0)
                {
                    var error = NativeErrors.Capture(result);
                    Hooks.TryRemove(id, out _);
                    throw error;
                }

                _hook = id;
            }

            if (old != 0)
            {
                Hooks.TryRemove(old, out _);
            }
        }

        /// <summary>
        /// (0.5) Run the unit on <paramref name="now"/> (monotonic seconds; null: the library's clock again) from now on
        /// (iad_sim_set_clock): its time goes on from where it is. <paramref name="now"/> is called with the unit's lock
        /// held, from any thread using the unit; it must not call the library.
        /// </summary>
        internal void SetClock(Func<double>? now)
        {
            var old = _clock;
            if (now is null)
            {
                NativeErrors.Check(NativeTestingMethods.iad_sim_set_clock(_handle, null, IntPtr.Zero));
                _clock = 0;
            }
            else
            {
                var id = Interlocked.Increment(ref _nextHookId);
                Clocks[id] = now;
                var result = NativeTestingMethods.iad_sim_set_clock(_handle, ClockThunk, new IntPtr(id));
                if (result != 0)
                {
                    var error = NativeErrors.Capture(result);
                    Clocks.TryRemove(id, out _);
                    throw error;
                }

                _clock = id;
            }

            if (old != 0)
            {
                Clocks.TryRemove(old, out _);
            }
        }

        /// <summary>One control command {"command": NAME, ...}; its result as JSON ({} when none).</summary>
        internal string Control(string commandJson)
        {
            var command = Utf8.EncodeRequired(commandJson, nameof(commandJson));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeTestingMethods.iad_sim_control_json(_handle, command, b, s, out r));
        }

        internal static TransportHandle TransportOf(int result, TransportHandle handle)
        {
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return handle;
        }

        /// <summary>The clock trampoline: never lets an exception cross into the core (the library's clock then).</summary>
        private static double OnClock(IntPtr user)
        {
            try
            {
                if (Clocks.TryGetValue(user.ToInt64(), out var now))
                {
                    return now();
                }
            }
            catch (Exception exc)
            {
                DriverLog.Error("IaiActuatorDriver.SimulatedUnit", "the simulated unit's clock raised; the library clock is used", exc);
            }

            return Timing.Now;
        }

        private static int OnRequest(IntPtr request, UIntPtr size, IntPtr reply, UIntPtr replyCapacity, out UIntPtr replyLength, IntPtr user)
        {
            replyLength = UIntPtr.Zero;
            try
            {
                if (!Hooks.TryGetValue(user.ToInt64(), out var hook))
                {
                    return NativeTestingMethods.SimPass;
                }

                var frame = new byte[NativeText.ToInt(size)];
                if (frame.Length > 0)
                {
                    Marshal.Copy(request, frame, 0, frame.Length);
                }

                var answer = hook(frame);
                switch (answer.Action)
                {
                    case NativeSimAction.Drop:
                        return NativeTestingMethods.SimDrop;
                    case NativeSimAction.Exception:
                        return NativeTestingMethods.SimException(answer.ExceptionCode);
                    case NativeSimAction.Fail:
                    case NativeSimAction.Reply:
                        var bytes = answer.Reply ?? Array.Empty<byte>();
                        var n = Math.Min(bytes.Length, NativeText.ToInt(replyCapacity));
                        if (n > 0)
                        {
                            Marshal.Copy(bytes, 0, reply, n);
                        }

                        replyLength = NativeText.Size(n);
                        return answer.Action == NativeSimAction.Fail ? NativeTestingMethods.SimFail : NativeTestingMethods.SimReply;
                    default:
                        return NativeTestingMethods.SimPass;
                }
            }
            catch (Exception)
            {
                return NativeTestingMethods.SimPass;  // no exception may cross into the library
            }
        }
    }

    /// <summary>(ABI 0.3) Several simulated units on one line by slave address (iad_sim_line; C++ SimLine).</summary>
    internal sealed class NativeSimLine : IDisposable
    {
        private readonly SimLineHandle _handle;

        private NativeSimLine(SimLineHandle handle)
        {
            _handle = handle;
        }

        /// <summary>The native handle.</summary>
        internal SimLineHandle Handle => _handle;

        /// <summary>A line <paramref name="name"/> (null = "SIM") with <paramref name="units"/> on it (two units cannot share a slave).</summary>
        internal static NativeSimLine Open(string? name, IEnumerable<NativeSim>? units = null) =>
            Open(units, (IntPtr[] tokens, out SimLineHandle handle) =>
                NativeMethods.iad_sim_line_open(Utf8.Encode(name, nameof(name)), tokens, NativeText.Size(tokens.Length), out handle));

        /// <summary>
        /// (0.6) A line behind a gateway (iad_sim_gateway_line_open; C++ SimGatewayLine): <paramref name="kind"/> "rcon"
        /// (null), "robonet" or "robonet-gateway"; <paramref name="optionsJson"/> as the header lists ("name", "manu",
        /// "gateway_alarm", "emergency_stop", "linked_only", "position_unit_mm", "serials", "silent_no_access"). A frame the
        /// gateway passes on reaches the unit through its own port (its request log and hooks apply). DOC, UNVERIFIED.
        /// </summary>
        internal static NativeSimLine OpenGateway(string? kind, string? optionsJson = null, IEnumerable<NativeSim>? units = null) =>
            Open(units, (IntPtr[] tokens, out SimLineHandle handle) =>
                NativeMethods.iad_sim_gateway_line_open(Utf8.Encode(kind, nameof(kind)), Utf8.Encode(optionsJson, nameof(optionsJson)), tokens, NativeText.Size(tokens.Length), out handle));

        private delegate int LineOpen(IntPtr[] tokens, out SimLineHandle handle);

        private static NativeSimLine Open(IEnumerable<NativeSim>? units, LineOpen open)
        {
            NativeDriver.EnsureLoaded();
            var list = new List<NativeSim>(units ?? Array.Empty<NativeSim>());
            var tokens = new IntPtr[list.Count];
            var added = new bool[list.Count];
            try
            {
                for (var i = 0; i < list.Count; i++)
                {
                    list[i].Handle.DangerousAddRef(ref added[i]);
                    tokens[i] = list[i].Handle.DangerousGetHandle();
                }

                var result = open(tokens, out var handle);
                if (result != 0)
                {
                    var error = NativeErrors.Capture(result);
                    handle.Dispose();
                    throw error;
                }

                return new NativeSimLine(handle);
            }
            finally
            {
                for (var i = 0; i < list.Count; i++)
                {
                    if (added[i])
                    {
                        list[i].Handle.DangerousRelease();
                    }
                }
            }
        }

        /// <summary>Release the handle (the line lives on while a transport, bus, axis or cell uses it).</summary>
        public void Dispose() => _handle.Dispose();

        internal void Add(NativeSim sim) => NativeErrors.Check(NativeMethods.iad_sim_line_add(_handle, sim.Handle));

        /// <summary>Remove the unit at <paramref name="slave"/> (<see cref="ArgumentOutOfRangeException"/> when absent).</summary>
        internal void Remove(int slave) => NativeErrors.Check(NativeMethods.iad_sim_line_remove(_handle, slave));

        /// <summary>A transport handle on the line (dispose it when done).</summary>
        internal TransportHandle Transport() => NativeSim.TransportOf(NativeMethods.iad_sim_line_transport(_handle, out var t), t);

        /// <summary>(0.6) The line's port for a bus (iad_sim_line_port): as <see cref="Transport"/>, holding the 1 ms timer resolution while open.</summary>
        internal TransportHandle Port() => NativeSim.TransportOf(NativeMethods.iad_sim_line_port(_handle, out var t), t);

        /// <summary>Every request on the line, oldest first: [{"t", "slave", "frame"}].</summary>
        internal string RequestsJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeTestingMethods.iad_sim_line_requests_json(_handle, b, s, out r));

        /// <summary>
        /// One control command ("unplug", "replug", "power_cycle", "clear_requests", "state"; on a gateway line also
        /// "gateway", "set_gateway", "gateway_handle").
        /// </summary>
        internal string Control(string commandJson)
        {
            var command = Utf8.EncodeRequired(commandJson, nameof(commandJson));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeTestingMethods.iad_sim_line_control_json(_handle, command, b, s, out r));
        }

        /// <summary>
        /// (0.6) A gateway line's state as JSON: {"kind", "silent_no_access", "function_codes", "dropped_broadcasts",
        /// "rejected", "gateway": null or the station's own state}, frames as hex text. <see cref="ArgumentException"/> on a
        /// plain line.
        /// </summary>
        internal string GatewayJson() => Control("{\"command\": \"gateway\"}");

        /// <summary>
        /// (0.6) Change the gateway station: <paramref name="fieldsJson"/> one JSON object of "manu", "alarm_code",
        /// "emergency_stop", "slave", "linked_only", "running", "silent_no_access".
        /// </summary>
        internal void SetGateway(string fieldsJson)
        {
            var fields = (fieldsJson ?? throw new ArgumentNullException(nameof(fieldsJson))).Trim();
            if (!fields.StartsWith("{", StringComparison.Ordinal) || !fields.EndsWith("}", StringComparison.Ordinal))
            {
                throw new ArgumentException("the gateway fields must be one JSON object", nameof(fieldsJson));
            }

            var inner = fields.Substring(1, fields.Length - 2).Trim();
            Control("{\"command\": \"set_gateway\"" + (inner.Length == 0 ? string.Empty : ", " + inner) + "}");
        }

        /// <summary>(0.6) The gateway station's own answer to one frame (null: none); the frame reaches no unit.</summary>
        internal byte[]? GatewayHandle(byte[] frame)
        {
            if (frame is null)
            {
                throw new ArgumentNullException(nameof(frame));
            }

            var hex = BitConverter.ToString(frame).Replace('-', ' ');
            var reply = (MiniJson.Parse(Control("{\"command\": \"gateway_handle\", \"frame\": \"" + hex + "\"}")) as JsonObject)?.String("reply");
            if (reply is null)
            {
                return null;
            }

            var parts = reply.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            var bytes = new byte[parts.Length];
            for (var i = 0; i < parts.Length; i++)
            {
                bytes[i] = Convert.ToByte(parts[i], 16);
            }

            return bytes;
        }
    }

    /// <summary>
    /// (ABI 0.3) The core's clock seam (process-wide, for tests): the real clock, a virtual clock that only moves when
    /// slept on or advanced, or the binding's own clock through callbacks. Waits on other threads keep real-time slices: a
    /// virtual clock is for single-threaded tests.
    /// </summary>
    internal static class NativeClock
    {
        private static readonly NativeTestingMethods.ClockNowFn NowThunk = OnNow;
        private static readonly NativeTestingMethods.ClockSleepFn SleepThunk = OnSleep;
        private static readonly object Gate = new object();
        private static Func<double>? _now;
        private static Action<double>? _sleep;
        private static double _last;

        /// <summary>The real (steady) clock again.</summary>
        internal static void UseReal()
        {
            lock (Gate)
            {
                NativeDriver.EnsureLoaded();
                NativeErrors.Check(NativeTestingMethods.iad_clock_use_real());
                _now = null;
                _sleep = null;
            }
        }

        /// <summary>A virtual clock from <paramref name="startS"/> (null: 1000.0) that moves only when slept on or advanced.</summary>
        internal static void UseVirtual(double? startS = null)
        {
            lock (Gate)
            {
                NativeDriver.EnsureLoaded();
                NativeErrors.Check(NativeTestingMethods.iad_clock_use_virtual(startS ?? double.NaN));
                _now = null;
                _sleep = null;
            }
        }

        /// <summary>The binding's own clock: <paramref name="now"/> (seconds, monotonic) and <paramref name="sleep"/>.</summary>
        internal static void UseCallbacks(Func<double> now, Action<double> sleep)
        {
            lock (Gate)
            {
                NativeDriver.EnsureLoaded();
                _now = now ?? throw new ArgumentNullException(nameof(now));
                _sleep = sleep ?? throw new ArgumentNullException(nameof(sleep));
                NativeErrors.Check(NativeTestingMethods.iad_clock_use_callbacks(NowThunk, SleepThunk, IntPtr.Zero));
            }
        }

        /// <summary>Move the virtual clock forward (<see cref="EcNotReadyException"/> when it is not installed).</summary>
        internal static void Advance(double seconds) => NativeErrors.Check(NativeTestingMethods.iad_clock_advance(seconds));

        /// <summary>The library's clock now (seconds, arbitrary origin).</summary>
        internal static double Now()
        {
            NativeErrors.Check(NativeTestingMethods.iad_clock_now(out var now));
            return now;
        }

        private static double OnNow(IntPtr user)
        {
            try
            {
                var now = Volatile.Read(ref _now);
                if (now != null)
                {
                    _last = now();
                }
            }
            catch (Exception)
            {
                // keep the last reading: no exception may cross into the library
            }

            return _last;
        }

        private static void OnSleep(double seconds, IntPtr user)
        {
            try
            {
                Volatile.Read(ref _sleep)?.Invoke(seconds);
            }
            catch (Exception)
            {
                // as OnNow
            }
        }
    }
}
