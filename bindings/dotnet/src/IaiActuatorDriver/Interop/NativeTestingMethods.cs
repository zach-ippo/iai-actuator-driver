using System;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// P/Invoke declarations of iai_actuator_driver_testing.h, in header order (conventions: NativeMethods.cs): the test
    /// hooks the same library exports without a stability promise (the library's clock, the simulator's hooks, the
    /// driver's time limits, the port list). Each function goes through a wrapper that first checks, once per process,
    /// that the library's testing version (iad_testing_abi_version) is exactly <see cref="TestingAbiVersion"/>; the
    /// stable ABI's own check (<see cref="NativeDriver.EnsureLoaded"/>) does not cover them.
    /// </summary>
    internal static class NativeTestingMethods
    {
        /// <summary>The version of iai_actuator_driver_testing.h this binding was written for (IAD_TESTING_ABI_VERSION).</summary>
        internal const uint TestingAbiVersion = 1;

        private const string Library = NativeMethods.Library;

        private const CallingConvention Winapi = CallingConvention.Winapi;  // IAD_CALL, as NativeMethods

        private static int _checked;

        /// <summary>
        /// Load the library (<see cref="NativeDriver.EnsureLoaded"/>) and check its testing version once: exactly
        /// <see cref="TestingAbiVersion"/>, or <see cref="NotSupportedException"/> (no compatibility between numbers).
        /// </summary>
        internal static void EnsureCompatible()
        {
            if (Volatile.Read(ref _checked) != 0)
            {
                return;
            }

            NativeDriver.EnsureLoaded();
            uint found;
            try
            {
                found = iad_testing_abi_version();
            }
            catch (EntryPointNotFoundException)
            {
                found = 0;
            }

            if (found != TestingAbiVersion)
            {
                throw new NotSupportedException(string.Format(
                    CultureInfo.InvariantCulture,
                    "{0} has the testing ABI {1}; this binding was written for {2} (iai_actuator_driver_testing.h, exact match): use the library and the binding of the same release",
                    NativeMethods.Library,
                    found,
                    TestingAbiVersion));
            }

            Volatile.Write(ref _checked, 1);
        }

        // ------------------------------------------------------------------------------------------ version

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern uint iad_testing_abi_version();

        // ------------------------------------------------------------------------------------------ callbacks

        /// <summary>IAD_SIM_PASS / DROP / REPLY: what a simulator's request hook answers.</summary>
        internal const int SimPass = 0;

        internal const int SimDrop = 1;

        internal const int SimReply = 2;

        /// <summary>IAD_SIM_FAIL: the request fails (the reply buffer holds the message).</summary>
        internal const int SimFail = 3;

        /// <summary>IAD_SIM_EXCEPTION(code): answer the Modbus exception <paramref name="code"/>.</summary>
        internal static int SimException(int code) => 0x100 | (code & 0xFF);

        /// <summary>iad_clock_now_fn: seconds on a monotonic clock.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate double ClockNowFn(IntPtr user);

        /// <summary>iad_clock_sleep_fn: sleep at least seconds (a virtual clock moves on by them).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate void ClockSleepFn(double seconds, IntPtr user);

        /// <summary>iad_sim_request_hook: IAD_SIM_PASS / DROP / REPLY / EXCEPTION(code) / FAIL.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int SimRequestHook(IntPtr request, UIntPtr size, IntPtr reply, UIntPtr replyCapacity, out UIntPtr replyLength, IntPtr user);

        // ------------------------------------------------------------------------------------------ the library's clock

        internal static int iad_clock_use_real()
        {
            EnsureCompatible();
            return ClockUseReal();
        }

        internal static int iad_clock_use_virtual(double startS)
        {
            EnsureCompatible();
            return ClockUseVirtual(startS);
        }

        internal static int iad_clock_use_callbacks(ClockNowFn? now, ClockSleepFn? sleep, IntPtr user)
        {
            EnsureCompatible();
            return ClockUseCallbacks(now, sleep, user);
        }

        internal static int iad_clock_advance(double seconds)
        {
            EnsureCompatible();
            return ClockAdvance(seconds);
        }

        internal static int iad_clock_now(out double now)
        {
            EnsureCompatible();
            return ClockNow(out now);
        }

        // ------------------------------------------------------------------------------------------ simulator hooks

        internal static int iad_sim_state_json(SimHandle sim, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required)
        {
            EnsureCompatible();
            return SimStateJson(sim, buffer, bufferSize, out required);
        }

        internal static int iad_sim_requests_json(SimHandle sim, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required)
        {
            EnsureCompatible();
            return SimRequestsJson(sim, buffer, bufferSize, out required);
        }

        internal static int iad_sim_events_json(SimHandle sim, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required)
        {
            EnsureCompatible();
            return SimEventsJson(sim, buffer, bufferSize, out required);
        }

        internal static int iad_sim_power_cycle(SimHandle sim, int loseHome)
        {
            EnsureCompatible();
            return SimPowerCycle(sim, loseHome);
        }

        internal static int iad_sim_power_off(SimHandle sim)
        {
            EnsureCompatible();
            return SimPowerOff(sim);
        }

        internal static int iad_sim_power_on(SimHandle sim, int loseHome)
        {
            EnsureCompatible();
            return SimPowerOn(sim, loseHome);
        }

        internal static int iad_sim_unplug(SimHandle sim)
        {
            EnsureCompatible();
            return SimUnplug(sim);
        }

        internal static int iad_sim_replug(SimHandle sim)
        {
            EnsureCompatible();
            return SimReplug(sim);
        }

        internal static int iad_sim_set_word_override(SimHandle sim, int address, int value)
        {
            EnsureCompatible();
            return SimSetWordOverride(sim, address, value);
        }

        internal static int iad_sim_set_unreadable(SimHandle sim, int start, int count, int unreadable)
        {
            EnsureCompatible();
            return SimSetUnreadable(sim, start, count, unreadable);
        }

        internal static int iad_sim_set_stale_status_polls(SimHandle sim, int polls)
        {
            EnsureCompatible();
            return SimSetStaleStatusPolls(sim, polls);
        }

        internal static int iad_sim_register_value(SimHandle sim, int address, out ushort value)
        {
            EnsureCompatible();
            return SimRegisterValue(sim, address, out value);
        }

        internal static int iad_sim_handle_frame(SimHandle sim, byte[] request, UIntPtr size, byte[]? buffer, UIntPtr bufferSize, out UIntPtr length)
        {
            EnsureCompatible();
            return SimHandleFrame(sim, request, size, buffer, bufferSize, out length);
        }

        /// <summary>The raw-pointer form for use inside a request hook (the request and reply are the library's buffers).</summary>
        internal static int iad_sim_handle_frame_raw(IntPtr sim, IntPtr request, UIntPtr size, IntPtr buffer, UIntPtr bufferSize, out UIntPtr length)
        {
            EnsureCompatible();
            return SimHandleFrameRaw(sim, request, size, buffer, bufferSize, out length);
        }

        internal static int iad_sim_set_request_hook(SimHandle sim, SimRequestHook? hook, IntPtr user)
        {
            EnsureCompatible();
            return SimSetRequestHook(sim, hook, user);
        }

        internal static int iad_sim_control_json(SimHandle sim, byte[] commandJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required)
        {
            EnsureCompatible();
            return SimControlJson(sim, commandJson, buffer, bufferSize, out required);
        }

        internal static int iad_sim_line_requests_json(SimLineHandle line, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required)
        {
            EnsureCompatible();
            return SimLineRequestsJson(line, buffer, bufferSize, out required);
        }

        internal static int iad_sim_line_control_json(SimLineHandle line, byte[] commandJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required)
        {
            EnsureCompatible();
            return SimLineControlJson(line, commandJson, buffer, bufferSize, out required);
        }

        internal static int iad_sim_set_clock(SimHandle sim, ClockNowFn? now, IntPtr user)
        {
            EnsureCompatible();
            return SimSetClock(sim, now, user);
        }

        // ------------------------------------------------------------------------------------------ driver hooks

        internal static int iad_testing_set_timing(byte[]? name, double seconds)
        {
            EnsureCompatible();
            return TestingSetTiming(name, seconds);
        }

        /// <summary>
        /// The non-volatile write guard: count one write of the axis's unit at <paramref name="nowS"/> (the library's
        /// clock, iad_clock_now; NaN: now) as a real write is counted, with its rate warning. Nothing is sent.
        /// </summary>
        internal static int iad_testing_axis_nv_record(AxisHandle axis, double nowS, byte[]? what, out int warned)
        {
            EnsureCompatible();
            return TestingAxisNvRecord(axis, nowS, what, out warned);
        }

        /// <summary>The guard's writes in the hour before <paramref name="nowS"/> (the library's clock; NaN: now).</summary>
        internal static int iad_testing_axis_nv_last_hour(AxisHandle axis, double nowS, out int count)
        {
            EnsureCompatible();
            return TestingAxisNvLastHour(axis, nowS, out count);
        }

        /// <summary>The serial ports every port listing of the core sees (a JSON list of {"port", "description",
        /// "manufacturer", "hardware_id", "vid", "pid"}); null restores the PC's enumeration.</summary>
        internal static int iad_testing_set_port_list(byte[]? portsJson)
        {
            EnsureCompatible();
            return TestingSetPortList(portsJson);
        }

        // ------------------------------------------------------------------------------------------ the imports

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_clock_use_real")]
        private static extern int ClockUseReal();

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_clock_use_virtual")]
        private static extern int ClockUseVirtual(double startS);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_clock_use_callbacks")]
        private static extern int ClockUseCallbacks(ClockNowFn? now, ClockSleepFn? sleep, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_clock_advance")]
        private static extern int ClockAdvance(double seconds);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_clock_now")]
        private static extern int ClockNow(out double now);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_state_json")]
        private static extern int SimStateJson(SimHandle sim, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_requests_json")]
        private static extern int SimRequestsJson(SimHandle sim, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_events_json")]
        private static extern int SimEventsJson(SimHandle sim, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_power_cycle")]
        private static extern int SimPowerCycle(SimHandle sim, int loseHome);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_power_off")]
        private static extern int SimPowerOff(SimHandle sim);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_power_on")]
        private static extern int SimPowerOn(SimHandle sim, int loseHome);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_unplug")]
        private static extern int SimUnplug(SimHandle sim);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_replug")]
        private static extern int SimReplug(SimHandle sim);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_set_word_override")]
        private static extern int SimSetWordOverride(SimHandle sim, int address, int value);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_set_unreadable")]
        private static extern int SimSetUnreadable(SimHandle sim, int start, int count, int unreadable);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_set_stale_status_polls")]
        private static extern int SimSetStaleStatusPolls(SimHandle sim, int polls);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_register_value")]
        private static extern int SimRegisterValue(SimHandle sim, int address, out ushort value);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_handle_frame")]
        private static extern int SimHandleFrame(SimHandle sim, byte[] request, UIntPtr size, [Out] byte[]? buffer, UIntPtr bufferSize, out UIntPtr length);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_handle_frame")]
        private static extern int SimHandleFrameRaw(IntPtr sim, IntPtr request, UIntPtr size, IntPtr buffer, UIntPtr bufferSize, out UIntPtr length);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_set_request_hook")]
        private static extern int SimSetRequestHook(SimHandle sim, SimRequestHook? hook, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_control_json")]
        private static extern int SimControlJson(SimHandle sim, byte[] commandJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_line_requests_json")]
        private static extern int SimLineRequestsJson(SimLineHandle line, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_line_control_json")]
        private static extern int SimLineControlJson(SimLineHandle line, byte[] commandJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_sim_set_clock")]
        private static extern int SimSetClock(SimHandle sim, ClockNowFn? now, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_testing_set_timing")]
        private static extern int TestingSetTiming(byte[]? name, double seconds);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_testing_axis_nv_record")]
        private static extern int TestingAxisNvRecord(AxisHandle axis, double nowS, byte[]? what, out int warned);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_testing_axis_nv_last_hour")]
        private static extern int TestingAxisNvLastHour(AxisHandle axis, double nowS, out int count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true, EntryPoint = "iad_testing_set_port_list")]
        private static extern int TestingSetPortList(byte[]? portsJson);
    }
}
