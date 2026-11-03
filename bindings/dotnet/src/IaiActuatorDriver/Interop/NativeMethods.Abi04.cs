using System;
using System.Runtime.InteropServices;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The ABI 0.4 additions of iai_actuator_driver.h, in header order (conventions: NativeMethods.cs): the bus lock held
    /// across calls, preemptible reads, suspending the status monitors of a bus, the IAI 0x45 opt-ins as flags, IAI 0x48,
    /// broadcasts, the line of a bus and the response timing. A hold of the bus lock and a preemptible scope belong to
    /// the calling OS thread.
    /// </summary>
    internal static partial class NativeMethods
    {
        /// <summary>IAD_IAI_ALLOW_UNSAFE: a command not on the safe list, another ID group, 0x45/0x05 with op1 != 0.</summary>
        internal const uint IaiAllowUnsafe = 0x1;

        /// <summary>IAD_IAI_ALLOW_NEVER_SEND: a command on the never-send list (only on the user's explicit request).</summary>
        internal const uint IaiAllowNeverSend = 0x2;

        /// <summary>IAD_IAI_NO_CHECK: a rejection is returned, not raised.</summary>
        internal const uint IaiNoCheck = 0x4;

        /// <summary>iad_read_gate_fn: 0 = send the attempt; any other value ends the read (IAD_ERR_CANCELLED).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int ReadGateFn(IntPtr user);

        // ------------------------------------------------------------------------------------------ 0.4: bus lock

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_lock(BusHandle bus, double timeoutS, out int locked);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_unlock(BusHandle bus);

        // ------------------------------------------------------------------------------------------ 0.4: preemptible reads

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_preemptible_begin(BusHandle bus, int words, ReadGateFn? gate, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_preemptible_end(BusHandle bus);

        // ------------------------------------------------------------------------------------------ 0.4: status monitors

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_suspend_monitors(BusHandle bus, double timeoutS, out int monitors);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_resume_monitors(BusHandle bus);

        // ------------------------------------------------------------------------------------------ 0.4: broadcasts

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_broadcast_iai_command(BusHandle bus, int group, int command, int op1, int op2, uint flags, double settleS, [Out] byte[]? frame, UIntPtr frameSize, out UIntPtr frameLength);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_broadcast(BusHandle bus, byte[] pdu, UIntPtr size, double settleS, [Out] byte[]? frame, UIntPtr frameSize, out UIntPtr frameLength);

        // ------------------------------------------------------------------------------------------ 0.4: the line

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_line(BusHandle bus, int line, double positionUnitMm);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_detect_line(BusHandle bus, int line, double timeoutS, double positionUnitMm, out int found);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_line_json(BusHandle bus, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ 0.4: response timing

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_response_delay(BusHandle bus, int slave, double delayMs);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_response_delay(BusHandle bus, int slave, out double delayMs);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_response_timeout(BusHandle bus, int slave, byte[] pdu, UIntPtr size, int expectedLength, out double seconds);

        // ---- csharp-1: the per-call options and the stateless functions (bound in the 0.5 round) ----

        // ------------------------------------------------------------------------------------------ 0.4: stateless functions

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_call_json(byte[] function, byte[]? argsJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ---- end csharp-1 ----
    }
}
