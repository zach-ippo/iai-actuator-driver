using System;
using System.Runtime.InteropServices;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The ABI 0.7 (py-fixes) additions of iai_actuator_driver.h (conventions: NativeMethods.cs).
    /// </summary>
    internal static partial class NativeMethods
    {
        // ------------------------------------------------------------------------------------------ ABI 0.7 (py-fixes)

        /// <summary>The result the last call on the calling thread kept on its handle (iad_axis_last_result_json /
        /// iad_cell_last_result_json of that call), whatever a call on another thread kept since.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_last_result_json(byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        /// <summary>One pass of the cell's link monitor on the calling thread (its states, passes and state events).</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_link_monitor_check_now(CellHandle cell, byte[]? axes, [In, Out] IadLinkCheck[] checks, UIntPtr capacity, out UIntPtr count);
    }
}
