using System;
using System.Runtime.InteropServices;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The ABI 0.6 additions of iai_actuator_driver.h the C# library uses, in header order (conventions:
    /// NativeMethods.cs).
    /// </summary>
    internal static partial class NativeMethods
    {
        // ------------------------------------------------------------------------------------------ ABI 0.6 (csharp)

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_port_detect_baud_rate(BusHandle bus, byte[]? slaves, byte[]? candidates, double probeTimeoutS, out int rate);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_read_device_info_json(BusHandle bus, int slave, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ ABI 0.6 (fixes)

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_transport(BusHandle bus, TransportHandle transport);

        // ------------------------------------------------------------------------------------------ ABI 0.6 (py-recovery)

        /// <summary>What queued an event (iad_event.source).</summary>
        internal const int EventSourceMonitor = 1;
        internal const int EventSourceCheckLink = 2;
        internal const int EventSourceRecover = 3;
        internal const int EventSourceSetLinkState = 4;
        internal const int EventSourceCell = 5;

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_session_from_info(AxisHandle axis, byte[]? infoJson, byte[]? name);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_session_update(AxisHandle axis, byte[] fieldsJson);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_link_state(AxisHandle axis, byte[] state, byte[]? message, byte[]? name, out int changed);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_port_present(AxisHandle axis, out int present);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_restart_signs(AxisHandle axis, ushort[] words, UIntPtr wordCount, byte[]? optionsJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_link_monitor_json(CellHandle cell, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ ABI 0.6 (py-info-cli)

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_info_call_json(AxisHandle axis, byte[] function, byte[]? argsJson, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_self_test_items_json(AxisHandle axis, int first, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ ABI 0.6 (core-sim-gw)

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_port(SimHandle sim, out TransportHandle transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_line_port(SimLineHandle line, out TransportHandle transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_gateway_line_open(byte[]? kind, byte[]? optionsJson, IntPtr[]? units, UIntPtr unitCount, out SimLineHandle line);
    }
}
