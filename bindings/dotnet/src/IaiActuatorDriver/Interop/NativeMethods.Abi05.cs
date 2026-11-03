using System;
using System.Runtime.InteropServices;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The ABI 0.5 additions of iai_actuator_driver.h the C# library uses, in header order (conventions:
    /// NativeMethods.cs): what the axis keeps without I/O, the reads of their own, a row write without the checksum,
    /// begin_stop and the non-volatile bookkeeping per unit (py-axis block; the simulator's clock and the test timings
    /// are in NativeTestingMethods); a bus shut down or released apart from closing it and the transport's set_read_timeout (core-bus),
    /// monitor suspension and the port owner (py-monitor), the line and gateway objects (py-gateway), ports and cells
    /// over the binding's buses and axes (py-cell).
    /// </summary>
    internal static partial class NativeMethods
    {
        // ---- csharp-2: the 0.5 simulator clock and axis state ----

        // ------------------------------------------------------------------------------------------ 0.5 (py-axis): axis state

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_last_status_words(AxisHandle axis, ref IadStatus like, [Out] ushort[] words, UIntPtr capacity, out UIntPtr count);

        /// <summary>iad_axis_last_status_words with <c>like</c> NULL: the newest sample.</summary>
        [DllImport(Library, EntryPoint = "iad_axis_last_status_words", CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_last_status_words_newest(AxisHandle axis, IntPtr like, [Out] ushort[] words, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_state_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_kind_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_soft_limits_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_axis_limits_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_speed_limit(AxisHandle axis, out double speed);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_support_words_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_categories_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_parameter_model_json(AxisHandle axis, int visibility, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_user_parameters_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_write_position_row(AxisHandle axis, int row, ref IadPositionEntry entry, ref IadNvWriteOptions options, int writeChecksum, out IadPositionWriteResult result);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_enable_modbus_control(AxisHandle axis, int verify, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_begin_stop(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_push_speed_json(AxisHandle axis, double speed, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_servo_on_timeout_json(AxisHandle axis, ushort[]? words, UIntPtr count, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_release_held(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_axis_limits_json(AxisHandle axis, byte[] limitsJson);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_nv_unit(AxisHandle axis, byte[]? unitKey, byte[]? label);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_nv_forget(byte[]? unitKeyPrefix);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_constants_json(byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ---- end csharp-2 ----

        // ---- csharp-s4: the rest of 0.5 (core-bus, py-monitor, py-gateway, py-cell) ----

        /// <summary>iad_transport_set_read_timeout_fn (0.5): the bus changes the read timeout (a probe's shorter wait and back).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportSetReadTimeoutFn(IntPtr user, double seconds);

        /// <summary>iad_port_line_fn: the port asks its owner to work out the line (IAD_OK, or the line stays undetected).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int PortLineFn(IntPtr user);

        /// <summary>iad_transport_callbacks_init with the 0.5 struct (struct_size set to its size first).</summary>
        [DllImport(Library, EntryPoint = "iad_transport_callbacks_init", CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_callbacks_init_05(ref IadTransportCallbacks05 callbacks);

        /// <summary>iad_transport_open_callbacks with the 0.5 struct.</summary>
        [DllImport(Library, EntryPoint = "iad_transport_open_callbacks", CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_open_callbacks_05(ref IadTransportCallbacks05 callbacks, byte[]? name, IntPtr user, out TransportHandle transport);

        // ------------------------------------------------------------------------------------------ 0.5 (core-bus): buses

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_shutdown(BusHandle bus);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_release(IntPtr bus);

        // ------------------------------------------------------------------------------------------ 0.5 (py-monitor): status monitor

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_suspend_monitor(AxisHandle axis, double timeoutS);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_resume_monitor(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_monitor_target(AxisHandle axis, out double targetMm, out double bandMm);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_port_owner(AxisHandle axis, int owner);

        // ------------------------------------------------------------------------------------------ 0.5 (py-gateway): lines

        /// <summary><c>gateway</c>: a gateway token (<see cref="SafeHandle.DangerousGetHandle"/>) or zero for none.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_open(int kind, IntPtr gateway, int detected, double positionUnitMm, out LineHandle line);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_close(IntPtr line);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_line(BusHandle bus, out LineHandle line);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_attach_line(BusHandle bus, LineHandle line);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_state_line_rules(BusHandle bus, byte[]? rulesJson);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_json(LineHandle line, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_allows(LineHandle line, byte[] operation, out int allowed);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_require(LineHandle line, byte[] operation, byte[]? what);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_check_request(LineHandle line, int slave, byte[] pdu, UIntPtr size);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_require_control(LineHandle line, int slave);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_position_unit(LineHandle line, int slave, out double unitMm, byte[]? source, UIntPtr sourceSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_set_position_unit(LineHandle line, double unitMm, int slave, byte[]? source);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_require_position_unit(LineHandle line, int slave, int control, out double unitMm);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_note_unreachable(LineHandle line, byte[] slaves);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_explain_unreachable(LineHandle line, byte[]? slaves, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        /// <summary>The handle is invalid (zero) when the line has no gateway.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_gateway(LineHandle line, out GatewayHandle gateway);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_line_rules_json(byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_open(BusHandle bus, int kind, int address, ushort[]? words, UIntPtr count, double readAt, double cacheS, out GatewayHandle gateway);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_close(IntPtr gateway);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_info_json(GatewayHandle gateway, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_status(GatewayHandle gateway, double maxAgeS, [Out] ushort[] words, UIntPtr capacity, out UIntPtr count, out double readAt);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_require_manu(GatewayHandle gateway, [Out] ushort[] words, UIntPtr capacity, out UIntPtr count, out double readAt);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_read_alarm(GatewayHandle gateway, out uint code);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_set_cache_s(GatewayHandle gateway, double cacheS);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_gateway_read_monitor_json(GatewayHandle gateway, byte[]? axes, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_probe_gateway(BusHandle bus, int line, int address, double timeoutS, [Out] ushort[] words, UIntPtr capacity, out UIntPtr count, out double readAt);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_read_serial_code(BusHandle bus, int slave, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_robonet_hint(BusHandle bus, int slave, int directoryTimedOut, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ 0.5 (py-cell): ports and cells

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_port(BusHandle bus, byte[]? name, int line, double positionUnitMm, int fixedRate, PortLineFn? detectLine, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_port_info_json(BusHandle bus, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_ensure_baud_rate(BusHandle bus, byte[]? slaves, out int rate, out int detected);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_port_detect_line(BusHandle bus, int force, out int kind);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_scan(BusHandle bus, byte[]? slaves, double timeoutS, int useLinkMap, [Out] int[]? found, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_scan_units_json(BusHandle bus, byte[]? slaves, double timeoutS, int identify, int useLinkMap, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_identify_json(BusHandle bus, int slave, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_baud_rate_all(BusHandle bus, int rate, byte[]? slaves, ref IadBaudOptions options, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        /// <summary><c>buses</c>: the bus tokens (<see cref="SafeHandle.DangerousGetHandle"/>, each add-ref'd for the call).</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_open_buses(IntPtr[] buses, UIntPtr busCount, byte[]? slavesJson, byte[]? scanSlaves, ref IadCellOptions options, ref IadAxisOptions axisOptions, out CellHandle cell);

        /// <summary><c>axes</c>: the axis tokens (each add-ref'd for the call).</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_open_axes(IntPtr[] axes, UIntPtr axisCount, byte[]? names, ref IadCellOptions options, out CellHandle cell);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_axis_ref(CellHandle cell, byte[] name, out AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_release(IntPtr cell);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_port_errors_json(CellHandle cell, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_read_all_json(CellHandle cell, byte[] what, byte[]? axes, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ---- end csharp-s4 ----
    }

    /// <summary>
    /// iad_transport_callbacks as of 0.5: the 0.3 struct with set_read_timeout appended (104 bytes on 64-bit). The core
    /// takes the whole struct only when struct_size says so: set <see cref="IadTransportCallbacks.StructSize"/> to
    /// <c>Marshal.SizeOf&lt;IadTransportCallbacks05&gt;()</c> before iad_transport_callbacks_init.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct IadTransportCallbacks05
    {
        public IadTransportCallbacks Callbacks;
        public IntPtr SetReadTimeout;
    }
}
