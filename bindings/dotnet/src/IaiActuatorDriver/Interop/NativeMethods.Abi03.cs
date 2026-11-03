using System;
using System.Runtime.InteropServices;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The ABI 0.3 additions of iai_actuator_driver.h, in header order (conventions: NativeMethods.cs): transports and
    /// buses as handles, the binding's own transport through callbacks, simulated units and lines, axes and cells opened
    /// on them and the cancel latch (the clock seam and the simulator's test hooks: NativeTestingMethods). Handles that the library may hand back more than once (a transport
    /// of a bus, a bus of an axis) are SafeHandles too: each call gives a new handle to release.
    /// </summary>
    internal static partial class NativeMethods
    {
        // ------------------------------------------------------------------------------------------ 0.3: constants

        /// <summary>IAD_INT_UNKNOWN (INT32_MIN): "not given / the default" for an int argument.</summary>
        internal const int IntUnknown = int.MinValue;

        /// <summary>IAD_UNKNOWN (0.8: 2; it was -1, which is True in Visual Basic): an iad_tribool that is not known. An
        /// input 0 is false and any other value except this one is true.</summary>
        internal const int TriUnknown = 2;

        // ------------------------------------------------------------------------------------------ 0.3: callbacks

        /// <summary>iad_transport_write_fn: send all bytes; 0 = success.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportWriteFn(IntPtr user, IntPtr data, UIntPtr size);

        /// <summary>iad_transport_read_fn: read up to size bytes within timeout_s; *got = the bytes read.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportReadFn(IntPtr user, IntPtr buffer, UIntPtr size, double timeoutS, out UIntPtr got);

        /// <summary>iad_transport_reset_input_fn.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportResetInputFn(IntPtr user);

        /// <summary>iad_transport_set_baud_rate_fn.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportSetBaudRateFn(IntPtr user, int rate);

        /// <summary>iad_transport_send_break_fn.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportSendBreakFn(IntPtr user, double seconds);

        /// <summary>iad_transport_present_fn: IAD_TRUE, IAD_FALSE or IAD_UNKNOWN (2).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportPresentFn(IntPtr user);

        /// <summary>iad_transport_reopen_fn (baud_rate 0: as before).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate int TransportReopenFn(IntPtr user, int baudRate);

        /// <summary>iad_transport_close_fn and iad_transport_release_fn (both: void(void* user)).</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate void TransportUserFn(IntPtr user);

        // ------------------------------------------------------------------------------------------ 0.3: library

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_callback_error(byte[]? message);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_set_log_level(int level);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_callbacks_init(ref IadTransportCallbacks callbacks);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_options_init(ref IadBusOptions options);

        // ------------------------------------------------------------------------------------------ 0.3: transports

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_open_callbacks(ref IadTransportCallbacks callbacks, byte[]? name, IntPtr user, out TransportHandle transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_open_serial(byte[] port, int baudRate, double readTimeoutS, out TransportHandle transport);

        /// <summary>Close the transport and release the handle (takes the handle, so a SafeHandle can be closed through it).</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_close(TransportHandle transport);

        /// <summary>Takes the raw token: called from <see cref="TransportHandle.ReleaseHandle"/>.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_release(IntPtr transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_info_json(TransportHandle transport, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_write(TransportHandle transport, byte[] data, UIntPtr size);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_read(TransportHandle transport, [Out] byte[] buffer, UIntPtr size, out UIntPtr got);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_reset_input(TransportHandle transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_set_baud_rate(TransportHandle transport, int rate);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_set_read_timeout(TransportHandle transport, double seconds);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_send_break(TransportHandle transport, double seconds);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_reopen(TransportHandle transport, int baudRate);

        // ------------------------------------------------------------------------------------------ 0.3: buses

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_open(TransportHandle transport, ref IadBusOptions options, out BusHandle bus);

        /// <summary>Takes the raw token: called from <see cref="BusHandle.ReleaseHandle"/> (closes an owned bus).</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_close(IntPtr bus);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_transport(BusHandle bus, out TransportHandle transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_info_json(BusHandle bus, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_retries(BusHandle bus, int retries);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_min_gap(BusHandle bus, double seconds);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_frame_sink(BusHandle bus, FrameSink? sink, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_read_registers(BusHandle bus, int slave, int address, int count, int retries, double maxWaitS, int chunk, [Out] ushort[] values);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_write_coil(BusHandle bus, int slave, int address, int on, int retries);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_write_register(BusHandle bus, int slave, int address, ushort value);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_write_registers(BusHandle bus, int slave, int address, ushort[] values, UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_read_write_registers(BusHandle bus, int slave, int readAddress, int readCount, int writeAddress, ushort[] values, UIntPtr count, [Out] ushort[] output);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_report_slave_id(BusHandle bus, int slave, int retries, double timeoutS, [Out] byte[]? buffer, UIntPtr bufferSize, out UIntPtr length);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_iai_command(BusHandle bus, int slave, int functionCode, int group, int command, uint[]? operands, UIntPtr operandCount, uint flags, int retries, out int rejection, [Out] uint[]? status, UIntPtr statusCount);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_broadcast_coil(BusHandle bus, int address, int on);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_probe(BusHandle bus, int slave, double timeoutS, int address, out int answered);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_detect_baud_rate(BusHandle bus, byte[]? targets, byte[]? candidates, int probeAddress, int currentFirst, double probeTimeoutS, out int rate);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_set_baud_rate(BusHandle bus, int rate);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_send_break(BusHandle bus, double seconds);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_bus_reopen(BusHandle bus, int baudRate);

        // ------------------------------------------------------------------------------------------ 0.3: simulators

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_open(byte[]? optionsJson, out SimHandle sim);

        /// <summary>Takes the raw token: called from <see cref="SimHandle.ReleaseHandle"/>.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_close(IntPtr sim);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_transport(SimHandle sim, out TransportHandle transport);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_line_open(byte[]? name, IntPtr[]? units, UIntPtr unitCount, out SimLineHandle line);

        /// <summary>Takes the raw token: called from <see cref="SimLineHandle.ReleaseHandle"/>.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_line_close(IntPtr line);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_line_add(SimLineHandle line, SimHandle sim);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_line_remove(SimLineHandle line, int slave);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_line_transport(SimLineHandle line, out TransportHandle transport);

        // ------------------------------------------------------------------------------------------ 0.3: axes and cells

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_open_bus(BusHandle bus, int slave, ref IadAxisOptions axisOptions, out AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_open_transport(TransportHandle transport, ref IadOpenOptions options, ref IadAxisOptions axisOptions, out AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_bus(AxisHandle axis, out BusHandle bus);

        /// <summary>The transports go as raw tokens: the caller keeps the SafeHandles alive (DangerousAddRef) across the call.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_open_transports(IntPtr[] transports, UIntPtr transportCount, byte[]? names, byte[]? slaves, ref IadCellOptions options, ref IadAxisOptions axisOptions, out CellHandle cell);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_bus(CellHandle cell, byte[] port, out BusHandle bus);

        // ------------------------------------------------------------------------------------------ 0.3: cancel latch

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_cancel_latch(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_cancel_reset(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_cancel_latched(AxisHandle axis, out int latched);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_cancel_latch(CellHandle cell, byte[]? axes);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_cancel_reset(CellHandle cell, byte[]? axes);
    }

    // ------------------------------------------------------------------------------------------ 0.3: structs

    /// <summary>
    /// iad_transport_callbacks: a sized struct (struct_size first) holding function pointers, so its size differs between
    /// 32-bit and 64-bit processes (96 bytes on 64-bit, the pointers from offset 24). The pointers are
    /// <see cref="Marshal.GetFunctionPointerForDelegate{TDelegate}(TDelegate)"/> of delegates kept alive for the life of
    /// the process (<see cref="TransportBridge"/>); IntPtr keeps the struct blittable.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct IadTransportCallbacks
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double ReadTimeoutS;
        public int BaudRate;
        public int RealTimeout;
        public IntPtr Write;
        public IntPtr Read;
        public IntPtr ResetInput;
        public IntPtr SetBaudRate;
        public IntPtr SendBreak;
        public IntPtr Present;
        public IntPtr Reopen;
        public IntPtr Close;
        public IntPtr Release;
    }

    /// <summary>iad_bus_options (56 bytes, a sized struct).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadBusOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double MinGapS;
        public int Retries;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }
}
