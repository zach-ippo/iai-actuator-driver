using System;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// P/Invoke declarations of iai_actuator_driver.h, in header order: ABI 0.1 here, the 0.2 additions in
    /// NativeMethods.Ext.cs. IAD_CALL is CallingConvention.Winapi (stdcall on 32-bit Windows only). Strings in are NUL-terminated UTF-8 byte arrays (<see cref="Utf8.Encode"/>); text and JSON out are byte buffers
    /// decoded as UTF-8; <c>const char*</c> results are <see cref="IntPtr"/> decoded by <see cref="Utf8.FromPointer"/>.
    /// This is done by hand rather than with <c>UnmanagedType.LPUTF8Str</c> so it behaves the same on every runtime the
    /// netstandard2.0 build can run on. size_t is <see cref="UIntPtr"/>. Structs go by <c>ref</c> (blittable: pinned,
    /// not copied). The binding always passes a struct where the header allows NULL.
    /// </summary>
    internal static partial class NativeMethods
    {
        /// <summary>Base name of the library: iai_actuator_driver.dll, libiai_actuator_driver.so / .dylib.</summary>
        internal const string Library = "iai_actuator_driver";

        /// <summary>IAD_CALL (0.8): the platform's convention, which is __stdcall on 32-bit Windows (the header's rule) and
        /// the one convention everywhere else (cdecl on Linux and macOS, the only one on x64).</summary>
        private const CallingConvention Winapi = CallingConvention.Winapi;

        // ------------------------------------------------------------------------------------------ library

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern uint iad_abi_version();

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern IntPtr iad_library_version();

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern IntPtr iad_last_error_message();

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_last_error_json(byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern IntPtr iad_result_name(int code);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_list_ports_json(byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ option defaults

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_options_init(ref IadAxisOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_open_options_init(ref IadOpenOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sim_options_init(ref IadSimOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_options_init(ref IadMoveOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_jog_options_init(ref IadMoveOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_position_move_options_init(ref IadPositionMoveOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_nv_write_options_init(ref IadNvWriteOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_param_write_options_init(ref IadParamWriteOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_baud_options_init(ref IadBaudOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_reset_options_init(ref IadResetOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_options_init(ref IadCellOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_position_entry_init(ref IadPositionEntry entry);

        // (0.8) the options of the axis commands
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_servo_options_init(ref IadServoOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_home_options_init(ref IadHomeOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_stop_options_init(ref IadStopOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_wait_idle_options_init(ref IadWaitIdleOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_change_options_init(ref IadChangeOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cancel_paused_move_options_init(ref IadCancelPausedMoveOptions options);

        // ------------------------------------------------------------------------------------------ axis: session

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_open(byte[] port, ref IadOpenOptions options, ref IadAxisOptions axisOptions, out AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_open_sim(byte[]? profile, ref IadSimOptions sim, ref IadOpenOptions options, ref IadAxisOptions axisOptions, out AxisHandle axis);

        /// <summary>Takes the raw token: called from <see cref="AxisHandle.ReleaseHandle"/>.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_close(IntPtr axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_connect(AxisHandle axis, int takeControl, out IadIdentity identity);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_identify(AxisHandle axis, out IadIdentity identity);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_identity_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_get_options(AxisHandle axis, ref IadAxisOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_options(AxisHandle axis, ref IadAxisOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_baud_rate(AxisHandle axis, out int rate, out int detected);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_modbus_control(AxisHandle axis, int on);

        // ------------------------------------------------------------------------------------------ axis: reading

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_status(AxisHandle axis, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_limits(AxisHandle axis, out IadLimits limits);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_limits_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_alarm_history_json(AxisHandle axis, int includeEmpty, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_max_positions(AxisHandle axis, out int count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_position_table(AxisHandle axis, int row, ref IadPositionEntry entry);

        // ------------------------------------------------------------------------------------------ axis: control

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_servo(AxisHandle axis, int on, ref IadServoOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_home(AxisHandle axis, ref IadHomeOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_reset_alarm(AxisHandle axis, int method, out IadStatus status);

        // ------------------------------------------------------------------------------------------ axis: motion

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_check_move(AxisHandle axis, double position, ref IadMoveOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_move_to(AxisHandle axis, double position, ref IadMoveOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_move_by(AxisHandle axis, double delta, ref IadMoveOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_jog(AxisHandle axis, double delta, ref IadMoveOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_move_to_position(AxisHandle axis, int row, ref IadPositionMoveOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_change_speed(AxisHandle axis, double speed, ref IadChangeOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_wait_idle(AxisHandle axis, ref IadWaitIdleOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_stop(AxisHandle axis, ref IadStopOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_pause(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_resume(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_cancel_paused_move(AxisHandle axis, ref IadCancelPausedMoveOptions options, out IadStatus status);

        // ------------------------------------------------------------------------------------------ axis: position table

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_write_position_table(AxisHandle axis, int row, ref IadPositionEntry entry, ref IadNvWriteOptions options, out IadPositionWriteResult result);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_teach_position(AxisHandle axis, int row, int method, ref IadNvWriteOptions options, out IadPositionWriteResult result);

        // ------------------------------------------------------------------------------------------ axis: parameters

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_parameters_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_parameter(AxisHandle axis, byte[] key, out IadParameter parameter);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_parameter_json(AxisHandle axis, byte[] key, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_write_parameter(AxisHandle axis, byte[] key, byte[] value, ref IadParamWriteOptions options, out IadParamWriteResult result);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_write_parameter_number(AxisHandle axis, byte[] key, double value, ref IadParamWriteOptions options, out IadParamWriteResult result);

        // ------------------------------------------------------------------------------------------ axis: non-volatile guard

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_nonvolatile_writes(AxisHandle axis, out long total, out int lastHour, out int uncommitted);

        // ------------------------------------------------------------------------------------------ axis: unit

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_baud_rate(AxisHandle axis, int rate, ref IadBaudOptions options, out IadBaudResult result);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_software_reset(AxisHandle axis, ref IadResetOptions options, out IadResetResult result);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_last_result_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ cell

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_open(byte[]? ports, ref IadCellOptions options, ref IadAxisOptions axisOptions, out CellHandle cell);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_open_sim(byte[]? profiles, int slavesPerLine, ref IadSimOptions sim, ref IadCellOptions options, ref IadAxisOptions axisOptions, out CellHandle cell);

        /// <summary>Takes the raw token: called from <see cref="CellHandle.ReleaseHandle"/>.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_close(IntPtr cell);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_axis_count(CellHandle cell, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_axis_name(CellHandle cell, UIntPtr index, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_open_axis(CellHandle cell, byte[] name, out AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_info_json(CellHandle cell, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_clear_fault(CellHandle cell, byte[] axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_status_all(CellHandle cell, byte[]? axes, double monitorMaxAgeS, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_servo_all(CellHandle cell, int on, byte[]? axes, double timeoutS, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_home_all(CellHandle cell, byte[]? axes, double timeoutS, int method, int stopOthersOnError, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_move_all(CellHandle cell, byte[] axes, double[] positions, UIntPtr positionCount, ref IadMoveOptions options, double[]? speeds, double[]? accelsG, double[]? bands, double[]? decelsG, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_wait_all(CellHandle cell, byte[]? axes, double timeoutS, int stopOthersOnError, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_stop_all(CellHandle cell, byte[]? axes, double timeoutS, int broadcast, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);
    }
}
