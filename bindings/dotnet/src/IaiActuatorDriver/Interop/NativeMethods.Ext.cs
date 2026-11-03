using System;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>The ABI 0.2 additions of iai_actuator_driver.h, in header order (conventions: NativeMethods.cs).</summary>
    internal static partial class NativeMethods
    {
        // ------------------------------------------------------------------------------------------ 0.2: sinks

        /// <summary>iad_log_sink: runs on the thread that logs; never calls the library.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate void LogSink(int level, IntPtr source, IntPtr message, IntPtr user);

        /// <summary>iad_frame_sink: runs on the thread that talks to the port; never calls the library.</summary>
        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        internal delegate void FrameSink(int direction, IntPtr bytes, UIntPtr length, IntPtr user);

        // ------------------------------------------------------------------------------------------ 0.2: library

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_set_log_sink(LogSink? sink, IntPtr user);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_push_options_init(ref IadPushOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_sequence_options_init(ref IadSequenceOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_monitor_options_init(ref IadMonitorOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_track_options_init(ref IadTrackOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_recovery_policy_init(ref IadRecoveryPolicy policy);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_check_options_init(ref IadCheckOptions options);

        // ------------------------------------------------------------------------------------------ 0.2: axis motion

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_push(AxisHandle axis, double target, double band, double forcePct, ref IadPushOptions options, out IadPushResult result);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_change_accel(AxisHandle axis, double accelG, ref IadChangeOptions options, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_move_sequence(AxisHandle axis, IadSequencePoint[] points, UIntPtr pointCount, ref IadSequenceOptions options, [In, Out] IadStatus[] statuses, UIntPtr capacity, out UIntPtr count);

        // ------------------------------------------------------------------------------------------ 0.2: move handles

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_start_move_to(AxisHandle axis, double position, ref IadMoveOptions options, out MoveHandle move);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_start_move_by(AxisHandle axis, double delta, ref IadMoveOptions options, out MoveHandle move);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_start_jog(AxisHandle axis, double delta, ref IadMoveOptions options, out MoveHandle move);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_done(MoveHandle move, out int done, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_wait(MoveHandle move, double timeoutS, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_poll(MoveHandle move, double maxWaitS, out int done, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_cancel(MoveHandle move, int wait, double timeoutS, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_result(MoveHandle move, out IadMoveInfo info);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_json(MoveHandle move, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        /// <summary>Takes the raw token: called from <see cref="MoveHandle.ReleaseHandle"/>.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_move_handle_close(IntPtr move);

        // ------------------------------------------------------------------------------------------ 0.2: status monitor

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_start_monitor(AxisHandle axis, ref IadMonitorOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_stop_monitor(AxisHandle axis, double timeoutS);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_monitor_latest(AxisHandle axis, out IadMonitorSample sample);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_monitor_wait_update(AxisHandle axis, long afterSeq, double timeoutS, out int got, out IadMonitorSample sample);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_monitor_wait_done(AxisHandle axis, double timeoutS, out int done, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_monitor_info_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_next_event(AxisHandle axis, double timeoutS, out IadEvent ev);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_frame_sink(AxisHandle axis, FrameSink? sink, IntPtr user);

        // ------------------------------------------------------------------------------------------ 0.2: fast mode

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_enable_fast(AxisHandle axis, int baudRate, int monitor, out IadFastSummary summary);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_disable_fast(AxisHandle axis, out IadFastSummary summary);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_fast_mode(AxisHandle axis, out int on);

        // ------------------------------------------------------------------------------------------ 0.2: recovery

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_track(AxisHandle axis, ref IadTrackOptions options, byte[]? name, byte[]? rows, byte[]? backupJson);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_session_from_backup(AxisHandle axis, byte[] backupJson, byte[]? name);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_session_from_backup_file(AxisHandle axis, byte[] path, byte[]? name);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_session_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_forget(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_note_target(AxisHandle axis, double targetMm);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_note_control(AxisHandle axis, int modbusControl, int safetySpeed, int baudRate, int homed);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_check_link(AxisHandle axis, out IadLinkCheck check);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_link_state(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_recover(AxisHandle axis, double timeoutS, ref IadRecoveryPolicy policy, byte[]? checkRows, byte[]? backupJson, byte[]? name, out IadRecoverySummary summary);

        // ------------------------------------------------------------------------------------------ 0.2: information

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_unit_info_json(AxisHandle axis, byte[]? sections, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_backup_json(AxisHandle axis, byte[]? rows, byte[]? path, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        /// <summary>Takes the raw tokens: the caller holds a reference on each handle for the call.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_backup_axes_json(IntPtr[] axes, UIntPtr axisCount, byte[]? names, byte[] directory, byte[]? rows, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_self_test(AxisHandle axis, ref IadCheckOptions options, byte[]? portLabel, out IadCheckSummary summary);

        // ------------------------------------------------------------------------------------------ 0.2: line, unit modes

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_line_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_gateway_status_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_position_unit(AxisHandle axis, double unitMm);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_action_mode(AxisHandle axis, int pc, int safety, int verify, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_safety_speed(AxisHandle axis, int on, int verify, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_cancel(AxisHandle axis);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_set_strategy(AxisHandle axis, int method);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_strategy(AxisHandle axis, out int method);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_stop_path(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_line_name(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_monitor_json(AxisHandle axis, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_lean_status(AxisHandle axis, int retries, out IadStatus status);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_boot_baud_rate(AxisHandle axis, out int rate);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_gateway_monitor_json(AxisHandle axis, byte[]? slaves, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_motion_caps(AxisHandle axis, out IadMotionCaps caps);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_read_home_pattern(AxisHandle axis, out int known, out uint pattern);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_axis_finish_home(AxisHandle axis);

        // ------------------------------------------------------------------------------------------ 0.2: cell

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_move_each(CellHandle cell, IadAxisMove[] moves, UIntPtr moveCount, int wait, double timeoutS, int stopOthersOnError, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_pause_all(CellHandle cell, byte[]? axes, int broadcast, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_resume_all(CellHandle cell, byte[]? axes, int broadcast, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_cancel(CellHandle cell, byte[]? axes);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_start_monitors(CellHandle cell, byte[]? axes, ref IadMonitorOptions options);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_stop_monitors(CellHandle cell, byte[]? axes);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_next_event(CellHandle cell, double timeoutS, out IadEvent ev);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_track_all(CellHandle cell, byte[]? axes, ref IadTrackOptions options, byte[]? rows, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_check_links(CellHandle cell, byte[]? axes, [In, Out] IadLinkCheck[] checks, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_monitor_links(CellHandle cell, double intervalS, byte[]? axes);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_stop_link_monitor(CellHandle cell);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_recover_all(CellHandle cell, byte[]? axes, double timeoutS, ref IadRecoveryPolicy policy, byte[]? checkRows, [In, Out] IadCellResult[] results, UIntPtr capacity, out UIntPtr count);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_monitor_all_json(CellHandle cell, byte[]? axes, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_lines_json(CellHandle cell, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_gateway_status_json(CellHandle cell, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_set_position_unit(CellHandle cell, byte[] port, double unitMm, byte[]? slaves);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_gateway_monitor_json(CellHandle cell, byte[] port, byte[]? slaves, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_backup_json(CellHandle cell, byte[]? axes, byte[] directory, byte[]? rows, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_last_result_json(CellHandle cell, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        // ------------------------------------------------------------------------------------------ 0.2: coordinated motion

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_coord_plan_json(CellHandle cell, byte[] request, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_cell_coord_run_json(CellHandle cell, byte[] request, byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);
    }
}
