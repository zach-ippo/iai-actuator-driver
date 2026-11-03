using System;
using System.Threading;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // (ABI 0.2) The status monitor, the event queue as C# events, and the frame handler of one axis.
    internal sealed partial class NativeAxis
    {
        private readonly object _eventGate = new object();
        private NativeEventPump? _pump;
        private long _frameId;

        /// <summary>
        /// Every queued event of this axis (status changes, done edges, link events, self-test lines), raised on the
        /// binding's event thread (<see cref="StartEventPump"/>; <see cref="StartMonitor"/> starts it). A handler runs on
        /// that thread: keep it short; an exception it throws is ignored.
        /// </summary>
        public event EventHandler<NativeEvent>? EventReceived;

        /// <summary>The monitor saw PNOW / ALMC / DSS1 / DSS2 / DSSE change (<see cref="NativeEvent.Status"/> is the sample).</summary>
        public event EventHandler<NativeEvent>? StatusChanged;

        /// <summary>The axis became done by the fast rule against its last target (the monitor's done edge).</summary>
        public event EventHandler<NativeEvent>? MotionDone;

        /// <summary>A link event: "link_lost", "unit_restarted", "unit_replaced", "recovered" (also faults and link-state changes).</summary>
        public event EventHandler<NativeEvent>? LinkChanged;

        /// <summary>One line of <see cref="SelfTest"/>: name = the item, cause = "PASS" / "FAIL" / "WARN" / "SKIP".</summary>
        public event EventHandler<NativeEvent>? CheckItem;

        /// <summary>The binding's event thread runs (<see cref="StartEventPump"/>).</summary>
        public bool EventPumpRunning
        {
            get
            {
                lock (_eventGate)
                {
                    return _pump?.IsRunning == true;
                }
            }
        }

        // ------------------------------------------------------------------------------------------ status monitor

        /// <summary>
        /// Start the background status monitor (Axis::start_monitor; read-only, lean 8-word polls) and queue its events on
        /// this axis (<see cref="NativeMonitorOptions.Events"/>). Already running: the options other than the queue's are
        /// ignored and the running monitor is kept. With <paramref name="raiseEvents"/> (the default) the binding's event
        /// thread raises them as <see cref="StatusChanged"/>, <see cref="MotionDone"/>... Stopped by
        /// <see cref="StopMonitor"/> and <see cref="Close"/>.
        /// </summary>
        public void StartMonitor(NativeMonitorOptions? options = null, bool raiseEvents = true)
        {
            var o = NativeMonitorOptions.ToNative(options);
            NativeErrors.Check(NativeMethods.iad_axis_start_monitor(_handle, ref o));
            if (raiseEvents)
            {
                StartEventPump();
            }
        }

        /// <summary>
        /// Stop the status monitor (no-op without one); <paramref name="timeout"/> null = 1 s. With
        /// <paramref name="stopEvents"/> (the default) the event thread ends too, after raising what is still queued.
        /// </summary>
        public void StopMonitor(double? timeout = null, bool stopEvents = true)
        {
            NativeErrors.Check(NativeMethods.iad_axis_stop_monitor(_handle, Known.OrNaN(timeout)));
            if (stopEvents)
            {
                StopEventPump();
            }
        }

        /// <summary>
        /// (0.5) Hold the status monitor (AxisMonitor::suspend): no polls until <see cref="ResumeMonitor"/>; a poll in flight
        /// is waited for, at most <paramref name="timeout"/> (null = 1 s). Nests; <see cref="EcNotReadyException"/> without
        /// a monitor.
        /// </summary>
        public void SuspendMonitor(double? timeout = null) =>
            NativeErrors.Check(NativeMethods.iad_axis_suspend_monitor(_handle, Known.OrNaN(timeout)));

        /// <summary>(0.5) Release the innermost <see cref="SuspendMonitor"/> of this handle.</summary>
        public void ResumeMonitor() => NativeErrors.Check(NativeMethods.iad_axis_resume_monitor(_handle));

        /// <summary>
        /// (0.5) The target and band (mm) the fast done rule checks a monitor sample against: the last 0x9900 move's, null
        /// when none is known (no move yet, or a home return, a push or a stored row since). No I/O.
        /// </summary>
        public (double Target, double Band)? MonitorTarget()
        {
            NativeErrors.Check(NativeMethods.iad_axis_monitor_target(_handle, out var target, out var band));
            return double.IsNaN(target) ? ((double, double)?)null : (target, band);
        }

        /// <summary>
        /// (0.5) This handle's axis is the one user of its bus (Axis::set_port_owner): <see cref="EnableFast"/> may then
        /// change the port's rate. Set before <see cref="EnableFast"/> on an axis opened on a bus it owns. No I/O.
        /// </summary>
        public void SetPortOwner(bool owner) => NativeErrors.Check(NativeMethods.iad_axis_set_port_owner(_handle, Known.Flag(owner)));

        /// <summary>The monitor's last sample, no I/O. <see cref="EcNotReadyException"/> when no monitor was started.</summary>
        public NativeMonitorSample MonitorLatest()
        {
            NativeErrors.Check(NativeMethods.iad_axis_monitor_latest(_handle, out var sample));
            return new NativeMonitorSample(sample);
        }

        /// <summary>
        /// Wait for the first sample with Seq &gt; <paramref name="afterSeq"/> (0: the next one) (AxisMonitor::wait_update).
        /// Null on timeout or when the monitor stopped; the monitor's error when it ended on one (port gone).
        /// <paramref name="timeout"/> null = 1 s.
        /// </summary>
        public NativeMonitorSample? MonitorWaitUpdate(long afterSeq = 0, double? timeout = null)
        {
            NativeErrors.Check(NativeMethods.iad_axis_monitor_wait_update(_handle, afterSeq, Known.OrNaN(timeout), out var got, out var sample));
            return Known.Bool(got) ? new NativeMonitorSample(sample) : null;
        }

        /// <summary>
        /// Wait until a sample shows the axis done by the fast rule against its last target (AxisMonitor::wait_done).
        /// Null on timeout; <paramref name="timeout"/> null = 10 s.
        /// </summary>
        public NativeStatus? MonitorWaitDone(double? timeout = null)
        {
            NativeErrors.Check(NativeMethods.iad_axis_monitor_wait_done(_handle, Known.OrNaN(timeout), out var done, out var st));
            return Known.Bool(done) ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// The monitor as JSON, no I/O: {"running", "name", "polls", "error_count", "link_checks", "poll_rate_hz",
        /// "age_s", "link_state", "gap_s", "retries", "link_check_s", "error", "last_error"}; "null" without a monitor.
        /// </summary>
        public string MonitorInfoJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_monitor_info_json(_handle, b, s, out r));

        // ------------------------------------------------------------------------------------------ event queue

        /// <summary>
        /// The next queued event of this axis, waiting up to <paramref name="timeout"/> s (0: no wait); null when none
        /// came. For a caller that drains the queue itself instead of the C# events (do not run both: each event is
        /// returned once). Status / done events need a monitor; link events come from the monitor,
        /// <see cref="CheckLink"/> and <see cref="Recover"/>.
        /// </summary>
        public NativeEvent? NextEvent(double timeout = 0.0)
        {
            NativeErrors.Check(NativeMethods.iad_axis_next_event(_handle, timeout, out var ev));
            return ev.Kind == 0 ? null : new NativeEvent(ev);
        }

        /// <summary>
        /// Start the binding's event thread: it drains this axis's queue and raises <see cref="EventReceived"/> and the
        /// event of each kind. No-op when it runs. Events are queued only by what feeds the queue (the monitor, a link
        /// check, a recovery, a self-test).
        /// </summary>
        public void StartEventPump()
        {
            lock (_eventGate)
            {
                if (_pump?.IsRunning == true)
                {
                    return;
                }

                _pump = new NativeEventPump("iad-events " + Name, NextEventCall, Dispatch);
            }
        }

        /// <summary>End the event thread after it raised what is still queued (no-op when it does not run).</summary>
        public void StopEventPump()
        {
            NativeEventPump? pump;
            lock (_eventGate)
            {
                pump = _pump;
                _pump = null;
            }

            pump?.Stop(TimeSpan.FromSeconds(5));
        }

        private int NextEventCall(double timeoutS, out IadEvent ev) => NativeMethods.iad_axis_next_event(_handle, timeoutS, out ev);

        private void Dispatch(NativeEvent e)
        {
            NativeEventPump.Raise(EventReceived, this, e);
            switch (e.Kind)
            {
                case NativeEventKind.Status:
                    NativeEventPump.Raise(StatusChanged, this, e);
                    break;
                case NativeEventKind.Done:
                    NativeEventPump.Raise(MotionDone, this, e);
                    break;
                case NativeEventKind.Link:
                case NativeEventKind.Fault:
                case NativeEventKind.LinkState:
                    NativeEventPump.Raise(LinkChanged, this, e);
                    break;
                case NativeEventKind.CheckItem:
                    NativeEventPump.Raise(CheckItem, this, e);
                    break;
            }
        }

        // ------------------------------------------------------------------------------------------ frames

        /// <summary>
        /// Install (<paramref name="handler"/> not null) or remove (null) the frame handler of this axis's port
        /// (iad_axis_set_frame_sink; on a shared port it sees the frames of every axis of the port): (direction, the
        /// frame's bytes). It runs on the thread that talks to the port: copy what you need, return quickly and never call
        /// the driver from it; an exception it throws is ignored. Removed by <see cref="Close"/>.
        /// </summary>
        public void SetFrameHandler(Action<FrameDirection, byte[]>? handler)
        {
            var id = NativeSinks.SetFrame(_handle, handler);
            NativeSinks.ForgetFrame(Interlocked.Exchange(ref _frameId, id));
        }

        /// <summary>Remove the frame handler this axis installed, if any (best effort: the port may be closed already).</summary>
        private void ReleaseFrameHandler()
        {
            var id = Interlocked.Exchange(ref _frameId, 0);
            if (id == 0)
            {
                return;
            }

            try
            {
                NativeMethods.iad_axis_set_frame_sink(_handle, null, IntPtr.Zero);
            }
            catch (ObjectDisposedException)
            {
                // Closed already: no more frames.
            }

            NativeSinks.ForgetFrame(id);
        }
    }
}
