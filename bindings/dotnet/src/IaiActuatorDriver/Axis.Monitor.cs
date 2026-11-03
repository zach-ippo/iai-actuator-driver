using System;
using System.Collections.Generic;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    // Fast mode, the status monitor and the axis's events. The monitor and the event queue are the core's; the events are
    // raised on a thread the binding owns, one at a time, in the order the core queued them.
    public sealed partial class Axis
    {
        /// <summary>The baud rate <see cref="EnableFast"/> raises the link to (VERIFIED on the EC-R6S rod).</summary>
        public const int FastBaudRate = 230400;

        private EventHandler<StatusChangedEventArgs>? _statusChanged;
        private EventHandler<MotionDoneEventArgs>? _motionDone;
        private EventHandler<LinkEventArgs>? _linkChanged;
        private Status? _lastChange;
        private AxisMonitor? _monitorView;

        // ------------------------------------------------------------------ fast mode

        /// <summary>
        /// Start a session like <see cref="Connect(bool)"/>, then <see cref="EnableFast"/> when <paramref name="fast"/> (the
        /// baud rate is raised to <see cref="FastBaudRate"/> when this axis owns the port).
        /// </summary>
        public DeviceInfo Connect(bool takeControl, bool fast)
        {
            var info = Connect(takeControl);
            if (fast)
            {
                EnableFast();
            }

            return info;
        }

        /// <summary>
        /// The fast path in one call: <see cref="CompletionRule.Fast"/> completion, the status monitor (<paramref name="monitor"/>;
        /// its events reach <see cref="StatusChanged"/>, <see cref="MotionDone"/> and <see cref="LinkChanged"/>) and the link raised
        /// to <paramref name="baudRate"/> (<c>null</c> or 0 keeps the rate). The baud change is tried only when this axis owns its
        /// port (<see cref="Axis.Open(string, int?, int, AxisOptions?, TimeSpan?, int, Action{FrameDirection, byte[]}?)"/>), the transport has a
        /// baud rate and the axis is at rest; otherwise the rate is kept and <see cref="FastModeSummary.Notes"/> says why. Safe to
        /// call again. 230400 is VERIFIED on the EC-R6S rod; the rest is UNVERIFIED on hardware (tested on the simulator).
        /// </summary>
        public FastModeSummary EnableFast(int? baudRate = FastBaudRate, bool monitor = true)
        {
            IadFastSummary s = default;
            Waiting(CancellationToken.None, () => NativeMethods.iad_axis_enable_fast(H, baudRate ?? 0, Known.Flag(monitor), out s));
            if (monitor)
            {
                StartEventPump();
            }

            return FastSummaryOf(s);
        }

        /// <summary>Undo <see cref="EnableFast"/>: <see cref="CompletionRule.Safe"/> completion again and the status monitor stopped. The baud rate is kept.</summary>
        public FastModeSummary DisableFast()
        {
            IadFastSummary s = default;
            Run(() => NativeMethods.iad_axis_disable_fast(H, out s));
            return FastSummaryOf(s);
        }

        private FastModeSummary FastSummaryOf(IadFastSummary s)
        {
            var native = new NativeFastSummary(s, Text(ThreadResultJson));
            return new FastModeSummary(native.BaudRate, native.Completion.FromNative(), native.Monitor, native.Notes);
        }

        /// <summary><see cref="EnableFast"/> ran and no <see cref="DisableFast"/> since.</summary>
        public bool FastMode
        {
            get
            {
                var on = 0;
                Run(() => NativeMethods.iad_axis_fast_mode(H, out on));
                return Known.Bool(on);
            }
        }

        /// <summary>What <see cref="EnableFast"/> could not do (the baud rate kept, and why); empty when it did everything.</summary>
        public IReadOnlyList<string> FastNotes => State().Strings("fast_notes");

        // ------------------------------------------------------------------ status monitor

        /// <summary>The status monitor, <c>null</c> when none was started (<see cref="StartMonitor"/>, <see cref="EnableFast"/>).</summary>
        public AxisMonitor? Monitor
        {
            get
            {
                if (_core.IsClosed || _core.MonitorInfoJson() == "null")
                {
                    return null;
                }

                return _monitorView ??= new AxisMonitor(this);
            }
        }

        /// <summary>
        /// Start the background status monitor (read-only: lean 8-word polls) and raise its events (<see cref="StatusChanged"/>,
        /// <see cref="MotionDone"/>, <see cref="LinkChanged"/>). One that runs already is kept (its settings are not changed).
        /// Stopped by <see cref="StopMonitor"/> and <see cref="Close"/>.
        /// </summary>
        public AxisMonitor StartMonitor(MonitorOptions? options = null)
        {
            var o = options ?? new MonitorOptions();
            var native = NativeMonitorOptions.ToNative(new NativeMonitorOptions
            {
                GapS = o.GapS,
                LinkCheckS = o.LinkCheckS ?? 0.0,
                Retries = o.Retries,
                QueueCapacity = o.QueueCapacity,
            });
            Run(() => NativeMethods.iad_axis_start_monitor(H, ref native));
            StartEventPump();
            return _monitorView ??= new AxisMonitor(this);
        }

        /// <summary>Stop the status monitor (no-op without one); <paramref name="timeoutS"/> is how long a poll in flight may take.</summary>
        public void StopMonitor(double timeoutS = 1.0)
        {
            if (_core.IsClosed)
            {
                return;
            }

            _monitorView?.Snapshot();
            Run(() => NativeMethods.iad_axis_stop_monitor(H, timeoutS));
        }

        // ------------------------------------------------------------------ events

        /// <summary>
        /// The monitor saw PNOW, ALMC, DSS1, DSS2 or DSSE change. Raised on a thread the binding owns, never on the thread that
        /// called the axis: keep the handler short and never call a blocking axis command from it; an exception it throws is ignored.
        /// Needs a running monitor (<see cref="StartMonitor"/> or <see cref="EnableFast"/>).
        /// </summary>
        public event EventHandler<StatusChangedEventArgs>? StatusChanged
        {
            add
            {
                lock (_sync)
                {
                    _statusChanged += value;
                }

                StartEventPump();
            }

            remove
            {
                lock (_sync)
                {
                    _statusChanged -= value;
                }
            }
        }

        /// <summary>The axis became done by the fast rule against its last commanded target (a rising edge of the monitor's samples). As <see cref="StatusChanged"/>.</summary>
        public event EventHandler<MotionDoneEventArgs>? MotionDone
        {
            add
            {
                lock (_sync)
                {
                    _motionDone += value;
                }

                StartEventPump();
            }

            remove
            {
                lock (_sync)
                {
                    _motionDone -= value;
                }
            }
        }

        /// <summary>
        /// The axis's link changed: "link_lost" (a transport error, or several failed polls in a row), "unit_restarted" (the link check
        /// saw the unit's uptime drop, or PC control or HEND lost), "unit_replaced", "recovered". Also raised for the results of
        /// <see cref="CheckLink"/> and <see cref="Recover"/>. The monitor never recovers anything itself: call <see cref="Recover"/>.
        /// As <see cref="StatusChanged"/>.
        /// </summary>
        public event EventHandler<LinkEventArgs>? LinkChanged
        {
            add
            {
                lock (_sync)
                {
                    _linkChanged += value;
                }

                StartEventPump();
            }

            remove
            {
                lock (_sync)
                {
                    _linkChanged -= value;
                }
            }
        }

        private void StartEventPump()
        {
            if (!_core.IsClosed)
            {
                _core.StartEventPump();
            }
        }

        /// <summary>Forward the core's events (constructor): they are queued whether or not anyone listens.</summary>
        private void HookEvents()
        {
            _core.StatusChanged += (_, e) =>
            {
                if (e.Status is null)
                {
                    return;
                }

                EventHandler<StatusChangedEventArgs>? handler;
                Status? previous;
                var status = e.Status.ToRecord();
                lock (_sync)
                {
                    handler = _statusChanged;
                    previous = _lastChange;
                    _lastChange = status;
                }

                Raise(handler, new StatusChangedEventArgs(status, previous, e.TimeS));
            };
            _core.MotionDone += (_, e) =>
            {
                if (e.Status is null)
                {
                    return;
                }

                EventHandler<MotionDoneEventArgs>? handler;
                lock (_sync)
                {
                    handler = _motionDone;
                }

                Raise(handler, new MotionDoneEventArgs(e.Status.ToRecord(), e.TimeS));
            };
            _core.LinkChanged += (_, e) =>
            {
                EventHandler<LinkEventArgs>? handler;
                lock (_sync)
                {
                    handler = _linkChanged;
                }

                Raise(handler, new LinkEventArgs(RecoveryEventOf(e)));
            };
        }

        /// <summary>Raise <paramref name="handler"/>: each subscriber on its own, so one that throws does not keep the others from the event.</summary>
        private void Raise<T>(EventHandler<T>? handler, T args)
            where T : EventArgs
        {
            if (handler is null)
            {
                return;
            }

            foreach (var one in handler.GetInvocationList())
            {
                try
                {
                    ((EventHandler<T>)one)(this, args);
                }
                catch (Exception exc)
                {
                    DriverLog.Warning(LogSource, "an event handler threw " + exc.GetType().Name + ": " + exc.Message);
                }
            }
        }

        internal static RecoveryEvent RecoveryEventOf(NativeEvent e) =>
            new RecoveryEvent(
                e.Name,
                e.Axis,
                e.Message,
                e.Cause,
                e.WallTime ?? DateTimeOffset.Now);

        /// <summary>Stop what this axis started in the core (the monitor and the event thread): <see cref="Close"/>.</summary>
        private void ShutdownEvents()
        {
            try
            {
                if (!_core.IsClosed)
                {
                    _monitorView?.Snapshot();
                    _core.StopMonitor(1.0, stopEvents: true);
                }
            }
            catch (Exception exc) when (exc is EcDriverException || exc is ObjectDisposedException)
            {
                DriverLog.Warning(LogSource, "could not stop the status monitor on close: " + exc.Message);
            }
        }
    }
}
