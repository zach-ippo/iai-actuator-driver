using System;
using System.Collections.Generic;
using System.Globalization;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// One axis on the native core: an ELECYLINDER unit or a CON controller at one slave address (iad_axis; C++ Axis).
    /// Method names follow the managed edition's <c>IaiActuatorDriver.Axis</c>; each does what the C++ method of the same
    /// name does (checks, completion rule, STOP handling and the non-volatile guard are the core's). Errors are the
    /// managed edition's exception classes (<see cref="NativeErrors"/>).
    /// </summary>
    /// <remarks>
    /// <para>Threads: calls on one axis are serialised by the core, except <see cref="Stop"/>, <see cref="Pause"/> and
    /// <see cref="Cancel"/>, which may be called from another thread while a move, home or <see cref="WaitIdle"/> waits on
    /// this axis (the way a stop button or a watchdog stops a blocking move). Axes on different ports run in parallel.
    /// The events (<see cref="StatusChanged"/>, <see cref="MotionDone"/>...) are raised on a thread the binding owns.</para>
    /// <para>Lifetime: <see cref="Dispose"/> (or <see cref="Close"/>) releases a STOP / pause the axis holds and closes
    /// the port when the axis owns it; an axis from <see cref="NativeCell"/> stays in the cell. The handle is also closed
    /// when the object is collected without Dispose. A call on a disposed axis throws
    /// <see cref="ObjectDisposedException"/>.</para>
    /// <para>Evidence: the C++ core's command line is VERIFIED on the EC-GRBP8M gripper and on the PCON-CB with an RCP6;
    /// this binding is VERIFIED on the PCON-CB with an RCP6 (steps in docs/NATIVE_ABI.md), tested on the native simulator
    /// besides; everything else UNVERIFIED.</para>
    /// </remarks>
    internal sealed partial class NativeAxis : IDisposable
    {
        private readonly AxisHandle _handle;

        internal NativeAxis(AxisHandle handle, string name)
        {
            _handle = handle;
            Name = name;
        }

        /// <summary>"COM3:1", "SIM:1" for a simulated axis, or the cell's name for one of a cell's axes.</summary>
        public string Name { get; }

        /// <summary>The native handle (for calls this class does not wrap).</summary>
        public AxisHandle Handle => _handle;

        /// <summary>The axis was closed.</summary>
        public bool IsClosed => _handle.IsClosed;

        // ------------------------------------------------------------------------------------------ open / close

        /// <summary>
        /// Open serial port <paramref name="portName"/> once and return an axis that owns it (iad_axis_open). With
        /// <paramref name="baudRate"/> null the rate is found in place (115200, 38400, 9600, 230400...);
        /// <see cref="ModbusTimeoutException"/> when no rate answers, <see cref="ModbusException"/> when the port is busy or
        /// absent. No identity is read: call <see cref="Connect"/>. (0.2) <paramref name="line"/>: the line kind (Auto probes
        /// once for a gateway); <paramref name="positionUnitMm"/>: an RCON line's position unit (0.01 or 0.001);
        /// <paramref name="fast"/>: then <see cref="EnableFast"/>.
        /// </summary>
        public static NativeAxis Open(
            string portName,
            int? baudRate = null,
            int slave = 1,
            NativeAxisOptions? options = null,
            TimeSpan? readTimeout = null,
            int? retries = null,
            TimeSpan? minGap = null,
            NativeLineKind line = NativeLineKind.Auto,
            double? positionUnitMm = null,
            bool fast = false)
        {
            NativeDriver.EnsureLoaded();
            var port = Utf8.EncodeRequired(portName, nameof(portName));
            var open = OpenOptions(slave, baudRate, readTimeout, retries, minGap, line, positionUnitMm, fast);
            var axisOptions = (options ?? new NativeAxisOptions()).ToNative();
            var result = NativeMethods.iad_axis_open(port, ref open, ref axisOptions, out var handle);
            return Adopt(result, handle, portName + ":" + slave.ToString(CultureInfo.InvariantCulture));
        }

        /// <summary>
        /// An axis on a simulated unit of the native core (iad_axis_open_sim): no hardware, never opens a port.
        /// <paramref name="profile"/>: "slider", "gripper", "rod", "pcon", "rotary" or "unknown" (null = "slider";
        /// <see cref="ArgumentException"/> otherwise). The unit answers at <paramref name="slave"/>. The rate is detected
        /// only when <paramref name="sim"/> models one and <paramref name="baudRate"/> is null. (0.2) <paramref name="fast"/>:
        /// then <see cref="EnableFast"/>.
        /// </summary>
        public static NativeAxis OpenSim(
            string? profile = null,
            NativeSimOptions? sim = null,
            int slave = 1,
            int? baudRate = null,
            NativeAxisOptions? options = null,
            bool fast = false)
        {
            NativeDriver.EnsureLoaded();
            var name = Utf8.Encode(profile, nameof(profile));
            var simOptions = NativeSimOptions.ToNative(sim);
            var open = OpenOptions(slave, baudRate, null, null, null, NativeLineKind.Auto, null, fast);
            var axisOptions = (options ?? new NativeAxisOptions()).ToNative();
            var result = NativeMethods.iad_axis_open_sim(name, ref simOptions, ref open, ref axisOptions, out var handle);
            return Adopt(result, handle, "SIM:" + slave.ToString(CultureInfo.InvariantCulture));
        }

        /// <summary>Wrap a handle an open function returned, or throw its error (the handle is then empty).</summary>
        internal static NativeAxis Adopt(int result, AxisHandle handle, string name)
        {
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return new NativeAxis(handle, name);
        }

        private static IadOpenOptions OpenOptions(
            int slave, int? baudRate, TimeSpan? readTimeout, int? retries, TimeSpan? minGap, NativeLineKind line, double? positionUnitMm, bool fast)
        {
            var o = SizedStruct.Fresh<IadOpenOptions>();
            NativeErrors.Check(NativeMethods.iad_open_options_init(ref o));
            o.Slave = slave;
            o.BaudRate = baudRate ?? 0;
            o.ReadTimeoutS = readTimeout?.TotalSeconds ?? o.ReadTimeoutS;
            o.Retries = retries ?? o.Retries;
            o.MinGapS = minGap?.TotalSeconds ?? o.MinGapS;
            o.Line = (int)line;
            o.PositionUnitMm = positionUnitMm ?? o.PositionUnitMm;
            o.Fast = Known.Flag(fast);
            return o;
        }

        /// <summary>
        /// Release a STOP / pause this axis holds, close the port when the axis owns it and release the handle
        /// (iad_axis_close). Safe to call twice. A call still running on another thread keeps the axis alive until it
        /// returns. The event thread ends and a frame handler set on this axis is removed first.
        /// </summary>
        public void Close()
        {
            StopEventPump();
            ReleaseFrameHandler();
            _handle.Dispose();
        }

        /// <inheritdoc cref="Close"/>
        public void Dispose() => Close();

        /// <inheritdoc />
        public override string ToString() => $"NativeAxis({Name})";

        // ------------------------------------------------------------------------------------------ session

        /// <summary>Read identity and ranges and, with <paramref name="takeControl"/>, take Modbus control at rest (Axis::connect).</summary>
        public NativeIdentity Connect(bool takeControl = true)
        {
            NativeErrors.Check(NativeMethods.iad_axis_connect(_handle, Known.Flag(takeControl), out var identity));
            return new NativeIdentity(identity);
        }

        /// <summary>Read the identity again (Axis::identify; READ-ONLY).</summary>
        public NativeIdentity Identify()
        {
            NativeErrors.Check(NativeMethods.iad_axis_identify(_handle, out var identity));
            return new NativeIdentity(identity);
        }

        /// <summary>
        /// The full identity record of the last <see cref="Connect"/> / <see cref="Identify"/> as JSON (to_json(DeviceInfo));
        /// no I/O. <see cref="EcNotReadyException"/> before the first connect / identify.
        /// </summary>
        public string UnitInfoJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_identity_json(_handle, b, s, out r));

        /// <summary>The axis settings (Axis::options / set_options). Setting replaces them all; null properties take the defaults.</summary>
        public NativeAxisOptions Options
        {
            get
            {
                var o = SizedStruct.Fresh<IadAxisOptions>();
                NativeErrors.Check(NativeMethods.iad_axis_get_options(_handle, ref o));
                return NativeAxisOptions.FromNative(o);
            }

            set
            {
                var o = (value ?? new NativeAxisOptions()).ToNative();
                NativeErrors.Check(NativeMethods.iad_axis_set_options(_handle, ref o));
            }
        }

        /// <summary>The port's baud rate; null on a transport without one.</summary>
        public int? BaudRate
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_baud_rate(_handle, out var rate, out _));
                return rate > 0 ? rate : (int?)null;
            }
        }

        /// <summary>The baud rate was detected at open rather than given.</summary>
        public bool BaudDetected
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_baud_rate(_handle, out _, out var detected));
                return Known.Bool(detected);
            }
        }

        /// <summary>Take Modbus control: PMSL ON at rest, verified by PMSS (Axis::enable_modbus_control).</summary>
        public void EnableModbusControl() => NativeErrors.Check(NativeMethods.iad_axis_modbus_control(_handle, 1));

        /// <summary>Release Modbus control: command authority back to PIO (Axis::disable_modbus_control).</summary>
        public void DisableModbusControl() => NativeErrors.Check(NativeMethods.iad_axis_modbus_control(_handle, 0));

        // ------------------------------------------------------------------------------------------ reading (READ-ONLY)

        /// <summary>One status read (Axis::status).</summary>
        public NativeStatus Status()
        {
            NativeErrors.Check(NativeMethods.iad_axis_status(_handle, out var st));
            return new NativeStatus(st);
        }

        /// <summary>Every range the driver enforces, read from the unit (Axis::limits).</summary>
        public NativeLimits Limits()
        {
            NativeErrors.Check(NativeMethods.iad_axis_limits(_handle, out var limits));
            return new NativeLimits(limits);
        }

        /// <summary>The same with each range's source in words, as JSON (to_json(AxisRanges)).</summary>
        public string LimitsJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_limits_json(_handle, b, s, out r));

        /// <summary>
        /// Alarm history, newest first (Axis::read_alarm_history). <paramref name="includeEmpty"/>: every slot, not only up
        /// to the first empty one.
        /// </summary>
        public IReadOnlyList<NativeAlarmRecord> AlarmHistory(bool includeEmpty = false)
        {
            var records = new List<NativeAlarmRecord>();
            if (MiniJson.Parse(AlarmHistoryJson(includeEmpty)) is List<object?> items)
            {
                foreach (var item in items)
                {
                    if (item is JsonObject record)
                    {
                        records.Add(new NativeAlarmRecord(record));
                    }
                }
            }

            return records;
        }

        /// <summary>The alarm history as JSON: [{"index", "code", "name", "level", "detail", "address", "time_ms", ...}].</summary>
        public string AlarmHistoryJson(bool includeEmpty = false) =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) =>
                NativeMethods.iad_axis_read_alarm_history_json(_handle, Known.Flag(includeEmpty), b, s, out r));

        /// <summary>Stored rows on this unit (Axis::max_positions: 3 on the ECs tested, 64 on the PCON).</summary>
        public int MaxPositions()
        {
            NativeErrors.Check(NativeMethods.iad_axis_max_positions(_handle, out var count));
            return count;
        }

        /// <summary>One stored row (Axis::read_position_table).</summary>
        public NativePositionEntry ReadPositionTable(int row)
        {
            var entry = SizedStruct.Fresh<IadPositionEntry>();
            NativeErrors.Check(NativeMethods.iad_axis_read_position_table(_handle, row, ref entry));
            return new NativePositionEntry(entry);
        }

        // ------------------------------------------------------------------------------------------ control

        /// <summary>
        /// Servo on / off (Axis::servo). A servo ON the status shows cannot work (EMG input, drive power cut, enable
        /// switch) is refused before anything is sent (<see cref="EcNotReadyException"/>). Returns the status when
        /// <paramref name="wait"/>, else null. <paramref name="timeout"/> null = the core's default.
        /// </summary>
        public NativeStatus? Servo(bool on, bool wait = true, double? timeout = null)
        {
            var o = IadServoOptions.Of(wait, Known.OrNaN(timeout));
            NativeErrors.Check(NativeMethods.iad_axis_servo(_handle, Known.Flag(on), ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// Home return (Axis::home); with <paramref name="wait"/> it waits for HEND and standstill and returns the status.
        /// <paramref name="method"/> null = the axis option.
        /// </summary>
        public NativeStatus? Home(bool wait = true, double? timeout = null, CommandMethod? method = null)
        {
            var code = NativeAxisOptions.MethodCode(method);
            var o = IadHomeOptions.Of(wait, Known.OrNaN(timeout), code);
            NativeErrors.Check(NativeMethods.iad_axis_home(_handle, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>Clear the current alarm (Axis::reset_alarm); returns the new status.</summary>
        public NativeStatus ResetAlarm(CommandMethod? method = null)
        {
            var code = NativeAxisOptions.MethodCode(method);
            NativeErrors.Check(NativeMethods.iad_axis_reset_alarm(_handle, code, out var st));
            return new NativeStatus(st);
        }

        // ------------------------------------------------------------------------------------------ motion

        /// <summary>Every check of <see cref="MoveTo"/>, nothing sent (Axis::check_move); throws what MoveTo would.</summary>
        public NativeStatus CheckMove(
            double position,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default)
        {
            var o = MoveOptions(speed, accelG, band, true, null, jog: false, decelG, accelPattern, NativeCompletion.Default);
            NativeErrors.Check(NativeMethods.iad_axis_check_move(_handle, position, ref o, out var st));
            return new NativeStatus(st);
        }

        /// <summary>
        /// Absolute move (Axis::move_to, the 0x9900 direct block), checked against the unit's ranges before anything is
        /// sent (<see cref="SoftLimitException"/>). With <paramref name="wait"/> it blocks until in position and still and
        /// returns the final status; otherwise it returns null right after the command. <paramref name="timeout"/> null =
        /// 2 x the move's estimate + 2 s. (0.2) <paramref name="decelG"/>: a deceleration other than
        /// <paramref name="accelG"/>, sent by the command-profile path (FC17 0x0F00 + PCRF; accel checked against MAXA,
        /// decel against MAXD); <paramref name="accelPattern"/>: CTLF b6-b7 (refused when the unit's MVPT lacks it);
        /// <paramref name="completion"/>: when the move counts as done (default: the axis option).
        /// </summary>
        public NativeStatus? MoveTo(
            double position,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = MoveOptions(speed, accelG, band, wait, timeout, jog: false, decelG, accelPattern, completion);
            NativeErrors.Check(NativeMethods.iad_axis_move_to(_handle, position, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>Relative to the measured position, sent as an absolute target (Axis::move_by). As <see cref="MoveTo"/>.</summary>
        public NativeStatus? MoveBy(
            double delta,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = MoveOptions(speed, accelG, band, wait, timeout, jog: false, decelG, accelPattern, completion);
            NativeErrors.Check(NativeMethods.iad_axis_move_by(_handle, delta, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>A small relative step at a low default speed (Axis::jog; limits jog_step). As <see cref="MoveTo"/>.</summary>
        public NativeStatus? Jog(
            double step,
            double speed = 10.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = MoveOptions(speed, accelG, band, wait, timeout, jog: true, decelG, accelPattern, completion);
            NativeErrors.Check(NativeMethods.iad_axis_jog(_handle, step, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// Run stored row <paramref name="row"/> (Axis::move_to_position; the row is read and checked first).
        /// <paramref name="speedPct"/>: 1..100 % of the row's speed (nothing written); null = the row's.
        /// <paramref name="allowAboveCaps"/>: only the unit's own limits apply to the row's speed / accel.
        /// </summary>
        public NativeStatus? MoveToPosition(int row, double? speedPct = null, bool wait = true, double? timeout = null, bool allowAboveCaps = false)
        {
            var o = SizedStruct.Fresh<IadPositionMoveOptions>();
            NativeErrors.Check(NativeMethods.iad_position_move_options_init(ref o));
            o.SpeedPct = Known.OrNaN(speedPct);
            o.Wait = Known.Flag(wait);
            o.TimeoutS = timeout ?? o.TimeoutS;
            o.AllowAboveCaps = Known.Flag(allowAboveCaps);
            NativeErrors.Check(NativeMethods.iad_axis_move_to_position(_handle, row, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// Change the speed of the move in progress (Axis::change_speed). Refused (nothing sent) without a move of this
        /// axis running, while paused, or outside the limits.
        /// </summary>
        public NativeStatus? ChangeSpeed(double speed, bool wait = true, double? timeout = null)
        {
            var o = IadChangeOptions.Of(wait, Known.OrNaN(timeout));
            NativeErrors.Check(NativeMethods.iad_axis_change_speed(_handle, speed, ref o, out var st));
            return wait ? new NativeStatus(st) : null;
        }

        /// <summary>
        /// Block until in position and still (Axis::wait_idle), at <paramref name="target"/> within
        /// <paramref name="band"/> when given. <see cref="MotionIncompleteException"/> when it settles away from the
        /// target; on timeout a decel stop is sent and <see cref="MotionTimeoutException"/> thrown. (0.2)
        /// <paramref name="completion"/>: the rule (default: the axis option); <see cref="NativeCompletion.Fast"/> uses one
        /// lean sample (the status monitor's when it runs).
        /// </summary>
        public NativeStatus WaitIdle(double? target = null, double? band = null, double timeout = 10.0, NativeCompletion completion = NativeCompletion.Default)
        {
            var o = IadWaitIdleOptions.Of(Known.OrNaN(target), Known.OrNaN(band), timeout, (int)completion);
            NativeErrors.Check(NativeMethods.iad_axis_wait_idle(_handle, ref o, out var st));
            return new NativeStatus(st);
        }

        /// <summary>
        /// Decel stop (Axis::stop), with or without Modbus control; blocks for the deceleration and returns the last
        /// status. With <paramref name="wait"/>, <see cref="MotionTimeoutException"/> when still moving after
        /// <paramref name="timeout"/> s. Safe to call from another thread while a move waits on this axis.
        /// </summary>
        public NativeStatus Stop(bool wait = false, double timeout = 5.0)
        {
            var o = IadStopOptions.Of(wait, timeout);
            NativeErrors.Check(NativeMethods.iad_axis_stop(_handle, ref o, out var st));
            return new NativeStatus(st);
        }

        /// <summary>Hold the move and keep the remaining travel (Axis::pause, STP ON). Safe from another thread.</summary>
        public void Pause() => NativeErrors.Check(NativeMethods.iad_axis_pause(_handle));

        /// <summary>Release the pause; a paused move continues by itself (Axis::resume, STP OFF).</summary>
        public void Resume() => NativeErrors.Check(NativeMethods.iad_axis_resume(_handle));

        /// <summary>
        /// Drop the remaining travel of a paused move and release the pause (Axis::cancel_paused_move). Refused (nothing
        /// sent) without an active pause (<see cref="EcNotReadyException"/>) or with a fault alarm
        /// (<see cref="EcAlarmException"/>). <paramref name="timeout"/> null = 5 s.
        /// </summary>
        public NativeStatus CancelPausedMove(double? timeout = null)
        {
            var o = IadCancelPausedMoveOptions.Of(Known.OrNaN(timeout));
            NativeErrors.Check(NativeMethods.iad_axis_cancel_paused_move(_handle, ref o, out var st));
            return new NativeStatus(st);
        }

        internal static IadMoveOptions MoveOptions(
            double speed,
            double accelG,
            double band,
            bool wait,
            double? timeout,
            bool jog,
            double? decelG = null,
            NativeAccelPattern accelPattern = NativeAccelPattern.Default,
            NativeCompletion completion = NativeCompletion.Default)
        {
            var o = SizedStruct.Fresh<IadMoveOptions>();
            NativeErrors.Check(jog ? NativeMethods.iad_jog_options_init(ref o) : NativeMethods.iad_move_options_init(ref o));
            o.Speed = speed;
            o.AccelG = accelG;
            o.Band = band;
            o.Wait = Known.Flag(wait);
            o.TimeoutS = Known.OrNaN(timeout);
            o.DecelG = Known.OrNaN(decelG);
            o.AccelPattern = (int)accelPattern;
            o.Completion = (int)completion;
            return o;
        }

        // ------------------------------------------------------------------------------------------ position table

        /// <summary>
        /// Write stored row <paramref name="row"/> (16 words with the checksum) and verify it (Axis::write_position_table).
        /// Non-volatile; at rest only. When the row already holds these words nothing is written
        /// (<see cref="NativePositionWriteResult.Skipped"/>) unless <paramref name="force"/>. Rows past max_positions,
        /// a row the EC's position mode does not run, unknown flag bits and role-bit changes are written with a warning
        /// (the result's warnings). <paramref name="allowNonvolatile"/> and <paramref name="allowUnsafe"/> are
        /// accepted and ignored (no longer needed).
        /// </summary>
        public NativePositionWriteResult WritePositionTable(int row, NativePositionEntry entry, bool allowNonvolatile = false, bool allowUnsafe = false, bool force = false)
        {
            if (entry is null)
            {
                throw new ArgumentNullException(nameof(entry));
            }

            var e = entry.ToNative();
            var o = NvOptions(allowNonvolatile, allowUnsafe, force);
            NativeErrors.Check(NativeMethods.iad_axis_write_position_table(_handle, row, ref e, ref o, out var result));
            return new NativePositionWriteResult(result, LastResultJsonOrEmpty());
        }

        /// <summary>
        /// Store the current position as row <paramref name="row"/> (Axis::teach_position). Non-volatile, as
        /// <see cref="WritePositionTable"/>; refused before homing and while moving (the flags are accepted and
        /// ignored).
        /// </summary>
        public NativePositionWriteResult TeachPosition(int row, bool allowNonvolatile = false, bool allowUnsafe = false, bool force = false)
        {
            var o = NvOptions(allowNonvolatile, allowUnsafe, force);
            NativeErrors.Check(NativeMethods.iad_axis_teach_position(_handle, row, (int)NativeTeachMethod.Table, ref o, out var result));
            return new NativePositionWriteResult(result, LastResultJsonOrEmpty());
        }

        private static IadNvWriteOptions NvOptions(bool allowNonvolatile, bool allowUnsafe, bool force)
        {
            var o = SizedStruct.Fresh<IadNvWriteOptions>();
            NativeErrors.Check(NativeMethods.iad_nv_write_options_init(ref o));
            o.AllowNonvolatile = Known.Flag(allowNonvolatile);
            o.AllowUnsafe = Known.Flag(allowUnsafe);
            o.Force = Known.Flag(force);
            return o;
        }

        // ------------------------------------------------------------------------------------------ parameters

        /// <summary>The unit's named parameters, decoded (Axis::parameters; EC: P1-P18, CON: No.x; READ-ONLY).</summary>
        public IReadOnlyList<NativeParameter> Parameters()
        {
            var list = new List<NativeParameter>();
            if (MiniJson.Parse(ParametersJson()) is List<object?> items)
            {
                foreach (var item in items)
                {
                    if (item is JsonObject p)
                    {
                        list.Add(new NativeParameter(p));
                    }
                }
            }

            return list;
        }

        /// <summary>The same as JSON: [to_json(ParameterInfo)...].</summary>
        public string ParametersJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_parameters_json(_handle, b, s, out r));

        /// <summary>One parameter (Axis::read_parameter; READ-ONLY). <paramref name="key"/>: "2", "P2", "No.35" or a raw item "8.15".</summary>
        public NativeParameter ReadParameter(string key)
        {
            var k = Utf8.EncodeRequired(key, nameof(key));
            NativeErrors.Check(NativeMethods.iad_axis_read_parameter(_handle, k, out var p));
            return new NativeParameter(p);
        }

        /// <summary>One parameter by its number (P-number on an EC, No. on a CON controller).</summary>
        public NativeParameter ReadParameter(int number) => ReadParameter(number.ToString(CultureInfo.InvariantCulture));

        /// <summary>One parameter as JSON, with the choices and the raw-item fields.</summary>
        public string ReadParameterJson(string key)
        {
            var k = Utf8.EncodeRequired(key, nameof(key));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_parameter_json(_handle, k, b, s, out r));
        }

        /// <summary>
        /// Write one parameter, verify it by read-back and (<paramref name="commit"/>) save it with PARW
        /// (Axis::write_parameter). <paramref name="value"/> as the PC tool shows it: a number ("12.5") or a label
        /// ("1:Forward", "disable"). Guards refuse before anything is written (<see cref="UnsafeCommandException"/>,
        /// <see cref="SoftLimitException"/>); at rest only. A home, dangerous or unlisted parameter is written with a
        /// warning: <paramref name="allowDangerous"/>, <paramref name="allowUnlisted"/> and
        /// <paramref name="allowHomeParams"/> are accepted and ignored (no longer needed). Non-volatile guard: an
        /// unchanged value is not written or committed (<see cref="NativeParameterWriteResult.Skipped"/>) unless
        /// <paramref name="force"/>. Applies after
        /// <see cref="SoftwareReset"/> or a power cycle.
        /// </summary>
        public NativeParameterWriteResult WriteParameter(
            string key,
            string value,
            bool commit = true,
            bool allowDangerous = false,
            bool allowUnlisted = false,
            bool allowHomeParams = false,
            bool force = false)
        {
            var k = Utf8.EncodeRequired(key, nameof(key));
            var v = Utf8.EncodeRequired(value, nameof(value));
            var o = ParamOptions(commit, allowDangerous, allowUnlisted, allowHomeParams, force);
            NativeErrors.Check(NativeMethods.iad_axis_write_parameter(_handle, k, v, ref o, out var result));
            return new NativeParameterWriteResult(result, LastResultJsonOrEmpty());
        }

        /// <summary>The same with a numeric value.</summary>
        public NativeParameterWriteResult WriteParameter(
            string key,
            double value,
            bool commit = true,
            bool allowDangerous = false,
            bool allowUnlisted = false,
            bool allowHomeParams = false,
            bool force = false)
        {
            var k = Utf8.EncodeRequired(key, nameof(key));
            var o = ParamOptions(commit, allowDangerous, allowUnlisted, allowHomeParams, force);
            NativeErrors.Check(NativeMethods.iad_axis_write_parameter_number(_handle, k, value, ref o, out var result));
            return new NativeParameterWriteResult(result, LastResultJsonOrEmpty());
        }

        /// <summary>The same by parameter number.</summary>
        public NativeParameterWriteResult WriteParameter(
            int number,
            double value,
            bool commit = true,
            bool allowDangerous = false,
            bool allowUnlisted = false,
            bool allowHomeParams = false,
            bool force = false) =>
            WriteParameter(number.ToString(CultureInfo.InvariantCulture), value, commit, allowDangerous, allowUnlisted, allowHomeParams, force);

        /// <summary>The same by parameter number, with a label or text value.</summary>
        public NativeParameterWriteResult WriteParameter(
            int number,
            string value,
            bool commit = true,
            bool allowDangerous = false,
            bool allowUnlisted = false,
            bool allowHomeParams = false,
            bool force = false) =>
            WriteParameter(number.ToString(CultureInfo.InvariantCulture), value, commit, allowDangerous, allowUnlisted, allowHomeParams, force);

        private static IadParamWriteOptions ParamOptions(bool commit, bool allowDangerous, bool allowUnlisted, bool allowHomeParams, bool force)
        {
            var o = SizedStruct.Fresh<IadParamWriteOptions>();
            NativeErrors.Check(NativeMethods.iad_param_write_options_init(ref o));
            o.Commit = Known.Flag(commit);
            o.AllowDangerous = Known.Flag(allowDangerous);
            o.AllowUnlisted = Known.Flag(allowUnlisted);
            o.AllowHomeParams = Known.Flag(allowHomeParams);
            o.Force = Known.Flag(force);
            return o;
        }

        // ------------------------------------------------------------------------------------------ non-volatile guard

        /// <summary>Non-volatile write cycles this axis handle caused (parameter commits + position-table writes).</summary>
        public long NonVolatileWrites
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_nonvolatile_writes(_handle, out var total, out _, out _));
                return total;
            }
        }

        /// <summary>The non-volatile write cycles of the last hour.</summary>
        public int NonVolatileWritesLastHour
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_nonvolatile_writes(_handle, out _, out var lastHour, out _));
                return lastHour;
            }
        }

        /// <summary>A <see cref="WriteParameter(string, string, bool, bool, bool, bool, bool)"/> without commit left a value unsaved.</summary>
        public bool HasUncommittedParameters
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_nonvolatile_writes(_handle, out _, out _, out var uncommitted));
                return Known.Bool(uncommitted);
            }
        }

        // ------------------------------------------------------------------------------------------ unit

        /// <summary>
        /// Change the unit's and the port's baud rate (Axis::set_baud_rate; volatile: until power-off). At rest only, and
        /// only when this axis is the only one on the bus. <see cref="SoftLimitException"/> (nothing sent) for a rate
        /// without an IAI code; an UNVERIFIED code is sent with a warning (<paramref name="allowUnverifiedCode"/> is
        /// accepted and ignored, no longer needed); <see cref="VerificationException"/> when the unit answers at
        /// another rate afterwards;
        /// <see cref="EcNotReadyException"/> on a transport without a baud rate or a port shared with other axes.
        /// </summary>
        public NativeBaudResult SetBaudRate(int rate, BaudChangeMethod method = BaudChangeMethod.Direct, double? settleS = null, bool allowUnverifiedCode = false)
        {
            var o = SizedStruct.Fresh<IadBaudOptions>();
            NativeErrors.Check(NativeMethods.iad_baud_options_init(ref o));
            o.Method = method == BaudChangeMethod.Break ? 1 : 0;
            o.SettleS = settleS ?? o.SettleS;
            o.AllowUnverifiedCode = Known.Flag(allowUnverifiedCode);
            NativeErrors.Check(NativeMethods.iad_axis_set_baud_rate(_handle, rate, ref o, out var result));
            return new NativeBaudResult(result, LastResultJsonOrEmpty());
        }

        /// <summary>
        /// Restart the unit's firmware and wait until it answers again (Axis::software_reset; at rest only, servo off
        /// first). Committed parameters apply afterwards.
        /// </summary>
        public NativeResetResult SoftwareReset(double? timeoutS = null, double? resetWaitS = null)
        {
            var o = SizedStruct.Fresh<IadResetOptions>();
            NativeErrors.Check(NativeMethods.iad_reset_options_init(ref o));
            o.TimeoutS = timeoutS ?? o.TimeoutS;
            o.ResetWaitS = resetWaitS ?? o.ResetWaitS;
            NativeErrors.Check(NativeMethods.iad_axis_software_reset(_handle, ref o, out var result));
            return new NativeResetResult(result, LastResultJsonOrEmpty());
        }

        /// <summary>
        /// The full result (warnings, notes) of the last successful parameter write, position-table write, teach, baud
        /// change or software reset on this axis, as JSON; no I/O. <see cref="EcNotReadyException"/> when there is none.
        /// (0.2) Also of <see cref="Push"/> (with wait), <see cref="EnableFast"/> / <see cref="DisableFast"/>,
        /// <see cref="CheckLink"/>, <see cref="Recover"/>, <see cref="SelfTest"/>, <see cref="ReadUnitInfoJson"/> and
        /// <see cref="BackupJson"/>.
        /// </summary>
        public string LastResultJson() =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_last_result_json(_handle, b, s, out r));

        /// <summary>
        /// The result the call that just ran on this thread kept (iad_last_result_json), "" if it cannot be read: the call is done
        /// and must not be reported as failed because its details are missing. Its own, not the handle's, which a call on
        /// another thread (a watchdog's link check) replaces.
        /// </summary>
        internal string LastResultJsonOrEmpty()
        {
            try
            {
                return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_last_result_json(b, s, out r));
            }
            catch (Exception e) when (e is EcDriverException || e is ArgumentException || e is InvalidOperationException || e is EntryPointNotFoundException)
            {
                return string.Empty;
            }
        }
    }
}
