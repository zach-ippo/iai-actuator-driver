using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // Blittable mirrors of the structs of iai_actuator_driver.h, field for field in header order. The header lays them out
    // without implicit padding (8-byte fields first, int32 fields in even numbers, text arrays in multiples of 8 bytes,
    // uint64 reserved slots at the end), so LayoutKind.Sequential gives the same offsets as C. iad_bool and iad_tribool
    // are int; text is a fixed byte array decoded as UTF-8 (never ByValTStr, which is ANSI). The sizes are checked by the
    // tests against the static_asserts of core/src/c_api.cpp. Reserved slots of inputs stay zero (default(T) and the init
    // functions zero them). (0.8) Every input is sized: StructSize and ReservedHeader first (the header's "Sized
    // structs"); an init function sets StructSize, a struct built here sets it with SizedStruct.Size<T>(); an
    // output is fixed for the ABI major.

    /// <summary>(0.8) The size of a sized input struct as this binding has it (its StructSize; an array's stride).</summary>
    internal static class SizedStruct
    {
        /// <summary>IAD_AXIS_NAME_SIZE: the bytes of an axis name with its NUL (0.8: 128).</summary>
        internal const int AxisNameSize = 128;

        internal static unsafe uint Size<T>()
            where T : unmanaged => (uint)sizeof(T);

        /// <summary>
        /// A zeroed <typeparamref name="T"/> with its StructSize set to its size (the first four bytes of every sized
        /// struct): what an <c>_init</c> function or an output call (iad_axis_get_options, iad_axis_read_position_table)
        /// is given, since the library writes only up to the struct_size it finds there (the header's "Sized structs").
        /// </summary>
        internal static unsafe T Fresh<T>()
            where T : unmanaged
        {
            var value = default(T);
            *(uint*)&value = (uint)sizeof(T);
            return value;
        }

        /// <summary>
        /// (0.8) The name a cell result or event belongs to: <paramref name="text"/> (the struct's axis field, cut at
        /// IAD_AXIS_NAME_SIZE - 1 bytes) when it is one of <paramref name="names"/>, else, for a name that did not fit,
        /// the name at <paramref name="index"/> (the axis's position in the requested list, or in the cell for an
        /// event); <paramref name="text"/> when there is no such position.
        /// </summary>
        internal static string NameOf(string text, int index, IReadOnlyList<string> names)
        {
            if (index < 0 || index >= names.Count)
            {
                return text;
            }

            for (var i = 0; i < names.Count; i++)
            {
                if (string.Equals(names[i], text, StringComparison.Ordinal))
                {
                    return text;
                }
            }

            return names[index];
        }
    }

    // ------------------------------------------------------------------------------------------ options (inputs)

    /// <summary>iad_axis_options (104 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadAxisOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double StrokeMin;
        public double StrokeMax;
        public double MaxSpeedMmS;
        public double MaxAccelG;
        public double MaxPushPct;
        public double PollIntervalS;
        public double SettleS;
        public int Method;
        public int AutoModbusControl;
        public double PushSettleS;   // 0.2
        public int Completion;       // 0.2
        public int Reserved0;
        public fixed ulong Reserved[2];
    }

    /// <summary>iad_open_options (72 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadOpenOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double ReadTimeoutS;
        public double MinGapS;
        public int Slave;
        public int BaudRate;
        public int Retries;
        public int Line;             // 0.2
        public double PositionUnitMm; // 0.2
        public int Fast;             // 0.2
        public int Reserved0;
        public fixed ulong Reserved[2];
    }

    /// <summary>iad_sim_options (80 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadSimOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeScale;
        public double PositionMm;
        public double StrokeMm;
        public double MaxSpeedMmS;
        public int BaudRate;
        public int Pmsl;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_move_options (80 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadMoveOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double Speed;
        public double AccelG;
        public double Band;
        public double TimeoutS;
        public int Wait;
        public int AccelPattern;     // 0.2
        public double DecelG;        // 0.2
        public int Completion;       // 0.2
        public int Reserved0;
        public fixed ulong Reserved[2];
    }

    /// <summary>iad_position_move_options (64 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadPositionMoveOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public double SpeedPct;
        public int Wait;
        public int AllowAboveCaps;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_nv_write_options (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadNvWriteOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public int AllowNonvolatile;
        public int Force;
        public int AllowUnsafe;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_param_write_options (64 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadParamWriteOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public int Commit;
        public int AllowDangerous;
        public int AllowUnlisted;
        public int AllowHomeParams;
        public int Force;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_baud_options (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadBaudOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double SettleS;
        public int Method;
        public int AllowUnverifiedCode;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_reset_options (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadResetOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public double ResetWaitS;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_cell_options (96 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadCellOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double ReadTimeoutS;
        public double MinGapS;
        public double ScanTimeoutS;
        public int MaxAxes;
        public int BaudRate;
        public int Retries;
        public int ScanFirst;
        public int ScanLast;
        public int Identify;
        public int StopOnFault;
        public int Line;             // 0.2
        public double PositionUnitMm; // 0.2
        public fixed ulong Reserved[3];
    }

    // (0.8) The options of the axis commands that took scalars before 0.8. Of() builds one with every field this binding
    // sets (the reserved slots zero), so no init call is needed; NaN = the default the header names.

    /// <summary>iad_servo_options (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadServoOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public int Wait;
        public int Reserved0;
        public fixed ulong Reserved[4];

        internal static IadServoOptions Of(bool wait, double timeoutS) =>
            new IadServoOptions { StructSize = SizedStruct.Size<IadServoOptions>(), TimeoutS = timeoutS, Wait = wait ? 1 : 0 };
    }

    /// <summary>iad_home_options (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadHomeOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public int Wait;
        public int Method;
        public fixed ulong Reserved[4];

        internal static IadHomeOptions Of(bool wait, double timeoutS, int method) =>
            new IadHomeOptions { StructSize = SizedStruct.Size<IadHomeOptions>(), TimeoutS = timeoutS, Wait = wait ? 1 : 0, Method = method };
    }

    /// <summary>iad_stop_options (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadStopOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public int Wait;
        public int Reserved0;
        public fixed ulong Reserved[4];

        internal static IadStopOptions Of(bool wait, double timeoutS) =>
            new IadStopOptions { StructSize = SizedStruct.Size<IadStopOptions>(), TimeoutS = timeoutS, Wait = wait ? 1 : 0 };
    }

    /// <summary>iad_wait_idle_options (72 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadWaitIdleOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double Target;
        public double Band;
        public double TimeoutS;
        public int Completion;
        public int Reserved0;
        public fixed ulong Reserved[4];

        internal static IadWaitIdleOptions Of(double target, double band, double timeoutS, int completion) =>
            new IadWaitIdleOptions
            {
                StructSize = SizedStruct.Size<IadWaitIdleOptions>(),
                Target = target,
                Band = band,
                TimeoutS = timeoutS,
                Completion = completion,
            };
    }

    /// <summary>iad_change_options (56 bytes; sized, 0.8): iad_axis_change_speed, iad_axis_change_accel.</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadChangeOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public int Wait;
        public int Reserved0;
        public fixed ulong Reserved[4];

        internal static IadChangeOptions Of(bool wait, double timeoutS) =>
            new IadChangeOptions { StructSize = SizedStruct.Size<IadChangeOptions>(), TimeoutS = timeoutS, Wait = wait ? 1 : 0 };
    }

    /// <summary>iad_cancel_paused_move_options (48 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadCancelPausedMoveOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double TimeoutS;
        public fixed ulong Reserved[4];

        internal static IadCancelPausedMoveOptions Of(double timeoutS) =>
            new IadCancelPausedMoveOptions { StructSize = SizedStruct.Size<IadCancelPausedMoveOptions>(), TimeoutS = timeoutS };
    }

    // ------------------------------------------------------------------------------------------ data (outputs)

    /// <summary>iad_status (208 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadStatus
    {
        public double PositionMm;
        public double VelocityMmS;
        public double CurrentMa;
        public double UptimeS;
        public int AlarmCode;
        public int AlarmLevel;
        public int ServoOn;
        public int Homed;
        public int InPosition;
        public int Moving;
        public int Alarm;
        public int Warning;
        public int ModbusEnabled;
        public int EmergencyStop;
        public int Paused;
        public int Homing;
        public int SafetySpeed;
        public int Ready;
        public int Idle;
        public int TargetReached;
        public int PushMissed;
        public int Pushing;
        public int EmgInput;
        public int DrivePowerCut;
        public int EnableOff;
        public int RawWords;
        public uint RawDss1;
        public uint RawDss2;
        public uint RawDsse;
        public uint RawDipm;
        public uint RawDopm;
        public int Rcon;             // 0.2
        public fixed ulong Reserved[8]; // 0.8: 64 bytes
    }

    /// <summary>iad_range (88 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadRange
    {
        public double Min;
        public double Max;
        public double UnitMax;
        public double Configured;
        public double RecommendedMin;
        public int Available;
        public int Count;
        public fixed byte Unit[8];
        public fixed ulong Reserved[4]; // 0.8: 32 bytes
    }

    /// <summary>iad_limits (1096 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadLimits
    {
        public IadRange Position;
        public IadRange Speed;
        public IadRange Accel;
        public IadRange Decel;
        public IadRange AccelDecel;
        public IadRange Push;
        public IadRange Band;
        public IadRange PushBand;
        public IadRange StoredRows;
        public IadRange JogStep;
        public IadRange SpeedOverride;
        public fixed byte Units[8];
        public fixed byte Kind[24];
        public fixed byte Family[24];
        public double PushSpeed;     // 0.2
        public fixed ulong Reserved[8]; // 0.8: 64 bytes
    }

    /// <summary>iad_identity (616 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadIdentity
    {
        public double SoftMinMm;
        public double SoftMaxMm;
        public double MaxSpeedMmS;
        public double MaxAccelG;
        public double MaxDecelG;
        public int Slave;
        public int SystemCode;
        public int UnitCode;
        public int RunStatus;
        public int AxisNumber;
        public int MaxPositions;
        public int ActuatorKind;
        public int UnitKnown;
        public int Rotary;
        public int Reserved0;
        public fixed byte ProductFamily[16];
        public fixed byte Model[64];
        public fixed byte ModelName[64];
        public fixed byte ControllerName[32];
        public fixed byte Controller[64];
        public fixed byte UnitName[48];
        public fixed byte SystemName[48];
        public fixed byte Firmware[8];
        public fixed byte SerialNumber[32];
        public fixed byte ActuatorSerial[32];
        public fixed byte UserName[48];
        public fixed byte Units[8];
        public fixed byte Kind[24];
        public fixed byte IdentitySource[16];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_position_entry (128 bytes; sized, 0.8). Input and output.</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadPositionEntry
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double PositionMm;
        public double BandMm;
        public double SpeedMmS;
        public double ZonePlusMm;
        public double ZoneMinusMm;
        public double AccelG;
        public double DecelG;
        public int PushPower;
        public int LoadThreshold;
        public int ControlFlags;
        public int Checksum;
        public int ChecksumOk;
        public int IsPush;
        public int Incremental;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_parameter (288 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadParameter
    {
        public double Value;
        public double Min;
        public double Max;
        public int Number;
        public int Writable;
        public int Dangerous;
        public int Reserved0;
        public fixed byte Name[64];
        public fixed byte Symbol[16];
        public fixed byte Item[16];
        public fixed byte Address[16];
        public fixed byte Text[64];
        public fixed byte Unit[16];
        public fixed byte Raw[16];
        public fixed byte Family[8];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_param_write_result (320 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadParamWriteResult
    {
        public int Number;
        public int Changed;
        public int Verified;
        public int Committed;
        public int CommitConfirmed;
        public int Saved;
        public int Skipped;
        public int Forced;
        public int ListedForModel;
        public int VerifiedForModel;
        public int HomeParameter;
        public int WarningCount;
        public fixed byte Key[16];
        public fixed byte Address[16];
        public fixed byte Name[64];
        public fixed byte Before[64];
        public fixed byte After[64];
        public fixed byte Unit[16];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_position_write_result (136 bytes; 0.8: the row read back in fields of its own, those of IadPositionEntry).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadPositionWriteResult
    {
        public double PositionMm;
        public double BandMm;
        public double SpeedMmS;
        public double ZonePlusMm;
        public double ZoneMinusMm;
        public double AccelG;
        public double DecelG;
        public int PushPower;
        public int LoadThreshold;
        public int ControlFlags;
        public int Checksum;
        public int ChecksumOk;
        public int IsPush;
        public int Incremental;
        public int Row;
        public int Skipped;
        public int Forced;
        public int WarningCount;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_reset_result (352 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadResetResult
    {
        public IadStatus Status;
        public double UptimeBeforeS;
        public double UptimeAfterS;
        public double ElapsedS;
        public int Accepted;
        public int ReplyLost;
        public int Confirmed;
        public int BaudRate;
        public int BaudChanged;
        public int NeedsHome;
        public fixed byte ConfirmedBy[64];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_baud_result (128 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadBaudResult
    {
        public int Previous;
        public int BaudRate;
        public int Method;
        public int Code;
        public int Verified;
        public int Probed;
        public int CodeVerified;
        public int Reserved0;
        public fixed byte Frame[64];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_cell_result (632 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadCellResult
    {
        public IadStatus Status;
        public int Code;

        /// <summary>(0.8) The axis's position in the requested list (all axes: the cell order).</summary>
        public int Index;
        public fixed byte Axis[128];
        public fixed byte Message[256];
        public fixed ulong Reserved[4];
    }

    // ------------------------------------------------------------------------------------------ 0.2

    /// <summary>iad_push_options (72 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadPushOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double Speed;
        public double AccelG;
        public double TimeoutS;
        public int Direction;
        public int Wait;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_push_result (272 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadPushResult
    {
        public IadStatus Status;
        public double PositionMm;
        public int Outcome;
        public int WarningCount;
        public fixed byte OutcomeName[16];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_sequence_point (56 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadSequencePoint
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double Position;
        public double Speed;
        public double AccelG;
        public double Band;
        public fixed ulong Reserved[2];
    }

    /// <summary>iad_sequence_options (88 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadSequenceOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double Speed;
        public double AccelG;
        public double Band;
        public double BlendBand;
        public double TimeoutS;
        public int Completion;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_move_info (296 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadMoveInfo
    {
        public IadStatus Result;
        public double Target;
        public double Band;
        public double TimeoutS;
        public double SentAtS;
        public double DeadlineS;
        public int Completion;
        public int HasResult;
        public int Cancelled;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_monitor_options (72 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadMonitorOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double GapS;
        public double LinkCheckS;
        public int Retries;
        public int Events;
        public int QueueCapacity;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_monitor_sample (304 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadMonitorSample
    {
        public IadStatus Status;
        public double TStartS;
        public double TS;
        public double AgeS;
        public double PollRateHz;
        public long Seq;
        public long Polls;
        public long ErrorCount;
        public int Running;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_event (736 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadEvent
    {
        public IadStatus Status;
        public double TimeS;
        public double WallTimeS;
        public long Sequence;
        public int Kind;
        public int Dropped;
        public fixed byte Axis[128];
        public fixed byte Name[32];
        public fixed byte Cause[32];
        public fixed byte Message[256];

        /// <summary>(0.6) IAD_EVENT_SOURCE_*: what queued it; 0 = not stated.</summary>
        public int Source;

        /// <summary>(0.8) A cell's event: the axis's position in the cell; IAD_INT_UNKNOWN otherwise.</summary>
        public int Index;

        /// <summary>(0.7) The OS id of the thread that queued it; 0 = not stated.</summary>
        public ulong CallerThread;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_fast_summary (304 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadFastSummary
    {
        public int BaudRate;
        public int Completion;
        public int Monitor;
        public int NoteCount;
        public fixed byte Notes[256];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_track_options (48 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadTrackOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public int AlarmHistory;
        public int Parameters;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_link_check (800 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadLinkCheck
    {
        public IadStatus Status;
        public double ElapsedS;
        public int Ok;
        public int ReasonCount;
        public fixed byte State[32];
        public fixed byte Reasons[256];
        public fixed byte Error[256];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_recovery_policy (80 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadRecoveryPolicy
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double PositionToleranceMm;
        public double PollS;
        public int BootBaudRate;
        public int RestoreBaudRate;
        public int RestoreControl;
        public int RestoreSafetySpeed;
        public int ClearMessageAlarms;
        public int CheckParameters;

        /// <summary>(0.6) Recover only a unit with a session (none tracked: IAD_ERR_RECOVERY).</summary>
        public int RequireSession;
        public int Reserved0;
        public fixed ulong Reserved[3];
    }

    /// <summary>iad_recovery_summary (768 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadRecoverySummary
    {
        public IadStatus Status;
        public double PositionMm;
        public double LastTargetMm;
        public double PositionErrorMm;
        public double ElapsedS;
        public int BaudRate;
        public int BaudRateFound;
        public int AlarmCode;
        public int Restarted;
        public int SameUnit;
        public int PortReopened;
        public int BaudRestored;
        public int ControlRestored;
        public int SafetySpeedRestored;
        public int NeedsHome;
        public int NeedsAttention;
        public int ReadyToMove;
        public int FaultAlarm;
        public int AtTarget;
        public int ServoOn;
        public int Homed;
        public int ModbusControl;
        public int SafetySpeed;
        public int NoteCount;

        /// <summary>(0.6) PC control was held before the fault and still is: nothing written.</summary>
        public int ControlKept;
        public fixed byte Axis[128];
        public fixed byte Cause[32];
        public fixed byte Summary[256];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_check_options (72 bytes; sized, 0.8).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadCheckOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double StepMm;
        public double SpeedMmS;
        public double AccelG;
        public int Motion;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_check_summary (56 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadCheckSummary
    {
        public int Passed;
        public int Items;
        public int Failed;
        public int Warned;
        public int Skipped;
        public int Reserved0;
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_axis_move (224 bytes; sized, 0.8: fields of its own, NaN or 0 = the default).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadAxisMove
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double Position;
        public double Speed;
        public double AccelG;
        public double Band;
        public double TimeoutS;
        public double DecelG;
        public int AccelPattern;
        public int Completion;
        public fixed byte Axis[128];
        public fixed ulong Reserved[4];
    }

    /// <summary>iad_motion_caps (336 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadMotionCaps
    {
        public double SpeedCap;
        public double AccelCap;
        public double AccelOnlyCap;
        public double DecelCap;
        public double PushCap;
        public double MinSpeed;
        public fixed byte MinSpeedSource[256];
        public fixed ulong Reserved[4];
    }
}
