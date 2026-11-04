using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver.Native
{
    /// <summary>
    /// Result codes of the C ABI (enum iad_result_code in iai_actuator_driver.h). 0 is success, every error is negative.
    /// Codes are never renumbered; a new kind gets a new code in a minor ABI version.
    /// </summary>
    public enum NativeResult
    {
        /// <summary>IAD_OK.</summary>
        Ok = 0,

        /// <summary>IAD_ERR_SOFT_LIMIT: a value outside the unit's range or a cap; nothing sent.</summary>
        SoftLimit = -1,

        /// <summary>IAD_ERR_NOT_READY: servo off, not homed, moving, no Modbus control...</summary>
        NotReady = -2,

        /// <summary>IAD_ERR_ALARM: the unit reports an alarm.</summary>
        Alarm = -3,

        /// <summary>IAD_ERR_MOTION_TIMEOUT: motion did not finish in time (a decel stop was sent).</summary>
        MotionTimeout = -4,

        /// <summary>IAD_ERR_MODBUS_TIMEOUT: no (complete) answer after all retries.</summary>
        ModbusTimeout = -5,

        /// <summary>IAD_ERR_FRAME: bad CRC or a mismatched answer.</summary>
        Frame = -6,

        /// <summary>IAD_ERR_TRANSPORT: the port failed (busy, absent, USB adapter pulled).</summary>
        Transport = -7,

        /// <summary>IAD_ERR_VERIFICATION: the read-back after a write did not match.</summary>
        Verification = -8,

        /// <summary>IAD_ERR_NONVOLATILE_REFUSED: a protected row (simple data, unaddressable).</summary>
        NonvolatileRefused = -9,

        /// <summary>IAD_ERR_INVALID_ARGUMENT: a NULL pointer, a closed handle, an unknown name or key; nothing sent.</summary>
        InvalidArgument = -10,

        /// <summary>IAD_ERR_UNSUPPORTED: not available in this build or on this handle.</summary>
        Unsupported = -11,

        /// <summary>IAD_ERR_INTERNAL: an unexpected error in the library (a bug).</summary>
        Internal = -12,

        /// <summary>IAD_ERR_BUFFER_TOO_SMALL: a text output did not fit (handled inside the binding).</summary>
        BufferTooSmall = -13,

        /// <summary>IAD_ERR_SERVO_BLOCKED: EMG input, drive power cut or enable switch released (a not-ready kind).</summary>
        ServoBlocked = -14,

        /// <summary>IAD_ERR_MOTION_INCOMPLETE: settled away from the target.</summary>
        MotionIncomplete = -15,

        /// <summary>IAD_ERR_SLAVE_EXCEPTION: the unit answered with a Modbus exception response.</summary>
        SlaveException = -16,

        /// <summary>IAD_ERR_COMMAND_REJECTED: an IAI 0x45 command answered with a rejection code.</summary>
        CommandRejected = -17,

        /// <summary>IAD_ERR_UNSAFE_REFUSED: a raw IAI command off the safe list or on the never-send list; a
        /// read-only, unconfirmed, not-offered or manual-rule refused parameter; a class that is not a parameter
        /// class.</summary>
        UnsafeRefused = -18,

        /// <summary>IAD_ERR_CELL: one or more axes of a group command failed.</summary>
        Cell = -19,

        /// <summary>IAD_ERR_MODBUS: any other Modbus error.</summary>
        Modbus = -20,

        /// <summary>IAD_ERR_DRIVER: any other driver error.</summary>
        Driver = -21,

        /// <summary>(0.2) IAD_ERR_RECOVERY: the port / unit did not come back in time, or the identity could not be confirmed.</summary>
        Recovery = -22,

        /// <summary>(0.2) IAD_ERR_UNIT_REPLACED: another unit answers where the tracked one was; nothing written.</summary>
        UnitReplaced = -23,

        /// <summary>(0.2) IAD_ERR_FILE: a file could not be read or written (a backup file, a backup directory).</summary>
        File = -24,

        /// <summary>(0.2) IAD_ERR_CANCELLED: a blocking call was ended by a cancel from another thread.</summary>
        Cancelled = -25,
    }

    /// <summary>
    /// How the interop layer turns a failed call of the native core into an exception: the library's own exception
    /// classes, so code written against <c>IaiActuatorDriver</c> catches the same types (docs/dev/NATIVE_ABI.md,
    /// "Results and errors"). Public for diagnostics (<see cref="ResultOf"/>, <see cref="ErrorJsonOf"/>):
    /// </summary>
    /// <remarks>
    /// <code>
    /// SoftLimit          SoftLimitException             NotReady, ServoBlocked  EcNotReadyException
    /// Alarm              EcAlarmException (AlarmCode)   MotionTimeout           MotionTimeoutException
    /// ModbusTimeout      ModbusTimeoutException         Frame                   ModbusCrcException / ModbusFrameException
    /// Transport, Modbus  ModbusException                Verification            VerificationException
    /// NonvolatileRefused NonVolatileWriteException      InvalidArgument         ArgumentException (kind
    ///                                                                           "std::out_of_range": ArgumentOutOfRangeException)
    /// Unsupported        NotSupportedException          Internal                InvalidOperationException
    /// MotionIncomplete   MotionIncompleteException      SlaveException          ModbusSlaveException
    /// CommandRejected    IaiCommandRejectedException    UnsafeRefused           UnsafeCommandException
    /// Cell               CellException                  Driver                  EcException / EcDriverException
    /// Recovery           RecoveryException              UnitReplaced            UnitReplacedException
    /// File               System.IO.IOException          Cancelled               OperationCanceledException
    /// </code>
    /// There is no servo-blocked or transport class yet, so those use the nearest base class; an unknown negative code
    /// (a newer library) gives <see cref="EcDriverException"/>. The message is the library's (iad_last_error_message:
    /// the C++ what(), notes included; with both notes and a retry count the exception classes render the two in their
    /// own order). Every exception carries the native details in <see cref="Exception.Data"/> under
    /// <see cref="ResultKey"/>, <see cref="NameKey"/>, <see cref="KindKey"/> and <see cref="JsonKey"/>; a driver
    /// exception also gets <c>Notes</c>, <c>StopAttempted</c> and (Modbus) <c>Attempts</c> from iad_last_error_json,
    /// set directly (the interop layer is part of this assembly).
    /// </remarks>
    public static class NativeErrors
    {
        /// <summary><see cref="Exception.Data"/> key: the <see cref="NativeResult"/> code (as int).</summary>
        public const string ResultKey = "IaiActuatorDriver.Native.Result";

        /// <summary><see cref="Exception.Data"/> key: the code's name, e.g. "IAD_ERR_SOFT_LIMIT".</summary>
        public const string NameKey = "IaiActuatorDriver.Native.Name";

        /// <summary><see cref="Exception.Data"/> key: the C++ class, e.g. "SoftLimitError" ("" when not reported).</summary>
        public const string KindKey = "IaiActuatorDriver.Native.Kind";

        /// <summary><see cref="Exception.Data"/> key: iad_last_error_json of the failed call ("" when not available).</summary>
        public const string JsonKey = "IaiActuatorDriver.Native.Json";

        /// <summary>The native result code of an exception this binding threw, <c>null</c> for any other exception.</summary>
        public static NativeResult? ResultOf(Exception exception) =>
            exception?.Data[ResultKey] is int code ? (NativeResult)code : (NativeResult?)null;

        /// <summary>The iad_last_error_json of an exception this binding threw, <c>null</c> for any other exception.</summary>
        public static string? ErrorJsonOf(Exception exception) => exception?.Data[JsonKey] as string;

        /// <summary>Name of a result code as the library spells it ("IAD_ERR_UNKNOWN" for another value). No library call.</summary>
        public static string Name(NativeResult code) => Name((int)code);

        internal static string Name(int code)
        {
            switch ((NativeResult)code)
            {
                case NativeResult.Ok: return "IAD_OK";
                case NativeResult.SoftLimit: return "IAD_ERR_SOFT_LIMIT";
                case NativeResult.NotReady: return "IAD_ERR_NOT_READY";
                case NativeResult.Alarm: return "IAD_ERR_ALARM";
                case NativeResult.MotionTimeout: return "IAD_ERR_MOTION_TIMEOUT";
                case NativeResult.ModbusTimeout: return "IAD_ERR_MODBUS_TIMEOUT";
                case NativeResult.Frame: return "IAD_ERR_FRAME";
                case NativeResult.Transport: return "IAD_ERR_TRANSPORT";
                case NativeResult.Verification: return "IAD_ERR_VERIFICATION";
                case NativeResult.NonvolatileRefused: return "IAD_ERR_NONVOLATILE_REFUSED";
                case NativeResult.InvalidArgument: return "IAD_ERR_INVALID_ARGUMENT";
                case NativeResult.Unsupported: return "IAD_ERR_UNSUPPORTED";
                case NativeResult.Internal: return "IAD_ERR_INTERNAL";
                case NativeResult.BufferTooSmall: return "IAD_ERR_BUFFER_TOO_SMALL";
                case NativeResult.ServoBlocked: return "IAD_ERR_SERVO_BLOCKED";
                case NativeResult.MotionIncomplete: return "IAD_ERR_MOTION_INCOMPLETE";
                case NativeResult.SlaveException: return "IAD_ERR_SLAVE_EXCEPTION";
                case NativeResult.CommandRejected: return "IAD_ERR_COMMAND_REJECTED";
                case NativeResult.UnsafeRefused: return "IAD_ERR_UNSAFE_REFUSED";
                case NativeResult.Cell: return "IAD_ERR_CELL";
                case NativeResult.Modbus: return "IAD_ERR_MODBUS";
                case NativeResult.Driver: return "IAD_ERR_DRIVER";
                case NativeResult.Recovery: return "IAD_ERR_RECOVERY";
                case NativeResult.UnitReplaced: return "IAD_ERR_UNIT_REPLACED";
                case NativeResult.File: return "IAD_ERR_FILE";
                case NativeResult.Cancelled: return "IAD_ERR_CANCELLED";
                default: return "IAD_ERR_UNKNOWN";
            }
        }

        // ------------------------------------------------------------------------------------------ raising

        /// <summary>Throw the exception of a failed call (no-op for IAD_OK). Call it right after the call, on the same thread.</summary>
        internal static void Check(int result)
        {
            if (result != 0)
            {
                throw Capture(result);
            }
        }

        /// <summary>
        /// The exception of a failed call, built from this thread's last error. Nothing may run on this thread between the
        /// call and this (the last error is per thread and the next library call replaces it).
        /// </summary>
        internal static Exception Capture(int result)
        {
            string message;
            string? json;
            try
            {
                message = Utf8.FromPointer(NativeMethods.iad_last_error_message());
                json = LastErrorJson();
            }
            catch (EntryPointNotFoundException)
            {
                message = string.Empty;
                json = null;
            }

            return Create(result, message, json);
        }

        /// <summary>iad_last_error_json of this thread, <c>null</c> when the library cannot give it.</summary>
        internal static string? LastErrorJson()
        {
            try
            {
                var buffer = new byte[4096];
                var r = NativeMethods.iad_last_error_json(buffer, NativeText.Size(buffer.Length), out var required);
                if (r == (int)NativeResult.BufferTooSmall && required.ToUInt64() > (ulong)buffer.Length && required.ToUInt64() < int.MaxValue)
                {
                    buffer = new byte[NativeText.ToInt(required)];
                    r = NativeMethods.iad_last_error_json(buffer, NativeText.Size(buffer.Length), out required);
                }

                return r == 0 ? Utf8.FromBuffer(buffer) : null;
            }
            catch (EntryPointNotFoundException)
            {
                return null;
            }
        }

        /// <summary>
        /// The exception for <paramref name="code"/> with the library's <paramref name="message"/> and last-error
        /// <paramref name="json"/> (either may be empty). Pure: no library call.
        /// </summary>
        internal static Exception Create(int code, string? message, string? json) =>
            Create(code, message, MiniJson.TryParse(json) as JsonObject, json);

        /// <summary>
        /// The exception of an error the core describes inside another output (a cell's per-axis or per-port error) in the
        /// form of iad_last_error_json: {"code", "kind", "message", "notes", ...}. Pure: no library call.
        /// </summary>
        internal static Exception FromDetails(JsonObject details) =>
            Create(details.Int("code") ?? (int)NativeResult.Driver, null, details, null);

        private static Exception Create(int code, string? message, JsonObject? details, string? json)
        {
            var text = NameMap.Managed(!string.IsNullOrEmpty(message) ? message! : details?.String("message") ?? string.Empty);
            if (text.Length == 0)
            {
                text = Name(code);
            }

            var notes = (details?.Strings("notes") ?? Array.Empty<string>()).Select(NameMap.Managed).ToList();
            var attempts = details?.Int("attempts") ?? 1;
            var kind = details?.String("kind") ?? string.Empty;
            var result = (NativeResult)code;

            // The library's message ends with the attempts suffix (Modbus kinds) and the notes. Split them off so the
            // managed fields (Attempts, Notes) hold them and Message renders them again (the managed classes put the
            // notes before the attempts; with only one of the two the text is the library's). When the text does not
            // end that way, keep it whole and leave those fields alone (no doubled notes).
            var modbusKind = result is NativeResult.ModbusTimeout or NativeResult.Frame or NativeResult.Transport
                or NativeResult.SlaveException or NativeResult.CommandRejected or NativeResult.Modbus;
            var suffix = (modbusKind && attempts > 1 ? string.Format(CultureInfo.InvariantCulture, " (gave up after {0} attempts)", attempts) : string.Empty)
                + (notes.Count > 0 ? " [" + string.Join("; ", notes) + "]" : string.Empty);
            var split = text.EndsWith(suffix, StringComparison.Ordinal);
            var core = split ? text.Substring(0, text.Length - suffix.Length) : text;

            var ownMessage = false;
            Exception exception;
            switch (result)
            {
                case NativeResult.SoftLimit:
                    exception = new SoftLimitException(core);
                    break;
                case NativeResult.NotReady:
                case NativeResult.ServoBlocked:
                    exception = new EcNotReadyException(core);
                    break;
                case NativeResult.Alarm:
                    exception = new EcAlarmException(details?.Int("alarm_code") ?? 0, core);
                    break;
                case NativeResult.MotionTimeout:
                    exception = new MotionTimeoutException(core);
                    break;
                case NativeResult.ModbusTimeout:
                    exception = new ModbusTimeoutException(core);
                    break;
                case NativeResult.Frame:
                    exception = kind == "ModbusCrcError" ? new ModbusCrcException(core) : new ModbusFrameException(core);
                    break;
                case NativeResult.Transport:
                case NativeResult.Modbus:
                    exception = new ModbusException(core);
                    break;
                case NativeResult.Verification:
                    exception = new VerificationException(core);
                    break;
                case NativeResult.NonvolatileRefused:
                    exception = new NonVolatileWriteException(core);
                    break;
                case NativeResult.InvalidArgument:
                    // (0.3) a value outside its range or a name / index that is not there: the .NET out-of-range type
                    exception = kind == "std::out_of_range" ? new ArgumentOutOfRangeException(null, text) : new ArgumentException(text);
                    break;
                case NativeResult.Unsupported:
                    exception = new NotSupportedException(text);
                    break;
                case NativeResult.Internal:
                case NativeResult.BufferTooSmall:
                    exception = new InvalidOperationException(text);
                    break;
                case NativeResult.MotionIncomplete:
                    exception = new MotionIncompleteException(core);
                    break;
                case NativeResult.SlaveException:
                    exception = new ModbusSlaveException(details?.Int("exception_code") ?? 0, details?.Int("function") ?? 0, core);
                    break;
                case NativeResult.CommandRejected:
                    if (details?.Int("function") is int function && details.Int("command") is int command && details.Int("rejection") is int rejection)
                    {
                        // The managed class builds its message, reason and hint from its own tables.
                        exception = new IaiCommandRejectedException(function, command, rejection, details.Int("status_byte") is int sb && sb >= 0 ? sb : (int?)null);
                        ownMessage = true;
                    }
                    else
                    {
                        exception = new ModbusException(core);
                    }

                    break;
                case NativeResult.UnsafeRefused:
                    exception = new UnsafeCommandException(core);
                    break;
                case NativeResult.Cell:
                    exception = CellFromJson(details, text);
                    ownMessage = true;
                    break;
                case NativeResult.Driver:
                    exception = kind == "EcError" ? new EcException(core) : new EcDriverException(core);
                    break;
                case NativeResult.Recovery:
                    exception = new RecoveryException(core);
                    break;
                case NativeResult.UnitReplaced:
                    exception = new UnitReplacedException(core);
                    break;
                case NativeResult.File:
                    exception = new System.IO.IOException(text);
                    break;
                case NativeResult.Cancelled:
                    exception = new OperationCanceledException(text);
                    break;
                default:
                    exception = new EcDriverException(core);
                    break;
            }

            if (exception is EcDriverException driver)
            {
                if (split || ownMessage)
                {
                    if (driver is ModbusException modbus && attempts > 1)
                    {
                        modbus.Attempts = attempts;
                    }

                    foreach (var note in notes)
                    {
                        driver.AddNote(note);
                    }
                }

                if (details?.Bool("stop_attempted") == true)
                {
                    driver.StopAttempted = true;
                }
            }

            Tag(exception, code, kind, json);
            return exception;
        }

        /// <summary>Attach the native details to <paramref name="exception"/>'s Data.</summary>
        internal static void Tag(Exception exception, int code, string kind, string? json)
        {
            exception.Data[ResultKey] = code;
            exception.Data[NameKey] = Name(code);
            exception.Data[KindKey] = kind;
            exception.Data[JsonKey] = json ?? string.Empty;
        }

        /// <summary>A <see cref="CellException"/> from the "axes" ({name: {code, message}}) of the last-error JSON.</summary>
        private static CellException CellFromJson(JsonObject? details, string message)
        {
            var errors = new Dictionary<string, Exception>(StringComparer.Ordinal);
            if (details?.Object("axes") is { } axes)
            {
                foreach (var name in axes.Keys)
                {
                    var axis = axes.Object(name);
                    errors[name] = axis is null ? Create((int)NativeResult.Driver, null, null) : FromDetails(axis);
                }
            }

            var cell = new CellException(details?.String("operation") ?? "cell", errors, new Dictionary<string, object?>());
            if (errors.Count == 0)
            {
                cell.AddNote(message);
            }

            return cell;
        }
    }
}
