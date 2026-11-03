using System;
using System.Collections.Generic;
using System.Globalization;

namespace IaiActuatorDriver
{
    // Exception hierarchy (the Python edition's names in brackets):
    //
    //   EcDriverException [EcDriverError]                every error raised on purpose by this library
    //   +- ModbusException [ModbusError]                 a Modbus transaction failed
    //   |  +- ModbusTimeoutException [ModbusTimeout]     no (complete) response within the timeout, after all retries
    //   |  +- ModbusCrcException [ModbusCRCError]        a response with a bad CRC, after all retries
    //   |  +- ModbusFrameException [ModbusFrameError]    a response that does not match the request
    //   |  +- ModbusSlaveException [ModbusException]     the slave answered with an exception response (fc | 0x80)
    //   |  +- IaiCommandRejectedException [IaiCommandRejected]  0x45 / 0x48 answered with a rejection code
    //   +- UnsafeCommandException [UnsafeCommandError]   refused to send a dangerous command; nothing was sent
    //   +- RecoveryException [RecoveryError]             a recovery could not bring the axis back (native core)
    //   |  +- UnitReplacedException [UnitReplacedError]  another unit answers where the tracked one was
    //   +- EcException [EcError]                         the actuator cannot do what was asked
    //      +- SoftLimitException, EcNotReadyException, EcAlarmException, MotionTimeoutException,
    //         MotionIncompleteException, NonVolatileWriteException, VerificationException

    /// <summary>Base class of every error this library raises on purpose, so callers can catch one type.</summary>
    public class EcDriverException : Exception
    {
        public EcDriverException(string message)
            : base(message)
        {
        }

        public EcDriverException(string message, Exception? inner)
            : base(message, inner)
        {
        }

        private readonly List<string> _notes = new List<string>();

        /// <summary>Notes the driver added while handling the error (e.g. "decel stop sent after this error").</summary>
        public IReadOnlyList<string> Notes
        {
            get
            {
                lock (_notes)
                {
                    return _notes.ToArray();
                }
            }
        }

        /// <summary>The axis class already sent its decel stop for this error (so an outer handler does not stop twice).</summary>
        public bool StopAttempted { get; internal set; }

        /// <inheritdoc />
        public override string Message
        {
            get
            {
                var notes = Notes;
                return notes.Count == 0 ? base.Message : base.Message + " [" + string.Join("; ", notes) + "]";
            }
        }

        internal void AddNote(string note)
        {
            lock (_notes)
            {
                _notes.Add(note);
            }
        }
    }

    // ------------------------------------------------------------------------------------------ Modbus / link layer

    /// <summary>A Modbus transaction failed (link layer). Timeouts, CRC and frame errors are retried before they are raised.</summary>
    public class ModbusException : EcDriverException
    {
        public ModbusException(string message)
            : base(message)
        {
        }

        public ModbusException(string message, Exception? inner)
            : base(message, inner)
        {
        }

        /// <summary>How many attempts were made before this error was raised (1 = no retry).</summary>
        public int Attempts { get; internal set; } = 1;

        /// <inheritdoc />
        public override string Message =>
            Attempts > 1 ? $"{base.Message} (gave up after {Attempts} attempts)" : base.Message;
    }

    /// <summary>No (complete) response arrived within the timeout, after all retries.</summary>
    public class ModbusTimeoutException : ModbusException
    {
        public ModbusTimeoutException(string message)
            : base(message)
        {
        }
    }

    /// <summary>A response arrived but its CRC did not match, after all retries.</summary>
    public class ModbusCrcException : ModbusException
    {
        public ModbusCrcException(string message)
            : base(message)
        {
        }
    }

    /// <summary>A response arrived but did not match the request (wrong slave, function or echo).</summary>
    public class ModbusFrameException : ModbusException
    {
        public ModbusFrameException(string message)
            : base(message)
        {
        }
    }

    /// <summary>
    /// The slave answered with a Modbus exception response (function code | 0x80). A valid answer: never retried.
    /// </summary>
    public class ModbusSlaveException : ModbusException
    {
        /// <summary>Modbus exception code names (DOC).</summary>
        public static readonly IReadOnlyDictionary<int, string> ExceptionNames = new Dictionary<int, string>
        {
            [0x01] = "Illegal Function",
            [0x02] = "Illegal Data Address",
            [0x03] = "Illegal Data Value",
            [0x04] = "Slave Device Failure",
        };

        public ModbusSlaveException(int code, int function, string? message = null)
            : base(message ?? DefaultMessage(code, function))
        {
            Code = code;
            Function = function;
        }

        /// <summary>Exception code (1 = illegal function, 2 = illegal address, 3 = illegal value, 4 = failure).</summary>
        public int Code { get; }

        /// <summary>Function code of the request that was rejected.</summary>
        public int Function { get; }

        /// <summary>Name of <see cref="Code"/>.</summary>
        public string CodeName => ExceptionNames.TryGetValue(Code, out var name) ? name : "Unknown exception";

        private static string DefaultMessage(int code, int function)
        {
            var name = ExceptionNames.TryGetValue(code, out var n) ? n : "Unknown exception";
            return string.Format(CultureInfo.InvariantCulture, "Modbus exception {0:X2}h ({1}) for function {2:X2}h", code, name, function);
        }
    }

    /// <summary>
    /// An IAI control command (function 0x45 / 0x48) was answered with a non-zero rejection code. The unit understood
    /// the frame and refused the command: it is never retried. <see cref="Reason"/> names the code (0x11 "1st operand
    /// error" is VERIFIED; the other names are UNVERIFIED), <see cref="Hint"/> is the driver's short advice ("" if none).
    /// Tables: <see cref="Registers.IAI_REJECTIONS"/> / <see cref="Registers.IAI2_REJECTIONS"/>.
    /// </summary>
    public class IaiCommandRejectedException : ModbusException
    {
        public IaiCommandRejectedException(int function, int command, int rejection, int? statusByte = null, IReadOnlyList<uint>? statusWords = null)
            : base(BuildMessage(function, command, rejection))
        {
            Function = function;
            Command = command;
            Rejection = rejection;
            StatusByte = statusByte;
            StatusWords = statusWords;
            Reason = Registers.IaiRejectionName(function, rejection);
            Hint = Registers.IaiRejectionHint(function, rejection, command);
        }

        /// <summary>0x45 or 0x48.</summary>
        public int Function { get; }

        /// <summary>The command code that was rejected.</summary>
        public int Command { get; }

        /// <summary>Rejection code: 0x45 one byte (0x01-0x3F), 0x48 one word (0x1000-0x13FF).</summary>
        public int Rejection { get; }

        /// <summary>0x45: the status byte of the reply.</summary>
        public int? StatusByte { get; }

        /// <summary>0x48: the four 32-bit status words of the reply.</summary>
        public IReadOnlyList<uint>? StatusWords { get; }

        /// <summary>Name of the rejection code ("unknown rejection code" when not in the table).</summary>
        public string Reason { get; }

        /// <summary>The driver's short advice ("" when there is none).</summary>
        public string Hint { get; }

        private static string BuildMessage(int function, int command, int rejection)
        {
            var reason = Registers.IaiRejectionName(function, rejection);
            var hint = Registers.IaiRejectionHint(function, rejection, command);
            var width = function == 0x48 ? 4 : 2;
            var text = string.Format(
                CultureInfo.InvariantCulture,
                "IAI command 0x{0} (function {1:X2}h) rejected with code 0x{2} ({3})",
                command.ToString("x2", CultureInfo.InvariantCulture),
                function,
                rejection.ToString("x" + width, CultureInfo.InvariantCulture),
                reason);
            return hint.Length > 0 ? text + ": " + hint : text;
        }
    }

    /// <summary>
    /// Refused to send a command that could damage configuration (baud / axis change, initialisation, non-volatile
    /// write...). Nothing was sent. Commands on the never-send lists need <c>allowNeverSend</c>; <c>allowUnsafe</c>
    /// alone is not enough for them.
    /// </summary>
    public class UnsafeCommandException : EcDriverException
    {
        public UnsafeCommandException(string message)
            : base(message)
        {
        }
    }

    /// <summary>
    /// (native core, ABI 0.2) A recovery could not bring the axis back: the port or the unit did not come back in time,
    /// or its identity could not be confirmed (IAD_ERR_RECOVERY; the Python edition's RecoveryError). Nothing was moved.
    /// The report so far is in <see cref="Native.NativeErrors.ErrorJsonOf"/> under "report".
    /// </summary>
    public class RecoveryException : EcDriverException
    {
        public RecoveryException(string message)
            : base(message)
        {
        }

        /// <summary>What the recovery had found and done when it failed (<see cref="Axis.Recover"/> sets it); <c>null</c> when not known.</summary>
        public RecoveryReport? Report { get; internal set; }
    }

    /// <summary>
    /// (native core, ABI 0.2) Another unit answers where the tracked one was (IAD_ERR_UNIT_REPLACED; UnitReplacedError):
    /// nothing was written to it. "expected", "found" and "report" are in <see cref="Native.NativeErrors.ErrorJsonOf"/>.
    /// </summary>
    public sealed class UnitReplacedException : RecoveryException
    {
        public UnitReplacedException(string message)
            : base(message)
        {
        }
    }

    // ------------------------------------------------------------------------------------------ actuator layer

    /// <summary>The actuator cannot do what was asked.</summary>
    public class EcException : EcDriverException
    {
        public EcException(string message)
            : base(message)
        {
        }
    }

    /// <summary>A target or motion parameter is outside the configured soft limits / caps. Nothing was sent.</summary>
    public class SoftLimitException : EcException
    {
        public SoftLimitException(string message)
            : base(message)
        {
        }
    }

    /// <summary>A precondition for the command is not met (servo off, not homed, Modbus control disabled...).</summary>
    public class EcNotReadyException : EcException
    {
        public EcNotReadyException(string message)
            : base(message)
        {
        }
    }

    /// <summary>The actuator reports an alarm.</summary>
    public class EcAlarmException : EcException
    {
        public EcAlarmException(int alarmCode, string? message = null)
            : base(message ?? "actuator alarm 0x" + alarmCode.ToString("x3", CultureInfo.InvariantCulture))
        {
            AlarmCode = alarmCode;
        }

        /// <summary>The alarm code (ALMC).</summary>
        public int AlarmCode { get; }
    }

    /// <summary>Motion did not complete in time. A decel stop was requested before this was raised.</summary>
    public class MotionTimeoutException : EcException
    {
        public MotionTimeoutException(string message)
            : base(message)
        {
        }
    }

    /// <summary>
    /// The axis settled (in position, not moving) away from the commanded target: a STOP from another master, the unit
    /// ignoring the command (STOP held, PIO in control) or the unit clamping the target to its own soft limit. Also
    /// thrown at once ("stopped from outside this wait", no STOP of its own) when <c>Stop</c> of the same axis from
    /// another thread ends a move wait.
    /// </summary>
    public class MotionIncompleteException : EcException
    {
        public MotionIncompleteException(string message)
            : base(message)
        {
        }
    }

    /// <summary>A write to non-volatile memory (position table) was refused (no permission, or a protected row).</summary>
    public class NonVolatileWriteException : EcException
    {
        public NonVolatileWriteException(string message)
            : base(message)
        {
        }
    }

    /// <summary>Read-back after a write did not match what was written.</summary>
    public class VerificationException : EcException
    {
        public VerificationException(string message)
            : base(message)
        {
        }
    }
}
