using System;

namespace IaiActuatorDriver
{
    /// <summary>Severity of a <see cref="DriverLog"/> message.</summary>
    public enum DriverLogLevel
    {
        Debug,
        Info,
        Warning,
        Error,
    }

    /// <summary>
    /// Diagnostic log of the driver. Subscribe to <see cref="Message"/> to receive the driver's debug / info /
    /// warning / error lines; with no subscriber nothing is logged. A subscriber that throws is ignored: logging must
    /// never break communication.
    /// </summary>
    public static class DriverLog
    {
        /// <summary>Raised for every log line: (level, source, message, exception or null).</summary>
        public static event Action<DriverLogLevel, string, string, Exception?>? Message;

        /// <summary>Whether anything subscribed to <see cref="Message"/> (the core's lines are not decoded otherwise).</summary>
        internal static bool HasListeners => Message != null;

        internal static void Write(DriverLogLevel level, string source, string message, Exception? exception = null)
        {
            var handler = Message;
            if (handler is null)
            {
                return;
            }

            try
            {
                handler(level, source, message, exception);
            }
            catch
            {
                // a broken log subscriber must never break communication
            }
        }

        internal static void Debug(string source, string message) => Write(DriverLogLevel.Debug, source, message);

        internal static void Info(string source, string message) => Write(DriverLogLevel.Info, source, message);

        internal static void Warning(string source, string message) => Write(DriverLogLevel.Warning, source, message);

        internal static void Error(string source, string message, Exception? exception = null) =>
            Write(DriverLogLevel.Error, source, message, exception);
    }
}
