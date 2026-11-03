using System;
using System.Collections.Generic;
using System.Linq;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// Collects the <see cref="DriverLog"/> lines raised on the creating thread (other tests run in parallel) at or above
    /// <see cref="DriverLogLevel.Warning"/> unless another level is given. Dispose to unsubscribe.
    /// </summary>
    internal sealed class LogCapture : IDisposable
    {
        private readonly object _sync = new object();
        private readonly List<(DriverLogLevel Level, string Source, string Message)> _lines = new List<(DriverLogLevel, string, string)>();
        private readonly int _thread = Environment.CurrentManagedThreadId;
        private readonly DriverLogLevel _minimum;

        public LogCapture(DriverLogLevel minimum = DriverLogLevel.Warning)
        {
            _minimum = minimum;
            DriverLog.Message += Sink;
        }

        /// <summary>The messages captured so far.</summary>
        public IReadOnlyList<string> Messages
        {
            get
            {
                lock (_sync)
                {
                    return _lines.Select(l => l.Message).ToList();
                }
            }
        }

        public void Dispose() => DriverLog.Message -= Sink;

        private void Sink(DriverLogLevel level, string source, string message, Exception? exc)
        {
            if (level < _minimum || Environment.CurrentManagedThreadId != _thread)
            {
                return;
            }

            lock (_sync)
            {
                _lines.Add((level, source, message));
            }
        }
    }
}
