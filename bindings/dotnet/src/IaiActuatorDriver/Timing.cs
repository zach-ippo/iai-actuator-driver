using System.Diagnostics;

namespace IaiActuatorDriver
{
    /// <summary>
    /// The managed monotonic clock: the time stamps of the managed test doubles (<see cref="FakeTransport"/>, the
    /// simulated line, the managed simulator) and of the non-volatile write notes. The waits, gaps and settle times are
    /// the core's.
    /// </summary>
    internal static class Timing
    {
        /// <summary>Seconds on a monotonic clock (arbitrary origin).</summary>
        public static double Now => Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
    }
}
