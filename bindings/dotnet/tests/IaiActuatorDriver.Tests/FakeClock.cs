using System.Threading;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Manually advanced clock for deterministic simulator tests (pass <see cref="Read"/> as the unit's clock).</summary>
    internal sealed class FakeClock
    {
        private long _ticks = 1000L * 1_000_000_000L; // 1000 s, in ns

        /// <summary>Current time in seconds.</summary>
        public double Now => Interlocked.Read(ref _ticks) / 1e9;

        /// <summary>The clock function.</summary>
        public double Read() => Now;

        /// <summary>Move the clock on by <paramref name="seconds"/>.</summary>
        public void Advance(double seconds) => Interlocked.Add(ref _ticks, (long)System.Math.Round(seconds * 1e9));
    }
}
