using IaiActuatorDriver.Simulation;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// A <see cref="SimulatedUnit"/> on a <see cref="FakeClock"/> that advances <see cref="StepS"/> on every request: each poll
    /// sees the axis exactly that much later, whatever the host's speed, so tests of short-lived states (the in-band settling
    /// tail, the G5 push blip) are deterministic.
    /// </summary>
    internal sealed class SteppedClockUnit : SimulatedUnit
    {
        public SteppedClockUnit(double stepS, SimulatedUnitOptions options)
            : this(new FakeClock(), stepS, options)
        {
        }

        private SteppedClockUnit(FakeClock clock, double stepS, SimulatedUnitOptions options)
            : base(options with { Clock = clock.Read })
        {
            FakeClock = clock;
            StepS = stepS;
        }

        public FakeClock FakeClock { get; }

        public double StepS { get; }

        protected override void WriteFrame(byte[] data)
        {
            FakeClock.Advance(StepS);
            base.WriteFrame(data);
        }
    }
}
