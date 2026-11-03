using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The tests that run the axis class against <see cref="Simulation.SimulatedUnit"/> in real time (timeouts, settle
    /// windows, time-scaled motion): run one at a time, so a busy CPU does not change what a poller sees.
    /// </summary>
    [CollectionDefinition(Name, DisableParallelization = true)]
    public sealed class SimCollection
    {
        public const string Name = "simulated units in real time";
    }
}
