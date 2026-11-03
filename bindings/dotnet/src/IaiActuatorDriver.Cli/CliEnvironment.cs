using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Simulation;

namespace IaiActuatorDriver.Cli
{
    /// <summary>
    /// What the tool needs from the outside world: the port list, how a serial port is opened and how a simulated line is
    /// made. Tests replace these so that no real serial port is ever opened.
    /// </summary>
    internal sealed class CliEnvironment
    {
        /// <summary>The real PC: the native core's serial ports and the built-in simulator.</summary>
        public static CliEnvironment Default { get; } = new CliEnvironment();

        /// <summary>The serial ports of this PC (READ-ONLY, opens nothing).</summary>
        public Func<IReadOnlyList<SerialPortInfo>> ListPorts { get; init; } = PortManager.ListPorts;

        /// <summary>Open a serial port: (name, baud rate, read timeout).</summary>
        public Func<string, int, TimeSpan?, ITransport> OpenSerial { get; init; } = (name, baud, timeout) => new SerialTransport(name, baud, timeout);

        /// <summary>A simulated line: (profile, slave addresses on it, modelled baud rate or <c>null</c>).</summary>
        public Func<string, IReadOnlyList<int>, int?, ITransport> CreateSim { get; init; } = SimSupport.CreateLine;

        /// <summary>The profile names <c>--sim</c> accepts.</summary>
        public IReadOnlyList<string> SimProfiles { get; init; } = SimSupport.Profiles;
    }

    /// <summary>The <c>--sim</c> hook: simulated units on a simulated line (no serial port is ever opened).</summary>
    internal static class SimSupport
    {
        /// <summary>The simulated units (<see cref="SimProfile"/>), lower case: slider, gripper, rod, pcon, rotary, unknown.</summary>
        public static IReadOnlyList<string> Profiles { get; } =
            new[] { SimProfile.Slider, SimProfile.Gripper, SimProfile.Rod, SimProfile.Pcon, SimProfile.Rotary, SimProfile.Unknown }
                .Select(p => p.ToString().ToLowerInvariant()).ToList();

        /// <summary>
        /// A simulated line named "sim" with one <see cref="SimulatedUnit"/> of <paramref name="profile"/> per address in
        /// <paramref name="slaves"/> (real time). <paramref name="baudRate"/> models the serial line's rate (the units then
        /// only answer at that rate); <c>null</c> models none.
        /// </summary>
        public static ITransport CreateLine(string profile, IReadOnlyList<int> slaves, int? baudRate)
        {
            if (profile is null || !Profiles.Contains(profile.ToLowerInvariant()) || !Enum.TryParse<SimProfile>(profile, ignoreCase: true, out var kind))
            {
                throw new ArgumentException($"--sim: unknown profile '{profile}' (choose from {string.Join(", ", Profiles)})", nameof(profile));
            }

            return SimLine.WithSlaves(slaves, new SimulatedUnitOptions { Profile = kind, BaudRate = baudRate }, "sim");
        }
    }
}
