using System;
using System.IO;
using IaiActuatorDriver.Native;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Net48Smoke
{
    /// <summary>
    /// The netstandard2.0 library on .NET Framework 4.8: the native core loads (by full path when
    /// IAI_ACTUATOR_DRIVER_NATIVE names it), one simulated axis moves, one cell of simulated ports answers. No port opened.
    /// </summary>
    public class Net48SmokeTests
    {
        private static readonly AxisOptions Options = new AxisOptions { Stroke = (0.0, 150.0), MaxSpeedMmS = 100.0, MaxAccelG = 0.3, PollIntervalS = 0.005 };

        [Fact]
        public void The_native_core_loads_with_the_abi_of_the_binding()
        {
            NativeDriver.EnsureLoaded();
            Assert.Equal((NativeDriver.AbiMajor, NativeDriver.AbiMinor), NativeDriver.AbiVersion);
            Assert.False(string.IsNullOrEmpty(NativeDriver.LibraryVersion));
            var named = Environment.GetEnvironmentVariable("IAI_ACTUATOR_DRIVER_NATIVE");
            if (!string.IsNullOrWhiteSpace(named))
            {
                Assert.Equal(Path.GetFullPath(named.Trim().Trim('"')), NativeDriver.OverridePath);  // the LoadLibrary preload
            }
        }

        [Fact]
        public void An_axis_moves_on_a_simulated_unit()
        {
            using var axis = Axis.Open(new SimulatedUnit(SimProfile.Slider, timeScale: 50.0), options: Options);
            Assert.Equal(73.95, axis.Status().PositionMm);
            axis.MoveTo(80.0, speed: 50.0);
            Assert.InRange(axis.Status().PositionMm, 79.9, 80.1);
        }

        [Fact]
        public void A_cell_answers_on_simulated_ports()
        {
            using var cell = Cell.Open(SimLine.SimPorts(2, options: new SimulatedUnitOptions { TimeScale = 50.0 }));
            Assert.Equal(2, cell.Count);
            var status = cell.StatusAll();
            Assert.True(status.Succeeded);
            Assert.Equal(2, status.Count);
        }
    }
}
