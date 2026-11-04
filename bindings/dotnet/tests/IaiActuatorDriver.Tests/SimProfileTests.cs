using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The six simulated units (<see cref="SimProfile"/>) driven by the same axis class: the driver adapts to whichever unit
    /// answers (identity, ranges and kind come from the unit itself), so one body runs against every profile.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class SimProfileTests
    {
        /// <summary>profile, product family, units, kind, stroke, start position, stored rows.</summary>
        public static IEnumerable<object[]> Profiles => new[]
        {
            new object[] { SimProfile.Slider, "EC", "mm", "slider (small)", 150.0, 73.95, 3 },
            new object[] { SimProfile.Gripper, "EC", "mm", "gripper", 9.5, 9.5, 3 },
            new object[] { SimProfile.Rod, "EC", "mm", "rod", 150.0, 0.0, 3 },
            new object[] { SimProfile.Pcon, "PCON", "mm", "linear", 150.0, 60.0, 64 },
            new object[] { SimProfile.Rotary, "EC", "deg", "rotary", 360.0, 0.0, 3 },
            new object[] { SimProfile.Unknown, "IAI controller", "mm", "kind 42", 100.0, 0.0, 2 },
        };

        private static (Axis Axis, SimulatedUnit Sim) Open(SimProfile profile)
        {
            var sim = new SimulatedUnit(SimKit.Sim(profile, timeScale: 50.0));
            var (axis, _) = SimKit.Open(sim, new AxisOptions { PollIntervalS = 0.005 });
            return (axis, sim);
        }

        [Theory]
        [MemberData(nameof(Profiles))]
        public void The_driver_reads_what_the_unit_is(SimProfile profile, string family, string units, string kind, double stroke, double start, int rows)
        {
            var (axis, sim) = Open(profile);
            using (axis)
            {
                var info = axis.Identify();
                Assert.Equal(family, info.ProductFamily);
                Assert.Equal(units, info.Units);
                Assert.Equal(kind, info.Kind);
                Assert.Equal(rows, info.MaxPositions);
                Assert.Equal((-0.30, stroke + 0.30), info.SoftLimitsMm!.Value);
                Assert.Equal(sim.SystemCode, info.SystemCode);
                Assert.Equal(sim.UnitCode, info.UnitCode);

                var limits = axis.Limits();
                Assert.Equal(units, limits.Units);
                Assert.Equal(0.0, limits.Position.Min);
                Assert.Equal(stroke, limits.Position.Max);
                Assert.Equal((double)(rows - 1), limits.StoredRows.Max);
                Assert.Equal(units, axis.Units);

                var st = axis.Status();
                Assert.Equal(start, st.PositionMm);
                Assert.True(st.ServoOn && st.Homed && st.InPosition);
                Assert.Equal(profile == SimProfile.Pcon, st.Warning); // the PCON powers up with message alarm 0x069
            }
        }

        [Theory]
        [MemberData(nameof(Profiles))]
        public void Servo_home_move_and_jog_work_on_every_profile(SimProfile profile, string family, string units, string kind, double stroke, double start, int rows)
        {
            _ = (family, units, kind, start, rows);
            var (axis, sim) = Open(profile);
            using (axis)
            {
                axis.Connect(takeControl: true);
                Assert.True(axis.Servo(false)!.ServoOn == false);
                Assert.True(axis.Servo(true)!.ServoOn);

                var homed = axis.Home()!;
                Assert.True(homed.Homed);
                Assert.True(Math.Abs(homed.PositionMm) <= 0.05, $"homed at {homed.PositionMm}");

                var mid = Math.Round(stroke / 2.0, 2);
                var moved = axis.MoveTo(mid, 20.0, 0.1)!;
                Assert.True(Math.Abs(moved.PositionMm - mid) <= 0.1, $"moved to {moved.PositionMm}, wanted {mid}");
                Assert.True(Math.Abs(sim.PositionMm - mid) <= 0.1);

                var jogged = axis.Jog(1.0, 10.0)!;
                Assert.True(Math.Abs(jogged.PositionMm - (mid + 1.0)) <= 0.1);
                Assert.True(jogged.Idle && jogged.Ready);
            }
        }

        [Theory]
        [MemberData(nameof(Profiles))]
        public void A_move_outside_the_units_own_range_is_refused_before_anything_is_sent(SimProfile profile, string family, string units, string kind, double stroke, double start, int rows)
        {
            _ = (family, units, kind, start, rows);
            var (axis, sim) = Open(profile);
            using (axis)
            {
                axis.Connect(takeControl: true);
                var before = sim.Requests.Count;
                Assert.Throws<SoftLimitException>(() => axis.MoveTo(stroke + 5.0, 10.0, 0.1));
                Assert.DoesNotContain(sim.Requests.Skip(before), f => f[1] == 0x10 || f[1] == 0x05 || f[1] == 0x45);
            }
        }

        [Fact]
        public void An_EC_unit_code_without_a_family_name_is_reported_and_driven_without_a_label()
        {
            // an EC (system 0xEC) on the rod's blocks with unit code 0x7E, which has no family name in the driver
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Rod, timeScale: 50.0) with { UnitCode = 0x7E });
            var (axis, _) = SimKit.Open(sim, new AxisOptions { PollIntervalS = 0.005 });
            using (axis)
            {
                var info = axis.Identify();
                Assert.Equal(("EC", 0xEC, 0x7E), (info.ProductFamily, info.SystemCode!.Value, info.UnitCode!.Value));
                Assert.Null(info.UnitName);
                Assert.False(info.UnitKnown); // display only
                Assert.DoesNotContain(info.Errors,
                    e => e.Contains("UNVERIFIED") || e.Contains("tried on hardware") || e.Contains("family table"));

                var model = axis.ReadParameterModel();
                Assert.False(model.Known);
                Assert.Empty(model.SeenOn);
                Assert.Equal("unit code 0x7E: every mapped item found on the unit is shown", model.Note);
                Assert.All(axis.UserParameterList(), p => Assert.DoesNotContain("UNVERIFIED", p.ModelNote));

                var written = axis.WriteParameter(2, 0.2, commit: false); // written the same way, no per-model warning
                Assert.True(written.Changed && written.Verified);
                Assert.Equal("00000014", written.RawAfter);
                Assert.DoesNotContain(written.Warnings, w => w.Contains("UNVERIFIED") || w.Contains("for this model"));

                axis.Connect(takeControl: true);
                Assert.True(axis.Home()!.Homed);
                Assert.True(Math.Abs(axis.MoveTo(75.0, 20.0, 0.1)!.PositionMm - 75.0) <= 0.1);
            }
        }

        [Fact]
        public void A_stroke_option_only_narrows_the_units_own_stroke()
        {
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Gripper, timeScale: 50.0));
            var (axis, _) = SimKit.Open(sim, new AxisOptions { PollIntervalS = 0.005, Stroke = (0.0, 150.0) });
            using (axis)
            {
                axis.Connect(takeControl: true);
                Assert.Equal(9.5, axis.Limits().Position.Max); // the unit's 9.5 mm wins over the 150 mm the caller hoped for
                Assert.Throws<SoftLimitException>(() => axis.MoveTo(20.0, 10.0, 0.1));
            }
        }

        [Fact]
        public void The_profile_names_are_the_lower_case_enum_names()
        {
            Assert.Equal(
                new[] { "slider", "gripper", "rod", "pcon", "rotary", "unknown" },
                Enum.GetValues(typeof(SimProfile)).Cast<SimProfile>().Select(p => new SimulatedUnit(p).ProfileName));
        }

        [Fact]
        public void Units_of_different_profiles_share_one_line_and_keep_their_own_ranges()
        {
            var line = new SimLine(
                Enum.GetValues(typeof(SimProfile)).Cast<SimProfile>().Select((p, i) => new SimulatedUnit(SimKit.Sim(p, 50.0) with { Slave = i + 1 })),
                "MIX");
            var options = new CellOptions { ScanTimeout = TimeSpan.FromMilliseconds(10), AxisOptions = new AxisOptions { PollIntervalS = 0.005 } };
            using var cell = Cell.Open(new Dictionary<string, SimLine> { ["MIX"] = line }, options: options);
            Assert.Equal(6, cell.Count);
            var limits = cell.LimitsAll();
            Assert.Equal(new[] { 150.0, 9.5, 150.0, 150.0, 360.0, 100.0 }, cell.Names.Select(n => limits[n].Position.Max!.Value));
            Assert.Equal(new[] { "mm", "mm", "mm", "mm", "deg", "mm" }, cell.Names.Select(cell.UnitsOf));
            Assert.Equal(new[] { "EC", "EC", "EC", "PCON", "EC", "IAI controller" }, cell.Names.Select(n => cell.Info[n].ProductFamily));
        }
    }
}
