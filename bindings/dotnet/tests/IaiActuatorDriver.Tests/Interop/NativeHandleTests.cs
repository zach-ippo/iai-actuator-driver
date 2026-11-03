using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// What only the internal handles show (<see cref="NativeAxis"/>, <see cref="NativeCell"/> on the core's simulated units,
    /// no port opened): the last error of the calling thread, the result names, the options round trip, closing and the
    /// handles the library itself refuses. The scenarios of the axis and the cell run through the public classes.
    /// </summary>
    public class NativeHandleTests
    {
        private static NativeSimOptions Fast => new NativeSimOptions { TimeScale = 20 };

        [NativeFact]
        public void The_axis_command_options_this_binding_builds_are_the_init_defaults()
        {
            // (0.8) Of() fills every field the binding sets; with the defaults it equals what the _init function writes.
            var servo = default(IadServoOptions);
            NativeErrors.Check(NativeMethods.iad_servo_options_init(ref servo));
            var ownServo = IadServoOptions.Of(true, double.NaN);
            Assert.Equal(servo.StructSize, ownServo.StructSize);
            Assert.Equal(servo.Wait, ownServo.Wait);
            Assert.True(double.IsNaN(servo.TimeoutS));
            var home = default(IadHomeOptions);
            NativeErrors.Check(NativeMethods.iad_home_options_init(ref home));
            Assert.Equal((home.StructSize, home.Wait, home.Method), (IadHomeOptions.Of(true, double.NaN, 0).StructSize, 1, 0));
            var stop = default(IadStopOptions);
            NativeErrors.Check(NativeMethods.iad_stop_options_init(ref stop));
            Assert.Equal((stop.StructSize, stop.Wait), (IadStopOptions.Of(false, double.NaN).StructSize, 0));
            var wait = default(IadWaitIdleOptions);
            NativeErrors.Check(NativeMethods.iad_wait_idle_options_init(ref wait));
            Assert.Equal((wait.StructSize, wait.Completion), (IadWaitIdleOptions.Of(double.NaN, double.NaN, double.NaN, 0).StructSize, 0));
            Assert.True(double.IsNaN(wait.Target) && double.IsNaN(wait.Band) && double.IsNaN(wait.TimeoutS));
            var change = default(IadChangeOptions);
            NativeErrors.Check(NativeMethods.iad_change_options_init(ref change));
            Assert.Equal((change.StructSize, change.Wait), (IadChangeOptions.Of(false, double.NaN).StructSize, 0));
            var cancel = default(IadCancelPausedMoveOptions);
            NativeErrors.Check(NativeMethods.iad_cancel_paused_move_options_init(ref cancel));
            Assert.Equal(cancel.StructSize, IadCancelPausedMoveOptions.Of(double.NaN).StructSize);
            Assert.True(double.IsNaN(cancel.TimeoutS));
        }

        [NativeFact]
        public void A_refusal_is_the_last_error_of_the_thread_until_a_success()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast);
            axis.Connect();

            var e = Assert.Throws<SoftLimitException>(() => axis.MoveTo(1000.0));
            Assert.Equal(NativeDriver.LastErrorMessage, e.Message);  // reading the last error does not change it
            Assert.NotEqual(string.Empty, e.Message);
            Assert.Equal(NativeResult.SoftLimit, NativeErrors.ResultOf(e));
            Assert.Contains("IAD_ERR_SOFT_LIMIT", NativeErrors.ErrorJsonOf(e));
            Assert.Throws<EcNotReadyException>(() => axis.CancelPausedMove());   // no pause
            Assert.Throws<EcNotReadyException>(() => axis.ChangeSpeed(80.0));    // no move of this axis
            Assert.Throws<EcNotReadyException>(() => axis.LastResultJson());     // no write yet
            Assert.Throws<UnsafeCommandException>(() => axis.ReadParameter("no-such-parameter"));

            // A success clears the last error of this thread.
            axis.Status();
            Assert.Equal(string.Empty, NativeDriver.LastErrorMessage);
        }

        [NativeFact]
        public void Result_names_match_the_library()
        {
            foreach (var code in Enum.GetValues(typeof(NativeResult)).Cast<NativeResult>())
            {
                Assert.Equal(NativeErrors.Name(code), NativeDriver.ResultName(code));
            }

            Assert.Equal("IAD_ERR_UNKNOWN", NativeDriver.ResultName((NativeResult)(-99)));
        }

        [NativeFact]
        public void Options_round_trip_and_caps_apply()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast, options: new NativeAxisOptions { MaxSpeedMmS = 100.0 });
            axis.Connect();
            Assert.Equal(100.0, axis.Options.MaxSpeedMmS);
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(50.0, speed: 200.0));
            axis.Options = new NativeAxisOptions { MaxSpeedMmS = 300.0, Strategy = CommandMethod.Iai };
            var options = axis.Options;
            Assert.Equal(300.0, options.MaxSpeedMmS);
            Assert.Equal(CommandMethod.Iai, options.Strategy);
            Assert.InRange(axis.MoveTo(50.0, speed: 200.0)!.PositionMm, 49.9, 50.1);
            axis.Options = new NativeAxisOptions { Strategy = CommandMethod.Iai2, Completion = NativeCompletion.Fast, PushSettleS = -1.0 };
            options = axis.Options;
            Assert.Equal(CommandMethod.Iai2, options.Strategy);  // EXPERIMENTAL in the core (0.2)
            Assert.Equal(NativeCompletion.Fast, options.Completion);
            Assert.True(options.PushSettleS < 0.0);
            axis.Options = new NativeAxisOptions();
            Assert.Equal(NativeCompletion.Safe, axis.Options.Completion);
            Assert.Equal(0.15, axis.Options.PushSettleS!.Value, 6);
        }

        [NativeFact]
        public void Close_is_safe_twice_and_a_closed_axis_refuses_calls()
        {
            var axis = NativeAxis.OpenSim("slider", Fast);
            axis.Status();
            axis.Close();
            axis.Close();
            axis.Dispose();
            Assert.True(axis.IsClosed);
            Assert.Throws<ObjectDisposedException>(() => axis.Status());
        }

        [NativeFact]
        public void An_axis_handle_of_the_cell_releases_only_itself()
        {
            using var cell = NativeCell.OpenSim(new[] { "slider" }, slavesPerLine: 2, sim: Fast);
            var axis = cell["SIM1:2"];
            Assert.Same(axis, cell["SIM1:2"]);
            Assert.Same(axis, cell[1]);
            Assert.Equal("SIM1:2", axis.Name);
            axis.Dispose();
            Assert.Equal(2, cell.StatusAll().Count);
            Assert.False(cell["SIM1:2"].IsClosed);
            Assert.Throws<ArgumentOutOfRangeException>(() => cell["SIM9:9"]);
            Assert.Throws<ArgumentOutOfRangeException>(() => cell.StatusAll(new[] { "SIM9:9" }));
        }

        [NativeFact]
        public void Closing_the_cell_closes_its_axes_and_the_library_refuses_their_handles()
        {
            var cell = NativeCell.OpenSim(sim: Fast);
            var owned = cell["SIM1:1"];

            // A handle the cell object does not track: the library itself refuses it once the cell is closed.
            Assert.Equal(0, NativeMethods.iad_cell_open_axis(cell.Handle, Utf8.Encode("SIM1:1")!, out var handle));
            var raw = new NativeAxis(handle, "SIM1:1");
            raw.Status();

            cell.Close();
            cell.Close();
            Assert.True(cell.IsClosed);
            Assert.True(owned.IsClosed);
            Assert.Throws<ObjectDisposedException>(() => owned.Status());
            Assert.Throws<ArgumentException>(() => raw.Status());
            raw.Close();  // closing it stays safe
            Assert.Throws<ObjectDisposedException>(() => cell.StatusAll());
        }

        [NativeFact]
        public void Bad_input_is_refused_before_the_library()
        {
            using var cell = NativeCell.OpenSim(sim: Fast);
            Assert.Throws<ArgumentException>(() => cell.StatusAll(new[] { "SIM1:1,SIM1:2" }));
            Assert.Throws<ArgumentException>(() => cell.MoveAll(new Dictionary<string, double>()));
            Assert.Throws<ArgumentException>(() => NativeCell.OpenSim(new[] { "no-such-profile" }));
        }
    }
}
