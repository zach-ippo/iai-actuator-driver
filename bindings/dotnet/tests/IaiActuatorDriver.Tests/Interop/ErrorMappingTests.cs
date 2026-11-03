using System;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// Result codes become the managed edition's exception classes, with the library's message and the details of
    /// iad_last_error_json (docs/NATIVE_ABI.md, "Results and errors"). No native library needed.
    /// </summary>
    public class ErrorMappingTests
    {
        [Theory]
        [InlineData(NativeResult.SoftLimit, typeof(SoftLimitException))]
        [InlineData(NativeResult.NotReady, typeof(EcNotReadyException))]
        [InlineData(NativeResult.Alarm, typeof(EcAlarmException))]
        [InlineData(NativeResult.MotionTimeout, typeof(MotionTimeoutException))]
        [InlineData(NativeResult.ModbusTimeout, typeof(ModbusTimeoutException))]
        [InlineData(NativeResult.Frame, typeof(ModbusFrameException))]
        [InlineData(NativeResult.Transport, typeof(ModbusException))]
        [InlineData(NativeResult.Verification, typeof(VerificationException))]
        [InlineData(NativeResult.NonvolatileRefused, typeof(NonVolatileWriteException))]
        [InlineData(NativeResult.InvalidArgument, typeof(ArgumentException))]
        [InlineData(NativeResult.Unsupported, typeof(NotSupportedException))]
        [InlineData(NativeResult.Internal, typeof(InvalidOperationException))]
        [InlineData(NativeResult.ServoBlocked, typeof(EcNotReadyException))]
        [InlineData(NativeResult.MotionIncomplete, typeof(MotionIncompleteException))]
        [InlineData(NativeResult.SlaveException, typeof(ModbusSlaveException))]
        [InlineData(NativeResult.CommandRejected, typeof(ModbusException))]
        [InlineData(NativeResult.UnsafeRefused, typeof(UnsafeCommandException))]
        [InlineData(NativeResult.Cell, typeof(CellException))]
        [InlineData(NativeResult.Modbus, typeof(ModbusException))]
        [InlineData(NativeResult.Driver, typeof(EcDriverException))]
        [InlineData(NativeResult.Recovery, typeof(RecoveryException))]
        [InlineData(NativeResult.UnitReplaced, typeof(UnitReplacedException))]
        [InlineData(NativeResult.File, typeof(System.IO.IOException))]
        [InlineData(NativeResult.Cancelled, typeof(OperationCanceledException))]
        [InlineData((NativeResult)(-99), typeof(EcDriverException))]
        public void Each_code_gives_the_managed_class(NativeResult code, Type expected)
        {
            var e = NativeErrors.Create((int)code, "something failed", null);
            Assert.Equal(expected, e.GetType());
            Assert.Equal((int)code, e.Data[NativeErrors.ResultKey]);
            Assert.Equal(code, NativeErrors.ResultOf(e));
            Assert.Equal(NativeErrors.Name(code), e.Data[NativeErrors.NameKey]);
        }

        [Theory]
        [InlineData(-26)]
        [InlineData(-27)]
        [InlineData(-1000)]
        [InlineData(int.MinValue)]
        public void An_unknown_negative_code_is_the_base_driver_error_with_its_code(int code)
        {
            // iai_actuator_driver.h reserves the codes from -26 down for later additions: a code that a newer library
            // returns is the base driver error with its code kept, never a crash of this binding
            Assert.Equal(-25, Enum.GetValues(typeof(NativeResult)).Cast<int>().Min());
            var json = "{\"code\": " + code + ", \"kind\": \"NewerError\", \"message\": \"a newer kind\"}";
            var e = NativeErrors.Create(code, "a newer kind", json);
            Assert.Equal(typeof(EcDriverException), e.GetType());
            Assert.Equal("a newer kind", e.Message);
            Assert.Equal(code, e.Data[NativeErrors.ResultKey]);
            Assert.Equal((NativeResult)code, NativeErrors.ResultOf(e));
            Assert.Equal("IAD_ERR_UNKNOWN", e.Data[NativeErrors.NameKey]);
            Assert.Equal("NewerError", e.Data[NativeErrors.KindKey]);
            Assert.Equal(json, NativeErrors.ErrorJsonOf(e));

            // no message and no error record: the name an unknown code has
            var bare = NativeErrors.Create(code, null, null);
            Assert.Equal(typeof(EcDriverException), bare.GetType());
            Assert.Equal("IAD_ERR_UNKNOWN", bare.Message);
            Assert.Equal((NativeResult)code, NativeErrors.ResultOf(bare));

            // one axis's error of a group command
            var cellJson = "{\"code\": -19, \"operation\": \"home_all\", \"axes\": {\"SIM1:1\": {\"code\": " + code
                + ", \"message\": \"m\"}}}";
            var cell = Assert.IsType<CellException>(
                NativeErrors.Create((int)NativeResult.Cell, "home_all failed", cellJson));
            var axis = cell.Errors["SIM1:1"];
            Assert.Equal(typeof(EcDriverException), axis.GetType());
            Assert.Equal((NativeResult)code, NativeErrors.ResultOf(axis));
        }

        [Fact]
        public void The_message_is_the_library_message()
        {
            var e = NativeErrors.Create((int)NativeResult.SoftLimit, "target 200.00 mm outside 0.00 .. 150.00 mm; nothing sent", null);
            Assert.Equal("target 200.00 mm outside 0.00 .. 150.00 mm; nothing sent", e.Message);
            Assert.Equal(string.Empty, NativeErrors.ErrorJsonOf(e));
        }

        [Fact]
        public void Notes_attempts_and_stop_flag_go_to_the_managed_fields()
        {
            const string json = "{\"code\": -5, \"name\": \"IAD_ERR_MODBUS_TIMEOUT\", \"kind\": \"ModbusTimeout\", "
                + "\"message\": \"no answer from slave 1\", \"notes\": [\"decel stop sent after this error\"], "
                + "\"stop_attempted\": true, \"attempts\": 4}";
            const string what = "no answer from slave 1 (gave up after 4 attempts) [decel stop sent after this error]";
            var e = Assert.IsType<ModbusTimeoutException>(NativeErrors.Create(-5, what, json));
            // The same parts, in the order the managed classes render them (notes, then the attempts).
            Assert.Equal("no answer from slave 1 [decel stop sent after this error] (gave up after 4 attempts)", e.Message);
            Assert.Equal(4, e.Attempts);
            Assert.Equal(new[] { "decel stop sent after this error" }, e.Notes);
            Assert.True(e.StopAttempted);
            Assert.Equal("ModbusTimeout", e.Data[NativeErrors.KindKey]);
            Assert.Equal(json, NativeErrors.ErrorJsonOf(e));
        }

        [Fact]
        public void Notes_alone_keep_the_library_message()
        {
            const string json = "{\"code\": -4, \"notes\": [\"decel stop sent after this error\"], \"stop_attempted\": true}";
            const string what = "axis still moving 5 s after STOP [decel stop sent after this error]";
            var e = Assert.IsType<MotionTimeoutException>(NativeErrors.Create(-4, what, json));
            Assert.Equal(what, e.Message);
            Assert.Single(e.Notes);
        }

        [Fact]
        public void A_message_that_does_not_end_with_the_notes_is_kept_whole()
        {
            const string json = "{\"code\": -1, \"notes\": [\"a note\"], \"stop_attempted\": true}";
            var e = Assert.IsType<SoftLimitException>(NativeErrors.Create(-1, "plain message", json));
            Assert.Equal("plain message", e.Message);
            Assert.Empty(e.Notes);
            Assert.True(e.StopAttempted);
        }

        [Fact]
        public void Detail_fields_fill_the_specific_classes()
        {
            var alarm = Assert.IsType<EcAlarmException>(NativeErrors.Create(-3, "actuator alarm 0x0d9", "{\"code\": -3, \"alarm_code\": 217}"));
            Assert.Equal(0x0D9, alarm.AlarmCode);

            var slave = Assert.IsType<ModbusSlaveException>(NativeErrors.Create(-16, "Modbus exception 02h", "{\"code\": -16, \"exception_code\": 2, \"function\": 3}"));
            Assert.Equal(2, slave.Code);
            Assert.Equal(3, slave.Function);

            var crc = NativeErrors.Create(-6, "bad CRC", "{\"code\": -6, \"kind\": \"ModbusCrcError\"}");
            Assert.IsType<ModbusCrcException>(crc);

            var rejected = Assert.IsType<IaiCommandRejectedException>(
                NativeErrors.Create(-17, "IAI command 0x0a rejected", "{\"code\": -17, \"function\": 69, \"command\": 10, \"rejection\": 17}"));
            Assert.Equal(0x45, rejected.Function);
            Assert.Equal(0x0A, rejected.Command);
            Assert.Equal(0x11, rejected.Rejection);

            Assert.IsType<EcException>(NativeErrors.Create(-21, "x", "{\"kind\": \"EcError\"}"));
        }

        [Fact]
        public void A_cell_error_lists_each_axis()
        {
            const string json = "{\"code\": -19, \"operation\": \"move_all\", \"axes\": {"
                + "\"SIM1:1\": {\"code\": -1, \"message\": \"target outside the stroke\"}, "
                + "\"SIM1:2\": {\"code\": -2, \"message\": \"servo off\"}}}";
            var e = Assert.IsType<CellException>(NativeErrors.Create(-19, "move_all failed on 2 axes", json));
            Assert.Equal("move_all", e.Operation);
            Assert.Equal(new[] { "SIM1:1", "SIM1:2" }, e.Errors.Keys.OrderBy(k => k));
            Assert.IsType<SoftLimitException>(e.Errors["SIM1:1"]);
            Assert.IsType<EcNotReadyException>(e.Errors["SIM1:2"]);
            Assert.Equal("servo off", e.Errors["SIM1:2"].Message);
        }

        [Fact]
        public void Bad_json_is_ignored()
        {
            var e = NativeErrors.Create(-2, "not ready", "{not json");
            Assert.IsType<EcNotReadyException>(e);
            Assert.Equal("not ready", e.Message);
        }

        [Fact]
        public void An_empty_message_falls_back_to_the_json_then_the_name()
        {
            Assert.Equal("from json", NativeErrors.Create(-2, "", "{\"message\": \"from json\"}").Message);
            Assert.Equal("IAD_ERR_NOT_READY", NativeErrors.Create(-2, "", null).Message);
        }
    }
}
