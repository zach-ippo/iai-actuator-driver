using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The C# mirrors of the ABI structs have the sizes and offsets core/src/c_api.cpp freezes with static_assert. No
    /// native library needed.
    /// </summary>
    public class LayoutTests
    {
        [Theory]
        [InlineData(typeof(IadAxisOptions), 104)]
        [InlineData(typeof(IadOpenOptions), 72)]
        [InlineData(typeof(IadSimOptions), 80)]
        [InlineData(typeof(IadMoveOptions), 80)]
        [InlineData(typeof(IadPositionMoveOptions), 64)]
        [InlineData(typeof(IadNvWriteOptions), 56)]
        [InlineData(typeof(IadParamWriteOptions), 64)]
        [InlineData(typeof(IadBaudOptions), 56)]
        [InlineData(typeof(IadResetOptions), 56)]
        [InlineData(typeof(IadCellOptions), 96)]
        [InlineData(typeof(IadStatus), 208)]
        [InlineData(typeof(IadRange), 88)]
        [InlineData(typeof(IadLimits), 1096)]
        [InlineData(typeof(IadIdentity), 616)]
        [InlineData(typeof(IadPositionEntry), 128)]
        [InlineData(typeof(IadParameter), 288)]
        [InlineData(typeof(IadParamWriteResult), 320)]
        [InlineData(typeof(IadPositionWriteResult), 136)]
        [InlineData(typeof(IadResetResult), 352)]
        [InlineData(typeof(IadBaudResult), 128)]
        [InlineData(typeof(IadCellResult), 632)]
        [InlineData(typeof(IadPushOptions), 72)]
        [InlineData(typeof(IadPushResult), 272)]
        [InlineData(typeof(IadSequencePoint), 56)]
        [InlineData(typeof(IadSequenceOptions), 88)]
        [InlineData(typeof(IadMoveInfo), 296)]
        [InlineData(typeof(IadMonitorOptions), 72)]
        [InlineData(typeof(IadMonitorSample), 304)]
        [InlineData(typeof(IadEvent), 736)]
        [InlineData(typeof(IadFastSummary), 304)]
        [InlineData(typeof(IadTrackOptions), 48)]
        [InlineData(typeof(IadLinkCheck), 800)]
        [InlineData(typeof(IadRecoveryPolicy), 80)]
        [InlineData(typeof(IadRecoverySummary), 768)]
        [InlineData(typeof(IadCheckOptions), 72)]
        [InlineData(typeof(IadCheckSummary), 56)]
        [InlineData(typeof(IadAxisMove), 224)]
        [InlineData(typeof(IadMotionCaps), 336)]
        [InlineData(typeof(IadServoOptions), 56)]
        [InlineData(typeof(IadHomeOptions), 56)]
        [InlineData(typeof(IadStopOptions), 56)]
        [InlineData(typeof(IadWaitIdleOptions), 72)]
        [InlineData(typeof(IadChangeOptions), 56)]
        [InlineData(typeof(IadCancelPausedMoveOptions), 48)]
        public void Struct_sizes_match_the_header(Type type, int size)
        {
            Assert.Equal(size, Marshal.SizeOf(type));
            // Blittable: the managed layout is the native one, so ref passes the struct itself (pinned, not copied).
            var managed = (int)typeof(Unsafe).GetMethod(nameof(Unsafe.SizeOf))!.MakeGenericMethod(type).Invoke(null, null)!;
            Assert.Equal(size, managed);
        }

        [Theory]
        [InlineData(typeof(IadAxisOptions))]
        [InlineData(typeof(IadOpenOptions))]
        [InlineData(typeof(IadSimOptions))]
        [InlineData(typeof(IadMoveOptions))]
        [InlineData(typeof(IadPositionMoveOptions))]
        [InlineData(typeof(IadNvWriteOptions))]
        [InlineData(typeof(IadParamWriteOptions))]
        [InlineData(typeof(IadBaudOptions))]
        [InlineData(typeof(IadResetOptions))]
        [InlineData(typeof(IadCellOptions))]
        [InlineData(typeof(IadPositionEntry))]
        [InlineData(typeof(IadPushOptions))]
        [InlineData(typeof(IadSequencePoint))]
        [InlineData(typeof(IadSequenceOptions))]
        [InlineData(typeof(IadMonitorOptions))]
        [InlineData(typeof(IadTrackOptions))]
        [InlineData(typeof(IadRecoveryPolicy))]
        [InlineData(typeof(IadCheckOptions))]
        [InlineData(typeof(IadAxisMove))]
        [InlineData(typeof(IadServoOptions))]
        [InlineData(typeof(IadHomeOptions))]
        [InlineData(typeof(IadStopOptions))]
        [InlineData(typeof(IadWaitIdleOptions))]
        [InlineData(typeof(IadChangeOptions))]
        [InlineData(typeof(IadCancelPausedMoveOptions))]
        public void Every_input_is_sized_from_0_8(Type type)
        {
            // struct_size, reserved_header first (the header's "Sized structs")
            Assert.Equal(0, (int)Marshal.OffsetOf(type, "StructSize"));
            Assert.Equal(4, (int)Marshal.OffsetOf(type, "ReservedHeader"));
        }

        [Fact]
        public void Field_offsets_match_the_header()
        {
            Assert.Equal(0, (int)Marshal.OffsetOf<IadStatus>(nameof(IadStatus.PositionMm)));
            Assert.Equal(32, (int)Marshal.OffsetOf<IadStatus>(nameof(IadStatus.AlarmCode)));
            Assert.Equal(140, (int)Marshal.OffsetOf<IadStatus>(nameof(IadStatus.Rcon)));
            Assert.Equal(144, (int)Marshal.OffsetOf<IadStatus>(nameof(IadStatus.Reserved)));
            Assert.Equal(208, (int)Marshal.OffsetOf<IadCellResult>(nameof(IadCellResult.Code)));
            Assert.Equal(212, (int)Marshal.OffsetOf<IadCellResult>(nameof(IadCellResult.Index)));
            Assert.Equal(216, (int)Marshal.OffsetOf<IadCellResult>(nameof(IadCellResult.Axis)));
            Assert.Equal(344, (int)Marshal.OffsetOf<IadCellResult>(nameof(IadCellResult.Message)));
            Assert.Equal(40, (int)Marshal.OffsetOf<IadIdentity>(nameof(IadIdentity.Slave)));
            Assert.Equal(80, (int)Marshal.OffsetOf<IadIdentity>(nameof(IadIdentity.ProductFamily)));
            Assert.Equal(968, (int)Marshal.OffsetOf<IadLimits>(nameof(IadLimits.Units)));
            Assert.Equal(232, (int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.Kind)));
            Assert.Equal(240, (int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.Axis)));
            Assert.Equal(432, (int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.Message)));
            Assert.Equal(692, (int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.Index)));
            Assert.Equal(696, (int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.CallerThread)));
            Assert.Equal(56, (int)Marshal.OffsetOf<IadPositionWriteResult>(nameof(IadPositionWriteResult.PushPower)));
            Assert.Equal(84, (int)Marshal.OffsetOf<IadPositionWriteResult>(nameof(IadPositionWriteResult.Row)));
            Assert.Equal(320, (int)Marshal.OffsetOf<IadRecoverySummary>(nameof(IadRecoverySummary.Axis)));
        }

        [Fact]
        public void The_input_fields_keep_their_offsets_after_the_0_8_header()
        {
            // The static_asserts of core/src/c_api.cpp ("inputs", "iad_axis_move").
            Assert.Equal(72, (int)Marshal.OffsetOf<IadAxisOptions>(nameof(IadAxisOptions.PushSettleS)));
            Assert.Equal(80, (int)Marshal.OffsetOf<IadAxisOptions>(nameof(IadAxisOptions.Completion)));
            Assert.Equal(36, (int)Marshal.OffsetOf<IadOpenOptions>(nameof(IadOpenOptions.Line)));
            Assert.Equal(40, (int)Marshal.OffsetOf<IadOpenOptions>(nameof(IadOpenOptions.PositionUnitMm)));
            Assert.Equal(48, (int)Marshal.OffsetOf<IadOpenOptions>(nameof(IadOpenOptions.Fast)));
            Assert.Equal(44, (int)Marshal.OffsetOf<IadMoveOptions>(nameof(IadMoveOptions.AccelPattern)));
            Assert.Equal(48, (int)Marshal.OffsetOf<IadMoveOptions>(nameof(IadMoveOptions.DecelG)));
            Assert.Equal(56, (int)Marshal.OffsetOf<IadMoveOptions>(nameof(IadMoveOptions.Completion)));
            Assert.Equal(60, (int)Marshal.OffsetOf<IadCellOptions>(nameof(IadCellOptions.Line)));
            Assert.Equal(64, (int)Marshal.OffsetOf<IadCellOptions>(nameof(IadCellOptions.PositionUnitMm)));
            Assert.Equal(64, (int)Marshal.OffsetOf<IadPositionEntry>(nameof(IadPositionEntry.PushPower)));
            Assert.Equal(8, (int)Marshal.OffsetOf<IadAxisMove>(nameof(IadAxisMove.Position)));
            Assert.Equal(48, (int)Marshal.OffsetOf<IadAxisMove>(nameof(IadAxisMove.DecelG)));
            Assert.Equal(56, (int)Marshal.OffsetOf<IadAxisMove>(nameof(IadAxisMove.AccelPattern)));
            Assert.Equal(64, (int)Marshal.OffsetOf<IadAxisMove>(nameof(IadAxisMove.Axis)));
            Assert.Equal(224u, SizedStruct.Size<IadAxisMove>());
            Assert.Equal(128, SizedStruct.AxisNameSize);
        }

        [Fact]
        public unsafe void Fixed_text_fields_decode_utf8_up_to_the_nul()
        {
            var r = default(IadCellResult);
            var text = System.Text.Encoding.UTF8.GetBytes("SIM1:2");
            for (var i = 0; i < text.Length; i++)
            {
                r.Axis[i] = text[i];
            }

            Assert.Equal("SIM1:2", Utf8.FromFixed(r.Axis, SizedStruct.AxisNameSize));

            // A field filled to its size without a NUL is read whole, never past it.
            var full = default(IadRange);
            for (var i = 0; i < 8; i++)
            {
                full.Unit[i] = (byte)'a';
            }

            Assert.Equal("aaaaaaaa", Utf8.FromFixed(full.Unit, 8));
        }

        [Fact]
        public void Strings_in_are_nul_terminated_utf8()
        {
            Assert.Null(Utf8.Encode(null));
            Assert.Equal(new byte[] { (byte)'C', (byte)'O', (byte)'M', (byte)'3', 0 }, Utf8.Encode("COM3"));
            Assert.Equal(new byte[] { 0xC2, 0xB5, 0 }, Utf8.Encode("µ"));
            Assert.Throws<ArgumentException>(() => Utf8.Encode("a\0b"));
            Assert.Equal("µm", Utf8.FromBuffer(new byte[] { 0xC2, 0xB5, (byte)'m', 0, (byte)'x' }));
        }
    }
}
