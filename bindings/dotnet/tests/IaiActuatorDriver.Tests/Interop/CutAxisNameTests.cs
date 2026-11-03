using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using IaiActuatorDriver.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The 0.8 review fixes: a cell's axis name longer than the 128-byte field of iad_cell_result / iad_event comes
    /// back cut, and the index (the axis's position in the request / the cell) is the key; every sized struct handed to an
    /// _init function or an output call has its StructSize set first (the header's "Sized structs").
    /// </summary>
    public class CutAxisNameTests
    {
        private static readonly string Long = new string('p', 150) + ":2";

        private static unsafe IadCellResult Result(string axis, int index, int code = 0)
        {
            var r = default(IadCellResult);
            var text = Encoding.UTF8.GetBytes(axis);
            for (var i = 0; i < Math.Min(text.Length, SizedStruct.AxisNameSize - 1); i++)
            {
                r.Axis[i] = text[i];
            }

            r.Index = index;
            r.Code = code;
            return r;
        }

        private static unsafe IadEvent Event(string axis, int index)
        {
            var e = default(IadEvent);
            var text = Encoding.UTF8.GetBytes(axis);
            for (var i = 0; i < Math.Min(text.Length, SizedStruct.AxisNameSize - 1); i++)
            {
                e.Axis[i] = text[i];
            }

            e.Kind = 1;
            e.Index = index;
            return e;
        }

        [Fact]
        public void NameOf_keeps_a_listed_name_and_resolves_a_cut_one_by_its_index()
        {
            var names = new List<string> { "A:1", Long, "B:1" };
            var cut = Long.Substring(0, SizedStruct.AxisNameSize - 1);
            Assert.Equal("A:1", SizedStruct.NameOf("A:1", 0, names));
            Assert.Equal("B:1", SizedStruct.NameOf("B:1", 2, names));  // a name in the list wins over the index
            Assert.Equal(Long, SizedStruct.NameOf(cut, 1, names));
            Assert.Equal(cut, SizedStruct.NameOf(cut, int.MinValue, names));  // IAD_INT_UNKNOWN: no position
            Assert.Equal(cut, SizedStruct.NameOf(cut, -1, names));
            Assert.Equal(cut, SizedStruct.NameOf(cut, 3, names));
        }

        [Fact]
        public void A_cut_result_name_is_keyed_by_its_position_in_the_request()
        {
            var requested = new List<string> { "A:1", Long };
            var buffer = new[] { Result("A:1", 0), Result(Long, 1, code: 6) };  // the cut text, an error code
            var perAxis = MiniJson.TryParse("{\"" + Long + "\":{\"message\":\"the whole message\"}}") as JsonObject;
            var results = NativeCellResults.From("StatusAll", buffer, 2, perAxis, requested);
            Assert.Equal(requested, results.Names);
            Assert.True(results.ContainsKey("A:1"));
            Assert.False(results.ContainsKey(Long));
            Assert.True(results.Errors.ContainsKey(Long));
            Assert.Contains("the whole message", results.Errors[Long].Message);
        }

        [Fact]
        public void A_cut_event_name_is_resolved_with_the_names_of_its_cell_only()
        {
            var cell = new List<string> { "A:1", Long };
            var cut = Long.Substring(0, SizedStruct.AxisNameSize - 1);
            Assert.Equal(Long, new NativeEvent(Event(cut, 1), cell).Axis);
            Assert.Equal("A:1", new NativeEvent(Event("A:1", 0), cell).Axis);
            Assert.Equal(cut, new NativeEvent(Event(cut, int.MinValue), cell).Axis);  // not a cell's event
            Assert.Equal(cut, new NativeEvent(Event(cut, 1)).Axis);  // an axis's own queue: no names, no change
        }

        [Fact]
        public unsafe void Fresh_sets_the_struct_size_of_every_input_that_is_handed_to_the_library_first()
        {
            Assert.Equal(104u, SizedStruct.Fresh<IadAxisOptions>().StructSize);
            Assert.Equal(80u, SizedStruct.Fresh<IadMoveOptions>().StructSize);
            Assert.Equal(128u, SizedStruct.Fresh<IadPositionEntry>().StructSize);
            Assert.Equal(56u, SizedStruct.Fresh<IadNvWriteOptions>().StructSize);
            Assert.Equal(56u, SizedStruct.Fresh<IadBusOptions>().StructSize);
            Assert.Equal(SizedStruct.Size<IadRecoveryPolicy>(), SizedStruct.Fresh<IadRecoveryPolicy>().StructSize);
            Assert.Equal(0u, SizedStruct.Fresh<IadMoveOptions>().ReservedHeader);
        }

        [NativeFact]
        public void The_init_functions_and_the_sized_outputs_get_the_size_and_leave_the_header_whole()
        {
            // the library writes only up to the struct_size it finds: the binding's structs come back whole
            var o = SizedStruct.Fresh<IadMoveOptions>();
            Assert.Equal(0, NativeMethods.iad_move_options_init(ref o));
            Assert.Equal(SizedStruct.Size<IadMoveOptions>(), o.StructSize);
            Assert.Equal(0u, o.ReservedHeader);
            var axis = SizedStruct.Fresh<IadAxisOptions>();
            Assert.Equal(0, NativeMethods.iad_axis_options_init(ref axis));
            Assert.Equal(SizedStruct.Size<IadAxisOptions>(), axis.StructSize);
            Assert.True(double.IsNaN(axis.MaxSpeedMmS));
        }
    }
}
