using System;
using System.Collections.Generic;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>The small JSON reader the binding uses for the library's JSON outputs. No native library needed.</summary>
    public class MiniJsonTests
    {
        [Fact]
        public void Reads_objects_arrays_and_scalars()
        {
            var value = MiniJson.Parse(" {\"a\": 1.5, \"b\": [true, false, null], \"c\": \"x\\\"y\\u00b5\\n\", \"d\": {}, \"e\": -2e3} ");
            var o = Assert.IsType<JsonObject>(value);
            Assert.Equal(new[] { "a", "b", "c", "d", "e" }, o.Keys);
            Assert.Equal(1.5, o.Number("a"));
            Assert.Equal(new List<object?> { true, false, null }, o.Array("b"));
            Assert.Equal("x\"yµ\n", o.String("c"));
            Assert.Empty(o.Object("d")!.Keys);
            Assert.Equal(-2000, o.Int("e"));
            Assert.Null(o.Int("missing"));
            Assert.Null(o.String("a"));
        }

        [Theory]
        [InlineData("")]
        [InlineData("{")]
        [InlineData("[1,]")]
        [InlineData("{\"a\" 1}")]
        [InlineData("\"open")]
        [InlineData("1 2")]
        [InlineData("nul")]
        public void Rejects_what_is_not_json(string text)
        {
            Assert.Throws<FormatException>(() => MiniJson.Parse(text));
            Assert.Null(MiniJson.TryParse(text));
        }
    }
}
