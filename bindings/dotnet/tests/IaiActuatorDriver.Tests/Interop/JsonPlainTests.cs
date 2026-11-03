using System.Collections.Generic;
using IaiActuatorDriver.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>The library's JSON outputs as plain values and back to text (the layer under the report, backup and unit-info dictionaries).</summary>
    public class JsonPlainTests
    {
        [Fact]
        public void Parsed_values_become_dictionaries_lists_and_whole_numbers_become_longs()
        {
            var plain = (Dictionary<string, object?>)JsonPlain.Plain(MiniJson.Parse("{\"n\":3,\"x\":2.5,\"ok\":true,\"none\":null,\"s\":\"a\",\"l\":[1,{\"k\":-7}]}"))!;
            Assert.Equal(new[] { "n", "x", "ok", "none", "s", "l" }, new List<string>(plain.Keys));  // document order
            Assert.Equal(3L, plain["n"]);
            Assert.Equal(2.5, plain["x"]);
            Assert.Equal(true, plain["ok"]);
            Assert.Null(plain["none"]);
            var list = (List<object?>)plain["l"]!;
            Assert.Equal(1L, list[0]);
            Assert.Equal(-7L, ((Dictionary<string, object?>)list[1]!)["k"]);
            Assert.Empty(JsonPlain.DictOf("not json"));
            Assert.Empty(JsonPlain.DictOf("[1,2]"));
        }

        [Fact]
        public void Text_is_compact_or_indented_and_escapes_what_it_must()
        {
            var value = new Dictionary<string, object?>
            {
                ["name"] = "a \"quoted\" line\nwith\ttabs \\ and \u0001",
                ["list"] = new List<object?> { 1, 2.5, true, null, new Dictionary<string, object?>() },
                ["empty"] = new List<object?>(),
                ["nan"] = double.NaN,
            };
            Assert.Equal(
                @"{""name"":""a \""quoted\"" line\nwith\ttabs \\ and \u0001"",""list"":[1,2.5,true,null,{}],""empty"":[],""nan"":null}",
                JsonPlain.Write(value, indented: false));
            var indented = JsonPlain.Write(new Dictionary<string, object?> { ["a"] = new List<object?> { 1, 2 }, ["b"] = new Dictionary<string, object?> { ["c"] = null } });
            Assert.Equal("{\n  \"a\": [\n    1,\n    2\n  ],\n  \"b\": {\n    \"c\": null\n  }\n}", indented);

            // what is written reads back as the same values
            var back = (Dictionary<string, object?>)JsonPlain.Plain(MiniJson.Parse(JsonPlain.Write(value)))!;
            Assert.Equal(value["name"], back["name"]);
            Assert.Null(back["nan"]);
        }
    }
}
