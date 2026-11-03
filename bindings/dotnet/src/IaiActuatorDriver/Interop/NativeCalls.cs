using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// (ABI 0.4) The core's stateless functions by name (iad_call_json, core/src/c_api_tables.cpp): the alarm and CON
    /// parameter tables and rules and the unit-information decoders, JSON in and out, no I/O. The managed tables stay
    /// (pure, decision 4 of the switch plan); the conformance tests check them against these.
    /// </summary>
    internal static class NativeCalls
    {
        private static IReadOnlyCollection<string>? _functions;

        /// <summary>
        /// One function, its result as JSON. <paramref name="argsJson"/>: an object of named arguments (null = {}), or an
        /// array of such objects (the result is then the array of the results). An unknown function is a
        /// <see cref="NotSupportedException"/>, bad arguments an <see cref="ArgumentException"/>.
        /// </summary>
        internal static string Call(string function, string? argsJson = null)
        {
            NativeDriver.EnsureLoaded();
            var name = Utf8.EncodeRequired(function, nameof(function));
            var args = Utf8.Encode(argsJson, nameof(argsJson));
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_call_json(name, args, b, s, out r));
        }

        /// <summary>One function, its result parsed (a <see cref="JsonObject"/>, a list, a string, a number, a bool or null).</summary>
        internal static object? CallParsed(string function, string? argsJson = null) => MiniJson.Parse(Call(function, argsJson));

        /// <summary>The names the loaded library offers (the "functions" function), read once.</summary>
        internal static IReadOnlyCollection<string> Functions =>
            _functions ??= new HashSet<string>(((IEnumerable<object?>)CallParsed("functions")!).OfType<string>(), StringComparer.Ordinal);

        /// <summary>The loaded library has <paramref name="function"/> (a binding can ask what a library lacks).</summary>
        internal static bool Has(string function) => Functions.Contains(function);
    }
}
