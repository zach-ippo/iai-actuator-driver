using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    public sealed partial class Axis
    {
        /// <summary>The sections <see cref="UnitInfo"/> reads from the unit, as the Python edition names them.</summary>
        public static readonly IReadOnlyList<string> UnitInfoSections = new[] { "identity", "parameters", "monitor", "alarms", "maintenance", "positions" };

        /// <summary>
        /// The sections <see cref="UnitInfo"/> adds that the Python edition does not have: <c>limits</c> (<see cref="Limits"/> with the
        /// unit's <see cref="AxisLimits"/>). Included when no section is named.
        /// </summary>
        public static readonly IReadOnlyList<string> UnitInfoExtraSections = new[] { "limits" };

        /// <summary>Schema name of the <see cref="UnitInfo"/> dictionary (the Python edition's).</summary>
        public const string UnitInfoSchema = "ec_driver.unit_info/1";

        /// <summary>
        /// Everything the unit tells about itself, decoded, as one JSON-serialisable dictionary (strings, numbers, booleans, lists and
        /// nested dictionaries only; it serialises as it is with System.Text.Json): the layout of the Python edition's
        /// <c>unit_info</c>. READ-ONLY: only FC03 and FC11 are sent, and every block read stays inside the unit's own directory.
        /// Keys: <c>meta</c> (schema, read_only, function_codes, taken_at, driver_version, sections, duration_s), <c>identity</c> (the
        /// <see cref="DeviceInfo"/> fields, link, strokes, caps, support words, the directory), <c>parameters</c> (the named
        /// parameters, the parameter model, every raw item of classes 3 / 4 / 7 / 8 / 9 / 0x0C), <c>monitor</c> (the decoded monitor
        /// block), <c>status_words</c>, <c>io</c> and <c>control_words</c> (the status, I/O and control words by bit),
        /// <c>alarms</c> (the current alarm with its level, reset and remedy, the history), <c>maintenance</c> (move and distance
        /// counters, calendar time), <c>positions</c> (the stored rows), <c>limits</c> (<see cref="UnitInfoExtraSections"/>; with
        /// <c>sections</c> null) and <c>errors</c> ("section: message" for every part that could not be read). A section not requested is
        /// <c>null</c>. <paramref name="sections"/>: a subset of <see cref="UnitInfoSections"/> and <see cref="UnitInfoExtraSections"/>;
        /// <c>null</c> = every section.
        /// </summary>
        public IDictionary<string, object?> UnitInfo(IEnumerable<string>? sections = null)
        {
            var requested = sections?.Distinct().ToList();
            if (requested != null)
            {
                var unknown = requested.Where(s => !UnitInfoSections.Contains(s) && !UnitInfoExtraSections.Contains(s)).ToList();
                if (unknown.Count > 0)
                {
                    throw new ArgumentException(
                        $"unknown section(s) {string.Join(", ", unknown)}; choose from {string.Join(", ", UnitInfoSections.Concat(UnitInfoExtraSections))}", nameof(sections));
                }
            }

            var fromUnit = requested?.Where(UnitInfoSections.Contains).ToList();
            var withLimits = requested is null || requested.Contains("limits");
            Dictionary<string, object?> result;
            if (fromUnit != null && fromUnit.Count == 0)
            {
                result = Skeleton(requested!);
            }
            else
            {
                var names = fromUnit is null ? null : Utf8.Encode(string.Join(",", fromUnit), nameof(sections));
                var json = TextOnce(
                    (byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_axis_read_unit_info_json(H, names, b, n, out r),
                    () => Text(ThreadResultJson));
                result = JsonPlain.DictOf(json);
            }

            var errors = result.TryGetValue("errors", out var existing) && existing is List<object?> list ? list : new List<object?>();
            result["errors"] = errors;
            if (result.TryGetValue("positions", out var positions) && positions is Dictionary<string, object?> rows && !rows.ContainsKey("stored_rows"))
            {
                try
                {
                    rows["stored_rows"] = (long)MaxPositions();
                }
                catch (EcDriverException exc)
                {
                    errors.Add($"positions: stored_rows: {exc.GetType().Name}: {exc.Message}");
                }
            }

            if (withLimits)
            {
                try
                {
                    var d = Limits().ToDictionary();
                    d["axis_limits"] = AxisLimits?.ToDictionary();
                    result["limits"] = d;
                }
                catch (EcDriverException exc)
                {
                    result["limits"] = null;
                    errors.Add($"limits: {exc.GetType().Name}: {exc.Message}");
                }

                if (result.TryGetValue("meta", out var meta) && meta is Dictionary<string, object?> m && m.TryGetValue("sections", out var s) && s is List<object?> names2 && !names2.Contains("limits"))
                {
                    names2.Add("limits");
                }
            }
            else
            {
                result["limits"] = null;
            }

            return result;
        }

        /// <summary>The dictionary of a call that asks the unit for nothing (only the extra sections were named).</summary>
        private static Dictionary<string, object?> Skeleton(IEnumerable<string> wanted)
        {
            var meta = new Dictionary<string, object?>
            {
                ["schema"] = UnitInfoSchema,
                ["read_only"] = true,
                ["function_codes"] = new List<object?> { "03", "11" },
                ["taken_at"] = DateTimeOffset.Now.ToString("yyyy-MM-ddTHH:mm:sszzz", CultureInfo.InvariantCulture),
                ["driver_version"] = NativeDriver.LibraryVersion,
                ["sections"] = wanted.Cast<object?>().ToList(),
                ["duration_s"] = 0L,
            };
            var result = new Dictionary<string, object?> { ["meta"] = meta };
            foreach (var name in new[] { "identity", "parameters", "monitor", "status_words", "io", "control_words", "alarms", "maintenance", "positions" })
            {
                result[name] = null;
            }

            result["errors"] = new List<object?>();
            return result;
        }
    }
}
