using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The library's JSON outputs as plain .NET values (dictionaries, lists, strings, numbers, booleans, <c>null</c>: what
    /// System.Text.Json and Newtonsoft serialise as they are), and back to text. A number written without a decimal point
    /// or an exponent (<c>3</c>) becomes a <see cref="long"/> when it fits, so a count is an integer; one written with
    /// them (<c>3.0</c>) stays a <see cref="double"/>, as the Python edition keeps it a float. A number whose token is not
    /// known (a value that did not come from <see cref="MiniJson"/>) is a <see cref="long"/> when it is whole.
    /// </summary>
    internal static class JsonPlain
    {
        /// <summary>A parsed value as plain values (objects keep their document order).</summary>
        internal static object? Plain(object? value)
        {
            switch (value)
            {
                case JsonObject obj:
                    return Dict(obj);
                case List<object?> list:
                    var parsed = list as JsonArray;
                    var items = new List<object?>(list.Count);
                    for (var i = 0; i < list.Count; i++)
                    {
                        items.Add(Number(list[i], parsed is null ? (bool?)null : parsed.IsFloat(i)));
                    }

                    return items;
                default:
                    return Number(value, null);
            }
        }

        /// <summary>A value as a plain one; <paramref name="isFloat"/>: the number was written as a float (null: not known).</summary>
        private static object? Number(object? value, bool? isFloat)
        {
            if (value is double d && isFloat != true && !double.IsNaN(d) && !double.IsInfinity(d) && Math.Floor(d) == d && Math.Abs(d) < 9e15)
            {
                return (long)d;
            }

            return value is JsonObject || value is List<object?> ? Plain(value) : value;
        }

        /// <summary>A parsed object as an ordered dictionary of plain values.</summary>
        internal static Dictionary<string, object?> Dict(JsonObject obj)
        {
            var result = new Dictionary<string, object?>(StringComparer.Ordinal);
            foreach (var key in obj.Keys)
            {
                result[key] = Number(obj.Get(key), obj.IsFloat(key));
            }

            return result;
        }

        /// <summary>The plain values of a JSON text; an empty dictionary when it is not an object.</summary>
        internal static Dictionary<string, object?> DictOf(string? json) =>
            MiniJson.TryParse(json) is JsonObject obj ? Dict(obj) : new Dictionary<string, object?>(StringComparer.Ordinal);

        /// <summary>JSON text of plain values (dictionaries, lists, strings, numbers, booleans, null), indented by two spaces.</summary>
        internal static string Write(object? value, bool indented = true)
        {
            var sb = new StringBuilder();
            Write(sb, value, indented ? 0 : -1);
            return sb.ToString();
        }

        private static void Write(StringBuilder sb, object? value, int depth)
        {
            switch (value)
            {
                case null:
                    sb.Append("null");
                    break;
                case string s:
                    Quote(sb, s);
                    break;
                case bool b:
                    sb.Append(b ? "true" : "false");
                    break;
                case double d:
                    sb.Append(double.IsNaN(d) || double.IsInfinity(d) ? "null" : Float(d.ToString("R", CultureInfo.InvariantCulture)));
                    break;
                case float f:
                    sb.Append(float.IsNaN(f) || float.IsInfinity(f) ? "null" : Float(f.ToString("R", CultureInfo.InvariantCulture)));
                    break;
                case IFormattable number when value is int || value is long || value is uint || value is ulong || value is short || value is ushort || value is byte:
                    sb.Append(number.ToString(null, CultureInfo.InvariantCulture));
                    break;
                case IDictionary dict:
                    Container(sb, depth, '{', '}', dict.Count, w =>
                    {
                        var first = true;
                        foreach (DictionaryEntry entry in dict)
                        {
                            Separator(sb, w, ref first);
                            Quote(sb, Convert.ToString(entry.Key, CultureInfo.InvariantCulture) ?? string.Empty);
                            sb.Append(depth >= 0 ? ": " : ":");
                            Write(sb, entry.Value, depth < 0 ? -1 : depth + 1);
                        }
                    });
                    break;
                case IEnumerable seq:
                    var list = new List<object?>();
                    foreach (var item in seq)
                    {
                        list.Add(item);
                    }

                    Container(sb, depth, '[', ']', list.Count, w =>
                    {
                        var first = true;
                        foreach (var item in list)
                        {
                            Separator(sb, w, ref first);
                            Write(sb, item, depth < 0 ? -1 : depth + 1);
                        }
                    });
                    break;
                default:
                    Quote(sb, Convert.ToString(value, CultureInfo.InvariantCulture) ?? string.Empty);
                    break;
            }
        }

        /// <summary>A float's text as the Python edition writes it: a whole value keeps a decimal point (<c>100.0</c>), so a reader keeps it a float.</summary>
        private static string Float(string text) => text.IndexOfAny(new[] { '.', 'E', 'e' }) >= 0 ? text : text + ".0";

        private static void Container(StringBuilder sb, int depth, char open, char close, int count, Action<int> body)
        {
            sb.Append(open);
            if (count > 0)
            {
                body(depth);
                if (depth >= 0)
                {
                    sb.Append('\n').Append(' ', 2 * depth);
                }
            }

            sb.Append(close);
        }

        private static void Separator(StringBuilder sb, int depth, ref bool first)
        {
            if (!first)
            {
                sb.Append(',');
            }

            first = false;
            if (depth >= 0)
            {
                sb.Append('\n').Append(' ', 2 * (depth + 1));
            }
        }

        private static void Quote(StringBuilder sb, string text)
        {
            sb.Append('"');
            foreach (var c in text)
            {
                switch (c)
                {
                    case '"':
                        sb.Append("\\\"");
                        break;
                    case '\\':
                        sb.Append("\\\\");
                        break;
                    case '\n':
                        sb.Append("\\n");
                        break;
                    case '\r':
                        sb.Append("\\r");
                        break;
                    case '\t':
                        sb.Append("\\t");
                        break;
                    default:
                        if (c < 0x20)
                        {
                            sb.Append("\\u").Append(((int)c).ToString("x4", CultureInfo.InvariantCulture));
                        }
                        else
                        {
                            sb.Append(c);
                        }

                        break;
                }
            }

            sb.Append('"');
        }
    }
}
