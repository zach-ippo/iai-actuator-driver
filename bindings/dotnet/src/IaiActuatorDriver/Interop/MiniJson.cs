using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// A small JSON reader for the library's own outputs (RFC 8259): objects become <see cref="JsonObject"/> (ordinal
    /// keys in document order), arrays <see cref="JsonArray"/> (a <see cref="List{T}"/>), numbers <see cref="double"/>,
    /// strings, booleans and <c>null</c>. A container remembers which of its numbers were written as floats (with a
    /// decimal point or an exponent: <see cref="JsonObject.IsFloat"/>, <see cref="JsonArray.IsFloat"/>), so
    /// <see cref="JsonPlain"/> can keep <c>3.0</c> a float and <c>3</c> an integer. It keeps the
    /// netstandard2.0 build free of package dependencies; callers who want a full JSON library parse the *Json methods'
    /// strings themselves.
    /// </summary>
    internal static class MiniJson
    {
        /// <summary>Parse one JSON value; <see cref="FormatException"/> when the text is not JSON.</summary>
        internal static object? Parse(string text)
        {
            var reader = new Reader(text ?? throw new ArgumentNullException(nameof(text)));
            reader.SkipSpace();
            var value = reader.ReadValue();
            reader.SkipSpace();
            if (!reader.AtEnd)
            {
                throw reader.Error("text after the JSON value");
            }

            return value;
        }

        /// <summary><see cref="Parse"/>, <c>null</c> instead of an exception (for the last-error JSON, best effort).</summary>
        internal static object? TryParse(string? text)
        {
            if (string.IsNullOrEmpty(text))
            {
                return null;
            }

            try
            {
                return Parse(text!);
            }
            catch (FormatException)
            {
                return null;
            }
        }

        private sealed class Reader
        {
            private readonly string _text;
            private int _pos;

            internal Reader(string text) => _text = text;

            internal bool AtEnd => _pos >= _text.Length;

            /// <summary>The number just read was written as a float (a '.', 'e' or 'E' in its token).</summary>
            private bool _floatToken;

            internal FormatException Error(string what) =>
                new FormatException(string.Format(CultureInfo.InvariantCulture, "invalid JSON at offset {0}: {1}", _pos, what));

            internal void SkipSpace()
            {
                while (_pos < _text.Length && (_text[_pos] == ' ' || _text[_pos] == '\t' || _text[_pos] == '\n' || _text[_pos] == '\r'))
                {
                    _pos++;
                }
            }

            internal object? ReadValue()
            {
                if (AtEnd)
                {
                    throw Error("unexpected end");
                }

                switch (_text[_pos])
                {
                    case '{':
                        return ReadObject();
                    case '[':
                        return ReadArray();
                    case '"':
                        return ReadString();
                    case 't':
                        Expect("true");
                        return true;
                    case 'f':
                        Expect("false");
                        return false;
                    case 'n':
                        Expect("null");
                        return null;
                    default:
                        return ReadNumber();
                }
            }

            private JsonObject ReadObject()
            {
                var result = new JsonObject();
                _pos++;
                SkipSpace();
                if (!AtEnd && _text[_pos] == '}')
                {
                    _pos++;
                    return result;
                }

                while (true)
                {
                    SkipSpace();
                    if (AtEnd || _text[_pos] != '"')
                    {
                        throw Error("expected a key");
                    }

                    var key = ReadString();
                    SkipSpace();
                    if (AtEnd || _text[_pos] != ':')
                    {
                        throw Error("expected ':'");
                    }

                    _pos++;
                    SkipSpace();
                    var value = ReadValue();
                    result.Set(key, value);
                    if (value is double && _floatToken)
                    {
                        result.MarkFloat(key);
                    }

                    SkipSpace();
                    if (AtEnd)
                    {
                        throw Error("unexpected end in an object");
                    }

                    if (_text[_pos] == ',')
                    {
                        _pos++;
                        continue;
                    }

                    if (_text[_pos] == '}')
                    {
                        _pos++;
                        return result;
                    }

                    throw Error("expected ',' or '}'");
                }
            }

            private JsonArray ReadArray()
            {
                var result = new JsonArray();
                _pos++;
                SkipSpace();
                if (!AtEnd && _text[_pos] == ']')
                {
                    _pos++;
                    return result;
                }

                while (true)
                {
                    SkipSpace();
                    var item = ReadValue();
                    if (item is double && _floatToken)
                    {
                        result.MarkFloat(result.Count);
                    }

                    result.Add(item);
                    SkipSpace();
                    if (AtEnd)
                    {
                        throw Error("unexpected end in an array");
                    }

                    if (_text[_pos] == ',')
                    {
                        _pos++;
                        continue;
                    }

                    if (_text[_pos] == ']')
                    {
                        _pos++;
                        return result;
                    }

                    throw Error("expected ',' or ']'");
                }
            }

            private string ReadString()
            {
                _pos++;
                var sb = new StringBuilder();
                while (true)
                {
                    if (AtEnd)
                    {
                        throw Error("unterminated string");
                    }

                    var c = _text[_pos++];
                    if (c == '"')
                    {
                        return sb.ToString();
                    }

                    if (c != '\\')
                    {
                        sb.Append(c);
                        continue;
                    }

                    if (AtEnd)
                    {
                        throw Error("unterminated escape");
                    }

                    var e = _text[_pos++];
                    switch (e)
                    {
                        case '"': sb.Append('"'); break;
                        case '\\': sb.Append('\\'); break;
                        case '/': sb.Append('/'); break;
                        case 'b': sb.Append('\b'); break;
                        case 'f': sb.Append('\f'); break;
                        case 'n': sb.Append('\n'); break;
                        case 'r': sb.Append('\r'); break;
                        case 't': sb.Append('\t'); break;
                        case 'u':
                            if (_pos + 4 > _text.Length)
                            {
                                throw Error("short \\u escape");
                            }

                            sb.Append((char)int.Parse(_text.Substring(_pos, 4), NumberStyles.HexNumber, CultureInfo.InvariantCulture));
                            _pos += 4;
                            break;
                        default:
                            throw Error("unknown escape");
                    }
                }
            }

            private double ReadNumber()
            {
                var start = _pos;
                while (_pos < _text.Length && "+-0123456789.eE".IndexOf(_text[_pos]) >= 0)
                {
                    _pos++;
                }

                if (_pos == start
                    || !double.TryParse(_text.Substring(start, _pos - start), NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
                {
                    throw Error("expected a value");
                }

                _floatToken = _text.IndexOfAny(new[] { '.', 'e', 'E' }, start, _pos - start) >= 0;
                return value;
            }

            private void Expect(string word)
            {
                if (string.CompareOrdinal(_text, _pos, word, 0, word.Length) != 0)
                {
                    throw Error("expected " + word);
                }

                _pos += word.Length;
            }
        }
    }

    /// <summary>A JSON object of <see cref="MiniJson"/>: keys in document order, typed getters that tolerate absence.</summary>
    internal sealed class JsonObject
    {
        private readonly Dictionary<string, object?> _values = new Dictionary<string, object?>(StringComparer.Ordinal);
        private readonly List<string> _keys = new List<string>();

        internal IReadOnlyList<string> Keys => _keys;

        internal void Set(string key, object? value)
        {
            if (!_values.ContainsKey(key))
            {
                _keys.Add(key);
            }

            _values[key] = value;
        }

        internal bool Has(string key) => _values.ContainsKey(key);

        private HashSet<string>? _floats;

        /// <summary>Note that the number under <paramref name="key"/> was written as a float (see <see cref="MiniJson"/>).</summary>
        internal void MarkFloat(string key) => (_floats ??= new HashSet<string>(StringComparer.Ordinal)).Add(key);

        /// <summary>The number under <paramref name="key"/> was written with a decimal point or an exponent (<c>3.0</c>, not <c>3</c>).</summary>
        internal bool IsFloat(string key) => _floats != null && _floats.Contains(key);

        internal object? Get(string key) => _values.TryGetValue(key, out var value) ? value : null;

        internal string? String(string key) => Get(key) as string;

        internal double? Number(string key) => Get(key) is double d ? d : (double?)null;

        internal int? Int(string key) => Get(key) is double d && d >= int.MinValue && d <= int.MaxValue ? (int)d : (int?)null;

        internal long? Long(string key) => Get(key) is double d ? (long)d : (long?)null;

        internal bool? Bool(string key) => Get(key) is bool b ? b : (bool?)null;

        internal JsonObject? Object(string key) => Get(key) as JsonObject;

        internal List<object?>? Array(string key) => Get(key) as List<object?>;

        /// <summary>The strings of an array value (non-strings skipped); empty when absent.</summary>
        internal IReadOnlyList<string> Strings(string key)
        {
            var result = new List<string>();
            if (Array(key) is { } items)
            {
                foreach (var item in items)
                {
                    if (item is string s)
                    {
                        result.Add(s);
                    }
                }
            }

            return result;
        }
    }

    /// <summary>A JSON array of <see cref="MiniJson"/>: a list that also remembers which numbers were written as floats.</summary>
    internal sealed class JsonArray : List<object?>
    {
        private HashSet<int>? _floats;

        /// <summary>Note that the number at <paramref name="index"/> was written as a float (see <see cref="MiniJson"/>).</summary>
        internal void MarkFloat(int index) => (_floats ??= new HashSet<int>()).Add(index);

        /// <summary>The number at <paramref name="index"/> was written with a decimal point or an exponent.</summary>
        internal bool IsFloat(int index) => _floats != null && _floats.Contains(index);
    }
}
