using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver.Cli
{
    /// <summary>A command line that cannot be run as given (exit code 2, like argparse).</summary>
    internal sealed class UsageException : Exception
    {
        public UsageException(string message)
            : base(message)
        {
        }
    }

    /// <summary>
    /// The parsed command line: positionals (command, its arguments) and <c>--options</c> (anywhere on the line,
    /// <c>--name value</c> or <c>--name=value</c>). A token such as <c>-5</c> or <c>-0.5</c> is a number, not an option.
    /// </summary>
    internal sealed class CliArgs
    {
        /// <summary>Every option and how many values it takes.</summary>
        public static readonly IReadOnlyDictionary<string, int> Arity = new Dictionary<string, int>(StringComparer.Ordinal)
        {
            // connection / global
            ["--port"] = 1,
            ["--sim"] = 1,
            ["--slave"] = 1,
            ["--axis"] = 1,
            ["--baud"] = 1,
            ["--timeout"] = 1,
            ["--retries"] = 1,
            ["--strategy"] = 1,
            ["--stroke"] = 2,
            ["--max-speed"] = 1,
            ["--max-accel"] = 1,
            ["--max-push"] = 1,
            ["--unit-max"] = 0,
            ["--log-frames"] = 0,
            ["--json"] = 0,
            ["--help"] = 0,
            ["--version"] = 0,

            // command options
            ["--section"] = 1,
            ["--count"] = 1,
            ["--interval"] = 1,
            ["--method"] = 1,
            ["--speed"] = 1,
            ["--accel"] = 1,
            ["--decel"] = 1,
            ["--band"] = 1,
            ["--no-wait"] = 0,
            ["--toward-home"] = 0,
            ["--speed-pct"] = 1,
            ["--allow-fast"] = 0,
            ["--safety"] = 0,
            ["--allow-unverified-code"] = 0,  // accepted and ignored (no longer needed)
            ["--no-commit"] = 0,
            ["--allow-dangerous"] = 0,  // accepted and ignored (no longer needed)
            ["--allow-home-params"] = 0,  // accepted and ignored (no longer needed)
            ["--allow-unlisted"] = 0,  // accepted and ignored (no longer needed)
            ["--force"] = 0,
            ["--motion"] = 0,
            ["--report"] = 1,
            ["--rows"] = 1,
            ["--backup"] = 1,
            ["--target"] = 1,
            ["--restore-baud"] = 1,
            ["--no-control"] = 0,
            ["--wait"] = 1,
            ["--watch"] = 0,
            ["--duration"] = 1,
            ["--fast"] = 0,
            ["--change-speed"] = 1,
            ["--change-accel"] = 1,
            ["--change-after"] = 1,
            ["--allow-nonvolatile"] = 0,  // accepted and ignored (no longer needed)

            // port commands
            ["--ports"] = 1,
            ["--slaves"] = 1,
            ["--scan-timeout"] = 1,
            ["--sim-slaves"] = 1,
        };

        private readonly Dictionary<string, List<string[]>> _options = new Dictionary<string, List<string[]>>(StringComparer.Ordinal);

        private CliArgs()
        {
        }

        /// <summary>Command and its arguments, in order.</summary>
        public List<string> Positionals { get; } = new List<string>();

        /// <summary>Every option given.</summary>
        public IEnumerable<string> OptionNames => _options.Keys;

        public static CliArgs Parse(IReadOnlyList<string> argv)
        {
            var args = new CliArgs();
            for (var i = 0; i < argv.Count; i++)
            {
                var token = argv[i] ?? string.Empty;
                if (token == "--")
                {
                    args.Positionals.AddRange(argv.Skip(i + 1));
                    break;
                }

                if (token == "-h")
                {
                    token = "--help";
                }

                if (token.StartsWith("--", StringComparison.Ordinal))
                {
                    var eq = token.IndexOf('=');
                    var name = eq > 0 ? token.Substring(0, eq) : token;
                    var inline = eq > 0 ? token.Substring(eq + 1) : null;
                    if (!Arity.TryGetValue(name, out var n))
                    {
                        throw new UsageException($"unknown option {name}");
                    }

                    var values = new List<string>();
                    if (inline != null)
                    {
                        if (n == 0)
                        {
                            throw new UsageException($"{name} takes no value");
                        }

                        values.Add(inline);
                    }

                    while (values.Count < n)
                    {
                        if (i + 1 >= argv.Count)
                        {
                            throw new UsageException(n == 1 ? $"{name} needs a value" : $"{name} needs {n} values");
                        }

                        values.Add(argv[++i]);
                    }

                    if (!args._options.TryGetValue(name, out var list))
                    {
                        list = new List<string[]>();
                        args._options[name] = list;
                    }

                    list.Add(values.ToArray());
                    continue;
                }

                if (token.Length > 1 && token[0] == '-' && !IsNumber(token))
                {
                    throw new UsageException($"unknown option {token}");
                }

                args.Positionals.Add(token);
            }

            return args;
        }

        public bool Has(string name) => _options.ContainsKey(name);

        /// <summary>The value of a one-value option (the last one when given twice), or <c>null</c>.</summary>
        public string? Value(string name) => _options.TryGetValue(name, out var list) ? list[list.Count - 1][0] : null;

        /// <summary>Every value of a repeatable one-value option, in order.</summary>
        public IReadOnlyList<string> Values(string name) => _options.TryGetValue(name, out var list) ? list.Select(v => v[0]).ToList() : new List<string>();

        /// <summary>The values of a several-value option (the last occurrence), or <c>null</c>.</summary>
        public string[]? Tuple(string name) => _options.TryGetValue(name, out var list) ? list[list.Count - 1] : null;

        public double? Double(string name)
        {
            var text = Value(name);
            return text is null ? (double?)null : ParseDouble(name, text);
        }

        public int? Int(string name)
        {
            var text = Value(name);
            return text is null ? (int?)null : ParseInt(name, text);
        }

        public static double ParseDouble(string what, string text)
        {
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value) && !double.IsNaN(value) && !double.IsInfinity(value))
            {
                return value;
            }

            throw new UsageException($"{what}: '{text}' is not a number");
        }

        public static int ParseInt(string what, string text)
        {
            if (int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var value))
            {
                return value;
            }

            throw new UsageException($"{what}: '{text}' is not an integer");
        }

        /// <summary>"1-4,7" -&gt; [1, 2, 3, 4, 7] (addresses 1..247, duplicates dropped).</summary>
        public static IReadOnlyList<int> SlaveList(string what, string text)
        {
            var result = new List<int>();
            foreach (var raw in text.Split(','))
            {
                var part = raw.Trim();
                if (part.Length == 0)
                {
                    continue;
                }

                var dash = part.IndexOf('-', 1);
                var first = ParseInt(what, dash > 0 ? part.Substring(0, dash) : part);
                var last = dash > 0 ? ParseInt(what, part.Substring(dash + 1)) : first;
                if (!(first >= 1 && first <= last && last <= 247))
                {
                    throw new UsageException($"{what}: bad slave range '{part}' (addresses 1..247)");
                }

                for (var s = first; s <= last; s++)
                {
                    if (!result.Contains(s))
                    {
                        result.Add(s);
                    }
                }
            }

            if (result.Count == 0)
            {
                throw new UsageException($"{what}: no slave address given");
            }

            return result;
        }

        /// <summary>Refuse options the command does not take.</summary>
        public void Allow(string command, IEnumerable<string> allowed)
        {
            var set = new HashSet<string>(allowed, StringComparer.Ordinal) { "--help", "--version" };
            var bad = _options.Keys.Where(k => !set.Contains(k)).ToList();
            if (bad.Count > 0)
            {
                throw new UsageException($"'{command}' does not take {string.Join(", ", bad)}");
            }
        }

        private static bool IsNumber(string token) =>
            double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out _);
    }
}
