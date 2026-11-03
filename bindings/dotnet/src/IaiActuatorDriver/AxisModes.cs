using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver
{
    /// <summary>
    /// The acceleration pattern of a 0x9900 move (CTLF b6-b7). <see cref="Default"/> leaves it to the unit's own setting
    /// (a trapezoid). UNVERIFIED on hardware: the simulator accepts every pattern.
    /// </summary>
    public enum AccelPattern
    {
        /// <summary>Not set: trapezoid (CTLF b6-b7 = 00).</summary>
        Default = 0,

        /// <summary>A trapezoid speed profile.</summary>
        Trapezoid = 1,

        /// <summary>S-curve (CON No.56 ratio); refused above an acceleration time of 2 s.</summary>
        SMotion = 2,

        /// <summary>First-order filter (CON No.55 time constant).</summary>
        Filter = 3,
    }

    /// <summary>When a move counts as done.</summary>
    public enum CompletionRule
    {
        /// <summary>PEND, the position within the band and PNOW unchanged over <see cref="Axis.SettleS"/> (the default).</summary>
        Safe = 1,

        /// <summary>One sample: PEND, within the band, DSSE MOVE clear, not homing, not paused (what fast mode uses).</summary>
        Fast = 2,
    }

    /// <summary>What <see cref="Axis.EnableFast"/> / <see cref="Axis.DisableFast"/> did.</summary>
    /// <param name="BaudRate">The port's rate afterwards; <c>null</c> on a transport without one.</param>
    /// <param name="Completion">The axis's completion rule afterwards.</param>
    /// <param name="Monitor">A status monitor runs.</param>
    /// <param name="Notes">What could not be done (the baud rate kept, and why).</param>
    public sealed record FastModeSummary(int? BaudRate, CompletionRule Completion, bool Monitor, IReadOnlyList<string> Notes)
    {
        /// <summary>The summary as a JSON-friendly dictionary (the Python edition's keys).</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["baudrate"] = BaudRate,
            ["completion"] = Completion == CompletionRule.Fast ? "fast" : "safe",
            ["monitor"] = Monitor,
            ["notes"] = Notes.Cast<object?>().ToList(),
        };
    }

    /// <summary>
    /// One point of <see cref="Axis.MoveSequence(IEnumerable{SequencePoint}, double, double, double, double?, CompletionRule?, double?, System.Threading.CancellationToken)"/>:
    /// a position with its own speed, acceleration or band (<c>null</c> = the sequence's).
    /// </summary>
    public sealed record SequencePoint(double Position)
    {
        /// <summary>Speed for the move to this point (units/s); <c>null</c> = the sequence's.</summary>
        public double? Speed { get; init; }

        /// <summary>Acceleration for the move to this point (G); <c>null</c> = the sequence's.</summary>
        public double? AccelG { get; init; }

        /// <summary>In-position band for this point (units); <c>null</c> = the sequence's.</summary>
        public double? Band { get; init; }

        /// <summary>A point with the sequence's own speed, acceleration and band.</summary>
        public static implicit operator SequencePoint(double position) => new SequencePoint(position);
    }

    /// <summary>What a started move knows without any I/O: <see cref="MoveHandle.Info"/>.</summary>
    /// <param name="Target">The target (units), rounded to 0.001.</param>
    /// <param name="Band">The in-position band (units).</param>
    /// <param name="TimeoutS">Seconds the move may take (the time left when a wait starts); <c>null</c> when not known.</param>
    /// <param name="Completion">The completion rule the move uses.</param>
    /// <param name="Cancelled">A decel stop was sent through the handle.</param>
    /// <param name="Result">The final status once known, else <c>null</c>.</param>
    public sealed record MoveInfo(double Target, double Band, double? TimeoutS, CompletionRule Completion, bool Cancelled, Status? Result)
    {
        /// <summary>The same as a JSON-friendly dictionary (the Python edition's keys).</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["target_mm"] = Target,
            ["band_mm"] = Band,
            ["completion"] = Completion == CompletionRule.Fast ? "fast" : "safe",
            ["timeout_s"] = TimeoutS,
            ["done"] = Result != null,
            ["cancelled"] = Cancelled,
            ["position_mm"] = Result?.PositionMm,
        };
    }

    internal static class AxisModeMap
    {
        internal static NativeAccelPattern ToNative(this AccelPattern pattern) => (NativeAccelPattern)(int)pattern;

        internal static NativeCompletion ToNative(this CompletionRule? rule) =>
            rule is { } value ? (NativeCompletion)(int)value : NativeCompletion.Default;

        internal static CompletionRule FromNative(this NativeCompletion rule) => rule == NativeCompletion.Fast ? CompletionRule.Fast : CompletionRule.Safe;
    }
}
