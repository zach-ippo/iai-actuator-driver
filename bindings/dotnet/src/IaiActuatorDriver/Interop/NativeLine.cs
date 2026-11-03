using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// (ABI 0.5) The line a bus carries (iad_line; C++ Line): its kind and rules, the gateway behind it, the position units
    /// of an RCON line and the explanations of unreachable axes. Operation names are the core's ("broadcast",
    /// "baud_change", "iai_command", "iai_command2", "software_reset", "action_mode", "home_iai", "coil",
    /// "report_slave_id", "axis_control"). A refusal is <see cref="UnsafeCommandException"/>. Every gateway rule is DOC
    /// only: UNVERIFIED on hardware. No public C# API yet (the gateway lines come in the API round after the switch).
    /// </summary>
    internal sealed class NativeLine : IDisposable
    {
        private readonly LineHandle _handle;

        private NativeLine(LineHandle handle)
        {
            _handle = handle;
        }

        /// <summary>The native handle.</summary>
        internal LineHandle Handle => _handle;

        /// <summary>
        /// A new line of <paramref name="kind"/> (Line's constructor), not attached to a bus; <paramref name="gateway"/> the
        /// gateway behind it (null: none), <paramref name="detected"/> a probe found it.
        /// </summary>
        internal static NativeLine Open(NativeLineKind kind, NativeGateway? gateway = null, bool detected = false, double? positionUnitMm = null)
        {
            NativeDriver.EnsureLoaded();
            var added = false;
            try
            {
                gateway?.Handle.DangerousAddRef(ref added);
                var token = gateway is null ? IntPtr.Zero : gateway.Handle.DangerousGetHandle();
                return Adopt(NativeMethods.iad_line_open((int)kind, token, Known.Flag(detected), Known.OrNaN(positionUnitMm), out var handle), handle);
            }
            finally
            {
                if (added)
                {
                    gateway!.Handle.DangerousRelease();
                }
            }
        }

        /// <summary>The line <paramref name="bus"/> carries now.</summary>
        internal static NativeLine Of(NativeBus bus) => Adopt(NativeMethods.iad_bus_line(bus.Handle, out var handle), handle);

        /// <summary>Every kind's rules, in IAD_LINE_DIRECT .. _ROBONET_GATEWAY order (no I/O).</summary>
        internal static string RulesJson()
        {
            NativeDriver.EnsureLoaded();
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_line_rules_json(b, s, out r));
        }

        /// <summary>Attach this line to <paramref name="bus"/>: its guard, gateway and position units serve every axis on it.</summary>
        internal void AttachTo(NativeBus bus) => NativeErrors.Check(NativeMethods.iad_bus_attach_line(bus.Handle, _handle));

        /// <summary>The line as JSON (kind, detected, rules, gateway, position_units, unreachable_slaves, explanation; no I/O).</summary>
        internal string Json() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_line_json(_handle, b, s, out r));

        /// <summary>The line carries <paramref name="operation"/>.</summary>
        internal bool Allows(string operation)
        {
            NativeErrors.Check(NativeMethods.iad_line_allows(_handle, Utf8.EncodeRequired(operation, nameof(operation)), out var allowed));
            return Known.Bool(allowed);
        }

        /// <summary>Refuse <paramref name="operation"/> when the line does not carry it (<paramref name="what"/>: null = the operation).</summary>
        internal void Require(string operation, string? what = null) =>
            NativeErrors.Check(NativeMethods.iad_line_require(_handle, Utf8.EncodeRequired(operation, nameof(operation)), Utf8.Encode(what, nameof(what))));

        /// <summary>The bus's guard on <paramref name="pdu"/> for <paramref name="slave"/> (0 = broadcast).</summary>
        internal void CheckRequest(int slave, byte[] pdu) =>
            NativeErrors.Check(NativeMethods.iad_line_check_request(_handle, slave, pdu, NativeText.Size(pdu.Length)));

        /// <summary>An RCON line reads its gateway's MANU flag (cached); nothing else is sent. <paramref name="slave"/> 0: none named.</summary>
        internal void RequireControl(int slave = 0) => NativeBus.Run(() => NativeMethods.iad_line_require_control(_handle, slave));

        /// <summary>The position unit of <paramref name="slave"/> (0: the line's) and where it came from; null when not known.</summary>
        internal (double UnitMm, string Source)? PositionUnit(int slave = 0)
        {
            var unit = double.NaN;
            var source = NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_line_position_unit(_handle, slave, out unit, b, s, out r), 64);
            return double.IsNaN(unit) ? ((double, string)?)null : (unit, source);
        }

        /// <summary>State the position unit (0.01 or 0.001 mm) of <paramref name="slave"/> (0: every axis); <paramref name="source"/> "stated" or "read".</summary>
        internal void SetPositionUnit(double unitMm, int slave = 0, string? source = null) =>
            NativeErrors.Check(NativeMethods.iad_line_set_position_unit(_handle, unitMm, slave, Utf8.Encode(source, nameof(source))));

        /// <summary>The position unit <paramref name="slave"/> moves in, or the refusal (<paramref name="control"/>: motion, else a position read).</summary>
        internal double RequirePositionUnit(int slave, bool control)
        {
            NativeErrors.Check(NativeMethods.iad_line_require_position_unit(_handle, slave, Known.Flag(control), out var unit));
            return unit;
        }

        /// <summary>Note <paramref name="slaves"/> as linked but silent.</summary>
        internal void NoteUnreachable(IEnumerable<int> slaves) =>
            NativeErrors.Check(NativeMethods.iad_line_note_unreachable(_handle, Utf8.EncodeRequired(NativeBus.Csv(slaves), nameof(slaves))));

        /// <summary>Why <paramref name="slaves"/> (null: the recorded ones) do not answer on this line; "" when there is nothing to explain.</summary>
        internal string ExplainUnreachable(IEnumerable<int>? slaves = null)
        {
            var s = slaves is null ? null : Utf8.Encode(NativeBus.Csv(slaves));
            return NativeText.Read((byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_line_explain_unreachable(_handle, s, b, n, out r));
        }

        /// <summary>The line's gateway; null when it has none.</summary>
        internal NativeGateway? Gateway()
        {
            var result = NativeMethods.iad_line_gateway(_handle, out var gateway);
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                gateway.Dispose();
                throw error;
            }

            if (gateway.IsInvalid)
            {
                gateway.Dispose();
                return null;
            }

            return new NativeGateway(gateway);
        }

        /// <summary>Release the handle (the line lives on while a bus carries it).</summary>
        public void Dispose() => _handle.Dispose();

        private static NativeLine Adopt(int result, LineHandle handle)
        {
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return new NativeLine(handle);
        }
    }

    /// <summary>
    /// (ABI 0.5) An RCON or ROBONET gateway of a line (iad_gateway; C++ RconGateway / RoboNetGateway): read-only, never
    /// written. A handle keeps the bus its gateway reads through alive. DOC only (ME0413, ME0208): UNVERIFIED on hardware.
    /// </summary>
    internal sealed class NativeGateway : IDisposable
    {
        /// <summary>Most words a gateway status has.</summary>
        private const int StatusWords = 64;

        private readonly GatewayHandle _handle;

        internal NativeGateway(GatewayHandle handle)
        {
            _handle = handle;
        }

        /// <summary>The native handle.</summary>
        internal GatewayHandle Handle => _handle;

        /// <summary>
        /// A gateway object on <paramref name="bus"/> (<paramref name="kind"/> Rcon: <paramref name="address"/> 0 = D0h,
        /// <paramref name="cacheS"/> null = 2 s; RobonetGateway: both ignored). Nothing is sent.
        /// </summary>
        internal static NativeGateway Open(NativeBus bus, NativeLineKind kind, int address = 0, double? cacheS = null)
        {
            var result = NativeMethods.iad_gateway_open(bus.Handle, (int)kind, address, null, UIntPtr.Zero, double.NaN, Known.OrNaN(cacheS), out var handle);
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return new NativeGateway(handle);
        }

        /// <summary>{"kind", "address", "cache_s", "words", "read_at"} (no I/O).</summary>
        internal string InfoJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_gateway_info_json(_handle, b, s, out r));

        /// <summary>The status words (READ-ONLY): the cached ones when younger than <paramref name="maxAgeS"/> (null: the gateway's default).</summary>
        internal (ushort[] Words, double ReadAt) Status(double? maxAgeS = null)
        {
            var words = new ushort[StatusWords];
            var count = UIntPtr.Zero;
            var readAt = double.NaN;
            NativeBus.Run(() => NativeMethods.iad_gateway_status(_handle, Known.OrNaN(maxAgeS), words, NativeText.Size(words.Length), out count, out readAt));
            return (words.Take(NativeText.ToInt(count)).ToArray(), readAt);
        }

        /// <summary>The RCON gateway's status, or <see cref="EcNotReadyException"/> unless it is in MANU (manual) mode.</summary>
        internal (ushort[] Words, double ReadAt) RequireManu()
        {
            var words = new ushort[StatusWords];
            var count = UIntPtr.Zero;
            var readAt = double.NaN;
            NativeBus.Run(() => NativeMethods.iad_gateway_require_manu(_handle, words, NativeText.Size(words.Length), out count, out readAt));
            return (words.Take(NativeText.ToInt(count)).ToArray(), readAt);
        }

        /// <summary>The RCON gateway's alarm code (0 = none). READ-ONLY.</summary>
        internal uint ReadAlarm()
        {
            uint code = 0;
            NativeBus.Run(() => NativeMethods.iad_gateway_read_alarm(_handle, out code));
            return code;
        }

        /// <summary>How long the RCON gateway's MANU read is trusted (s, &gt;= 0).</summary>
        internal void SetCacheS(double cacheS) => NativeErrors.Check(NativeMethods.iad_gateway_set_cache_s(_handle, cacheS));

        /// <summary>The RCON all-axes monitor of <paramref name="axes"/> (null: all 16) as JSON {"&lt;axis&gt;": [15 words]}. READ-ONLY.</summary>
        internal string ReadMonitorJson(IEnumerable<int>? axes = null)
        {
            var a = axes is null ? null : Utf8.Encode(NativeBus.Csv(axes));
            return NativeBus.Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_gateway_read_monitor_json(_handle, a, b, s, out r));
        }

        /// <summary>Release the handle.</summary>
        public void Dispose() => _handle.Dispose();
    }

    /// <summary>(ABI 0.5) The bus side of the lines and the port: the line's rules stated, the gateway probes, the serial code.</summary>
    internal static class NativeBusLines
    {
        /// <summary>
        /// State what the axes on <paramref name="bus"/>'s line may send ({"name": "rcon", "coils": false, ...}; each key
        /// optional); null clears.
        /// </summary>
        internal static void StateRules(NativeBus bus, string? rulesJson) =>
            NativeErrors.Check(NativeMethods.iad_bus_state_line_rules(bus.Handle, Utf8.Encode(rulesJson, nameof(rulesJson))));

        /// <summary>
        /// One FC03 read of a gateway's status (<paramref name="line"/> Rcon: <paramref name="address"/> 0 = D0h;
        /// RobonetGateway), no retries; null when no gateway answered. READ-ONLY.
        /// </summary>
        internal static ushort[]? ProbeGateway(NativeBus bus, NativeLineKind line, int address = 0, double? timeoutS = null)
        {
            var words = new ushort[64];
            var count = UIntPtr.Zero;
            NativeBus.Run(() => NativeMethods.iad_bus_probe_gateway(bus.Handle, (int)line, address, Known.OrNaN(timeoutS), words, NativeText.Size(words.Length), out count, out _));
            var n = NativeText.ToInt(count);
            return n == 0 ? null : words.Take(n).ToArray();
        }

        /// <summary>The 0x9400 serial code of <paramref name="slave"/>; "" when refused or empty. READ-ONLY.</summary>
        internal static string ReadSerialCode(NativeBus bus, int slave) =>
            NativeBus.Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_bus_read_serial_code(bus.Handle, slave, b, s, out r));

        /// <summary>The ROBONET hint for a silent <paramref name="slave"/> (UNVERIFIED heuristic); "" when there is none. No I/O.</summary>
        internal static string RobonetHint(NativeBus bus, int slave, bool directoryTimedOut) =>
            NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_bus_robonet_hint(bus.Handle, slave, Known.Flag(directoryTimedOut), b, s, out r));

        /// <summary>Work out the port's line once (Port::detect_line; <paramref name="force"/> asks again): the kind the bus carries.</summary>
        internal static NativeLineKind DetectPortLine(NativeBus bus, bool force = false)
        {
            var kind = 0;
            NativeBus.Run(() => NativeMethods.iad_bus_port_detect_line(bus.Handle, Known.Flag(force), out kind));
            return (NativeLineKind)kind;
        }

        /// <summary>The addresses of <paramref name="slaves"/> that answer one probe each (Port::scan; the rate and line settled first). READ-ONLY.</summary>
        internal static IReadOnlyList<int> Scan(NativeBus bus, IEnumerable<int> slaves, double? timeoutS = null, bool useLinkMap = true)
        {
            var list = slaves.ToList();
            var found = new int[Math.Max(list.Count, 1)];
            var count = UIntPtr.Zero;
            var s = Utf8.Encode(NativeBus.Csv(list));
            NativeBus.Run(() => NativeMethods.iad_bus_scan(bus.Handle, s, Known.OrNaN(timeoutS), Known.Flag(useLinkMap), found, NativeText.Size(found.Length), out count));
            return found.Take(Math.Min(NativeText.ToInt(count), found.Length)).ToList().AsReadOnly();
        }

        /// <summary>What one address says about itself as a scan hit (FC11, or the serial code on RCON; a failure stays in "error"). READ-ONLY.</summary>
        internal static string IdentifyJson(NativeBus bus, int slave) =>
            NativeBus.Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_bus_identify_json(bus.Handle, slave, b, s, out r));
    }
}
