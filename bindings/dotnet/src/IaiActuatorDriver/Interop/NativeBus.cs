using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// (ABI 0.3) A Modbus RTU master of the native core on a transport (iad_bus; C++ ModbusRtu): one transaction at a
    /// time, the retries, the gaps, the frame sink. The thin layer under the public <see cref="ModbusRtu"/> facade. Every
    /// call that can reach a bridged transport (<see cref="TransportBridge"/>) raises what that transport threw, the way
    /// the managed edition did.
    /// </summary>
    internal sealed class NativeBus : IDisposable
    {
        private readonly BusHandle _handle;
        private readonly bool _owned;
        private long _frameSink;

        private NativeBus(BusHandle handle, bool owned)
        {
            _handle = handle;
            _owned = owned;
        }

        /// <summary>The native handle.</summary>
        internal BusHandle Handle => _handle;

        /// <summary>The handle was released.</summary>
        internal bool IsReleased => _handle.IsClosed;

        /// <summary>
        /// A bus on <paramref name="transport"/> (iad_bus_open; no I/O). The bus shares the transport: closing the bus closes
        /// it. The caller keeps (and releases) its own transport handle.
        /// </summary>
        internal static NativeBus Open(TransportHandle transport, int retries = ModbusRtu.DefaultRetries, double minGapS = 0.005)
        {
            var options = SizedStruct.Fresh<IadBusOptions>();
            NativeErrors.Check(NativeMethods.iad_bus_options_init(ref options));
            options.Retries = retries;
            options.MinGapS = minGapS;
            var result = NativeMethods.iad_bus_open(transport, ref options, out var handle);
            return Adopt(result, handle, owned: true);
        }

        /// <summary>
        /// Wrap a bus handle an open function returned (iad_bus_open, iad_axis_bus, iad_cell_bus), or throw its error.
        /// <paramref name="owned"/>: <see cref="Dispose"/> closes the bus (a bus this binding opened), else it only lets go
        /// of the handle.
        /// </summary>
        internal static NativeBus Adopt(int result, BusHandle handle, bool owned = false)
        {
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw error;
            }

            return new NativeBus(handle, owned);
        }

        /// <summary>
        /// Close (a bus from <see cref="Open"/>: with its transport; iad_bus_shutdown) and release the handle
        /// (iad_bus_release; a bus handle of an axis or a cell is only released). The frame sink is removed first. Safe to
        /// call twice. A bus object that is only collected releases its handle and never closes the port.
        /// </summary>
        public void Dispose()
        {
            if (_handle.IsClosed)
            {
                return;
            }

            try
            {
                if (_frameSink != 0)
                {
                    NativeMethods.iad_bus_set_frame_sink(_handle, null, IntPtr.Zero);
                }
            }
            catch (ObjectDisposedException)
            {
            }
            finally
            {
                NativeSinks.ForgetFrame(_frameSink);
                _frameSink = 0;
                try
                {
                    if (_owned)
                    {
                        NativeMethods.iad_bus_shutdown(_handle);
                    }
                }
                catch (ObjectDisposedException)
                {
                }
                finally
                {
                    _handle.Dispose();
                }
            }
        }

        // ------------------------------------------------------------------------------------------ settings

        /// <summary>The bus's settings and state as JSON (iad_bus_info_json; no I/O).</summary>
        internal string InfoJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_bus_info_json(_handle, b, s, out r));

        /// <summary>A new handle on the bus's transport (iad_bus_transport; dispose it when done).</summary>
        internal TransportHandle Transport()
        {
            var result = NativeMethods.iad_bus_transport(_handle, out var transport);
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                transport.Dispose();
                throw error;
            }

            return transport;
        }

        /// <summary>
        /// (0.6) Move the bus onto <paramref name="transport"/> in place (iad_bus_set_transport): its settings, line, frame
        /// sink and everything keyed by it (the axes and cells on it, their sessions and non-volatile counts) stay. Waits
        /// for the transaction in flight; nothing is sent. The old transport is neither closed nor released; the bus takes
        /// its own reference to the new one (dispose <paramref name="transport"/> when done with it).
        /// </summary>
        internal void SetTransport(TransportHandle transport) =>
            NativeErrors.Check(NativeMethods.iad_bus_set_transport(_handle, transport ?? throw new ArgumentNullException(nameof(transport))));

        internal void SetRetries(int retries) => NativeErrors.Check(NativeMethods.iad_bus_set_retries(_handle, retries));

        internal void SetMinGap(double seconds) => NativeErrors.Check(NativeMethods.iad_bus_set_min_gap(_handle, seconds));

        /// <summary>Install (not null) or remove (null) the bus's frame sink: every request, response and partial response.</summary>
        internal void SetFrameSink(Action<FrameDirection, byte[]>? handler)
        {
            var old = _frameSink;
            _frameSink = NativeSinks.SetBusFrame(_handle, handler);
            NativeSinks.ForgetFrame(old);
        }

        // ------------------------------------------------------------------------------------------ requests

        /// <summary>FC03 (iad_bus_read_registers); <paramref name="retries"/> null = the bus's.</summary>
        internal ushort[] ReadRegisters(int slave, int address, int count, int? retries = null)
        {
            var values = new ushort[Math.Max(count, 1)];
            Run(() => NativeMethods.iad_bus_read_registers(_handle, slave, address, count, retries ?? NativeMethods.IntUnknown, double.NaN, NativeMethods.IntUnknown, values));
            return values;
        }

        internal void WriteCoil(int slave, int address, bool on, int? retries = null) =>
            Run(() => NativeMethods.iad_bus_write_coil(_handle, slave, address, on ? 1 : 0, retries ?? NativeMethods.IntUnknown));

        internal void WriteRegister(int slave, int address, ushort value) =>
            Run(() => NativeMethods.iad_bus_write_register(_handle, slave, address, value));

        internal void WriteRegisters(int slave, int address, ushort[] values) =>
            Run(() => NativeMethods.iad_bus_write_registers(_handle, slave, address, values, NativeText.Size(values.Length)));

        internal ushort[] ReadWriteRegisters(int slave, int readAddress, int readCount, int writeAddress, ushort[] values)
        {
            var output = new ushort[Math.Max(readCount, 1)];
            Run(() => NativeMethods.iad_bus_read_write_registers(_handle, slave, readAddress, readCount, writeAddress, values, NativeText.Size(values.Length), output));
            return output;
        }

        /// <summary>FC11: the data bytes after the byte count (no retries by default; each attempt waits up to 0.65 s).</summary>
        internal byte[] ReportSlaveId(int slave, int? retries = null) => ReportSlaveId(slave, retries ?? 0, 0.65);

        /// <summary>IAI 0x45 (iad_bus_iai_command): (rejection, status).</summary>
        internal (int Rejection, int Status) IaiCommand(int slave, int group, int command, int op1, int op2, bool allowUnsafe, bool check, int? retries) =>
            IaiCommand(slave, group, command, op1, op2, (allowUnsafe ? NativeMethods.IaiAllowUnsafe : 0u) | (check ? 0u : NativeMethods.IaiNoCheck), retries);

        internal void BroadcastCoil(int address, bool on) => Run(() => NativeMethods.iad_bus_broadcast_coil(_handle, address, on ? 1 : 0));

        /// <summary>One FC03 of one register, no retries, at most <paramref name="timeoutS"/>: a valid frame came.</summary>
        internal bool Probe(int slave, double timeoutS, int address)
        {
            var answered = 0;
            Run(() => NativeMethods.iad_bus_probe(_handle, slave, timeoutS, address, out answered));
            return answered != 0;
        }

        /// <summary>
        /// The rate one of <paramref name="slaves"/> answers at (iad_bus_detect_baud_rate; candidates null = the default
        /// order, an empty list = the present rate only).
        /// </summary>
        internal int DetectBaudRate(IEnumerable<int> slaves, IEnumerable<int>? candidates) =>
            DetectBaudRate(slaves, candidates, NativeMethods.IntUnknown, true, null);

        internal void SetBaudRate(int rate) => Run(() => NativeMethods.iad_bus_set_baud_rate(_handle, rate));

        internal void SendBreak(double seconds) => Run(() => NativeMethods.iad_bus_send_break(_handle, seconds));

        internal void Reopen(int? baudRate = null) => Run(() => NativeMethods.iad_bus_reopen(_handle, baudRate ?? 0));

        // ------------------------------------------------------------------------------------------ 0.4

        /// <summary>
        /// (0.4) Hold the bus lock for the calling thread until <see cref="Unlock"/> (recursive; no request of another
        /// thread goes out meanwhile). <paramref name="timeoutS"/> null = as long as it takes; false when the time ran out.
        /// </summary>
        internal bool Lock(double? timeoutS = null)
        {
            var locked = 0;
            Run(() => NativeMethods.iad_bus_lock(_handle, timeoutS ?? double.NaN, out locked));
            return locked != 0;
        }

        /// <summary>(0.4) Give one hold of the bus lock back.</summary>
        internal void Unlock() => NativeErrors.Check(NativeMethods.iad_bus_unlock(_handle));

        /// <summary>(0.4) Suspend the status monitors of every axis on this bus; how many were suspended.</summary>
        internal int SuspendMonitors(double? timeoutS = null)
        {
            NativeErrors.Check(NativeMethods.iad_bus_suspend_monitors(_handle, timeoutS ?? double.NaN, out var monitors));
            return monitors;
        }

        /// <summary>(0.4) Resume the monitors of the last <see cref="SuspendMonitors"/>.</summary>
        internal void ResumeMonitors() => NativeErrors.Check(NativeMethods.iad_bus_resume_monitors(_handle));

        /// <summary>(0.4) IAI 0x45 with the opt-ins as IAD_IAI_* flags: (rejection, status).</summary>
        internal (int Rejection, int Status) IaiCommand(int slave, int group, int command, int op1, int op2, uint flags, int? retries)
        {
            var rejection = 0;
            var status = new uint[1];
            var operands = new[] { unchecked((uint)op1), unchecked((uint)op2) };  // 16 bits each: the core checks
            Run(() => NativeMethods.iad_bus_iai_command(_handle, slave, 0x45, group, command, operands, NativeText.Size(operands.Length), flags, retries ?? NativeMethods.IntUnknown, out rejection, status, NativeText.Size(status.Length)));
            return (rejection, (int)status[0]);
        }

        /// <summary>(0.4) IAI 0x48: (rejection, the four status words).</summary>
        internal (int Rejection, uint[] Status) IaiCommand2(int slave, int group, int command, uint[] operands, uint flags, int? retries)
        {
            var rejection = 0;
            var status = new uint[4];
            Run(() => NativeMethods.iad_bus_iai_command(_handle, slave, 0x48, group, command, operands.Length > 0 ? operands : null, NativeText.Size(operands.Length), flags, retries ?? NativeMethods.IntUnknown, out rejection, status, NativeText.Size(status.Length)));
            return (rejection, status);
        }

        /// <summary>(0.4) IAI 0x45 to address 0; the bus is held for <paramref name="settleS"/> after the frame. Returns the frame sent.</summary>
        internal byte[] BroadcastIaiCommand(int group, int command, int op1, int op2, uint flags, double settleS)
        {
            var frame = new byte[16];
            var length = UIntPtr.Zero;
            Run(() => NativeMethods.iad_bus_broadcast_iai_command(_handle, group, command, op1, op2, flags, settleS, frame, NativeText.Size(frame.Length), out length));
            return frame.Take(NativeText.ToInt(length)).ToArray();
        }

        /// <summary>(0.4) An FC05 / FC06 / FC10 PDU to address 0. Returns the frame sent.</summary>
        internal byte[] Broadcast(byte[] pdu, double? settleS = null)
        {
            var frame = new byte[pdu.Length + 3];
            var length = UIntPtr.Zero;
            Run(() => NativeMethods.iad_bus_broadcast(_handle, pdu, NativeText.Size(pdu.Length), settleS ?? double.NaN, frame, NativeText.Size(frame.Length), out length));
            return frame.Take(NativeText.ToInt(length)).ToArray();
        }

        /// <summary>(0.4) State the line of this bus (no I/O).</summary>
        internal void SetLine(NativeLineKind line, double? positionUnitMm = null) =>
            NativeErrors.Check(NativeMethods.iad_bus_set_line(_handle, (int)line, positionUnitMm ?? double.NaN));

        /// <summary>(0.4) Work out the line (READ-ONLY probes) and attach it; the kind found.</summary>
        internal NativeLineKind DetectLine(NativeLineKind line = NativeLineKind.Auto, double? timeoutS = null, double? positionUnitMm = null)
        {
            var found = 0;
            Run(() => NativeMethods.iad_bus_detect_line(_handle, (int)line, timeoutS ?? double.NaN, positionUnitMm ?? double.NaN, out found));
            return (NativeLineKind)found;
        }

        /// <summary>(0.4) The line of this bus as JSON (no I/O).</summary>
        internal string LineJson() => NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_bus_line_json(_handle, b, s, out r));

        /// <summary>(0.4) The response delay RTIM of <paramref name="slave"/> (ms) used in its response timeout.</summary>
        internal void SetResponseDelay(int slave, double delayMs) => NativeErrors.Check(NativeMethods.iad_bus_set_response_delay(_handle, slave, delayMs));

        internal double ResponseDelay(int slave)
        {
            NativeErrors.Check(NativeMethods.iad_bus_response_delay(_handle, slave, out var delay));
            return delay;
        }

        /// <summary>(0.4) How long the bus waits for the answer to <paramref name="pdu"/> (s; no I/O).</summary>
        internal double ResponseTimeout(int slave, byte[] pdu, int expectedLength = 0)
        {
            NativeErrors.Check(NativeMethods.iad_bus_response_timeout(_handle, slave, pdu, NativeText.Size(pdu.Length), expectedLength, out var seconds));
            return seconds;
        }

        /// <summary>
        /// (0.4) FC03 with the per-call options (iad_bus_read_registers): <paramref name="maxWaitS"/> caps the wait per
        /// attempt (null: no cap), <paramref name="chunk"/> is the registers per frame (null: 60).
        /// </summary>
        internal ushort[] ReadRegisters(int slave, int address, int count, int? retries, double? maxWaitS, int? chunk = null)
        {
            var values = new ushort[Math.Max(count, 1)];
            Run(() => NativeMethods.iad_bus_read_registers(_handle, slave, address, count, retries ?? NativeMethods.IntUnknown, maxWaitS ?? double.NaN, chunk ?? NativeMethods.IntUnknown, values));
            return values;
        }

        /// <summary>
        /// (0.4) FC11 with the per-call options (iad_bus_report_slave_id): <paramref name="retries"/> null = the bus's;
        /// each attempt waits at least <paramref name="timeoutS"/> for the answer (null: the transport's timeout only).
        /// </summary>
        internal byte[] ReportSlaveId(int slave, int? retries, double? timeoutS)
        {
            var buffer = new byte[256];
            var length = UIntPtr.Zero;
            Run(() => NativeMethods.iad_bus_report_slave_id(_handle, slave, retries ?? NativeMethods.IntUnknown, timeoutS ?? double.NaN, buffer, NativeText.Size(buffer.Length), out length));
            var data = new byte[NativeText.ToInt(length)];
            Array.Copy(buffer, data, data.Length);
            return data;
        }

        /// <summary>
        /// (0.4) The rate one of <paramref name="slaves"/> answers at, each probed at <paramref name="probeAddress"/>
        /// (iad_bus_detect_baud_rate): the present rate first or the candidates in their order, each probe capped by
        /// <paramref name="probeTimeoutS"/> (null: the core's rule, 0.05 s for several slaves, else the transport's timeout).
        /// </summary>
        internal int DetectBaudRate(IEnumerable<int> slaves, IEnumerable<int>? candidates, int probeAddress, bool currentFirst, double? probeTimeoutS)
        {
            var rate = 0;
            var t = Utf8.Encode(Csv(slaves));
            var c = Candidates(candidates);
            Run(() => NativeMethods.iad_bus_detect_baud_rate(_handle, t, c, probeAddress, currentFirst ? 1 : 0, probeTimeoutS ?? double.NaN, out rate));
            return rate;
        }

        /// <summary>"1,2,3" of <paramref name="values"/>.</summary>
        internal static string Csv(IEnumerable<int> values) => string.Join(",", values.Select(v => v.ToString(CultureInfo.InvariantCulture)));

        /// <summary>
        /// The candidates argument of a detection: null = the core's default order; an empty list asks the present rate
        /// only (the core reads "" as the default order, so the present rate is named).
        /// </summary>
        private byte[]? Candidates(IEnumerable<int>? candidates)
        {
            if (candidates is null)
            {
                return null;
            }

            var list = candidates.ToList();
            if (list.Count == 0)
            {
                var present = (MiniJson.TryParse(InfoJson()) as JsonObject)?.Int("baud_rate") ?? 0;
                if (present > 0)
                {
                    list.Add(present);
                }
            }

            return Utf8.Encode(Csv(list));
        }

        // ------------------------------------------------------------------------------------------ 0.5 (py-cell): the port

        /// <summary>
        /// (0.5) Make this bus a port (iad_bus_set_port; no I/O): its name, its line kind (Direct: no gateway probes) and a
        /// rate that is never detected (<paramref name="fixedRate"/>).
        /// </summary>
        internal void SetPort(string name, NativeLineKind line, bool fixedRate, double? positionUnitMm = null) =>
            NativeErrors.Check(NativeMethods.iad_bus_set_port(_handle, Utf8.Encode(name, nameof(name)), (int)line, positionUnitMm ?? double.NaN, fixedRate ? 1 : 0, null, IntPtr.Zero));

        /// <summary>(0.5) The port as JSON {"name", "baud_rate", "baud_detected", "line", "closed"} (no I/O).</summary>
        internal JsonObject PortInfo() =>
            MiniJson.TryParse(NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_bus_port_info_json(_handle, b, s, out r))) as JsonObject ?? new JsonObject();

        /// <summary>(0.5) Detect the port's rate once, in place (Port::ensure_baud_rate); the rate, null without one.</summary>
        internal int? EnsureBaudRate(IEnumerable<int>? slaves)
        {
            var rate = 0;
            var s = slaves is null ? null : Utf8.Encode(Csv(slaves));
            Run(() => NativeMethods.iad_bus_ensure_baud_rate(_handle, s, out rate, out _));
            return rate > 0 ? rate : (int?)null;
        }

        /// <summary>(0.6) The port's own detection with <paramref name="candidates"/> (null: the default order); the rate found.</summary>
        internal int PortDetectBaudRate(IEnumerable<int> slaves, IEnumerable<int>? candidates, double? probeTimeoutS)
        {
            var rate = 0;
            var s = Utf8.Encode(Csv(slaves));
            var c = Candidates(candidates);
            Run(() => NativeMethods.iad_bus_port_detect_baud_rate(_handle, s, c, probeTimeoutS ?? double.NaN, out rate));
            return rate;
        }

        /// <summary>(0.5) The scan with what each address says about itself (Port::scan_units), as the core's JSON array.</summary>
        internal List<object?> ScanUnits(IEnumerable<int> slaves, double? timeoutS, bool identify)
        {
            var s = Utf8.Encode(Csv(slaves));
            return MiniJson.Parse(Text((byte[]? b, UIntPtr n, out UIntPtr r) =>
                NativeMethods.iad_bus_scan_units_json(_handle, s, timeoutS ?? double.NaN, identify ? 1 : 0, 1, b, n, out r))) as List<object?> ?? new List<object?>();
        }

        /// <summary>(0.5) The line-wide baud change (Port::set_baud_rate) of <paramref name="slaves"/>, as the core's JSON.</summary>
        internal JsonObject SetBaudRateAll(int rate, IEnumerable<int> slaves, IadBaudOptions options)
        {
            var s = Utf8.Encode(Csv(slaves));
            return MiniJson.Parse(Text((byte[]? b, UIntPtr n, out UIntPtr r) =>
                NativeMethods.iad_bus_set_baud_rate_all(_handle, rate, s, ref options, b, n, out r))) as JsonObject ?? new JsonObject();
        }

        /// <summary>(0.6) What <paramref name="slave"/> tells about itself (read_device_info on this bus), as the core's JSON.</summary>
        internal JsonObject ReadDeviceInfo(int slave) =>
            MiniJson.Parse(Text((byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_bus_read_device_info_json(_handle, slave, b, n, out r))) as JsonObject ?? new JsonObject();

        /// <summary>
        /// The text output of a call that does I/O and must not run twice (a scan, a baud change): one call with a buffer
        /// far above what these outputs reach. Raises the call's error, or what a bridged transport threw.
        /// </summary>
        internal static string Text(NativeText.Call call)
        {
            TransportBridge.ClearError();
            var buffer = new byte[1024 * 1024];
            var result = call(buffer, NativeText.Size(buffer.Length), out _);
            if (result != 0)
            {
                throw TransportBridge.Raise(result);
            }

            return Utf8.FromBuffer(buffer);
        }

        /// <summary>One library call that may reach a bridged transport: its error, or what the transport threw.</summary>
        internal static void Run(Func<int> call)
        {
            TransportBridge.ClearError();
            var result = call();
            if (result != 0)
            {
                throw TransportBridge.Raise(result);
            }
        }
    }
}
