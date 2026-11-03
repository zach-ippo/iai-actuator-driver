using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>(ABI 0.3) Axes on buses and transports, the bus of an axis, and the cancel latch.</summary>
    internal sealed partial class NativeAxis
    {
        /// <summary>
        /// One axis on <paramref name="bus"/> at <paramref name="slave"/> (iad_axis_open_bus): no I/O, call
        /// <see cref="Connect"/> next. The axis does not own the bus: closing it releases its STOP / pause and leaves the
        /// bus open.
        /// </summary>
        public static NativeAxis OpenOnBus(NativeBus bus, int slave, NativeAxisOptions? options = null, string? name = null)
        {
            if (bus is null)
            {
                throw new ArgumentNullException(nameof(bus));
            }

            var axisOptions = (options ?? new NativeAxisOptions()).ToNative();
            TransportBridge.ClearError();
            var result = NativeMethods.iad_axis_open_bus(bus.Handle, slave, ref axisOptions, out var handle);
            return Adopt(result, handle, name ?? "BUS:" + slave.ToString(CultureInfo.InvariantCulture));
        }

        /// <summary>
        /// iad_axis_open on any transport (iad_axis_open_transport: baud detection when the transport has a rate, the line
        /// probe). The axis owns its own bus on the transport: closing the axis closes the transport. The caller still
        /// releases its <paramref name="transport"/> handle.
        /// </summary>
        public static NativeAxis OpenOnTransport(
            TransportHandle transport,
            int slave = 1,
            int? baudRate = null,
            NativeAxisOptions? options = null,
            int? retries = null,
            TimeSpan? minGap = null,
            string? name = null)
        {
            if (transport is null)
            {
                throw new ArgumentNullException(nameof(transport));
            }

            var open = OpenOptions(slave, baudRate, null, retries, minGap, NativeLineKind.Auto, null, false);
            var axisOptions = (options ?? new NativeAxisOptions()).ToNative();
            TransportBridge.ClearError();
            var result = NativeMethods.iad_axis_open_transport(transport, ref open, ref axisOptions, out var handle);
            if (result != 0)
            {
                handle.Dispose();
                throw TransportBridge.Raise(result);
            }

            return new NativeAxis(handle, name ?? "TRANSPORT:" + slave.ToString(CultureInfo.InvariantCulture));
        }

        /// <summary>A handle on the bus of this axis (iad_axis_bus; disposing it releases the handle only).</summary>
        public NativeBus Bus()
        {
            var result = NativeMethods.iad_axis_bus(_handle, out var bus);
            return NativeBus.Adopt(result, bus);
        }

        /// <summary>
        /// Set the cancel latch (iad_axis_cancel_latch): <see cref="Cancel"/> now, and every blocking call of this axis
        /// that starts while the latch is set is refused with <see cref="OperationCanceledException"/> before anything is
        /// sent. Stop, pause and resume are never refused. Safe from any thread.
        /// </summary>
        public void CancelLatch() => NativeErrors.Check(NativeMethods.iad_axis_cancel_latch(_handle));

        /// <summary>Clear the cancel latch (calls work again).</summary>
        public void CancelReset() => NativeErrors.Check(NativeMethods.iad_axis_cancel_reset(_handle));

        /// <summary>Is the cancel latch set?</summary>
        public bool CancelLatched
        {
            get
            {
                NativeErrors.Check(NativeMethods.iad_axis_cancel_latched(_handle, out var latched));
                return latched != 0;
            }
        }
    }

    /// <summary>(ABI 0.3) Cells on transports, the bus of a port, and the cancel latch.</summary>
    internal sealed partial class NativeCell
    {
        /// <summary>
        /// A cell on <paramref name="transports"/> (iad_cell_open_transports; Cell::open on named transports): each named by
        /// <paramref name="names"/> in order (null = each transport's own name), <paramref name="slaves"/> the addresses
        /// to use (null = scan each). Closing the cell closes the transports; the caller still releases its handles.
        /// </summary>
        public static NativeCell OpenOnTransports(
            IReadOnlyList<TransportHandle> transports,
            IEnumerable<string>? names = null,
            IEnumerable<int>? slaves = null,
            NativeCellOptions? options = null)
        {
            if (transports is null || transports.Count == 0)
            {
                throw new ArgumentException("at least one transport is needed", nameof(transports));
            }

            NativeDriver.EnsureLoaded();
            var n = names is null ? null : Utf8.Encode(JoinNames(names), nameof(names));
            var s = slaves is null ? null : Utf8.Encode(string.Join(",", slaves.Select(v => v.ToString(CultureInfo.InvariantCulture))), nameof(slaves));
            var cellOptions = NativeCellOptions.ToNative(options);
            var axisOptions = (options?.AxisOptions ?? new NativeAxisOptions()).ToNative();
            var tokens = new IntPtr[transports.Count];
            var added = new bool[transports.Count];
            try
            {
                for (var i = 0; i < transports.Count; i++)
                {
                    transports[i].DangerousAddRef(ref added[i]);
                    tokens[i] = transports[i].DangerousGetHandle();
                }

                TransportBridge.ClearError();
                var result = NativeMethods.iad_cell_open_transports(tokens, NativeText.Size(tokens.Length), n, s, ref cellOptions, ref axisOptions, out var handle);
                return Adopt(result, handle);
            }
            finally
            {
                for (var i = 0; i < transports.Count; i++)
                {
                    if (added[i])
                    {
                        transports[i].DangerousRelease();
                    }
                }
            }
        }

        /// <summary>A handle on the bus of the cell's port <paramref name="port"/> (disposing it releases the handle only).</summary>
        public NativeBus Bus(string port)
        {
            var name = Utf8.EncodeRequired(port, nameof(port));
            var result = NativeMethods.iad_cell_bus(_handle, name, out var bus);
            return NativeBus.Adopt(result, bus);
        }

        /// <summary>
        /// Set the cancel latch on the listed axes (null = all; iad_cell_cancel_latch). A listed name the cell does not have
        /// never holds the call back: the axes it has are latched, then a <see cref="CellException"/> names each unknown
        /// name (an <see cref="ArgumentOutOfRangeException"/> of its own), as for <see cref="Cancel"/>.
        /// </summary>
        public void CancelLatch(IEnumerable<string>? axes = null) =>
            NativeErrors.Check(NativeMethods.iad_cell_cancel_latch(_handle, axes is null ? null : Utf8.Encode(JoinNames(axes), nameof(axes))));

        /// <summary>Clear the cancel latch on the listed axes (null = all); unknown names as for <see cref="CancelLatch"/>.</summary>
        public void CancelReset(IEnumerable<string>? axes = null) =>
            NativeErrors.Check(NativeMethods.iad_cell_cancel_reset(_handle, axes is null ? null : Utf8.Encode(JoinNames(axes), nameof(axes))));
    }
}
