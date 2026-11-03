using System;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// An iad_axis* of the native library. The value is a token the library looks up in its registry, never an address,
    /// and is never reused, so a stale handle is detected by the library instead of touching freed memory. The runtime
    /// keeps the handle alive across every call that uses it (a Dispose on another thread waits for the call to return)
    /// and closes it once, also when Dispose is forgotten. <see cref="ReleaseHandle"/> calls iad_axis_close: it releases
    /// a STOP / pause the axis holds and closes the port when the axis owns it; on a handle from iad_cell_open_axis it releases
    /// only the handle.
    /// </summary>
    internal sealed class AxisHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_axis**</c> output.</summary>
        public AxisHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_axis_close(handle) == 0;
    }

    /// <summary>
    /// An iad_cell* of the native library (a token, as <see cref="AxisHandle"/>). <see cref="ReleaseHandle"/> calls
    /// iad_cell_close: it releases every STOP / pause the axes hold and closes every port; the cell's axis handles then
    /// fail with <see cref="ArgumentException"/> (closing them stays safe). (0.5) With <see cref="ReleaseOnly"/> it calls
    /// iad_cell_release instead: the axes and ports stay open for the binding's own objects, which close them.
    /// </summary>
    internal sealed class CellHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_cell**</c> output.</summary>
        public CellHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <summary>Let go of the handle only (iad_cell_release), never closing the axes and ports.</summary>
        internal volatile bool ReleaseOnly;

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() =>
            (ReleaseOnly ? NativeMethods.iad_cell_release(handle) : NativeMethods.iad_cell_close(handle)) == 0;
    }

    /// <summary>
    /// (ABI 0.2) An iad_move_handle* of the native library: a move started without waiting (a token, as
    /// <see cref="AxisHandle"/>). <see cref="ReleaseHandle"/> calls iad_move_handle_close: it releases the handle only, the move
    /// itself is not affected. The handle keeps its axis object alive in the library (not its port).
    /// </summary>
    internal sealed class MoveHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_move_handle**</c> output.</summary>
        public MoveHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_move_handle_close(handle) == 0;
    }

    /// <summary>
    /// (ABI 0.3) An iad_transport* (a token, as <see cref="AxisHandle"/>). <see cref="ReleaseHandle"/> calls
    /// iad_transport_release: the handle only; the buses, axes and cells using the transport keep it. Closing the port is
    /// iad_transport_close (or closing the bus that owns it).
    /// </summary>
    internal sealed class TransportHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_transport**</c> output.</summary>
        public TransportHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_transport_release(handle) == 0;
    }

    /// <summary>
    /// (ABI 0.3) An iad_bus* (a token). <see cref="ReleaseHandle"/> calls iad_bus_release (0.5): the handle only, never
    /// closing the bus, so a bus object the garbage collector finalises never closes a port the program still uses. The
    /// bus and its transport go with the last handle that uses them; an explicit close is iad_bus_shutdown
    /// (<see cref="NativeBus.Dispose"/> of a bus it opened).
    /// </summary>
    internal sealed class BusHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_bus**</c> output.</summary>
        public BusHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_bus_release(handle) == 0;
    }

    /// <summary>
    /// (ABI 0.5) An iad_line* (a token). <see cref="ReleaseHandle"/> calls iad_line_close: the handle only; the line lives
    /// on while a bus carries it.
    /// </summary>
    internal sealed class LineHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_line**</c> output.</summary>
        public LineHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_line_close(handle) == 0;
    }

    /// <summary>
    /// (ABI 0.5) An iad_gateway* (a token): an RCON or ROBONET gateway of a line, read-only. <see cref="ReleaseHandle"/>
    /// calls iad_gateway_close (the handle only).
    /// </summary>
    internal sealed class GatewayHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_gateway**</c> output.</summary>
        public GatewayHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_gateway_close(handle) == 0;
    }

    /// <summary>
    /// (ABI 0.3) An iad_sim* (a token). <see cref="ReleaseHandle"/> calls iad_sim_close: the handle only; the simulated
    /// unit lives on while a transport, bus, axis or line uses it.
    /// </summary>
    internal sealed class SimHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_sim**</c> output.</summary>
        public SimHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_sim_close(handle) == 0;
    }

    /// <summary>(ABI 0.3) An iad_sim_line* (a token). <see cref="ReleaseHandle"/> calls iad_sim_line_close (the handle only).</summary>
    internal sealed class SimLineHandle : SafeHandle
    {
        /// <summary>Created by the P/Invoke marshaller for an <c>iad_sim_line**</c> output.</summary>
        public SimLineHandle()
            : base(IntPtr.Zero, ownsHandle: true)
        {
        }

        /// <inheritdoc />
        public override bool IsInvalid => handle == IntPtr.Zero;

        /// <inheritdoc />
        protected override bool ReleaseHandle() => NativeMethods.iad_sim_line_close(handle) == 0;
    }
}
