using System;
using System.IO;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    /// <summary>
    /// Byte pipe used by <see cref="ModbusRtu"/>: <see cref="SerialTransport"/> for a real port, <see cref="FakeTransport"/>
    /// in memory. Implementations need not be thread-safe: <see cref="ModbusRtu"/> serialises every call under its lock.
    /// </summary>
    public interface ITransport : IDisposable
    {
        /// <summary>Send all of <paramref name="count"/> bytes of <paramref name="buffer"/> from <paramref name="offset"/>.</summary>
        void Write(byte[] buffer, int offset, int count);

        /// <summary>
        /// Read up to <paramref name="count"/> bytes, blocking at most <see cref="ReadTimeout"/>; returns the number read
        /// (fewer than asked, or 0, means timeout). Never throws for a timeout.
        /// </summary>
        int Read(byte[] buffer, int offset, int count);

        /// <summary>Discard any received bytes not yet read.</summary>
        void ResetInputBuffer();

        /// <summary>
        /// Current baud rate, or 0 when the transport has none (an in-memory fake). Setting it changes the rate in place:
        /// the port stays open, so no line break reaches the unit. Throws <see cref="NotSupportedException"/> on a
        /// transport without a baud rate.
        /// </summary>
        int BaudRate { get; set; }

        /// <summary>Per-read timeout (<see cref="ModbusRtu.Probe"/> shortens it for one request).</summary>
        TimeSpan ReadTimeout { get; set; }

        /// <summary>
        /// Hold the line in BREAK for <paramref name="duration"/>. A break of 150 ms or more drops a unit to 9600 baud
        /// (DOC, ME0162; UNVERIFIED on an EC): never used by the driver's normal paths.
        /// </summary>
        void SendBreak(TimeSpan duration);

        /// <summary>Release the underlying resource. Safe to call twice.</summary>
        void Close();
    }


    /// <summary>
    /// <see cref="ITransport"/> on a serial port, opened by the native core (iad_transport_open_serial: Win32 / POSIX):
    /// 8 data bits, no parity, 1 stop bit, DTR and RTS asserted, no handshake. The port is opened once in the constructor
    /// and kept open: opening / closing in a loop, or a long line break, drops an EC back to its power-up baud rate, so the
    /// baud rate is changed in place and no break is ever sent implicitly. A <see cref="ModbusRtu"/> on it uses the native
    /// port directly (no callback per byte); the members below call the same port for direct use.
    /// </summary>
    /// <remarks>
    /// Evidence: the core's serial port is VERIFIED on Windows (the core's CLI and the C ABI binding, on the EC-GRBP8M
    /// gripper and the PCON-CB with an RCP6); on Linux from 1.1.0 (the PCON-CB through the IAI USB adapter in WSL 2,
    /// from the core's CLI, Python and C), not from .NET.
    /// </remarks>
    public sealed class SerialTransport : ITransport
    {
        private readonly TransportHandle _handle;
        private readonly object _sync = new object();
        private TimeSpan _readTimeout;
        private int _baudRate;
        private bool _closed;

        /// <summary>Open <paramref name="portName"/> (e.g. "COM3").</summary>
        /// <param name="portName">Serial port name.</param>
        /// <param name="baudRate">Starting rate: 115200 for a unit that IA-OS switched up; ECs boot at 38400 (VERIFIED).
        /// <see cref="ModbusRtu.DetectBaudRate(int, System.Collections.Generic.IEnumerable{int}?, int, bool, TimeSpan?)"/>
        /// finds the rate in place.</param>
        /// <param name="readTimeout">Per-read timeout (default 0.2 s).</param>
        /// <exception cref="UnauthorizedAccessException">The port is in use by another program.</exception>
        /// <exception cref="IOException">The port does not exist (unplugged?) or cannot be opened.</exception>
        public SerialTransport(string portName, int baudRate = ModbusRtu.DefaultBaudRate, TimeSpan? readTimeout = null)
        {
            if (string.IsNullOrWhiteSpace(portName))
            {
                throw new ArgumentException("port name required", nameof(portName));
            }

            if (baudRate <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(baudRate), "baud rate must be > 0");
            }

            _readTimeout = readTimeout ?? TimeSpan.FromSeconds(0.2);
            if (_readTimeout <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(readTimeout), "read timeout must be > 0");
            }

            PortName = portName;
            NativeDriver.EnsureLoaded();
            var result = NativeMethods.iad_transport_open_serial(Utf8.EncodeRequired(portName, nameof(portName)), baudRate, _readTimeout.TotalSeconds, out var handle);
            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                handle.Dispose();
                throw OpenError(portName, error);
            }

            _handle = handle;
            _baudRate = baudRate;
        }

        /// <summary>The port name, e.g. "COM3".</summary>
        public string PortName { get; }

        /// <summary>The core's transport (for a <see cref="ModbusRtu"/> on this port).</summary>
        internal TransportHandle Handle => _handle;

        /// <inheritdoc />
        /// <remarks>After <see cref="Close"/>: the last rate the port had.</remarks>
        public int BaudRate
        {
            get
            {
                lock (_sync)
                {
                    if (_closed || _handle.IsClosed)
                    {
                        return _baudRate;
                    }

                    var info = MiniJson.TryParse(NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_transport_info_json(_handle, b, s, out r), 1024)) as JsonObject;
                    var rate = info?.Int("baud_rate") ?? 0;
                    if (rate > 0)
                    {
                        _baudRate = rate;  // the core changes it in place (detection, a baud change)
                    }

                    return rate;
                }
            }

            set
            {
                Io("set the baud rate of", () => NativeMethods.iad_transport_set_baud_rate(_handle, value));
                lock (_sync)
                {
                    _baudRate = value;
                }

                Io("clear", () => NativeMethods.iad_transport_reset_input(_handle));
            }
        }

        /// <inheritdoc />
        public TimeSpan ReadTimeout
        {
            get => _readTimeout;
            set
            {
                if (value <= TimeSpan.Zero)
                {
                    throw new ArgumentOutOfRangeException(nameof(value), "read timeout must be > 0");
                }

                Io("set the read timeout of", () => NativeMethods.iad_transport_set_read_timeout(_handle, value.TotalSeconds));
                _readTimeout = value;
            }
        }

        /// <inheritdoc />
        public void Write(byte[] buffer, int offset, int count)
        {
            CheckRange(buffer, offset, count);
            var data = new byte[count];
            Array.Copy(buffer, offset, data, 0, count);
            Io("write to", () => NativeMethods.iad_transport_write(_handle, data, NativeText.Size(count)));
        }

        /// <inheritdoc />
        public int Read(byte[] buffer, int offset, int count)
        {
            CheckRange(buffer, offset, count);
            if (count == 0)
            {
                return 0;
            }

            var data = new byte[count];
            var got = UIntPtr.Zero;
            Io("read from", () => NativeMethods.iad_transport_read(_handle, data, NativeText.Size(count), out got));
            var n = Math.Min(NativeText.ToInt(got), count);
            Array.Copy(data, 0, buffer, offset, n);
            return n;
        }

        /// <inheritdoc />
        public void ResetInputBuffer() => Io("clear", () => NativeMethods.iad_transport_reset_input(_handle));

        /// <inheritdoc />
        public void SendBreak(TimeSpan duration)
        {
            if (duration <= TimeSpan.Zero || duration > TimeSpan.FromSeconds(2))
            {
                throw new ArgumentOutOfRangeException(nameof(duration), "break duration must be in (0, 2] s");
            }

            Io("send a break on", () => NativeMethods.iad_transport_send_break(_handle, duration.TotalSeconds));
        }

        /// <inheritdoc />
        public void Close()
        {
            lock (_sync)
            {
                if (_closed)
                {
                    return;
                }

                _closed = true;
                if (!_handle.IsClosed)
                {
                    NativeMethods.iad_transport_close(_handle);  // closes the port and releases the handle
                    _handle.SetHandleAsInvalid();
                }
            }
        }

        /// <inheritdoc />
        public void Dispose() => Close();

        /// <inheritdoc />
        public override string ToString() => $"SerialTransport({PortName}, {(_closed ? "closed" : BaudRate.ToString(System.Globalization.CultureInfo.InvariantCulture))})";

        /// <summary>The managed edition's exception for a port that cannot be opened.</summary>
        private static Exception OpenError(string portName, Exception error)
        {
            if (error is ModbusException)
            {
                var text = error.Message;
                if (text.IndexOf("in use", StringComparison.OrdinalIgnoreCase) >= 0 || text.IndexOf("access", StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return new UnauthorizedAccessException(
                        $"{portName} is in use by another program (IA-OS, another iai-actuator-driver / iai-gui, a terminal). Close it and try again - only one program can own a serial port",
                        error);
                }

                return new IOException(text, error);
            }

            return error;
        }

        private void Io(string what, Func<int> call)
        {
            if (_closed)
            {
                throw new IOException($"{PortName} is closed");
            }

            int result;
            try
            {
                result = call();
            }
            catch (ObjectDisposedException exc)
            {
                throw new IOException($"{PortName} is closed", exc);
            }

            if (result != 0)
            {
                var error = NativeErrors.Capture(result);
                if (error is ModbusException)
                {
                    throw new IOException($"cannot {what} {PortName}: {error.Message}", error);
                }

                throw error;
            }
        }

        private static void CheckRange(byte[] buffer, int offset, int count)
        {
            if (buffer is null)
            {
                throw new ArgumentNullException(nameof(buffer));
            }

            if (offset < 0 || count < 0 || offset + count > buffer.Length)
            {
                throw new ArgumentOutOfRangeException(nameof(count), "offset and count must lie within the buffer");
            }
        }
    }
}
