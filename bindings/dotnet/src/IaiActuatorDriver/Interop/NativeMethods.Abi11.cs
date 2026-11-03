using System.Runtime.InteropServices;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The ABI 1.1 additions of iai_actuator_driver.h (conventions: NativeMethods.cs): a serial port opened with
    /// another character framing (data bits, parity, stop bits). Imports only: no public call uses them yet, so the
    /// binding still accepts a 1.0 library (<see cref="NativeDriver.AbiMinor"/>), which lacks these entry points.
    /// </summary>
    internal static partial class NativeMethods
    {
        // ------------------------------------------------------------------------------------------ 1.1: constants

        /// <summary>IAD_PARITY_NONE: no parity bit (the default).</summary>
        internal const int ParityNone = 0;

        /// <summary>IAD_PARITY_EVEN.</summary>
        internal const int ParityEven = 1;

        /// <summary>IAD_PARITY_ODD.</summary>
        internal const int ParityOdd = 2;

        // ------------------------------------------------------------------------------------------ 1.1: library

        /// <summary>The defaults of iad_transport_open_serial_options: 115200 baud, read timeout 0.2 s, 8N1.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_serial_options_init(ref IadSerialOptions options);

        // ------------------------------------------------------------------------------------------ 1.1: transports

        /// <summary>iad_transport_open_serial with its settings in a sized struct, the framing among them.</summary>
        [DllImport(Library, CallingConvention = Winapi, ExactSpelling = true)]
        internal static extern int iad_transport_open_serial_options(byte[] port, ref IadSerialOptions options, out TransportHandle transport);
    }

    /// <summary>iad_serial_options (1.1, 64 bytes, a sized struct).</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal unsafe struct IadSerialOptions
    {
        public uint StructSize;
        public uint ReservedHeader;
        public double ReadTimeoutS;
        public int BaudRate;
        public int DataBits;
        public int Parity;
        public int StopBits;
        public fixed ulong Reserved[4];
    }
}
