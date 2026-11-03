using System.Runtime.InteropServices;
using System.Text;
using IaiActuatorDriver.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The ABI 1.1 imports (NativeMethods.Abi11.cs): the serial framing options struct, its defaults and the refusals
    /// of iad_transport_open_serial_options. Every call names a port that does not exist: no port opened.
    /// </summary>
    public class Abi11BindingTests
    {
        private static readonly byte[] NoSuchPort = Encoding.UTF8.GetBytes("IAD-NO-SUCH-PORT\0");

        [Fact]
        public void The_11_struct_has_the_size_the_core_freezes()
        {
            Assert.Equal(64, Marshal.SizeOf<IadSerialOptions>());
            Assert.Equal(64u, SizedStruct.Fresh<IadSerialOptions>().StructSize);
            Assert.Equal(16, (int)Marshal.OffsetOf<IadSerialOptions>(nameof(IadSerialOptions.BaudRate)));
            Assert.Equal(24, (int)Marshal.OffsetOf<IadSerialOptions>(nameof(IadSerialOptions.Parity)));
            Assert.Equal(32, (int)Marshal.OffsetOf<IadSerialOptions>(nameof(IadSerialOptions.Reserved)));
            Assert.Equal((0, 1, 2), (NativeMethods.ParityNone, NativeMethods.ParityEven, NativeMethods.ParityOdd));
        }

        [NativeFact]
        public void The_defaults_are_115200_baud_and_8n1()
        {
            var options = SizedStruct.Fresh<IadSerialOptions>();
            Assert.Equal(0, NativeMethods.iad_serial_options_init(ref options));
            Assert.Equal(64u, options.StructSize);
            Assert.Equal(0.2, options.ReadTimeoutS);
            Assert.Equal(115200, options.BaudRate);
            Assert.Equal(8, options.DataBits);
            Assert.Equal(NativeMethods.ParityNone, options.Parity);
            Assert.Equal(1, options.StopBits);
        }

        [NativeFact]
        public void An_unsupported_framing_is_refused_before_the_port_is_opened()
        {
            const int InvalidArgument = -10;
            const int TransportError = -7;
            var options = SizedStruct.Fresh<IadSerialOptions>();
            NativeMethods.iad_serial_options_init(ref options);
            options.DataBits = 6;
            Assert.Equal(InvalidArgument, NativeMethods.iad_transport_open_serial_options(NoSuchPort, ref options, out var refused));
            Assert.True(refused.IsInvalid);
            options.DataBits = 7;
            options.Parity = NativeMethods.ParityEven;
            options.StopBits = 2;
            Assert.Equal(TransportError, NativeMethods.iad_transport_open_serial_options(NoSuchPort, ref options, out var absent));
            Assert.True(absent.IsInvalid); // a valid framing reaches the open, which finds no port
        }
    }
}
