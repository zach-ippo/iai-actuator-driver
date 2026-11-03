using System;
using System.Collections.Generic;
using System.Text;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>NativeText.Read (the text outputs' buffers) against a scripted call. No native library.</summary>
    public class NativeTextTests
    {
        /// <summary>
        /// A call that reports BufferTooSmall with each of <paramref name="asks"/> in turn, then writes
        /// <paramref name="text"/>; <paramref name="sizes"/> sees each buffer size.
        /// </summary>
        private static NativeText.Call Growing(string text, int[] asks, Action<int> sizes)
        {
            var calls = 0;
            return (byte[]? buffer, UIntPtr bufferSize, out UIntPtr required) =>
            {
                sizes((int)bufferSize.ToUInt64());
                var bytes = Encoding.UTF8.GetBytes(text);
                if (calls < asks.Length)
                {
                    required = new UIntPtr((uint)asks[calls++]);
                    return (int)NativeResult.BufferTooSmall;
                }

                calls++;
                required = new UIntPtr((uint)bytes.Length + 1);
                Array.Copy(bytes, buffer!, bytes.Length);
                buffer![bytes.Length] = 0;
                return 0;
            };
        }

        [Fact]
        public void A_text_that_grows_between_the_size_query_and_the_read_is_read_again()
        {
            // the first answer asks for 40 bytes; a re-measured plan has grown to 64 by the second call
            var seen = new List<int>();
            var text = new string('x', 63);
            Assert.Equal(text, NativeText.Read(Growing(text, new[] { 40, 64 }, seen.Add), initialSize: 16));
            Assert.Equal(new[] { 16, 40, 64 }, seen);
        }

        [Fact]
        public void A_text_that_keeps_growing_is_not_read_forever()
        {
            var seen = new List<int>();
            var call = Growing("never", new[] { 40, 64, 128, 256 }, seen.Add);
            Assert.Throws<InvalidOperationException>(() => NativeText.Read(call, initialSize: 16));
            Assert.Equal(new[] { 16, 40, 64 }, seen);
        }
    }
}
