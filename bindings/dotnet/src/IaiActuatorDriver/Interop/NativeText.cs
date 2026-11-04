using System;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>The (buffer, buffer_size, required) text outputs of the C ABI: JSON and names.</summary>
    internal static class NativeText
    {
        /// <summary>One call of a text output function.</summary>
        internal delegate int Call(byte[]? buffer, UIntPtr bufferSize, out UIntPtr required);

        /// <summary>First buffer size (64 KiB, docs/dev/NATIVE_ABI.md); grown as the library asks.</summary>
        internal const int InitialSize = 64 * 1024;

        /// <summary>Calls of <see cref="Read"/> at most: the first, then one per larger size asked for.</summary>
        internal const int ReadAttempts = 3;

        /// <summary>
        /// Read a text output: a 64 KiB buffer, then again with the size the library reported while it did not fit,
        /// up to <see cref="ReadAttempts"/> calls (a re-measured plan can grow between the size query and the read;
        /// the text functions are read-only, so a repeat is harmless). Throws the call's error otherwise, and
        /// InvalidOperationException when the text kept growing.
        /// </summary>
        internal static string Read(Call call, int initialSize = InitialSize)
        {
            var size = initialSize;
            for (var attempt = 0; attempt < ReadAttempts; attempt++)
            {
                var buffer = new byte[size];
                var result = call(buffer, Size(buffer.Length), out var required);
                if (result == (int)NativeResult.BufferTooSmall && required.ToUInt64() > (ulong)buffer.Length)
                {
                    if (required.ToUInt64() > int.MaxValue)
                    {
                        throw new InvalidOperationException("the library asked for a text buffer over 2 GB");
                    }

                    size = ToInt(required);
                    continue;
                }

                NativeErrors.Check(result);
                return Utf8.FromBuffer(buffer);
            }

            throw new InvalidOperationException("the text output kept growing; try again");
        }

        /// <summary>
        /// Read the text of a call that must not run twice (it writes a file, reads for long or moves): one call with a
        /// large buffer and, when that did not fit, the copy the library kept on the handle (<paramref name="kept"/>, no
        /// I/O).
        /// </summary>
        internal static string ReadOnce(Call call, Func<string> kept, int initialSize = 1024 * 1024)
        {
            var buffer = new byte[initialSize];
            var result = call(buffer, Size(buffer.Length), out _);
            if (result == (int)NativeResult.BufferTooSmall)
            {
                return kept();
            }

            NativeErrors.Check(result);
            return Utf8.FromBuffer(buffer);
        }

        /// <summary>A size_t argument (UIntPtr has no conversion from int on netstandard2.0).</summary>
        internal static UIntPtr Size(int value) => new UIntPtr((uint)Math.Max(value, 0));

        /// <summary>A size_t result as int (clamped; the counts here are small).</summary>
        internal static int ToInt(UIntPtr value) => (int)Math.Min(value.ToUInt64(), int.MaxValue);
    }
}
