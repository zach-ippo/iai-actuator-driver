using System;
using System.Text;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>UTF-8 text across the C ABI: NUL-terminated byte arrays in, fixed arrays / buffers / pointers out.</summary>
    internal static class Utf8
    {
        private static readonly UTF8Encoding Utf8NoBom = new UTF8Encoding(encoderShouldEmitUTF8Identifier: false, throwOnInvalidBytes: false);

        /// <summary>NUL-terminated UTF-8 of <paramref name="text"/>; <c>null</c> stays <c>null</c> (the C NULL).</summary>
        internal static byte[]? Encode(string? text, string paramName = "text")
        {
            if (text is null)
            {
                return null;
            }

            if (text.IndexOf('\0') >= 0)
            {
                throw new ArgumentException("text must not contain a NUL character", paramName);
            }

            var bytes = new byte[Utf8NoBom.GetByteCount(text) + 1];
            Utf8NoBom.GetBytes(text, 0, text.Length, bytes, 0);
            return bytes;
        }

        /// <summary>As <see cref="Encode"/>, for a parameter that must not be <c>null</c>.</summary>
        internal static byte[] EncodeRequired(string text, string paramName) =>
            Encode(text ?? throw new ArgumentNullException(paramName), paramName)!;

        /// <summary>The text a <c>const char*</c> result points to ("" for NULL).</summary>
        internal static unsafe string FromPointer(IntPtr pointer)
        {
            if (pointer == IntPtr.Zero)
            {
                return string.Empty;
            }

            var p = (byte*)pointer;
            var length = 0;
            while (p[length] != 0)
            {
                length++;
            }

            return length == 0 ? string.Empty : Utf8NoBom.GetString(p, length);
        }

        /// <summary>The text of a fixed <c>char[size]</c> field, up to its NUL (or the whole field without one).</summary>
        internal static unsafe string FromFixed(byte* p, int size)
        {
            var length = 0;
            while (length < size && p[length] != 0)
            {
                length++;
            }

            return length == 0 ? string.Empty : Utf8NoBom.GetString(p, length);
        }

        /// <summary>The text of a caller buffer the library filled, up to its NUL.</summary>
        internal static string FromBuffer(byte[] buffer)
        {
            var length = Array.IndexOf(buffer, (byte)0);
            if (length < 0)
            {
                length = buffer.Length;
            }

            return Utf8NoBom.GetString(buffer, 0, length);
        }
    }
}
