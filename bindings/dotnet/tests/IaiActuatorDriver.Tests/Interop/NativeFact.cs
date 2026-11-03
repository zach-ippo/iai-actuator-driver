using System;
using System.IO;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// A test that needs the native library (copied from core/build/ at build time). Skipped, with the reason, when the
    /// library is not next to the tests or cannot be loaded; a function missing from a library that loads is a failure.
    /// </summary>
    public sealed class NativeFactAttribute : FactAttribute
    {
        public NativeFactAttribute()
        {
            if (NativeLib.SkipReason is { } reason)
            {
                Skip = reason;
            }
        }
    }

    /// <summary>A theory that needs the native library (skipped as <see cref="NativeFactAttribute"/>).</summary>
    public sealed class NativeTheoryAttribute : TheoryAttribute
    {
        public NativeTheoryAttribute()
        {
            if (NativeLib.SkipReason is { } reason)
            {
                Skip = reason;
            }
        }
    }

    /// <summary>Where the tests look for the native library, and why they skip.</summary>
    internal static class NativeLib
    {
        private static readonly Lazy<string?> Reason = new Lazy<string?>(Probe);

        /// <summary>The library file next to the test assembly.</summary>
        internal static string Path =>
            System.IO.Path.Combine(
                AppContext.BaseDirectory,
                OperatingSystem.IsWindows() ? "iai_actuator_driver.dll"
                : OperatingSystem.IsMacOS() ? "libiai_actuator_driver.dylib"
                : "libiai_actuator_driver.so");

        /// <summary>Null when the library loads, else why the native tests are skipped.</summary>
        internal static string? SkipReason => Reason.Value;

        private static string? Probe()
        {
            if (!File.Exists(Path))
            {
                return $"native library not found at {Path}: build it with 'python build.py shared' in core/, then rebuild "
                    + "the tests (the library project copies core/build/iai_actuator_driver.dll next to them)";
            }

            if (!NativeLibrary.TryLoad(Path, out var handle))
            {
                return $"native library {Path} cannot be loaded (built for another CPU or OS?)";
            }

            NativeLibrary.Free(handle);
            return null;
        }
    }
}
