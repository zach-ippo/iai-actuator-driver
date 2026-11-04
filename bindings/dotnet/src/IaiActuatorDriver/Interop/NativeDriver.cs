using System;
using System.Globalization;
using System.Threading;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver.Native
{
    /// <summary>
    /// The native core under this library as a whole: its versions, the ABI check and the serial port list (public for
    /// diagnostics; the binding itself is the internal interop layer). It is written for ABI
    /// <see cref="AbiMajor"/>.<see cref="AbiMinor"/> of iai_actuator_driver.h; <see cref="EnsureLoaded"/> runs once
    /// before the first handle is opened (every open function calls it) and refuses a library with another ABI.
    /// </summary>
    /// <remarks>
    /// The runtime finds iai_actuator_driver.dll (libiai_actuator_driver.so / .dylib) next to the application or in the
    /// NuGet package's runtimes/&lt;rid&gt;/native/. Evidence: the binding is VERIFIED on the PCON-CB with an RCP6 and on
    /// the EC-GRBP8M gripper (steps in docs/dev/NATIVE_ABI.md), tested on the native simulator besides; everything else
    /// UNVERIFIED.
    /// </remarks>
    public static class NativeDriver
    {
        /// <summary>The ABI major version this binding was written for (IAD_ABI_VERSION_MAJOR).</summary>
        public const int AbiMajor = 1;

        /// <summary>
        /// The ABI minor version this binding was written for: the least minor it accepts. It stays 0 while no public
        /// call needs a later one; the 1.1 imports (NativeMethods.Abi11.cs) are not used by any public call yet.
        /// </summary>
        public const int AbiMinor = 0;

        /// <summary>Base name of the native library.</summary>
        public const string LibraryName = NativeMethods.Library;

        private static int _loaded;

        /// <summary>The loaded library's ABI version (iad_abi_version).</summary>
        public static (int Major, int Minor) AbiVersion
        {
            get
            {
                var v = Call(NativeMethods.iad_abi_version);
                return ((int)(v >> 16), (int)(v & 0xFFFF));
            }
        }

        /// <summary>Version of the native core, e.g. "1.0.0" (iad_library_version; independent of the ABI version).</summary>
        public static string LibraryVersion => Utf8.FromPointer(Call(NativeMethods.iad_library_version));

        /// <summary>
        /// Whether a library with ABI <paramref name="major"/>.<paramref name="minor"/> works with this binding: the same
        /// major and a minor at least <see cref="AbiMinor"/> (a higher minor only adds).
        /// </summary>
        public static bool IsAbiCompatible(int major, int minor) =>
            major == AbiMajor && minor >= AbiMinor;

        /// <summary>
        /// Load the library and check its ABI version (once per process). Throws <see cref="DllNotFoundException"/> when
        /// the library is missing, <see cref="BadImageFormatException"/> when it is built for another CPU and
        /// <see cref="NotSupportedException"/> when its ABI does not match this binding.
        /// </summary>
        public static void EnsureLoaded()
        {
            if (Volatile.Read(ref _loaded) != 0)
            {
                return;
            }

            var (major, minor) = AbiVersion;
            if (!IsAbiCompatible(major, minor))
            {
                throw new NotSupportedException(AbiMismatchMessage(major, minor));
            }

            Volatile.Write(ref _loaded, 1);
        }

        /// <summary>
        /// The environment variable that names the native library file to load instead of the default search
        /// ("IAI_ACTUATOR_DRIVER_NATIVE"): a debug build or a release candidate. Read once, before the first call.
        /// </summary>
        public const string OverrideVariable = NativeLoader.OverrideVariable;

        /// <summary>The file <see cref="OverrideVariable"/> names (full path), or null when it is not set.</summary>
        public static string? OverridePath => NativeLoader.OverridePath;

        /// <summary>The text of the error for a library with another ABI (also names the file when it was given by <see cref="OverrideVariable"/>).</summary>
        internal static string AbiMismatchMessage(int major, int minor) => string.Format(
            CultureInfo.InvariantCulture,
            "{0}{1} has ABI {2}.{3}; this binding was written for ABI {4}.{5} (it needs the same major and a minor at least {5}): use the library and the binding of the same release",
            LibraryName,
            NativeLoader.OverridePath is { } path ? " (" + path + ")" : string.Empty,
            major,
            minor,
            AbiMajor,
            AbiMinor);

        /// <summary>Name of a result code as the library spells it (iad_result_name), e.g. "IAD_ERR_SOFT_LIMIT".</summary>
        public static string ResultName(NativeResult code) => Utf8.FromPointer(Call(() => NativeMethods.iad_result_name((int)code)));

        /// <summary>
        /// The PC's serial ports as JSON, IAI USB adapters first: [{"port", "description", "manufacturer",
        /// "hardware_id", "vid", "pid", "is_iai"}]. Opens nothing.
        /// </summary>
        public static string ListPortsJson()
        {
            EnsureLoaded();
            return NativeText.Read((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_list_ports_json(b, s, out r));
        }

        /// <summary>Message of the last failed call on this thread (iad_last_error_message; "" after a success).</summary>
        public static string LastErrorMessage => Utf8.FromPointer(Call(NativeMethods.iad_last_error_message));

        /// <summary>
        /// (ABI 0.2) Install (<paramref name="handler"/> not null) or remove (null) the process-wide handler of the core's
        /// log lines (iad_set_log_sink; C++ set_log_handler): (level, source such as "iai_actuator_driver.axis",
        /// message). It runs on the thread that logs (a cell worker, a monitor thread): copy what you need, return
        /// quickly and never call the driver from it. An exception it throws is ignored. Replaces an earlier handler.
        /// </summary>
        public static void SetLogHandler(Action<DriverLogLevel, string, string>? handler)
        {
            EnsureLoaded();
            NativeSinks.SetLog(handler);
        }

        /// <summary>Run a library call, turning a missing library into a <see cref="DllNotFoundException"/> that says what to do.</summary>
        private static T Call<T>(Func<T> call)
        {
            NativeLoader.Prepare();  // the IAI_ACTUATOR_DRIVER_NATIVE override, before the first call binds the library
            try
            {
                return call();
            }
            catch (DllNotFoundException e)
            {
                throw new DllNotFoundException(NativeLoader.NotFoundMessage(), e);
            }
            catch (BadImageFormatException e)
            {
                throw new BadImageFormatException(NativeLoader.BadImageMessage(), e);
            }
        }
    }
}
