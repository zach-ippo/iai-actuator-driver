using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// Where the native core comes from. By default the runtime's own search: next to the application, then the NuGet
    /// package's runtimes/&lt;rid&gt;/native/ (from deps.json). <see cref="OverrideVariable"/> names one file instead
    /// (a debug build, a release candidate): on .NET 5+ through <c>NativeLibrary.SetDllImportResolver</c>, on .NET
    /// Framework / netstandard2.0 by loading that file by full path before the first call (LoadLibrary / dlopen; the
    /// runtime then binds to the module already loaded; UNVERIFIED on Linux / Mono). A missing or unloadable library is a
    /// <see cref="DllNotFoundException"/> / <see cref="BadImageFormatException"/> naming the runtime identifier, the
    /// places searched and the ABI this binding needs (<see cref="NotFoundMessage"/>); there is no managed fallback.
    /// </summary>
    internal static class NativeLoader
    {
        /// <summary>The environment variable naming the native library file to use instead of the default search.</summary>
        internal const string OverrideVariable = "IAI_ACTUATOR_DRIVER_NATIVE";

        private static readonly object Gate = new object();
        private static bool _prepared;
        private static string? _overridePath;
        private static IntPtr _overrideHandle;

        /// <summary>The file named by <see cref="OverrideVariable"/> when set (null otherwise), read once per process.</summary>
        internal static string? OverridePath
        {
            get
            {
                Prepare();
                return _overridePath;
            }
        }

        /// <summary>
        /// Set up the override once per process (before the first library call). Throws <see cref="DllNotFoundException"/>
        /// when the override names a file that cannot be loaded, and <see cref="PlatformNotSupportedException"/> in a 32-bit
        /// process without an override (no 32-bit build is shipped).
        /// </summary>
        internal static void Prepare()
        {
            if (_prepared)
            {
                return;
            }

            lock (Gate)
            {
                if (_prepared)
                {
                    return;
                }

                var path = Environment.GetEnvironmentVariable(OverrideVariable);
                _overridePath = string.IsNullOrWhiteSpace(path) ? null : Path.GetFullPath(path!.Trim().Trim('"'));
                if (_overridePath is null && !Environment.Is64BitProcess)
                {
                    throw new PlatformNotSupportedException(
                        "the native library " + NativeMethods.Library + " is built for 64-bit processes only (win-x64, linux-x64): "
                        + "run the application as a 64-bit process (Platform target x64 or Any CPU without 'Prefer 32-bit'), or set "
                        + OverrideVariable + " to a 32-bit build of the library");
                }

                if (_overridePath != null)
                {
#if NET5_0_OR_GREATER
                    try
                    {
                        NativeLibrary.SetDllImportResolver(typeof(NativeLoader).Assembly, Resolve);
                    }
                    catch (InvalidOperationException)
                    {
                        // a resolver is set already (a host did it): the file is still loaded below, so the runtime binds to it
                    }
#endif
                    _overrideHandle = LoadOverride(_overridePath);
                }

                _prepared = true;
            }
        }

        /// <summary>Load <paramref name="path"/> by full path; <see cref="DllNotFoundException"/> naming it when that fails.</summary>
        internal static IntPtr LoadOverride(string path)
        {
            if (!File.Exists(path))
            {
                throw new DllNotFoundException(string.Format(
                    CultureInfo.InvariantCulture,
                    "{0} names {1}, which does not exist: point it at iai_actuator_driver.dll / libiai_actuator_driver.so (built with 'python build.py shared' in core/) or unset it",
                    OverrideVariable,
                    path));
            }

            IntPtr handle;
            string? why = null;
#if NET5_0_OR_GREATER
            try
            {
                handle = NativeLibrary.Load(path);
            }
            catch (Exception e) when (e is DllNotFoundException || e is BadImageFormatException)
            {
                handle = IntPtr.Zero;
                why = e.Message;
            }
#else
            handle = PlatformLoad(path, out why);
#endif
            if (handle == IntPtr.Zero)
            {
                throw new DllNotFoundException(string.Format(
                    CultureInfo.InvariantCulture,
                    "{0} names {1}, which cannot be loaded ({2}): it must be the {3} build of the library{4}",
                    OverrideVariable,
                    path,
                    why ?? "unknown reason",
                    Rid,
                    Environment.Is64BitProcess ? " (a 64-bit process needs the x64 build)" : " (a 32-bit process needs a 32-bit build)"));
            }

            return handle;
        }

#if NET5_0_OR_GREATER
        private static IntPtr Resolve(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
        {
            if (!string.Equals(libraryName, NativeMethods.Library, StringComparison.Ordinal) || _overridePath is null)
            {
                return IntPtr.Zero;  // the default search
            }

            return _overrideHandle != IntPtr.Zero ? _overrideHandle : LoadOverride(_overridePath);
        }
#else
        private static IntPtr PlatformLoad(string path, out string? why)
        {
            why = null;
            try
            {
                if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                {
                    var h = LoadLibraryW(path);
                    if (h == IntPtr.Zero)
                    {
                        why = "LoadLibrary error " + Marshal.GetLastWin32Error().ToString(CultureInfo.InvariantCulture);
                    }

                    return h;
                }

                var handle = dlopen(path, 2 /* RTLD_NOW */);
                if (handle == IntPtr.Zero)
                {
                    why = "dlopen failed";
                }

                return handle;
            }
            catch (Exception e) when (e is DllNotFoundException || e is EntryPointNotFoundException)
            {
                why = e.Message;
                return IntPtr.Zero;
            }
        }

        [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true, ExactSpelling = true)]
        private static extern IntPtr LoadLibraryW(string path);

        [DllImport("libdl", ExactSpelling = true)]
        private static extern IntPtr dlopen(string path, int flags);
#endif

        /// <summary>The runtime identifier of this process, e.g. "win-x64".</summary>
        internal static string Rid
        {
            get
            {
#if NET5_0_OR_GREATER
                return RuntimeInformation.RuntimeIdentifier;
#else
                var os = RuntimeInformation.IsOSPlatform(OSPlatform.Windows) ? "win"
                    : RuntimeInformation.IsOSPlatform(OSPlatform.OSX) ? "osx"
                    : "linux";
                return os + "-" + RuntimeInformation.ProcessArchitecture.ToString().ToLowerInvariant();
#endif
            }
        }

        /// <summary>The file name the runtime looks for on this OS.</summary>
        internal static string FileName =>
            RuntimeInformation.IsOSPlatform(OSPlatform.Windows) ? "iai_actuator_driver.dll"
            : RuntimeInformation.IsOSPlatform(OSPlatform.OSX) ? "libiai_actuator_driver.dylib"
            : "libiai_actuator_driver.so";

        /// <summary>The places the runtime searches (the override first when set).</summary>
        internal static IReadOnlyList<string> SearchedPaths()
        {
            var paths = new List<string>();
            if (_overridePath != null)
            {
                paths.Add(_overridePath + " (" + OverrideVariable + ")");
            }

            var baseDir = AppContext.BaseDirectory ?? string.Empty;
            paths.Add(Path.Combine(baseDir, FileName));
            var assemblyDir = Path.GetDirectoryName(typeof(NativeLoader).Assembly.Location);
            if (!string.IsNullOrEmpty(assemblyDir) && !string.Equals(Path.GetFullPath(assemblyDir!), Path.GetFullPath(baseDir.Length > 0 ? baseDir : "."), StringComparison.OrdinalIgnoreCase))
            {
                paths.Add(Path.Combine(assemblyDir!, FileName));
            }

            paths.Add(Path.Combine(baseDir, "runtimes", Rid, "native", FileName));
            paths.Add("the system library path (PATH / LD_LIBRARY_PATH)");
            return paths;
        }

        /// <summary>The text of the load error: what is missing, for which runtime, where it was looked for and the ABI needed.</summary>
        internal static string NotFoundMessage() => string.Format(
            CultureInfo.InvariantCulture,
            "the native library {0} ({1}, ABI {2}.{3}) was not found for {4}; searched: {5}. Put it next to the application, "
            + "reference the IaiActuatorDriver package (runtimes/{4}/native/), or set {6} to the file (build it with "
            + "'python build.py shared' in core/)",
            NativeMethods.Library,
            FileName,
            NativeDriver.AbiMajor,
            NativeDriver.AbiMinor,
            Rid,
            string.Join("; ", SearchedPaths()),
            OverrideVariable);

        /// <summary>The text of the error for a library built for another CPU.</summary>
        internal static string BadImageMessage() => string.Format(
            CultureInfo.InvariantCulture,
            "the native library {0} is built for another CPU than this {1} process ({2}): a 64-bit process needs the x64 build",
            NativeMethods.Library,
            Environment.Is64BitProcess ? "64-bit" : "32-bit",
            Rid);
    }
}
