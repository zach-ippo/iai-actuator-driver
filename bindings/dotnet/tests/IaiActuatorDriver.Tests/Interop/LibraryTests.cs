using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>Loading the library and its versions (iad_abi_version, iad_library_version, iad_last_error_message).</summary>
    public class LibraryTests
    {
        [NativeFact]
        public void The_abi_version_is_the_one_the_binding_was_written_for()
        {
            Assert.Equal((NativeDriver.AbiMajor, NativeDriver.AbiMinor), NativeDriver.AbiVersion);
            NativeDriver.EnsureLoaded();
            NativeDriver.EnsureLoaded();  // once is enough; twice is harmless
        }

        [NativeFact]
        public void The_library_version_is_a_version_number()
        {
            Assert.Matches(@"^\d+\.\d+\.\d+", NativeDriver.LibraryVersion);
        }

        [NativeFact]
        public void The_last_error_is_empty_on_a_new_thread()
        {
            string? message = null;
            var thread = new Thread(() => message = NativeDriver.LastErrorMessage);
            thread.Start();
            thread.Join();
            Assert.Equal(string.Empty, message);
        }

        [Fact]
        public void Abi_compatibility_is_the_same_major_and_at_least_the_minor()
        {
            Assert.Equal((1, 0), (NativeDriver.AbiMajor, NativeDriver.AbiMinor));
            Assert.True(NativeDriver.IsAbiCompatible(1, 0));
            Assert.True(NativeDriver.IsAbiCompatible(1, 1));  // a higher minor only adds
            Assert.True(NativeDriver.IsAbiCompatible(1, 7));
            Assert.False(NativeDriver.IsAbiCompatible(0, 8));  // the 0.8 development label of the same layout
            Assert.False(NativeDriver.IsAbiCompatible(0, 0));
            Assert.False(NativeDriver.IsAbiCompatible(2, 0));
        }

        [NativeFact]
        public void The_testing_header_has_its_own_exact_version()
        {
            NativeTestingMethods.EnsureCompatible();
            NativeTestingMethods.EnsureCompatible();  // checked once
            Assert.Equal(NativeTestingMethods.TestingAbiVersion, NativeTestingMethods.iad_testing_abi_version());
        }
    }
}
