using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The ABI 0.7 (core-fixes) test hook iad_testing_set_port_list: the serial ports every port listing of the core
    /// sees, process-wide. No port opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class Abi07CoreFixesBindingTests
    {
        [NativeFact]
        public void The_port_list_hook_replaces_the_pcs_ports_and_null_restores_them()
        {
            NativeDriver.ListPortsJson(); // loads the library
            NativeErrors.Check(NativeTestingMethods.iad_testing_set_port_list(Utf8.EncodeRequired("[{\"port\":\"COM42\",\"vid\":4292}]", "ports")));
            try
            {
                var listed = NativeDriver.ListPortsJson();
                Assert.Contains("\"port\":\"COM42\"", listed);
                Assert.Contains("\"vid\":4292", listed);
                // a wrong list is refused and leaves the one set
                var refused = NativeTestingMethods.iad_testing_set_port_list(Utf8.EncodeRequired("[{\"port\":\"\"}]", "ports"));
                Assert.Equal((int)NativeResult.InvalidArgument, refused);
                Assert.Contains("COM42", NativeDriver.ListPortsJson());
            }
            finally
            {
                NativeErrors.Check(NativeTestingMethods.iad_testing_set_port_list(null));
            }
            Assert.DoesNotContain("COM42", NativeDriver.ListPortsJson());
        }
    }
}
