using System;
using System.Linq;
using System.Reflection;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The binding of the native core lives inside the library (one assembly, one package): internal, except the
    /// diagnostics types of <c>IaiActuatorDriver.Native</c>. No library call.
    /// </summary>
    public class InteropSurfaceTests
    {
        private static readonly Assembly Library = typeof(Axis).Assembly;

        [Fact]
        public void The_binding_is_part_of_the_library_assembly()
        {
            Assert.Same(Library, typeof(NativeDriver).Assembly);
            Assert.Same(Library, typeof(NativeAxis).Assembly);
            Assert.DoesNotContain(Library.GetReferencedAssemblies(), a => a.Name == "IaiActuatorDriver.Native");
        }

        [Fact]
        public void Only_the_diagnostics_types_are_public()
        {
            var interop = Library.GetExportedTypes().Where(t => t.Namespace == "IaiActuatorDriver.Interop").Select(t => t.Name);
            Assert.Empty(interop);
            var native = Library.GetExportedTypes().Where(t => t.Namespace == "IaiActuatorDriver.Native").Select(t => t.Name).OrderBy(n => n, StringComparer.Ordinal);
            Assert.Equal(new[] { "NativeDriver", "NativeErrors", "NativeResult" }, native);
        }

        [Fact]
        public void The_recovery_exceptions_are_library_exceptions()
        {
            Assert.Equal("IaiActuatorDriver", typeof(RecoveryException).Namespace);
            Assert.True(typeof(EcDriverException).IsAssignableFrom(typeof(UnitReplacedException)));
            Assert.IsType<UnitReplacedException>(NativeErrors.Create((int)NativeResult.UnitReplaced, "another unit answers", null));
        }
    }
}
