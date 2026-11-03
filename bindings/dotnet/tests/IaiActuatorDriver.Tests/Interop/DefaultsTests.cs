using System;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The C ABI 1.0 rule of the defaults as the binding writes it: IAD_UNKNOWN is 2, 0 is the default of every
    /// enumeration (IAD_METHOD_* start at 1), a struct without an _init function never gets an explicit 0 for its speed,
    /// acceleration or band, and every import and callback is IAD_CALL (CallingConvention.Winapi). No native library
    /// needed.
    /// </summary>
    public class DefaultsTests
    {
        [Fact]
        public void A_tribool_is_0_1_or_2_and_unknown_is_2()
        {
            Assert.Equal(2, NativeMethods.TriUnknown);
            Assert.Null(Known.Tri(NativeMethods.TriUnknown));
            Assert.False(Known.Tri(0));
            Assert.True(Known.Tri(1));
            Assert.Equal(NativeMethods.TriUnknown, Known.Tri((bool?)null));
            Assert.Equal(0, Known.Tri((bool?)false));
            Assert.Equal(1, Known.Tri((bool?)true));
        }

        [Fact]
        public void Command_strategies_start_at_1_and_the_default_is_0()
        {
            Assert.Equal(1, (int)NativeCommandStrategy.Coil);
            Assert.Equal(2, (int)NativeCommandStrategy.Iai);
            Assert.Equal(3, (int)NativeCommandStrategy.Iai2);
            Assert.Equal(4, (int)NativeCommandStrategy.Register);
            Assert.Equal(0, NativeAxisOptions.MethodCode(null));
            Assert.Equal(1, NativeAxisOptions.MethodCode(CommandMethod.Coil));
            Assert.Equal(2, NativeAxisOptions.MethodCode(CommandMethod.Iai));
            Assert.Equal(3, NativeAxisOptions.MethodCode(CommandMethod.Iai2));
        }

        [Fact]
        public void An_explicit_zero_is_refused_where_the_core_reads_it_as_the_default()
        {
            Assert.True(double.IsNaN(Known.Slot("speed", null)));
            Assert.True(double.IsNaN(Known.Slot("speed", double.NaN)));
            Assert.Equal(12.0, Known.Slot("speed", 12.0));
            Assert.Throws<SoftLimitException>(() => Known.Slot("speed", 0.0));
            Assert.Throws<SoftLimitException>(() => new NativeSequencePoint(10.0, speed: 0.0).ToNative());
            Assert.Throws<SoftLimitException>(() => new NativeSequencePoint(10.0, band: 0.0).ToNative());
            Assert.Throws<SoftLimitException>(() => new NativeAxisMove("COM3:1", 10.0) { AccelG = 0.0 }.ToNative());
            var move = new NativeAxisMove("COM3:1", 10.0).ToNative();
            Assert.Equal(50.0, move.Speed);
            Assert.Equal(0.1, move.AccelG);
        }

        [Fact]
        public void Every_import_and_callback_uses_the_platform_convention()
        {
            const BindingFlags all = BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly;
            foreach (var type in new[] { typeof(NativeMethods), typeof(NativeTestingMethods) })
            {
                var imports = type.GetMethods(all).Select(m => (m, a: m.GetCustomAttribute<DllImportAttribute>())).Where(x => x.a != null).ToList();
                Assert.True(imports.Count >= 25, type.Name);
                foreach (var (method, attribute) in imports)
                {
                    Assert.True(attribute!.CallingConvention == CallingConvention.Winapi, $"{type.Name}.{method.Name}");
                }
            }

            var delegates = typeof(NativeMethods).GetNestedTypes(all).Where(t => typeof(Delegate).IsAssignableFrom(t)).ToList();
            Assert.NotEmpty(delegates);
            foreach (var type in delegates)
            {
                var pointer = type.GetCustomAttribute<UnmanagedFunctionPointerAttribute>();
                Assert.True(pointer != null && pointer.CallingConvention == CallingConvention.Winapi, type.Name);
            }
        }
    }
}
