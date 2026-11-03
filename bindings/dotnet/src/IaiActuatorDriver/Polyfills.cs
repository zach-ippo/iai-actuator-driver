// Language-feature polyfills for netstandard2.0 (records and init accessors need this marker type).

#if !NET5_0_OR_GREATER
namespace System.Runtime.CompilerServices
{
    /// <summary>Lets the compiler emit <c>init</c> accessors and records on netstandard2.0.</summary>
    internal static class IsExternalInit
    {
    }
}
#endif
