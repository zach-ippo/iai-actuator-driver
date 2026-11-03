using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    // The axis on the native core (switch stage S3): an Axis is the core's axis (iad_axis, C++ Axis) opened on the core
    // bus under its ModbusRtu (iad_axis_open_bus). Every read, guard, wait, retry and the non-volatile accounting run in the
    // core; this class keeps the .NET argument checks, the settings it was given (pushed to the core's options) and turns
    // the core's outputs into the public records. A call reaches a managed transport (FakeTransport, a test double)
    // through the callback transport on the calling thread, so its exception comes back as before (TransportBridge).
    public sealed partial class Axis
    {
        /// <summary>Most words a status sample has (0x9000 x22).</summary>
        private const int StatusWordsMax = 22;

        /// <summary>The non-volatile bookkeeping key of each transport other than a serial port (by the object).</summary>
        private static readonly ConditionalWeakTable<ITransport, NvTransportKey> NvKeys = new ConditionalWeakTable<ITransport, NvTransportKey>();

        private static long _nextNvKey;

        private readonly NativeAxis _core;

        /// <summary>The core axis handle.</summary>
        private AxisHandle H => _core.Handle;

        /// <summary>The core's axis under this object (the interop tests reach the functions this class does not wrap).</summary>
        internal NativeAxis Core => _core;

        /// <summary>Open the core's axis on the bus and give it this unit's non-volatile bookkeeping (shared per unit).</summary>
        private NativeAxis OpenCore()
        {
            var core = NativeAxis.OpenOnBus(Bus.Native, Slave, CoreOptions(), Bus.Transport is SerialTransport serial ? serial.PortName + ":" + Slave.ToString(Inv) : null);
            try
            {
                var (key, label) = NvUnit(Bus.Transport, Slave);
                NativeErrors.Check(NativeMethods.iad_axis_set_nv_unit(core.Handle, Utf8.Encode(key), Utf8.Encode(label)));
                if (_ownsBus)
                {
                    // the one user of its bus (Axis.Open): a fast mode may change the port's rate (iad_axis_enable_fast)
                    NativeErrors.Check(NativeMethods.iad_axis_set_port_owner(core.Handle, 1));
                }
            }
            catch
            {
                core.Close();
                throw;
            }

            return core;
        }

        /// <summary>
        /// Take over a core axis that exists already (a cell's own axis on this bus, iad_cell_axis_ref): this object's
        /// settings and the unit's non-volatile bookkeeping go to it, as to an axis this object opens.
        /// </summary>
        private NativeAxis AdoptCore(NativeAxis core)
        {
            try
            {
                var o = CoreOptions().ToNative();
                NativeErrors.Check(NativeMethods.iad_axis_set_options(core.Handle, ref o));
                var (key, label) = NvUnit(Bus.Transport, Slave);
                NativeErrors.Check(NativeMethods.iad_axis_set_nv_unit(core.Handle, Utf8.Encode(key), Utf8.Encode(label)));
            }
            catch
            {
                core.Close();
                throw;
            }

            return core;
        }

        /// <summary>
        /// The key and label of the non-volatile write guard's bookkeeping of <paramref name="slave"/> on
        /// <paramref name="transport"/>: a serial port by its name (a new handle on the same port reaches the same unit;
        /// any case on Windows), any other transport (a simulator, a test double) by the object itself.
        /// </summary>
        private static (string? Key, string Label) NvUnit(ITransport transport, int slave)
        {
            var n = slave.ToString(Inv);
            if (transport is SerialTransport serial)
            {
                return (NvPortKey(serial.PortName, slave), serial.PortName + " slave " + n);
            }

            var id = NvKeys.GetValue(transport, _ => new NvTransportKey(Interlocked.Increment(ref _nextNvKey))).Id;
            return ("obj:" + id.ToString(Inv) + ":" + n, "slave " + n);
        }

        /// <summary>The core's bookkeeping key of <paramref name="slave"/> on the serial port <paramref name="portName"/> (Windows: any case).</summary>
        internal static string NvPortKey(string portName, int slave) =>
            "port:" + (RuntimeInformation.IsOSPlatform(OSPlatform.Windows) ? portName.ToLowerInvariant() : portName) + ":" + slave.ToString(Inv);

        /// <summary>The axis settings as the core's options (a push settle of 0: none, the first sample counts).</summary>
        private NativeAxisOptions CoreOptions()
        {
            lock (_sync)
            {
                return new NativeAxisOptions
                {
                    Stroke = _stroke,
                    MaxSpeedMmS = _maxSpeed,
                    MaxAccelG = _maxAccel,
                    MaxPushPct = _maxPush,
                    Strategy = _strategy.Method,
                    PollIntervalS = _pollInterval,
                    SettleS = _settle,
                    AutoModbusControl = _autoModbusControl,
                    PushSettleS = _pushSettle > 0 ? _pushSettle : -1.0,
                    Completion = _completion is { } rule ? (NativeCompletion)(int)rule : (NativeCompletion?)null,
                };
            }
        }

        /// <summary>Hand the settings to the core axis (after the constructor: every setter).</summary>
        private void ApplyOptions()
        {
            if (_core is null)
            {
                return;  // still constructing: the core axis opens with them
            }

            var o = CoreOptions().ToNative();
            bool pending;
            lock (_sync)
            {
                pending = _completion.HasValue;
            }

            var current = SizedStruct.Fresh<IadAxisOptions>();
            if (!pending && NativeMethods.iad_axis_get_options(H, ref current) == 0)
            {
                o.Completion = current.Completion;  // the completion rule lives in the core (fast mode sets it): a setter of another option keeps it
            }

            NativeErrors.Check(NativeMethods.iad_axis_set_options(H, ref o));
        }

        // ------------------------------------------------------------------ calls

        /// <summary>One call of the core axis: its error, or what a managed transport threw during it.</summary>
        private static void Run(Func<int> call) => NativeBus.Run(call);

        /// <summary>
        /// A call that may wait (a move, home, a stop): <paramref name="cancellationToken"/> sets the core's cancel latch,
        /// which ends the wait (a decel stop first when motion was waited for) and is cleared again when the call returns.
        /// A cancelled call raises <see cref="OperationCanceledException"/> with the token. A token cancelled before the call
        /// sends nothing but the decel stop (STOP coil sequence), as every cancelled motion wait ends.
        /// </summary>
        private void Waiting(CancellationToken cancellationToken, Func<int> call)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                try
                {
                    Stop();
                }
                catch (Exception exc) when (IsLinkError(exc))
                {
                    DriverLog.Warning(LogSource, "decel stop after the cancellation failed: " + exc.Message);
                }

                throw new OperationCanceledException(cancellationToken);
            }

            if (!cancellationToken.CanBeCanceled)
            {
                Run(call);
                return;
            }

            var latched = 0;
            int result;
            TransportBridge.ClearError();
            using (cancellationToken.Register(() =>
            {
                Interlocked.Exchange(ref latched, 1);
                NativeMethods.iad_axis_cancel_latch(H);
            }))
            {
                result = call();
            }

            Exception? error = null;
            if (result != 0)
            {
                error = TransportBridge.Raise(result);
            }

            if (Volatile.Read(ref latched) != 0)
            {
                NativeMethods.iad_axis_cancel_reset(H);
                if (result == (int)NativeResult.Cancelled)
                {
                    throw new OperationCanceledException(error?.Message ?? "cancelled", error, cancellationToken);
                }
            }

            if (error != null)
            {
                throw error;
            }
        }

        /// <summary>A text output of the core axis (JSON), through the transport bridge.</summary>
        private string Text(NativeText.Call call)
        {
            TransportBridge.ClearError();
            var buffer = new byte[NativeText.InitialSize];
            var result = call(buffer, NativeText.Size(buffer.Length), out var required);
            if (result == (int)NativeResult.BufferTooSmall)
            {
                buffer = new byte[Math.Max(NativeText.ToInt(required), 1)];
                TransportBridge.ClearError();
                result = call(buffer, NativeText.Size(buffer.Length), out _);
            }

            if (result != 0)
            {
                throw TransportBridge.Raise(result);
            }

            return Utf8.FromBuffer(buffer);
        }

        private object? Json(NativeText.Call call) => MiniJson.Parse(Text(call));

        private JsonObject JsonObj(NativeText.Call call) => Json(call) as JsonObject ?? new JsonObject();

        /// <summary>What the core axis keeps without I/O (iad_axis_state_json).</summary>
        private JsonObject State() => JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_state_json(H, b, s, out r));

        /// <summary>
        /// The details of the call that just ran on this thread (iad_last_result_json; no I/O): its own, never the result of a call on
        /// another thread that replaced the axis's meanwhile (a watchdog's <see cref="CheckLink"/>).
        /// </summary>
        private JsonObject LastResult() => JsonObj(ThreadResultJson);

        /// <summary>iad_last_result_json as a <see cref="NativeText.Call"/>.</summary>
        private static int ThreadResultJson(byte[]? buffer, UIntPtr size, out UIntPtr required) => NativeMethods.iad_last_result_json(buffer, size, out required);

        // ------------------------------------------------------------------ statuses

        /// <summary>
        /// The public <see cref="Status"/> of an iad_status the core returned: decoded from the raw words of that sample
        /// (iad_axis_last_status_words), with the core's position (an RCON unit counting 0.001 mm) and velocity; null
        /// when the struct holds no sample.
        /// </summary>
        private Status? ToStatus(IadStatus s)
        {
            if (s.RawWords <= 0 || double.IsNaN(s.PositionMm))
            {
                return null;
            }

            var words = new ushort[StatusWordsMax];
            Status st;
            if (NativeMethods.iad_axis_last_status_words(H, ref s, words, NativeText.Size(words.Length), out var count) == 0
                && NativeText.ToInt(count) >= Registers.STATUS_BLOCK_MIN_WORDS)
            {
                st = IaiActuatorDriver.Status.FromRegisters(words.Take(NativeText.ToInt(count)).ToArray());
            }
            else
            {
                st = new NativeStatus(s).ToRecord();  // a sample no longer kept: the 8 words the struct carries
            }

            var velocity = Known.Double(s.VelocityMmS);
            return st with { PositionMm = s.PositionMm, VelocityMmS = velocity.HasValue ? Math.Round(velocity.Value, 2) : (double?)null };
        }

        /// <summary>A status the core returned with a call that always writes one.</summary>
        private Status Required(IadStatus s) => ToStatus(s) ?? new NativeStatus(s).ToRecord();

        /// <summary>The public status of an iad_status the core returned for this axis elsewhere (a cell's group command); null without a sample.</summary>
        internal Status? StatusOf(IadStatus s) => ToStatus(s);

        /// <summary>The key of one transport object in the non-volatile bookkeeping; forgotten with the object.</summary>
        private sealed class NvTransportKey
        {
            internal NvTransportKey(long id) => Id = id;

            ~NvTransportKey()
            {
                try
                {
                    NativeMethods.iad_nv_forget(Utf8.Encode("obj:" + Id.ToString(CultureInfo.InvariantCulture) + ":"));
                }
                catch (Exception)
                {
                    // the library is gone (process exit): nothing to forget
                }
            }

            internal long Id { get; }
        }
    }
}
