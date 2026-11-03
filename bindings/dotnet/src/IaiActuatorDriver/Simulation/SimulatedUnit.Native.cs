using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.ExceptionServices;
using System.Text;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Simulation
{
    // The simulated unit on the native core's simulator (iad_sim, C++ HookedSimUnit). The constructor resolves the
    // settings and opens the core's unit from them; the members then convert: the properties read its state JSON, the
    // test hooks are its controls, the transport calls go to its simulated port.
    //
    // An override of the protected members runs in the core's request hook: every frame written to the unit goes through
    // WriteFrame, and HandleFrame, Fc11, Fc45, Fc48, WriteCoil and ReadWord are called for the requests they answer, their
    // base members giving the core's own answer (iad_sim_handle_frame, the one library call a hook may make). A frame no
    // override answers is passed on to the core unchanged, with its line model (injected faults, the FC11 delay); an answer
    // an override built goes out as it is, InjectFault inside the hook applying to it. An override that throws fails the
    // write (IAD_SIM_FAIL), and the write throws what it threw.
    public partial class SimulatedUnit
    {
        private const string NotInCore = "is not in the core simulator of this library";

        private static readonly ConcurrentDictionary<Type, CoreHooks> HooksByType = new ConcurrentDictionary<Type, CoreHooks>();

        /// <summary>The request the core's hook is running on this thread (an override's base members answer it).</summary>
        [ThreadStatic]
        private static HookCall? _hookCall;

        /// <summary>The unit's own answer to the read an override of <see cref="ReadWord"/> is answering on this thread.</summary>
        [ThreadStatic]
        private static OwnRead? _ownRead;

        /// <summary>What an override threw in the core's hook on this thread (the write that ran it throws it).</summary>
        [ThreadStatic]
        private static Exception? _hookFailure;

        private readonly NativeSim _core;
        private readonly TransportHandle _port;
        private readonly CoreHooks _coreHooks;

        [Flags]
        private enum CoreHooks
        {
            None = 0,
            WriteFrame = 1,
            HandleFrame = 2,
            Fc11 = 4,
            Fc45 = 8,
            Fc48 = 16,
            WriteCoil = 32,
            ReadWord = 64,
        }

        /// <summary>The core's simulator under this unit.</summary>
        internal NativeSim CoreSim => _core;

        private static CoreHooks HooksOf(Type type) => HooksByType.GetOrAdd(type, t =>
        {
            var hooks = CoreHooks.None;
            foreach (var (hook, name, parameters) in new[]
            {
                (CoreHooks.WriteFrame, nameof(WriteFrame), new[] { typeof(byte[]) }),
                (CoreHooks.HandleFrame, nameof(HandleFrame), new[] { typeof(byte[]) }),
                (CoreHooks.Fc11, nameof(Fc11), new[] { typeof(byte[]) }),
                (CoreHooks.Fc45, nameof(Fc45), new[] { typeof(byte[]) }),
                (CoreHooks.Fc48, nameof(Fc48), new[] { typeof(byte[]) }),
                (CoreHooks.WriteCoil, nameof(WriteCoil), new[] { typeof(int), typeof(bool) }),
                (CoreHooks.ReadWord, nameof(ReadWord), new[] { typeof(int) }),
            })
            {
                var method = t.GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public, null, parameters, null);
                if (method != null && method.DeclaringType != typeof(SimulatedUnit))
                {
                    hooks |= hook;
                }
            }

            return hooks;
        });

        /// <summary>
        /// The core's options JSON for this unit: the settings resolved by the constructor (the profile's values where the
        /// options leave them open), so the core's defaults never differ from this class's.
        /// </summary>
        private string CoreOptionsJson(SimulatedUnitOptions o, double positionMm)
        {
            var json = new StringBuilder();
            void Add(string key, string value) => json.Append(json.Length == 0 ? "{" : ", ").Append('"').Append(key).Append("\": ").Append(value);
            Add("profile", "\"" + ProfileName + "\"");
            Add("slave", Int(Slave));
            Add("time_scale", Num(TimeScale));
            Add("position_mm", Num(positionMm));
            Add("stroke_mm", Num(StrokeMm));
            Add("max_speed_mm_s", Num(MaxSpeedMmS));
            Add("max_accel_g", Num(MaxAccelG));
            Add("system_code", Int(SystemCode));
            Add("unit_code", Int(UnitCode));
            Add("firmware", "[" + Int(Firmware.High) + ", " + Int(Firmware.Low) + "]");
            Add("dopm", Dopm == DopmLayout.TwoPosition ? "\"two_position\"" : "\"three_position\"");
            Add("pmsl", Bool(o.Pmsl));
            Add("recompute_position_checksum", Bool(RecomputePositionChecksum));
            Add("drop_reply_after_retarget", Bool(DropReplyAfterRetarget));
            Add("stop_latches", Bool(StopLatches));
            Add("move_bit_clears_in_decel", Bool(MoveBitClearsInDecel));
            Add("safety_speed", Bool(o.SafetySpeed));
            Add("soft_limits", Bool(o.SoftLimits));
            Add("ls_range_mm", Num(o.LsRangeMm));
            Add("early_in_position", Bool(EarlyInPosition));
            Add("settle_tail_mm", Num(SettleTailMm));
            Add("settle_speed_mm_s", Num(SettleSpeedMmS));
            if (o.PushBlipMm.HasValue)
            {
                Add("push_blip_mm", Num(o.PushBlipMm.Value));
            }

            Add("push_blip_len_mm", Num(PushBlipLenMm));
            Add("absolute", Bool(Absolute));
            if (o.BaudRate.HasValue)
            {
                Add("baud_rate", Int(o.BaudRate.Value));
                Add("unit_baud_rate", Int(o.EcBaudRate ?? o.BaudRate.Value));
            }

            Add("boot_baud_rate", Int(BootBaudRate));
            if (ResetBaudRate.HasValue)
            {
                Add("reset_baud_rate", Int(ResetBaudRate.Value));
            }

            Add("reset_s", Num(ResetS));
            Add("slave_id_delay_s", Num(SlaveIdDelayS));
            Add("parw_busy_s", Num(ParwBusyS));
            Add("parw_dsse_busy", Bool(ParwDsseBusy));
            Add("ignored_baud_codes", "[" + string.Join(", ", IgnoredBaudCodes.OrderBy(c => c).Select(Int)) + "]");
            return json.Append('}').ToString();
        }

        /// <summary>
        /// Open the core's unit (the end of the constructor). Read-only data with an exact register form is served through
        /// the core's word overrides: <paramref name="index"/>, the index table of <see cref="SimulatedUnitOptions.BkecPath"/>,
        /// and <see cref="SimulatedUnitOptions.CollisionDetection"/> (MVPT b13). A <see cref="SimulatedUnitOptions.Clock"/>
        /// of the unit's own becomes the core unit's clock (iad_sim_set_clock).
        /// </summary>
        private (NativeSim Sim, TransportHandle Port, CoreHooks Hooks) OpenCore(SimulatedUnitOptions o, double positionMm, IReadOnlyDictionary<int, ushort>? index)
        {
            var sim = NativeSim.Open(CoreOptionsJson(o, positionMm));
            try
            {
                if (o.Clock != null)
                {
                    sim.SetClock(o.Clock);
                }

                if (index != null)
                {
                    foreach (var kv in index)
                    {
                        if (CoreServed(sim, kv.Key) != kv.Value)
                        {
                            sim.SetWordOverride(kv.Key, kv.Value);
                        }
                    }
                }

                if (o.CollisionDetection)
                {
                    sim.SetWordOverride(0x0103, (CoreServed(sim, 0x0103) ?? 0) | Registers.MVPT_CDAS);
                }

                var hooks = HooksOf(GetType());
                if (hooks != CoreHooks.None)
                {
                    var self = new WeakReference<SimulatedUnit>(this);
                    sim.SetRequestHook(frame => self.TryGetTarget(out var unit) ? unit.OnCoreRequest(frame) : NativeSimAnswer.Pass);
                }

                return (sim, sim.Transport(), hooks);
            }
            catch
            {
                sim.Dispose();
                throw;
            }
        }

        private static int? CoreServed(NativeSim sim, int address)
        {
            try
            {
                return sim.RegisterValue(address);
            }
            catch (ArgumentException)
            {
                return null;
            }
        }

        // ================================================================== state

        private JsonObject CoreState() => (JsonObject)MiniJson.Parse(_core.StateJson())!;

        private bool CoreBool(string key) => CoreState().Bool(key) ?? false;

        private int CoreInt(string key) => CoreState().Int(key) ?? 0;

        private double CoreNumber(string key) => CoreState().Number(key) ?? 0.0;

        private Dictionary<int, ushort> CoreWords(string key)
        {
            var words = new Dictionary<int, ushort>();
            if (CoreState().Object(key) is { } map)
            {
                foreach (var address in map.Keys)
                {
                    words[int.Parse(address, NumberStyles.Integer, CultureInfo.InvariantCulture)] = (ushort)(map.Int(address) ?? 0);
                }
            }

            return words;
        }

        private JsonObject CoreControl(string command, string? arguments = null) =>
            (MiniJson.Parse(_core.Control("{\"command\": \"" + command + "\"" + (arguments is null ? string.Empty : ", " + arguments) + "}")) as JsonObject) ?? new JsonObject();

        /// <summary>A control the core may not have yet: <see cref="NotSupportedException"/> naming it while the core refuses it.</summary>
        private void CoreControlNew(string command, string arguments, string what)
        {
            try
            {
                CoreControl(command, arguments);
            }
            catch (ArgumentException exc) when (exc.Message.IndexOf("unknown simulator", StringComparison.Ordinal) >= 0)
            {
                throw new NotSupportedException($"SimulatedUnit: {what} {NotInCore} ({exc.Message})", exc);
            }
        }

        private void CoreSetState(string key, bool value, string what) => CoreControlNew("set_state", "\"" + key + "\": " + Bool(value), what);

        /// <summary>The core's contact blip (state "push_blip_mm"; a library without it: the value set here last).</summary>
        private double? CorePushBlip()
        {
            var state = CoreState();
            if (state.Has("push_blip_mm"))
            {
                return state.Number("push_blip_mm");
            }

            lock (_sync)
            {
                return _pushBlipMm;
            }
        }

        private void CoreSetPushBlip(double? value)
        {
            CoreControlNew("set_option", "\"name\": \"push_blip_mm\", \"value\": " + (value.HasValue ? Num(value.Value) : "null"), "changing PushBlipMm");
            lock (_sync)
            {
                _pushBlipMm = value;
            }
        }

        private (double Min, double Max) CoreSoftLimits()
        {
            var span = CoreState().Array("soft_limits_mm");
            return span is { Count: 2 } && span[0] is double lo && span[1] is double hi ? (lo, hi) : _softLimits;
        }

        private double? CoreObstacle() => CoreState().Number("obstacle_mm");

        private void CoreSetObstacle(double? mm) => CoreControl("set_obstacle", "\"mm\": " + (mm.HasValue ? Num(mm.Value) : "null"));

        private IReadOnlyList<byte[]> CoreRequests()
        {
            var list = MiniJson.Parse(_core.RequestsJson()) as List<object?> ?? new List<object?>();
            return list.OfType<JsonObject>().Select(r => FromHex(r.String("frame") ?? string.Empty)).ToList();
        }

        private IReadOnlyList<string> CoreEvents() => (MiniJson.Parse(_core.EventsJson()) as List<object?> ?? new List<object?>()).OfType<string>().ToList();

        private int? CoreUnitBaudRate() => CoreState().Int("unit_baud_rate");

        private bool CoreCoilState(int address) => CoreControl("coil_state", "\"address\": " + Int(address)).Bool("on") ?? false;

        private void CoreSetCoilState(int address, bool on) => CoreControl("set_coil_state", "\"address\": " + Int(address) + ", \"on\": " + Bool(on));

        private ushort[] CorePositionRow(int n) =>
            (CoreControl("position_row", "\"row\": " + Int(n)).Array("words") ?? new List<object?>()).Select(w => (ushort)(w is double d ? d : 0)).ToArray();

        private void CoreInjectFault(SimFault kind, int count)
        {
            if (_hookCall?.Unit == this)
            {
                lock (_sync)
                {
                    for (var i = 0; i < count; i++)
                    {
                        _faults.Enqueue(kind);  // inside the request hook: applied to the answer of this frame
                    }
                }

                return;
            }

            if (count > 0)
            {
                var name = kind switch
                {
                    SimFault.Crc => "crc",
                    SimFault.Partial => "partial",
                    SimFault.WrongSlave => "wrong_slave",
                    _ => "timeout",
                };
                CoreControl("inject_fault", "\"kind\": \"" + name + "\", \"count\": " + Int(count));
            }
        }

        private void CoreTriggerAlarm(int code) => CoreControl("trigger_alarm", "\"code\": " + Int(code));

        private bool CorePioMove(double targetMm, double speedMmS, double accelG, bool holdStart) =>
            CoreControl("pio_move", "\"target_mm\": " + Num(targetMm) + ", \"speed_mm_s\": " + Num(speedMmS) + ", \"accel_g\": " + Num(accelG) + ", \"hold_start\": " + Bool(holdStart)).Bool("started") ?? false;

        // ================================================================== the simulated port

        /// <summary>One call on the simulated port: a failure of the port (closed, unplugged) as an <see cref="IOException"/>.</summary>
        private static void CoreIo(string what, Func<int> call)
        {
            var result = call();
            if (result == 0)
            {
                return;
            }

            var error = NativeErrors.Capture(result);
            if (error is ModbusException)
            {
                var text = error.Message.StartsWith("simulated", StringComparison.Ordinal) ? error.Message : $"cannot {what} the simulated port: {error.Message}";
                throw new IOException(text, error);
            }

            throw error;
        }

        private void CoreWrite(byte[] frame)
        {
            _hookFailure = null;
            var result = NativeMethods.iad_transport_write(_port, frame, NativeText.Size(frame.Length));
            var failure = _hookFailure;
            _hookFailure = null;
            if (failure != null && result != 0)
            {
                ExceptionDispatchInfo.Capture(failure).Throw();  // an override threw: its own exception
            }

            CoreIo("write to", () => result);
        }

        private int CoreRead(byte[] buffer, int offset, int count)
        {
            var data = new byte[Math.Max(count, 0)];
            var got = UIntPtr.Zero;
            CoreIo("read from", () => NativeMethods.iad_transport_read(_port, data, NativeText.Size(data.Length), out got));
            var n = Math.Min(NativeText.ToInt(got), count);
            Array.Copy(data, 0, buffer, offset, n);
            return n;
        }

        private int CoreBaudRate() => CoreState().Int("baud_rate") ?? 0;

        private void CoreSetBaudRate(int value)
        {
            if (CoreBaudRate() == 0)
            {
                throw new NotSupportedException("this SimulatedUnit has no baud rate (set SimulatedUnitOptions.BaudRate to model the line)");
            }

            CoreIo("set the baud rate of", () => NativeMethods.iad_transport_set_baud_rate(_port, value));
        }

        /// <summary>Close the simulated port (a handle of its own is closed; this unit's handle stays, its I/O then fails).</summary>
        private void CoreClose()
        {
            using var port = _core.Transport();
            NativeErrors.Check(NativeMethods.iad_transport_close(port));
            port.SetHandleAsInvalid();
        }

        // ================================================================== request hook (overrides of the protected members)

        private NativeSimAnswer OnCoreRequest(byte[] frame)
        {
            var outer = _hookCall;
            var call = new HookCall(this);
            _hookCall = call;
            try
            {
                WriteFrame(frame);
            }
            catch (Exception exc)
            {
                _hookFailure = exc;  // the write fails and throws it (CoreWrite)
                return NativeSimAnswer.Fail(exc.Message);
            }
            finally
            {
                _hookCall = outer;
            }

            if (call.Pass)
            {
                return NativeSimAnswer.Pass;
            }

            return call.Reply is null ? NativeSimAnswer.Drop : new NativeSimAnswer(NativeSimAction.Reply, call.Reply);
        }

        /// <summary>The base <see cref="WriteFrame"/> on the core: inside the hook, the answer; else the frame to the port.</summary>
        private void CoreWriteFrame(byte[] data)
        {
            var call = _hookCall;
            if (call?.Unit != this)
            {
                CoreWrite(data);
                return;
            }

            bool faulted;
            lock (_sync)
            {
                faulted = _faults.Count > 0;
            }

            if (!faulted && !AnswersItself(data))
            {
                call.Pass = true;  // the core's own handling, its line model included
                return;
            }

            var response = HandleFrame(data);
            if (response != null)
            {
                lock (_sync)
                {
                    response = _faults.Count > 0 ? WithFault(response, _faults.Dequeue()) : response;
                }
            }

            call.Reply = response;
        }

        private static byte[]? WithFault(byte[] response, SimFault fault)
        {
            switch (fault)
            {
                case SimFault.Timeout:
                    return null;
                case SimFault.Crc:
                    response = (byte[])response.Clone();
                    response[response.Length - 1] ^= 0xFF;
                    return response;
                case SimFault.Partial:
                    return response.Take(Math.Max(1, response.Length / 2)).ToArray();
                default:
                    return Crc16.BuildFrame((response[0] % 247) + 1, response.Skip(1).Take(response.Length - 3).ToArray());
            }
        }

        /// <summary>An override answers <paramref name="frame"/> (else the core's own handling runs unchanged).</summary>
        private bool AnswersItself(byte[] frame)
        {
            if ((_coreHooks & CoreHooks.HandleFrame) != 0)
            {
                return true;
            }

            return frame.Length > 1 && (_coreHooks & HookFor(frame[1])) != 0;
        }

        private static CoreHooks HookFor(int function) => function switch
        {
            0x03 => CoreHooks.ReadWord,
            0x17 => CoreHooks.ReadWord,
            0x05 => CoreHooks.WriteCoil,
            0x11 => CoreHooks.Fc11,
            0x45 => CoreHooks.Fc45,
            0x48 => CoreHooks.Fc48,
            _ => CoreHooks.None,
        };

        /// <summary>The base <see cref="HandleFrame"/> on the core: the core's answer, the overridden functions through their members.</summary>
        private byte[]? CoreHandleFrame(byte[] frame)
        {
            if (frame.Length < 4 || !Crc16.Check(frame))
            {
                return null;
            }

            int address = frame[0], function = frame[1];
            if (address != Slave && address != 0)
            {
                return null;
            }

            if ((_coreHooks & HookFor(function)) == 0)
            {
                return CoreAnswer(frame);
            }

            var body = new byte[frame.Length - 4];
            Array.Copy(frame, 2, body, 0, body.Length);
            byte[] pdu;
            try
            {
                pdu = function switch
                {
                    0x03 => CoreReadPdu(frame, body),
                    0x17 => CoreReadPdu(frame, body),
                    0x05 => CoreFc05(body),
                    0x11 => Fc11(body),
                    0x45 => Fc45(body),
                    _ => Fc48(body),
                };
            }
            catch (SimException exc)
            {
                pdu = new[] { (byte)(function | 0x80), (byte)exc.Code };
            }
            catch (NoAnswerException)
            {
                return null;
            }

            return address == 0 ? null : Crc16.BuildFrame(Slave, pdu);
        }

        /// <summary>The core's whole answer to <paramref name="frame"/> (it acts on it); null: no answer.</summary>
        private byte[]? CoreAnswer(byte[] frame)
        {
            var answer = _core.HandleFrame(frame);
            return answer.Length == 0 ? null : answer;
        }

        /// <summary>The core's answer PDU to <paramref name="function"/> with <paramref name="body"/>; its exception as <see cref="SimException"/>.</summary>
        private byte[] CorePdu(int function, byte[] body)
        {
            var answer = CoreAnswer(Crc16.BuildFrame(Slave, Prepend((byte)function, body))) ?? throw new NoAnswerException();
            if (answer.Length >= 5 && answer[1] == (function | 0x80))
            {
                throw new SimException(answer[2]);
            }

            return answer.Skip(1).Take(answer.Length - 3).ToArray();
        }

        private byte[] CoreFc05(byte[] body)
        {
            if (body.Length != 4)
            {
                throw new SimException(0x03);
            }

            int address = U16(body, 0), value = U16(body, 2);
            if (value != 0xFF00 && value != 0x0000)
            {
                throw new SimException(0x03);
            }

            WriteCoil(address, value == 0xFF00);
            return Prepend(0x05, body);
        }

        /// <summary>The base <see cref="WriteCoil"/> on the core: the coil frame to the core's unit.</summary>
        private void CoreWriteCoil(int address, bool on) =>
            CorePdu(0x05, new[] { (byte)(address >> 8), (byte)address, (byte)(on ? 0xFF : 0x00), (byte)0x00 });

        /// <summary>An FC03 / FC17 answer with the words through <see cref="ReadWord"/>, the core's own answer under it.</summary>
        private byte[] CoreReadPdu(byte[] frame, byte[] body)
        {
            var function = frame[1];
            if (body.Length < 4)
            {
                throw new SimException(0x03);
            }

            int start = U16(body, 0), count = U16(body, 2);
            var own = CoreAnswer(frame) ?? throw new NoAnswerException();
            var read = new OwnRead(this, start);
            if (own.Length >= 5 && own[1] == (function | 0x80))
            {
                read.Code = own[2];
            }
            else
            {
                read.Words = Enumerable.Range(0, (own.Length - 5) / 2).Select(i => (ushort)((own[3 + (2 * i)] << 8) | own[4 + (2 * i)])).ToArray();
            }

            if (count < 1 || count > 125)
            {
                throw new SimException(read.Code != 0 ? read.Code : 0x03);
            }

            var outer = _ownRead;
            _ownRead = read;
            try
            {
                var pdu = new byte[2 + (2 * count)];
                pdu[0] = function;
                pdu[1] = (byte)(2 * count);
                for (var i = 0; i < count; i++)
                {
                    var w = ReadWord(start + i);
                    pdu[2 + (2 * i)] = (byte)(w >> 8);
                    pdu[3 + (2 * i)] = (byte)w;
                }

                return pdu;
            }
            finally
            {
                _ownRead = outer;
            }
        }

        /// <summary>The base <see cref="ReadWord"/> on the core: the word of the read being answered, else the core's value now.</summary>
        private int CoreReadWord(int address)
        {
            var read = _ownRead;
            if (read?.Unit == this && address >= read.Start && (read.Code != 0 || address < read.Start + read.Words.Length))
            {
                if (read.Code != 0)
                {
                    throw new SimException(read.Code);
                }

                return read.Words[address - read.Start];
            }

            if (_hookCall?.Unit == this)
            {
                var pdu = CorePdu(0x03, new[] { (byte)(address >> 8), (byte)address, (byte)0x00, (byte)0x01 });
                return (pdu[2] << 8) | pdu[3];
            }

            try
            {
                return _core.RegisterValue(address);
            }
            catch (ArgumentOutOfRangeException)
            {
                throw new SimException(0x02);
            }
        }

        // ================================================================== helpers

        private static string Int(int value) => value.ToString(CultureInfo.InvariantCulture);

        private static string Num(double value) => value.ToString("R", CultureInfo.InvariantCulture);

        private static string Bool(bool value) => value ? "true" : "false";

        private static byte[] FromHex(string text) =>
            text.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries).Select(h => byte.Parse(h, NumberStyles.HexNumber, CultureInfo.InvariantCulture)).ToArray();

        /// <summary>A request the core's hook runs on (per thread).</summary>
        private sealed class HookCall
        {
            internal HookCall(SimulatedUnit unit) => Unit = unit;

            internal SimulatedUnit Unit { get; }

            /// <summary>No override answered: the core handles the frame as usual.</summary>
            internal bool Pass { get; set; }

            /// <summary>The answer (null: none).</summary>
            internal byte[]? Reply { get; set; }
        }

        /// <summary>The core's own answer to the read an override of <see cref="ReadWord"/> is answering.</summary>
        private sealed class OwnRead
        {
            internal OwnRead(SimulatedUnit unit, int start)
            {
                Unit = unit;
                Start = start;
            }

            internal SimulatedUnit Unit { get; }

            internal int Start { get; }

            internal ushort[] Words { get; set; } = Array.Empty<ushort>();

            /// <summary>The Modbus exception the unit answered (0: none).</summary>
            internal int Code { get; set; }
        }

        /// <summary>The core's unit gave no answer to a request an override asked it for.</summary>
        private sealed class NoAnswerException : Exception
        {
        }
    }
}
