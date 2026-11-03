using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    // The compatibility self-test and the read-only backup. Both are the core's (info::backup, run_check).
    public sealed partial class Axis
    {
        /// <summary>The longest wait of the calling thread between two looks at the lines the self-test has finished.</summary>
        private const int CheckSliceMs = 50;

        /// <summary>A text output of a call that must run once (it reads the unit): a result too large for the buffer comes from the copy the core kept.</summary>
        private string TextOnce(NativeText.Call call, Func<string> kept)
        {
            TransportBridge.ClearError();
            var buffer = new byte[1024 * 1024];
            var result = call(buffer, NativeText.Size(buffer.Length), out _);
            if (result == (int)NativeResult.BufferTooSmall)
            {
                return kept();
            }

            if (result != 0)
            {
                throw TransportBridge.Raise(result);
            }

            return Utf8.FromBuffer(buffer);
        }

        // ------------------------------------------------------------------ backup

        /// <summary>
        /// A read-only backup of this unit as JSON text (the Python edition's <c>backup</c> layout, schema "ec_driver.backup/1"): identity,
        /// directory, support words, parameters, stored rows, alarm history and maintenance counters. Only FC03 and FC11 are sent.
        /// <paramref name="rows"/>: "used" (the rows the unit uses, the default), "all" (an EC has 768: slow) or numbers such as "0,1,2".
        /// <paramref name="path"/>: also written there (UTF-8). A backup of the same unit can be given to <see cref="Recover"/>
        /// (<see cref="RecoveryPolicy.BackupJson"/>) and to <see cref="TrackBackup"/>.
        /// </summary>
        public string BackupJson(string? path = null, string rows = "used")
        {
            var r0 = Utf8.Encode(rows, nameof(rows));
            var p = Utf8.Encode(path, nameof(path));
            return TextOnce(
                (byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_axis_backup_json(H, r0, p, b, n, out r),
                LastResultText);
        }

        /// <summary>
        /// <see cref="BackupJson"/> as a JSON-friendly dictionary: <c>meta</c> (schema, read_only, function_codes, taken_at, rows, duration_s,
        /// and <c>path</c> when written), <c>identity</c>, <c>directory</c>, <c>support</c>, <c>accl</c>, <c>parameters</c>, <c>positions</c>,
        /// <c>alarms</c>, <c>maintenance</c> and <c>errors</c> (what could not be read).
        /// </summary>
        public IDictionary<string, object?> Backup(string? path = null, string rows = "used") => JsonPlain.DictOf(BackupJson(path, rows));

        /// <summary><see cref="Backup(string?, string)"/> of the given stored rows.</summary>
        public IDictionary<string, object?> Backup(IEnumerable<int> rows, string? path = null) =>
            Backup(path, string.Join(",", (rows ?? throw new ArgumentNullException(nameof(rows))).Select(n => n.ToString(Inv))));

        /// <summary>
        /// <see cref="BackupJson"/> of several axes into <paramref name="directory"/> (created if missing), one <c>&lt;name&gt;.json</c> each
        /// (characters other than letters, digits, '-', '_' and '.' become '_'). <paramref name="names"/>: null = each axis's "port:slave", else one per
        /// axis. An axis that does not answer is reported and the others are still saved. Returns {name: file path, or "error: Type: message"}.
        /// </summary>
        public static IDictionary<string, string> BackupAll(IEnumerable<Axis> axes, string directory, IEnumerable<string>? names = null, string rows = "used")
        {
            var list = (axes ?? throw new ArgumentNullException(nameof(axes))).ToList();
            var json = NativeAxis.BackupAxesJson(list.Select(a => a._core), directory, names, rows);
            return JsonPlain.DictOf(json).ToDictionary(kv => kv.Key, kv => Convert.ToString(kv.Value, Inv) ?? string.Empty);
        }

        // ------------------------------------------------------------------ the self-test

        /// <summary>
        /// The compatibility self-test (the Python edition's <c>iai-check</c>): link, slave id, index table, identification, soft limits against
        /// the configured stroke, speed limit against the caps, status, alarm and alarm history, position table, direct-move block, Modbus
        /// control, safety speed, the unit information and the parameter ranges. Read-only (nothing is written to the unit) unless
        /// <see cref="CheckOptions.Motion"/>: then also a short move out and back inside the effective stroke, after which the servo and
        /// Modbus-control state it found is restored. A unit that does not answer, or answers differently, is a <see cref="CheckResult.Fail"/> line,
        /// never an exception. <paramref name="onItem"/> sees each line as it is finished, on the calling thread (the test itself runs on a helper
        /// thread meanwhile); an exception it throws, or a cancelled <paramref name="cancellationToken"/>, ends the test (a decel stop when the
        /// motion check moves) and is raised once the test has ended.
        /// </summary>
        public CheckReport RunCheck(CheckOptions? options = null, Action<CheckItem>? onItem = null, CancellationToken cancellationToken = default)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var o = options ?? new CheckOptions();
            var native = SizedStruct.Fresh<IadCheckOptions>();
            NativeErrors.Check(NativeMethods.iad_check_options_init(ref native));
            native.Motion = Known.Flag(o.Motion);
            native.StepMm = o.StepMm;
            native.SpeedMmS = o.SpeedMmS;
            native.AccelG = o.AccelG;
            var label = Utf8.Encode(o.PortLabel ?? (Bus.Transport is SerialTransport serial ? serial.PortName : null), nameof(options));

            var reportJson = string.Empty;
            Exception? failure = null;
            var worker = new Thread(() =>
            {
                try
                {
                    IadCheckSummary summary = default;
                    Waiting(cancellationToken, () => NativeMethods.iad_axis_self_test(H, ref native, label, out summary));
                    reportJson = LastResultText();  // the result is the thread's own: read it where the test ran
                }
                catch (Exception exc)
                {
                    failure = exc;
                }
            })
            {
                IsBackground = true,
                Name = "iad-check",
            };
            worker.Start();

            var delivered = 0;
            try
            {
                while (!worker.Join(CheckSliceMs))
                {
                    delivered = Deliver(delivered, onItem);
                }
            }
            catch (Exception)
            {
                // onItem threw: end the test (a decel stop when it moves), wait for it, then raise
                try
                {
                    NativeMethods.iad_axis_cancel_latch(H);
                    if (worker.Join(10000))
                    {
                        NativeMethods.iad_axis_cancel_reset(H);
                    }
                }
                catch (Exception cancelFailure)
                {
                    DriverLog.Warning(LogSource, "could not cancel the self-test: " + cancelFailure.Message);
                }

                throw;
            }

            if (failure != null)
            {
                ExceptionDispatchInfo.Capture(failure).Throw();
            }

            var report = CheckReport.From(reportJson);
            if (onItem != null)
            {
                foreach (var item in report.Items.Skip(delivered))
                {
                    onItem(item);
                }
            }

            return report;
        }

        /// <summary>Hand <paramref name="onItem"/> the lines the running test has finished since <paramref name="first"/>; the number delivered so far.</summary>
        private int Deliver(int first, Action<CheckItem>? onItem)
        {
            if (onItem is null)
            {
                return first;
            }

            var json = Text((byte[]? b, UIntPtr n, out UIntPtr r) => NativeMethods.iad_axis_self_test_items_json(H, first, b, n, out r));
            var items = (MiniJson.TryParse(json) as List<object?> ?? new List<object?>()).OfType<JsonObject>().Select(CheckItem.From).ToList();
            foreach (var item in items)
            {
                onItem(item);
                first++;
            }

            return first;
        }
    }
}
