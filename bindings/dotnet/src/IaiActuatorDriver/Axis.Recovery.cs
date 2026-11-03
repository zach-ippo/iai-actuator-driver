using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    // Power and link loss: what the axis remembers (the session), one cheap link check, and the recovery. The session, the link
    // state and the recovery are the core's; a recovery never starts motion. VERIFIED in the Python edition on the EC-GRBP8M
    // gripper (power cycle and line break) and the PCON-CB (line break); UNVERIFIED in this edition (simulator only).
    public sealed partial class Axis
    {
        /// <summary>Longest time a recovery waits for the port and the unit to answer again (<see cref="Recover"/>), s.</summary>
        public const double RecoverTimeoutS = 30.0;

        /// <summary>
        /// Remember what the application has set up while the link is up: the identity (serial number, model, firmware), the baud
        /// rate, PC control, safety speed, HEND, the uptime and - when asked - the alarm history, stored rows and parameters. The
        /// baseline <see cref="CheckLink"/> and <see cref="Recover"/> compare with. READ-ONLY. <paramref name="name"/> labels the
        /// session (default: the axis's "port:slave"); <paramref name="rows"/> copies stored rows ("all" or "0,1,5"; <c>null</c> =
        /// none); <paramref name="backupJson"/> is a <see cref="BackupJson"/> to compare against later.
        /// </summary>
        public RecoverySession Track(string? name = null, string? rows = null, bool alarmHistory = true, bool parameters = false, string? backupJson = null)
        {
            var o = NativeAxis.TrackOptions(alarmHistory, parameters);
            var n = Utf8.Encode(name, nameof(name));
            var r = Utf8.Encode(rows, nameof(rows));
            var b = Utf8.Encode(backupJson, nameof(backupJson));
            Run(() => NativeMethods.iad_axis_track(H, ref o, n, r, b));
            return Session ?? new RecoverySession();
        }

        /// <summary>
        /// A session from a <see cref="BackupJson"/> (its JSON text; no I/O): a recovery then requires the unit the backup was taken from
        /// (serial number, model). <see cref="ArgumentException"/> when the text holds no identity or is not JSON.
        /// </summary>
        public RecoverySession TrackBackup(string backupJson, string? name = null)
        {
            var b = Utf8.EncodeRequired(backupJson, nameof(backupJson));
            var n = Utf8.Encode(name, nameof(name));
            Run(() => NativeMethods.iad_axis_session_from_backup(H, b, n));
            return Session ?? new RecoverySession();
        }

        /// <summary>The same from a backup file (UTF-8 JSON); <see cref="IOException"/> when it cannot be read.</summary>
        public RecoverySession TrackBackupFile(string path, string? name = null)
        {
            var p = Utf8.EncodeRequired(path, nameof(path));
            var n = Utf8.Encode(name, nameof(name));
            Run(() => NativeMethods.iad_axis_session_from_backup_file(H, p, n));
            return Session ?? new RecoverySession();
        }

        /// <summary>
        /// A session from the identity this axis read last (<see cref="Connect(bool)"/> or <see cref="Identify"/>; no I/O): a recovery
        /// then checks the unit is the same one. A session the unit has already is kept. <see cref="EcNotReadyException"/> when the
        /// axis read no identity.
        /// </summary>
        public RecoverySession TrackIdentity(string? name = null)
        {
            var n = Utf8.Encode(name, nameof(name));
            Run(() => NativeMethods.iad_axis_session_from_info(H, null, n));
            return Session ?? new RecoverySession();
        }

        /// <summary>The unit's session (no I/O); <c>null</c> when it has none (<see cref="Track"/> was not called).</summary>
        public RecoverySession? Session
        {
            get
            {
                try
                {
                    return RecoverySession.From(Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_session_json(H, b, s, out r)));
                }
                catch (EcNotReadyException)
                {
                    return null;
                }
            }
        }

        /// <summary>Drop the unit's session, link state and remembered target.</summary>
        public void Forget() => Run(() => NativeMethods.iad_axis_forget(H));

        /// <summary>
        /// Record the last commanded target for the recovery's position check (the axis records its own moves; <c>null</c>: none, for
        /// example after a home return).
        /// </summary>
        public void NoteTarget(double? targetMm) => Run(() => NativeMethods.iad_axis_note_target(H, Known.OrNaN(targetMm)));

        /// <summary>Tell the session about a deliberate change so a recovery restores it: PC control, safety speed, baud rate (0 / <c>null</c>: unchanged) and HEND; <c>null</c> = unchanged.</summary>
        public void NoteControl(bool? modbusControl = null, bool? safetySpeed = null, int? baudRate = null, bool? homed = null) =>
            Run(() => NativeMethods.iad_axis_note_control(H, Known.Tri(modbusControl), Known.Tri(safetySpeed), baudRate ?? 0, Known.Tri(homed)));

        /// <summary>
        /// One cheap link check: one status read (22 words) compared with the session for restart signs. READ-ONLY. The state is "ok"
        /// (the session's baseline is moved forward), "port_vanished" / "unit_silent" (the read failed; <see cref="LinkCheck.Error"/>),
        /// "unit_restarted" (<see cref="LinkCheck.Reasons"/>; the baseline is kept for <see cref="Recover"/>) or "recovering". A link
        /// event reaches <see cref="LinkChanged"/> when the state changes. Without a session only the link itself is checked. An error that is not a
        /// link fault is thrown.
        /// </summary>
        public LinkCheck CheckLink()
        {
            IadLinkCheck check = default;
            Run(() => NativeMethods.iad_axis_check_link(H, out check));
            StartEventPumpIfListening();
            return LinkCheck.From(new NativeLinkCheck(check, Text(ThreadResultJson)));
        }

        /// <summary>
        /// The unit's last known link state, no I/O: "ok", "port_vanished", "unit_silent", "unit_restarted", "unit_replaced",
        /// "recovering" (<see cref="LinkStates"/>).
        /// </summary>
        public string LinkState => Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_link_state(H, b, s, out r));

        /// <summary>
        /// Bring the axis back after a power or link loss and report what changed: wait for the port, reopen it, find the baud rate,
        /// require the same unit, restore baud rate / PC control / safety speed (<paramref name="policy"/>), reset a message alarm and
        /// assess alarm, HEND and position. <b>Never starts motion.</b> Without a <see cref="Track"/> session the identity read last
        /// (<see cref="Connect(bool)"/> / <see cref="Identify"/>) is the baseline. The status monitor is held while it runs and restarted
        /// afterwards. <paramref name="timeout"/> (s) <c>null</c> = <see cref="RecoverTimeoutS"/>. Throws <see cref="RecoveryException"/>
        /// (port or unit did not come back; <see cref="RecoveryException.Report"/> has what was done) or
        /// <see cref="UnitReplacedException"/> (another unit answers: nothing was written to it). A cancelled
        /// <paramref name="cancellationToken"/> ends the wait with <see cref="OperationCanceledException"/>.
        /// </summary>
        public RecoveryReport Recover(double? timeout = null, RecoveryPolicy? policy = null, string? name = null, CancellationToken cancellationToken = default)
        {
            var p = policy ?? new RecoveryPolicy();
            var native = NativeRecoveryPolicy.ToNative(p.ToNative());
            var rows = p.CheckRows is null ? null : p.CheckRows.Count == 0 ? "all" : NativeLists.Rows(p.CheckRows);
            var backup = p.BackupJson ?? (p.BackupFile is null ? null : File.ReadAllText(p.BackupFile, Encoding.UTF8));
            var checkRows = Utf8.Encode(rows, nameof(policy));
            var backupJson = Utf8.Encode(backup, nameof(policy));
            var label = Utf8.Encode(name, nameof(name));
            IadRecoverySummary summary = default;
            try
            {
                Waiting(cancellationToken, () =>
                    NativeMethods.iad_axis_recover(H, Known.OrNaN(timeout), ref native, checkRows, backupJson, label, out summary));
            }
            catch (RecoveryException exc)
            {
                var details = MiniJson.TryParse(NativeErrors.ErrorJsonOf(exc)) as JsonObject;
                exc.Report ??= RecoveryReport.FromDetails(details?.Object("report") is { } report ? JsonPlain.Dict(report) : null);
                throw;
            }

            StartEventPumpIfListening();
            return RecoveryReport.From(new NativeRecoverySummary(summary, Text(ThreadResultJson)));
        }

        /// <summary>A recovery or a check queues link events: raise them when anything listens.</summary>
        private void StartEventPumpIfListening()
        {
            bool listening;
            lock (_sync)
            {
                listening = _linkChanged != null;
            }

            if (listening)
            {
                StartEventPump();
            }
        }
    }
}
