using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver
{
    /// <summary>System / unit code and firmware of a unit, from FC11 (Report Slave ID) or from CTCL (class 3 item 1).</summary>
    /// <param name="SystemCode">Byte 0 of FC11 (0xEC = EC 24 V, 0xAE = PCON, ...).</param>
    /// <param name="RunStatus">Byte 1 of FC11 (0xFF on every unit tested); <c>null</c> from CTCL.</param>
    /// <param name="UnitCode">Byte 2 of FC11: the product family's unit code.</param>
    /// <param name="Firmware">Firmware version as 4 hex digits.</param>
    /// <param name="Source">"FC11" or "CTCL 0x0300".</param>
    public sealed record SlaveIdentity(int SystemCode, int? RunStatus, int UnitCode, string Firmware, string Source)
    {
        /// <summary>
        /// Parse the FC11 data bytes <c>[system, run status, unit, firmware hi, firmware lo]</c> (VERIFIED: EC FF 02 00 13
        /// slider, EC FF 0A 00 02 gripper, EC FF 00 00 02 rod, AE FF 44 00 00 PCON). <c>null</c> for fewer than 5 bytes.
        /// </summary>
        public static SlaveIdentity? FromSlaveId(IReadOnlyList<byte>? data)
        {
            if (data is null || data.Count < 5)
            {
                return null;
            }

            return new SlaveIdentity(data[0], data[1], data[2], string.Format(CultureInfo.InvariantCulture, "{0:X2}{1:X2}", data[3], data[4]), "FC11");
        }

        /// <summary>
        /// Parse CTCL (u32: b31-24 system, b23-16 unit, b15-0 firmware; VERIFIED equal to FC11 on every unit tested).
        /// <c>null</c> when the system byte is 0 (no identity there).
        /// </summary>
        public static SlaveIdentity? FromCtcl(uint value)
        {
            if (value >> 24 == 0)
            {
                return null;
            }

            return new SlaveIdentity((int)(value >> 24), null, (int)((value >> 16) & 0xFF), (value & 0xFFFF).ToString("X4", CultureInfo.InvariantCulture), "CTCL 0x0300");
        }
    }

    /// <summary>
    /// The identification block (0x0600, 72 words; VERIFIED on the EC). Text fields are printable ASCII, <c>null</c> when
    /// blank or not text (a CON controller keeps binary data where an EC keeps its model string).
    /// </summary>
    public sealed record IdentificationBlock(string? SerialNumber, string? ActuatorSerial, string? UserName, string? ModelName, uint? Accl)
    {
        /// <summary>ACCL bits 8-11 (0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo; UNVERIFIED).</summary>
        public int? ExtensibilityLevel => Accl.HasValue ? (int)((Accl.Value >> 8) & 0xF) : (int?)null;

        /// <summary>Decode the block's words (fields past the end of a short block are <c>null</c>).</summary>
        public static IdentificationBlock FromWords(IReadOnlyList<ushort> words)
        {
            if (words is null)
            {
                throw new ArgumentNullException(nameof(words));
            }

            string? Text((int Offset, int Words) field) =>
                words.Count >= field.Offset + field.Words ? Registers.WordsToText(words.Skip(field.Offset).Take(field.Words)) : null;

            uint? accl = words.Count > Registers.IDENT_ACCL + 1 ? Registers.FromU32Words(words[Registers.IDENT_ACCL], words[Registers.IDENT_ACCL + 1]) : (uint?)null;
            return new IdentificationBlock(
                Text(Registers.IDENT_SERIAL), Text(Registers.IDENT_ACTUATOR_SERIAL), Text(Registers.IDENT_USER_NAME), Text(Registers.IDENT_MODEL), accl);
        }
    }

    /// <summary>
    /// What a unit says about itself. Fields are <c>null</c> when the unit did not answer them; every string is printable
    /// ASCII.
    /// </summary>
    /// <remarks>
    /// Identity comes from the Report Slave ID (FC11) or, when that is silent, CTCL (class 3 item 1 at 0x0300; the same
    /// fields as FC11 on every unit tested): system / unit code / firmware (<see cref="IdentitySource"/>). The system code
    /// gives the <see cref="ProductFamily"/> (EC, PCON, ACON/DCON, SCON, else "IAI controller"), the unit code the EC family
    /// where the driver has a name for it (<see cref="UnitName"/>; <see cref="UnitKnown"/> is for display only: every
    /// unit is commanded the same way, ranges come from the unit itself). The 0x0600 block gives the model string, the
    /// serials and <see cref="UserName"/> (ANA1-3, a name the user can edit in IA-OS, not a model field). The
    /// axis-limit fields (soft limits, MAXV / MAXA / MAXD, AKID, units, stored rows) are filled by
    /// <see cref="Axis.Identify"/>; <see cref="DeviceIdentity.Read"/> leaves them <c>null</c>.
    /// </remarks>
    public sealed record DeviceInfo
    {
        public int Slave { get; init; }

        public int? SystemCode { get; init; }

        public string? SystemName { get; init; }

        public int? RunStatus { get; init; }

        public int? UnitCode { get; init; }

        /// <summary>EC family of the unit code, or the "not an actuator" text (e.g. a conveyor control motor).</summary>
        public string? UnitName { get; init; }

        /// <summary>Firmware version as 4 hex digits.</summary>
        public string? Firmware { get; init; }

        /// <summary>The FC11 data bytes as received.</summary>
        public IReadOnlyList<byte>? SlaveIdRaw { get; init; }

        public IndexHeader? IndexHeader { get; init; }

        /// <summary>The directory records (empty when the index table was not readable).</summary>
        public IReadOnlyList<IndexCategory> Categories { get; init; } = Array.Empty<IndexCategory>();

        /// <summary>CTS1-4: controller serial number.</summary>
        public string? SerialNumber { get; init; }

        /// <summary>ANA1-3: the user-editable unit name (not a model field).</summary>
        public string? UserName { get; init; }

        /// <summary>AS01-AS20: the actuator model string (EC), e.g. "EC-R6S-150-1-PN".</summary>
        public string? ModelName { get; init; }

        /// <summary>(min, max) from the soft limits 0x0700 (axis class).</summary>
        public (double Min, double Max)? SoftLimitsMm { get; init; }

        /// <summary>MAXV, item 7.4 (axis class).</summary>
        public double? MaxSpeedMmS { get; init; }

        /// <summary>Parts the unit rejected, did not answer or answered with unexpected bytes (printable ASCII).</summary>
        public IReadOnlyList<string> Errors { get; init; } = Array.Empty<string>();

        /// <summary>"FC11" or "CTCL 0x0300" (FC11 not answered).</summary>
        public string? IdentitySource { get; init; }

        /// <summary>The driver has a family name for the (system, unit) code (<see cref="Registers.EC_FAMILIES"/>; display only).</summary>
        public bool? UnitKnown { get; init; }

        /// <summary>AKID item 7.33 bits 0-7 (axis class).</summary>
        public int? ActuatorKind { get; init; }

        public string? ActuatorKindName { get; init; }

        /// <summary>AXNO (0x8004, read-only): the axis number the unit believes it has (= slave address - 1).</summary>
        public int? AxisNumber { get; init; }

        /// <summary>ACS1-4: actuator serial number (UNVERIFIED).</summary>
        public string? ActuatorSerial { get; init; }

        /// <summary>ACCL 0x0612 (bits 8-11 = extensibility level).</summary>
        public uint? Accl { get; init; }

        /// <summary>MAXA item 7.5 (axis class).</summary>
        public double? MaxAccelG { get; init; }

        /// <summary>MAXD item 7.6 (axis class).</summary>
        public double? MaxDecelG { get; init; }

        public string? SpeedLimitSource { get; init; }

        /// <summary>Family of the system code: "EC", "PCON", "ACON/DCON", "SCON" or "IAI controller".</summary>
        public string? ProductFamily { get; init; }

        /// <summary>The CON controller's name block (0x8010, e.g. "PCON-CB (FB)"; the field split is UNVERIFIED).</summary>
        public string? ControllerName { get; init; }

        /// <summary>"mm", or "deg" on a rotary axis (AKID 1).</summary>
        public string Units { get; init; } = "mm";

        /// <summary>Stored rows the unit uses (MAXP 3.2; axis class).</summary>
        public int? MaxPositions { get; init; }

        /// <summary>The actuator kind in words ("slider", "rotary" ... "unknown").</summary>
        public string Kind => Registers.AxisKindName(ActuatorKind, RotaryAtyp);

        /// <summary>ATYP 7.7 bit 8: rotary on a unit without AKID (CON controllers; UNVERIFIED). Filled by the axis class.</summary>
        public bool? RotaryAtyp { get; init; }

        /// <summary>EC family name of the unit code (same as <see cref="UnitName"/>).</summary>
        public string? Family => UnitName;

        /// <summary>ACCL bits 8-11 (0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo).</summary>
        public int? ExtensibilityLevel => Accl.HasValue ? (int)((Accl.Value >> 8) & 0xF) : (int?)null;

        /// <summary>AKID kind 1: a rotary axis (positions in deg).</summary>
        public bool? Rotary => ActuatorKind.HasValue ? ActuatorKind.Value == Registers.AKID_ROTARY : RotaryAtyp;

        /// <summary>Report Slave ID in words, e.g. 'EC 24 V, unit 0x0A (EC(MS) "2-size EC")'.</summary>
        public string Controller
        {
            get
            {
                if (!SystemCode.HasValue)
                {
                    return "unknown";
                }

                var name = SystemName ?? "system code 0x" + SystemCode.Value.ToString("x2", CultureInfo.InvariantCulture);
                if (UnitCode.HasValue)
                {
                    name += ", unit 0x" + UnitCode.Value.ToString("X2", CultureInfo.InvariantCulture);
                    if (!string.IsNullOrEmpty(UnitName))
                    {
                        name += $" ({UnitName})";
                    }
                }

                return name;
            }
        }

        /// <summary>Actuator kind from AKID in words, e.g. "gripper (AKID 4)"; <c>null</c> if AKID was not read.</summary>
        public string? Actuator => ActuatorKind.HasValue ? $"{ActuatorKindName ?? "unknown kind"} (AKID {ActuatorKind.Value})" : null;

        /// <summary>
        /// Display name (printable ASCII): the 0x0600 model string if the unit has one (EC), else the controller name block
        /// (CON), else <see cref="Controller"/>.
        /// </summary>
        public string Model => Registers.Printable(ModelName ?? ControllerName ?? Controller);

        /// <summary>The directory record of <paramref name="categoryId"/>, if present.</summary>
        public IndexCategory? Category(int categoryId) => Categories.FirstOrDefault(c => c.CategoryId == categoryId);
    }

    /// <summary>
    /// Reads the identity part of <see cref="DeviceInfo"/>: FC11 (with the CTCL fallback), the directory, the
    /// identification block, the CON controller name and AXNO. READ-ONLY (FC03 + FC11). The reads and their rules are the
    /// native core's (read_device_info on the bus, iad_bus_read_device_info_json).
    /// </summary>
    public static class DeviceIdentity
    {
        /// <summary>
        /// Read what <paramref name="slave"/> tells about itself. FC11 is sent once and waited for
        /// <see cref="ModbusRtu.SlaveIdTimeout"/> (EC #3 answers after 0.2-0.4 s, VERIFIED); when it is silent the same
        /// fields come from CTCL. Parts the unit rejects, does not answer or answers with unexpected bytes are reported in
        /// <see cref="DeviceInfo.Errors"/> - this never throws on them (a closed bus or a transport failure still throws).
        /// The axis-limit fields stay <c>null</c> (<see cref="Axis.Identify"/> fills them).
        /// </summary>
        public static DeviceInfo Read(ModbusRtu bus, int slave)
        {
            if (bus is null)
            {
                throw new ArgumentNullException(nameof(bus));
            }

            if (slave < 1 || slave > 247)
            {
                throw new ArgumentOutOfRangeException(nameof(slave), slave, "slave address must be 1..247");
            }

            var info = AxisRecords.ToDeviceInfo(bus.OnPort(native => native.ReadDeviceInfo(slave)));
            return info with { Errors = info.Errors.Select(Registers.Printable).ToList().AsReadOnly() };
        }
    }
}
