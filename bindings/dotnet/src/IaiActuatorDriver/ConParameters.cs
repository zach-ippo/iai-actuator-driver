using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver
{
    /// <summary>
    /// One numbered parameter of a CON-family controller (PCON, ACON, DCON, SCON ...), as IAI's PC tool numbers it, and where
    /// it lives (class.item, found through the unit's own ID map, so the address may differ per model).
    /// </summary>
    /// <remarks>
    /// Encodings (<see cref="Kind"/>): "scaled" raw x <see cref="Scale"/> (<see cref="Signed"/> s32); "raw" the integer;
    /// "pct255" raw 0-255 = 0-100 %; "bits" the value is the bit field <see cref="Bits"/> (value bit j = item bit Bits[j],
    /// read-modify-write); "pair3" two bits as 0 / 1 / 2 (raw 3 -&gt; 2); "baud" an IAI baud code shown as bit/s; "parity"
    /// the home pattern number (odd = 1 Forward, even = 0 Reverse; a change writes the pattern -1 / +1); "ipv4" an address,
    /// first octet in the high byte; "flag" read-only here (non-zero = 1).
    /// </remarks>
    public sealed record ConParam(int No, string Name, string Symbol, int ClassId, int ItemId, string Kind)
    {
        public int Entry { get; init; }

        public double Scale { get; init; } = 1.0;

        public bool Signed { get; init; }

        public IReadOnlyList<int> Bits { get; init; } = Array.Empty<int>();

        public string Unit { get; init; } = string.Empty;

        public double? Lo { get; init; }

        public double? Hi { get; init; }

        public IReadOnlyDictionary<int, string> Choices { get; init; } = new Dictionary<int, string>();

        /// <summary>
        /// Written with a warning (home, motor / encoder, lead / axis type, gains, PIO / fieldbus / baud
        /// configuration).
        /// </summary>
        public bool Dangerous { get; init; }

        public bool Writable { get; init; } = true;

        /// <summary>"class.item" (class in hex above 9), "[entry]" for a second entry.</summary>
        public string Key
        {
            get
            {
                var cls = ClassId > 9 ? ClassId.ToString("X", CultureInfo.InvariantCulture) : ClassId.ToString(CultureInfo.InvariantCulture);
                return $"{cls}.{ItemId}" + (Entry != 0 ? $"[{Entry}]" : string.Empty);
            }
        }
    }

    /// <summary>
    /// The numbered CON parameters (no I/O): table, decoding, text, parsing and encoding. Read on a PCON-CB + RCP6 (VERIFIED:
    /// values and item locations); the encodings of items that were not changed on hardware are UNVERIFIED. A parameter is
    /// offered only when the unit's ID map has its item.
    /// </summary>
    public static class ConParameters
    {
        /// <summary>IAI baud code -&gt; bit/s (the SIO baud-rate parameter).</summary>
        public static readonly IReadOnlyDictionary<int, int> BAUD_CODES = new Dictionary<int, int>
        {
            [4] = 9600, [5] = 19200, [6] = 38400, [7] = 76800, [17] = 14400, [18] = 28800, [19] = 57600, [20] = 115200, [21] = 230400, [256] = 460800,
        };

        /// <summary>CON home direction (No.5 ORG) and home offset (No.22 OFST), two of the dangerous ones.</summary>
        public static readonly IReadOnlyCollection<int> HOME_PARAMETERS = new[] { 5, 22 };

        /// <summary>The table.</summary>
        public static readonly IReadOnlyList<ConParam> CON_PARAMETERS = new[]
        {
            new ConParam(1, "Zone Output Position(1) +", "ZNM", 12, 1, "scaled") { Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99 },
            new ConParam(2, "Zone Output Position(1) -", "ZNL", 12, 2, "scaled") { Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99 },
            new ConParam(3, "Soft limit +", "LIMM", 7, 1, "scaled") { Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99 },
            new ConParam(4, "Soft limit -", "LIML", 7, 2, "scaled") { Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99 },
            new ConParam(5, "Home direction", "ORG", 9, 16, "parity") { Lo = 0.0, Hi = 1.0, Choices = C((0, "Reverse"), (1, "Forward")), Dangerous = true },
            new ConParam(6, "Push recognition time", "PSWT", 9, 3, "raw") { Unit = "ms", Lo = 0.0, Hi = 9999.0 },
            new ConParam(7, "Servo gain selection", "PLG0", 9, 6, "raw") { Lo = 0.0, Hi = 31.0, Dangerous = true },
            new ConParam(8, "Default velocity", "VCMD", 8, 10, "scaled") { Scale = 0.01, Unit = "mm/s", Lo = 1.0, Hi = 9999.0 },
            new ConParam(9, "Default ACC", "ACMD", 8, 11, "scaled") { Scale = 0.01, Unit = "G", Lo = 0.01, Hi = 9.99 },
            new ConParam(10, "Default position band", "INP", 8, 9, "scaled") { Scale = 0.01, Unit = "mm", Lo = 0.0, Hi = 999.99 },
            new ConParam(12, "Default positioning current limit", "SPOW", 8, 12, "pct255") { Unit = "%", Lo = 0.0, Hi = 70.0 },
            new ConParam(13, "Default homing current limit", "ODPW", 9, 19, "pct255") { Unit = "%", Lo = 0.0, Hi = 100.0 },
            new ConParam(15, "STOP' Input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 7 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(16, "SIO Baudrate", "BRSL", 8, 3, "baud") { Unit = "bps", Lo = 0.0, Hi = 230400.0, Dangerous = true },
            new ConParam(17, "Min delay for activating local transmitter", "RTIM", 8, 4, "raw") { Unit = "ms", Lo = 0.0, Hi = 255.0 },
            new ConParam(18, "Home Input Polarity", "AIOF", 9, 27, "pair3") { Bits = new[] { 0, 1 }, Lo = 0.0, Hi = 2.0, Choices = C((0, "Not Used"), (1, "Normally open"), (2, "Normally closed")), Dangerous = true },
            new ConParam(21, "ServoON' Input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 6 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(22, "Home offset", "OFST", 9, 21, "scaled") { Scale = 0.01, Unit = "mm", Lo = 0.0, Hi = 9999.99, Dangerous = true },
            new ConParam(23, "Zone Output Position(2) +", "ZNM", 12, 1, "scaled") { Entry = 1, Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99 },
            new ConParam(24, "Zone Output Position(2) -", "ZNL", 12, 2, "scaled") { Entry = 1, Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99 },
            new ConParam(25, "PIO pattern", "IOPN", 8, 2, "raw") { Signed = true, Lo = -1.0, Hi = 32767.0 },
            new ConParam(26, "PIO Jog Velocity", "IOJV", 8, 16, "scaled") { Scale = 0.01, Unit = "mm/s", Lo = 1.0, Hi = 9999.0 },
            new ConParam(27, "Move command type", "FPIO", 8, 1, "bits") { Bits = new[] { 16 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Level"), (1, "Edge")) },
            new ConParam(28, "Pole sense initial moving direction", "PHSP", 9, 1, "bits") { Bits = new[] { 16 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Reverse"), (1, "Forward")), Dangerous = true },
            new ConParam(29, "Excitation phase signal detection time", "PHSP", 9, 1, "bits") { Bits = new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }, Unit = "ms", Lo = 1.0, Hi = 999.0, Dangerous = true },
            new ConParam(30, "Excitation phase signal detection type", "PHSP", 9, 1, "bits") { Bits = new[] { 17, 18 }, Lo = 0.0, Hi = 2.0, Choices = C((0, "Conventional method"), (1, "New method 1"), (2, "New method 2")), Dangerous = true },
            new ConParam(31, "Velocity loop proportional gain", "VLPG", 9, 7, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(32, "Velocity loop integral gain", "VLPT", 9, 8, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(33, "Torque filter time constant", "TRQF", 9, 9, "raw") { Lo = 0.0, Hi = 2500.0, Dangerous = true },
            new ConParam(34, "Push velocity", "PSHV", 8, 15, "scaled") { Scale = 0.01, Unit = "mm/s", Lo = 1.0, Hi = 9999.0 },
            new ConParam(35, "Safety velocity", "SAFV", 8, 21, "scaled") { Scale = 0.01, Unit = "mm/s", Lo = 1.0, Hi = 250.0 },
            new ConParam(36, "Automatic servo OFF delay time", "ASO1", 8, 20, "raw") { Unit = "s", Lo = 0.0, Hi = 9999.0 },
            new ConParam(37, "Automatic servo off delay time2", "ASO2", 8, 44, "raw") { Unit = "s", Lo = 0.0, Hi = 9999.0 },
            new ConParam(38, "Automatic servo off delay time3", "ASO3", 8, 45, "raw") { Unit = "s", Lo = 0.0, Hi = 9999.0 },
            new ConParam(39, "Positioning complete signal output method", "FPIO", 8, 1, "bits") { Bits = new[] { 0 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "PEND"), (1, "INP")) },
            new ConParam(40, "Home input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 8 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(41, "Operation mode input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 15 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(42, "Enable function", "FPIO", 8, 1, "bits") { Bits = new[] { 22 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(43, "Home confirmation sensor input polarity", "AIOF", 9, 27, "pair3") { Bits = new[] { 6, 7 }, Lo = 0.0, Hi = 2.0, Choices = C((0, "Not Used"), (1, "Normally open"), (2, "Normally closed")), Dangerous = true },
            new ConParam(45, "Silent interval magnification", "SIVM", 8, 19, "raw") { Lo = 0.0, Hi = 10.0 },
            new ConParam(46, "Velocity override", "OVRD", 8, 18, "pct255") { Unit = "%", Lo = 1.0, Hi = 100.0 },
            new ConParam(47, "PIO Jog Velocity2", "IOV2", 8, 22, "scaled") { Scale = 0.01, Unit = "mm/s", Lo = 1.0, Hi = 9999.0 },
            new ConParam(48, "PIO Inching Distance", "IOID", 8, 23, "scaled") { Scale = 0.01, Unit = "mm", Lo = 0.01, Hi = 1.0 },
            new ConParam(49, "PIO Inching Distance2", "IOD2", 8, 24, "scaled") { Scale = 0.01, Unit = "mm", Lo = 0.01, Hi = 1.0 },
            new ConParam(50, "Load output judgment time", "LDWT", 9, 25, "raw") { Unit = "ms", Lo = 0.0, Hi = 9999.0 },
            new ConParam(51, "Disable 'Range of torque authorization'", "FPIO", 8, 1, "bits") { Bits = new[] { 25 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")) },
            new ConParam(52, "Addition and subtraction velocity mode initial value", "CTLF", 8, 14, "bits") { Bits = new[] { 6, 7 }, Lo = 0.0, Hi = 2.0, Choices = C((0, "Trapezoid"), (1, "S-shaped motion"), (2, "first order lag filter")) },
            new ConParam(53, "Stop mode initial value", "CTLF", 8, 14, "bits") { Bits = new[] { 8, 9, 10 }, Lo = 0.0, Hi = 7.0 },
            new ConParam(55, "Position command primary filter time constant", "PLPF", 9, 39, "scaled") { Scale = 0.1, Unit = "ms", Lo = 0.0, Hi = 100.0 },
            new ConParam(56, "Sigmoid motion ratio setting", "SCRV", 9, 40, "raw") { Unit = "%", Lo = 0.0, Hi = 100.0 },
            new ConParam(57, "Torque limiting value", "TQLM", 9, 36, "pct255") { Unit = "%", Lo = 0.0, Hi = 70.0 },
            new ConParam(58, "Clear deviation at servo off and alarm happens", "FSTP", 9, 15, "bits") { Bits = new[] { 7 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Disable"), (1, "Enable")), Dangerous = true },
            new ConParam(59, "Deviation err monitor when the trq is being limited", "FSTP", 9, 15, "bits") { Bits = new[] { 9 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Disable"), (1, "Enable")), Dangerous = true },
            new ConParam(60, "Deviation counter clear input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 12 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(61, "Torque limitation instruction input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 13 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(62, "Pulse count direction", "FPIO", 8, 1, "bits") { Bits = new[] { 29 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Forward"), (1, "Reverse")), Dangerous = true },
            new ConParam(63, "Command pulse input mode", "CPMD", 9, 29, "bits") { Bits = new[] { 0, 1 }, Lo = 0.0, Hi = 2.0, Dangerous = true },
            new ConParam(64, "Command pulse input mode polarity", "CPMD", 9, 29, "bits") { Bits = new[] { 4 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Positive"), (1, "Negative")), Dangerous = true },
            new ConParam(65, "Electronic gear numerator", "CNUM", 9, 30, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(66, "Electronic gear denominator", "CDEN", 9, 31, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(67, "Compulsion stop input enable/disable selection", "FPIO", 8, 1, "bits") { Bits = new[] { 14 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Enable"), (1, "Disable")), Dangerous = true },
            new ConParam(71, "Positional feedforward gain", "PLFG", 9, 33, "raw") { Lo = 0.0, Hi = 100.0, Dangerous = true },
            new ConParam(77, "Screw lead", "LEAD", 7, 3, "scaled") { Scale = 0.01, Unit = "mm", Lo = 0.01, Hi = 999.99, Dangerous = true },
            new ConParam(78, "Axis action type", "ATYP", 7, 7, "bits") { Bits = new[] { 8 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Linear axis"), (1, "Rotary axis")), Dangerous = true },
            new ConParam(79, "Rotary axis mode", "ATYP", 7, 7, "bits") { Bits = new[] { 9 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Normal mode"), (1, "Index Mode")), Dangerous = true },
            new ConParam(80, "Rotary axis shortcut select", "ATYP", 7, 7, "bits") { Bits = new[] { 10 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Disable"), (1, "Enable")), Dangerous = true },
            new ConParam(83, "Absolute unit", "ETYP", 9, 11, "bits") { Bits = new[] { 3 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Not Used"), (1, "Use")), Dangerous = true },
            new ConParam(84, "Fieldbus operation mode", "FMOD", 8, 46, "raw") { Lo = 0.0, Hi = 9.0, Dangerous = true },
            new ConParam(85, "Fieldbus node address", "NADR", 8, 47, "raw") { Lo = 0.0, Hi = 65535.0, Dangerous = true },
            new ConParam(86, "Fieldbus communication speed", "FBRS", 8, 48, "raw") { Lo = 0.0, Hi = 4.0, Dangerous = true },
            new ConParam(87, "Network type", "NTYP", 8, 49, "raw") { Lo = 0.0, Hi = 12.0, Choices = C((0, "None"), (1, "CC-Link"), (2, "DeviceNet"), (3, "PROFIBUS-DP"), (4, "CompoNet"), (5, "MECHATROLINK"), (6, "EtherCAT"), (7, "EtherNet/IP"), (8, "PROFINET IO"), (9, "MECHATROLINK III SV"), (10, "MECHATROLINK III I/O"), (11, "CC-Link IE Field"), (12, "EtherCAT DriveProfile")), Dangerous = true },
            new ConParam(88, "Software limit margin", "SLMA", 7, 10, "scaled") { Scale = 0.01, Unit = "mm", Lo = 0.0, Hi = 9999.99 },
            new ConParam(90, "Fieldbus input/output format", "FPIO", 8, 1, "bits") { Bits = new[] { 26, 27 }, Lo = 0.0, Hi = 3.0, Choices = C((0, "Byte / word swap disabled"), (1, "Only byte swap"), (2, "Only for word swap"), (3, "Byte swap + word swap")), Dangerous = true },
            new ConParam(91, "Current limit value during stop after push failure", "FSTP", 9, 15, "bits") { Bits = new[] { 3 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Current limit value during motion"), (1, "Current limit value during pressing")) },
            new ConParam(110, "Stop method at SrvOff", "FSTP", 9, 15, "bits") { Bits = new[] { 19 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Sudden stop"), (1, "Deceleration stop")) },
            new ConParam(111, "Calendar Function", "FRTC", 8, 53, "bits") { Bits = new[] { 0 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Not Used"), (1, "Use")) },
            new ConParam(112, "Monitoring mode", "FMNT", 8, 54, "bits") { Bits = new[] { 0, 1, 2, 3 }, Lo = 0.0, Hi = 3.0, Choices = C((0, "Not Used"), (1, "4CH Record Mode"), (2, "8CH Record Mode"), (3, "2CH Record Mode")) },
            new ConParam(113, "Monitoring cycle", "FMNT", 8, 54, "bits") { Bits = new[] { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 }, Unit = "ms", Lo = 1.0, Hi = 60000.0 },
            new ConParam(140, "IP address", "IPAD", 8, 55, "ipv4") { Lo = 0.0, Hi = 4294967295.0, Dangerous = true },
            new ConParam(141, "Subnet mask", "SNMK", 8, 56, "ipv4") { Lo = 0.0, Hi = 4294967295.0, Dangerous = true },
            new ConParam(142, "Default gateway", "DFGW", 8, 57, "ipv4") { Lo = 0.0, Hi = 4294967295.0, Dangerous = true },
            new ConParam(143, "Overload warning load level ratio", "OLWL", 9, 102, "raw") { Unit = "%", Lo = 50.0, Hi = 100.0 },
            new ConParam(144, "Gain scheduling limit magnification", "GSUL", 9, 99, "raw") { Unit = "%", Lo = 0.0, Hi = 1023.0, Dangerous = true },
            new ConParam(145, "Gain scheduling velocity loop proportional gain", "GSPC", 9, 100, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(146, "Gain scheduling velocity loop integral gain", "GSIC", 9, 101, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(147, "Total moving count threshold", "TMCT", 8, 59, "raw") { Lo = 0.0, Hi = 999999999.0 },
            new ConParam(148, "Total moving distance threshold", "ODOT", 8, 60, "raw") { Unit = "m", Lo = 0.0, Hi = 999999999.0 },
            new ConParam(149, "Zone output switch", "FPIO", 8, 1, "bits") { Bits = new[] { 1 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "PZONE output select"), (1, "ZONE1 / ZONE2 output select")) },
            new ConParam(151, "Minor fault alarm output", "FSTP", 9, 15, "bits") { Bits = new[] { 24 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Output at overload warning"), (1, "Message level alarm output")) },
            new ConParam(152, "High output setting", "ATYP", 7, 7, "bits") { Bits = new[] { 18 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Disable"), (1, "Enable")), Dangerous = true },
            new ConParam(153, "Boost up velocity loop proportional gain", "BUPC", 9, 105, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(154, "Boost up velocity loop integral gain", "BUIC", 9, 106, "raw") { Lo = 1.0, Hi = 99999999.0, Dangerous = true },
            new ConParam(155, "Battery maintenance", "ATYP", 7, 7, "bits") { Bits = new[] { 30, 31 }, Lo = 0.0, Hi = 3.0, Choices = C((0, "20 days"), (1, "15 days"), (2, "10 days"), (3, "5 days")), Dangerous = true },
            new ConParam(156, "(Torque authorization)/(Minor failure) output switch", "SPIO", 8, 50, "bits") { Bits = new[] { 24 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Load judgment / torque level status signal output"), (1, "Message level alarm output")) },
            new ConParam(159, "FB Half Direct Value Mode velocity unit", "SPIO", 8, 50, "bits") { Bits = new[] { 25 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "1mm/s"), (1, "0.1mm/s")) },
            new ConParam(165, "Delay-time after exiting shutdown", "SDDT", 8, 66, "raw") { Unit = "ms", Lo = 0.0, Hi = 10000.0 },
            new ConParam(166, "Current limit expansion Function", "DCET", 9, 50, "flag") { Lo = 0.0, Hi = 1.0, Choices = C((0, "Disable"), (1, "Enable")), Dangerous = true, Writable = false },
            new ConParam(167, "Pulse train datum position", "RPOS", 7, 30, "scaled") { Scale = 0.01, Signed = true, Unit = "mm", Lo = -9999.99, Hi = 9999.99, Dangerous = true },
            new ConParam(168, "Collision detecting function", "SPIO", 8, 50, "bits") { Bits = new[] { 26, 27, 28 }, Lo = 0.0, Hi = 7.0 },
            new ConParam(182, "Automatic current-down", "FSTP", 9, 15, "bits") { Bits = new[] { 26 }, Lo = 0.0, Hi = 1.0, Choices = C((0, "Disable"), (1, "Enable")) },
            new ConParam(187, "Non-positioner mode command output completion judgement time", "CPMD", 9, 29, "bits") { Bits = new[] { 16, 17, 18, 19, 20, 21, 22, 23 }, Unit = "ms", Lo = 0.0, Hi = 255.0 },
            new ConParam(188, "Network Number", "NWNO", 8, 90, "raw") { Lo = 1.0, Hi = 239.0, Writable = false },
            new ConParam(226, "Motor control switching threshold level", "SDDM", 9, 51, "raw") { Unit = "%", Lo = 0.0, Hi = 99999999.0, Dangerous = true },
        };

        /// <summary>Number -&gt; parameter.</summary>
        public static readonly IReadOnlyDictionary<int, ConParam> BY_NO = CON_PARAMETERS.ToDictionary(p => p.No);

        /// <summary>The value of <paramref name="p"/> as the PC tool shows it (double for "scaled", long for numbers, string for "ipv4", <c>null</c> when not set).</summary>
        public static object? Decode(ConParam p, uint raw)
        {
            switch (p.Kind)
            {
                case "scaled":
                    return Math.Round((p.Signed ? unchecked((int)raw) : (double)raw) * p.Scale, Decimals(p.Scale));
                case "raw":
                    return p.Signed ? (long)unchecked((int)raw) : (long)raw;
                case "pct255":
                    return (long)Math.Round((raw & 0xFF) * 100 / 255.0, MidpointRounding.ToEven);
                case "bits":
                    return (long)Field(raw, p.Bits);
                case "pair3":
                    var pair = Field(raw, p.Bits.Take(2).ToList());
                    return pair == 3 ? 2L : pair == 1 ? 1L : 0L;
                case "baud":
                    return raw <= int.MaxValue && BAUD_CODES.TryGetValue((int)raw, out var rate) ? (long)rate : (object?)null;
                case "parity":
                    return raw == 0 || raw == 0xFFFFFFFF ? (object?)null : (long)(raw & 1);
                case "ipv4":
                    return string.Join(".", new[] { 24, 16, 8, 0 }.Select(s => ((raw >> s) & 0xFF).ToString(CultureInfo.InvariantCulture)));
                default:
                    return raw != 0 ? 1L : 0L; // flag
            }
        }

        /// <summary><paramref name="value"/> with its label or unit, e.g. "1:Forward" or "150.30 mm".</summary>
        public static string Text(ConParam p, object? value)
        {
            if (value is null)
            {
                return "not set";
            }

            if (value is long l && l >= int.MinValue && l <= int.MaxValue && p.Choices.TryGetValue((int)l, out var label))
            {
                return $"{l}:{label}";
            }

            var unit = p.Unit.Length > 0 ? " " + p.Unit : string.Empty;
            if (p.Kind == "scaled" && UserParameters.TryNumber(value, out var d))
            {
                return d.ToString("F" + Decimals(p.Scale).ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture) + unit;
            }

            var shown = value is IFormattable f ? f.ToString(null, CultureInfo.InvariantCulture) : value.ToString() ?? string.Empty;
            return shown + unit;
        }

        /// <summary>
        /// A value for <paramref name="p"/> (number, numeric string, a label such as "Forward" or "1:Forward", a dotted IPv4
        /// address for "ipv4"). Throws <see cref="ArgumentException"/> for anything else or a value outside Lo..Hi.
        /// </summary>
        public static object Parse(ConParam p, object? value)
        {
            if (value is bool)
            {
                throw new ArgumentException($"No.{p.No}: pass a number or a label, not {value}");
            }

            if (p.Kind == "ipv4")
            {
                if (value is string ip)
                {
                    var parts = ip.Trim().Split('.');
                    if (parts.Length != 4 || !parts.All(x => x.Length > 0 && x.Length <= 3 && x.All(char.IsDigit) && int.Parse(x, CultureInfo.InvariantCulture) <= 255))
                    {
                        throw new ArgumentException($"No.{p.No}: '{ip}' is not an IPv4 address");
                    }

                    return string.Join(".", parts.Select(x => int.Parse(x, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture)));
                }

                if (UserParameters.TryNumber(value, out var n) && n == Math.Floor(n) && n >= 0 && n <= uint.MaxValue)
                {
                    var v = (uint)n;
                    return string.Join(".", new[] { 24, 16, 8, 0 }.Select(s => ((v >> s) & 0xFF).ToString(CultureInfo.InvariantCulture)));
                }

                throw new ArgumentException($"No.{p.No}: {UserParameters.Show(value)} is not an IPv4 address");
            }

            double number;
            if (value is string text)
            {
                var t = text.Trim();
                var split = t.Split(new[] { ':' }, 2);
                var head = split[0].Trim();
                var label = split[split.Length - 1].Trim().ToLowerInvariant();
                var matches = p.Choices
                    .Where(kv => kv.Value.ToLowerInvariant() == label || (label.Length > 0 && kv.Value.ToLowerInvariant().StartsWith(label, StringComparison.Ordinal)))
                    .Select(kv => kv.Key).ToList();
                if (head.Length > 0 && double.TryParse(head, NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture, out var parsed))
                {
                    number = parsed;
                }
                else if (matches.Count == 1)
                {
                    number = matches[0];
                }
                else
                {
                    throw new ArgumentException($"No.{p.No}: '{text}' is not a number or one of {ChoicesText(p)}");
                }
            }
            else if (!UserParameters.TryNumber(value, out number))
            {
                throw new ArgumentException($"No.{p.No}: {UserParameters.Show(value)} is not a finite number");
            }

            if (double.IsNaN(number) || double.IsInfinity(number))
            {
                throw new ArgumentException($"No.{p.No}: {UserParameters.Show(value)} is not a finite number");
            }

            object result;
            if (p.Kind == "scaled")
            {
                number = Math.Round(number, Decimals(p.Scale));
                result = number;
            }
            else if (number != Math.Floor(number))
            {
                throw new ArgumentException($"No.{p.No}: {UserParameters.Show(value)} must be a whole number");
            }
            else
            {
                result = (long)number;
            }

            if (p.Kind == "baud")
            {
                if (!BAUD_CODES.Values.Any(r => r == number))
                {
                    throw new ArgumentException($"No.{p.No}: {number} bit/s has no IAI baud code ({string.Join(", ", BAUD_CODES.Values.OrderBy(r => r))})");
                }

                return result;
            }

            if (p.Choices.Count > 0 && (p.Kind == "bits" || p.Kind == "pair3" || p.Kind == "parity" || p.Kind == "raw") && !p.Choices.Keys.Any(k => k == number))
            {
                throw new ArgumentException($"No.{p.No}: {number} is not one of {ChoicesText(p)}");
            }

            if (p.Lo.HasValue && p.Hi.HasValue && !(p.Lo.Value - 1e-9 <= number && number <= p.Hi.Value + 1e-9))
            {
                throw new ArgumentException(
                    $"No.{p.No}: {number.ToString(CultureInfo.InvariantCulture)} outside {p.Lo.Value.ToString(CultureInfo.InvariantCulture)}..{p.Hi.Value.ToString(CultureInfo.InvariantCulture)}"
                    + (p.Unit.Length > 0 ? " " + p.Unit : string.Empty));
            }

            return result;
        }

        /// <summary>The new raw 32-bit item value for <paramref name="value"/> (from <see cref="Parse"/>; bits outside the parameter are kept).</summary>
        public static uint Encode(ConParam p, uint raw, object value)
        {
            switch (p.Kind)
            {
                case "scaled":
                {
                    UserParameters.TryNumber(value, out var d);
                    var counts = (long)Math.Round(d / p.Scale, MidpointRounding.ToEven);
                    if (counts < int.MinValue || counts > uint.MaxValue)
                    {
                        throw new ArgumentException($"No.{p.No}: {UserParameters.Show(value)} cannot be encoded");
                    }

                    return unchecked((uint)counts);
                }

                case "raw":
                {
                    UserParameters.TryNumber(value, out var r);
                    return unchecked((uint)(long)r);
                }

                case "pct255":
                {
                    UserParameters.TryNumber(value, out var pct);
                    return (raw & ~0xFFu) | (uint)((long)pct * 255 / 100);
                }

                case "bits":
                case "pair3":
                {
                    var bits = p.Kind == "pair3" ? p.Bits.Take(2).ToList() : p.Bits.ToList();
                    UserParameters.TryNumber(value, out var fv);
                    var field = p.Kind == "pair3" && (long)fv == 2 ? 3L : (long)fv;
                    if (field < 0 || (field >> bits.Count) != 0)
                    {
                        throw new ArgumentException($"No.{p.No}: {UserParameters.Show(value)} does not fit {bits.Count} bit(s)");
                    }

                    for (var j = 0; j < bits.Count; j++)
                    {
                        raw = (raw & ~(1u << bits[j])) | ((uint)((field >> j) & 1) << bits[j]);
                    }

                    return raw;
                }

                case "baud":
                {
                    UserParameters.TryNumber(value, out var rate);
                    return (uint)BAUD_CODES.First(kv => kv.Value == (int)rate).Key;
                }

                case "parity":
                {
                    if (raw == 0 || raw == 0xFFFFFFFF)
                    {
                        throw new ArgumentException($"No.{p.No}: the home pattern is 0x{raw:X8}: not settable");
                    }

                    UserParameters.TryNumber(value, out var dir);
                    if ((raw & 1) == (uint)dir)
                    {
                        return raw;
                    }

                    return (int)dir == 0 ? raw + 1 : raw - 1;
                }

                case "ipv4":
                {
                    var o = ((string)value).Split('.').Select(x => uint.Parse(x, CultureInfo.InvariantCulture)).ToArray();
                    return (o[0] << 24) | (o[1] << 16) | (o[2] << 8) | o[3];
                }

                default:
                    throw new ArgumentException($"No.{p.No} ({p.Kind}) is read-only");
            }
        }

        internal static int Decimals(double scale)
        {
            var text = scale.ToString("G", CultureInfo.InvariantCulture);
            var dot = text.IndexOf('.');
            return dot < 0 ? 0 : text.Length - dot - 1;
        }

        private static IReadOnlyDictionary<int, string> C(params (int Key, string Label)[] pairs) => pairs.ToDictionary(p => p.Key, p => p.Label);

        private static int Field(uint raw, IReadOnlyList<int> bits)
        {
            var value = 0;
            for (var j = 0; j < bits.Count; j++)
            {
                value |= (int)((raw >> bits[j]) & 1) << j;
            }

            return value;
        }

        private static string ChoicesText(ConParam p) =>
            p.Choices.Count == 0 ? "the values" : "{" + string.Join(", ", p.Choices.Select(kv => $"{kv.Key}: {kv.Value}")) + "}";
    }
}
