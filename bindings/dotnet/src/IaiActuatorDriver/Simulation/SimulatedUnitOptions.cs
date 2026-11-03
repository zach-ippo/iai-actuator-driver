using System;
using System.Collections.Generic;

namespace IaiActuatorDriver.Simulation
{
    /// <summary>The per-unit defaults of <see cref="SimulatedUnit"/>: one profile per unit tested, plus two simulator-only units.</summary>
    public enum SimProfile
    {
        /// <summary>EC-DS4H-150 slider in 3-position mode: 150 mm, 700.00 mm/s, FC11 unit 0x02 firmware 0x0013, 27-record directory, absolute (WA). The default.</summary>
        Slider,

        /// <summary>EC-GRBP8M-20 gripper in 2-position mode: 9.50 mm, 53.00 mm/s (MAXA / MAXD 0.30 G), FC11 unit 0x0A firmware 0x0002, 25-record directory, absolute (WA).</summary>
        Gripper,

        /// <summary>EC-R6S-150 rod (EC #3): 150 mm, 800.00 mm/s, FC11 unit 0x00, 22-record directory, incremental (loses HEND at a software reset), FC11 answered after 0.3 s.</summary>
        Rod,

        /// <summary>PCON-CB controller + RCP6-WSA10R slider (FC11 AE FF 44 00 00): 150 mm, 840.00 mm/s, 64 positions, message alarm 0x069 at power-up.</summary>
        Pcon,

        /// <summary>Simulator-only: an EC on the rod's blocks with AKID (item 7.33) = 1, a rotary axis in degrees (0..360) (a simulator profile).</summary>
        Rotary,

        /// <summary>Simulator-only: a unit whose codes have no name in the driver (FC11 system 0x99, unit 0x7E, AKID 42, blank model string) on the rod's blocks.</summary>
        Unknown,
    }

    /// <summary>Layout of the PIO output image DOPM (0x9004).</summary>
    public enum DopmLayout
    {
        /// <summary>3-position mode: b0 LS0, b1 LS1, b2 LS2, b3 *ALM (the slider).</summary>
        ThreePosition,

        /// <summary>2-position mode: b0 LS0, b1 LS1, b2 *ALM (the gripper, the rod).</summary>
        TwoPosition,
    }

    /// <summary>A fault <see cref="SimulatedUnit.InjectFault"/> applies to the next responses.</summary>
    public enum SimFault
    {
        /// <summary>No answer at all.</summary>
        Timeout,

        /// <summary>The last CRC byte is flipped.</summary>
        Crc,

        /// <summary>Only the first half of the frame arrives.</summary>
        Partial,

        /// <summary>The answer comes from another slave address.</summary>
        WrongSlave,
    }

    /// <summary>
    /// Settings of a <see cref="SimulatedUnit"/>. Every <c>null</c> takes the value of <see cref="Profile"/>; the others are the
    /// simulator's own defaults (chosen to match the hardware tests, see <see cref="SimulatedUnit"/>).
    /// </summary>
    public sealed record SimulatedUnitOptions
    {
        /// <summary>Slave address the unit answers to (axis 0 = 1).</summary>
        public int Slave { get; init; } = 1;

        /// <summary>Per-unit defaults (stroke, start position, speed, FC11 answer, DOPM layout, directory, model string, rows).</summary>
        public SimProfile Profile { get; init; } = SimProfile.Slider;

        /// <summary>Stroke; the soft limits are -0.30 .. stroke + 0.30 (alarm 0x0D9 outside).</summary>
        public double? StrokeMm { get; init; }

        /// <summary>Initial position.</summary>
        public double? PositionMm { get; init; }

        /// <summary>Simulated seconds per real second (e.g. 20 makes tests fast).</summary>
        public double TimeScale { get; init; } = 1.0;

        /// <summary>Monotonic clock in seconds (inject a fake for deterministic tests); <c>null</c>: the system's.</summary>
        public Func<double>? Clock { get; init; }

        /// <summary>A PC-tool backup (<c>AAAA,VVVV</c> lines) to read the directory from; <c>null</c>: the profile's embedded copy.</summary>
        public string? BkecPath { get; init; }

        /// <summary>The maximum speed served as MAXV (item 7.4) and as the default speed 0x0812 (equal on every unit tested); VCMD above it raises an alarm.</summary>
        public double? MaxSpeedMmS { get; init; }

        /// <summary>ACMD above this raises an alarm.</summary>
        public double MaxAccelG { get; init; } = 1.0;

        /// <summary>FC11 system code (EC 0xEC; PCON 0xAE).</summary>
        public int? SystemCode { get; init; }

        /// <summary>FC11 unit code.</summary>
        public int? UnitCode { get; init; }

        /// <summary>FC11 firmware bytes (high, low).</summary>
        public (int High, int Low)? Firmware { get; init; }

        /// <summary>DOPM layout.</summary>
        public DopmLayout? Dopm { get; init; }

        /// <summary>Initial Modbus-control state (PMSL / DSSE PMSS). The hardware boots with 0.</summary>
        public bool Pmsl { get; init; } = true;

        /// <summary>Update word +0xF of a position row after a table write (an assumption: unknown whether the EC does this).</summary>
        public bool RecomputePositionChecksum { get; init; } = true;

        /// <summary>Do not answer the first request after a move command that arrives while moving (seen once on hardware).</summary>
        public bool DropReplyAfterRetarget { get; init; }

        /// <summary>STOP stays ON until written OFF, blocking new moves (worst case; UNVERIFIED). <c>false</c>: the controller resets STOP itself (DOC, CON).</summary>
        public bool StopLatches { get; init; } = true;

        /// <summary>DSSE MOVE goes 0 during the final deceleration (seen live).</summary>
        public bool MoveBitClearsInDecel { get; init; } = true;

        /// <summary>Initial SFTY state (DSS1 / DRG1 bit 14). The live start value was 0.</summary>
        public bool SafetySpeed { get; init; }

        /// <summary>Serve the soft-limit block 0x0700 (<c>false</c>: exception 02 there).</summary>
        public bool SoftLimits { get; init; } = true;

        /// <summary>Half-width of the ZONS / DOPM LS ranges around each row (EC parameter 2).</summary>
        public double LsRangeMm { get; init; } = Registers.LS_RANGE_MM;

        /// <summary>PEND / MEND rise inside the INP band while still moving (seen live). <c>false</c>: only at standstill.</summary>
        public bool EarlyInPosition { get; init; } = true;

        /// <summary>The last <see cref="SettleTailMm"/> of a normal move is covered at no more than <see cref="SettleSpeedMmS"/> (0 disables the tail).</summary>
        public double SettleTailMm { get; init; } = 0.05;

        /// <summary>Speed of the settling tail.</summary>
        public double SettleSpeedMmS { get; init; } = 0.5;

        /// <summary>During a push stroke, show the contact fingerprint between this and + <see cref="PushBlipLenMm"/> (hardware test G5), then carry on.</summary>
        public double? PushBlipMm { get; init; }

        /// <summary>Length of the push blip window.</summary>
        public double PushBlipLenMm { get; init; } = 0.10;

        /// <summary>Absolute encoder (keeps HEND at a software reset) or incremental (loses it). Default: the profile's.</summary>
        public bool? Absolute { get; init; }

        /// <summary>Model the serial line at this PC-side rate (<c>null</c>: no baud rate at all).</summary>
        public int? BaudRate { get; init; }

        /// <summary>The unit's own rate (default = <see cref="BaudRate"/>).</summary>
        public int? EcBaudRate { get; init; }

        /// <summary>The unit's rate after <see cref="SimulatedUnit.PowerCycle"/> (38400, VERIFIED).</summary>
        public int BootBaudRate { get; init; } = 38400;

        /// <summary>The unit's rate after a software reset (<c>null</c>: unchanged; the hardware's is unknown).</summary>
        public int? ResetBaudRate { get; init; }

        /// <summary>Simulated seconds the unit is silent after a software reset.</summary>
        public double ResetS { get; init; } = 1.0;

        /// <summary>Real seconds before an FC11 answer can be read (default: the profile's; 0.3 on the rod).</summary>
        public double? SlaveIdDelayS { get; init; }

        /// <summary>Real seconds DRGE b7 reads 1 after the PARW coil before the commit completes (ON-WIRE: set at +55 ms, clear at +235 ms). 0 = at once.</summary>
        public double ParwBusyS { get; init; } = 0.1;

        /// <summary>DSSE b7 reads 0 while the commit runs (seen on a PCON; off by default).</summary>
        public bool ParwDsseBusy { get; init; }

        /// <summary>Baud-change codes the unit ignores (default 0x06, as EC #3 did).</summary>
        public IReadOnlyCollection<int> IgnoredBaudCodes { get; init; } = new[] { 0x06 };

        /// <summary>Serve MVPT (0x0103) bit 13 CDAS = 1 (the collision-detection support flag; none of the units tested sets it).</summary>
        public bool CollisionDetection { get; init; }
    }
}
