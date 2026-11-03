// Register map of the IAI units this driver speaks to (ELECYLINDER and the CON family), ported from the Python
// edition's registers.py (C# Registers.cs / Registers.Tables.cs). Constants and word helpers are header-only
// (constexpr / inline: the bus, the simulator and the axis all use them); the lookup functions at the end are
// implemented by the model unit (src/registers.cpp).
//
// Evidence words: VERIFIED (read or driven on hardware with the Python edition), ON-WIRE (seen in serial traffic of
// IA-OS), DOC (public IAI manuals, mostly ME0162 for the CON family), UNVERIFIED. Addresses of the class blocks differ
// per model: the driver finds them through the unit's own directory (index table at 0x0000), the constants below are
// the addresses seen on the units tested.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace iai_actuator_driver {
namespace reg {

// ------------------------------------------------------------------------------------------ scales
inline constexpr double kPositionScale = 0.01;  ///< units (mm / deg) per count: PNOW, PCMD, INP, zones (VERIFIED)
inline constexpr double kVelocityScale = 0.01;  ///< units/s per count: VCMD (VERIFIED); VNOW too, but coarse
inline constexpr double kAccelScale = 0.01;     ///< G per count: ACMD, DCMD (VERIFIED readback)
inline constexpr double kGmms2 = 9800.0;        ///< 1 G in mm/s^2 as IAI defines it
inline constexpr int kAcmdMaxRaw = 300;         ///< 3.00 G, upper limit of ACMD (DOC ME0162-12A p.5-132)

// ------------------------------------------------------------------------------------------ directory (index table)
inline constexpr int kIndexHeader = 0x0000;        ///< MODL, DQNT, LQNT, BFRD, BFTD, RSV x3 (VERIFIED)
inline constexpr int kIndexHeaderWords = 8;
inline constexpr int kIndexRecords = 0x0008;       ///< LQNT records of 8 words: CI MD DQ LQ DA IA BA MS (VERIFIED)
inline constexpr int kSupportCategory = 0x0001;    ///< DA 0x0100: SUPT, DIAG, MVPT, SPTS, CTYP, SUP2, ... (fixed order)
inline constexpr int kStatusImageCategory = 0x0002;
inline constexpr int kControllerMonitorCategory = 0x0003;  ///< 3.1 CTCL, 3.2 MAXP, 3.10 present alarm, 3.52 SRWT...
inline constexpr int kMonitor2Category = 0x0004;
inline constexpr int kAlarmHistoryCategory = 0x0005;    ///< DA 0x0500
inline constexpr int kIdentificationCategory = 0x0006;  ///< DA 0x0600
inline constexpr int kLimitParamsCategory = 0x0007;     ///< DA 0x0700 (soft limits first)
inline constexpr int kMotionParamsCategory = 0x0008;    ///< DA 0x0800
inline constexpr int kFactoryParamsCategory = 0x0009;   ///< DA 0x0900 (user parameters P3 / P4 live here)
inline constexpr int kZonesCategory = 0x000C;           ///< DA 0x0C00
inline constexpr int kIoControlCategory = 0x000D;       ///< DA 0x0D00, coil base 0x0400
inline constexpr int kPositionTableCategory = 0x0010;   ///< DA 0x1000
inline constexpr int kSystemCategory = 0x0100;          ///< DA 0x8000: FLGS, CKID, AXNO, ...
inline constexpr int kMonitorCategory = 0x0101;         ///< DA 0x9000
inline constexpr int kCommand1Category = 0x0102;        ///< DA 0x9800
inline constexpr int kCommand2Category = 0x0103;        ///< DA 0x9900
inline constexpr int kMaintenanceCategory = 0x0200;     ///< DA 0x8400
inline constexpr int kExpectedCoilBase = 0x0400;
/// Classes whose parameters are written to RAM and committed with PARW.
inline constexpr int kWritableParameterClasses[] = {0x0007, 0x0008, 0x0009, 0x000C};

// ------------------------------------------------------------------------------------------ monitor block 0x9000
inline constexpr int kMonitorBase = 0x9000;
inline constexpr int kPnow = 0x9000;  ///< int32 position, 0.01 mm (VERIFIED)
inline constexpr int kAlmc = 0x9002;  ///< present alarm code, 0 = none (VERIFIED)
inline constexpr int kDipm = 0x9003;
inline constexpr int kDopm = 0x9004;  ///< PIO output image (layout differs per PIO pattern; not used for decisions)
inline constexpr int kDss1 = 0x9005;  ///< device status 1 (VERIFIED values / DOC bits)
inline constexpr int kDss2 = 0x9006;
inline constexpr int kDsse = 0x9007;  ///< expansion device status
inline constexpr int kStat = 0x9008;  ///< uint32 system status (DOC)
inline constexpr int kVnow = 0x900A;  ///< int32 speed, coarse on the EC (~12.5 mm/s steps)
inline constexpr int kCnow = 0x900C;  ///< int32 motor current (DOC, mA)
inline constexpr int kDevi = 0x900E;  ///< int32 deviation, pulses
inline constexpr int kStim = 0x9010;  ///< uint32 ms since power on (VERIFIED; a software reset does not restart it)
inline constexpr int kSipm = 0x9012;
inline constexpr int kZons = 0x9013;  ///< zone / position-marker status (VERIFIED LS bits)
inline constexpr int kPoss = 0x9014;  ///< completed position number b0-b9 (DOC)
inline constexpr int kSsse = 0x9015;  ///< system status expansion (DOC)
inline constexpr int kStatusReadWords = 22;  ///< 0x9000..0x9015 in one read (the default)
inline constexpr int kStatusBlockWords = 20; ///< first fallback when 22 words are refused
inline constexpr int kStatusShortWords = 14; ///< ..CNOW
inline constexpr int kStatusMinWords = 8;    ///< last fallback 0x9000..0x9007 (all VERIFIED readable)

// DSS1
inline constexpr std::uint16_t kDss1Emgs = 1u << 15;  ///< emergency stop active
inline constexpr std::uint16_t kDss1Sfty = 1u << 14;  ///< safety speed enabled (VERIFIED value)
inline constexpr std::uint16_t kDss1Pwr = 1u << 13;   ///< controller ready
inline constexpr std::uint16_t kDss1Sv = 1u << 12;    ///< servo on (VERIFIED)
inline constexpr std::uint16_t kDss1Psfl = 1u << 11;  ///< push missed (VERIFIED)
inline constexpr std::uint16_t kDss1Almh = 1u << 10;  ///< major alarm (operation-cancel / cold-start level, VERIFIED)
inline constexpr std::uint16_t kDss1Alml = 1u << 9;   ///< minor (message-level) alarm / warning (UNVERIFIED)
inline constexpr std::uint16_t kDss1Aber = 1u << 8;   ///< absolute error
inline constexpr std::uint16_t kDss1Bkrl = 1u << 7;   ///< brake released (1 while the servo is on)
inline constexpr std::uint16_t kDss1Stp = 1u << 5;    ///< pause active (VERIFIED)
inline constexpr std::uint16_t kDss1Hend = 1u << 4;   ///< home return complete (VERIFIED)
inline constexpr std::uint16_t kDss1Pend = 1u << 3;   ///< inside the INP band (also after STOP / contact; can rise early)
inline constexpr std::uint16_t kDss1TargetReached = 1u << 0;  ///< EC MEND: last move ended at its target (0 after STOP)
// DSS2
inline constexpr std::uint16_t kDss2Enbs = 1u << 15;  ///< teaching-tool enable switch: 0 = released, servo held off
inline constexpr std::uint16_t kDss2Mods = 1u << 11;  ///< teaching mode
inline constexpr std::uint16_t kDss2Pwrt = 1u << 10;  ///< position data write (teach) complete
// DSSE
inline constexpr std::uint16_t kDsseEmgp = 1u << 15;  ///< emergency stop input (DOC)
inline constexpr std::uint16_t kDsseMpuv = 1u << 14;  ///< motor drive power cut / low (DOC)
inline constexpr std::uint16_t kDsseRmds = 1u << 13;  ///< 1 = MANU, 0 = AUTO (PIO has control)
inline constexpr std::uint16_t kDsseGhms = 1u << 11;  ///< home return in progress (VERIFIED, also during the creep)
inline constexpr std::uint16_t kDssePush = 1u << 10;  ///< push in progress / holding after contact (VERIFIED)
inline constexpr std::uint16_t kDssePsns = 1u << 9;   ///< excitation detection complete
inline constexpr std::uint16_t kDssePmss = 1u << 8;   ///< 1 = PIO disabled, Modbus has control (VERIFIED)
inline constexpr std::uint16_t kDsseParw = 1u << 7;   ///< parameter write idle / complete (1 = idle)
inline constexpr std::uint16_t kDsseMove = 1u << 5;   ///< moving (VERIFIED, but clears while still decelerating)
inline constexpr std::uint16_t kDssePushContact = 1u << 0;  ///< EC: holding after push contact (VERIFIED)
// STAT (uint32)
inline constexpr std::uint32_t kStatMpow = 1u << 0;   ///< motor drive power on
inline constexpr std::uint32_t kStatAeep = 1u << 16;  ///< non-volatile write in progress
inline constexpr std::uint32_t kStatAsof = 1u << 17;  ///< automatic servo OFF in effect
inline constexpr std::uint32_t kStatBatl = 1u << 31;  ///< absolute battery low (SCON)
// ZONS / POSS / SSSE
inline constexpr std::uint16_t kZonsLs0 = 1u << 12;   ///< near row 0, the backward end (VERIFIED)
inline constexpr std::uint16_t kZonsLs1 = 1u << 13;   ///< near row 1, the forward end (VERIFIED)
inline constexpr std::uint16_t kZonsLs2 = 1u << 14;   ///< near row 2, the middle (VERIFIED)
inline constexpr std::uint16_t kPossMask = 0x03FF;
inline constexpr std::uint16_t kSsseAlmc = 1u << 11;  ///< a cold-start-level alarm is present
inline constexpr std::uint16_t kSsseRtc = 1u << 8;    ///< real-time clock in use
inline constexpr double kLsRangeMm = 0.10;            ///< EC parameter 2 "LS signal detection range" on both units

// ------------------------------------------------------------------------------------------ control words 0x0D00
inline constexpr int kDrg1 = 0x0D00;  ///< device control 1 (VERIFIED readback 0x1000 / 0x5000)
inline constexpr int kDrg2 = 0x0D01;
inline constexpr int kDrge = 0x0D02;  ///< expansion control (b8 PMSL; b7 PARW commit busy)
inline constexpr int kPosr = 0x0D03;  ///< position number for CSTR / TEAC
inline constexpr std::uint16_t kDrgeParw = 1u << 7;
// DRG1 bits (DOC ME0162-12A p.5-124 to 5-128; RCON ME0413 p.5-11). The "register" command strategy writes DRG1 with
// FC06 (a read-modify-write that keeps SON / SFTY / STP); EMG and BKRL are never set by this driver.
inline constexpr std::uint16_t kDrg1Emg = 1u << 15;   ///< emergency stop: never set
inline constexpr std::uint16_t kDrg1Sfty = 1u << 14;  ///< safety speed (level)
inline constexpr std::uint16_t kDrg1Son = 1u << 12;   ///< servo on (level)
inline constexpr std::uint16_t kDrg1Alrs = 1u << 8;   ///< alarm reset (edge); with STP held it drops the remaining travel
inline constexpr std::uint16_t kDrg1Bkrl = 1u << 7;   ///< brake release: never set
inline constexpr std::uint16_t kDrg1Stp = 1u << 5;    ///< pause (level)
inline constexpr std::uint16_t kDrg1Home = 1u << 4;   ///< home return (edge)
inline constexpr std::uint16_t kDrg1Cstr = 1u << 3;   ///< CON only ("-" on RCON, DOC ME0413 p.5-11)
inline constexpr std::uint16_t kDrg1Keep = kDrg1Son | kDrg1Sfty | kDrg1Stp;  ///< levels every DRG1 write carries over
/// POSS b0-b6: 7 bits on RCON driver units (DOC ME0413 p.5-10 to 5-23; 10 bits elsewhere: kPossMask).
inline constexpr std::uint16_t kPossMaskRcon = 0x007F;

// ------------------------------------------------------------------------------------------ coils (FC05)
inline constexpr int kCoilSfty = 0x0401;   ///< safety speed (level; volatile; VERIFIED)
inline constexpr int kCoilSon = 0x0403;    ///< servo on (level; VERIFIED)
inline constexpr int kCoilAlrs = 0x0407;   ///< alarm reset (rising edge; VERIFIED)
inline constexpr int kCoilStp = 0x040A;    ///< pause (level; VERIFIED)
inline constexpr int kCoilHome = 0x040B;   ///< home return (edge; release after HEND; VERIFIED)
inline constexpr int kCoilCstr = 0x040C;   ///< start move to POSR (DOC; not used: 0x9800 is used instead)
inline constexpr int kCoilMod = 0x0414;    ///< teaching mode (experimental teach path only)
inline constexpr int kCoilTeac = 0x0415;   ///< teach current position into row POSR (experimental)
inline constexpr int kCoilPmsl = 0x0427;   ///< FF00 = Modbus control (VERIFIED)
inline constexpr int kCoilParw = 0x0428;   ///< commit RAM parameters to non-volatile memory (ON-WIRE; self-clearing)
inline constexpr int kCoilStop = 0x042C;   ///< decel stop: FF00, hold until standstill, 0000 (VERIFIED)
inline constexpr int kCoilJogPlus = 0x0416;   ///< NEVER USED: runs at ~100 mm/s from a parameter (VERIFIED)
inline constexpr int kCoilJogMinus = 0x0417;  ///< NEVER USED
inline constexpr int kCoilSfrs = 0x0420;   ///< NEVER USED: with the servo on it raised alarm 0x090

// ------------------------------------------------------------------------------------------ command blocks
inline constexpr int kCommand1Base = 0x9800;  ///< POSR: FC06 = immediate move to that row (VERIFIED)
inline constexpr int kDirectMoveBase = 0x9900;  ///< FC10 starts / re-targets a move (VERIFIED)
inline constexpr int kPcmd = 0x9900;
inline constexpr int kInp = 0x9902;
inline constexpr int kVcmd = 0x9904;
inline constexpr int kAcmd = 0x9906;
inline constexpr int kPpow = 0x9907;
inline constexpr int kCtlf = 0x9908;
inline constexpr int kDirectMoveWords = 9;  ///< never write 0x9909 (a DRG1 mirror)
inline constexpr std::uint16_t kCtlfAbsolute = 0x0000;
inline constexpr std::uint16_t kCtlfPush = 1u << 1;
inline constexpr std::uint16_t kCtlfPushAway = 1u << 2;
inline constexpr std::uint16_t kCtlfInc = 1u << 3;  ///< incremental (never used by the driver)

// ------------------------------------------------------------------------------------------ push power
inline constexpr int kPpowMin = 0x33;     ///< 20 %
inline constexpr int kPpowMax = 0xB2;     ///< 70 %
inline constexpr int kPpowRawMax = 0x1FE; ///< 200 % (large RCS units)
inline constexpr double kMnppDefaultPct = 20.0;  ///< push recommendation when MNPP (7.58) is absent

// ------------------------------------------------------------------------------------------ position table
inline constexpr int kPositionTableBase = 0x1000;  ///< VERIFIED rows 0..2
inline constexpr int kPositionStride = 16;          ///< words per row (VERIFIED)
inline constexpr int kPositionWordsWritable = 15;   ///< +0x0..+0xE; +0xF is the checksum
inline constexpr int kPositionMaxEntries = 0x300;   ///< LQ of category 0x10 (768 on an EC)
inline constexpr int kPositionActiveRows = 3;       ///< an EC uses rows 0 (backward end), 1 (forward end), 2 (middle)
inline constexpr int kSimpleDataFirstRow = 256;     ///< rows 256-267: an EC's internal simple-data store, never written
inline constexpr int kSimpleDataLastRow = 267;
inline constexpr std::uint16_t kPositionCtlfKnown = 0x003E;   ///< PUSH, DIR, INC and the 0x0010 / 0x0020 role bits
inline constexpr std::uint16_t kPositionCtlfRoleBits = 0x0030; ///< 0x0010 row 1, 0x0020 row 2 (meaning UNVERIFIED)
// word offsets inside a row
inline constexpr int kPosPcmd = 0x0;   ///< int32, 0.01 mm
inline constexpr int kPosInp = 0x2;    ///< uint32, 0.01 mm
inline constexpr int kPosVcmd = 0x4;   ///< uint32, 0.01 mm/s
inline constexpr int kPosZnmp = 0x6;   ///< int32
inline constexpr int kPosZnlp = 0x8;   ///< int32
inline constexpr int kPosAcmd = 0xA;   ///< uint16, 0.01 G
inline constexpr int kPosDcmd = 0xB;   ///< uint16, 0.01 G
inline constexpr int kPosPpow = 0xC;
inline constexpr int kPosLpow = 0xD;
inline constexpr int kPosCtlf = 0xE;
inline constexpr int kPosChecksum = 0xF;  ///< low byte makes the byte sum of the 32-byte row 0 mod 256 (VERIFIED)

// ------------------------------------------------------------------------------------------ info blocks
inline constexpr int kAlarmHistoryBase = 0x0500;   ///< 32 entries x 6 words {detail, address, 0, code, time hi, lo}
inline constexpr int kAlarmHistoryStride = 6;
inline constexpr int kAlarmHistoryEntries = 32;    ///< index 0 is the newest (VERIFIED)
inline constexpr int kAlarmPowerUp = 0x0FFF;       ///< "PowerUP No Error" marker written at every power-up (VERIFIED)
inline constexpr int kIdentificationBase = 0x0600; ///< 72 words (VERIFIED readable)
inline constexpr int kIdentificationWords = 0x48;
inline constexpr int kIdentSerialOffset = 0x00;    ///< 8 words: controller serial number (VERIFIED)
inline constexpr int kIdentActuatorSerialOffset = 0x08;  ///< 8 words (UNVERIFIED)
inline constexpr int kIdentAcclOffset = 0x12;      ///< u32 ACCL: bits 8-11 extensibility level
inline constexpr int kIdentUserNameOffset = 0x1A;  ///< 6 words: the user-editable unit name (not a model field)
inline constexpr int kIdentModelOffset = 0x20;     ///< 0x28 words: actuator model string, e.g. "EC-R6S-150-1-PN"
inline constexpr int kIdentModelWords = 0x28;
inline constexpr int kSoftLimits = 0x0700;         ///< int32 +max, int32 -min, 0.01 mm (class 7 items 1-2)
inline constexpr double kSoftLimitMarginMm = 0.30; ///< the EC stores stroke + 0.30 mm at each end
inline constexpr int kMotionParamsBase = 0x0800;
inline constexpr int kAxnoOffset = 4;              ///< AXNO u32 at 0x8004: axis number = address - 1 (read-only)
inline constexpr int kControllerNameOffset = 0x10; ///< 0x8010.. on the PCON: "PCON    " "CB  " ...
inline constexpr int kControllerNameWords = 8;
inline constexpr int kAlarmRtcStop = 0x069;        ///< message level: RTC oscillation stop (PCON at power-up)

// class 7 item IDs (limits)
inline constexpr int kAxisItemLimm = 1;   ///< + soft limit (P1 + 0.30 mm)
inline constexpr int kAxisItemLiml = 2;   ///< - soft limit
inline constexpr int kAxisItemLead = 3;   ///< screw lead, 0.01 mm
inline constexpr int kAxisItemMaxv = 4;   ///< maximum speed, 0.01 mm/s (VERIFIED)
inline constexpr int kAxisItemMaxa = 5;   ///< maximum acceleration, 0.01 G (VERIFIED)
inline constexpr int kAxisItemMaxd = 6;   ///< maximum deceleration, 0.01 G
inline constexpr int kAxisItemAtyp = 7;   ///< axis type bits (TYPE b8 rotary, BUEN b18 = P8, RINP b24)
inline constexpr int kAxisItemMxph = 8;   ///< maximum push speed, 0.01 mm/s
inline constexpr int kAxisItemMxpp = 9;   ///< maximum push current, % (the hard cap; 0 = no push)
inline constexpr int kAxisItemAkid = 33;  ///< actuator kind, bits 0-7
inline constexpr int kAxisItemTcpc = 34;  ///< push force conversion factor (0 = no push)
inline constexpr int kAxisItemTcoa = 35;  ///< push force conversion offset
inline constexpr int kAxisItemMnpp = 58;  ///< recommended minimum push, %
inline constexpr std::uint32_t kAtypRotary = 1u << 8;
inline constexpr std::uint32_t kAtypBuen = 1u << 18;
inline constexpr std::uint32_t kAtypRinp = 1u << 24;
// class 8 item IDs
inline constexpr int kParamItemBrsl = 3;   ///< 8.3 power-up baud code (6 = 38400 on all units tested)
inline constexpr int kParamItemRtim = 4;   ///< 8.4 response delay, ms (5)
inline constexpr int kParamItemVcmd = 10;  ///< 8.10 default command speed
inline constexpr int kParamItemCtlf = 14;  ///< 8.14 control flags (P5 b6, P6 b10)
inline constexpr int kParamItemPshv = 15;  ///< 8.15 push speed
inline constexpr int kParamItemSpio = 50;  ///< 8.50 (P7 b23, P10 b18, ...)
// class 9 / 3 item IDs
inline constexpr int kDriverItemOrg = 16;          ///< 9.16 home pattern (P3 = its parity; 0x45 home op1)
inline constexpr int kControllerItemCtcl = 1;      ///< 3.1 system / unit / firmware (as FC11)
inline constexpr int kControllerItemMaxp = 2;      ///< 3.2 maximum position count
inline constexpr int kControllerItemMxpt = 16;     ///< 3.16 highest allowed P9 value
inline constexpr int kControllerItemSrwt = 52;     ///< 3.52 software reset wait, ms
inline constexpr int kControllerItemMini = 67;     ///< 3.67 minimum positioning band
// class 1 support bits (UNVERIFIED meanings)
inline constexpr std::uint32_t kSuptSrst = 1u << 2;   ///< software reset supported
inline constexpr std::uint32_t kSuptIncm = 1u << 8;   ///< relative moves
inline constexpr std::uint32_t kSuptPush = 1u << 11;  ///< push motion
inline constexpr std::uint32_t kSup2Cinp = 1u << 24;  ///< minimum positioning band input limit

// ------------------------------------------------------------------------------------------ IAI 0x45 commands
inline constexpr int kIaiCmdBaud = 0x04;          ///< broadcast, op1 = baud code, op2 = 5 (ON-WIRE); never-send list
inline constexpr int kIaiBaudOp2 = 0x0005;
inline constexpr int kIaiCmdReset = 0x05;         ///< op1 0 = alarm reset, 1 = software reset
inline constexpr int kIaiResetAlarm = 0x0000;     ///< 01 45 00 05 00 00 00 00 (VERIFIED on the EC-R6S rod)
inline constexpr int kIaiResetSoftware = 0x0001;  ///< 01 45 00 05 00 01 00 00 (ON-WIRE + VERIFIED)
inline constexpr int kIaiCmdMovePosition = 0x06;  ///< op1 = row, op2 = override: runs (op2 + 1) / 256 (255 = 100 %)
inline constexpr int kIaiCmdServo = 0x09;         ///< op1 1 on / 0 off (VERIFIED)
inline constexpr int kIaiCmdHome = 0x0A;          ///< op1 = ORG & 0xFFFF (VERIFIED on the EC-R6S rod)
inline constexpr int kIaiCmdStop = 0x0B;
inline constexpr int kIaiCmdActionMode = 0x0C;    ///< op1 1 = PC / 0 = PIO, op2 1 = safety speed (VERIFIED)
inline constexpr int kRejectHoming = 0x24;        ///< "homing in progress"
// IAI 0x48 command codes (16 bit; UNVERIFIED: the command set of units with SUP3 IVC1 = 1, none of the ECs tested)
inline constexpr int kIai2CmdActionMode = 0x0022;
inline constexpr int kIai2CmdAlarmReset = 0x0023;
inline constexpr int kIai2CmdServo = 0x0030;
inline constexpr int kIai2CmdHome = 0x0031;
inline constexpr int kIai2CmdStop = 0x0034;
inline constexpr int kRejectHoming2 = 0x1006;     ///< 0x48 "motion command during homing"

// ------------------------------------------------------------------------------------------ identity
inline constexpr int kSystemEc = 0xEC;   ///< EC 24 V (VERIFIED)
inline constexpr int kSystemEc200 = 0xED;
inline constexpr int kSystemPcon = 0xAE; ///< PCON (VERIFIED: PCON-CB + RCP6)
inline constexpr int kSystemAcon = 0xAC;
inline constexpr int kSystemScon = 0xAA;
inline constexpr const char* kGenericFamily = "IAI controller";
inline constexpr int kAkidRotary = 1;    ///< rotary iff AKID = 1: positions in deg

// ------------------------------------------------------------------------------------------ word helpers (inline)

/// (high word, low word) of a signed 32-bit value, high word at the lower address.
inline std::pair<std::uint16_t, std::uint16_t> to_i32_words(std::int32_t value) noexcept {
    const auto v = static_cast<std::uint32_t>(value);
    return {static_cast<std::uint16_t>(v >> 16), static_cast<std::uint16_t>(v & 0xFFFF)};
}
/// (high word, low word) of an unsigned 32-bit value.
inline std::pair<std::uint16_t, std::uint16_t> to_u32_words(std::uint32_t value) noexcept {
    return {static_cast<std::uint16_t>(value >> 16), static_cast<std::uint16_t>(value & 0xFFFF)};
}
/// Signed 32-bit value of (high, low).
inline std::int32_t from_i32_words(std::uint16_t high, std::uint16_t low) noexcept {
    return static_cast<std::int32_t>((static_cast<std::uint32_t>(high) << 16) | low);
}
/// Unsigned 32-bit value of (high, low).
inline std::uint32_t from_u32_words(std::uint16_t high, std::uint16_t low) noexcept {
    return (static_cast<std::uint32_t>(high) << 16) | low;
}
/// Engineering units -> counts, rounded to nearest (73.95 mm -> 7395). std::invalid_argument for NaN / inf.
inline std::int64_t to_counts(double value, double scale = kPositionScale) {
    if (!std::isfinite(value)) throw std::invalid_argument("not a finite number");
    return static_cast<std::int64_t>(std::llround(value / scale));
}
/// Counts -> units rounded to 0.01 (the editions report positions this way).
inline double from_counts(std::int64_t counts, double scale = kPositionScale) noexcept {
    return std::round(static_cast<double>(counts) * scale * 100.0) / 100.0;
}
/// First register of position-table row n (0..kPositionMaxEntries-1, else std::out_of_range).
inline int position_address(int n) {
    if (n < 0 || n >= kPositionMaxEntries) throw std::out_of_range("position number must be 0..767");
    return kPositionTableBase + kPositionStride * n;
}
/// The +0xF word for the 15 words +0x0..+0xE of a row: its low byte makes the byte sum of the 32-byte row 0 mod 256.
inline std::uint16_t position_checksum(const std::vector<std::uint16_t>& words) {
    if (words.size() < static_cast<std::size_t>(kPositionWordsWritable)) throw std::invalid_argument("need 15 words");
    unsigned total = 0;
    for (int i = 0; i < kPositionWordsWritable; ++i) total += (words[i] >> 8) + (words[i] & 0xFF);
    return static_cast<std::uint16_t>((0x100 - (total & 0xFF)) & 0xFF);
}
/// Push percent -> PPOW counts, floor(pct x 255 / 100) (20 % -> 0x33, 70 % -> 0xB2, 100 % -> 255).
inline int push_power(double percent) {
    if (!std::isfinite(percent)) throw std::invalid_argument("not a finite number");
    if (percent == std::floor(percent)) return static_cast<int>(percent) * 255 / 100;
    return static_cast<int>(std::floor(percent * 255.0 / 100.0 + 1e-9));
}
/// PPOW counts -> percent.
inline double push_pct(int raw) noexcept { return raw * 100.0 / 255.0; }

// ------------------------------------------------------------------------------------------ lookups (model unit)

/// Product family of an FC11 system code: "EC", "PCON", "ACON/DCON", "SCON" or kGenericFamily.
std::string product_family(std::optional<int> system_code);
/// 0xEC / 0xED.
bool is_ec_family(std::optional<int> system_code);
/// "EC 24 V", "PCON"... ("" when unknown).
std::string system_name(std::optional<int> system_code);
/// EC family name of an FC11 (system, unit) pair ("EC standard", 'EC(MS) "2-size EC"'...), nullopt when unknown.
std::optional<std::string> ec_family(std::optional<int> system_code, std::optional<int> unit_code);
/// "deg" for a rotary axis (AKID 1, or ATYP TYPE without AKID), else "mm".
std::string axis_units(std::optional<int> actuator_kind, std::optional<bool> rotary_atyp = std::nullopt);
/// AKID kind in words ("slider", "rod", "gripper"... "unknown", "kind <n>").
std::string axis_kind_name(std::optional<int> actuator_kind, std::optional<bool> rotary_atyp = std::nullopt);
/// Directory category name ("position table"...; "?" when unknown).
std::string category_name(int category_id);
/// Register extent of a directory block (C# Registers.CategoryExtent / Python category_extent).
struct BlockExtent {
    int words = 0;      ///< a count certainly inside the block
    bool exact = false; ///< words is the exact size
    int words_max = 0;  ///< the largest extent the record allows
};
BlockExtent category_extent(int category_id, int member_count, int list_count, int member_size);
/// Words of the class-1 support members, in order: SUPT 2, DIAG 1, MVPT 1, SPTS 1, CTYP 1, SUP2 2, AEHS 2, ACHS 2,
/// ARSF 2, SPTM 2, SUP3 2.
const std::vector<std::pair<std::string, int>>& support_members();
/// Big-endian ASCII words -> printable ASCII (padding stripped; non-printable bytes dropped).
std::string words_to_ascii(const std::vector<std::uint16_t>& words);
/// Like words_to_ascii, nullopt when the words are not text (any byte other than printable ASCII or NUL) or blank.
std::optional<std::string> words_to_text(const std::vector<std::uint16_t>& words);
/// Any text as printable ASCII.
std::string printable(const std::string& text);
/// EC series of a model string ("EC-DS4H-150" -> "S4", "EC-R6S" -> "R6", "EC-GRBP8M" -> "GRB"), nullopt if unknown.
std::optional<std::string> ec_series(const std::optional<std::string>& model);
/// (documented minimum speed, source) from the model and lead; a recommendation only (warn, never refuse).
std::pair<std::optional<double>, std::string> recommended_min_speed(const std::optional<std::string>& model,
                                                                    std::optional<double> lead_mm);

}  // namespace reg
}  // namespace iai_actuator_driver
