using System;
using System.Collections.Generic;

namespace IaiActuatorDriver
{
    /// <summary>
    /// How servo / home / alarm-reset commands are sent. Moves always use the 0x9900 block and stops always use the STOP
    /// coil, whatever the strategy.
    /// </summary>
    public abstract class CommandStrategy
    {
        /// <summary>The method this strategy implements.</summary>
        public abstract CommandMethod Method { get; }

        /// <summary>The home command needs the unit's home pattern ORG (item 9.16).</summary>
        public virtual bool HomeNeedsOrg => false;

        /// <summary>The strategy for <paramref name="method"/>.</summary>
        public static CommandStrategy For(CommandMethod method) => method switch
        {
            CommandMethod.Coil => new StandardCoilStrategy(),
            CommandMethod.Iai => new IaiCommandStrategy(),
            CommandMethod.Iai2 => new IaiCommand2Strategy(),
            _ => throw new ArgumentOutOfRangeException(nameof(method), method, "unknown command method"),
        };

        /// <summary>Turn the servo on or off.</summary>
        public abstract void Servo(ModbusRtu bus, int slave, bool on);

        /// <summary>Start a home return (<paramref name="org"/>: the unit's home pattern, item 9.16, for strategies that need it).</summary>
        public abstract void StartHome(ModbusRtu bus, int slave, uint? org = null);

        /// <summary>Called after homing ended or failed (e.g. to release an edge-triggered coil).</summary>
        public virtual void FinishHome(ModbusRtu bus, int slave)
        {
        }

        /// <summary>Reset the current alarm (not a software reset).</summary>
        public abstract void ResetAlarm(ModbusRtu bus, int slave);

        /// <inheritdoc />
        public override string ToString() => GetType().Name + "()";
    }

    /// <summary>FC05 coils (SON, ALRS and HOME VERIFIED on the test units). The default.</summary>
    public sealed class StandardCoilStrategy : CommandStrategy
    {
        /// <inheritdoc />
        public override CommandMethod Method => CommandMethod.Coil;

        /// <inheritdoc />
        public override void Servo(ModbusRtu bus, int slave, bool on) => bus.WriteCoil(slave, Registers.COIL_SON, on);

        /// <inheritdoc />
        public override void StartHome(ModbusRtu bus, int slave, uint? org = null)
        {
            bus.WriteCoil(slave, Registers.COIL_HOME, false); // HOME acts on the rising edge
            bus.WriteCoil(slave, Registers.COIL_HOME, true);
        }

        /// <inheritdoc />
        public override void FinishHome(ModbusRtu bus, int slave) => bus.WriteCoil(slave, Registers.COIL_HOME, false);

        /// <inheritdoc />
        public override void ResetAlarm(ModbusRtu bus, int slave)
        {
            bus.WriteCoil(slave, Registers.COIL_ALRS, true); // ALRS acts on the rising edge
            bus.WriteCoil(slave, Registers.COIL_ALRS, false);
        }
    }

    /// <summary>
    /// IAI function 0x45 control commands. Servo cmd 9 op1 1/0: VERIFIED on EC #3 (servo OFF also ON-WIRE). Home cmd 0x0A
    /// with op1 = the home pattern ORG (item 9.16) &amp; 0xFFFF (<c>01 45 00 0A 00 07 00 00</c>): VERIFIED on EC #3 (op1 = 0
    /// is rejected 0x11 "1st operand error"); sent once, without retries. Alarm reset cmd 5 op1 0: frame accepted with no
    /// alarm active; clearing a real alarm UNVERIFIED. A rejection raises <see cref="IaiCommandRejectedException"/>.
    /// </summary>
    public sealed class IaiCommandStrategy : CommandStrategy
    {
        /// <inheritdoc />
        public override CommandMethod Method => CommandMethod.Iai;

        /// <inheritdoc />
        public override bool HomeNeedsOrg => true;

        /// <inheritdoc />
        public override void Servo(ModbusRtu bus, int slave, bool on) => bus.IaiCommand(slave, 0, Registers.IAI_CMD_SERVO, on ? 1 : 0);

        /// <inheritdoc />
        public override void StartHome(ModbusRtu bus, int slave, uint? org = null)
        {
            if (!org.HasValue || (org.Value & 0xFFFF) == 0)
            {
                throw new EcNotReadyException(
                    "the 0x45 home needs op1 = the unit's home pattern ORG (item 9.16); none given (op1 = 0 is rejected 0x11 '1st operand error'). Nothing sent");
            }

            // Sent once: a lost reply may hide a home that runs (Axis.Home then checks the status).
            bus.IaiCommand(slave, 0, Registers.IAI_CMD_HOME, (int)(org.Value & 0xFFFF), retries: 0);
        }

        /// <inheritdoc />
        public override void ResetAlarm(ModbusRtu bus, int slave) =>
            bus.IaiCommand(slave, 0, Registers.IAI_CMD_RESET, Registers.IAI_RESET_ALARM); // op1 0 = alarm reset (1 = software reset)
    }

    /// <summary>
    /// EXPERIMENTAL: IAI function 0x48 control commands. UNVERIFIED: untested on any unit, frame layout included. Expected
    /// only on units whose SUP3 IVC1 bit is set, which none of the units tested has (they all answer 0x45).
    /// </summary>
    public sealed class IaiCommand2Strategy : CommandStrategy
    {
        /// <inheritdoc />
        public override CommandMethod Method => CommandMethod.Iai2;

        /// <inheritdoc />
        public override void Servo(ModbusRtu bus, int slave, bool on) =>
            bus.IaiCommand2(slave, Registers.IAI2_CMD_SERVO, new List<uint> { on ? 1u : 0u });

        /// <inheritdoc />
        public override void StartHome(ModbusRtu bus, int slave, uint? org = null) => bus.IaiCommand2(slave, Registers.IAI2_CMD_HOME);

        /// <inheritdoc />
        public override void ResetAlarm(ModbusRtu bus, int slave) => bus.IaiCommand2(slave, Registers.IAI2_CMD_ALARM_RESET);
    }
}
