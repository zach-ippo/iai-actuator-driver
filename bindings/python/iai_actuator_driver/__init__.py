"""Unofficial Python driver for IAI actuators (ELECYLINDER and CON-family controllers such as the PCON) over Modbus RTU.

NOT an IAI product and not supported by IAI. Register meanings are worked out from public manuals,
serial-line captures and live read-back on hardware; see the README for the evidence
levels (VERIFIED / ON-WIRE / DOC / UNVERIFIED) before relying on anything.

Quick start::

    from iai_actuator_driver import Elecylinder
    with Elecylinder.open("COM3") as ec:          # baud auto-detected; or Elecylinder.open(SimulatedEC())
        print(ec.connect().model, ec.status())    # every range comes from the unit itself: ec.limits()

    with Cell.open() as cell:                     # every IAI USB adapter, every answering axis
        cell.home_all(); cell.move_all({"COM3:1": 50.0, "COM4:1": 20.0})
"""

from ._backend import BACKEND, backend_info  # first: loads the native library, before any driver module
from ._protocol import crc16
from .cell import Cell, Port, ScanHit, list_ports, scan
from .coord import (
    Arc,
    AxisRange,
    Circle,
    CoordinatedGroup,
    CoordinatedMotionError,
    CoordinatedResult,
    LineTiming,
    MotionPlan,
    Path,
    Polyline,
)
from .coord import Line as LineSegment
from .coord import plan as plan_path
from .ec import (
    STRATEGIES,
    Axis,
    CommandStrategy,
    Elecylinder,
    IaiCommand2Strategy,
    IaiCommandStrategy,
    MoveHandle,
    RegisterCommandStrategy,
    StandardCoilStrategy,
)
from .errors import (
    CellError,
    EcAlarmError,
    EcDriverError,
    EcError,
    EcNotReadyError,
    IaiCommandRejected,
    LineRuleError,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    MotionIncompleteError,
    MotionTimeoutError,
    NonVolatileWriteError,
    RecoveryError,
    ServoBlockedError,
    SoftLimitError,
    TransportError,
    UnitReplacedError,
    UnsafeCommandError,
    VerificationError,
)
from .gateway import (
    GatewayModeError,
    Line,
    PositionUnitError,
    UnsupportedLineError,
    detect_line,
    read_serial_code,
)
from .modbus import ModbusRTU, SerialTransport, Transport
from .monitor import AxisMonitor
from .recovery import RecoveryEvent, RecoveryPolicy, RecoveryReport
from .sim import SimulatedEC
from .sim_line import SimGatewayLine, SimLine, sim_line, sim_ports, sim_rcon_line, sim_robonet_line
from .values import (
    AlarmRecord,
    AxisLimits,
    BaudChangeResult,
    DeviceInfo,
    IndexCategory,
    IndexHeader,
    ParameterWriteResult,
    PositionEntry,
    PushResult,
    SoftwareResetResult,
    Status,
)

__version__ = "1.2.0"

__all__ = [
    "__version__",
    "BACKEND",
    "backend_info",
    "Elecylinder",
    "Axis",
    "Cell",
    "CellError",
    "Port",
    "ScanHit",
    "list_ports",
    "scan",
    "SimLine",
    "SimGatewayLine",
    "sim_line",
    "sim_ports",
    "sim_rcon_line",
    "sim_robonet_line",
    "Line",
    "detect_line",
    "read_serial_code",
    "Status",
    "DeviceInfo",
    "AxisLimits",
    "IndexHeader",
    "IndexCategory",
    "PositionEntry",
    "PushResult",
    "MoveHandle",
    "AxisMonitor",
    "RecoveryPolicy",
    "RecoveryReport",
    "RecoveryEvent",
    "AlarmRecord",
    "ParameterWriteResult",
    "SoftwareResetResult",
    "BaudChangeResult",
    "CommandStrategy",
    "StandardCoilStrategy",
    "IaiCommandStrategy",
    "IaiCommand2Strategy",
    "RegisterCommandStrategy",
    "STRATEGIES",
    "ModbusRTU",
    "SerialTransport",
    "Transport",
    "SimulatedEC",
    "crc16",
    "EcDriverError",
    "ModbusError",
    "ModbusTimeout",
    "ModbusCRCError",
    "ModbusFrameError",
    "ModbusException",
    "TransportError",
    "RecoveryError",
    "UnitReplacedError",
    "IaiCommandRejected",
    "UnsafeCommandError",
    "LineRuleError",
    "GatewayModeError",
    "PositionUnitError",
    "UnsupportedLineError",
    "ServoBlockedError",
    "EcError",
    "SoftLimitError",
    "EcNotReadyError",
    "EcAlarmError",
    "MotionTimeoutError",
    "MotionIncompleteError",
    "NonVolatileWriteError",
    "VerificationError",
    "Path",
    "LineSegment",
    "Arc",
    "Circle",
    "Polyline",
    "AxisRange",
    "LineTiming",
    "MotionPlan",
    "plan_path",
    "CoordinatedGroup",
    "CoordinatedResult",
    "CoordinatedMotionError",
]
