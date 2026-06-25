## @package GentoRobot
#  Python wrapper for Gento robot SDK.
#  Provides classes and functions to control Gento series robots (Marvin, Skye, Luna, etc.)
#  via a shared library (libGentoSDKPY.dll/.so).

import ctypes
from ctypes import *
import threading
import os
import inspect
from typing import Optional, Union, Sequence, List, Tuple
import time
import math
import threading

current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)


# ==================== Enumerations ====================
class FXObjType:
    """Object type identifiers for robot components."""
    OBJ_ARM0 = 0
    OBJ_ARM1 = 1
    OBJ_HEAD = 2
    OBJ_BODY = 3
    OBJ_LIFT = 4


class FXObjMask:
    """Bit masks for selecting multiple objects."""
    OBJ_ARM0_FLAG = 1 << 0
    OBJ_ARM1_FLAG = 1 << 1
    OBJ_HEAD_FLAG = 1 << 2
    OBJ_BODY_FLAG = 1 << 3
    OBJ_LIFT_FLAG = 1 << 4
    OBJ_ALL_FLAG = (OBJ_ARM0_FLAG | OBJ_ARM1_FLAG | OBJ_HEAD_FLAG | OBJ_BODY_FLAG | OBJ_LIFT_FLAG)


class FXLogMask:
    """Log level masks."""
    FX_LOG_DEBG_FLAG = 1 << 0
    FX_LOG_INFO_FLAG = 1 << 1
    FX_LOG_WARN_FLAG = 1 << 2
    FX_LOG_ERROR_FLAG = 1 << 3
    FX_LOG_ALL_FLAG = (FX_LOG_DEBG_FLAG | FX_LOG_INFO_FLAG | FX_LOG_WARN_FLAG | FX_LOG_ERROR_FLAG)


class FXChnType:
    """Communication channel types."""
    CHN_CANFD = 1
    CHN_485A = 2
    CHN_485B = 3


class FXTerminalType:
    """Terminal types."""
    TERMINAL_ARM0 = 0
    TERMINAL_ARM1 = 1


class FXHandType:
    """Hand types"""
    FX_HAND_LEFT = 0
    FX_HAND_RIGHT = 1


class FXHandAction:
    """Runtime action commands for the hand module"""
    FX_HAND_ACTION_DISABLE = 0
    FX_HAND_ACTION_ENABLE = 1
    FX_HAND_ACTION_RESET = 2


class FXUserDataType:
    """User data types for sample data registration."""
    FX_char = 0
    FX_short = 1
    FX_int = 2
    FX_long = 3
    FX_long_long = 5
    FX_unsigned_char = 6
    FX_unsigned_short = 7
    FX_unsigned_int = 8
    FX_unsigned_long = 9
    FX_unsigned_long_long = 10
    FX_float = 11
    FX_double = 12


class FXHandState:
    """Current state of the hand module"""
    FX_HAND_STATE_DISABLED = 0
    FX_HAND_STATE_ENABLED = 1
    FX_HAND_STATE_ERROR = 100


class FXStateType:
    """Global FX system states."""
    FX_STATE_IDLE = 0
    FX_STATE_POSITION = 1
    FX_STATE_IMP_JOINT = 2
    FX_STATE_IMP_CART = 3
    FX_STATE_IMP_FORCE = 4
    FX_STATE_DRAG_JOINT = 5
    FX_STATE_DRAG_CART_X = 6
    FX_STATE_DRAG_CART_Y = 7
    FX_STATE_DRAG_CART_Z = 8
    FX_STATE_DRAG_CART_R = 9
    FX_STATE_RELEASE = 10
    FX_STATE_PD = 11
    FX_STATE_ERROR = 100
    FX_STATE_TRANSFERRING = 101
    FX_STATE_UNKNOWN = 200


# ==================== RT/SG structures ====================
class StateCtr(Structure):
    """Control state structure."""
    _fields_ = [
        ("m_CurState", c_uint16),
        ("m_CmdState", c_uint16),
        ("m_ERRCode", c_uint32),
    ]


class ARM_OUT(Structure):
    """Arm feedback data structure."""
    _fields_ = [
        ("m_ARM_FBK_Joint_Pos", c_float * 7),
        ("m_ARM_FBK_Joint_Vel", c_float * 7),
        ("m_ARM_FBK_Joint_Cmd", c_float * 7),
        ("m_ARM_FBK_Joint_SensorTor", c_float * 7),
        ("m_ARM_FBK_Joint_ExternalTorEst", c_float * 7),
        ("m_ARM_FBK_Base_FNEst", c_float * 6),
        ("m_ARM_FBK_Base_Gyro", c_float * 6),
        ("m_ARM_FBK_Flange_FTSensor", c_float * 6),
        ("m_ARM_FBK_Tag", c_int32),
    ]


class ARM_IN(Structure):
    """Arm command data structure."""
    _fields_ = [
        ("m_ARM_CMD_Joint_Tor", c_float * 7),
        ("m_ARM_CMD_Joint_Pos", c_float * 7),
        ("m_ARM_CMD_Ctrl_DragType", c_int16),
        ("m_ARM_CMD_Ctrl_ForceType", c_int16),
        ("m_ARM_CMD_Ctrl_ForceDir", c_float * 5),
        ("m_ARM_CMD_Ctrl_TorqueDir", c_float * 5),
        ("m_ARM_CMD_Tag", c_int32),
        ("m_ARM_CMD_PD_Serial", c_int16),
        ("pad", c_byte * 2),
    ]


class ARM_RT(Structure):
    """Real-time arm data."""
    _fields_ = [
        ("m_ARM_State", StateCtr),
        ("m_ARM_IN", ARM_IN),
        ("m_ARM_OUT", ARM_OUT),
    ]


class HEAD_OUT(Structure):
    """Head feedback data."""
    _fields_ = [
        ("m_HEAD_FBK_Joint_Pos", c_float * 3),
        ("m_HEAD_FBK_Tag", c_int32),
    ]


class HEAD_IN(Structure):
    """Head command data."""
    _fields_ = [
        ("m_HEAD_CMD_Joint_Pos", c_float * 3),
        ("m_HEAD_CMD_Tag", c_int32),
    ]


class HEAD_RT(Structure):
    """Real-time head data."""
    _fields_ = [
        ("m_HEAD_State", StateCtr),
        ("m_HEAD_IN", HEAD_IN),
        ("m_HEAD_OUT", HEAD_OUT),
    ]


class BODY_OUT(Structure):
    """Body feedback data."""
    _fields_ = [
        ("m_BODY_FBK_Joint_Pos", c_float * 6),
        ("m_BODY_FBK_Joint_Vel", c_float * 6),
        ("m_BODY_FBK_Joint_SensorTor", c_float * 6),
        ("m_BODY_FBK_Base_Gyro", c_float * 6),
        ("m_BODY_FBK_Tag", c_int32),
    ]


class BODY_IN(Structure):
    """Body command data."""
    _fields_ = [
        ("m_BODY_CMD_Ctrl_Type", c_int32),
        ("m_BODY_CMD_Joint_Pos", c_float * 6),
        ("m_BODY_CMD_Tag", c_int32),
        ("m_BODY_CMD_PD_Serial", c_int16),
        ("pad", c_byte * 2),
    ]


class BODY_RT(Structure):
    """Real-time body data."""
    _fields_ = [
        ("m_BODY_State", StateCtr),
        ("m_BODY_IN", BODY_IN),
        ("m_BODY_OUT", BODY_OUT),
    ]


class LIFT_OUT(Structure):
    """Lift feedback data."""
    _fields_ = [
        ("m_LIFT_FBK_Joint_Pos", c_float * 2),
        ("m_LIFT_FBK_Tag", c_int32),
    ]


class LIFT_IN(Structure):
    """Lift command data."""
    _fields_ = [
        ("m_LIFT_CMD_Joint_Pos", c_float * 2),
        ("m_LIFT_CMD_Tag", c_int32),
    ]


class LIFT_RT(Structure):
    """Real-time lift data."""
    _fields_ = [
        ("m_LIFT_State", StateCtr),
        ("m_LIFT_IN", LIFT_IN),
        ("m_LIFT_OUT", LIFT_OUT),
    ]


class HAND_IN(Structure):
    """Hand command data."""
    _fields_ = [
        ("m_HAND_CMD_action", c_byte),
        ("pad", c_byte * 3),
        ("m_HAND_CMD_Joint_Pos", c_byte * 24),
        ("m_HAND_CMD_Joint_Tor", c_byte * 24),
    ]


class HAND_OUT(Structure):
    """Hand feedback data."""
    _fields_ = [
        ("m_HAND_FBK_State", c_byte),
        ("pad", c_byte * 3),
        ("m_HAND_FBK_Joint_ErrorCode", c_int16 * 24),
        ("m_HAND_FBK_Joint_Pos", c_byte * 24),
        ("m_HAND_FBK_Joint_Vel", c_byte * 24),
        ("m_HAND_FBK_Joint_Cmd", c_byte * 24),
    ]


class HAND_RT(Structure):
    """Real-time hand data."""
    _fields_ = [
        ("m_HAND_IN", HAND_IN),
        ("m_HAND_OUT", HAND_OUT),
    ]


class ROBOT_RT(Structure):
    """Root real-time data structure containing all components."""
    _fields_ = [
        ("m_RT_FrameSerial", c_int32),
        ("m_HEAD", HEAD_RT),
        ("m_ARMS", ARM_RT * 2),
        ("m_HANDS", HAND_RT * 2),
        ("m_BODY", BODY_RT),
        ("m_LIFT", LIFT_RT),
        ("wait_serial", c_ubyte),
        ("pad", c_ubyte * 3),
    ]


class ARM_SET(Structure):
    """Arm configuration parameters."""
    _fields_ = [
        ("m_ARM_Ctrl_ImpType", c_int32),
        ("m_ARM_Ctrl_VelRatio", c_float),
        ("m_ARM_Ctrl_AccRatio", c_float),
        ("m_ARM_Ctrl_JointK", c_float * 7),
        ("m_ARM_Ctrl_JointD", c_float * 7),
        ("m_ARM_Ctrl_CartK", c_float * 7),
        ("m_ARM_Ctrl_CartD", c_float * 7),
        ("m_ARM_Ctrl_ToolKine", c_float * 6),
        ("m_ARM_Ctrl_ToolDyna", c_float * 10),
        ("m_ARM_SET_SetTag", c_ubyte * 16),
        ("m_ARM_SET_UpdateTag", c_ubyte * 16),
    ]


class ARM_GET(Structure):
    """Arm additional feedback."""
    _fields_ = [
        ("m_ARM_FBK_Joint_Tor", c_float * 7),
        ("m_ARM_FBK_Joint_ExtPos", c_float * 7),
        ("m_ARM_FBK_Flange_DI", c_byte),
        ("m_ARM_FBK_LowSpdFlag", c_byte),
        ("m_ARM_FBK_TrajState", c_byte),
        ("m_ARM_FBK_PD_CmdQuality", c_byte),
    ]


class ARM_SG(Structure):
    """Arm static (configuration) data."""
    _fields_ = [
        ("m_ARM_SET", ARM_SET),
        ("m_ARM_GET", ARM_GET),
    ]


class HEAD_SET(Structure):
    """Head configuration."""
    _fields_ = [
        ("m_HEAD_Ctrl_VelRatio", c_float),
        ("m_HEAD_Ctrl_AccRatio", c_float),
        ("m_HEAD_SET_SetTag", c_ubyte * 4),
        ("m_HEAD_SET_UpdateTag", c_ubyte * 4),
    ]


class HEAD_GET(Structure):
    """Head additional feedback."""
    _fields_ = [
        ("m_HEAD_FBK_Joint_Tor", c_float * 3),
        ("m_HEAD_FBK_Joint_ExtPos", c_float * 3),
    ]


class HEAD_SG(Structure):
    """Head static data."""
    _fields_ = [
        ("m_HEAD_SET", HEAD_SET),
        ("m_HEAD_GET", HEAD_GET),
    ]


class BODY_SET(Structure):
    """Body configuration."""
    _fields_ = [
        ("m_BODY_Ctrl_VelRatio", c_float),
        ("m_BODY_Ctrl_AccRatio", c_float),
        ("m_BODY_Ctrl_PDK", c_float * 6),
        ("m_BODY_Ctrl_PDD", c_float * 6),
        ("m_BODY_SET_SetTag", c_ubyte * 6),
        ("m_BODY_SET_UpdateTag", c_ubyte * 6),
    ]


class BODY_GET(Structure):
    """Body additional feedback."""
    _fields_ = [
        ("m_BODY_FBK_Joint_Tor", c_float * 6),
        ("m_BODY_FBK_Joint_ExtPos", c_float * 6),
        ("m_BODY_FBK_TrajState", c_byte),
        ("m_BODY_FBK_PD_CmdQuality", c_byte),
        ("m_pad", c_byte * 2),
    ]


class BODY_SG(Structure):
    """Body static data."""
    _fields_ = [
        ("m_BODY_SET", BODY_SET),
        ("m_BODY_GET", BODY_GET),
    ]


class LIFT_SET(Structure):
    """Lift configuration."""
    _fields_ = [
        ("m_LIFT_Ctrl_VelRatio", c_float),
        ("m_LIFT_Ctrl_AccRatio", c_float),
        ("m_LIFT_SET_SetTag", c_ubyte * 4),
        ("m_LIFT_SET_UpdateTag", c_ubyte * 4),
    ]


class LIFT_GET(Structure):
    """Lift additional feedback."""
    _fields_ = [
        ("m_LIFT_FBK_Joint_Tor", c_float * 2),
        ("m_LIFT_FBK_TrajState", c_byte),
        ("m_pad", c_byte * 3),
    ]


class LIFT_SG(Structure):
    """Lift static data."""
    _fields_ = [
        ("m_LIFT_SET", LIFT_SET),
        ("m_LIFT_GET", LIFT_GET),
    ]


class HAND_GET(Structure):
    """Hand additional feedback."""
    _fields_ = [
        ("m_HAND_FBK_Joint_Tor", c_byte * 24),
        ("m_HAND_FBK_Joint_Temp", c_byte * 24),
    ]


class HAND_SET(Structure):
    """Hand configuration."""
    _fields_ = [
        ("m_HAND_Ctrl_KP", c_byte * 24),
        ("m_HAND_Ctrl_KD", c_byte * 24),
        ("m_HAND_Ctrl_MaxTor", c_byte * 24),
        ("m_HAND_SET_SetTag", c_ubyte * 8),
        ("m_HAND_SET_UpdateTag", c_ubyte * 8),
    ]


class HAND_SG(Structure):
    """Hand static data."""
    _fields_ = [
        ("m_HAND_SET", HAND_SET),
        ("m_HAND_GET", HAND_GET),
    ]


class OP_SET(Structure):
    """Operation command structure."""
    _fields_ = [
        ("m_OpIns", c_int16),
        ("m_OpValueS", c_byte * 30),
        ("m_OpValueI", c_int32),
        ("m_OpValueF", c_float),
        ("m_OpCmdSerial", c_int16),
        ("m_OpRetSerial", c_int16),
    ]


class ROBOT_SG(Structure):
    """Root static data structure."""
    _fields_ = [
        ("m_RT_FrameSerial", c_int32),
        ("m_HEAD", HEAD_SG),
        ("m_ARMS", ARM_SG * 2),
        ("m_HANDS", HAND_SG * 2),
        ("m_BODY", BODY_SG),
        ("m_LIFT", LIFT_SG),
        ("m_OP_SET", OP_SET),
    ]


# ==================== Motion planning structures ====================
class FX_InvKineSolverParams(Structure):
    """Inverse kinematics solver parameters."""
    _fields_ = [
        ("m_Input_IK_TargetTCP", c_double * 16),
        ("m_Input_IK_RefJoint", c_double * 7),
        ("m_Input_IK_ZSPType", c_int32),
        ("m_Input_IK_ZSPPara", c_double * 6),
        ("m_Input_ZSP_Angle", c_double),
        ("m_DGR1", c_double),
        ("m_DGR2", c_double),
        ("m_DGR3", c_double),
        ("m_Output_RetJoint", c_double * 7),
        ("m_OutPut_AllJoint", c_double * 64),
        ("m_OutPut_Result_Num", c_int32),
        ("m_Output_IsOutRange", c_int),
        ("m_Output_IsDeg", c_int * 7),
        ("m_Output_JntExdTags", c_int * 7),
        ("m_Output_JntExdABS", c_double),
        ("m_Output_IsJntExd", c_int),
        ("m_Output_RunLmtP", c_double * 7),
        ("m_Output_RunLmtN", c_double * 7),
    ]


class ArmsSynchronousPlanningParams(Structure):
    """Parameters for dual-arm synchronous motion planning."""
    _fields_ = [
        ("Max_Body_Start_PRR", c_double * 3),
        ("Arm0_Start_XYZABC", c_double * 6),
        ("Arm0_End_XYZABC", c_double * 6),
        ("Arm0_Ref_Joints", c_double * 7),
        ("Arm0_ZSP_Type", c_int32),
        ("Arm0_ZSP_Para", c_double * 6),
        ("Arm1_Start_XYZABC", c_double * 6),
        ("Arm1_End_XYZABC", c_double * 6),
        ("Arm1_Ref_Joints", c_double * 7),
        ("Arm1_ZSP_Type", c_int32),
        ("Arm1_ZSP_Para", c_double * 6),
        ("Freq", c_int32),
        ("Vel", c_double),
        ("Acc", c_double),
        ("Sync_Type", c_int32),
        ("World_Co_Flag", c_ubyte),
    ]


# ==================== code dictionary ====================
robot_type_map = {
    0: 'Null',
    1: 'MarvinProM3',
    2: 'MarvinProM6',
    3: 'GentoSkye',
    4: 'GentoLuna'
}

state_map = {
    0: "IDLE",
    1: "Position",
    2: "ImpJoint",
    3: "ImpCart",
    4: "ImpForce",
    5: "DragJoint",
    6: "DragCartX",
    7: "DragCartY",
    8: "DragCartZ",
    9: "DragCartR",
    10: "Release",
    11: "PD",
    100: "Error",
    101: "Transferring",
    200: "Unknown"
}

drag_type_map = {
    0: 'Null',
    1: 'Joint',
    2: 'CART_X',
    3: 'CART_Y',
    4: 'CART_Z',
    5: 'CART_R'
}

error_dict = {
    1: "Failed to load robot.ini config file",
    2: "Parameter exception",
    3: "Master initialization failed",
    4: "Slave configuration failed",
    5: "Failed to activate master",
    6: "Internal error 1",
    7: "Internal error 2",
    100: "Internal error 3",
    101: "Emergency",
    102: "Servo error",
    103: "PvtStreamBroken",
    104: "RequestPositionMode",
    105: "ResponsePositionMode",
    106: "RequestTorqueMode",
    107: "ResponseTorqueMode",
    108: "RequestEnableServo",
    109: "ResponseEnableServo",
    110: "ResponseDisableServo",
    111: "ServoStateAbnormal",
    112: "SlavePdoAbnormal",
    113: "SlaveStateAbnormal",
    114: "BusLinkDown",
}

fault_code_dict_EN = {
    "0x2250": "Drive short circuit",
    "0x2280": "Drive short circuit",
    "0x2310": "Continuous over current Phase U",
    "0x2311": "Continuous over current Phase V",
    "0x2312": "Continuous over current Phase W",
    "0x2320": "Hardware over current",
    "0x2330": "Drive output short circuit to ground",
    "0x3130": "Main power input abnormal",
    "0x3210": "DC bus over voltage",
    "0x3220": "DC bus under voltage",
    "0x4210": "Power module overheating",
    "0x6010": "CPU1 watchdog expired",
    "0x6011": "CPU2 watchdog expired",
    "0x7112": "Regeneration resistor overload",
    "0x8311": "Motor continuous overload",
    "0x8611": "Excessive position following error",
    "0x8612": "Positive software position limited",
    "0x8613": "Negative software position limited",
    "0x8800": "Encoder data overflow",
    "0xFF00": "CPU1 abnormal",
    "0xFF01": "CPU2 abnormal",
    "0xFF02": "CPU1 memory abnormal",
    "0xFF03": "CPU2 memory abnormal",
    "0xFF04": "CPU memory confliction",
    "0xFF05": "Magnetic pole positioning failed",
    "0xFF06": "Encoder data abnormal",
    "0xFF07": "Encoder communication abnormal",
    "0xFF08": "Encoder communication timeout",
    "0xFF09": "Encoder internal abnormal 1",
    "0xFF10": "The other axes of the drive are abnormal",
    "0xFF11": "Motor brake disconnection",
    "0xFF12": "Reserved",
    "0xFF13": "Reserved",
    "0xFF14": "Control encoder over speed",
    "0xFF15": "Drive continuous overload",
    "0xFF16": "Reserved",
    "0xFF17": "Drive output lack phase",
    "0xFF18": "Motor stall",
    "0xFF19": "Coprocessor communication abnormal",
    "0xFF20": "Encoder AB signal change abnormal",
    "0xFF21": "Excessive current following error",
    "0xFF22": "Target position value abnormal",
    "0xFF23": "Encoder data overflow on power-up",
    "0xFF24": "Target position value overflow",
    "0xFF25": "Motor brake abnormal",
    "0xFF26": "Control power under voltage",
    "0xFF27": "STO1 triggered alarm",
    "0xFF28": "STO2 triggered alarm",
    "0xFF29": "Positive hardware limit switch triggered alarm",
    "0xFF30": "Negative hardware limit switch triggered alarm",
    "0xFF31": "Motor over speed",
    "0xFF32": "Emergency stop switch triggered alarm",
    "0xFF33": "Torque saturation fault",
    "0xFF34": "Excessive velocity following error",
    "0xFF35": "Drive short circuit 2",
    "0xFF36": "Homing failed",
    "0xFF37": "EtherCAT process data error",
    "0xFF38": "EtherCAT command illegal",
    "0xFF39": "EtherCAT communication period error",
    "0xFF40": "Profile position operation error",
    "0xFF41": "EtherCAT sync mode error",
    "0xFF42": "Target position value over range",
    "0xFF43": "Rectifier module overtemperature",
    "0xFF44": "Heatsink overtemperature",
    "0xFF45": "Motor instantaneous overload Phase U",
    "0xFF46": "Motor instantaneous overload Phase V",
    "0xFF47": "Motor instantaneous overload Phase W",
    "0xFF48": "Dynamic brake overload",
    "0xFF49": "Drive internal abnormal 1",
    "0xFF50": "Limit switch abnormal",
    "0xFF51": "EtherCAT bus communication error",
    "0xFF52": "Interface encoder resolution change",
    "0xFF53": "Encoder overheat",
    "0xFF54": "Encoder battery undervoltage fault",
    "0xFF55": "Reserved",
    "0xFF56": "Reserved",
    "0xFF57": "The control mode is set incorrectly",
    "0xFF58": "Deviation of the power-on position out of range",
    "0xFF59": "Encoder acceleration abnormal fault",
    "0xFF60": "Motor blocked",
    "0xFF61": "Motor overtemperature",
    "0xFF62": "Incremental encoder Z signal abnormal",
    "0xFF63": "Exception writing EEPROM data",
    "0xFF64": "Abnormal reading EEPROM data",
    "0xFF65": "Control/power module mismatch",
    "0xFF66": "Abnormal holding brake control circuit",
    "0xFF67": "CPU overtemperature",
    "0xFF68": "CPU1 overload",
    "0xFF69": "CPU2 overload",
    "0xFF70": "CPU1 handshake failed",
    "0xFF71": "Drivemaster communication timeout",
    "0xFF72": "Reserved",
    "0xFF73": "Torque sensor abnormal",
    "0xFF74": "Reserved",
    "0xFF75": "ESC configured EEPROM exception",
    "0xFF76": "ESC internal access error",
    "0xFF77": "Servo enable not ready",
    "0xFF78": "CPU2 handshake failed",
    "0xFF79": "CPU1 main task timeout",
    "0xFF80": "Main power loss",
    "0xFF81": "Power failure of main power supply",
    "0xFF82": "An internal CPU error",
    "0xFF83": "Position actual value overflow",
    "0xFF84": "Reserved",
    "0xFF85": "Encoder internal abnormal 2",
    "0xFF86": "Reserved",
    "0xFF87": "Encoder internal abnormal 3",
    "0xFF88": "Reserved",
    "0xFF89": "Reserved",
    "0xFF8A": "STO1 circuit diagnosis abnormal",
    "0xFF8B": "STO2 circuit diagnosis abnormal",
    "0xFF8C": "Hall signal abnormal",
    "0xFF8D": "Encoder Hall-AB signal missing phase abnormal",
    "0xFF8E": "The second position following error",
    "0xFF8F": "STO wiring is abnormal",
    "0xFF90": "The second speed following error",
    "0xFF91": "Drive internal error 2",
}




class SwitchableLock:
    def __init__(self):
        self._real_lock = threading.Lock()
        self._enabled = True

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    def __enter__(self):
        if self._enabled:
            self._real_lock.acquire()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._enabled:
            self._real_lock.release()


# ==================== GentoRobot class ====================
class GentoRobot:
    """Main class for controlling Gento series robots.

    This class provides methods to connect to the robot, read/write real-time and
    static data, switch control modes, set parameters, and perform motion planning.
    It wraps the low-level C++ SDK functions.
    """

    def __init__(self, precision: int = 3):
        """Initialize the robot instance and load the SDK library.

        @param precision: Number of decimal places to round float values in dictionaries (default 3).
        @raise RuntimeError: If unable to get RT/SG pointers from the SDK.
        """
        self._lock = SwitchableLock()

        self._thread_id = 1

        self.precision = precision
        self._round_float = lambda v: round(v, self.precision)

        if os.name == 'nt':
            self.dll = ctypes.WinDLL(os.path.join(current_path, 'libGentoSDKPY.dll'))
        else:
            self.dll = ctypes.CDLL(os.path.join(current_path, 'libGentoSDKPY.so'))
        self._init_funcs()

        self._connected = False
        self.sys_version = None
        self.sdk_version = None

        self._kin_handle = None
        self.max_point = 5000

        self._rt_ptr = self.dll.FX_L1_Fbk_GetRT()
        self._sg_ptr = self.dll.FX_L1_Fbk_GetSG()
        if not self._rt_ptr or not self._sg_ptr:
            raise RuntimeError("Failed to get RT/SG pointers")

        self._rt = cast(self._rt_ptr, POINTER(ROBOT_RT))
        self._sg = cast(self._sg_ptr, POINTER(ROBOT_SG))

    def _init_funcs(self):
        """Set argument types and return types for all SDK functions."""
        d = self.dll

        # RobotCache
        d.FX_L1_Fbk_GetRT.argtypes = []
        d.FX_L1_Fbk_GetRT.restype = c_void_p

        d.FX_L1_Fbk_GetSG.argtypes = []
        d.FX_L1_Fbk_GetSG.restype = c_void_p

        # System
        d.FX_L1_System_SetLogLevel.argtypes = [c_uint]
        d.FX_L1_System_SetLogLevel.restype = None

        d.FX_L1_System_GetLogLevel.argtypes = []
        d.FX_L1_System_GetLogLevel.restype = c_int32

        d.FX_L1_System_GetControllerVersion.argtypes = []
        d.FX_L1_System_GetControllerVersion.restype = c_int32

        d.FX_L1_System_GetSDKVersion.argtypes = []
        d.FX_L1_System_GetSDKVersion.restype = c_int32

        d.FX_L1_System_Link.argtypes = [c_ubyte, c_ubyte, c_ubyte, c_ubyte, c_uint]
        d.FX_L1_System_Link.restype = c_int32

        d.FX_L1_System_GetLinkState.argtypes = []
        d.FX_L1_System_GetLinkState.restype = c_int32

        d.FX_L1_System_Unlink.argtypes = []
        d.FX_L1_System_Unlink.restype = None

        d.FX_L1_System_Reboot.argtypes = []
        d.FX_L1_System_Reboot.restype = c_int32

        d.FX_L1_System_Update.argtypes = [c_char_p, c_char_p]
        d.FX_L1_System_Update.restype = c_int

        d.FX_L1_System_SendFile.argtypes = [c_char_p, c_char_p]
        d.FX_L1_System_SendFile.restype = c_int32

        d.FX_L1_System_RecvFile.argtypes = [c_char_p, c_char_p]
        d.FX_L1_System_RecvFile.restype = c_int32

        # State feedback
        d.FX_L1_Fbk_GetCtrlObjDof.argtypes = [c_int32]
        d.FX_L1_Fbk_GetCtrlObjDof.restype = c_int32

        d.FX_L1_Fbk_CurrentState.argtypes = [c_int32]
        d.FX_L1_Fbk_CurrentState.restype = c_int32

        d.FX_L1_Fbk_GetRobotType.argtypes = []
        d.FX_L1_Fbk_GetRobotType.restype = c_int32

        d.FX_L1_Fbk_GetCtrlObjServoVersion.argtypes = [c_int32, POINTER((c_char * 30) * 7)]
        d.FX_L1_Fbk_GetCtrlObjServoVersion.restype = c_int32

        d.FX_L1_Fbk_GetCtrlObjSensorVersionAndSerial.argtypes = [c_int32, POINTER(c_int32 * 7),
                                                                 POINTER(c_int32 * 7)]
        d.FX_L1_Fbk_GetCtrlObjSensorVersionAndSerial.restype = c_int32

        d.FX_L1_State_GetServoErrorCode.argtypes = [c_int32, POINTER(c_uint * 7)]
        d.FX_L1_State_GetServoErrorCode.restype = c_int32

        d.FX_L1_State_ResetError.argtypes = [c_uint, c_uint, POINTER(c_uint)]
        d.FX_L1_State_ResetError.restype = c_int32

        d.FX_L1_Fbk_GetCtrlObjPhysicalState.argtypes = [c_int32, POINTER(c_int)]
        d.FX_L1_Fbk_GetCtrlObjPhysicalState.restype = c_int32

        d.FX_L1_Fbk_GetUserData.argtypes = [c_void_p]
        d.FX_L1_Fbk_GetUserData.restype = None

        d.FX_L1_Fbk_ResetUserDataSet.argtypes = []
        d.FX_L1_Fbk_ResetUserDataSet.restype = None

        d.FX_L1_Fbk_RegisterUserDataSet.argtypes = [c_char_p, c_int32, c_int32, c_int32]
        d.FX_L1_Fbk_RegisterUserDataSet.restype = c_int32

        d.FX_L1_Fbk_CheckUserDataSet.argtypes = [c_int32]
        d.FX_L1_Fbk_CheckUserDataSet.restype = c_int32

        # State switching
        switch_funcs = [
            "SwitchToIdle", "SwitchToPositionMode", "SwitchToImpJointMode",
            "SwitchToImpCartMode", "SwitchToImpForceMode", "SwitchToDragJoint",
            "SwitchToDragCartX", "SwitchToDragCartY", "SwitchToDragCartZ",
            "SwitchToDragCartR", "SwitchToCollaborativeRelease", "SwitchToPDMode"
        ]
        for func_name in switch_funcs:
            full_name = f"FX_L1_State_{func_name}"
            try:
                func = getattr(d, full_name)
            except AttributeError:
                print(f"Warning: {full_name} not found in DLL, skipping this function.")
                continue

            if func_name == "SwitchToPositionMode":
                func.argtypes = [c_int32, c_uint32, c_double, c_double]
            elif func_name in ["SwitchToImpJointMode", "SwitchToImpCartMode", "SwitchToPDMode"]:
                func.argtypes = [c_int32, c_uint32, c_double, c_double, POINTER(c_double * 7),
                                 POINTER(c_double * 7)]
            elif func_name == "SwitchToImpForceMode":
                func.argtypes = [c_int32, c_uint32, POINTER(c_double * 5), POINTER(c_double * 5)]
            elif func_name in ["SwitchToDragJoint", "SwitchToDragCartX", "SwitchToDragCartY",
                               "SwitchToDragCartZ", "SwitchToDragCartR", ]:
                func.argtypes = [c_int32, c_uint32, POINTER(c_double * 7), POINTER(c_double * 7)]
            elif func_name == "SwitchToCollaborativeRelease":
                func.argtypes = [c_int32, c_uint32]
            else:
                func.argtypes = [c_int32, c_uint32]
            func.restype = c_int32

        # Parameters
        d.FX_L1_Param_SetInt32.argtypes = [c_char_p, c_int32]
        d.FX_L1_Param_SetInt32.restype = c_int32

        d.FX_L1_Param_SetFloat.argtypes = [c_char_p, c_float]
        d.FX_L1_Param_SetFloat.restype = c_int32

        d.FX_L1_Param_GetInt32.argtypes = [c_char_p, POINTER(c_int32)]
        d.FX_L1_Param_GetInt32.restype = c_int32

        d.FX_L1_Param_GetFloat.argtypes = [c_char_p, POINTER(c_float)]
        d.FX_L1_Param_GetFloat.restype = c_int32

        d.FX_L1_Param_GetString.argtypes = [c_char_p, c_char_p]
        d.FX_L1_Param_GetString.restype = c_int32

        # Terminal
        d.FX_L1_Terminal_ClearData.argtypes = [c_int32]
        d.FX_L1_Terminal_ClearData.restype = c_int32

        d.FX_L1_Terminal_GetData.argtypes = [c_int32, c_uint32, POINTER(c_int32), POINTER(c_ubyte * 64),
                                             POINTER(c_uint32)]
        d.FX_L1_Terminal_GetData.restype = c_int32

        d.FX_L1_Terminal_SetData.argtypes = [c_int32, c_int32, c_uint32, POINTER(c_ubyte), c_uint32, POINTER(c_uint32)]
        d.FX_L1_Terminal_SetData.restype = c_int32

        # Config
        d.FX_L1_Config_SetBrakeLock.argtypes = [c_int32, c_ubyte]
        d.FX_L1_Config_SetBrakeLock.restype = c_int32

        d.FX_L1_Config_SetBrakeUnlock.argtypes = [c_int32, c_ubyte]
        d.FX_L1_Config_SetBrakeUnlock.restype = c_int32

        d.FX_L1_Config_ResetEncOffset.argtypes = [c_int32, c_ubyte]
        d.FX_L1_Config_ResetEncOffset.restype = c_int32

        d.FX_L1_Config_ClearEncError.argtypes = [c_int32, c_ubyte]
        d.FX_L1_Config_ClearEncError.restype = c_int32

        d.FX_L1_Config_DisableSoftLimit.argtypes = [c_int32, c_ubyte]
        d.FX_L1_Config_DisableSoftLimit.restype = c_int32

        d.FX_L1_Config_ResetAxisSensorOffset.argtypes = [c_int32, c_uint32]
        d.FX_L1_Config_ResetAxisSensorOffset.restype = c_int32

        d.FX_L1_Config_ResetSensorOffset.argtypes = [c_int32]
        d.FX_L1_Config_ResetSensorOffset.restype = c_int32

        d.FX_L1_Config_SetTraj.argtypes = [c_int32, c_uint32, POINTER(c_double)]
        d.FX_L1_Config_SetTraj.restype = c_int32

        d.FX_L1_Config_SetPDCmdCycleTime.argtypes = [c_int]
        d.FX_L1_Config_SetPDCmdCycleTime.restype = c_int

        # Runtime commands
        d.FX_L1_Runtime_EmergencyStop.argtypes = [c_uint32, c_uint32]
        d.FX_L1_Runtime_EmergencyStop.restype = c_uint32

        d.FX_L1_Runtime_SetTag.argtypes = [c_uint32, c_int32, c_int32]
        d.FX_L1_Runtime_SetTag.restype = c_int32

        d.FX_L1_Runtime_SetJointPosCmd.argtypes = [c_uint32, c_int32, POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetJointPosCmd.restype = c_int32

        d.FX_L1_Runtime_SetJointPosPDCmd.argtypes = [c_uint32, c_int32, POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetJointPosPDCmd.restype = c_int32

        d.FX_L1_Runtime_SetForceCtrl.argtypes = [c_uint32, c_int32, POINTER(c_double * 5)]
        d.FX_L1_Runtime_SetForceCtrl.restype = c_int32

        d.FX_L1_Runtime_SetTorqueCtrl.argtypes = [c_uint32, c_int32, POINTER(c_double * 5)]
        d.FX_L1_Runtime_SetTorqueCtrl.restype = c_int32

        d.FX_L1_Runtime_SetVelRatio.argtypes = [c_uint32, c_int32, c_double]
        d.FX_L1_Runtime_SetVelRatio.restype = c_int32

        d.FX_L1_Runtime_SetAccRatio.argtypes = [c_uint32, c_int32, c_double]
        d.FX_L1_Runtime_SetAccRatio.restype = c_int32

        d.FX_L1_Runtime_SetSpeedRatio.argtypes = [c_uint32, c_int32, c_double, c_double]
        d.FX_L1_Runtime_SetSpeedRatio.restype = c_int32

        d.FX_L1_Runtime_SetJointK.argtypes = [c_uint32, c_int32, POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetJointK.restype = c_int32

        d.FX_L1_Runtime_SetJointD.argtypes = [c_uint32, c_int32, POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetJointD.restype = c_int32

        d.FX_L1_Runtime_SetJointKD.argtypes = [c_uint32, c_int32, POINTER(c_double * 7), POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetJointKD.restype = c_int32

        d.FX_L1_Runtime_SetCartK.argtypes = [c_uint32, c_int32, POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetCartK.restype = c_int32

        d.FX_L1_Runtime_SetCartD.argtypes = [c_uint32, c_int32, POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetCartD.restype = c_int32

        d.FX_L1_Runtime_SetCartKD.argtypes = [c_uint32, c_int32, POINTER(c_double * 7), POINTER(c_double * 7)]
        d.FX_L1_Runtime_SetCartKD.restype = c_int32

        d.FX_L1_Runtime_SetToolK.argtypes = [c_uint32, c_int32, POINTER(c_double * 6)]
        d.FX_L1_Runtime_SetToolK.restype = c_int32

        d.FX_L1_Runtime_SetToolD.argtypes = [c_uint32, c_int32, POINTER(c_double * 10)]
        d.FX_L1_Runtime_SetToolD.restype = c_int32

        d.FX_L1_Runtime_SetToolKD.argtypes = [c_uint32, c_int32, POINTER(c_double * 6), POINTER(c_double * 10)]
        d.FX_L1_Runtime_SetToolKD.restype = c_int32

        d.FX_L1_Runtime_SetBodyPDP.argtypes = [c_uint32, POINTER(c_double * 6)]
        d.FX_L1_Runtime_SetBodyPDP.restype = c_int32

        d.FX_L1_Runtime_SetBodyPDD.argtypes = [c_uint32, POINTER(c_double * 6)]
        d.FX_L1_Runtime_SetBodyPDD.restype = c_int32

        d.FX_L1_Runtime_SetBodyPD.argtypes = [c_uint32, POINTER(c_double * 6), POINTER(c_double * 6)]
        d.FX_L1_Runtime_SetBodyPD.restype = c_int32

        d.FX_L1_Runtime_RunTraj.argtypes = [c_uint32, c_uint32]
        d.FX_L1_Runtime_RunTraj.restype = c_uint32

        d.FX_L1_Runtime_StopTraj.argtypes = [c_uint32, c_uint32]
        d.FX_L1_Runtime_StopTraj.restype = c_uint32

        # hand
        d.FX_L1_Runtime_SetHandAction.argtypes = [c_uint32, c_int32, c_int32]
        d.FX_L1_Runtime_SetHandAction.restype = c_int32

        d.FX_L1_Runtime_SetHandPos.argtypes = [c_uint32, c_int32, POINTER(c_int32)]
        d.FX_L1_Runtime_SetHandPos.restype = c_int32

        d.FX_L1_Runtime_SetHandP.argtypes = [c_uint32, c_int32, POINTER(c_int32)]
        d.FX_L1_Runtime_SetHandP.restype = c_int32

        d.FX_L1_Runtime_SetHandD.argtypes = [c_uint32, c_int32, POINTER(c_int32)]
        d.FX_L1_Runtime_SetHandD.restype = c_int32

        d.FX_L1_Runtime_SetHandMaxTor.argtypes = [c_uint32, c_int32, POINTER(c_int32)]
        d.FX_L1_Runtime_SetHandMaxTor.restype = c_int32

        # Kinematics functions
        d.FX_L1_Kinematics_Create.argtypes = []
        d.FX_L1_Kinematics_Create.restype = c_void_p

        d.FX_L1_Kinematics_Destroy.argtypes = [c_void_p]
        d.FX_L1_Kinematics_Destroy.restype = None

        d.FX_L1_Kinematics_SetLogLevel.argtypes = [c_uint]
        d.FX_L1_Kinematics_SetLogLevel.restype = None

        d.FX_L1_Kinematics_InitSingleArm_ByInputParams.argtypes = [
            c_void_p,
            c_int32,
            POINTER(c_int32),
            (c_double * 4) * 8,
            (c_double * 4) * 8,
            (c_double * 3) * 4,
            c_double * 3,
            c_double * 7,
            (c_double * 3) * 7,
            (c_double * 6) * 7,
        ]
        d.FX_L1_Kinematics_InitSingleArm_ByInputParams.restype = c_int32

        d.FX_L1_Kinematics_InitSingleArm_ByIniConfig.argtypes = [c_void_p, c_int32]
        d.FX_L1_Kinematics_InitSingleArm_ByIniConfig.restype = c_int32

        d.FX_L1_Kinematics_SetTool.argtypes = [c_void_p, c_int32, POINTER(c_double * 16)]
        d.FX_L1_Kinematics_SetTool.restype = c_int32

        d.FX_L1_Kinematics_RemoveTool.argtypes = [c_void_p, c_int32]
        d.FX_L1_Kinematics_RemoveTool.restype = c_int32

        d.FX_L1_Kinematics_ForwardKinematics.argtypes = [c_void_p, c_int32, POINTER(c_double * 7),
                                                         POINTER(c_double * 16)]
        d.FX_L1_Kinematics_ForwardKinematics.restype = c_int32

        d.FX_L1_Kinematics_Jacobian.argtypes = [c_void_p, c_int32, POINTER(c_double * 7), POINTER(c_double * 42)]
        d.FX_L1_Kinematics_Jacobian.restype = c_int32

        d.FX_L1_Kinematics_InverseKinematics.argtypes = [c_void_p, c_int32, POINTER(FX_InvKineSolverParams)]
        d.FX_L1_Kinematics_InverseKinematics.restype = c_int32

        d.FX_L1_Kinematics_SetSkyeBodyCondition.argtypes = [c_void_p, POINTER(c_double * 3), POINTER(c_double * 3),
                                                            c_double, c_double, c_double, c_double]
        d.FX_L1_Kinematics_SetSkyeBodyCondition.restype = c_int32

        d.FX_L1_Kinematics_SkyeBodyForwardKinematics.argtypes = [c_void_p, POINTER(c_double * 3),
                                                                 POINTER(c_double * 16), POINTER(c_double * 16)]
        d.FX_L1_Kinematics_SkyeBodyForwardKinematics.restype = c_int32

        d.FX_L1_Kinematics_SkyeBodyInverseKinematics.argtypes = [c_void_p, POINTER(c_double * 3), POINTER(c_double * 3),
                                                                 POINTER(c_double * 3)]
        d.FX_L1_Kinematics_SkyeBodyInverseKinematics.restype = c_int32

        d.FX_L1_Kinematics_SkyeBodyInverseKinematicsWithRef.argtypes = [c_void_p, POINTER(c_double * 3),
                                                                        POINTER(c_double * 3), POINTER(c_double * 3),
                                                                        POINTER(c_double * 3)]
        d.FX_L1_Kinematics_SkyeBodyInverseKinematicsWithRef.restype = c_int32

        d.FX_L1_Kinematics_PlanJointMove.argtypes = [c_void_p, c_int32, POINTER(c_double * 7), POINTER(c_double * 7),
                                                     c_double, c_double, c_int32, c_void_p, POINTER(c_int32)]
        d.FX_L1_Kinematics_PlanJointMove.restype = c_int32

        d.FX_L1_Kinematics_PlanLinearMove.argtypes = [c_void_p, c_int32, POINTER(c_double * 6), POINTER(c_double * 6),
                                                      POINTER(c_double * 7), c_double, c_double, c_int32, c_void_p,
                                                      POINTER(c_int32)]
        d.FX_L1_Kinematics_PlanLinearMove.restype = c_int32

        d.FX_L1_Kinematics_PlanLinearMoveKeepJoints.argtypes = [c_void_p, c_int32, POINTER(c_double * 7),
                                                                POINTER(c_double * 7),
                                                                c_double, c_double, c_int32, c_void_p, POINTER(c_int32)]
        d.FX_L1_Kinematics_PlanLinearMoveKeepJoints.restype = c_int32

        d.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart.argtypes = [
            c_void_p,
            c_int32,
            POINTER(c_double * 7),
            POINTER(c_double * 6),
            POINTER(c_double * 6),
            c_double,
            c_int32,
            POINTER(c_double),
            c_double,
            c_double,
            c_int32
        ]
        d.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart.restype = c_int32

        d.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints.argtypes = [
            c_void_p,
            c_int32,
            POINTER(c_double),
            c_double,
            c_int32,
            POINTER(c_double),
            c_double,
            c_double
        ]
        d.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints.restype = c_int32

        d.FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPoints.argtypes = [
            c_void_p,
            POINTER(c_double),
            POINTER(c_int32)
        ]
        d.FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPoints.restype = c_int32

        d.FX_L1_Kinematics_ArmsSynchronousPlanning.argtypes = [c_void_p, POINTER(ArmsSynchronousPlanningParams),
                                                               c_void_p, c_void_p, POINTER(c_int32)]
        d.FX_L1_Kinematics_ArmsSynchronousPlanning.restype = c_int32

        d.FX_L1_Kinematics_DynamicsIdentification.argtypes = [c_int32, c_char_p, POINTER(c_double),
                                                              POINTER(c_double * 3), POINTER(c_double * 6)]
        d.FX_L1_Kinematics_DynamicsIdentification.restype = c_int32

        # Helpers
        d.FX_L1_XYZABC2Matrix.argtypes = [POINTER(c_double * 6), POINTER(c_double * 16)]
        d.FX_L1_XYZABC2Matrix.restype = None
        d.FX_L1_Matrix2XYZABC.argtypes = [POINTER(c_double * 16), POINTER(c_double * 6)]
        d.FX_L1_Matrix2XYZABC.restype = None

    @property
    def rt(self) -> ROBOT_RT:
        """Get the real-time data structure (ctypes pointer)."""
        return self._rt.contents

    @property
    def sg(self) -> ROBOT_SG:
        """Get the static data structure (ctypes pointer)."""
        return self._sg.contents

    def get_rt_dict(self) -> dict:
        """Return a dictionary containing all real-time feedback data.

        @return: Dictionary with keys: frame_serial, head, arms, body, lift, hands.
        @note: Returns {"error": "Robot not connected"} if not connected.
        """
        if not self._connected:
            return {"error": "Robot not connected"}
        rt = self.rt
        data = {
            "frame_serial": rt.m_RT_FrameSerial,
            "head": {
                "state": {
                    "cur": rt.m_HEAD.m_HEAD_State.m_CurState,
                    "cmd": rt.m_HEAD.m_HEAD_State.m_CmdState,
                    "err": rt.m_HEAD.m_HEAD_State.m_ERRCode,
                },
                "cmd_pos": [self._round_float(rt.m_HEAD.m_HEAD_IN.m_HEAD_CMD_Joint_Pos[i]) for i in range(3)],
                "fb_pos": [self._round_float(rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos[i]) for i in range(3)],
            },
            "arms": [],
            "body": {
                "state": {
                    "cur": rt.m_BODY.m_BODY_State.m_CurState,
                    "cmd": rt.m_BODY.m_BODY_State.m_CmdState,
                    "err": rt.m_BODY.m_BODY_State.m_ERRCode,
                },
                "cmd_type": rt.m_BODY.m_BODY_IN.m_BODY_CMD_Ctrl_Type,
                "cmd_pos": [self._round_float(rt.m_BODY.m_BODY_IN.m_BODY_CMD_Joint_Pos[i]) for i in range(6)],
                "fb_pos": [self._round_float(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos[i]) for i in range(6)],
                "fb_vel": [self._round_float(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Vel[i]) for i in range(6)],
                "fb_sensor": [self._round_float(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_SensorTor[i]) for i in range(6)],
                "gyro": [self._round_float(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Base_Gyro[i]) for i in range(6)],
            },
            "lift": {
                "state": {
                    "cur": rt.m_LIFT.m_LIFT_State.m_CurState,
                    "cmd": rt.m_LIFT.m_LIFT_State.m_CmdState,
                    "err": rt.m_LIFT.m_LIFT_State.m_ERRCode,
                },
                "cmd_pos": [self._round_float(rt.m_LIFT.m_LIFT_IN.m_LIFT_CMD_Joint_Pos[i]) for i in range(2)],
                "fb_pos": [self._round_float(rt.m_LIFT.m_LIFT_OUT.m_LIFT_FBK_Joint_Pos[i]) for i in range(2)],
            },
            "hands": [],
        }
        for arm_idx in range(2):
            arm = rt.m_ARMS[arm_idx]
            arm_data = {
                "state": {
                    "cur": arm.m_ARM_State.m_CurState,
                    "cmd": arm.m_ARM_State.m_CmdState,
                    "err": arm.m_ARM_State.m_ERRCode,
                },
                "cmd": {
                    "joint_trq": [self._round_float(arm.m_ARM_IN.m_ARM_CMD_Joint_Tor[i]) for i in range(7)],
                    "joint_pos": [self._round_float(arm.m_ARM_IN.m_ARM_CMD_Joint_Pos[i]) for i in range(7)],
                    "drag_type": arm.m_ARM_IN.m_ARM_CMD_Ctrl_DragType,
                    "force_ctrl_type": arm.m_ARM_IN.m_ARM_CMD_Ctrl_ForceType,
                    "force_dir": [self._round_float(arm.m_ARM_IN.m_ARM_CMD_Ctrl_ForceDir[i]) for i in range(5)],
                    "torque_dir": [self._round_float(arm.m_ARM_IN.m_ARM_CMD_Ctrl_TorqueDir[i]) for i in range(5)],
                },
                "fb": {
                    "fb_pos": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Joint_Pos[i]) for i in range(7)],
                    "fb_vel": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Joint_Vel[i]) for i in range(7)],
                    "cmd_pos": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Joint_Cmd[i]) for i in range(7)],
                    "fb_sensor": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i]) for i in range(7)],
                    "fb_ext_torque": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst[i]) for i in
                                      range(7)],
                    "base_force": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Base_FNEst[i]) for i in range(6)],
                    "base_gyro": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Base_Gyro[i]) for i in range(6)],
                    "flange_force": [self._round_float(arm.m_ARM_OUT.m_ARM_FBK_Flange_FTSensor[i]) for i in range(6)],
                },
            }
            data["arms"].append(arm_data)
        for hand_idx in range(2):
            hand = rt.m_HANDS[hand_idx]
            hand_data = {
                "cmd": {
                    "action": hand.m_HAND_IN.m_HAND_CMD_action,
                    "pos": [hand.m_HAND_IN.m_HAND_CMD_Joint_Pos[i] for i in range(24)],
                    "torque": [hand.m_HAND_IN.m_HAND_CMD_Joint_Tor[i] for i in range(24)],
                },
                "fb": {
                    "state": hand.m_HAND_OUT.m_HAND_FBK_State,
                    "error_code": [hand.m_HAND_OUT.m_HAND_FBK_Joint_ErrorCode[i] for i in range(24)],
                    "fb_pos": [hand.m_HAND_OUT.m_HAND_FBK_Joint_Pos[i] for i in range(24)],
                    "vel": [hand.m_HAND_OUT.m_HAND_FBK_Joint_Vel[i] for i in range(24)],
                    "cmd_pos": [hand.m_HAND_OUT.m_HAND_FBK_Joint_Cmd[i] for i in range(24)],
                },
            }
            data["hands"].append(hand_data)
        return data

    def get_sg_dict(self) -> dict:
        """Return a dictionary containing all static configuration data.

        @return: Dictionary with keys: frame_serial, head, arms, hands, body, lift, op_set.
        @note: Returns {"error": "Robot not connected"} if not connected.
        """
        if not self._connected:
            return {"error": "Robot not connected"}
        sg = self.sg
        data = {
            "frame_serial": sg.m_RT_FrameSerial,
            "head": {
                "set": {
                    "vel_ratio": self._round_float(sg.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_VelRatio),
                    "acc_ratio": self._round_float(sg.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_AccRatio),
                    "set_tags": [sg.m_HEAD.m_HEAD_SET.m_HEAD_SET_SetTag[i] for i in range(4)],
                    "update_tags": [sg.m_HEAD.m_HEAD_SET.m_HEAD_SET_UpdateTag[i] for i in range(4)],
                },
                "get": {
                    "current": [self._round_float(sg.m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_Tor[i]) for i in range(3)],
                    "ext_pos": [self._round_float(sg.m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_ExtPos[i]) for i in range(3)],
                },
            },
            "arms": [],
            "hands": [],
            "body": {
                "set": {
                    "vel_ratio": self._round_float(sg.m_BODY.m_BODY_SET.m_BODY_Ctrl_VelRatio),
                    "acc_ratio": self._round_float(sg.m_BODY.m_BODY_SET.m_BODY_Ctrl_AccRatio),
                    "pdk": [self._round_float(sg.m_BODY.m_BODY_SET.m_BODY_Ctrl_PDK[i]) for i in range(6)],
                    "pdd": [self._round_float(sg.m_BODY.m_BODY_SET.m_BODY_Ctrl_PDD[i]) for i in range(6)],
                    "set_tags": [sg.m_BODY.m_BODY_SET.m_BODY_SET_SetTag[i] for i in range(6)],
                    "update_tags": [sg.m_BODY.m_BODY_SET.m_BODY_SET_UpdateTag[i] for i in range(6)],
                },
                "get": {
                    "joint_torque": [self._round_float(sg.m_BODY.m_BODY_GET.m_BODY_FBK_Joint_Tor[i]) for i in range(6)],
                    "ext_pos": [self._round_float(sg.m_BODY.m_BODY_GET.m_BODY_FBK_Joint_ExtPos[i]) for i in range(6)],
                    "traj_state": sg.m_BODY.m_BODY_GET.m_BODY_FBK_TrajState,
                },
            },
            "lift": {
                "set": {
                    "vel_ratio": self._round_float(sg.m_LIFT.m_LIFT_SET.m_LIFT_Ctrl_VelRatio),
                    "acc_ratio": self._round_float(sg.m_LIFT.m_LIFT_SET.m_LIFT_Ctrl_AccRatio),
                    "set_tags": [sg.m_LIFT.m_LIFT_SET.m_LIFT_SET_SetTag[i] for i in range(4)],
                    "update_tags": [sg.m_LIFT.m_LIFT_SET.m_LIFT_SET_UpdateTag[i] for i in range(4)],
                },
                "get": {
                    "joint_torque": [self._round_float(sg.m_LIFT.m_LIFT_GET.m_LIFT_FBK_Joint_Tor[i]) for i in range(2)],
                    "traj_state": sg.m_LIFT.m_LIFT_GET.m_LIFT_FBK_TrajState,
                },
            },
            "op_set": {
                "ins": sg.m_OP_SET.m_OpIns,
                "value_s": bytes(sg.m_OP_SET.m_OpValueS).decode('utf-8', errors='replace').rstrip('\x00'),
                "value_i": sg.m_OP_SET.m_OpValueI,
                "value_f": self._round_float(sg.m_OP_SET.m_OpValueF),
                "cmd_serial": sg.m_OP_SET.m_OpCmdSerial,
                "ret_serial": sg.m_OP_SET.m_OpRetSerial,
            },
        }
        for arm_idx in range(2):
            arm_sg = sg.m_ARMS[arm_idx]
            arm_data = {
                "set": {
                    "imp_type": arm_sg.m_ARM_SET.m_ARM_Ctrl_ImpType,
                    "vel_ratio": self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_VelRatio),
                    "acc_ratio": self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_AccRatio),
                    "joint_k": [self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_JointK[i]) for i in range(7)],
                    "joint_d": [self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_JointD[i]) for i in range(7)],
                    "cart_k": [self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_CartK[i]) for i in range(7)],
                    "cart_d": [self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_CartD[i]) for i in range(7)],
                    "tool_kine": [self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_ToolKine[i]) for i in range(6)],
                    "tool_dyna": [self._round_float(arm_sg.m_ARM_SET.m_ARM_Ctrl_ToolDyna[i]) for i in range(10)],
                    "set_tags": [arm_sg.m_ARM_SET.m_ARM_SET_SetTag[i] for i in range(16)],
                    "update_tags": [arm_sg.m_ARM_SET.m_ARM_SET_UpdateTag[i] for i in range(16)],
                },
                "get": {
                    "joint_torque": [self._round_float(arm_sg.m_ARM_GET.m_ARM_FBK_Joint_Tor[i]) for i in range(7)],
                    "ext_pos": [self._round_float(arm_sg.m_ARM_GET.m_ARM_FBK_Joint_ExtPos[i]) for i in range(7)],
                    "tip_di": arm_sg.m_ARM_GET.m_ARM_FBK_Flange_DI,
                    "low_speed_flag": arm_sg.m_ARM_GET.m_ARM_FBK_LowSpdFlag,
                    "traj_state": arm_sg.m_ARM_GET.m_ARM_FBK_TrajState,
                    "pd_cmd_quality": arm_sg.m_ARM_GET.m_ARM_FBK_PD_CmdQuality,
                },
            }
            data["arms"].append(arm_data)
        for hand_idx in range(2):
            hand_sg = sg.m_HANDS[hand_idx]
            hand_data = {
                "set": {
                    "kp": [hand_sg.m_HAND_SET.m_HAND_Ctrl_KP[i] for i in range(24)],
                    "kd": [hand_sg.m_HAND_SET.m_HAND_Ctrl_KD[i] for i in range(24)],
                    "max_torque": [hand_sg.m_HAND_SET.m_HAND_Ctrl_MaxTor[i] for i in range(24)],
                    "set_tag": [hand_sg.m_HAND_SET.m_HAND_SET_SetTag[i] for i in range(8)],
                    "update_tag": [hand_sg.m_HAND_SET.m_HAND_SET_UpdateTag[i] for i in range(8)],
                },
                "get": {
                    "joint_torque": [hand_sg.m_HAND_GET.m_HAND_FBK_Joint_Tor[i] for i in range(24)],
                    "joint_temp": [hand_sg.m_HAND_GET.m_HAND_FBK_Joint_Temp[i] for i in range(24)],
                },
            }
            data["hands"].append(hand_data)
        return data

    def enable_lock(self):
        self._lock.enable()

    def disable_lock(self):
        self._lock.disable()

    # ==================== System ====================
    def get_sdk_version(self) -> str:
        """Return the SDK version as a hexadecimal string (8 characters)."""
        ver = self.dll.FX_L1_System_GetSDKVersion()
        return format(ver & 0xFFFFFFFF, '08X')

    def get_controller_version(self) -> str:
        """Return the controller version as a hexadecimal string (8 characters)."""
        ver = self.dll.FX_L1_System_GetControllerVersion()
        return format(ver & 0xFFFFFFFF, '08X')

    def set_log_level(self, level: int):
        """Set the log level mask.

        @param level: Bitmask using FXLogMask constants.
        """
        self.dll.FX_L1_System_SetLogLevel(level)

    def get_log_level(self) -> int:
        """Get the current log level mask.

        @return: Log level bitmask (see FXLogMask).
        """
        with self._lock:
            return self.dll.FX_L1_System_GetLogLevel()

    def link(self, ip1: int, ip2: int, ip3: int, ip4: int, log_level=FXLogMask.FX_LOG_INFO_FLAG) -> int:
        """Establish connection to the robot controller.

        @param ip1: First octet of the IP address.
        @param ip2: Second octet.
        @param ip3: Third octet.
        @param ip4: Fourth octet.
        @param log_level: Log mask (default FX_LOG_INFO_FLAG).
        @return: On success, returns a positive system version number.
                 On failure, returns an error code.
        """
        with self._lock:
            ret = self.dll.FX_L1_System_Link(ip1, ip2, ip3, ip4, log_level)
            if ret >= 0:
                self._connected = True
            else:
                self._connected = False
            return ret

    def check_link_state(self):
        """ Check link state

        @return:
                0 typically indicates a disconnected or inactive link;
                1 values indicate an active or established connection;
                -1 values indicate an active connection but can't receive.
        """
        with self._lock:
            return self.dll.FX_L1_System_GetLinkState()

    def unlink(self):
        """Disconnect from the robot controller."""
        with self._lock:
            self.dll.FX_L1_System_Unlink()
            self._connected = False

    def reboot(self) -> int:
        """Reboot the robot controller.

        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_System_Reboot()

    def system_update(self, update_file_path: str, ini_file_path: str) -> int:
        """Update the robot system using provided firmware files.

        @param update_file_path: Path to the update file.
        @param ini_file_path: Path to the ini configuration file.
        @return: 0 on success, negative error code otherwise.
        """
        update_arg = update_file_path.encode('utf-8') if update_file_path else None
        ini_arg = ini_file_path.encode('utf-8') if ini_file_path else None
        with self._lock:
            return self.dll.FX_L1_System_Update(update_arg, ini_arg)

    def send_file(self, local_path: str, remote_path: str) -> int:
        """Send a file to the robot controller.

        @param local_path: Local file path.
        @param remote_path: Destination path on the controller.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_System_SendFile(local_path.encode('utf-8'), remote_path.encode('utf-8'))

    def recv_file(self, local_path: str, remote_path: str) -> int:
        """Receive a file from the robot controller.

        @param local_path: Local path to save the file.
        @param remote_path: Remote file path on the controller.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_System_RecvFile(local_path.encode('utf-8'), remote_path.encode('utf-8'))

    # ==================== State feedback ====================

    def get_ctrl_obj_dof(self, obj_type: int) -> int:
        """Get the number of degrees of freedom for a given object.

        @param obj_type: Object type (FXObjType).
        @return: DOF count.
        """
        with self._lock:
            return self.dll.FX_L1_Fbk_GetCtrlObjDof(obj_type)

    def get_ctrl_obj_servo_version(self, obj_type: int) -> Tuple[int, List[str]]:
        """Get servo version strings for each axis of the object.

        @param obj_type: Object type (FXObjType).
        @return: Tuple of (return_code, version_strings). return_code is 0 on success,
                 non-zero on failure (see FXFuncReturn). On failure, version_strings is an empty list.
        """
        VersionArrayType = (c_char * 30) * 7
        version_arr = VersionArrayType()
        with self._lock:
            ret = self.dll.FX_L1_Fbk_GetCtrlObjServoVersion(obj_type, version_arr)
            if ret < 0:
                return ret, []
            result = []
            for i in range(7):
                raw_bytes = bytes(version_arr[i])
                null_pos = raw_bytes.find(b'\x00')
                if null_pos != -1:
                    raw_bytes = raw_bytes[:null_pos]
                result.append(raw_bytes.decode('utf-8', errors='ignore'))
            return ret, result

    def get_ctrl_obj_sensor_version_and_serial(self, obj_type: int) -> Tuple[int, List[int], List[int]]:
        """Get sensor version and serial numbers for each axis.

        @param obj_type: Object type (FXObjType).
        @return: Tuple of (return_code, versions, serials). return_code is 0 on success,
                 non-zero on failure. On failure, versions and serials are empty lists.
        """
        version_arr = (c_int32 * 7)()
        serial_arr = (c_int32 * 7)()
        with self._lock:
            ret = self.dll.FX_L1_Fbk_GetCtrlObjSensorVersionAndSerial(obj_type, version_arr, serial_arr)
            if ret < 0:
                return ret, [], []
            versions = [version_arr[i] for i in range(7)]
            serials = [serial_arr[i] for i in range(7)]
            return ret, versions, serials

    def get_ctrl_obj_physical_state(self, obj_type: int) -> tuple[int, int]:
        """Get the physical state of the object.

        @param obj_type: Object type (FXObjType).
        @return: Tuple (return_code, state_value).
        """
        state = c_int()
        with self._lock:
            ret = self.dll.FX_L1_Fbk_GetCtrlObjPhysicalState(obj_type, ctypes.byref(state))
            return ret, state.value

    def current_state(self, obj_type: int) -> int:
        """Get the current control state of the object.

        @param obj_type: Object type (FXObjType).
        @return: State code (see state_map).
        """
        with self._lock:
            return self.dll.FX_L1_Fbk_CurrentState(obj_type)

    def get_robot_type(self) -> str:
        """Get the robot model name.

        @return: String describing the robot type (e.g., 'GentoSkye', 'GentoLuna').
        """
        with self._lock:
            ret = self.dll.FX_L1_Fbk_GetRobotType()
            robot_type = robot_type_map.get(ret, f"{ret}")
            return robot_type

    def get_servo_error_codes(self, obj_type: int) -> tuple[int, str]:
        """Get human-readable servo error codes for each axis.

        @param obj_type: Object type (FXObjType).
        @return: error code, multi-line string with errors per axis.
        """
        arr = (c_uint * 7)()
        with self._lock:
            ret = self.dll.FX_L1_State_GetServoErrorCode(obj_type, arr)
            if ret != 0:
                return ret, ''
            if obj_type in (FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1):
                num_axes = 7
            elif obj_type == FXObjType.OBJ_BODY:
                num_axes = 6
            elif obj_type == FXObjType.OBJ_HEAD:
                num_axes = 3
            elif obj_type == FXObjType.OBJ_LIFT:
                num_axes = 2
            else:
                num_axes = 7
            axis_names = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"][:num_axes]
            lines = []
            for i in range(num_axes):
                code_val = arr[i]
                code_str = f"0x{code_val:04X}"
                if code_val == 0:
                    desc = "No error"
                else:
                    desc = fault_code_dict_EN.get(code_str, "Unknown error")
                lines.append(f"{axis_names[i]}: {code_str} - {desc}")
            return ret, "\n".join(lines)

    def reset_error(self, obj_type: int, timeout: int) -> tuple[int, int]:
        """Reset error states for the specified object.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @return: Tuple (return_code, system_error_code).
        """
        system_errorcode = c_uint(0)
        with self._lock:
            ret = self.dll.FX_L1_State_ResetError(obj_type, timeout, byref(system_errorcode))
        return ret, system_errorcode.value

    def fbk_reset_user_data_set(self) -> None:
        """Reset all registered user data sets.

        Clears all previously registered user data sets.
        """
        with self._lock:
            self.dll.FX_L1_Fbk_ResetUserDataSet()

    def fbk_register_user_data_set(self, name: str, data_type: int, sub: int, data_num: int) -> int:
        """Register a user data set for feedback sampling.

        @param name:      Name identifier for the user data set (e.g., 'ROBOT_RT.m_RT_FrameSerial').
        @param data_type: Data type (FXUserDataType).
        @param sub:       Sub-index for the data set.
        @param data_num:  Number of data elements.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Fbk_RegisterUserDataSet(name.encode('utf-8'), data_type, sub, data_num)

    def fbk_check_user_data_set(self, user_data_len: int) -> int:
        """Check the validity of user data set length.

        @param user_data_len: Total length of user data in bytes to verify.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Fbk_CheckUserDataSet(user_data_len)

    def fbk_get_user_data(self, buffer) -> None:
        """Sample the user data from feedback into the provided buffer.

        @param buffer: A ctypes buffer (e.g., created by create_string_buffer or cast)
                       large enough to hold the registered user data.
        """
        with self._lock:
            self.dll.FX_L1_Fbk_GetUserData(buffer)

    # ==================== State switching ====================
    def switch_to_idle(self, obj_type: int, timeout: int) -> int:
        """Switch the specified object to Idle state.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_State_SwitchToIdle(obj_type, timeout)

    def switch_to_position_mode(self, obj_type: int, timeout: int, vel: float, acc: float) -> int:
        """Switch to position control mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param vel: Velocity ratio (1-100 %).
        @param acc: Acceleration ratio (1-100 %).
        @return: 0 on success, negative error code otherwise.
        """
        vel = max(1, min(vel, 100))
        acc = max(1, min(acc, 100))
        with self._lock:
            return self.dll.FX_L1_State_SwitchToPositionMode(obj_type, timeout, vel, acc)

    def switch_to_imp_joint_mode(self, obj_type: int, timeout: int, vel: float, acc: float,
                                 k: List[float], d: List[float]) -> int:
        """Switch to joint impedance control mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param vel: Velocity ratio (1-100 %).
        @param acc: Acceleration ratio (1-100 %).
        @param k: Joint stiffness coefficients (7 elements, >=0).
        @param d: Joint damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        vel = max(1, min(vel, 100))
        acc = max(1, min(acc, 100))
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToImpJointMode(obj_type, timeout, vel, acc, k_arr, d_arr)

    def switch_to_imp_cart_mode(self, obj_type: int, timeout: int, vel: float, acc: float,
                                k: List[float], d: List[float]) -> int:
        """Switch to Cartesian impedance control mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param vel: Velocity ratio (1-100 %).
        @param acc: Acceleration ratio (1-100 %).
        @param k: Cartesian stiffness coefficients (7 elements, >=0).
        @param d: Cartesian damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        vel = max(1, min(vel, 100))
        acc = max(1, min(acc, 100))
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToImpCartMode(obj_type, timeout, vel, acc, k_arr, d_arr)

    def switch_to_pd_mode(self, obj_type: int, timeout: int,
                          vel_ratio: float, acc_ratio: float,
                          k: List[float], d: List[float]) -> int:
        """Switch to PD (Proportional-Derivative) control mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param vel_ratio: Velocity ratio (1-100 %).
        @param acc_ratio: Acceleration ratio (1-100 %).
        @param k: PD proportional gains (7 elements, >=0).
        @param d: PD derivative gains (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        vel = max(1, min(vel_ratio, 100))
        acc = max(1, min(acc_ratio, 100))
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToPDMode(obj_type, timeout,
                                                       vel_ratio, acc_ratio,
                                                       k_arr, d_arr)

    def switch_to_imp_force_mode(self, obj_type: int, timeout: int,
                                 force_ctrl: List[float], torque_ctrl: List[float]) -> int:
        """Switch to force/impedance hybrid control mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param force_ctrl: Force control parameters (5 elements).
        @param torque_ctrl: Torque control parameters (5 elements).
        @param raise_error: If True, raise RuntimeError on failure; otherwise return error message string.
        @return: 0 on success, negative error code otherwise.
        """
        f_arr = (c_double * 5)(*force_ctrl)
        t_arr = (c_double * 5)(*torque_ctrl)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToImpForceMode(obj_type, timeout, f_arr, t_arr)

    def switch_to_drag_joint(self, obj_type: int, timeout: int, k: List[float], d: List[float]) -> int:
        """Switch to joint-level drag (gravity compensation) mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param k: Joint stiffness coefficients (7 elements, >=0).
        @param d: Joint damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToDragJoint(obj_type, timeout, k_arr, d_arr)

    def switch_to_drag_cart_x(self, obj_type: int, timeout: int, k: List[float], d: List[float]) -> int:
        """Switch to Cartesian X-direction drag mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param k: Cartesian stiffness coefficients (7 elements, >=0).
        @param d: Cartesian damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToDragCartX(obj_type, timeout, k_arr, d_arr)

    def switch_to_drag_cart_y(self, obj_type: int, timeout: int, k: List[float], d: List[float]) -> int:
        """Switch to Cartesian Y-direction drag mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param k: Cartesian stiffness coefficients (7 elements, >=0).
        @param d: Cartesian damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToDragCartY(obj_type, timeout, k_arr, d_arr)

    def switch_to_drag_cart_z(self, obj_type: int, timeout: int, k: List[float], d: List[float]) -> int:
        """Switch to Cartesian Z-direction drag mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param k: Cartesian stiffness coefficients (7 elements, >=0).
        @param d: Cartesian damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToDragCartZ(obj_type, timeout, k_arr, d_arr)

    def switch_to_drag_cart_r(self, obj_type: int, timeout: int, k: List[float], d: List[float]) -> int:
        """Switch to Cartesian rotational drag mode.

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @param k: Cartesian stiffness coefficients (7 elements, >=0).
        @param d: Cartesian damping coefficients (7 elements, >=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_State_SwitchToDragCartR(obj_type, timeout, k_arr, d_arr)

    def switch_to_collab_release(self, obj_type: int, timeout: int) -> int:
        """Switch to collaborative release mode (zero gravity, passive).

        @param obj_type: Object type (FXObjType).
        @param timeout: Timeout in milliseconds.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_State_SwitchToCollaborativeRelease(obj_type, timeout)

    # ==================== Parameters ====================
    def param_set_int(self, name: str, value: int) -> int:
        """Set an integer parameter.

        @param name: Parameter name.
        @param value: Integer value.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Param_SetInt32(name.encode('utf-8'), value)

    def param_set_float(self, name: str, value: float) -> int:
        """Set a float parameter.

        @param name: Parameter name.
        @param value: Float value.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Param_SetFloat(name.encode('utf-8'), value)

    def param_get_int(self, name: str) -> tuple[int, int]:
        """Get an integer parameter.

        @param name: Parameter name.
        @return: Tuple (return_code, integer value).
        """
        ret_value = c_int32()
        with self._lock:
            ret = self.dll.FX_L1_Param_GetInt32(name.encode('utf-8'), byref(ret_value))
            return ret, ret_value.value

    def param_get_float(self, name: str) -> tuple[int, float]:
        """Get a float parameter.

        @param name: Parameter name.
        @return: Tuple (return_code, float value).
        """
        ret_value = c_float()
        with self._lock:
            ret = self.dll.FX_L1_Param_GetFloat(name.encode('utf-8'), byref(ret_value))
            return ret, ret_value.value

    def param_get_string(self, name: str, max_len: int = 256) -> tuple[int, str]:
        """Get a string parameter.

        @param name: Parameter name.
        @return: Tuple (return_code, string).
        """
        buf = create_string_buffer(max_len)
        with self._lock:
            ret = self.dll.FX_L1_Param_GetString(
                name.encode('utf-8'),
                buf
            )
            value = buf.value.decode('utf-8')
            return ret, value

    # ==================== Terminal ====================
    def terminal_clear(self, terminal_type: int) -> int:
        """Clear terminal data buffer.

        @param terminal_type: Terminal type (FXTerminalType).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Terminal_ClearData(terminal_type)

    def terminal_get(self, terminal_type: int, timeout: int = 1000) -> Tuple[int, bytes, int]:
        """Read data from the terminal.

        @param terminal_type: Terminal type (FXTerminalType).
        @param timeout: Timeout in milliseconds.
        @return: Tuple (channel_type, data_bytes, receiving_time_ms).
        """
        output_chn = c_int32()
        data = (c_ubyte * 64)()
        recv_time = c_uint32()
        with self._lock:
            n = self.dll.FX_L1_Terminal_GetData(terminal_type, timeout, byref(output_chn), data, byref(recv_time))
        if n <= 0:
            return (0, b'', 0)
        return (output_chn.value, bytes(data[:n]), recv_time.value)

    def terminal_set(self, terminal_type: int, chn_type: int, data, timeout: int = 1000) -> Tuple[int, int]:
        """Write data to the terminal.

        @param terminal_type: Terminal type (FXTerminalType).
        @param chn_type: Channel type (FXChnType).
        @param data: Data as bytes or hex string (e.g., 'FF 01 02').
        @param timeout: Timeout in milliseconds.
        @return: Tuple (return_code, sending_time_ms).
        @raise ValueError: If data format is invalid or too long.
        """
        if isinstance(data, str):
            hex_str = data.replace(' ', '')
            if len(hex_str) % 2 != 0:
                raise ValueError("Hex string must have even length")
            data = bytes.fromhex(hex_str)
        elif not isinstance(data, bytes):
            raise TypeError("data must be bytes or a hex string")
        data_len = len(data)
        if data_len > 64:
            raise ValueError("Data too long (max 64 bytes)")
        buf = (c_ubyte * data_len)(*data)
        send_time = c_uint32()
        with self._lock:
            ret = self.dll.FX_L1_Terminal_SetData(terminal_type, chn_type, timeout, buf, data_len, byref(send_time))
        return (ret, send_time.value)

    # ==================== Configuration ====================
    def config_brake_lock(self, obj_type: int, axis_mask: int) -> int:
        """Lock brakes for selected axes.

        @param obj_type: Object type (FXObjType).
        @param axis_mask: Bitmask of axes.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_SetBrakeLock(obj_type, axis_mask & 0xFF)

    def config_brake_unlock(self, obj_type: int, axis_mask: int) -> int:
        """Unlock brakes for selected axes.

        @param obj_type: Object type (FXObjType).
        @param axis_mask: Bitmask of axes to unlock (only lower 8 bits used).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_SetBrakeUnlock(obj_type, axis_mask & 0xFF)

    def config_reset_enc_offset(self, obj_type: int, axis_mask: int) -> int:
        """Reset encoder offsets for selected axes.

        @param obj_type: Object type (FXObjType).
        @param axis_mask: Bitmask of axes to reset (only lower 8 bits used).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_ResetEncOffset(obj_type, axis_mask & 0xFF)

    def config_clear_enc_error(self, obj_type: int, axis_mask: int) -> int:
        """Clear encoder errors for selected axes.

        @param obj_type: Object type (FXObjType).
        @param axis_mask: Bitmask of axes to clear (only lower 8 bits used).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_ClearEncError(obj_type, axis_mask & 0xFF)

    def config_disable_soft_limit(self, obj_type: int, axis_mask: int) -> int:
        """Disable software limits for selected axes.

        @param obj_type: Object type (FXObjType).
        @param axis_mask: Bitmask of axes to disable limits for (only lower 8 bits used).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_DisableSoftLimit(obj_type, axis_mask & 0xFF)

    def config_clear_axis_sensor_offset(self, obj_type: int, axis_id: int) -> int:
        """Clear sensor offset for a single axis.

        @param obj_type: Object type (FXObjType).
        @param axis_id: Axis index (0‑based).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_ResetAxisSensorOffset(obj_type, axis_id)

    def config_clear_sensor_offset(self, obj_type: int) -> int:
        """Clear all sensor offsets for the object.

        @param obj_type: Object type (FXObjType).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_ResetSensorOffset(obj_type)

    def config_set_traj(self, obj_type: int, point_set, point_num: int) -> int:
        """Set trajectory points for the object.

        @param obj_type: Object type (FXObjType).
        @param point_set: Pointer to trajectory points.
        @param point_num: Number of points.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Config_SetTraj(obj_type, point_num, point_set)

    def config_set_pd_cmd_cycle_time(self, cycle_time: int) -> int:
        """Set the command cycle time for PD control.

        @param cycle_time: Cycle time in milliseconds. Must be >= 0.
        @return: 0 on success, negative error code otherwise.
        """
        if cycle_time < 0:
            raise ValueError("cycle_time must be positive")
        with self._lock:
            return self.dll.FX_L1_Config_SetPDCmdCycleTime(cycle_time)

    # ==================== Runtime commands ====================
    def emergency_stop(self, obj_mask: int) -> int:
        """Trigger emergency stop for the objects specified by the mask.

        @param obj_mask: Bitmask (FXObjMask).
        @return: bitmask (FXObjMask).
        """
        with self._lock:
            return self.dll.FX_L1_Runtime_EmergencyStop(self._thread_id, obj_mask)

    def runtime_set_tag(self, obj_type: int, tag: int) -> int:
        """Set a user-defined tag on the specified object.

        @param obj_type: Object type (FXObjType).
        @param tag: User-defined tag value.
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Runtime_SetTag(self._thread_id, obj_type, tag)

    def runtime_set_joint_pos_cmd(self, obj_type: int, positions: List[float]) -> int:
        """Send a joint position command to the specified object.

        @param obj_type: Object type (FXObjType). Supports ARM0, ARM1, BODY, HEAD, LIFT.
        @param positions: List of target joint positions. Length must match the object's DOF:
                          arm:7, body:6, head:3, lift:2.
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If obj_type is unsupported or number of positions is incorrect.
        """
        if obj_type in (FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1):
            expected = 7
        elif obj_type == FXObjType.OBJ_BODY:
            expected = 6
        elif obj_type == FXObjType.OBJ_HEAD:
            expected = 3
        elif obj_type == FXObjType.OBJ_LIFT:
            expected = 2
        else:
            raise ValueError(f"Unsupported object type: {obj_type}")
        if len(positions) != expected:
            raise ValueError(f"Expected {expected} positions for object type {obj_type}, got {len(positions)}")
        padded = list(positions) + [0.0] * (7 - expected)
        arr = (c_double * 7)(*padded)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetJointPosCmd(self._thread_id, obj_type, arr)

    def runtime_set_joint_pos_pd_cmd(self, obj_type: int, positions: List[float]) -> int:
        """Send a joint position command in PD control mode.

        @param obj_type: Object type (FXObjType). Supports ARM0, ARM1.
        @param positions: List of 7 target joint positions (radians).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If positions does not have exactly 7 elements.
        """
        if len(positions) != 7:
            raise ValueError(f"PD position command requires 7 elements, got {len(positions)}")
        arr = (c_double * 7)(*positions)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetJointPosPDCmd(self._thread_id, obj_type, arr)

    def runtime_set_force_ctrl(self, obj_type: int, force_ctrl: List[float]) -> int:
        """Set force control parameters for impedance force mode.

        @param obj_type: Object type (FXObjType) – typically an arm.
        @param force_ctrl: List of 5 floats defining force control(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If force_ctrl does not have exactly 5 elements.
        """
        if len(force_ctrl) != 5:
            raise ValueError("force_ctrl must have 5 elements")
        arr = (c_double * 5)(*force_ctrl)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetForceCtrl(self._thread_id, obj_type, arr)

    def runtime_set_torque_ctrl(self, obj_type: int, torque_ctrl: List[float]) -> int:
        """Set torque control parameters for impedance force mode.

        @param obj_type: Object type (FXObjType) – typically an arm.
        @param torque_ctrl: List of 5 floats defining torque control(dir_x,dir_y,dir_z,torque(N*m),distance(deg)).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If torque_ctrl does not have exactly 5 elements.
        """
        if len(torque_ctrl) != 5:
            raise ValueError("torque_ctrl must have 5 elements")
        arr = (c_double * 5)(*torque_ctrl)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetTorqueCtrl(self._thread_id, obj_type, arr)

    def runtime_set_vel_ratio(self, obj_type: int, ratio: float) -> int:
        """Set the velocity scaling ratio for motion commands.

        @param obj_type: Object type (FXObjType).
        @param ratio: Velocity ratio in percent (1‑100). Values outside this range are clamped.
        @return: 0 on success, negative error code otherwise.
        """
        ratio = max(1, min(ratio, 100))
        with self._lock:
            return self.dll.FX_L1_Runtime_SetVelRatio(self._thread_id, obj_type, ratio)

    def runtime_set_acc_ratio(self, obj_type: int, ratio: float) -> int:
        """Set the acceleration scaling ratio for motion commands.

        @param obj_type: Object type (FXObjType).
        @param ratio: Acceleration ratio in percent (1‑100). Values outside this range are clamped.
        @return: 0 on success, negative error code otherwise.
        """
        ratio = max(1, min(ratio, 100))
        with self._lock:
            return self.dll.FX_L1_Runtime_SetAccRatio(self._thread_id, obj_type, ratio)

    def runtime_set_speed_ratio(self, obj_type: int, vel_ratio: float, acc_ratio: float) -> int:
        """Set both velocity and acceleration scaling ratios.

        @param obj_type: Object type (FXObjType).
        @param vel_ratio: Velocity ratio in percent (1‑100).
        @param acc_ratio: Acceleration ratio in percent (1‑100).
        @return: 0 on success, negative error code otherwise.
        """
        vel_ratio = max(1, min(vel_ratio, 100))
        acc_ratio = max(1, min(acc_ratio, 100))
        with self._lock:
            return self.dll.FX_L1_Runtime_SetSpeedRatio(self._thread_id, obj_type, vel_ratio, acc_ratio)

    def runtime_set_joint_k(self, obj_type: int, k: List[float]) -> int:
        """Set joint‑space stiffness coefficients for impedance control.

        @param obj_type: Object type (FXObjType) – must be an arm (7 DOF).
        @param k: List of 7 stiffness coefficients (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If k does not have exactly 7 elements.
        """
        if len(k) != 7:
            raise ValueError("k must have 7 elements")
        k = [max(0, v) for v in k]
        arr = (c_double * 7)(*k)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetJointK(self._thread_id, obj_type, arr)

    def runtime_set_joint_d(self, obj_type: int, d: List[float]) -> int:
        """Set joint‑space damping coefficients for impedance control.

        @param obj_type: Object type (FXObjType) – must be an arm (7 DOF).
        @param d: List of 7 damping coefficients (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If d does not have exactly 7 elements.
        """
        if len(d) != 7:
            raise ValueError("d must have 7 elements")
        d = [max(0, v) for v in d]
        arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetJointD(self._thread_id, obj_type, arr)

    def runtime_set_joint_kd(self, obj_type: int, k: List[float], d: List[float]) -> int:
        """Set both joint‑space stiffness and damping coefficients.

        @param obj_type: Object type (FXObjType) – must be an arm (7 DOF).
        @param k: List of 7 stiffness coefficients (>=0).
        @param d: List of 7 damping coefficients (>=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetJointKD(self._thread_id, obj_type, k_arr, d_arr)

    def runtime_set_cart_k(self, obj_type: int, k: List[float]) -> int:
        """Set Cartesian‑space stiffness coefficients for impedance control.

        @param obj_type: Object type (FXObjType) – must be an arm (7 DOF).
        @param k: List of 7 stiffness coefficients (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If k does not have exactly 7 elements.
        """
        if len(k) != 7:
            raise ValueError("k must have 7 elements")
        k = [max(0, v) for v in k]
        arr = (c_double * 7)(*k)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetCartK(self._thread_id, obj_type, arr)

    def runtime_set_cart_d(self, obj_type: int, d: List[float]) -> int:
        """Set Cartesian‑space damping coefficients for impedance control.

        @param obj_type: Object type (FXObjType) – must be an arm (7 DOF).
        @param d: List of 7 damping coefficients (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If d does not have exactly 7 elements.
        """
        if len(d) != 7:
            raise ValueError("d must have 7 elements")
        d = [max(0, v) for v in d]
        arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetCartD(self._thread_id, obj_type, arr)

    def runtime_set_cart_kd(self, obj_type: int, k: List[float], d: List[float]) -> int:
        """Set both Cartesian‑space stiffness and damping coefficients.

        @param obj_type: Object type (FXObjType) – must be an arm (7 DOF).
        @param k: List of 7 stiffness coefficients (>=0).
        @param d: List of 7 damping coefficients (>=0).
        @return: 0 on success, negative error code otherwise.
        """
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (c_double * 7)(*k)
        d_arr = (c_double * 7)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetCartKD(self._thread_id, obj_type, k_arr, d_arr)

    def runtime_set_tool_k(self, obj_type: int, k: List[float]) -> int:
        """Set tool kinematics (offset from flange to TCP).

        @param obj_type: Object type (FXObjType) – must be an arm.
        @param k: List of 6 floats: [x, y, z, rx, ry, rz] (position and orientation).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If k does not have exactly 6 elements.
        """
        if len(k) != 6:
            raise ValueError("kinematics must have 6 elements (tool offset)")
        arr = (c_double * 6)(*k)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetToolK(self._thread_id, obj_type, arr)

    def runtime_set_tool_d(self, obj_type: int, d: List[float]) -> int:
        """Set tool dynamics parameters.

        @param obj_type: Object type (FXObjType) – must be an arm.
        @param d: List of 10 floats: [mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If d does not have exactly 10 elements.
        """
        if len(d) != 10:
            raise ValueError("dynamics must have 10 elements (tool dynamics)")
        arr = (c_double * 10)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetToolD(self._thread_id, obj_type, arr)

    def runtime_set_tool_kd(self, obj_type: int, k: List[float], d: List[float]) -> int:
        """Set both tool kinematics and dynamics in one call.

        @param obj_type: Object type (FXObjType) – must be an arm.
        @param k: List of 6 floats: tool offset [x, y, z, rx, ry, rz].
        @param d: List of 10 floats: tool dynamics [mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If k does not have 6 elements or d does not have 10 elements.
        """
        if len(k) != 6:
            raise ValueError("kinematics must have 6 elements (tool offset)")
        if len(d) != 10:
            raise ValueError("dynamics must have 10 elements (tool dynamics)")
        k_arr = (c_double * 6)(*k)
        d_arr = (c_double * 10)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetToolKD(self._thread_id, obj_type, k_arr, d_arr)

    def runtime_set_body_pdp(self, p: List[float]) -> int:
        """Set body proportional (P) gains.

        @param p: List of 6 proportional gains (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If p does not have exactly 6 elements.
        """
        if len(p) != 6:
            raise ValueError("p must have 6 elements")
        p = [max(0, v) for v in p]
        arr = (c_double * 6)(*p)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetBodyPDP(self._thread_id, arr)

    def runtime_set_body_pdd(self, d: List[float]) -> int:
        """Set body derivative (D) gains.

        @param d: List of 6 derivative gains (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If d does not have exactly 6 elements.
        """
        if len(d) != 6:
            raise ValueError("d must have 6 elements")
        d = [max(0, v) for v in d]
        arr = (c_double * 6)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetBodyPDD(self._thread_id, arr)

    def runtime_set_body_pd(self, p: List[float], d: List[float]) -> int:
        """Set both body proportional and derivative gains.

        @param p: List of 6 proportional gains (>=0).
        @param d: List of 6 derivative gains (>=0).
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If p or d does not have exactly 6 elements.
        """
        if len(p) != 6 or len(d) != 6:
            raise ValueError("p and d must have 6 elements each")
        p = [max(0, v) for v in p]
        d = [max(0, v) for v in d]
        p_arr = (c_double * 6)(*p)
        d_arr = (c_double * 6)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetBodyPD(self._thread_id, p_arr, d_arr)

    def runtime_run_traj(self, obj_mask: int) -> int:
        """Start trajectory execution for the objects specified by the mask.

        @param obj_mask: Bitmask of objects (FXObjMask).
        @return: bitmask (FXObjMask) of objects that successfully started trajectory execution.
        """
        with self._lock:
            return self.dll.FX_L1_Runtime_RunTraj(self._thread_id, obj_mask)

    def runtime_stop_traj(self, obj_mask: int) -> int:
        """Stop trajectory execution for the objects specified by the mask.

        @param obj_mask: Bitmask of objects (FXObjMask).
        @return: bitmask (FXObjMask) of objects that successfully stopped trajectory execution.
        """
        with self._lock:
            return self.dll.FX_L1_Runtime_StopTraj(self._thread_id, obj_mask)

    def runtime_set_hand_action(self, hand_type: int, hand_action: int) -> int:
        """Set the runtime action command for the specified hand.

        @param hand_type: Hand identifier (FXHandType).
        @param hand_action: Action to be applied (FXHandAction).
        @return: 0 on success, negative error code otherwise.
        """
        with self._lock:
            return self.dll.FX_L1_Runtime_SetHandAction(self._thread_id, hand_type, hand_action)

    def runtime_set_hand_pos(self, hand_type: int, pos: List[int]) -> int:
        """Set the target position ratio command for the specified hand.

        @param hand_type: Hand identifier (FXHandType).
        @param pos: Target position array for each joint (length = 24), value=[0,100].
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If pos does not have exactly 24 elements.
        """
        if len(pos) != 24:
            raise ValueError("pos must have 24 elements")
        arr = (c_int * 24)(*pos)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetHandPos(self._thread_id, hand_type, arr)

    def runtime_set_hand_p(self, hand_type: int, p: List[int]) -> int:
        """Set the proportional gain (P) for the specified hand.

        @param hand_type: Hand identifier (FXHandType).
        @param p: Proportional gain array for each joint (length = 24), value=[0,100].
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If p does not have exactly 24 elements.
        """
        if len(p) != 24:
            raise ValueError("p must have 24 elements")
        arr = (c_int * 24)(*p)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetHandP(self._thread_id, hand_type, arr)

    def runtime_set_hand_d(self, hand_type: int, d: List[int]) -> int:
        """Set the derivative gain (D) for the specified hand.

        @param hand_type: Hand identifier (FXHandType).
        @param d: Derivative gain array for each joint (length = 24), value=[0,100].
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If d does not have exactly 24 elements.
        """
        if len(d) != 24:
            raise ValueError("d must have 24 elements")
        arr = (c_int * 24)(*d)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetHandD(self._thread_id, hand_type, arr)

    def runtime_set_hand_max_tor(self, hand_type: int, max_tor: List[int]) -> int:
        """Set the maximum torque limit for the specified hand.

        @param hand_type: Hand identifier (@ref FXHandType).
        @param max_tor: Maximum torque array for each joint (length = 24), value=[0,100].
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If max_tor does not have exactly 24 elements.
        """
        if len(max_tor) != 24:
            raise ValueError("max_tor must have 24 elements")
        arr = (c_int * 24)(*max_tor)
        with self._lock:
            return self.dll.FX_L1_Runtime_SetHandMaxTor(self._thread_id, hand_type, arr)

    # ==================== Kinematics ====================
    # ---------- Private handle management ----------
    def _ensure_kin_handle(self):
        """Create kinematics handle if not already created."""
        if self._kin_handle is None:
            self._kin_handle = self.dll.FX_L1_Kinematics_Create()
            if self._kin_handle is None:
                raise RuntimeError("Failed to create kinematics handle")

    def _destroy_kin_handle(self):
        """Destroy kinematics handle if it exists."""
        if self._kin_handle:
            self.dll.FX_L1_Kinematics_Destroy(self._kin_handle)
            self._kin_handle = None

    # ---------- Initialization ----------
    def kine_log_level(self, log_level=FXLogMask.FX_LOG_INFO_FLAG):
        """Set kinematics logging level.

         @param log_level: Log mask (FXLogMask).
         """
        with self._lock:
            self.dll.FX_L1_Kinematics_SetLogLevel(log_level)

    def init_single_arm_manual(self, robot_serial: int, robot_type: int,
                               DH: List[List[float]],
                               PNVA: List[List[float]],
                               BOUND: List[List[float]],
                               GRV: List[float],
                               MASS: List[float],
                               MCP: List[List[float]],
                               I: List[List[float]]
                               ) -> int:
        """Manually initialize kinematics for a single arm using provided parameters.

       @param robot_serial: Serial number of the robot.
       @param robot_type: Type code.
       @param DH: Denavit-Hartenberg parameters (8x4).
       @param PNVA: P/N/V/A parameters (8x4).
       @param BOUND: Joint limits (4x3).
       @param GRV: Gravity vector (3).
       @param MASS: Link masses (7).
       @param MCP: Center of mass positions (7x3).
       @param I: Inertia tensors (7x6).
       @return: 0 on success, negative error code otherwise.
       """
        self._ensure_kin_handle()
        type_var = c_int32(robot_type)

        dh_arr = ((c_double * 4) * 8)()
        for i in range(8):
            for j in range(4):
                dh_arr[i][j] = DH[i][j]

        pnva_arr = ((c_double * 4) * 8)()
        for i in range(8):
            for j in range(4):
                pnva_arr[i][j] = PNVA[i][j]

        bound_arr = ((c_double * 3) * 4)()
        for i in range(4):
            for j in range(3):
                bound_arr[i][j] = BOUND[i][j]

        grv_arr = (c_double * 3)(*GRV)
        mass_arr = (c_double * 7)(*MASS)

        mcp_arr = ((c_double * 3) * 7)()
        for i in range(7):
            for j in range(3):
                mcp_arr[i][j] = MCP[i][j]

        i_arr = ((c_double * 6) * 7)()
        for i in range(7):
            for j in range(6):
                i_arr[i][j] = I[i][j]
        with self._lock:
            return self.dll.FX_L1_Kinematics_InitSingleArm_ByInputParams(
                self._kin_handle, robot_serial,
                byref(type_var), dh_arr, pnva_arr, bound_arr,
                grv_arr, mass_arr, mcp_arr, i_arr
            )

    def init_single_arm_config(self, robot_serial: int) -> int:
        """Initialize kinematics for a single arm using configuration file.

       @param robot_serial: Serial number of the robot.
       @return: 0 on success, negative error code otherwise.
       """
        self._ensure_kin_handle()
        with self._lock:
            return self.dll.FX_L1_Kinematics_InitSingleArm_ByIniConfig(
                self._kin_handle, robot_serial)

    # ---------- tools ----------
    def set_tool(self, arm_idx: int,
                 tool_matrix: Optional[Union[Sequence[float], Sequence[Sequence[float]]]] = None) -> int:
        """Set the tool transformation matrix for an arm.

       @param arm_idx: Arm index (0 or 1).
       @param tool_matrix: 4x4 matrix (as 16-element list or 4x4 nested list). If None, tool is removed.
       @return: 0 on success, negative error code otherwise.
       """
        self._ensure_kin_handle()
        if tool_matrix is None:
            return self.remove_tool(arm_idx)
        if (isinstance(tool_matrix, (list, tuple)) and len(tool_matrix) == 4 and
            isinstance(tool_matrix[0], (list, tuple, Sequence)) and not isinstance(tool_matrix[0], (int, float))):
            flat = []
            for row in tool_matrix:
                if len(row) != 4:
                    raise ValueError("Each row of tool_matrix must have exactly 4 elements")
                flat.extend(row)
            tool_list = flat
        else:
            tool_list = list(tool_matrix)
        if len(tool_list) != 16:
            raise ValueError("tool_matrix must have exactly 16 elements (when flattened) or be a 4x4 nested list")
        tool_array = (c_double * 16)(*tool_list)
        with self._lock:
            return self.dll.FX_L1_Kinematics_SetTool(self._kin_handle, arm_idx, tool_array)

    def remove_tool(self, arm_idx: int) -> int:
        """Remove the tool transformation from the specified arm.

        @param arm_idx: Arm index (0 or 1).
        @return: 0 on success, negative error code otherwise.
        """
        self._ensure_kin_handle()
        with self._lock:
            return self.dll.FX_L1_Kinematics_RemoveTool(self._kin_handle, arm_idx)

    # ---------- Forward kinematics ----------
    def convert_to_8x8_matrix(self, flat_list: List[float]) -> List[List[float]]:
        """Convert a flat list of 64 floats to a 8x8 matrix.

        @param flat_list: List of 64 numbers (int or float).
        @return: 8x8 matrix as a list of 8 rows, each row is a list of 8 floats.
        @raise ValueError: If flat_list does not contain exactly 64 elements.
        """
        if len(flat_list) != 64:
            raise ValueError("list must be 64 elements")
        matrix_8x8 = []
        for i in range(8):
            row_start = i * 8
            row_end = row_start + 8
            matrix_8x8.append(flat_list[row_start:row_end])
        return matrix_8x8

    def distance3(self, p1: List[float], p2: List[float]) -> float:
        """Compute Euclidean distance between two 3D points.

        @param p1: First point as [x, y, z].
        @param p2: Second point as [x, y, z].
        @return: Euclidean distance.
        """
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

    def invert_rigid_matrix4x4(self, mat: List[List[float]]) -> List[List[float]]:
        """Invert a 4x4 rigid transformation matrix.

        Assumes the matrix is of the form [R t; 0 1] where R is orthonormal.
        The inverse is [R^T, -R^T * t; 0, 1].

        @param mat: 4x4 transformation matrix as a list of 4 rows, each with 4 floats.
        @return: Inverse of the matrix as a 4x4 list of lists.
        """
        inv = [[0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 1]]
        for i in range(3):
            for j in range(3):
                inv[i][j] = mat[j][i]
        tx = mat[0][3]
        ty = mat[1][3]
        tz = mat[2][3]
        inv[0][3] = -(inv[0][0] * tx + inv[0][1] * ty + inv[0][2] * tz)
        inv[1][3] = -(inv[1][0] * tx + inv[1][1] * ty + inv[1][2] * tz)
        inv[2][3] = -(inv[2][0] * tx + inv[2][1] * ty + inv[2][2] * tz)
        return inv

    def matrix_multiply(self, lhs: List[List[float]], rhs: List[List[float]],
                        out: Optional[List[List[float]]] = None) -> List[List[float]]:
        """Multiply two 4x4 matrices.

        @param lhs: Left-hand side 4x4 matrix.
        @param rhs: Right-hand side 4x4 matrix.
        @param out: Optional output matrix (4x4) to store the result. If None, a new matrix is created.
        @return: The product matrix (4x4).
        """
        if out is None:
            out = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                total = 0.0
                for k in range(4):
                    total += lhs[i][k] * rhs[k][j]
                out[i][j] = total
        return out

    def forward_kinematics(self, arm_idx: int, joints: List[float]) -> Union[List[List[float]], int]:
        """Compute forward kinematics for the specified arm.

        Calculates the end-effector pose (4x4 transformation matrix) from given joint angles.

        @param arm_idx: Arm index (0 or 1).
        @param joints: Joint positions as a list of 7 floats.
        @return: 4x4 transformation matrix as a list of 4 rows, each a list of 4 floats,
                 or None if the kinematics calculation fails.
        @raise ValueError: If joints does not contain exactly 7 elements.
        """
        if len(joints) != 7:
            raise ValueError("joints must have 7 elements")
        self._ensure_kin_handle()
        j_arr = (c_double * 7)(*joints)
        pose_arr = (c_double * 16)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_ForwardKinematics(self._kin_handle, arm_idx, j_arr, pose_arr)
            if ret == 0:
                matrix = [[pose_arr[i * 4 + j] for j in range(4)] for i in range(4)]
                return matrix
            else:
                return ret

    # ---------- Jacobian ----------
    def jacobian(self, arm_idx: int, joints: List[float]) -> Optional[List[List[float]]]:
        """Compute the Jacobian matrix for the specified arm.

        The Jacobian maps joint velocities to Cartesian end-effector velocities.
        The returned matrix has 6 rows (linear and angular velocities) and 7 columns.

        @param arm_idx: Arm index (0 or 1).
        @param joints: Joint positions as a list of 7 floats.
        @return: A 6x7 Jacobian matrix as a list of 6 rows, each row a list of 7 floats.
                 Returns None if the computation fails.
        @raise ValueError: If joints does not contain exactly 7 elements.
        """
        if len(joints) != 7:
            raise ValueError("joints must have 7 elements")
        self._ensure_kin_handle()
        j_arr = (c_double * 7)(*joints)
        jac_arr = (c_double * 42)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_Jacobian(self._kin_handle, arm_idx, j_arr, jac_arr)
            if ret == 0:
                jacobian = [[jac_arr[i * 7 + j] for j in range(7)] for i in range(6)]
                return jacobian
            else:
                return ret

    # ---------- Inverse kinematics ----------
    def inverse_kinematics(self, arm_idx: int, ik_structure_params: FX_InvKineSolverParams) -> Optional[
        FX_InvKineSolverParams]:
        """Solve inverse kinematics for the specified arm.

        The input parameter structure contains target TCP pose, reference joint angles,
        and solver settings. On success, the structure is updated with the solution(s).

        @param arm_idx: Arm index (0 or 1).
        @param ik_structure_params: FX_InvKineSolverParams structure with input fields filled.
        @return: The same structure with output fields populated (solutions), or None if IK fails.
        """
        self._ensure_kin_handle()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_InverseKinematics(self._kin_handle, arm_idx, byref(ik_structure_params))
            if ret == 0:
                return ik_structure_params
            else:
                return ret

    # ---------- Body kinematics ----------
    def set_skye_body_condition(self,
                                std_body: List[float], k_body: List[float],
                                std_left_len: float, k_left: float,
                                std_right_len: float, k_right: float) -> int:
        """Set body kinematics parameters for the Skye robot (dual‑arm torso).

        Configures the standard positions and stiffness values for the body joints
        that affect the shoulder positions.

        @param std_body: Standard body position [x, y, z] (3 elements).
        @param k_body: Body stiffness coefficients [kx, ky, kz] (3 elements).
        @param std_left_len: Standard length of the left arm link.
        @param k_left: Stiffness of the left arm joint.
        @param std_right_len: Standard length of the right arm link.
        @param k_right: Stiffness of the right arm joint.
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If std_body or k_body does not have exactly 3 elements.
        """
        if len(std_body) != 3 or len(k_body) != 3:
            raise ValueError("std_body and k_body must have 3 elements each")
        self._ensure_kin_handle()
        std_arr = (c_double * 3)(*std_body)
        k_arr = (c_double * 3)(*k_body)
        with self._lock:
            return self.dll.FX_L1_Kinematics_SetSkyeBodyCondition(
                self._kin_handle, std_arr, k_arr,
                std_left_len, k_left, std_right_len, k_right
            )

    def skye_body_forward_kinematics(self, body_joints: List[float]) -> Union[
        Tuple[List[List[float]], List[List[float]]], int]:
        """Compute forward kinematics for the Skye robot body.

        Given the three body joint values, returns the 4x4 transformation matrices
        for the left and right shoulders.

        @param body_joints: Body joint positions as a list of 3 floats.
        @return: On success, a tuple of two 4x4 matrices (left_shoulder, right_shoulder),
                 each as a list of 4 rows, each row a list of 4 floats.
                 Negative error code otherwise.
        @raise ValueError: If body_joints does not contain exactly 3 elements.
        """
        if len(body_joints) != 3:
            raise ValueError("body_joints must have 3 elements")
        self._ensure_kin_handle()
        j_arr = (c_double * 3)(*body_joints)
        left_pose = (c_double * 16)()
        right_pose = (c_double * 16)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_SkyeBodyForwardKinematics(self._kin_handle, j_arr, left_pose, right_pose)
            if ret == 0:
                left_mat = [[left_pose[i * 4 + j] for j in range(4)] for i in range(4)]
                right_mat = [[right_pose[i * 4 + j] for j in range(4)] for i in range(4)]
                return (left_mat, right_mat)
            else:
                return ret

    def skye_body_inverse_kinematics(self, left_tcp: List[float], right_tcp: List[float]) -> Union[List[float], int]:
        """Compute body joint values from dual-arm TCP positions.

        Given the desired left and right TCP positions (in base frame), compute
        the required body joint angles.

        @param left_tcp: Left TCP position [x, y, z] (3 elements).
        @param right_tcp: Right TCP position [x, y, z] (3 elements).
        @return: Body joint positions as a list of 3 floats.
                 Negative error code otherwise.
        @raise ValueError: If either input list does not have exactly 3 elements.
        """
        if len(left_tcp) != 3 or len(right_tcp) != 3:
            raise ValueError("left_tcp and right_tcp must have 3 elements each")
        self._ensure_kin_handle()
        left_arr = (c_double * 3)(*left_tcp)
        right_arr = (c_double * 3)(*right_tcp)
        body_joints = (c_double * 3)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_SkyeBodyInverseKinematics(self._kin_handle, left_arr, right_arr,
                                                                      body_joints)
            if ret == 0:
                return [body_joints[i] for i in range(3)]
            else:
                return ret

    def skye_body_inverse_kinematics_with_ref(self, ref_body_joints: List[float],
                                              left_tcp: List[float], right_tcp: List[float]) -> Union[List[float], int]:
        """Compute body joint values with a reference body pose.

        This version uses a reference body joint configuration to help resolve
        redundancy when solving for body joints from TCP positions.

        @param ref_body_joints: Reference body joint angles (3 elements).
        @param left_tcp: Desired left TCP position [x, y, z] (3 elements).
        @param right_tcp: Desired right TCP position [x, y, z] (3 elements).
        @return: Computed body joint positions as a list of 3 floats.
                 Negative error code otherwise.
        @raise ValueError: If any input list does not have exactly 3 elements.
        """
        if len(ref_body_joints) != 3 or len(left_tcp) != 3 or len(right_tcp) != 3:
            raise ValueError("All input arrays must have 3 elements")
        self._ensure_kin_handle()
        ref_arr = (c_double * 3)(*ref_body_joints)
        left_arr = (c_double * 3)(*left_tcp)
        right_arr = (c_double * 3)(*right_tcp)
        body_joints = (c_double * 3)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_SkyeBodyInverseKinematicsWithRef(
                self._kin_handle, ref_arr, left_arr, right_arr, body_joints
            )
            if ret == 0:
                return [body_joints[i] for i in range(3)]
            else:
                return ret

    # ---------- Motion planning ----------
    def plan_joints(self, arm_idx: int,
                    start_joints: List[float], end_joints: List[float],
                    vel: float, acc: float, freq: int
                    ) -> Union[Tuple[ctypes.Array, int], int]:
        """Plan a joint‑space point‑to‑point motion.

        @param arm_idx: Arm index (0 or 1).
        @param start_joints: Starting joint positions (7 elements).
        @param end_joints: Ending joint positions (7 elements).
        @param vel: Velocity scale factor (0‑100 % or actual units).
        @param acc: Acceleration scale factor.
        @param freq: Planning frequency in Hz.
        @return: On success, a tuple (point_set, num_points) where point_set is a ctypes array
                 of size max_point*7 containing the trajectory points, and num_points is the
                 actual number of generated points.
                 Negative error code otherwise.
        @raise ValueError: If start_joints or end_joints does not contain exactly 7 elements.
        """
        if len(start_joints) != 7 or len(end_joints) != 7:
            raise ValueError("start_joints and end_joints must have 7 elements")
        self._ensure_kin_handle()
        start_arr = (c_double * 7)(*start_joints)
        end_arr = (c_double * 7)(*end_joints)
        total_doubles = self.max_point * 7
        point_set = (c_double * total_doubles)()
        num = c_int(0)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanJointMove(
                self._kin_handle, arm_idx, start_arr, end_arr,
                c_double(vel), c_double(acc), c_int(freq),
                point_set, byref(num)
            )
        if ret == 0:
            return point_set, num.value
        else:
            return ret

    def plan_linear(self, arm_idx: int,
                    start_xyzabc: List[float], end_xyzabc: List[float],
                    ref_joints: List[float],
                    vel: float, acc: float, freq: int) -> Union[Tuple[ctypes.Array, int], int]:
        """Plan a linear Cartesian (MoveL) trajectory.

        @param arm_idx: Arm index (0 or 1).
        @param start_xyzabc: Starting pose [x, y, z, roll, pitch, yaw] (6 elements).
        @param end_xyzabc: Ending pose (6 elements).
        @param ref_joints: Reference joint positions for inverse kinematics (7 elements).
        @param vel: Velocity scale factor.
        @param acc: Acceleration scale factor.
        @param freq: Planning frequency in Hz.
        @return: On success, a tuple (point_set, num_points) where point_set is a ctypes array
                 of size max_point*7 containing the joint‑space trajectory points, and num_points
                 is the number of generated points.
                 Negative error code otherwise.
        @raise ValueError: If start_xyzabc or end_xyzabc does not have 6 elements, or
                           if ref_joints does not have 7 elements.
        """
        if len(start_xyzabc) != 6 or len(end_xyzabc) != 6:
            raise ValueError("start_xyzabc and end_xyzabc must have 6 elements")
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have 7 elements")
        self._ensure_kin_handle()

        start_arr = (c_double * 6)(*start_xyzabc)
        end_arr = (c_double * 6)(*end_xyzabc)
        ref_arr = (c_double * 7)(*ref_joints)

        total_doubles = self.max_point * 7
        point_set = (c_double * total_doubles)()
        num = c_int(0)

        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMove(
                self._kin_handle, arm_idx,
                start_arr, end_arr, ref_arr,
                c_double(vel), c_double(acc), c_int(freq),
                point_set, byref(num)
            )
            if ret == 0:
                return point_set, num.value
            else:
                return ret

    def plan_linear_keep_joints(self, arm_idx: int,
                                start_joints: List[float], end_joints: List[float],
                                vel: float, acc: float, freq: int) -> Union[Tuple[ctypes.Array, int], int]:
        """Plan a linear Cartesian trajectory while maintaining joint configuration.

        This planner interpolates the end‑effector in Cartesian space but uses the given
        start and end joint positions to resolve redundancy; the path may not be strictly
        linear in joint space.

        @param arm_idx: Arm index (0 or 1).
        @param start_joints: Starting joint positions (7 elements).
        @param end_joints: Ending joint positions (7 elements).
        @param vel: Velocity scale factor.
        @param acc: Acceleration scale factor.
        @param freq: Planning frequency in Hz.
        @return: On success, a tuple (point_set, num_points) as in plan_joints().
                 Negative error code otherwise.
        @raise ValueError: If start_joints or end_joints does not have exactly 7 elements.
        """
        if len(start_joints) != 7 or len(end_joints) != 7:
            raise ValueError("start_joints and end_joints must have 7 elements")
        self._ensure_kin_handle()
        start_arr = (c_double * 7)(*start_joints)
        end_arr = (c_double * 7)(*end_joints)

        total_doubles = self.max_point * 7
        point_set = (c_double * total_doubles)()
        num = c_int(0)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMoveKeepJoints(
                self._kin_handle, arm_idx, start_arr, end_arr,
                c_double(vel), c_double(acc), c_int(freq),
                point_set, byref(num)
            )
            if ret == 0:
                return point_set, num.value
            else:
                return ret

    # Multi-segment linear motion planning
    def plan_linear_multi_points_set_start(self, arm_idx: int,
                                           ref_joints: List[float],
                                           start_xyzabc: List[float], end_xyzabc: List[float],
                                           allow_range: float, zsp_type: int,
                                           zsp_para: List[float],
                                           vel: float, acc: float, freq: int) -> int:
        """Start a multi‑segment Cartesian MoveL planning sequence.

        This function initialises the multi‑segment planner with the first segment.
        Subsequent segments are added using plan_linear_multi_points_set_next().
        After all segments are defined, use plan_linear_multi_points_get_points() to
        retrieve the final trajectory.

        @param arm_idx: Arm index (0 or 1).
        @param ref_joints: Reference joint positions for IK (7 elements).
        @param start_xyzabc: Starting pose of the first segment (6 elements).
        @param end_xyzabc: Ending pose of the first segment (6 elements).
        @param allow_range: Allowed deviation tolerance.
        @param zsp_type: Type of Z‑S‑P (zero‑space posture) parameterisation.
        @param zsp_para: ZSP parameters (6 elements).
        @param vel: Velocity scale factor.
        @param acc: Acceleration scale factor.
        @param freq: Planning frequency in Hz.
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If ref_joints does not have 7 elements, or any pose / zsp_para
                           does not have the required number of elements.
        """
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have 7 elements")
        if len(start_xyzabc) != 6 or len(end_xyzabc) != 6:
            raise ValueError("start_xyzabc and end_xyzabc must have 6 elements")
        if len(zsp_para) != 6:
            raise ValueError("zsp_para must have 6 elements")
        self._ensure_kin_handle()

        ref_arr = (c_double * 7)(*ref_joints)
        start_arr = (c_double * 6)(*start_xyzabc)
        end_arr = (c_double * 6)(*end_xyzabc)
        zsp_arr = (c_double * 6)(*zsp_para)

        with self._lock:
            return self.dll.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(
                self._kin_handle, arm_idx,
                ref_arr, start_arr, end_arr,
                c_double(allow_range), c_int32(zsp_type), zsp_arr,
                c_double(vel), c_double(acc), c_int32(freq)
            )

    def plan_linear_multi_points_set_next(self, arm_idx: int,
                                          next_xyzabc: List[float],
                                          allow_range: float, zsp_type: int,
                                          zsp_para: List[float],
                                          vel: float, acc: float) -> int:
        """Add a subsequent segment to the multi‑segment Cartesian plan.

        @param arm_idx: Arm index (0 or 1).
        @param next_xyzabc: Ending pose of this segment (6 elements). The starting pose
                            is automatically taken as the end of the previous segment.
        @param allow_range: Allowed deviation tolerance.
        @param zsp_type: ZSP type.
        @param zsp_para: ZSP parameters (6 elements).
        @param vel: Velocity scale factor (may be overridden from start).
        @param acc: Acceleration scale factor.
        @return: 0 on success, negative error code otherwise.
        @raise ValueError: If next_xyzabc or zsp_para does not have exactly 6 elements.
        """
        if len(next_xyzabc) != 6:
            raise ValueError("next_xyzabc must have 6 elements")
        if len(zsp_para) != 6:
            raise ValueError("zsp_para must have 6 elements")
        self._ensure_kin_handle()
        next_arr = (c_double * 6)(*next_xyzabc)
        zsp_arr = (c_double * 6)(*zsp_para)
        with self._lock:
            return self.dll.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(
                self._kin_handle, arm_idx, next_arr,
                c_double(allow_range), c_int32(zsp_type), zsp_arr,
                c_double(vel), c_double(acc)
            )

    def plan_linear_multi_points_get_points(self) -> Union[Tuple[ctypes.Array, int], int]:
        """Retrieve the generated trajectory points from the multi‑segment planner.

        After defining segments with plan_linear_multi_points_set_start() and
        plan_linear_multi_points_set_next(), call this function to obtain the final
        joint‑space trajectory.

        @return: On success, a tuple (point_set, num_points) where:
                 - point_set: ctypes array of (max_point * 7) doubles containing joint
                   positions for all points (packed consecutively).
                 - num_points: actual number of trajectory points (<= max_point).
                 Negative error code otherwise.
        """
        self._ensure_kin_handle()
        total_doubles = self.max_point * 7
        point_set = (c_double * total_doubles)()
        num = c_int(0)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPoints(
                self._kin_handle, point_set, byref(num)
            )
            if ret == 0:
                return point_set, num.value
            else:
                return ret

    def plan_linear_synchronous(self, arms_structure_params: ArmsSynchronousPlanningParams) -> Union[
        Tuple[ctypes.Array, ctypes.Array, int], int]:
        """Plan a synchronised dual‑arm linear path with a fixed body.

        The planning considers both arms simultaneously to ensure coordination.
        The generated joint trajectories have the same number of points for both arms.

        @param arms_structure_params: An ArmsSynchronousPlanningParams structure containing
                                      start/end poses, reference joints, ZSP parameters,
                                      velocity, acceleration, frequency and sync type.
        @return: On success, a tuple (arm0_point_set, arm1_point_set, num_points) where:
                 - arm0_point_set: ctypes array of (max_point * 7) doubles for arm 0.
                 - arm1_point_set: ctypes array of (max_point * 7) doubles for arm 1.
                 - num_points: number of trajectory points (same for both arms).
                 Negative error code otherwise.
        """
        self._ensure_kin_handle()
        total_doubles = self.max_point * 7
        arm0_point_set = (c_double * total_doubles)()
        arm1_point_set = (c_double * total_doubles)()
        num = c_int(0)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_ArmsSynchronousPlanning(
                self._kin_handle, byref(arms_structure_params), arm0_point_set, arm1_point_set, byref(num))
            if ret == 0:
                return arm0_point_set, arm1_point_set, num.value
            else:
                return ret

    def dynamics_identification(self, robot_type: int, file_path: str) -> Union[List[float], int]:
        """Identify dynamic parameters from a sampled data file.

        The sampled data must be collected from a suitable excitation trajectory.
        The function returns a list of identified dynamic parameters or an error message.

        @param robot_type: Robot type code (see robot_type_map).
        @param file_path: Path to the sampled data file (CSV or similar format).
        @return: On success: a list of 10 floats in the order:
                 [mass, com_x, com_y, com_z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
                 Negative error code otherwise.
        """
        self._ensure_kin_handle()
        mass = c_double()
        mr = (c_double * 3)()
        inertia = (c_double * 6)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_DynamicsIdentification(
                robot_type, file_path.encode('utf-8'), byref(mass), byref(mr), byref(inertia)
            )
        if ret == 0:
            dyn_para = []
            m_val = mass.value
            mcp_list = [mr[i] for i in range(3)]
            ii_list = [inertia[i] for i in range(6)]
            dyn_para.append(m_val)
            for i in mcp_list:
                dyn_para.append(i)
            dyn_para.append(ii_list[0])
            dyn_para.append(ii_list[3])
            dyn_para.append(ii_list[4])
            dyn_para.append(ii_list[1])
            dyn_para.append(ii_list[5])
            dyn_para.append(ii_list[2])
            return dyn_para
        else:
            return ret

    # ---------- Helper conversions ----------
    def xyzabc2matrix(self, xyzabc: List[float]) -> List[List[float]]:
        """Convert an XYZABC pose to a 4x4 transformation matrix.

        @param xyzabc: A list of 6 floats: [x, y, z, roll, pitch, yaw] (angles in degrees).
        @return: A 4x4 transformation matrix as a list of 4 rows, each row a list of 4 floats.
        @raise ValueError: If xyzabc does not contain exactly 6 elements.
        """
        if len(xyzabc) != 6:
            raise ValueError("xyzabc must have 6 elements")
        arr_in = (c_double * 6)(*xyzabc)
        arr_out = (c_double * 16)()
        with self._lock:
            self.dll.FX_L1_XYZABC2Matrix(arr_in, arr_out)
        matrix = [[arr_out[i * 4 + j] for j in range(4)] for i in range(4)]
        return matrix

    def mat4x4_to_mat1x16(self, pose_mat: List[List[float]]) -> List[float]:
        """Flatten a 4x4 matrix into a 16‑element list (row‑major order).

        @param pose_mat: A 4x4 transformation matrix as a list of 4 rows, each row a list of 4 floats.
        @return: A flat list of 16 floats, concatenated row by row.
        """
        matrix_data = []
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])
        return matrix_data

    def matrix2xyzabc(self, matrix: List[List[float]]) -> List[float]:
        """Convert a 4x4 transformation matrix to an XYZABC pose.

        @param matrix: A 4x4 transformation matrix as a list of 4 rows, each row a list of 4 floats.
        @return: A list of 6 floats: [x, y, z, roll, pitch, yaw] (angles in degrees).
        @raise ValueError: If the flattened matrix does not have exactly 16 elements.
        """
        mat16 = self.mat4x4_to_mat1x16(matrix)
        if len(mat16) != 16:
            raise ValueError("matrix must have 16 elements")
        arr_in = (c_double * 16)(*mat16)
        arr_out = (c_double * 6)()
        with self._lock:
            self.dll.FX_L1_Matrix2XYZABC(arr_in, arr_out)
        return [arr_out[i] for i in range(6)]

    def check_sequences_approx_equal(self, seq1, seq2, tolerance=0.1, expected_len=None):
        """Check if two sequences of numbers are approximately equal within tolerance.

        @param seq1: First numeric sequence (e.g., list, tuple).
        @param seq2: Second numeric sequence.
        @param tolerance: Maximum allowed absolute difference between corresponding elements.
        @param expected_len: Expected length of each sequence. If provided, both sequences must
                             have exactly this length; if None, only requires equal lengths.
        @return: True if both sequences are non‑empty, meet the length requirement, and all
                 absolute differences are less than tolerance; otherwise False.
        """
        if not seq1 or not seq2:
            return False
        if expected_len is not None:
            if len(seq1) != expected_len or len(seq2) != expected_len:
                return False
        else:
            if len(seq1) != len(seq2):
                return False
        return all(abs(a - b) < tolerance for a, b in zip(seq1, seq2))

    # ---------- Cleanup ----------
    def cleanup(self):
        """Clean up all resources including kinematics handle."""
        self._destroy_kin_handle()

    @staticmethod
    def _get_operate_error_msg(error_code: int) -> str:
        errors = {
            0: "Success",
            -1: "Operation failed",
            -2: "Link failed",
            -3: "Link no response",
            -4: "Version incompatible",
            -5: "Invalid input argument",
            -6: "Invalid condition",
            -7: "Send file failed",
            -8: "Receive file failed",
            -9: "Invalid object",
            -10: "Operation timeout",
            -11: "Set parameter failed",
            -12: "Get parameter failed",
            -13: "Save parameter failed",
            -14: "Invalid terminal",
            -15: "Invalid parameter setting",
            -16: "Communication wait ready timeout",
            -17: "Communication send failed",
            -18: "Communication wait reply timeout",
            -19: "Format command failed",
            -20: "Invalid robot type to support or robot unlinked",
            -21: "Invalid hand type to support",
            -22: "Controller has already linked by other client",
            -23: "Invalid thread id",

            -1000: "Kinematics context or arm environment is not initialized",
            -1001: "Kinematics environment initialization failed",
            -1002: "Tool transform update failed",
            -1010: "Inverse kinematics target is unreachable or singular",
            -1011: "Inverse kinematics result exceeds joint limits",
            -1020: "Motion planning failed for an unspecified kinematics reason",
            -1021: "Planned motion exceeds joint limits",
            -1022: "Planned Cartesian target is unreachable",
            -1023: "Planned trajectory exceeds the maximum point count",
            -1024: "Dual-arm synchronized plan produced mismatched point counts",
            -1030: "Dynamics identification failed",
            -1099: "Internal kinematics or planning error",
        }
        return errors.get(error_code, f"Unknown error code: {error_code}")

    def _operate_check(self, ret: int, raise_error=False) -> Union[int, str]:
        func_name = inspect.currentframe().f_back.f_code.co_name
        if ret == 0:
            return 0
        msg = self._get_operate_error_msg(ret)
        print(f"[Warning] {func_name} failed: {msg}")
        if raise_error:
            raise RuntimeError(msg)
        return f"{ret}: {msg}"


# ==================== RobotDataManager ====================
class RobotDataManager:
    """Background thread manager for continuously fetching RT and SG data."""

    def __init__(self, robot):
        """Initialize the data manager and start background threads.

        @param robot: GentoRobot instance.
        """
        self.robot = robot
        self._latest_rt = None
        self._latest_sg = None
        self._running = True
        self._lock = threading.Lock()
        self._rt_thread = threading.Thread(target=self._fetch_rt, daemon=True)
        self._sg_thread = threading.Thread(target=self._fetch_sg, daemon=True)
        self._rt_thread.start()
        self._sg_thread.start()

    def _fetch_rt(self):
        """Background thread to fetch real-time data."""
        while self._running:
            try:
                data = self.robot.get_rt_dict()
                with self._lock:
                    self._latest_rt = data
            except:
                with self._lock:
                    self._latest_rt = None
            time.sleep(0.001)

    def _fetch_sg(self):
        """Background thread to fetch static data."""
        while self._running:
            try:
                data = self.robot.get_sg_dict()
                with self._lock:
                    self._latest_sg = data
            except:
                with self._lock:
                    self._latest_sg = None
            time.sleep(0.002)

    @property
    def latest_rt(self):
        """Get the latest real-time data dictionary (thread-safe)."""
        with self._lock:
            return self._latest_rt

    @property
    def latest_sg(self):
        """Get the latest static data dictionary (thread-safe)."""
        with self._lock:
            return self._latest_sg

    def stop(self):
        """Stop the background threads."""
        self._running = False
        self._rt_thread.join(timeout=1.0)
        self._sg_thread.join(timeout=1.0)


# ==================== Example usage ====================
if __name__ == "__main__":
    robot = GentoRobot()
    sdk_version = robot.get_sdk_version()
    print(f'sdk version:{sdk_version}')
    ret = robot.link(6, 6, 7, 190)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        exit(-1)
    else:
        print(f"connected, system version:{robot.sys_version}, SDK version:{robot.sdk_version}")

    print(f" ==== robot:{robot.get_robot_type()}")

    rt_dict = robot.get_rt_dict()
    print("RT frame serial:", rt_dict["frame_serial"])
    print("Arm0 joint positions:", rt_dict["arms"][0]["fb"]["fb_pos"])
    print("Arm1 joint positions:", rt_dict["arms"][1]["fb"]["fb_pos"])
    print("head joint positions:", rt_dict["head"]["fb_pos"])
    print("body joint positions:", rt_dict["body"]["fb_pos"])
    print("lift joint positions:", rt_dict["lift"]["fb_pos"])

    sg_dict = robot.get_sg_dict()
    print("Arm0 low speed flag:", sg_dict["arms"][0]['get']["low_speed_flag"])
    print("TipDI:", sg_dict["arms"][0]['get']["tip_di"])
    print("TrajState:", sg_dict["arms"][0]['get']["traj_state"])

    robot.unlink()

    time.sleep(2)

    ret = robot.link(6, 6, 7, 190)
    if ret <= 0:
        print(f"Failed to connect, error code: {ret}")
        exit(1)

    rt_dict = robot.get_rt_dict()
    print("RT frame serial:", rt_dict["frame_serial"])
    print("Arm0 joint positions:", rt_dict["arms"][0]["fb"]["fb_pos"])
    print("Arm1 joint positions:", rt_dict["arms"][1]["fb"]["fb_pos"])
    print("head joint positions:", rt_dict["head"]["fb_pos"])
    print("body joint positions:", rt_dict["body"]["fb_pos"])
    print("lift joint positions:", rt_dict["lift"]["fb_pos"])

    sg_dict = robot.get_sg_dict()
    print("Arm0 low speed flag:", sg_dict["arms"][0]['get']["low_speed_flag"])
    print("TipDI:", sg_dict["arms"][0]['get']["tip_di"])
    print("TrajState:", sg_dict["arms"][0]['get']["traj_state"])

    robot.unlink()
