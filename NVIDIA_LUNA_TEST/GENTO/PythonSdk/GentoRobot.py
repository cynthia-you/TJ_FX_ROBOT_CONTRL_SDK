import ctypes
from ctypes import *
import threading
import os
from typing import List, Tuple, Optional
import time

current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)

# ==================== Type aliases ====================
FX_UCHAR = c_ubyte
FX_CHAR = c_byte
FX_UINT8 = c_ubyte
FX_INT32 = c_int32
FX_UINT32 = c_uint32
FX_INT16 = c_int16
FX_UINT16 = c_uint16
FX_FLOAT = c_float
FX_DOUBLE = c_double
FX_BOOL = c_ubyte
FX_VOID = None

# ==================== Enumerations ====================
class FXObjType:
    OBJ_ARM0 = 0
    OBJ_ARM1 = 1
    OBJ_HEAD = 2
    OBJ_BODY = 3
    OBJ_LIFT = 4

class FXObjMask:
    OBJ_ARM0_FLAG = 1 << 0
    OBJ_ARM1_FLAG = 1 << 1
    OBJ_HEAD_FLAG = 1 << 2
    OBJ_BODY_FLAG = 1 << 3
    OBJ_LIFT_FLAG = 1 << 4
    OBJ_ALL_FLAG = (OBJ_ARM0_FLAG | OBJ_ARM1_FLAG | OBJ_HEAD_FLAG | OBJ_BODY_FLAG | OBJ_LIFT_FLAG)

class FXChnType:
    CHN_CANFD = 1
    CHN_485A = 2
    CHN_485B = 3

class FXTerminalType:
    TERMINAL_ARM0 = 0
    TERMINAL_ARM1 = 1

class FXStateType:
    STATE_IDLE = 0
    STATE_POSITION = 1
    STATE_TORQUE = 2
    STATE_RELEASE = 3
    STATE_ERROR = 100

# ==================== RT/SG structures====================
class StateCtr(Structure):
    _fields_ = [
        ("m_CurState", FX_UINT16),
        ("m_CmdState", FX_UINT16),
        ("m_ERRCode", FX_UINT32),
    ]

class ARM_OUT(Structure):
    _fields_ = [
        ("m_ARM_FBK_Joint_Pos", FX_FLOAT * 7),
        ("m_ARM_FBK_Joint_Vel", FX_FLOAT * 7),
        ("m_ARM_FBK_Joint_Cmd", FX_FLOAT * 7),
        ("m_ARM_FBK_Joint_SensorTor", FX_FLOAT * 7),
        ("m_ARM_FBK_Joint_ExternalTorEst", FX_FLOAT * 7),
        ("m_ARM_FBK_Base_FNEst", FX_FLOAT * 6),
        ("m_ARM_FBK_Base_Gyro", FX_FLOAT * 6),
        ("m_ARM_FBK_Flange_FTSensor", FX_FLOAT * 6),
    ]

class ARM_IN(Structure):
    _fields_ = [
        ("m_ARM_CMD_Joint_Tor", FX_FLOAT * 7),
        ("m_ARM_CMD_Joint_Pos", FX_FLOAT * 7),
        ("m_ARM_CMD_Ctrl_DragType", FX_INT16),
        ("m_ARM_CMD_Ctrl_ForceType", FX_INT16),
        ("m_ARM_CMD_Ctrl_ForceDir", FX_FLOAT * 5),
        ("m_ARM_CMD_Ctrl_TorqueDir", FX_FLOAT * 5),
    ]

class ARM_RT(Structure):
    _fields_ = [
        ("m_ARM_State", StateCtr),
        ("m_ARM_IN", ARM_IN),
        ("m_ARM_OUT", ARM_OUT),
    ]

class HEAD_OUT(Structure):
    _fields_ = [("m_HEAD_FBK_Joint_Pos", FX_FLOAT * 3)]

class HEAD_IN(Structure):
    _fields_ = [("m_HEAD_CMD_Joint_Pos", FX_FLOAT * 3)]

class HEAD_RT(Structure):
    _fields_ = [
        ("m_HEAD_State", StateCtr),
        ("m_HEAD_IN", HEAD_IN),
        ("m_HEAD_OUT", HEAD_OUT),
    ]

class BODY_OUT(Structure):
    _fields_ = [
        ("m_BODY_FBK_Joint_Pos", FX_FLOAT * 6),
        ("m_BODY_FBK_Joint_Vel", FX_FLOAT * 6),
        ("m_BODY_FBK_Joint_SensorTor", FX_FLOAT * 6),
        ("m_BODY_FBK_Base_Gyro", FX_FLOAT * 6),
    ]

class BODY_IN(Structure):
    _fields_ = [
        ("m_BODY_CMD_Ctrl_Type", FX_INT32),
        ("m_BODY_CMD_Joint_Pos", FX_FLOAT * 6),
    ]

class BODY_RT(Structure):
    _fields_ = [
        ("m_BODY_State", StateCtr),
        ("m_BODY_IN", BODY_IN),
        ("m_BODY_OUT", BODY_OUT),
    ]

class LIFT_OUT(Structure):
    _fields_ = [("m_LIFT_FBK_Joint_Pos", FX_FLOAT * 2)]

class LIFT_IN(Structure):
    _fields_ = [("m_LIFT_CMD_Joint_Pos", FX_FLOAT * 2)]

class LIFT_RT(Structure):
    _fields_ = [
        ("m_LIFT_State", StateCtr),
        ("m_LIFT_IN", LIFT_IN),
        ("m_LIFT_OUT", LIFT_OUT),
    ]

class HAND_OUT(Structure):
    _fields_ = [
        ("m_HAND_FBK_Joint_Pos", FX_INT16 * 24),
        ("m_HAND_FBK_Joint_Tor", FX_INT16 * 24),
        ("m_HAND_FBK_F", FX_INT16 * 24),
    ]

class HAND_IN(Structure):
    _fields_ = [
        ("m_HAND_CMD_Joint_Pos", FX_INT16 * 24),
        ("m_HAND_CMD_Joint_Tor", FX_INT16 * 24),
    ]

class HAND_RT(Structure):
    _fields_ = [
        ("m_HAND_State", StateCtr),
        ("m_HAND_IN", HAND_IN),
        ("m_HAND_OUT", HAND_OUT),
    ]

class ROBOT_RT(Structure):
    _fields_ = [
        ("m_RT_FrameSerial", FX_INT32),
        ("m_HEAD", HEAD_RT),
        ("m_ARMS", ARM_RT * 2),
        ("m_HANDS", HAND_RT * 2),
        ("m_BODY", BODY_RT),
        ("m_LIFT", LIFT_RT),
        ("wait_serial", FX_UCHAR),
        ("pad", FX_UCHAR * 3),
    ]

class ARM_SET(Structure):
    _fields_ = [
        ("m_ARM_Ctrl_ImpType", FX_INT32),
        ("m_ARM_Ctrl_VelRatio", FX_FLOAT),
        ("m_ARM_Ctrl_AccRatio", FX_FLOAT),
        ("m_ARM_Ctrl_JointK", FX_FLOAT * 7),
        ("m_ARM_Ctrl_JointD", FX_FLOAT * 7),
        ("m_ARM_Ctrl_CartK", FX_FLOAT * 7),
        ("m_ARM_Ctrl_CartD", FX_FLOAT * 7),
        ("m_ARM_Ctrl_ToolKine", FX_FLOAT * 6),
        ("m_ARM_Ctrl_ToolDyna", FX_FLOAT * 10),
        ("m_ARM_SET_SetTags", FX_BOOL * 16),
        ("m_ARM_SET_UpDateTag", FX_BOOL * 16),
    ]

class ARM_GET(Structure):
    _fields_ = [
        ("m_ARM_FBK_Joint_Tor", FX_FLOAT * 7),
        ("m_ARM_FBK_Joint_ExtPos", FX_FLOAT * 7),
        ("m_ARM_FBK_Flange_DI", FX_CHAR),
        ("m_ARM_FBK_LowSpdFlag", FX_CHAR),
        ("m_ARM_FBK_TrajState", FX_CHAR),
        ("m_pad", FX_CHAR * 1),
    ]

class ARM_SG(Structure):
    _fields_ = [
        ("m_ARM_SET", ARM_SET),
        ("m_ARM_GET", ARM_GET),
    ]

class HEAD_SET(Structure):
    _fields_ = [
        ("m_HEAD_Ctrl_VelRatio", FX_FLOAT),
        ("m_HEAD_Ctrl_AccRatio", FX_FLOAT),
        ("m_HEAD_SET_SetTags", FX_BOOL * 4),
        ("m_HEAD_SET_UpDateTag", FX_BOOL * 4),
    ]

class HEAD_GET(Structure):
    _fields_ = [
        ("m_HEAD_FBK_Joint_Tor", FX_FLOAT * 3),
        ("m_HEAD_FBK_Joint_ExtPos", FX_FLOAT * 3),
    ]

class HEAD_SG(Structure):
    _fields_ = [
        ("m_HEAD_SET", HEAD_SET),
        ("m_HEAD_GET", HEAD_GET),
    ]

class BODY_SET(Structure):
    _fields_ = [
        ("m_BODY_Ctrl_VelRatio", FX_FLOAT),
        ("m_BODY_Ctrl_AccRatio", FX_FLOAT),
        ("m_BODY_Ctrl_PDK", FX_FLOAT * 6),
        ("m_BODY_Ctrl_PDD", FX_FLOAT * 6),
        ("m_BODY_SET_SetTags", FX_BOOL * 6),
        ("m_BODY_SET_UpDateTag", FX_BOOL * 6),
    ]

class BODY_GET(Structure):
    _fields_ = [
        ("m_BODY_FBK_Joint_Tor", FX_FLOAT * 6),
        ("m_BODY_FBK_Joint_ExtPos", FX_FLOAT * 6),
        ("m_BODY_FBK_TrajState", FX_CHAR),
        ("m_pad", FX_CHAR * 3),
    ]

class BODY_SG(Structure):
    _fields_ = [
        ("m_BODY_SET", BODY_SET),
        ("m_BODY_GET", BODY_GET),
    ]

class LIFT_SET(Structure):
    _fields_ = [
        ("m_LIFT_Ctrl_VelRatio", FX_FLOAT),
        ("m_LIFT_Ctrl_AccRatio", FX_FLOAT),
        ("m_LIFT_SET_SetTags", FX_BOOL * 4),
        ("m_LIFT_SET_UpDateTag", FX_BOOL * 4),
    ]

class LIFT_GET(Structure):
    _fields_ = [
        ("m_LIFT_FBK_Joint_Tor", FX_FLOAT * 2),
        ("m_LIFT_FBK_TrajState", FX_CHAR),
        ("m_pad", FX_CHAR * 3),
    ]

class LIFT_SG(Structure):
    _fields_ = [
        ("m_LIFT_SET", LIFT_SET),
        ("m_LIFT_GET", LIFT_GET),
    ]

class HAND_SET(Structure):
    _fields_ = [
        ("m_HAND_Ctrl_KP", FX_INT16 * 24),
        ("m_HAND_Ctrl_KD", FX_INT16 * 24),
        ("m_HAND_Ctrl_Vel", FX_INT16 * 24),
    ]

class HAND_SG(Structure):
    _fields_ = [
        ("m_HAND_SET", HAND_SET),
    ]

class OP_SET(Structure):
    _fields_ = [
        ("m_OpIns", FX_INT16),
        ("m_OpValueS", FX_CHAR * 30),
        ("m_OpValueI", FX_INT32),
        ("m_OpValueF", FX_FLOAT),
        ("m_OpCmdSerial", FX_INT16),
        ("m_OpRetSerial", FX_INT16),
    ]

class ROBOT_SG(Structure):
    _fields_ = [
        ("m_RT_FrameSerial", FX_INT32),
        ("m_HEAD", HEAD_SG),
        ("m_ARMS", ARM_SG * 2),
        ("m_HANDS", HAND_SG * 2),
        ("m_BODY", BODY_SG),
        ("m_LIFT", LIFT_SG),
        ("m_OP_SET", OP_SET),
    ]

# ==================== Motion planning structures ====================
class FX_InvKineSolverParams(Structure):
    _fields_ = [
        ("robot_serial", FX_INT32),
        ("target_pose", FX_DOUBLE * 6),
        ("ref_joints", FX_DOUBLE * 7),
        ("solution", FX_DOUBLE * 7),
        ("max_iter", FX_INT32),
        ("tol", FX_DOUBLE),
        ("solution_valid", FX_BOOL),
    ]

class DualArmFixedBodyParams(Structure):
    _fields_ = [
        ("body_joints", FX_DOUBLE * 3),
        ("left_start_joints", FX_DOUBLE * 7),
        ("right_start_joints", FX_DOUBLE * 7),
        ("left_target_pose", FX_DOUBLE * 6),
        ("right_target_pose", FX_DOUBLE * 6),
        ("vel_ratio", FX_DOUBLE),
        ("acc_ratio", FX_DOUBLE),
    ]

# ==================== fault code dictionary ====================
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

# ==================== GentoRobot class ====================
class GentoRobot:
    def __init__(self, precision: int = 3):
        self._lock = threading.Lock()
        self.precision = precision
        self._round_float = lambda v: round(v, self.precision)

        if os.name == 'nt':
            self.dll = ctypes.WinDLL(os.path.join(current_path, 'libGentoSDKPY.dll'))
        else:
            self.dll = ctypes.CDLL(os.path.join(current_path, 'libGentoSDKPY.so'))
        self._init_funcs()

        self._kin_handle = None

        ret = self.dll.RobotCache_Init()
        if ret != 0:
            raise RuntimeError("RobotCache_Init failed")

        self._rt_ptr = self.dll.RobotCache_GetRT()
        self._sg_ptr = self.dll.RobotCache_GetSG()
        if not self._rt_ptr or not self._sg_ptr:
            raise RuntimeError("Failed to get RT/SG pointers")

        self._rt = cast(self._rt_ptr, POINTER(ROBOT_RT))
        self._sg = cast(self._sg_ptr, POINTER(ROBOT_SG))

    def _init_funcs(self):
        d = self.dll

        # RobotCache
        d.RobotCache_Init.argtypes = []
        d.RobotCache_Init.restype = c_int
        d.RobotCache_GetRT.argtypes = []
        d.RobotCache_GetRT.restype = c_void_p
        d.RobotCache_GetSG.argtypes = []
        d.RobotCache_GetSG.restype = c_void_p
        d.RobotCache_Cleanup.argtypes = []
        d.RobotCache_Cleanup.restype = None

        # System
        d.FX_L1_System_Link.argtypes = [FX_UCHAR, FX_UCHAR, FX_UCHAR, FX_UCHAR, POINTER(FX_INT32), FX_UINT32]
        d.FX_L1_System_Link.restype = FX_INT32
        d.FX_L1_System_Reboot.argtypes = []
        d.FX_L1_System_Reboot.restype = FX_BOOL

        # Communication
        d.FX_L1_Comm_Clear.argtypes = [FX_UINT32]
        d.FX_L1_Comm_Clear.restype = FX_BOOL
        d.FX_L1_Comm_Send.argtypes = []
        d.FX_L1_Comm_Send.restype = FX_BOOL
        d.FX_L1_Comm_SendAndWait.argtypes = [FX_UINT32]
        d.FX_L1_Comm_SendAndWait.restype = FX_BOOL

        # State feedback
        d.FX_L1_Fbk_GetCtrlObjDof.argtypes = [FX_INT32]
        d.FX_L1_Fbk_GetCtrlObjDof.restype = FX_INT32
        d.FX_L1_Fbk_CurrentState.argtypes = [FX_INT32]
        d.FX_L1_Fbk_CurrentState.restype = FX_INT32
        d.FX_L1_State_GetServoErrorCode.argtypes = [FX_INT32, POINTER(FX_INT32 * 7)]
        d.FX_L1_State_GetServoErrorCode.restype = FX_BOOL
        d.FX_L1_State_ResetError.argtypes = [FX_UINT32]
        d.FX_L1_State_ResetError.restype = FX_UINT32
        d.FX_L1_Runtime_EmergencyStop.argtypes = [FX_UINT32]
        d.FX_L1_Runtime_EmergencyStop.restype = FX_UINT32

        # State switching
        switch_funcs = [
            "SwitchToIdle", "SwitchToPositionMode", "SwitchToImpJointMode",
            "SwitchToImpCartMode", "SwitchToImpForceMode", "SwitchToDragJoint",
            "SwitchToDragCartX", "SwitchToDragCartY", "SwitchToDragCartZ",
            "SwitchToDragCartR", "SwitchToCollaborativeRelease"
        ]
        for func_name in switch_funcs:
            full_name = f"FX_L1_State_{func_name}"
            try:
                func = getattr(d, full_name)
            except AttributeError:
                print(f"Warning: {full_name} not found in DLL, skipping this function.")
                continue

            if func_name == "SwitchToPositionMode":
                func.argtypes = [FX_INT32, FX_UINT32, FX_DOUBLE, FX_DOUBLE]
            elif func_name in ["SwitchToImpJointMode", "SwitchToImpCartMode"]:
                func.argtypes = [FX_INT32, FX_UINT32, FX_DOUBLE, FX_DOUBLE, POINTER(FX_DOUBLE * 7),
                                 POINTER(FX_DOUBLE * 7)]
            elif func_name == "SwitchToImpForceMode":
                func.argtypes = [FX_INT32, FX_UINT32, POINTER(FX_DOUBLE * 5), POINTER(FX_DOUBLE * 5)]
            elif func_name in ["SwitchToDragJoint", "SwitchToDragCartX", "SwitchToDragCartY",
                               "SwitchToDragCartZ", "SwitchToDragCartR", "SwitchToCollaborativeRelease"]:
                func.argtypes = [FX_INT32, FX_UINT32]
            else:
                func.argtypes = [FX_INT32, FX_UINT32]
            func.restype = FX_INT32

        # Parameters
        d.FX_L1_Param_SetInt32.argtypes = [c_char_p, FX_INT32]
        d.FX_L1_Param_SetInt32.restype = FX_BOOL
        d.FX_L1_Param_SetFloat.argtypes = [c_char_p, FX_FLOAT]
        d.FX_L1_Param_SetFloat.restype = FX_BOOL
        d.FX_L1_Param_GetInt32.argtypes = [c_char_p, POINTER(FX_INT32)]
        d.FX_L1_Param_GetInt32.restype = FX_BOOL
        d.FX_L1_Param_GetFloat.argtypes = [c_char_p, POINTER(FX_FLOAT)]
        d.FX_L1_Param_GetFloat.restype = FX_BOOL

        # Terminal
        d.FX_L1_Terminal_ClearData.argtypes = [FX_INT32]
        d.FX_L1_Terminal_ClearData.restype = FX_BOOL
        d.FX_L1_Terminal_GetData.argtypes = [FX_INT32, POINTER(FX_INT32), POINTER(FX_UCHAR * 64)]
        d.FX_L1_Terminal_GetData.restype = FX_INT32
        d.FX_L1_Terminal_SetData.argtypes = [FX_INT32, FX_INT32, POINTER(FX_UCHAR), FX_UINT32]
        d.FX_L1_Terminal_SetData.restype = FX_BOOL

        # Config
        d.FX_L1_Config_SetBrakeLock.argtypes = [FX_INT32, FX_UINT8]
        d.FX_L1_Config_SetBrakeLock.restype = FX_BOOL
        d.FX_L1_Config_SetBrakeUnlock.argtypes = [FX_INT32, FX_UINT8]
        d.FX_L1_Config_SetBrakeUnlock.restype = FX_BOOL
        d.FX_L1_Config_ResetEncOffset.argtypes = [FX_INT32, FX_UINT8]
        d.FX_L1_Config_ResetEncOffset.restype = FX_BOOL
        d.FX_L1_Config_ClearEncError.argtypes = [FX_INT32, FX_UINT8]
        d.FX_L1_Config_ClearEncError.restype = FX_BOOL
        d.FX_L1_Config_DisableSoftLimit.argtypes = [FX_INT32, FX_UINT8]
        d.FX_L1_Config_DisableSoftLimit.restype = FX_BOOL
        d.FX_L1_Config_ClearAxisSensorOffset.argtypes = [FX_INT32, FX_UINT32]
        d.FX_L1_Config_ClearAxisSensorOffset.restype = FX_BOOL
        d.FX_L1_Config_ClearSensorOffset.argtypes = [FX_INT32]
        d.FX_L1_Config_ClearSensorOffset.restype = FX_BOOL
        d.FX_L1_Config_SetTraj.argtypes = [FX_INT32, FX_UINT32, POINTER(FX_DOUBLE)]
        d.FX_L1_Config_SetTraj.restype = FX_BOOL

        # Runtime commands
        d.FX_L1_Runtime_SetJointPosCmd.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetJointPosCmd.restype = FX_BOOL
        d.FX_L1_Runtime_SetForceCtrl.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 5)]
        d.FX_L1_Runtime_SetForceCtrl.restype = FX_BOOL
        d.FX_L1_Runtime_SetTorqueCtrl.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 5)]
        d.FX_L1_Runtime_SetTorqueCtrl.restype = FX_BOOL
        d.FX_L1_Runtime_SetVelRatio.argtypes = [FX_INT32, FX_DOUBLE]
        d.FX_L1_Runtime_SetVelRatio.restype = FX_BOOL
        d.FX_L1_Runtime_SetAccRatio.argtypes = [FX_INT32, FX_DOUBLE]
        d.FX_L1_Runtime_SetAccRatio.restype = FX_BOOL
        d.FX_L1_Runtime_SetSpeedRatio.argtypes = [FX_INT32, FX_DOUBLE, FX_DOUBLE]
        d.FX_L1_Runtime_SetSpeedRatio.restype = FX_BOOL
        d.FX_L1_Runtime_SetJointK.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetJointK.restype = FX_BOOL
        d.FX_L1_Runtime_SetJointD.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetJointD.restype = FX_BOOL
        d.FX_L1_Runtime_SetJointKD.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7), POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetJointKD.restype = FX_BOOL
        d.FX_L1_Runtime_SetCartK.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetCartK.restype = FX_BOOL
        d.FX_L1_Runtime_SetCartD.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetCartD.restype = FX_BOOL
        d.FX_L1_Runtime_SetCartKD.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 7), POINTER(FX_DOUBLE * 7)]
        d.FX_L1_Runtime_SetCartKD.restype = FX_BOOL
        d.FX_L1_Runtime_SetToolK.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 6)]
        d.FX_L1_Runtime_SetToolK.restype = FX_BOOL
        d.FX_L1_Runtime_SetToolD.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 10)]
        d.FX_L1_Runtime_SetToolD.restype = FX_BOOL
        d.FX_L1_Runtime_SetToolKD.argtypes = [FX_INT32, POINTER(FX_DOUBLE * 6), POINTER(FX_DOUBLE * 10)]
        d.FX_L1_Runtime_SetToolKD.restype = FX_BOOL
        d.FX_L1_Runtime_SetBodyPDP.argtypes = [POINTER(FX_DOUBLE * 6)]
        d.FX_L1_Runtime_SetBodyPDP.restype = FX_BOOL
        d.FX_L1_Runtime_SetBodyPDD.argtypes = [POINTER(FX_DOUBLE * 6)]
        d.FX_L1_Runtime_SetBodyPDD.restype = FX_BOOL
        d.FX_L1_Runtime_SetBodyPD.argtypes = [POINTER(FX_DOUBLE * 6), POINTER(FX_DOUBLE * 6)]
        d.FX_L1_Runtime_SetBodyPD.restype = FX_BOOL
        d.FX_L1_Runtime_RunTraj.argtypes = [FX_UINT32]
        d.FX_L1_Runtime_RunTraj.restype = FX_UINT32
        d.FX_L1_Runtime_StopTraj.argtypes = [FX_UINT32]
        d.FX_L1_Runtime_StopTraj.restype = FX_UINT32

        # Kinematics functions
        d.FX_L1_Kinematics_Create.argtypes = []
        d.FX_L1_Kinematics_Create.restype = c_void_p

        d.FX_L1_Kinematics_Destroy.argtypes = [c_void_p]
        d.FX_L1_Kinematics_Destroy.restype = None

        d.FX_L1_Kinematics_LogSwitch.argtypes = [c_void_p, FX_INT32]
        d.FX_L1_Kinematics_LogSwitch.restype = None

        d.FX_L1_Kinematics_InitSingleArm.argtypes = [
            c_void_p,  # handle
            FX_INT32,  # RobotSerial
            POINTER(FX_INT32),  # type (int*)
            (FX_DOUBLE * 4) * 8,  # DH[8][4]
            (FX_DOUBLE * 4) * 8,  # PNVA[8][4]
            (FX_DOUBLE * 3) * 4,  # BOUND[4][3]
            FX_DOUBLE * 3,  # GRV[3]
            FX_DOUBLE * 7,  # MASS[7]
            (FX_DOUBLE * 3) * 7,  # MCP[7][3]
            (FX_DOUBLE * 6) * 7,  # I[7][6]
        ]
        d.FX_L1_Kinematics_InitSingleArm.restype = FX_INT32

        d.FX_L1_Kinematics_InitDualArm.argtypes = [
            c_void_p,  # handle
            POINTER(FX_INT32 * 2),  # type[2]
            ((FX_DOUBLE * 4) * 8) * 2,  # DH[2][8][4]
            ((FX_DOUBLE * 4) * 8) * 2,  # PNVA[2][8][4]
            ((FX_DOUBLE * 3) * 4) * 2,  # BOUND[2][4][3]
        ]
        d.FX_L1_Kinematics_InitDualArm.restype = FX_INT32

        d.FX_L1_Kinematics_ForwardKinematics.argtypes = [c_void_p, FX_INT32, POINTER(FX_DOUBLE * 7), POINTER(FX_DOUBLE * 16)]
        d.FX_L1_Kinematics_ForwardKinematics.restype = FX_INT32

        d.FX_L1_Kinematics_Jacobian.argtypes = [c_void_p, FX_INT32, POINTER(FX_DOUBLE * 7), POINTER(FX_DOUBLE * 42)]
        d.FX_L1_Kinematics_Jacobian.restype = FX_INT32

        d.FX_L1_Kinematics_InverseKinematics.argtypes = [c_void_p, FX_INT32, POINTER(FX_InvKineSolverParams)]
        d.FX_L1_Kinematics_InverseKinematics.restype = FX_INT32

        d.FX_L1_Kinematics_SetBodyCondition.argtypes = [c_void_p, POINTER(FX_DOUBLE * 3), POINTER(FX_DOUBLE * 3),
                                                        FX_DOUBLE, FX_DOUBLE, FX_DOUBLE, FX_DOUBLE]
        d.FX_L1_Kinematics_SetBodyCondition.restype = FX_INT32

        d.FX_L1_Kinematics_BodyForward.argtypes = [c_void_p, POINTER(FX_DOUBLE * 3),
                                                   POINTER(FX_DOUBLE * 16), POINTER(FX_DOUBLE * 16)]
        d.FX_L1_Kinematics_BodyForward.restype = FX_INT32

        d.FX_L1_Kinematics_CalcBodyPosition.argtypes = [c_void_p, POINTER(FX_DOUBLE * 3), POINTER(FX_DOUBLE * 3),
                                                        POINTER(FX_DOUBLE * 3)]
        d.FX_L1_Kinematics_CalcBodyPosition.restype = FX_INT32

        d.FX_L1_Kinematics_CalcBodyPositionWithRef.argtypes = [c_void_p, POINTER(FX_DOUBLE * 3),
                                                               POINTER(FX_DOUBLE * 3), POINTER(FX_DOUBLE * 3),
                                                               POINTER(FX_DOUBLE * 3)]
        d.FX_L1_Kinematics_CalcBodyPositionWithRef.restype = FX_INT32

        d.FX_L1_Kinematics_PlanJointMove.argtypes = [c_void_p, FX_INT32, POINTER(FX_DOUBLE * 7), POINTER(FX_DOUBLE * 7),
                                                     FX_DOUBLE, FX_DOUBLE, c_void_p, POINTER(FX_INT32)]
        d.FX_L1_Kinematics_PlanJointMove.restype = FX_INT32

        d.FX_L1_Kinematics_PlanLinearMove.argtypes = [c_void_p, FX_INT32, POINTER(FX_DOUBLE * 6), POINTER(FX_DOUBLE * 6),
                                                      POINTER(FX_DOUBLE * 7), FX_DOUBLE, FX_DOUBLE, FX_INT32, c_void_p, POINTER(FX_INT32)]
        d.FX_L1_Kinematics_PlanLinearMove.restype = FX_INT32

        d.FX_L1_Kinematics_PlanLinearKeepJoints.argtypes = [c_void_p, FX_INT32, POINTER(FX_DOUBLE * 7), POINTER(FX_DOUBLE * 7),
                                                            FX_DOUBLE, FX_DOUBLE, FX_INT32, c_void_p, POINTER(FX_INT32)]
        d.FX_L1_Kinematics_PlanLinearKeepJoints.restype = FX_INT32

        d.FX_L1_Kinematics_PlanDualArmFixedBody.argtypes = [c_void_p, POINTER(DualArmFixedBodyParams),
                                                            c_void_p, c_void_p, POINTER(FX_INT32)]
        d.FX_L1_Kinematics_PlanDualArmFixedBody.restype = FX_INT32

        # Helpers
        d.FX_L1_XYZABC2Matrix.argtypes = [POINTER(FX_DOUBLE * 6), POINTER(FX_DOUBLE * 16)]
        d.FX_L1_XYZABC2Matrix.restype = None
        d.FX_L1_Matrix2XYZABC.argtypes = [POINTER(FX_DOUBLE * 16), POINTER(FX_DOUBLE * 6)]
        d.FX_L1_Matrix2XYZABC.restype = None

        # PointSet
        d.FX_L1_CPointSet_Create.argtypes = []
        d.FX_L1_CPointSet_Create.restype = c_void_p
        d.FX_L1_CPointSet_Destroy.argtypes = [c_void_p]
        d.FX_L1_CPointSet_Destroy.restype = None
        d.FX_L1_CPointSet_OnInit.argtypes = [c_void_p, FX_INT32]
        d.FX_L1_CPointSet_OnInit.restype = FX_BOOL
        d.FX_L1_CPointSet_OnGetPointNum.argtypes = [c_void_p]
        d.FX_L1_CPointSet_OnGetPointNum.restype = FX_INT32
        d.FX_L1_CPointSet_OnGetPoint.argtypes = [c_void_p, FX_INT32]
        d.FX_L1_CPointSet_OnGetPoint.restype = POINTER(FX_DOUBLE)
        d.FX_L1_CPointSet_OnSetPoint.argtypes = [c_void_p, POINTER(FX_DOUBLE)]
        d.FX_L1_CPointSet_OnSetPoint.restype = FX_BOOL
        d.FX_L1_CPointSet_OnAppendPoint.argtypes = [c_void_p, POINTER(FX_DOUBLE)]
        d.FX_L1_CPointSet_OnAppendPoint.restype = FX_BOOL

        # File transfer
        d.FX_L1_SendFile.argtypes = [c_char_p, c_char_p]
        d.FX_L1_SendFile.restype = FX_BOOL
        d.FX_L1_RecvFile.argtypes = [c_char_p, c_char_p]
        d.FX_L1_RecvFile.restype = FX_BOOL

    @property
    def rt(self) -> ROBOT_RT:
        return self._rt.contents

    @property
    def sg(self) -> ROBOT_SG:
        return self._sg.contents

    def get_rt_dict(self) -> dict:
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
                "state": {
                    "cur": hand.m_HAND_State.m_CurState,
                    "cmd": hand.m_HAND_State.m_CmdState,
                    "err": hand.m_HAND_State.m_ERRCode,
                },
                "cmd": {
                    "pos": [hand.m_HAND_IN.m_HAND_CMD_Joint_Pos[i] for i in range(24)],
                    "torque": [hand.m_HAND_IN.m_HAND_CMD_Joint_Tor[i] for i in range(24)],
                },
                "fb": {
                    "pos": [hand.m_HAND_OUT.m_HAND_FBK_Joint_Pos[i] for i in range(24)],
                    "torque": [hand.m_HAND_OUT.m_HAND_FBK_Joint_Tor[i] for i in range(24)],
                    "force": [hand.m_HAND_OUT.m_HAND_FBK_F[i] for i in range(24)],
                },
            }
            data["hands"].append(hand_data)
        return data

    def get_sg_dict(self) -> dict:
        if not self._connected:
            return {"error": "Robot not connected"}
        sg = self.sg
        data = {
            "frame_serial": sg.m_RT_FrameSerial,
            "head": {
                "set": {
                    "vel_ratio": self._round_float(sg.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_VelRatio),
                    "acc_ratio": self._round_float(sg.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_AccRatio),
                    "set_tags": [sg.m_HEAD.m_HEAD_SET.m_HEAD_SET_SetTags[i] for i in range(4)],
                    "update_tags": [sg.m_HEAD.m_HEAD_SET.m_HEAD_SET_UpDateTag[i] for i in range(4)],
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
                    "set_tags": [sg.m_BODY.m_BODY_SET.m_BODY_SET_SetTags[i] for i in range(6)],
                    "update_tags": [sg.m_BODY.m_BODY_SET.m_BODY_SET_UpDateTag[i] for i in range(6)],
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
                    "set_tags": [sg.m_LIFT.m_LIFT_SET.m_LIFT_SET_SetTags[i] for i in range(4)],
                    "update_tags": [sg.m_LIFT.m_LIFT_SET.m_LIFT_SET_UpDateTag[i] for i in range(4)],
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
                    "set_tags": [arm_sg.m_ARM_SET.m_ARM_SET_SetTags[i] for i in range(16)],
                    "update_tags": [arm_sg.m_ARM_SET.m_ARM_SET_UpDateTag[i] for i in range(16)],
                },
                "get": {
                    "joint_torque": [self._round_float(arm_sg.m_ARM_GET.m_ARM_FBK_Joint_Tor[i]) for i in range(7)],
                    "ext_pos": [self._round_float(arm_sg.m_ARM_GET.m_ARM_FBK_Joint_ExtPos[i]) for i in range(7)],
                    "tip_di": arm_sg.m_ARM_GET.m_ARM_FBK_Flange_DI,
                    "low_speed_flag": arm_sg.m_ARM_GET.m_ARM_FBK_LowSpdFlag,
                    "traj_state": arm_sg.m_ARM_GET.m_ARM_FBK_TrajState,
                },
            }
            data["arms"].append(arm_data)
        for hand_idx in range(2):
            hand_sg = sg.m_HANDS[hand_idx]
            hand_data = {
                "set": {
                    "kp": [hand_sg.m_HAND_SET.m_HAND_Ctrl_KP[i] for i in range(24)],
                    "kd": [hand_sg.m_HAND_SET.m_HAND_Ctrl_KD[i] for i in range(24)],
                    "vel": [hand_sg.m_HAND_SET.m_HAND_Ctrl_Vel[i] for i in range(24)],
                },
            }
            data["hands"].append(hand_data)
        return data

    # ==================== System ====================
    def link(self, ip1: int, ip2: int, ip3: int, ip4: int, log_switch: int = 0) -> int:
        with self._lock:
            ver = FX_INT32()
            ret = self.dll.FX_L1_System_Link(ip1, ip2, ip3, ip4, byref(ver), log_switch)
            if ret > 0:
                self._connected = True
                return ver.value
            else:
                self._connected = False
                return ret

    def reboot(self) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_System_Reboot())

    # ==================== Communication ====================
    def comm_clear(self, timeout: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Comm_Clear(timeout))

    def comm_send(self) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Comm_Send())

    def comm_send_and_wait(self, timeout: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Comm_SendAndWait(timeout))

    # ==================== State feedback ====================
    def get_ctrl_obj_dof(self, obj_type: int) -> int:
        return self.dll.FX_L1_Fbk_GetCtrlObjDof(obj_type)

    def current_state(self, obj_type: int) -> int:
        return self.dll.FX_L1_Fbk_CurrentState(obj_type)

    def get_servo_error_codes(self, obj_type: int) -> Optional[str]:
        arr = (FX_INT32 * 7)()
        with self._lock:
            if self.dll.FX_L1_State_GetServoErrorCode(obj_type, arr):
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
                    code_val = arr[i] & 0xFFFF
                    code_str = f"0x{code_val:04X}"
                    if code_val == 0:
                        desc = "No error"
                    else:
                        desc = fault_code_dict_EN.get(code_str, "Unknown error")
                    lines.append(f"{axis_names[i]}: {code_str} - {desc}")
                return "\n".join(lines)
        return None

    def reset_error(self, obj_mask: int) -> int:
        with self._lock:
            return self.dll.FX_L1_State_ResetError(obj_mask)

    def emergency_stop(self, obj_mask: int) -> int:
        with self._lock:
            return self.dll.FX_L1_Runtime_EmergencyStop(obj_mask)

    # ==================== State switching ====================
    def switch_to_idle(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToIdle(obj_type, timeout)
        return self._switch_check("switch_to_idle", ret, raise_error)

    def switch_to_position_mode(self, obj_type: int, timeout: int, vel: float, acc: float,
                                raise_error: bool = False) -> int:
        vel = max(1, min(vel, 100))
        acc = max(1, min(acc, 100))
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToPositionMode(obj_type, timeout, vel, acc)
        return self._switch_check("switch_to_position_mode", ret, raise_error)

    def switch_to_imp_joint_mode(self, obj_type: int, timeout: int, vel: float, acc: float,
                                 k: List[float], d: List[float], raise_error: bool = False) -> int:
        vel = max(1, min(vel, 100))
        acc = max(1, min(acc, 100))
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (FX_DOUBLE * 7)(*k)
        d_arr = (FX_DOUBLE * 7)(*d)
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToImpJointMode(obj_type, timeout, vel, acc, k_arr, d_arr)
        return self._switch_check("switch_to_imp_joint_mode", ret, raise_error)

    def switch_to_imp_cart_mode(self, obj_type: int, timeout: int, vel: float, acc: float,
                                k: List[float], d: List[float], raise_error: bool = False) -> int:
        vel = max(1, min(vel, 100))
        acc = max(1, min(acc, 100))
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (FX_DOUBLE * 7)(*k)
        d_arr = (FX_DOUBLE * 7)(*d)
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToImpCartMode(obj_type, timeout, vel, acc, k_arr, d_arr)
        return self._switch_check("switch_to_imp_cart_mode", ret, raise_error)

    def switch_to_imp_force_mode(self, obj_type: int, timeout: int,
                                 force_ctrl: List[float], torque_ctrl: List[float], raise_error: bool = False) -> int:
        f_arr = (FX_DOUBLE * 5)(*force_ctrl)
        t_arr = (FX_DOUBLE * 5)(*torque_ctrl)
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToImpForceMode(obj_type, timeout, f_arr, t_arr)
        return self._switch_check("switch_to_imp_force_mode", ret, raise_error)

    def switch_to_drag_joint(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToDragJoint(obj_type, timeout)
        return self._switch_check("switch_to_drag_joint", ret, raise_error)

    def switch_to_drag_cart_x(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToDragCartX(obj_type, timeout)
        return self._switch_check("switch_to_drag_cart_x", ret, raise_error)

    def switch_to_drag_cart_y(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToDragCartY(obj_type, timeout)
        return self._switch_check("switch_to_drag_cart_y", ret, raise_error)

    def switch_to_drag_cart_z(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToDragCartZ(obj_type, timeout)
        return self._switch_check("switch_to_drag_cart_z", ret, raise_error)

    def switch_to_drag_cart_r(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToDragCartR(obj_type, timeout)
        return self._switch_check("switch_to_drag_cart_r", ret, raise_error)

    def switch_to_collab_release(self, obj_type: int, timeout: int, raise_error: bool = False) -> int:
        with self._lock:
            ret = self.dll.FX_L1_State_SwitchToCollaborativeRelease(obj_type, timeout)
        return self._switch_check("switch_to_collab_release", ret, raise_error)

    # ==================== Parameters ====================
    def param_set_int(self, name: str, value: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Param_SetInt32(name.encode('utf-8'), value))

    def param_set_float(self, name: str, value: float) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Param_SetFloat(name.encode('utf-8'), value))

    def param_get_int(self, name: str) -> Optional[int]:
        ret = FX_INT32()
        with self._lock:
            if self.dll.FX_L1_Param_GetInt32(name.encode('utf-8'), byref(ret)):
                return ret.value
        return None

    def param_get_float(self, name: str) -> Optional[float]:
        ret = FX_FLOAT()
        with self._lock:
            if self.dll.FX_L1_Param_GetFloat(name.encode('utf-8'), byref(ret)):
                return ret.value
        return None

    # ==================== Terminal ====================
    def terminal_clear(self, terminal_type: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Terminal_ClearData(terminal_type))

    def terminal_get(self, terminal_type: int) -> Tuple[int, bytes]:
        chn = FX_INT32()
        data = (FX_UCHAR * 64)()
        with self._lock:
            n = self.dll.FX_L1_Terminal_GetData(terminal_type, byref(chn), data)
        if n <= 0:
            return (0, b'')
        return (chn.value, bytes(data[:n]))

    def terminal_set(self, terminal_type: int, chn_type: int, data) -> bool:
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
        buf = (FX_UCHAR * data_len)(*data)
        with self._lock:
            return bool(self.dll.FX_L1_Terminal_SetData(terminal_type, chn_type, buf, data_len))

    # ==================== Configuration ====================
    def config_brake_lock(self, obj_type: int, axis_mask: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_SetBrakeLock(obj_type, axis_mask & 0xFF))

    def config_brake_unlock(self, obj_type: int, axis_mask: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_SetBrakeUnlock(obj_type, axis_mask & 0xFF))

    def config_reset_enc_offset(self, obj_type: int, axis_mask: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_ResetEncOffset(obj_type, axis_mask & 0xFF))

    def config_clear_enc_error(self, obj_type: int, axis_mask: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_ClearEncError(obj_type, axis_mask & 0xFF))

    def config_disable_soft_limit(self, obj_type: int, axis_mask: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_DisableSoftLimit(obj_type, axis_mask & 0xFF))

    def config_clear_axis_sensor_offset(self, obj_type: int, axis_id: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_ClearAxisSensorOffset(obj_type, axis_id))

    def config_clear_sensor_offset(self, obj_type: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_Config_ClearSensorOffset(obj_type))

    def config_set_traj(self, obj_type: int, point_data: List[float]) -> bool:
        point_num = len(point_data) // (7 if obj_type in (FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1) else 6)
        arr = (FX_DOUBLE * len(point_data))(*point_data)
        with self._lock:
            return bool(self.dll.FX_L1_Config_SetTraj(obj_type, point_num, arr))

    # ==================== Runtime commands ====================
    def runtime_set_joint_pos_cmd(self, obj_type: int, positions: List[float]) -> bool:
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
        arr = (FX_DOUBLE * 7)(*padded)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetJointPosCmd(obj_type, arr))

    def runtime_set_force_ctrl(self, obj_type: int, force_ctrl: List[float]) -> bool:
        if len(force_ctrl) != 5:
            raise ValueError("force_ctrl must have 5 elements")
        arr = (FX_DOUBLE * 5)(*force_ctrl)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetForceCtrl(obj_type, arr))

    def runtime_set_torque_ctrl(self, obj_type: int, torque_ctrl: List[float]) -> bool:
        if len(torque_ctrl) != 5:
            raise ValueError("torque_ctrl must have 5 elements")
        arr = (FX_DOUBLE * 5)(*torque_ctrl)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetTorqueCtrl(obj_type, arr))

    def runtime_set_vel_ratio(self, obj_type: int, ratio: float) -> bool:
        ratio = max(1, min(ratio, 100))
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetVelRatio(obj_type, ratio))

    def runtime_set_acc_ratio(self, obj_type: int, ratio: float) -> bool:
        ratio = max(1, min(ratio, 100))
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetAccRatio(obj_type, ratio))

    def runtime_set_speed_ratio(self, obj_type: int, vel_ratio: float, acc_ratio: float) -> bool:
        vel_ratio = max(1, min(vel_ratio, 100))
        acc_ratio = max(1, min(acc_ratio, 100))
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetSpeedRatio(obj_type, vel_ratio, acc_ratio))

    def runtime_set_joint_k(self, obj_type: int, k: List[float]) -> bool:
        if len(k) != 7:
            raise ValueError("k must have 7 elements")
        k = [max(0, v) for v in k]
        arr = (FX_DOUBLE * 7)(*k)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetJointK(obj_type, arr))

    def runtime_set_joint_d(self, obj_type: int, d: List[float]) -> bool:
        if len(d) != 7:
            raise ValueError("d must have 7 elements")
        d = [max(0, v) for v in d]
        arr = (FX_DOUBLE * 7)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetJointD(obj_type, arr))

    def runtime_set_joint_kd(self, obj_type: int, k: List[float], d: List[float]) -> bool:
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (FX_DOUBLE * 7)(*k)
        d_arr = (FX_DOUBLE * 7)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetJointKD(obj_type, k_arr, d_arr))

    def runtime_set_cart_k(self, obj_type: int, k: List[float]) -> bool:
        if len(k) != 7:
            raise ValueError("k must have 7 elements")
        k = [max(0, v) for v in k]
        arr = (FX_DOUBLE * 7)(*k)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetCartK(obj_type, arr))

    def runtime_set_cart_d(self, obj_type: int, d: List[float]) -> bool:
        if len(d) != 7:
            raise ValueError("d must have 7 elements")
        d = [max(0, v) for v in d]
        arr = (FX_DOUBLE * 7)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetCartD(obj_type, arr))

    def runtime_set_cart_kd(self, obj_type: int, k: List[float], d: List[float]) -> bool:
        k = [max(0, v) for v in k]
        d = [max(0, v) for v in d]
        k_arr = (FX_DOUBLE * 7)(*k)
        d_arr = (FX_DOUBLE * 7)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetCartKD(obj_type, k_arr, d_arr))

    def runtime_set_tool_k(self, obj_type: int, k: List[float]) -> bool:
        """Tool offset: [x, y, z, rx, ry, rz] (6 elements)"""
        if len(k) != 6:
            raise ValueError("kinematics must have 6 elements (tool offset)")
        arr = (FX_DOUBLE * 6)(*k)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetToolK(obj_type, arr))

    def runtime_set_tool_d(self, obj_type: int, d: List[float]) -> bool:
        """Tool dynamics: [mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz] (10 elements)"""
        if len(d) != 10:
            raise ValueError("dynamics must have 10 elements (tool dynamics)")
        arr = (FX_DOUBLE * 10)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetToolD(obj_type, arr))

    def runtime_set_tool_kd(self, obj_type: int, k: List[float], d: List[float]) -> bool:
        if len(k) != 6:
            raise ValueError("kinematics must have 6 elements (tool offset)")
        if len(d) != 10:
            raise ValueError("dynamics must have 10 elements (tool dynamics)")
        k_arr = (FX_DOUBLE * 6)(*k)
        d_arr = (FX_DOUBLE * 10)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetToolKD(obj_type, k_arr, d_arr))

    def runtime_set_body_pdp(self, p: List[float]) -> bool:
        if len(p) != 6:
            raise ValueError("p must have 6 elements")
        p = [max(0, v) for v in p]
        arr = (FX_DOUBLE * 6)(*p)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetBodyPDP(arr))

    def runtime_set_body_pdd(self, d: List[float]) -> bool:
        if len(d) != 6:
            raise ValueError("d must have 6 elements")
        d = [max(0, v) for v in d]
        arr = (FX_DOUBLE * 6)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetBodyPDD(arr))

    def runtime_set_body_pd(self, p: List[float], d: List[float]) -> bool:
        if len(p) != 6 or len(d) != 6:
            raise ValueError("p and d must have 6 elements each")
        p = [max(0, v) for v in p]
        d = [max(0, v) for v in d]
        p_arr = (FX_DOUBLE * 6)(*p)
        d_arr = (FX_DOUBLE * 6)(*d)
        with self._lock:
            return bool(self.dll.FX_L1_Runtime_SetBodyPD(p_arr, d_arr))

    def runtime_run_traj(self, obj_mask: int) -> int:
        with self._lock:
            return self.dll.FX_L1_Runtime_RunTraj(obj_mask)

    def runtime_stop_traj(self, obj_mask: int) -> int:
        with self._lock:
            return self.dll.FX_L1_Runtime_StopTraj(obj_mask)

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
    def init_single_arm(self, robot_serial: int, robot_type: int,
                        DH: List[List[float]],  # 8x4
                        PNVA: List[List[float]],  # 8x4
                        BOUND: List[List[float]],  # 4x3
                        GRV: List[float],  # 3
                        MASS: List[float],  # 7
                        MCP: List[List[float]],  # 7x3
                        I: List[List[float]]  # 7x6
                        ) -> bool:
        self._ensure_kin_handle()
        type_var = FX_INT32(robot_type)

        # DH: 8x4
        dh_arr = ((FX_DOUBLE * 4) * 8)()
        for i in range(8):
            for j in range(4):
                dh_arr[i][j] = DH[i][j]

        # PNVA: 8x4
        pnva_arr = ((FX_DOUBLE * 4) * 8)()
        for i in range(8):
            for j in range(4):
                pnva_arr[i][j] = PNVA[i][j]

        # BOUND: 4x3
        bound_arr = ((FX_DOUBLE * 3) * 4)()
        for i in range(4):
            for j in range(3):
                bound_arr[i][j] = BOUND[i][j]

        grv_arr = (FX_DOUBLE * 3)(*GRV)
        mass_arr = (FX_DOUBLE * 7)(*MASS)

        # MCP: 7x3
        mcp_arr = ((FX_DOUBLE * 3) * 7)()
        for i in range(7):
            for j in range(3):
                mcp_arr[i][j] = MCP[i][j]

        # I: 7x6
        i_arr = ((FX_DOUBLE * 6) * 7)()
        for i in range(7):
            for j in range(6):
                i_arr[i][j] = I[i][j]

        with self._lock:
            ret = self.dll.FX_L1_Kinematics_InitSingleArm(
                self._kin_handle, robot_serial,
                byref(type_var), dh_arr, pnva_arr, bound_arr,
                grv_arr, mass_arr, mcp_arr, i_arr
            )
        return ret == 0

    def init_dual_arm(self,
                      robot_types: List[int],  # length 2
                      DH: List[List[List[float]]],  # 2 x 8 x 4
                      PNVA: List[List[List[float]]],  # 2 x 8 x 4
                      BOUND: List[List[List[float]]]  # 2 x 4 x 3
                      ) -> bool:
        self._ensure_kin_handle()
        type_arr = (FX_INT32 * 2)(*robot_types)

        # DH: 2x8x4
        dh_arr = (((FX_DOUBLE * 4) * 8) * 2)()
        for arm in range(2):
            for i in range(8):
                for j in range(4):
                    dh_arr[arm][i][j] = DH[arm][i][j]

        # PNVA: 2x8x4
        pnva_arr = (((FX_DOUBLE * 4) * 8) * 2)()
        for arm in range(2):
            for i in range(8):
                for j in range(4):
                    pnva_arr[arm][i][j] = PNVA[arm][i][j]

        # BOUND: 2x4x3
        bound_arr = (((FX_DOUBLE * 3) * 4) * 2)()
        for arm in range(2):
            for i in range(4):
                for j in range(3):
                    bound_arr[arm][i][j] = BOUND[arm][i][j]

        with self._lock:
            ret = self.dll.FX_L1_Kinematics_InitDualArm(
                self._kin_handle, type_arr, dh_arr, pnva_arr, bound_arr
            )
        return ret == 0

    # ---------- Forward kinematics ----------
    def forward_kinematics(self, arm_idx: int, joints: List[float]) -> Optional[List[float]]:
        """Compute forward kinematics. Returns 4x4 matrix as list of 16 floats."""
        if len(joints) != 7:
            raise ValueError("joints must have 7 elements")
        self._ensure_kin_handle()
        j_arr = (FX_DOUBLE * 7)(*joints)
        pose_arr = (FX_DOUBLE * 16)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_ForwardKinematics(self._kin_handle, arm_idx, j_arr, pose_arr)
            if ret == 0:
                return [pose_arr[i] for i in range(16)]
        return None

    # ---------- Jacobian ----------
    def jacobian(self, arm_idx: int, joints: List[float]) -> Optional[List[float]]:
        """Compute Jacobian matrix (6x7) as list of 42 floats."""
        if len(joints) != 7:
            raise ValueError("joints must have 7 elements")
        self._ensure_kin_handle()
        j_arr = (FX_DOUBLE * 7)(*joints)
        jac_arr = (FX_DOUBLE * 42)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_Jacobian(self._kin_handle, arm_idx, j_arr, jac_arr)
            if ret == 0:
                return [jac_arr[i] for i in range(42)]
        return None

    # ---------- Inverse kinematics ----------
    def inverse_kinematics(self, arm_idx: int, target_pose: List[float],
                           ref_joints: List[float], max_iter: int = 100, tol: float = 1e-6) -> Optional[List[float]]:
        """
        Solve inverse kinematics.
        target_pose: [x, y, z, rx, ry, rz] (angles in degrees)
        ref_joints: initial guess (7 values)
        Returns joint angles list of 7 floats on success.
        """
        if len(target_pose) != 6:
            raise ValueError("target_pose must have 6 elements")
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have 7 elements")
        self._ensure_kin_handle()
        params = FX_InvKineSolverParams()
        params.robot_serial = arm_idx
        for i in range(6):
            params.target_pose[i] = target_pose[i]
        for i in range(7):
            params.ref_joints[i] = ref_joints[i]
        params.max_iter = max_iter
        params.tol = tol
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_InverseKinematics(self._kin_handle, arm_idx, byref(params))
            if ret == 0 and params.solution_valid:
                return [params.solution[i] for i in range(7)]
        return None

    # ---------- Body kinematics ----------
    def set_body_condition(self,
                           std_body: List[float], k_body: List[float],
                           std_left_len: float, k_left: float,
                           std_right_len: float, k_right: float) -> bool:
        """Set MAX body kinematics parameters."""
        if len(std_body) != 3 or len(k_body) != 3:
            raise ValueError("std_body and k_body must have 3 elements each")
        self._ensure_kin_handle()
        std_arr = (FX_DOUBLE * 3)(*std_body)
        k_arr = (FX_DOUBLE * 3)(*k_body)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_SetBodyCondition(
                self._kin_handle, std_arr, k_arr,
                std_left_len, k_left, std_right_len, k_right
            )
        return ret == 0

    def body_forward(self, body_joints: List[float]) -> Optional[Tuple[List[float], List[float]]]:
        """
        Body forward kinematics.
        Returns two 4x4 matrices (left_shoulder, right_shoulder) each as list of 16 floats.
        """
        if len(body_joints) != 3:
            raise ValueError("body_joints must have 3 elements")
        self._ensure_kin_handle()
        j_arr = (FX_DOUBLE * 3)(*body_joints)
        left_pose = (FX_DOUBLE * 16)()
        right_pose = (FX_DOUBLE * 16)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_BodyForward(self._kin_handle, j_arr, left_pose, right_pose)
            if ret == 0:
                return ([left_pose[i] for i in range(16)], [right_pose[i] for i in range(16)])
        return None

    def body_forward_xyzabc(self, body_joints: List[float]) -> Optional[Tuple[List[float], List[float]]]:
        """
        Body forward kinematics returning XYZABC poses (6 values each).
        """
        matrices = self.body_forward(body_joints)
        if matrices is None:
            return None
        left_matrix, right_matrix = matrices
        left_xyzabc = self.matrix2xyzabc(left_matrix)
        right_xyzabc = self.matrix2xyzabc(right_matrix)
        return left_xyzabc, right_xyzabc

    def calc_body_position(self, left_tcp: List[float], right_tcp: List[float]) -> Optional[List[float]]:
        """Compute body joint values from dual-arm TCP positions."""
        if len(left_tcp) != 3 or len(right_tcp) != 3:
            raise ValueError("left_tcp and right_tcp must have 3 elements each")
        self._ensure_kin_handle()
        left_arr = (FX_DOUBLE * 3)(*left_tcp)
        right_arr = (FX_DOUBLE * 3)(*right_tcp)
        body_joints = (FX_DOUBLE * 3)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_CalcBodyPosition(self._kin_handle, left_arr, right_arr, body_joints)
            if ret == 0:
                return [body_joints[i] for i in range(3)]
        return None

    def calc_body_position_with_ref(self, ref_body_joints: List[float],
                                    left_tcp: List[float], right_tcp: List[float]) -> Optional[List[float]]:
        """Compute body joint values with a reference body pose."""
        if len(ref_body_joints) != 3 or len(left_tcp) != 3 or len(right_tcp) != 3:
            raise ValueError("All input arrays must have 3 elements")
        self._ensure_kin_handle()
        ref_arr = (FX_DOUBLE * 3)(*ref_body_joints)
        left_arr = (FX_DOUBLE * 3)(*left_tcp)
        right_arr = (FX_DOUBLE * 3)(*right_tcp)
        body_joints = (FX_DOUBLE * 3)()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_CalcBodyPositionWithRef(
                self._kin_handle, ref_arr, left_arr, right_arr, body_joints
            )
            if ret == 0:
                return [body_joints[i] for i in range(3)]
        return None

    # ---------- Motion planning ----------
    def plan_joint_move(self, arm_idx: int,
                        start_joints: List[float], end_joints: List[float],
                        vel_ratio: float, acc_ratio: float,
                        point_set_handle, point_num_ptr) -> bool:
        """Plan a joint-space MoveJ path."""
        if len(start_joints) != 7 or len(end_joints) != 7:
            raise ValueError("start_joints and end_joints must have 7 elements")
        self._ensure_kin_handle()
        start_arr = (FX_DOUBLE * 7)(*start_joints)
        end_arr = (FX_DOUBLE * 7)(*end_joints)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanJointMove(
                self._kin_handle, arm_idx, start_arr, end_arr,
                vel_ratio, acc_ratio, point_set_handle, point_num_ptr
            )
        return ret == 0

    def plan_linear_move(self, arm_idx: int,
                         start_xyzabc: List[float], end_xyzabc: List[float],
                         ref_joints: List[float],
                         vel: float, acc: float, freq: int,
                         point_set_handle, point_num_ptr) -> bool:
        """Plan a Cartesian linear MoveL path."""
        if len(start_xyzabc) != 6 or len(end_xyzabc) != 6:
            raise ValueError("start_xyzabc and end_xyzabc must have 6 elements")
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have 7 elements")
        self._ensure_kin_handle()
        start_arr = (FX_DOUBLE * 6)(*start_xyzabc)
        end_arr = (FX_DOUBLE * 6)(*end_xyzabc)
        ref_arr = (FX_DOUBLE * 7)(*ref_joints)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMove(
                self._kin_handle, arm_idx, start_arr, end_arr, ref_arr,
                vel, acc, freq, point_set_handle, point_num_ptr
            )
        return ret == 0

    def plan_linear_keep_joints(self, arm_idx: int,
                                start_joints: List[float], end_joints: List[float],
                                vel: float, acc: float, freq: int,
                                point_set_handle, point_num_ptr) -> bool:
        """Plan a linear path while keeping joint posture."""
        if len(start_joints) != 7 or len(end_joints) != 7:
            raise ValueError("start_joints and end_joints must have 7 elements")
        self._ensure_kin_handle()
        start_arr = (FX_DOUBLE * 7)(*start_joints)
        end_arr = (FX_DOUBLE * 7)(*end_joints)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearKeepJoints(
                self._kin_handle, arm_idx, start_arr, end_arr,
                vel, acc, freq, point_set_handle, point_num_ptr
            )
        return ret == 0

    # Multi-segment linear motion planning
    def plan_linear_move_multi_points_set_start(self, arm_idx: int,
                                                ref_joints: List[float],
                                                start_xyzabc: List[float], end_xyzabc: List[float],
                                                allow_range: float, zsp_type: int,
                                                zsp_para: List[float],
                                                vel: float, acc: float, freq: int) -> bool:
        """Start a multi-segment Cartesian MoveL planning sequence."""
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have 7 elements")
        if len(start_xyzabc) != 6 or len(end_xyzabc) != 6:
            raise ValueError("start_xyzabc and end_xyzabc must have 6 elements")
        if len(zsp_para) != 6:
            raise ValueError("zsp_para must have 6 elements")
        self._ensure_kin_handle()
        ref_arr = (FX_DOUBLE * 7)(*ref_joints)
        start_arr = (FX_DOUBLE * 6)(*start_xyzabc)
        end_arr = (FX_DOUBLE * 6)(*end_xyzabc)
        zsp_arr = (FX_DOUBLE * 6)(*zsp_para)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(
                self._kin_handle, arm_idx, ref_arr, start_arr, end_arr,
                allow_range, zsp_type, zsp_arr, vel, acc, freq
            )
        return ret == 0

    def plan_linear_move_multi_points_set_next(self, arm_idx: int,
                                               next_xyzabc: List[float],
                                               allow_range: float, zsp_type: int,
                                               zsp_para: List[float],
                                               vel: float, acc: float) -> bool:
        """Append the next segment to a multi-segment Cartesian MoveL path."""
        if len(next_xyzabc) != 6:
            raise ValueError("next_xyzabc must have 6 elements")
        if len(zsp_para) != 6:
            raise ValueError("zsp_para must have 6 elements")
        self._ensure_kin_handle()
        next_arr = (FX_DOUBLE * 6)(*next_xyzabc)
        zsp_arr = (FX_DOUBLE * 6)(*zsp_para)
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(
                self._kin_handle, arm_idx, next_arr,
                allow_range, zsp_type, zsp_arr, vel, acc
            )
        return ret == 0

    def plan_linear_move_multi_points_get_path(self, point_set_handle, point_num_ptr) -> bool:
        """Export the planned path of a multi-segment Cartesian MoveL sequence."""
        self._ensure_kin_handle()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPath(
                self._kin_handle, point_set_handle, point_num_ptr
            )
        return ret == 0

    def plan_dual_arm_fixed_body(self, params_ptr, left_point_set, right_point_set, point_num_ptr) -> bool:
        """Plan a synchronized dual-arm linear path with fixed body."""
        self._ensure_kin_handle()
        with self._lock:
            ret = self.dll.FX_L1_Kinematics_PlanDualArmFixedBody(
                self._kin_handle, params_ptr, left_point_set, right_point_set, point_num_ptr
            )
        return ret == 0

    # ---------- Helper conversions ----------
    def xyzabc2matrix(self, xyzabc: List[float]) -> List[float]:
        """Convert XYZABC pose to 4x4 transformation matrix (16 floats, row-major)."""
        if len(xyzabc) != 6:
            raise ValueError("xyzabc must have 6 elements")
        arr_in = (FX_DOUBLE * 6)(*xyzabc)
        arr_out = (FX_DOUBLE * 16)()
        with self._lock:
            self.dll.FX_L1_XYZABC2Matrix(arr_in, arr_out)
        return [arr_out[i] for i in range(16)]

    def matrix2xyzabc(self, matrix: List[float]) -> List[float]:
        """Convert 4x4 transformation matrix to XYZABC pose (angles in degrees)."""
        if len(matrix) != 16:
            raise ValueError("matrix must have 16 elements")
        arr_in = (FX_DOUBLE * 16)(*matrix)
        arr_out = (FX_DOUBLE * 6)()
        with self._lock:
            self.dll.FX_L1_Matrix2XYZABC(arr_in, arr_out)
        return [arr_out[i] for i in range(6)]

    # ---------- PointSet ----------
    def pointset_create(self) -> c_void_p:
        with self._lock:
            return self.dll.FX_L1_CPointSet_Create()

    def pointset_destroy(self, pset):
        with self._lock:
            self.dll.FX_L1_CPointSet_Destroy(pset)

    def pointset_init(self, pset, ptype: int) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_CPointSet_OnInit(pset, ptype))

    def pointset_get_num(self, pset) -> int:
        return self.dll.FX_L1_CPointSet_OnGetPointNum(pset)

    def pointset_get_point(self, pset, index: int) -> Optional[List[float]]:
        ptr = self.dll.FX_L1_CPointSet_OnGetPoint(pset, index)
        if ptr:
            return [ptr[i] for i in range(7)]
        return None

    def pointset_set_point(self, pset, point: List[float]) -> bool:
        arr = (FX_DOUBLE * len(point))(*point)
        with self._lock:
            return bool(self.dll.FX_L1_CPointSet_OnSetPoint(pset, arr))

    def pointset_append_point(self, pset, point: List[float]) -> bool:
        arr = (FX_DOUBLE * len(point))(*point)
        with self._lock:
            return bool(self.dll.FX_L1_CPointSet_OnAppendPoint(pset, arr))

    # ---------- Cleanup ----------
    def cleanup(self):
        """Clean up all resources including kinematics handle."""
        self._destroy_kin_handle()
        self.dll.RobotCache_Cleanup()

    @staticmethod
    def _get_switch_error_msg(error_code: int) -> str:
        errors = {
            0: "Success",
            -1: "Clear set timeout",
            -2: "Send and wait timeout",
            -3: "Format cmd failed, command list too long",
            -4: "Invalid object type",
            -5: "Controller execution timeout",
            -6: "State transition not allowed in current state",
            -7: "Object is moving, state transition not allowed",
            -8: "Invalid input parameters",
        }
        return errors.get(error_code, f"Unknown error code: {error_code}")

    def _switch_check(self, func_name: str, ret: int, raise_error: bool):
        if ret != 0:
            msg = self._get_switch_error_msg(ret)
            print(f"[Warning] {func_name} failed: {msg}")
            if raise_error:
                raise RuntimeError(msg)
        return ret

    # ==================== File transfer ====================
    def send_file(self, local_path: str, remote_path: str) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_SendFile(local_path.encode('utf-8'), remote_path.encode('utf-8')))

    def recv_file(self, local_path: str, remote_path: str) -> bool:
        with self._lock:
            return bool(self.dll.FX_L1_RecvFile(local_path.encode('utf-8'), remote_path.encode('utf-8')))


# ==================== RobotDataManager (unchanged) ====================
class RobotDataManager:
    def __init__(self, robot):
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
        with self._lock:
            return self._latest_rt

    @property
    def latest_sg(self):
        with self._lock:
            return self._latest_sg

    def stop(self):
        self._running = False
        self._rt_thread.join(timeout=1.0)
        self._sg_thread.join(timeout=1.0)

if __name__ == "__main__":
    robot = GentoRobot()
    version = robot.link(6, 6, 7, 190, log_switch=1)
    if version <= 0:
        print(f"Failed to connect, error code: {version}")
        exit(1)

    rt_dict = robot.get_rt_dict()
    print("RT frame serial:", rt_dict["frame_serial"])
    print("Arm0 joint positions:", rt_dict["arms"][0]["fb"]["joint_pos"])
    print("Arm1 joint positions:", rt_dict["arms"][1]["fb"]["joint_pos"])
    print("head joint positions:", rt_dict["head"]["fb_pos"])
    print("body joint positions:", rt_dict["body"]["fb_pos"])
    print("lift joint positions:", rt_dict["lift"]["fb_pos"])

    sg_dict = robot.get_sg_dict()
    print("Arm0 low speed flag:", sg_dict["arms"][0]['get']["low_speed_flag"])
    print("TipDI:", sg_dict["arms"][0]['get']["tip_di"])
    print("TrajState:", sg_dict["arms"][0]['get']["traj_state"])