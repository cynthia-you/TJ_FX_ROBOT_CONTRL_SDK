from ctypes import *
import ctypes
import inspect
from textwrap import dedent
import os
import math
import logging
from ctypes import *

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

current_path = os.getcwd()


# 定义基本类型
FX_INT32L = c_long
FX_DOUBLE = c_double
FX_BOOL = c_bool


# 定义 Vect7 类型 (7个double的数组)
class Vect7(Structure):
    _fields_ = [("data", FX_DOUBLE * 7)]

    def __init__(self, values=None):
        super().__init__()
        if values is not None:
            if len(values) != 7:
                raise ValueError("Vect7 requires exactly 7 values")
            for i, val in enumerate(values):
                self.data[i] = val

    def to_list(self):
        return [self.data[i] for i in range(7)]

    def __str__(self):
        return str(self.to_list())


# 定义 Matrix4 类型 (4x4矩阵，16个double)
class Matrix4(Structure):
    _fields_ = [("data", FX_DOUBLE * 16)]

    def __init__(self, values=None):
        super().__init__()
        if values is not None:
            if len(values) != 16:
                raise ValueError("Matrix4 requires exactly 16 values")
            for i, val in enumerate(values):
                self.data[i] = val

    def to_list(self):
        return [self.data[i] for i in range(16)]

    def __str__(self):
        return str(self.to_list())


# 定义主结构体 FX_InvKineSolvePara
class FX_InvKineSolvePara(Structure):
    _fields_ = [
        ("m_Input_IK_TargetTCP", Matrix4),
        ("m_Input_IK_RefJoint", Vect7),
        ("m_Input_IK_ZSPType", FX_INT32L),
        ("m_Input_IK_ZSPPara", FX_DOUBLE * 6),
        ("m_Input_ZSP_Angle", FX_DOUBLE),
        ("m_DGR1", FX_DOUBLE),
        ("m_DGR2", FX_DOUBLE),
        ("m_DGR3", FX_DOUBLE),
        ("m_Output_RetJoint", Vect7),
        ("m_Output_IsOutRange", FX_BOOL),
        ("m_Output_IsDeg", FX_BOOL * 7),
        ("m_Output_JntExdTags", FX_BOOL * 7),
        ("m_Output_IsJntExd", FX_BOOL),
        ("m_Output_RunLmtP", Vect7),
        ("m_Output_RunLmtN", Vect7)
    ]

    def __init__(self):
        super().__init__()
        # 初始化数组
        for i in range(6):
            self.m_Input_IK_ZSPPara[i] = 0.0

        # 初始化布尔数组
        for i in range(7):
            self.m_Output_IsDeg[i] = False
            self.m_Output_JntExdTags[i] = False

    def set_input_ik_zsp_para(self, values):
        if len(values) != 6:
            raise ValueError("m_Input_IK_ZSPPara requires exactly 6 values")
        for i, val in enumerate(values):
            self.m_Input_IK_ZSPPara[i] = val

    def get_input_ik_zsp_para(self):
        return [self.m_Input_IK_ZSPPara[i] for i in range(6)]

    def set_output_is_deg(self, values):
        if len(values) != 7:
            raise ValueError("m_Output_IsDeg requires exactly 7 values")
        for i, val in enumerate(values):
            self.m_Output_IsDeg[i] = val

    def get_output_is_deg(self):
        return [self.m_Output_IsDeg[i] for i in range(7)]

    def set_output_jnt_exd_tags(self, values):
        if len(values) != 7:
            raise ValueError("m_Output_JntExdTags requires exactly 7 values")
        for i, val in enumerate(values):
            self.m_Output_JntExdTags[i] = val

    def get_output_jnt_exd_tags(self):
        return [self.m_Output_JntExdTags[i] for i in range(7)]

# 定义 FX_Jacobi 结构体
class FX_Jacobi(Structure):
    _fields_ = [
        ("m_AxisNum", FX_INT32L),
        ("m_Jcb", (FX_DOUBLE * 7) * 6)  # 6x7 二维数组
    ]

    def __init__(self):
        super().__init__()
        self.m_AxisNum = 0
        # 初始化二维数组为0
        for i in range(6):
            for j in range(7):
                self.m_Jcb[i][j] = 0.0

    def set_jcb(self, matrix):
        """
        设置雅可比矩阵的值

        参数:
        matrix: 6x7 二维列表或numpy数组
        """
        if len(matrix) != 6 or any(len(row) != 7 for row in matrix):
            raise ValueError("雅可比矩阵必须是6x7的二维数组")

        for i in range(6):
            for j in range(7):
                self.m_Jcb[i][j] = matrix[i][j]

    def get_jcb(self):
        """
        获取雅可比矩阵的值

        返回:
        6x7 二维列表
        """
        result = []
        for i in range(6):
            row = []
            for j in range(7):
                row.append(self.m_Jcb[i][j])
            result.append(row)
        return result

    def __str__(self):
        """
        返回雅可比矩阵的字符串表示
        """
        result = f"AxisNum: {self.m_AxisNum}\nJacobian Matrix:\n"
        for i in range(6):
            row = [f"{self.m_Jcb[i][j]:.6f}" for j in range(7)]
            result += "  " + "  ".join(row) + "\n"
        return result


class Marvin_Kine:
    def __init__(self):
        """初始化机器人控制类"""
        import sys
        logger.info(f'user platform: {sys.platform}')
        if sys.platform == 'win32':
            self.kine = ctypes.WinDLL(os.path.join(current_path, 'python/libKine.dll'))
        else:
            self.kine = ctypes.CDLL(os.path.join(current_path, 'python/libKine.so'))

        # 创建结构体实例
        self.sp = FX_InvKineSolvePara()
        self.jacobi = FX_Jacobi()
        self.jacobi_dot = FX_Jacobi()

    def help(self, method_name: str = None) -> None:
        """显示帮助信息
        参数:method_name (str): 可选的方法名，显示特定方法的帮助信息
        """
        print(f"\n{' API 帮助 ':=^50}\n")

        # 获取所有公共方法
        methods = [
            (name, func)
            for name, func in inspect.getmembers(self, inspect.ismethod)
            if not name.startswith('_') and name != 'help'
        ]

        # 如果没有指定方法名，显示所有方法列表
        if method_name is None:
            print("可用方法:")
            for name, func in methods:
                # 获取函数签名
                signature = inspect.signature(func)
                # 获取参数列表
                params = []
                for param in signature.parameters.values():
                    param_str = param.name
                    if param.default is not param.empty:
                        param_str += f"={param.default!r}"
                    if param.annotation is not param.empty:
                        param_str += f": {param.annotation.__name__}"
                    if param.kind == param.VAR_POSITIONAL:
                        param_str = "*" + param_str
                    elif param.kind == param.VAR_KEYWORD:
                        param_str = "**" + param_str
                    elif param.kind == param.KEYWORD_ONLY:
                        param_str = "[kw] " + param_str
                    params.append(param_str)

                param_list = ", ".join(params)
                print(f"  - {name}({param_list})")

            print("\n使用 help('方法名') 获取详细帮助信息")
            print(f"{'=' * 50}")
            return

        # 显示特定方法的帮助
        method_dict = dict(methods)
        if method_name in method_dict:
            func = method_dict[method_name]
            doc = inspect.getdoc(func) or "没有文档说明"

            # 获取函数签名
            signature = inspect.signature(func)

            print(f"方法: {method_name}{signature}")
            print("\n" + dedent(doc))

            # 显示参数详细信息
            print("\n参数详情:")
            for param in signature.parameters.values():
                param_info = f"  {param.name}: "
                if param.annotation is not param.empty:
                    param_info += f"类型: {param.annotation.__name__}, "
                if param.default is not param.empty:
                    param_info += f"默认值: {param.default!r}"
                # param_info += f"类型: {_param_kind_to_str(param.kind)}"
                print(param_info)
        else:
            print(f"错误: 没有找到方法 '{method_name}'")

        print(f"{'=' * 50}")

    def _param_kind_to_str(kind):
        """将参数类型转换为可读字符串"""
        mapping = {
            inspect.Parameter.POSITIONAL_ONLY: "位置参数",
            inspect.Parameter.POSITIONAL_OR_KEYWORD: "位置或关键字参数",
            inspect.Parameter.VAR_POSITIONAL: "可变位置参数(*args)",
            inspect.Parameter.KEYWORD_ONLY: "仅关键字参数",
            inspect.Parameter.VAR_KEYWORD: "可变关键字参数(**kwargs)"
        }
        return mapping.get(kind, "未知参数类型")

    def load_config(self, config_path: str):
        ''' 使用前，请一定确认机型，导入正确的配置文件。导入机械臂配置信息
        :param config_path: 本地机械臂配置文件a.MvKDCfg, 可相对路径.
        • a.MvKDCfg文件中包含与运动学、动力学计算相关的双臂参数，进行计算之前需要导入机械臂配置相关文件
        • TYPE=1006，DL机型；TYPE=1007，Pilot-SRS机型（双臂为MARVIN）；TYPE=1017，Pilot-CCS机型双臂为MARVIN）！
        • GRV参数为双臂重力方向，如[0.000,9.810,0.000];DH参数为双臂MDH参数，包含各关节MDH参数及法兰MDH参数；PNVA参数为双臂各关节所允许的正负最大加速度及加加速度；BD参数为Pilot-CCS机型特定参数，为六七关节自干涉允许范围的拟合二阶多项式曲线，其他机型中该参数均为0；Mass参数为双臂各关节质量；MCP参数为双臂各关节质心；I参数为双臂各关节惯量
        • MDH参数单位为度和毫米（mm），速度加速度单位为度/秒，关节质量、关节质心、关节惯量单位均为国际标准单位
        :return:
        '''

        if not os.path.exists(config_path):
            raise ValueError("no config file")

        # 定义函数原型
        self.kine.LOADMvCfg.argtypes = [
            c_char_p,  # FX_CHAR* path
            ctypes.POINTER(c_long * 2),  # FX_INT32L TYPE[2]
            ctypes.POINTER((c_double * 3) * 2),  # FX_DOUBLE GRV[2][3]
            ctypes.POINTER(((c_double * 4) * 8) * 2),  # FX_DOUBLE DH[2][8][4]
            ctypes.POINTER(((c_double * 4) * 7) * 2),  # FX_DOUBLE PNVA[2][7][4]
            ctypes.POINTER(((c_double * 3) * 4) * 2),  # FX_DOUBLE BD[2][4][3]
            ctypes.POINTER((c_double * 7) * 2),  # FX_DOUBLE Mass[2][7]
            ctypes.POINTER(((c_double * 3) * 7) * 2),  # FX_DOUBLE MCP[2][7][3]
            ctypes.POINTER(((c_double * 6) * 7) * 2)  # FX_DOUBLE I[2][7][6]
        ]
        self.kine.LOADMvCfg.restype = c_bool  # 返回类型FX_BOOL

        # 初始化所有数组参数
        TYPE = (c_long * 2)()
        GRV = ((c_double * 3) * 2)()
        DH = (((c_double * 4) * 8) * 2)()
        PNVA = (((c_double * 4) * 7) * 2)()
        BD = (((c_double * 3) * 4) * 2)()
        Mass = ((c_double * 7) * 2)()
        MCP = (((c_double * 3) * 7) * 2)()
        I = (((c_double * 6) * 7) * 2)()

        # 调用函数

        success = self.kine.LOADMvCfg(
            config_path.encode('utf-8'),
            ctypes.byref(TYPE),
            ctypes.byref(GRV),
            ctypes.byref(DH),
            ctypes.byref(PNVA),
            ctypes.byref(BD),
            ctypes.byref(Mass),
            ctypes.byref(MCP),
            ctypes.byref(I)
        )

        # 处理结果
        if success:
            result = {
                'TYPE': [TYPE[i] for i in range(2)],
                'GRV': [[GRV[i][j] for j in range(3)] for i in range(2)],
                'DH': [[[DH[i][j][k] for k in range(4)] for j in range(8)] for i in range(2)],
                'PNVA': [[[PNVA[i][j][k] for k in range(4)] for j in range(7)] for i in range(2)],
                'BD': [[[BD[i][j][k] for k in range(3)] for j in range(4)] for i in range(2)],
                'Mass': [[Mass[i][j] for j in range(7)] for i in range(2)],
                'MCP': [[[MCP[i][j][k] for k in range(3)] for j in range(7)] for i in range(2)],
                'I': [[[I[i][j][k] for k in range(6)] for j in range(7)] for i in range(2)]
            }
            logger.info("Load config successful")
            return result
        else:
            logger.error("Load config failed")
            return None

    def initial_kine(self, robot_serial: int, robot_type: int, dh: list, pnva: list, j67: list):
        '''初始化运动学相关参数
        • 运动学相关计算前，需要按照该顺序调用初始化函数，将配置中导入的参数进行初始化
        • RobotSerial=0，左臂；RobotSerial=1，右臂
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param type: int.机器人机型代号
        :param dh: list(8,4), 每个轴DH：alpha, a d,theta.
        :param pnva: list(7,4), 每个轴:关节上界p,关节下界n，最大速度v,最大加速度a.
        :param j67: list(4,3),仅CCS机型生效， 67关节干涉限制。
        :return:
            bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if type(robot_type) != int:
            raise ValueError("robot_type  must be int type")


        if len(dh) != 8:
            raise ValueError("dh  must be 8 rows")
        else:
            for i in range(len(dh)):
                if len(dh[i]) != 4:
                    raise ValueError("dh  must be 4 columns")

        if len(pnva) != 7:
            raise ValueError("pnva  must be 7 rows")
        else:
            for i in range(len(pnva)):
                if len(pnva[i]) != 4:
                    raise ValueError("pnva  must be 4 columns")

        if len(j67) != 4:
            raise ValueError("j67  must be 4 rows")
        else:
            for i in range(len(j67)):
                if len(j67[i]) != 3:
                    raise ValueError("j67  must be 3 columns")

        Serial = ctypes.c_long(robot_serial)
        robot_type_ = c_long(robot_type)

        DH = ((c_double * 4) * 8)()
        for i in range(8):
            for j in range(4):
                DH[i][j] = dh[i][j]

        PNVA = ((c_double * 4) * 7)()
        for i in range(7):
            for j in range(4):
                PNVA[i][j] = pnva[i][j]

        J67 = ((c_double * 3) * 4)()
        for i in range(4):
            for j in range(3):
                J67[i][j] = j67[i][j]

        ''' ini type'''
        self.kine.FX_Robot_Init_Type.argtypes = [c_long, c_long]
        self.kine.FX_Robot_Init_Type.restype = c_bool
        success1 = self.kine.FX_Robot_Init_Type(Serial, robot_type_)

        ''' ini dh'''
        # FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4]);
        self.kine.FX_Robot_Init_Kine.argtypes = [c_long, (c_double * 4) * 8]
        self.kine.FX_Robot_Init_Kine.restype = c_bool
        success2 = self.kine.FX_Robot_Init_Kine(Serial, DH)

        ''' ini Lmt'''
        # FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3]);
        self.kine.FX_Robot_Init_Lmt.argtypes = [c_long, (c_double * 4) * 7, (c_double * 3) * 4]
        self.kine.FX_Robot_Init_Lmt.restype = c_bool
        success3 = self.kine.FX_Robot_Init_Lmt(Serial, PNVA, J67)

        # print(success1,success2,success3)
        if success1 and success2 and success3:
            logger.info('Initial kinematics successful')
            return True
        elif not success1:
            logger.error('Initial kinematics failed:FX_Robot_Init_Type')
            return False
        elif not success2:
            logger.error('Initial kinematics failed:FX_Robot_Init_Kine')
            return False
        elif not success3:
            logger.error('Initial kinematics failed:FX_Robot_Init_Lmt')
            return False

    def set_tool_kine(self, robot_serial: int, tool_mat: list):
        '''工具运动学设置
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param tool_mat: list(4,4) 工具的运动学信息，齐次变换矩阵，相对末端法兰的旋转和平移，请确认法兰坐标系。
        :return:bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(tool_mat) != 4:
            raise ValueError("tool_mat  must be 4 rows")
        else:
            for i in range(len(tool_mat)):
                if len(tool_mat[i]) != 4:
                    raise ValueError("tool_mat  must be 4 columns")

        Serial = ctypes.c_long(robot_serial)

        TOOL = ((c_double * 4) * 4)()
        for i in range(4):
            for j in range(4):
                TOOL[i][j] = tool_mat[i][j]

        '''set tool'''
        self.kine.FX_Robot_Tool_Set.argtypes = [c_long, (c_double * 4) * 4]
        self.kine.FX_Robot_Tool_Set.restype = c_bool
        success1 = self.kine.FX_Robot_Tool_Set(Serial, TOOL)
        if success1:
            logger.info('set tool kinematics info successful')
            return True
        else:
            logger.error('set tool kinematics info failed!')
            return False

    def remove_tool_kine(self, robot_serial: int):
        '''移除工具运动学设置
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :return:bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        Serial = ctypes.c_long(robot_serial)
        '''remove tool'''
        self.kine.FX_Robot_Tool_Rmv.argtypes = [c_long]
        self.kine.FX_Robot_Tool_Rmv.restype = c_bool
        success1 = self.kine.FX_Robot_Tool_Rmv(Serial)
        if success1:
            logger.info('remove tool kinematics info successful')
            return True
        else:
            logger.error('remove tool kinematics info failed!')
            return False



    def fk(self, robot_serial: int, joints: list):
        '''关节角度正解到末端TCP位置和姿态4*4
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param joints: list(7,1). 角度值
        :return:
            4x4的位姿矩阵，list(4,4)
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(joints) != 7:
            raise ValueError("shape error: fk input joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)

        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_double = (ctypes.c_double * 7)(j0, j1, j2, j3, j4, j5, j6)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        self.kine.FX_Robot_Kine_FK.argtypes = [c_long,
                                               ctypes.POINTER(ctypes.c_double * 7),
                                               ctypes.POINTER((ctypes.c_double * 4) * 4)]
        self.kine.FX_Robot_Kine_FK.restype = c_bool
        success1 = self.kine.FX_Robot_Kine_FK(Serial, ctypes.byref(joints_double), ctypes.byref(pg))
        if success1:
            fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
            for i in range(4):
                for j in range(4):
                    fk_mat[i][j] = pg[i][j]
            # logger.info(f'fk result, matrix:{fk_mat}')
            return fk_mat
        else:
            return False

    def ik(self, robot_serial: int, pose_mat: list, ref_joints: list):
        '''末端位置和姿态逆解到关节值
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param pose_mat: list(4,4), 位置姿态4x4list.
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :return:
            结构体，以下几项最相关：
                m_Output_RetJoint      ：逆运动学解出的关节角度（单位：度）
                m_Output_IsOutRange    ：当前位姿是否超出位置可达空间（False：未超出；True：超出）
                m_Output_IsDeg[7]      ：各关节是否发生奇异（False：未奇异；True：奇异）:
                m_Output_IsJntExd      : 是否有关节超出位置正负限制（False：未超出；True：超出）:
                m_Output_JntExdTags[7] ：各关节是否超出位置正负限制（False：未超出；True：超出）:
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(pose_mat) != 4:
            raise ValueError("pose_mat  must be 4 rows")
        else:
            for i in range(len(pose_mat)):
                if len(pose_mat[i]) != 4:
                    raise ValueError("pose_mat  must be 4 columns")

        if len(ref_joints) != 7:
            raise ValueError("ref_joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)
        # 将 4x4 矩阵数据复制到 sp.m_Input_IK_TargetTCP
        matrix_data = []
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])

        self.sp.m_Input_IK_TargetTCP = Matrix4(matrix_data)

        # 将关节角度值复制到 sp.m_Input_IK_RefJoint
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        jv = (c_double * 7)(j0_, j1_, j2_, j3_, j4_, j5_, j6_)
        self.sp.m_Input_IK_RefJoint = Vect7(jv)

        # 调用逆运动学函数
        self.kine.FX_Robot_Kine_IK.argtypes = [c_long, POINTER(FX_InvKineSolvePara)]
        self.kine.FX_Robot_Kine_IK.restype = c_bool
        success = self.kine.FX_Robot_Kine_IK(Serial, byref(self.sp))
        if not success:
            logger.error("Robot Inverse Kinematics Error")
            return False
        else:
            logger.error("Robot Inverse Kinematics excess!")
            logger.info(f"ik joints:{self.sp.m_Output_RetJoint.to_list()}")
            return self.sp

    def ik_nsp(self, robot_serial: int, pose_mat: list, ref_joints: list, zsp_type: int, zsp_para: list,
               zsp_angle: float, dgr: list):
        '''逆解优化：可调整方向,不能单独使用，ik得到的逆运动学解的臂角不满足当前选解需求时使用。
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param pose_mat: list(4,4), 位置姿态4x4list.
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :param zsp_type: int, 零空间约束类型（0：与参考角欧式距离最小；1：与参考臂角平面最近）
        :param zsp_para: list(6,), 若选择零空间约束类型为1，则需额外输入参考角平面参数,[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
        :param zsp_angle: float, 末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）
        :param dgr: list(2,), 选择123关节和567关节发生奇异允许的角度范围，如无额外要求无需输入，默认值为0.05（单位：度）
        :return:
            结构体，以下几项最相关：
                m_Output_RetJoint      ：逆运动学解出的关节角度（单位：度）
                m_Output_IsOutRange    ：当前位姿是否超出位置可达空间（False：未超出；True：超出）
                m_Output_IsDeg[7]      ：各关节是否发生奇异（False：未奇异；True：奇异）:
                m_Output_IsJntExd      : 是否有关节超出位置正负限制（False：未超出；True：超出）:
                m_Output_JntExdTags[7] ：各关节是否超出位置正负限制（False：未超出；True：超出）:
        '''

        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(pose_mat) != 4:
            raise ValueError("pose_mat  must be 4 rows")
        else:
            for i in range(len(pose_mat)):
                if len(pose_mat[i]) != 4:
                    raise ValueError("pose_mat  must be 4 columns")

        if len(ref_joints) != 7:
            raise ValueError("ref_joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)

        # 将 4x4 矩阵数据复制到 sp.m_Input_IK_TargetTCP
        matrix_data = []
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])

        self.sp.m_Input_IK_TargetTCP = Matrix4(matrix_data)

        # 将关节角度值复制到 sp.m_Input_IK_RefJoint
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        jv = (c_double * 7)(j0_, j1_, j2_, j3_, j4_, j5_, j6_)
        self.sp.m_Input_IK_RefJoint = Vect7(jv)

        self.sp.m_Input_IK_ZSPType = zsp_type
        if zsp_type == 1:
            p0, p1, p2, p3, p4, p5 = zsp_para
            zsp_para_value = (c_double * 6)(p0, p1, p2, p3, p4, p5)
            self.sp.m_Input_IK_ZSPPara = zsp_para_value
        self.sp.m_Input_ZSP_Angle -= zsp_angle

        dgr1, dgr2 = dgr
        # dgr_value=(c_double*2)(dgr1,dgr2)
        self.sp.m_DGR1 = dgr1
        self.sp.m_DGR2 = dgr2

        self.kine.FX_Robot_Kine_IK_NSP.argtypes = [c_long, POINTER(FX_InvKineSolvePara)]
        self.kine.FX_Robot_Kine_IK_NSP.restype = c_bool
        success = self.kine.FX_Robot_Kine_IK_NSP(Serial, byref(self.sp))
        if not success:
            logger.error("Robot Inverse Kinematics NSP Error")
            return False
        else:
            logger.info("Robot Inverse Kinematics NSP Success")
            logger.info(f"ik joints:{self.sp.m_Output_RetJoint.to_list()}")
            return self.sp

    def joints2JacobMatrix(self, robot_serial: int, joints: list):
        '''当前关节角度转成雅可比矩阵
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param joints: list(7,1), 当前关节
        :return: 雅可比矩阵6*7矩阵
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        if len(joints) != 7:
            raise ValueError("joints must be (7,)")

        Serial = ctypes.c_long(robot_serial)

        joints_double = ctypes.c_double * 7
        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)

        example_matrix = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        ]

        # 设置雅可比矩阵
        self.jacobi.set_jcb(example_matrix)

        self.kine.FX_Robot_Kine_Jacb.argtypes = [c_long, c_double * 7, POINTER(FX_Jacobi)]
        self.kine.FX_Robot_Kine_Jacb.restype = c_bool
        success = self.kine.FX_Robot_Kine_Jacb(Serial, joints_value, byref(self.jacobi))

        if not success:
            logger.error("Joints2Jacobi Error")
            return False
        else:
            logger.info("Joints2Jacobi Success")
            logger.info(f"Jacobi matrix:{self.jacobi.get_jcb()}")
            return self.jacobi.get_jcb()


    def mat4x4_to_xyzabc(self,pose_mat:list):
        '''末端位置和姿态转XYZABC
        :param pose_mat: list(4,4), 位置姿态4x4list.
        :return:
                （6,1）位姿信息XYZ及欧拉角ABC（单位：mm/度）
        '''
        if len(pose_mat) != 4:
            raise ValueError("pose_mat  must be 4 rows")
        else:
            for i in range(len(pose_mat)):
                if len(pose_mat[i]) != 4:
                    raise ValueError("pose_mat  must be 4 columns")

        matrix_data =( (c_double*4)*4)()
        for i in range(4):
            for j in range(4):
                matrix_data[i][j]=pose_mat[i][j]

        xyzabc=(c_double*6)(0,0,0,0,0,0)

        self.kine.FX_Matrix42XYZABCDEG.argtypes = [(c_double*4)*4,c_double*6]
        self.kine.FX_Matrix42XYZABCDEG.restype = c_bool
        success = self.kine.FX_Matrix42XYZABCDEG(matrix_data,xyzabc)

        if not success:
            logger.error("Pose mat to xyzabc Error")
            return False
        else:
            # logger.info("Pose mat to xyzabc Success")

            pose_6d=[xyzabc[i] for i in range(6)]
            # logger.info(f"xyzabc:{pose_6d}")
            return pose_6d


    def xyzabc_to_mat4x4(self,xyzabc:list):
        '''末端XYZABC转位置和姿态矩阵
        param xyzabc: list(6,),
        return:
            mat4x4  list(4,4)

        '''
        if len(xyzabc) != 6:
            raise ValueError("length of xyzabc must be 6")

        j0, j1, j2, j3, j4, j5 = xyzabc
        joints_double = (ctypes.c_double * 6)(j0, j1, j2, j3, j4, j5)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        self.kine.FX_XYZABC2Matrix4DEG.argtypes = [ctypes.POINTER(ctypes.c_double * 6),
                                     ctypes.POINTER((ctypes.c_double * 4) * 4)]

        self.kine.FX_XYZABC2Matrix4DEG(ctypes.byref(joints_double), ctypes.byref(pg))
        fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        for i in range(4):
            for j in range(4):
                fk_mat[i][j] = pg[i][j]
        if not fk_mat:
            logger.error("xyzabc to mat4x4 Error")
            return False
        else:
            logger.info("xyzabc to mat4x4 Success")
            return fk_mat


    def movL(self,robot_serial: int,start_xyzabc:list, end_xyzabc:list,ref_joints:list,vel:float,acc:float,save_path):
        '''直线规划（MOVL）

        :param robot_serial: int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param start_xyzabc:
        :param end_xyzabc:
        :param ref_joints:
        :param vel:
        :param acc:
        :param save_path:
        :return: bool
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        Serial = ctypes.c_long(robot_serial)

        path = save_path.encode('utf-8')
        path_char = ctypes.c_char_p(path)

        s0,s1,s2,s3,s4,s5=start_xyzabc
        start= (ctypes.c_double * 6)( s0,s1,s2,s3,s4,s5)

        e0,e1,e2,e3,e4,e5=end_xyzabc
        end= (ctypes.c_double * 6)(e0,e1,e2,e3,e4,e5)

        vel_value=c_double(vel)

        acc_value=c_double(acc)

        j0, j1, j2, j3, j4, j5, j6 = ref_joints
        joints_vel_value = (c_double * 7)(j0, j1, j2, j3, j4, j5, j6)

        self.kine.FX_Robot_PLN_MOVL.argtypes=[c_long,c_double*6,c_double*6,c_double*7,c_double,c_double,c_char_p]
        self.kine.FX_Robot_PLN_MOVL.restype=c_bool
        success1=self.kine.FX_Robot_PLN_MOVL(Serial,start,end,joints_vel_value,vel_value,acc_value,path_char)
        if success1:
            logger.info(f'Plan MOVL successful, PATH saved as :{save_path}')
            return  True

        else:
            logger.error(f'Plan MOVL failed!')
            return False

    def identify_tool_dyn(self, robot_type: int, ipath: str):
        '''工具动力学参数辨识
        :param robot_type: int . 1:CCS机型，2:SRS机型
        :param ipath: sting, 相对路径导入工具辨识轨迹数据。
        :return:
            m,mcp,i
        错误返回参考:
        //typedef enum {
        //	LOAD_IDEN_NoErr = 0, // No error
        //	LOAD_IDEN_CalErr = 1, // Calculation error, 计算错误，需重新采集数据计算
        //	LOAD_IDEN_OpenSmpDateFieErr = 2, //  Open sample file error 打开采集数据文件错误，须检查采样文件
        //	LOAD_IDEN_OpenCfgFileErr = 3, // Open config file error 配置文件被修改
        //	LOAD_IDEN_DataSmpErr = 4 // Data sample error 采集时间不够，缺少有效数据
        //}LoadIdenErrCode;
        '''
        if type(robot_type) != int:
            raise ValueError("robot_type must be int type")

        if not os.path.exists(ipath):
            raise ValueError(f"no {ipath}, pls check!")

        if robot_type == 1:
            print(f'CCS tool identy')
        elif robot_type == 2:
            print(f'SRS tool identy')

        robot_type_ = c_int(robot_type)
        iden_path = ipath.encode('utf-8')
        path_char = ctypes.c_char_p(iden_path)

        # 创建指针变量而不是数组
        mm_ptr = pointer(c_double(0))
        mcp_ptr = (c_double * 3)()
        ii_ptr = (c_double * 6)()

        # 设置函数原型
        self.kine.FX_Robot_Iden_LoadDyn.argtypes = [
            c_int,
            c_char_p,
            POINTER(c_double),
            POINTER(c_double * 3),
            POINTER(c_double * 6)
        ]
        self.kine.FX_Robot_Iden_LoadDyn.restype = c_int32

        # 调用函数
        ret_int = self.kine.FX_Robot_Iden_LoadDyn(
            robot_type_,
            path_char,
            mm_ptr,
            mcp_ptr,
            ii_ptr
        )
        if ret_int == 0:
            logger.info('Identify tool dynamics successful')

            # 提取结果
            dyn_para = []
            m_val = mm_ptr.contents.value
            mcp_list = [mcp_ptr[i] for i in range(3)]
            ii_list = [ii_ptr[i] for i in range(6)]
            'ixx iyy izz ixy ixz iyz'

            dyn_para.append(m_val)
            for i in mcp_list:
                dyn_para.append(i)

            dyn_para.append(ii_list[0])
            dyn_para.append(ii_list[3])
            dyn_para.append(ii_list[4])
            dyn_para.append(ii_list[1])
            dyn_para.append(ii_list[5])
            dyn_para.append(ii_list[2])

            logger.info(f'tool dynamics[m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]: {dyn_para}')
            return True,dyn_para
        else:
            logger.error('Identify tool dynamics failed!')
            logger.error(
                f'identify_tool_dyn 返回错误码:{ret_int}\n ret=1, 计算错误，需重新采集数据计算\n ret=2,打开采集数据文件错误，须检查采样文件\n ret=3,配置文件被修改\n ret=4, 采集时间不够，缺少有效数据')
            if ret_int == 1:
                return False,'ret=1, 计算错误，需重新采集数据计算'
            elif ret_int == 2:
                return False,'ret=2,打开采集数据文件错误，须检查采样文件'
            elif ret_int == 3:
                return False,"ret=3,配置文件被修改"
            elif ret_int == 4:
                return False,'ret=4, 采集时间不够，缺少有效数据'



if __name__ == "__main__":
    kk = Marvin_Kine()  # 实例化
    kk.help()  # 查看方法
    kk.help('load_config')
    exit()
