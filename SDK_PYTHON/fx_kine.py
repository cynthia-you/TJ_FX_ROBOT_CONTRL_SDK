from ctypes import *
import ctypes
import inspect
from textwrap import dedent
import logging
import os
import sys
from typing import List
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)


# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

class Marvin_Kine:
    def __init__(self):
        """初始化机器人控制类"""
        logger.info(f'user platform: {sys.platform}')
        if sys.platform == 'win32':
            self.kine = ctypes.WinDLL(os.path.join(current_path, 'libKine.dll'))
        else:
            self.kine = ctypes.CDLL(os.path.join(current_path, 'libKine.so'))
        # 创建结构体实例
        self.sp = FX_InvKineSolvePara()
        self.jacobi = FX_Jacobi()
        self.jacobi_dot = FX_Jacobi()

        self.robot_tag=None
        self._setup_function_prototypes()

    def _setup_function_prototypes(self):
        """设置所有C函数的参数类型和返回类型"""

        # 原有的MOVL函数
        self.kine.FX_Robot_PLN_MOVL.argtypes = [
            ctypes.c_long,  # RobotSerial
            ctypes.POINTER(ctypes.c_double),  # Start_XYZABC (6个double)
            ctypes.POINTER(ctypes.c_double),  # End_XYZABC (6个double)
            ctypes.POINTER(ctypes.c_double),  # Ref_Joints (7个double)
            ctypes.c_double,  # Vel
            ctypes.c_double,  # ACC
            ctypes.c_char_p  # OutPutPath
        ]
        self.kine.FX_Robot_PLN_MOVL.restype = ctypes.c_bool

        # CPointSet创建函数
        self.kine.FX_CPointSet_Create.argtypes = []
        self.kine.FX_CPointSet_Create.restype = ctypes.c_void_p

        # CPointSet销毁函数
        self.kine.FX_CPointSet_Destroy.argtypes = [ctypes.c_void_p]
        self.kine.FX_CPointSet_Destroy.restype = None

        # CPointSet初始化函数
        self.kine.FX_CPointSet_OnInit.argtypes = [ctypes.c_void_p, ctypes.c_long]
        self.kine.FX_CPointSet_OnInit.restype = ctypes.c_bool

        # CPointSet获取点数函数
        self.kine.FX_CPointSet_OnGetPointNum.argtypes = [ctypes.c_void_p]
        self.kine.FX_CPointSet_OnGetPointNum.restype = ctypes.c_long

        # CPointSet获取点数据函数
        self.kine.FX_CPointSet_OnGetPoint.argtypes = [ctypes.c_void_p, ctypes.c_long]
        self.kine.FX_CPointSet_OnGetPoint.restype = ctypes.POINTER(ctypes.c_double)

        # C风格的MOVLA函数
        self.kine.FX_Robot_PLN_MOVLA_C.argtypes = [
            ctypes.c_long,  # RobotSerial
            ctypes.POINTER(ctypes.c_double),  # Start_XYZABC
            ctypes.POINTER(ctypes.c_double),  # End_XYZABC
            ctypes.POINTER(ctypes.c_double),  # Ref_Joints
            ctypes.c_double,  # Vel
            ctypes.c_double,  # ACC
            ctypes.c_void_p  # ret_pset
        ]
        self.kine.FX_Robot_PLN_MOVLA_C.restype = ctypes.c_bool

        # C风格的MOVL_KEEPJA函数
        self.kine.FX_Robot_PLN_MOVL_KeepJA_C.argtypes = [
            ctypes.c_long,  # RobotSerial
            ctypes.POINTER(ctypes.c_double),  # startjoints
            ctypes.POINTER(ctypes.c_double),  # stopjoints
            ctypes.c_double,  # Vel
            ctypes.c_double,  # ACC
            ctypes.c_void_p  # ret_pset
        ]
        self.kine.FX_Robot_PLN_MOVL_KeepJA_C.restype = ctypes.c_bool


    def create_point_set(self, point_type: int = 6) -> ctypes.c_void_p:
        """创建CPointSet对象"""
        pset = self.kine.FX_CPointSet_Create()
        if pset:
            # 初始化点集类型，6对应6维数据(x,y,z,a,b,c)
            self.kine.FX_CPointSet_OnInit(pset, point_type)
        return pset

    def destroy_point_set(self, pset: ctypes.c_void_p):
        """销毁CPointSet对象"""
        if pset:
            self.kine.FX_CPointSet_Destroy(pset)

    def get_point_set_data(self, pset: ctypes.c_void_p, dimension: int = 6) -> List[List[float]]:
        """
        从CPointSet对象中获取所有数据

        参数:
            pset: CPointSet指针
            dimension: 每个点的维度，默认6维(x,y,z,a,b,c)
        """
        if not pset:
            return []

        num_points = self.kine.FX_CPointSet_OnGetPointNum(pset)
        data = []

        for i in range(num_points):
            point_ptr = self.kine.FX_CPointSet_OnGetPoint(pset, i)
            if point_ptr:
                # 读取dimension维度的数据
                point = [point_ptr[j] for j in range(dimension)]
                data.append(point)

        return data



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

    def log_switch(self,switch:int):
        '''
        :param switch: 打印日志开：1；打印日志关：0
        '''
        self.kine.FX_LOG_SWITCH.argtypes = [c_long]
        switch_ = c_long(switch)
        self.kine.FX_LOG_SWITCH(switch_)

    def load_config(self, arm_type: int, config_path: str):
        ''' 使用前，请一定确认机型，导入正确的配置文件。导入机械臂配置信息
        :param srm_type: 选择左臂还是右臂, 左臂:0, 右臂:1
        :param config_path: 本地机械臂配置文件a.MvKDCfg, 可相对路径.
        • a.MvKDCfg文件中包含与运动学、动力学计算相关的双臂参数，进行计算之前需要导入机械臂配置相关文件
        • TYPE=1007，Pilot-SRS机型（双臂为MARVIN）；TYPE=1017，Pilot-CCS机型双臂为MARVIN）！
        • GRV参数为双臂重力方向，如[0.000,9.810,0.000];DH参数为双臂MDH参数，包含各关节MDH参数及法兰MDH参数；PNVA参数为双臂各关节所允许的正负最大加速度及加加速度；BD参数为Pilot-CCS机型特定参数，为六七关节自干涉允许范围的拟合二阶多项式曲线，其他机型中该参数均为0；Mass参数为双臂各关节质量；MCP参数为双臂各关节质心；I参数为双臂各关节惯量
        • MDH参数单位为度和毫米（mm），速度加速度单位为度/秒，关节质量、关节质心、关节惯量单位均为国际标准单位
        :return:
        '''

        if arm_type != 0 and arm_type != 1:
            raise ValueError("arm_type must be 0 or 1")

        if not os.path.exists(config_path):
            raise ValueError("no config file")

        self.robot_tag=arm_type

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

    def initial_kine(self, robot_type: int, dh: list, pnva: list, j67: list):
        '''初始化运动学相关参数
        • 运动学相关计算前，需要按照该顺序调用初始化函数，将配置中导入的参数进行初始化
        :param type: int.机器人机型代号
        :param dh: list(8,4), 每个轴DH：alpha, a d,theta.
        :param pnva: list(7,4), 每个轴:关节上界p,关节下界n，最大速度v,最大加速度a.
        :param j67: list(4,3),仅CCS机型生效， 67关节干涉限制。
        :return:
            bool
        '''

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

        Serial = ctypes.c_long(self.robot_tag)
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

    def set_tool_kine(self, tool_mat: list):
        '''工具运动学设置
        :param tool_mat: list(4,4) 工具的运动学信息，齐次变换矩阵，相对末端法兰的旋转和平移，请确认法兰坐标系。添加工具运动学信息后，正解IK将正解工具TCP处。
        :return:bool
        '''
        if len(tool_mat) != 4:
            raise ValueError("tool_mat  must be 4 rows")
        else:
            for i in range(len(tool_mat)):
                if len(tool_mat[i]) != 4:
                    raise ValueError("tool_mat  must be 4 columns")

        Serial = ctypes.c_long(self.robot_tag)

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

    def remove_tool_kine(self):
        '''移除工具运动学设置
        :return:bool
        '''
        Serial = ctypes.c_long(self.robot_tag)
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

    def fk(self,joints: list):
        '''关节角度正解到末端TCP位置和姿态4*4
        :param joints: list(7,1). 角度值，单位：度
        :return:
            末端4x4位姿矩阵，list(4,4)
        '''

        if len(joints) != 7:
            raise ValueError("shape error: fk input joints must be (7,)")

        Serial = ctypes.c_long(self.robot_tag)

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
            logger.info(f'fk result, matrix:{fk_mat}')
            return fk_mat
        else:
            return False

    def fk_nsp(self,joints: list):
        '''关节角度正解到末端TCP位置和姿态4*4，并得到基于该角度下的零空间参数XYZ方矩阵
        :param joints: list(7,1). 角度值，单位：度
        :return:
            末端4x4位姿矩阵，list(4,4)
            零空间参数矩阵 array(3,3), 其中第一列可以作为逆解结构体里面m_Input_IK_ZSPPara的x y z的输入值。
        '''

        if len(joints) != 7:
            raise ValueError("shape error: fk input joints must be (7,)")

        Serial = ctypes.c_long(self.robot_tag)

        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_double = (ctypes.c_double * 7)(j0, j1, j2, j3, j4, j5, j6)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        Matrix3x3 = ((ctypes.c_double * 3) * 3)
        nspg = Matrix3x3()

        self.kine.FX_Robot_Kine_FK_NSP.argtypes = [c_long,
                                               ctypes.POINTER(ctypes.c_double * 7),
                                               ctypes.POINTER((ctypes.c_double * 4) * 4),
                                               ctypes.POINTER((ctypes.c_double * 3) * 3)]
        self.kine.FX_Robot_Kine_FK_NSP.restype = c_bool
        success1 = self.kine.FX_Robot_Kine_FK_NSP(Serial, ctypes.byref(joints_double), ctypes.byref(pg), ctypes.byref(nspg))
        if success1:
            fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
            for i in range(4):
                for j in range(4):
                    fk_mat[i][j] = pg[i][j]

            nsp_mat=[[0, 0, 0], [0, 0, 0], [0, 0, 0]]
            for i in range(3):
                for j in range(3):
                    nsp_mat[i][j] = nspg[i][j]
            logger.info(f'fk_nsp result, matrix:{fk_mat}')
            logger.info(f'nsp matrix:{nsp_mat}')
            return fk_mat,nsp_mat
        else:
            return False

    def ik(self, structure_data):
        '''末端位置和姿态逆解到关节值
        :param 结构体数据
            输入参数：
                m_Input_IK_TargetTCP：末端位置姿态4x4列表，可通过正解接口获取或者指定末端的位置和旋转
                m_Input_IK_RefJoint：参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零
                m_Input_IK_ZSPType：零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
                m_Input_IK_ZSPPara：若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
                m_Input_ZSP_Angle：末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
                m_DGR1：(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR2：(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR3：预留接口

            结构体的输出参数：
                m_Output_RetJoint      :逆运动学解出的关节角度（单位：度）
                m_OutPut_AllJoint      :逆运动学的全部解（每一行代表一组解,分别存放1-7关节的角度值）（单位：度）
                m_Output_IsOutRange    :当前位姿是否超出位置可达空间（False：未超出；True：超出）
                m_OutPut_Result_Num    :逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
                m_Output_IsDeg[7]      :各关节是否发生奇异（False：未奇异；True：奇异）
                m_Output_IsJntExd      :是否有关节超出位置正负限制（False：未超出；True：超出）
                m_Output_JntExdTags[7] :各关节是否超出位置正负限制（False：未超出；True：超出）
                m_Output_RunLmtP       :各个关节运行的正限位, 可作为计算六七关节的干涉参考最大限制
                m_Output_RunLmtN       :各个关节运行的负限位，可作为计算六七关节的干涉参考最大限制

         输出：
            成功：True/1; 失败：False/0
            失败情况:
                    1. 输入矩阵超出机器人可达关节空间
                    2. 第四关节为0, 奇异

        • 特别提示:
                结构体以下输出项的TAG仅绑定对m_Output_RetJoint输出的关节描述
                    • m_Output_IsOutRange     :用于判断当前位姿是否超出位置可达空间（0：未超出；1：超出）
                    • m_Output_IsDeg[7]       :用于判断各关节是否发生奇异（0：未奇异；1：奇异）
                    • m_Output_JntExdABS      :各关节超限绝对值总和(FX_Robot_PLN_MOVL_KeepJ使用)
                    • m_Output_IsJntExd       :用于判断是否有关节超出位置正负限制（0：未超出；1：超出）
                    • m_Output_JntExdTags[7]  :用于判断各关节是否超出位置正负限制（0：未超出；1：超出）

                如果选用用多组解m_OutPut_AllJoint. 请自行对选的解做判断,符合以下三个条件才能控制机械臂正常驱动:
                    1. 第二关节的角度不在正负0.05度范围内(在此范围将奇异)
                    2. 对输出的各个关节做软限位判定:
                        调用接口ini_result=kk.load_config(config_path=os.path.join(current_path,'ccs_m6_31.MvKDCfg'))后,
                        ini_result['PNVA'][:]矩阵里的请两列对应各个关节的正负限位
                        选取的解的每个关节都满足在限位置内
                    3. 如果条件1和2都满足,还要做六七关节干涉判定:
                        判定方法:
                            调用接口ini_result=kk.load_config(config_path=os.path.join(current_path,'ccs_m6_31.MvKDCfg'))后,
                            ini_result['BD'][:]矩阵里依次为++, -+,  --, +- 四个象限的干涉参数
                            以CCS为例:
                                如果选的解的六七关节都为正, 则选用在++象限里的参数:[0.018004, -2.3205, 108.0],三个参数分别视为a0,a1,a2,
                                第6关节的值为j6,此时使用公式j7=(a0^2)*j6+ a1*j6+a2  将得到第7个关节的最大限制位置
                                如果选取的解里面的第7关节小于j7, 则不发生干涉, 本组解可被驱动到达.
        '''
        Serial = ctypes.c_long(self.robot_tag)
        self.sp=structure_data
        # 调用逆运动学函数
        self.kine.FX_Robot_Kine_IK.argtypes = [c_long, POINTER(FX_InvKineSolvePara)]
        self.kine.FX_Robot_Kine_IK.restype = c_bool
        success = self.kine.FX_Robot_Kine_IK(Serial, byref(self.sp))
        if not success:
            logger.error("Robot Inverse Kinematics Error")
            if self.sp.m_Output_IsOutRange==1:
                logger.info(f'IK m_Output_IsOutRange:{self.sp.m_Output_IsOutRange}')
                logger.info("Robot Inverse Kinematics excess!")

            if self.sp.m_Output_IsDeg[3]==1:
                logger.info(f'IK Joint4 m_Output_IsDeg:{self.sp.m_Output_IsDeg[3]}')
                logger.info("Robot Inverse Kinematics Degen!")
            return False
        else:
            logger.info("Robot Inverse Kinematics success")
            logger.info(f'ik result numbers :{self.sp.m_OutPut_Result_Num}')
            logger.info(f"ik joints(close to reference joints):{self.sp.m_Output_RetJoint.to_list()}")
            all_joint_list = self.sp.m_OutPut_AllJoint.to_list()
            all_joint_8x8 = convert_to_8x8_matrix(all_joint_list)
            logger.info(f'all ik joints:{all_joint_8x8 }')

            return self.sp

    def mat4x4_to_mat1x16(self,pose_mat):
        matrix_data=[]
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])
        return matrix_data

    def ik_nsp(self, sturcture_data):
        '''逆解优化：可调整方向,不能单独使用，ik得到的逆运动学解的臂角不满足当前选解需求时使用。
            输入参数：
                m_Input_IK_TargetTCP：末端位置姿态4x4列表，可通过正解接口获取或者指定末端的位置和旋转
                m_Input_IK_RefJoint：参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零
                m_Input_IK_ZSPType：零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
                m_Input_IK_ZSPPara：若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
                m_Input_ZSP_Angle：末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
                m_DGR1：(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR2：(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR3：预留接口

            结构体的输出参数：
                m_Output_RetJoint      :逆运动学解出的关节角度（单位：度）
                m_OutPut_AllJoint      :逆运动学的全部解（每一行代表一组解,分别存放1-7关节的角度值）（单位：度）
                m_Output_IsOutRange    :当前位姿是否超出位置可达空间（False：未超出；True：超出）
                m_OutPut_Result_Num    :逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
                m_Output_IsDeg[7]      :各关节是否发生奇异（False：未奇异；True：奇异）
                m_Output_IsJntExd      :是否有关节超出位置正负限制（False：未超出；True：超出）
                m_Output_JntExdTags[7] :各关节是否超出位置正负限制（False：未超出；True：超出）
                m_Output_RunLmtP       :各个关节运行的正限位, 可作为计算六七关节的干涉参考最大限制
                m_Output_RunLmtN       :各个关节运行的负限位，可作为计算六七关节的干涉参考最大限制
        输出：
            成功：True/1; 失败：False/0
        '''
        Serial = ctypes.c_long(self.robot_tag)
        self.sp=sturcture_data
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

    def joints2JacobMatrix(self, joints: list):
        '''当前关节角度转成雅可比矩阵
        :param joints: list(7,1), 当前关节
        :return: 雅可比矩阵6*7矩阵
        '''
        if len(joints) != 7:
            raise ValueError("joints must be (7,)")

        Serial = ctypes.c_long(self.robot_tag)

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
            logger.info("Pose mat to xyzabc Success")

            pose_6d=[xyzabc[i] for i in range(6)]
            logger.info(f"xyzabc:{pose_6d}")
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

    def calculate_end_xyzabc(self,start_xyzabc:list,pose_offset:list, rot_type:int, angle_param:list):
        '''
        • 输入起始点位姿、位置偏移、旋转类型及各轴旋转角度，输出为结束点位姿
        输入：
            1.start_xyzabc：list(6,),起始点位姿（单位：mm/度）
            2. pose_offset:list(3,),末端点相对于起始点位置的偏移（单位：mm）
            3. rot_type:int,自定义旋转类型，见下方FX_ROTATION_TYPE里类型对应的value
            4. angle_param:list(3,),输入各轴旋转角度(例如：FX_ROT_EULER_XYZ类型，Angle_Param[0]为x轴旋转，Angle_Param[1]为y轴旋转，Angle_Param[2]为z轴旋转)
        输出：
            end_xyzabc：list(6,),结束点位姿（单位：mm/度）

        • 建议：如果只有位置平移，可以不使用本接口，直接基于start_xyzabc进行位置偏移对end_xyzabc赋值
        • 旋转类型大致分为三种：
            1.FX_ROT_NULL：不进行姿态变换
            2.FX_ROT_EULER_xxx:欧拉角变换，基于末端坐标系旋转
            3.FX_ROT_FIXED_xxx:固定角变换，基于基坐标系旋转
            enum FX_ROTATION_TYPE
            {
                FX_ROT_NULL = 11,

                FX_ROT_EULER_XYZ = 101,
                FX_ROT_EULER_XZY = 102,
                FX_ROT_EULER_YXZ = 103,
                FX_ROT_EULER_YZX = 104,
                FX_ROT_EULER_ZXY = 105,
                FX_ROT_EULER_ZYX = 106,

                FX_ROT_EULER_XYX = 107,
                FX_ROT_EULER_XZX = 108,
                FX_ROT_EULER_YXY = 109,
                FX_ROT_EULER_YZY = 110,
                FX_ROT_EULER_ZXZ = 111,
                FX_ROT_EULER_ZYZ = 112,

                FX_ROT_FIXED_XYZ = 201,
                FX_ROT_FIXED_XZY = 202,
                FX_ROT_FIXED_YXZ = 203,
                FX_ROT_FIXED_YZX = 204,
                FX_ROT_FIXED_ZXY = 205,
                FX_ROT_FIXED_ZYX = 206,

                FX_ROT_FIXED_XYX = 207,
                FX_ROT_FIXED_XZX = 208,
                FX_ROT_FIXED_YXY = 209,
                FX_ROT_FIXED_YZY = 210,
                FX_ROT_FIXED_ZXZ = 211,
                FX_ROT_FIXED_ZYZ = 212,
            };
        • 本接口输出的结束点位姿可以直接输入直线规划接口进行规划
        '''
        if len(start_xyzabc) != 6:
            raise ValueError("start_xyzabc must be (6,)")
        if len(pose_offset) != 3:
            raise ValueError("pose_offset must be (3,)")
        if len(angle_param) != 3:
            raise ValueError("angle_param must be (3,)")
        if type(rot_type)!=int:
            raise ValueError("rot_type must be (3,)")

        s0, s1, s2, s3, s4, s5 = start_xyzabc
        start = (ctypes.c_double * 6)(s0, s1, s2, s3, s4, s5)
        ps0,ps1,ps2=pose_offset
        pose_off=(ctypes.c_double * 3)(ps0,ps1,ps2)
        angle0,angle1,angle2=angle_param
        angle=(ctypes.c_double * 3)(angle0,angle1,angle2)
        rot = ctypes.c_long(rot_type)

        end_xyzabc = (c_double * 6)(0, 0, 0, 0, 0, 0)
        self.kine.FX_Robot_CalEndXYZABC.argtypes = [c_double * 6, c_double * 3,c_long, c_double * 3,c_double*6]
        self.kine.FX_Robot_CalEndXYZABC.restype = c_bool
        success = self.kine.FX_Robot_CalEndXYZABC(start,pose_off,rot,angle,end_xyzabc)

        if not success:
            logger.error("get end_xyzabc Error")
            return False
        else:
            logger.info("get end_xyzabc Success")

            pose_6d = [end_xyzabc[i] for i in range(6)]
            logger.info(f"end_xyzabc:{pose_6d}")
            return pose_6d


    def movL(self,start_xyzabc:list, end_xyzabc:list,ref_joints:list,vel:float,acc:float,save_path):
        '''直线规划，规划文件的频率500Hz，即每2ms执行一行
        :param start_xyzabc:起始点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param end_xyzabc:结束点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param ref_joints:参考关节构型，也是规划文件的起始点位。
        :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为1000 mm/s^2
        :param save_path:保存的规划文件的路径
        :return: bool
        特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
                2 输出规划文件的频率为500Hz
                3 movL的特点在于根据提供的起始目标笛卡尔位姿和终止目标笛卡尔位姿规划一段直线路径点，该接口不约束到达终点时的机器人构型。
        '''
        Serial = ctypes.c_long(self.robot_tag)

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
            if os.path.exists(save_path):
                logger.info(f'Plan MOVL successful, PATH saved as :{save_path}')
                return True
        else:
            logger.error(f'Plan MOVL failed!')
            return False

    def movLA(self, start_xyzabc: List[float], end_xyzabc: List[float],
              ref_joints: List[float], vel: float, acc: float,
              dimension: int = 7) -> List[List[float]]:

        '''直线规划，执行MOVLA规划并返回点集数据(频率500Hz)
        :param start_xyzabc:起始点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param end_xyzabc:结束点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param ref_joints:参考关节构型，也是规划文件的起始点位。
        :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为1000 mm/s^2
        :return: 规划得到的点集列表
        特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
                2 输出规划文件的频率为500Hz
                3 movL的特点在于根据提供的起始目标笛卡尔位姿和终止目标笛卡尔位姿规划一段直线路径点，该接口不约束到达终点时的机器人构型。
        '''
        Serial = ctypes.c_long(self.robot_tag)

        # 创建起点数组
        if len(start_xyzabc) != 6:
            raise ValueError("start_xyzabc must have 6 elements")
        start_array = (ctypes.c_double * 6)(*start_xyzabc)

        # 创建终点数组
        if len(end_xyzabc) != 6:
            raise ValueError("end_xyzabc must have 6 elements")
        end_array = (ctypes.c_double * 6)(*end_xyzabc)

        # 创建关节角数组
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have 7 elements")
        joints_array = (ctypes.c_double * 7)(*ref_joints)

        vel_value = ctypes.c_double(vel)
        acc_value = ctypes.c_double(acc)

        # 创建CPointSet对象
        pset = self.create_point_set(dimension)
        if not pset:
            raise RuntimeError("Failed to create CPointSet object")

        try:
            # 调用规划函数
            success = self.kine.FX_Robot_PLN_MOVLA_C(
                Serial,
                ctypes.cast(start_array, ctypes.POINTER(ctypes.c_double)),
                ctypes.cast(end_array, ctypes.POINTER(ctypes.c_double)),
                ctypes.cast(joints_array, ctypes.POINTER(ctypes.c_double)),
                vel_value,
                acc_value,
                pset
            )

            if success:
                # 获取点集数据
                data = self.get_point_set_data(pset, dimension)
                print(f'Plan MOVLA successful, got {len(data)} points')
                return data
            else:
                print('Plan MOVLA failed!')
                return []
        finally:
            # 确保清理资源
            self.destroy_point_set(pset)

    def movL_KeepJ(self,start_joints:list, end_joints:list,vel:float,acc:float,save_path):
        '''直线规划保持关节构型, 规划文件的点位频率50Hz，即每20ms执行一行

        :param start_joints:起始点各个关节位置（单位：角度）
        :param end_joints:终点各个关节位置（单位：角度）
        :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为1000 mm/s^2
        :param save_path:规划文件的保存路径
        :return: bool
        特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
                2 输出点位频率为500Hz
                3 该接口是不同于MOVL的规划接口，movL_KeepJ根据起始关节和结束关节规划一条直线路径。
        '''

        Serial = ctypes.c_long(self.robot_tag)

        path = save_path.encode('utf-8')
        path_char = ctypes.c_char_p(path)

        s0,s1,s2,s3,s4,s5,s6=start_joints
        start= (ctypes.c_double * 7)( s0,s1,s2,s3,s4,s5,s6)

        e0,e1,e2,e3,e4,e5,e6=end_joints
        end= (ctypes.c_double * 7)(e0,e1,e2,e3,e4,e5,e6)

        vel_value=c_double(vel)
        acc_value = c_double(acc)

        self.kine.FX_Robot_PLN_MOVL_KeepJ.argtypes=[c_long,c_double*7,c_double*7,c_double,c_double,c_char_p]
        self.kine.FX_Robot_PLN_MOVL_KeepJ.restype=c_bool
        success1=self.kine.FX_Robot_PLN_MOVL_KeepJ(Serial,start,end,vel_value,acc_value,path_char)
        if success1:
            if os.path.exists(save_path):
                logger.info(f'Plan MOVL KeepJ successful, PATH saved as :{save_path}')
                return True
        else:
            logger.error(f'Plan MOVL KeepJ failed!')
            return False

    def movL_KeepJA(self, start_joints: List[float], end_joints: List[float],
              vel: float, acc: float,
              dimension: int = 7) -> List[List[float]]:
        '''直线规划，执行movL_KeepJA规划并返回点集数据(频率500Hz)

               :param start_joints:起始点各个关节位置（单位：角度）
               :param end_joints:终点各个关节位置（单位：角度）
               :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
               :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
               :return: 规划得到的点集列表
               特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的点集.
                       2 输出点位频率为500Hz
                       3 该接口是不同于MOVLA的规划接口，movL_KeepJA根据起始关节和结束关节规划一条直线路径。
               '''

        Serial = ctypes.c_long(self.robot_tag)
        s0, s1, s2, s3, s4, s5, s6 = start_joints
        start = (ctypes.c_double * 7)(s0, s1, s2, s3, s4, s5, s6)
        e0, e1, e2, e3, e4, e5, e6 = end_joints
        end = (ctypes.c_double * 7)(e0, e1, e2, e3, e4, e5, e6)
        vel_value = c_double(vel)
        acc_value = c_double(acc)
        # 创建CPointSet对象
        pset = self.create_point_set(dimension)
        if not pset:
            raise RuntimeError("Failed to create CPointSet object")

        try:
            # 调用规划函数
            success = self.kine.FX_Robot_PLN_MOVL_KeepJA_C(
                Serial,
                ctypes.cast(start, ctypes.POINTER(ctypes.c_double)),
                ctypes.cast(end, ctypes.POINTER(ctypes.c_double)),
                vel_value,
                acc_value,
                pset
            )

            if success:
                # 获取点集数据
                data = self.get_point_set_data(pset, dimension)
                print(f'Plan MOVL_KeepJA successful, got {len(data)} points')
                return data
            else:
                print('Plan MOVL_KeepJA failed!')
                return []
        finally:
            # 确保清理资源
            self.destroy_point_set(pset)

    def identify_tool_dyn(self, robot_type: int, ipath: str):
        '''工具动力学参数辨识
        :param robot_type: int . 1:CCS机型，2:SRS机型
        :param ipath: sting, 相对路径导入工具辨识轨迹数据。
        :return:
            辨识成功，返回一个长度为10的list:
                        m,mcp*3,i*6
            辨识失败，返回错误类型：
                    ret=1, 计算错误，需重新采集数据计算； 
                    ret=2,打开采集数据文件错误，须检查采样文件； 
                    ret=3,配置文件被修改； 
                    ret=4, 采集时间不够，缺少有效数据

        '''
        if type(robot_type) != int:
            raise ValueError("robot_type must be int type")

        if not os.path.exists(ipath):
            raise ValueError(f"no {ipath}, pls check!")

        if robot_type==1:
            print(f'CCS tool identy')
        elif robot_type==2:
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
            POINTER(c_double*3),
            POINTER(c_double*6)
        ]
        self.kine.FX_Robot_Iden_LoadDyn.restype = c_int

        # 调用函数
        ret_int = self.kine.FX_Robot_Iden_LoadDyn(
            robot_type_,
            path_char,
            mm_ptr,
            mcp_ptr,
            ii_ptr
        )
        if ret_int==0:
            logger.info('Identify tool dynamics successful')

            # 提取结果
            dyn_para=[]
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
            return dyn_para
        else:
            logger.error('Identify tool dynamics failed!')
            logger.error(f'identify_tool_dyn 返回错误码:{ret_int}\n ret=1, 计算错误，需重新采集数据计算\n ret=2,打开采集数据文件错误，须检查采样文件\n ret=3,配置文件被修改\n ret=4, 采集时间不够，缺少有效数据')
            if ret_int==1:
                return 'ret=1, 计算错误，需重新采集数据计算'
            elif ret_int==2:
                return 'ret=2,打开采集数据文件错误，须检查采样文件'
            elif ret_int==3:
                return "ret=3,配置文件被修改"
            elif ret_int==4:
                return 'ret=4, 采集时间不够，缺少有效数据'

def convert_to_8x8_matrix(flat_list):
    if len(flat_list) != 64:
        raise ValueError("列表必须有64个元素")
    matrix_8x8 = []
    for i in range(8):  # 8行
        row_start = i * 8
        row_end = row_start + 8
        matrix_8x8.append(flat_list[row_start:row_end])
    return matrix_8x8

def convert_to_8x8_matrix(flat_list):
    """将一维列表转换为8x8二维列表，并过滤掉全为零的行"""
    if len(flat_list) != 64:
        raise ValueError("列表必须有64个元素")

    matrix_8x8 = []
    for i in range(8):  # 8行
        row_start = i * 8
        row_end = row_start + 8
        row = flat_list[row_start:row_end]
        if not all(abs(val) < 1e-10 for val in row):
            matrix_8x8.append(row)

    return matrix_8x8

FX_INT32L = c_long
FX_DOUBLE = c_double
FX_BOOL = c_bool
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

class Matrix8(Structure):
    _fields_ = [("data", FX_DOUBLE * 64)]

    def __init__(self, values=None):
        super().__init__()
        if values is not None:
            if len(values) != 64:
                raise ValueError("Matrix8 requires exactly 64 values")
            for i, val in enumerate(values):
                self.data[i] = val

    def to_list(self):
        return [self.data[i] for i in range(64)]

    def __str__(self):
        return str(self.to_list())

# 定义主结构体 FX_InvKineSolvePara
class FX_InvKineSolvePara(ctypes.Structure):
    _fields_ = [
        # 输入部分
        ("m_Input_IK_TargetTCP", Matrix4), #末端位置姿态4x4列表，可通过正解接口获取或者指定末端的位置和旋转
        ("m_Input_IK_RefJoint", Vect7), #参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零
        ("m_Input_IK_ZSPType", FX_INT32L), #零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
        ("m_Input_IK_ZSPPara", FX_DOUBLE * 6), #若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
        ("m_Input_ZSP_Angle", FX_DOUBLE), #末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
        ("m_DGR1", FX_DOUBLE), #(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
        ("m_DGR2", FX_DOUBLE), #(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
        ("m_DGR3", FX_DOUBLE), #预留接口
        # 输出部分
        ("m_Output_RetJoint", Vect7), #逆运动学解出的关节角度（单位：度）
        ("m_OutPut_AllJoint", Matrix8), #逆运动学的全部解（每一行代表一组解, 分别存放1 - 7关节的角度值）（单位：度）
        ("m_OutPut_Result_Num", FX_INT32L), #逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
        ("m_Output_IsOutRange", FX_BOOL), #当前位姿是否超出位置可达空间（False：未超出；True：超出）
        ("m_Output_IsDeg", FX_BOOL * 7), #各关节是否发生奇异（False：未奇异；True：奇异）
        ("m_Output_JntExdTags", FX_BOOL * 7), #各关节是否超出位置正负限制（False：未超出；True：超出）
        ("m_Output_JntExdABS", FX_DOUBLE), #所有关节中超出限位的最大角度的绝对值，比如解出一组关节角度，7关节超限，的值为-95，已知软限位为-90度，m_Output_JntExdABS=5.
        ("m_Output_IsJntExd", FX_BOOL), #是否有关节超出位置正负限制（False：未超出；True：超出）
        ("m_Output_RunLmtP", Vect7), #各个关节运行的正限位, 可作为计算六七关节的干涉参考最大限制。
        ("m_Output_RunLmtN", Vect7) #各个关节运行的负限位，可作为计算六七关节的干涉参考最大限制。                                                                                mm
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

        # 初始化其他字段
        self.m_OutPut_Result_Num = 0
        self.m_Output_JntExdABS = 0.0
        self.m_Output_IsJntExd = False
        self.m_Output_IsOutRange = False

    # ==================== 输入部分设置方法 ====================
    def set_input_ik_target_tcp(self, matrix):
        """设置目标TCP位姿矩阵(4x4)"""
        if len(matrix) != 16:
            raise ValueError("m_Input_IK_TargetTCP requires exactly 16 values for 4x4 matrix")
        for i, val in enumerate(matrix):
            self.m_Input_IK_TargetTCP.data[i] = val

    def set_input_ik_ref_joint(self, values):
        """设置参考关节角度(7个值)"""
        if len(values) != 7:
            raise ValueError("m_Input_IK_RefJoint requires exactly 7 values")
        for i, val in enumerate(values):
            self.m_Input_IK_RefJoint.data[i] = val

    def set_input_ik_zsp_type(self, value):
        """设置ZSP类型"""
        self.m_Input_IK_ZSPType = value

    def set_input_ik_zsp_para(self, values):
        """设置ZSP参数(6个值)"""
        if len(values) != 6:
            raise ValueError("m_Input_IK_ZSPPara requires exactly 6 values")
        for i, val in enumerate(values):
            self.m_Input_IK_ZSPPara[i] = val

    def set_input_zsp_angle(self, value):
        """设置ZSP角度"""
        self.m_Input_ZSP_Angle = value

    def set_dgr1(self, value):
        """设置DGR1"""
        self.m_DGR1 = value

    def set_dgr2(self, value):
        """设置DGR2"""
        self.m_DGR2 = value

    def set_dgr3(self, value):
        """设置DGR3"""
        self.m_DGR3 = value

    def set_all_inputs(self, **kwargs):
        """批量设置所有输入参数"""
        setters = {
            'target_tcp': self.set_input_ik_target_tcp,
            'ref_joint': self.set_input_ik_ref_joint,
            'zsp_type': self.set_input_ik_zsp_type,
            'zsp_para': self.set_input_ik_zsp_para,
            'zsp_angle': self.set_input_zsp_angle,
            'dgr1': self.set_dgr1,
            'dgr2': self.set_dgr2,
            'dgr3': self.set_dgr3
        }

        for key, value in kwargs.items():
            if key in setters:
                setters[key](value)
            else:
                raise ValueError(f"Unknown input parameter: {key}")

    # ==================== 输出部分获取方法 ====================

    def get_output_ret_joint(self):
        """获取返回的关节角度(7个值)"""
        return [self.m_Output_RetJoint.data[i] for i in range(7)]

    def get_output_all_joint(self):
        """获取所有关节的矩阵值(8x8=64个值)"""
        return [self.m_OutPut_AllJoint.data[i] for i in range(64)]

    def get_output_result_num(self):
        """获取结果数量"""
        return self.m_OutPut_Result_Num

    def get_output_is_out_range(self):
        """获取是否超出范围"""
        return self.m_Output_IsOutRange

    def get_output_is_deg(self):
        """获取是否为奇异点(7个布尔值)"""
        return [self.m_Output_IsDeg[i] for i in range(7)]

    def get_output_jnt_exd_tags(self):
        """获取关节扩展标签(7个布尔值)"""
        return [self.m_Output_JntExdTags[i] for i in range(7)]

    def get_output_jnt_exd_abs(self):
        """获取关节扩展绝对值"""
        return self.m_Output_JntExdABS

    def get_output_is_jnt_exd(self):
        """获取是否有关节扩展"""
        return self.m_Output_IsJntExd

    def get_output_run_lmt_positive(self):
        """获取正方向运行限制(7个值)"""
        return [self.m_Output_RunLmtP.data[i] for i in range(7)]

    def get_output_run_lmt_negative(self):
        """获取负方向运行限制(7个值)"""
        return [self.m_Output_RunLmtN.data[i] for i in range(7)]

    def get_all_outputs(self):
        """获取所有输出参数"""
        return {
            'ret_joint': self.get_output_ret_joint(),
            'all_joint': self.get_output_all_joint(),
            'result_num': self.get_output_result_num(),
            'is_out_range': self.get_output_is_out_range(),
            'is_deg': self.get_output_is_deg(),
            'jnt_exd_tags': self.get_output_jnt_exd_tags(),
            'jnt_exd_abs': self.get_output_jnt_exd_abs(),
            'is_jnt_exd': self.get_output_is_jnt_exd(),
            'run_lmt_positive': self.get_output_run_lmt_positive(),
            'run_lmt_negative': self.get_output_run_lmt_negative()
        }

    # ==================== 辅助方法 ====================

    def set_output_jnt_exd_tags(self, values):
        """设置关节扩展标签(7个布尔值)"""
        if len(values) != 7:
            raise ValueError("m_Output_JntExdTags requires exactly 7 values")
        for i, val in enumerate(values):
            self.m_Output_JntExdTags[i] = val

    def get_input_ik_zsp_para(self):
        """获取ZSP参数(6个值)"""
        return [self.m_Input_IK_ZSPPara[i] for i in range(6)]

    def __repr__(self):
        """调试用：显示结构体信息"""
        return f"FX_InvKineSolvePara:\n" \
               f"  输入: TCP={self.get_output_ret_joint()}, " \
               f"参考关节={self.get_input_ik_zsp_para()}, " \
               f"ZSP类型={self.m_Input_IK_ZSPType}\n" \
               f"  输出: 结果数={self.m_OutPut_Result_Num}, " \
               f"超限={self.m_Output_IsOutRange}"

class FX_Jacobi(Structure):
    _fields_ = [
        ("m_AxisNum", FX_INT32L),
        ("m_Jcb", (FX_DOUBLE * 7) * 6)  #6x7 二维数组
    ]
    def __init__(self):
        super().__init__()
        self.m_AxisNum = 0
        # 初始化二维数组为0
        for i in range(6):
            # in case the m_Jcb is not contiguous in memory
            row = self.m_Jcb[i]
            for j in range(7):
                row[j] = 0.0

    def set_jcb(self, matrix):
        """
        设置雅可比矩阵的值

        参数:
        matrix: 6x7 二维列表或numpy数组
        """
        if len(matrix) != 6 or any(len(row) != 7 for row in matrix):
            raise ValueError("雅可比矩阵必须是6x7的二维数组")

        for i in range(6):
            row = self.m_Jcb[i]
            for j in range(7):
                row[j] = matrix[i][j]

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

def inv_main():
    # 创建结构体实例
    ik_params = FX_InvKineSolvePara()

    # 设置输入参数
    ik_params.m_Input_IK_ZSPType = 1
    ik_params.m_Input_ZSP_Angle = 45.0

    # 设置TCP矩阵（示例值）
    tcp_values = [1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1]
    ik_params.m_Input_IK_TargetTCP = Matrix4(tcp_values)

    # 设置关节参考位置（示例值）
    ref_joint = [0, 0.4, 0, 0, 0, 0, 0]
    ik_params.m_Input_IK_RefJoint = Vect7(ref_joint)

    # 设置ZSP参数
    ik_params.set_input_ik_zsp_para([1.0, 0.,0.,0.,0.,0.])

    # 输出结构体大小
    print(f"FX_InvKineSolvePara size: {sizeof(ik_params)} bytes")
    print(f"Matrix8 size: {sizeof(Matrix8)} bytes")

if __name__ == "__main__":
    kk = Marvin_Kine()  # 实例化
    kk.help()  # 查看方法
    kk.help('load_config')

    #逆解结构体
    inv_main()
    exit()
