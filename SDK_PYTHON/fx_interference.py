from ctypes import *
import ctypes
import os
import sys

current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)


# ===== 干涉检测返回码 (对应 Interference.h 中 FX_InterfErrCode) =====
FUNC_RET_SUCCESS = 0           # 操作成功
FUNC_RET_INVALID_INPUT_ARG = -1  # 输入参数非法
FUNC_RET_OPERATION_FAILED = -2   # 操作失败
FUNC_RET_ISInterference = -3     # 存在干涉(超过阈值)
FUNC_RET_NOInterference = -4     # 无干涉


class Marvin_Interference:
    """干涉检测(碰撞检测)类

    封装 interferenceCheck/Interf/Interference.h 中的 C 接口,
    提供坐标系刷新、凸包重建、碰撞距离计算及干涉判定等功能。

    典型调用流程:
        interf = Marvin_Interference()
        interf.create()
        interf.init(CordDef, CalLinkDef, InputDef, ConvexDef, ICDef)
        interf.update_cord(cord_num, joints)
        interf.update_convex()
        interf.calc_interf_distance()
        state = interf.on_is_interf(thresholds)
        ...
        interf.destroy()
    """

    def __init__(self):
        """初始化干涉检测类, 加载动态库并设置函数原型"""
        logger = sys.stdout  # 保持轻量, 不引入 logging 配置
        if sys.platform == 'win32':
            self.interf = ctypes.WinDLL(os.path.join(current_path, 'libInterfCheck.dll'))
        else:
            self.interf = ctypes.CDLL(os.path.join(current_path, 'libInterfCheck.so'))

        self._handle = None
        self._setup_function_prototypes()

    def _setup_function_prototypes(self):
        """设置所有 C 函数的参数类型和返回类型"""

        # FX_Interf_Create
        self.interf.FX_Interf_Create.argtypes = []
        self.interf.FX_Interf_Create.restype = ctypes.c_void_p

        # FX_Interf_Destroy
        self.interf.FX_Interf_Destroy.argtypes = [ctypes.c_void_p]
        self.interf.FX_Interf_Destroy.restype = None

        # FX_Interf_Init
        self.interf.FX_Interf_Init.argtypes = [
            ctypes.c_void_p,  # handle
            ctypes.c_char_p,  # CordDef
            ctypes.c_char_p,  # CalLinkDef
            ctypes.c_char_p,  # InputDef
            ctypes.c_char_p,  # ConvexDef
            ctypes.c_char_p,  # ICDef
        ]
        self.interf.FX_Interf_Init.restype = ctypes.c_int

        # FX_Interf_UpdateCord
        self.interf.FX_Interf_UpdateCord.argtypes = [
            ctypes.c_void_p,                 # handle
            ctypes.c_long,                   # cord_num
            ctypes.POINTER(ctypes.c_double), # input
        ]
        self.interf.FX_Interf_UpdateCord.restype = ctypes.c_int

        # FX_Interf_UpdateConvex
        self.interf.FX_Interf_UpdateConvex.argtypes = [ctypes.c_void_p]
        self.interf.FX_Interf_UpdateConvex.restype = ctypes.c_int

        # FX_Interf_CalcInterfDistance
        self.interf.FX_Interf_CalcInterfDistance.argtypes = [ctypes.c_void_p]
        self.interf.FX_Interf_CalcInterfDistance.restype = ctypes.c_int

        # FX_Interf_GetCoM
        self.interf.FX_Interf_GetCoM.argtypes = [
            ctypes.c_void_p,                          # handle
            ctypes.POINTER(ctypes.c_double),          # mcp_average[3]
        ]
        self.interf.FX_Interf_GetCoM.restype = ctypes.c_int

        # FX_Interf_GetCordsNum
        self.interf.FX_Interf_GetCordsNum.argtypes = [
            ctypes.c_void_p,                 # handle
            ctypes.POINTER(ctypes.c_long),   # cords_num
        ]
        self.interf.FX_Interf_GetCordsNum.restype = ctypes.c_int

        # FX_Interf_GetMass
        self.interf.FX_Interf_GetMass.argtypes = [
            ctypes.c_void_p,                 # handle
            ctypes.POINTER(ctypes.c_double), # quality
        ]
        self.interf.FX_Interf_GetMass.restype = ctypes.c_int

        # FX_Interf_GetMinSpan
        self.interf.FX_Interf_GetMinSpan.argtypes = [
            ctypes.c_void_p,                 # handle
            ctypes.POINTER(ctypes.c_double), # MinSpan
        ]
        self.interf.FX_Interf_GetMinSpan.restype = ctypes.c_int

        # FX_Interf_GetInterfSpan
        self.interf.FX_Interf_GetInterfSpan.argtypes = [
            ctypes.c_void_p,                 # handle
            ctypes.c_long,                   # serial
            ctypes.POINTER(ctypes.c_double), # MinSpan
        ]
        self.interf.FX_Interf_GetInterfSpan.restype = ctypes.c_int

        # FX_Interf_OnIsInterf
        self.interf.FX_Interf_OnIsInterf.argtypes = [
            ctypes.c_void_p,                 # handle
            ctypes.POINTER(ctypes.c_double), # inteThsh
        ]
        self.interf.FX_Interf_OnIsInterf.restype = ctypes.c_int

    # ===== 对外接口 =====

    def create(self):
        """创建干涉检测单例句柄

        Returns:
            int 句柄地址(整数), 失败返回 None
        """
        self._handle = self.interf.FX_Interf_Create()
        return self._handle

    def destroy(self):
        """销毁干涉检测并释放资源, 调用后句柄失效"""
        if self._handle is not None:
            self.interf.FX_Interf_Destroy(self._handle)
            self._handle = None

    def init(self, cord_def, cal_link_def, input_def, convex_def, ic_def):
        """使用多组配置定义初始化干涉模块

        参数既可传入文件路径字符串, 也可传入定义内容字符串。
        在执行任何坐标更新或距离计算之前必须先调用此 API。

        Args:
            cord_def:     坐标系定义 (文件路径或定义字符串)
            cal_link_def: 连杆标定参数定义 (文件路径或定义字符串)
            input_def:    外部输入变量映射定义 (文件路径或定义字符串)
            convex_def:   凸碰撞几何体定义 (文件路径或定义字符串)
            ic_def:       干涉检查约束规则定义 (文件路径或定义字符串)

        Returns:
            int 返回码, 见 FX_InterfErrCode (0 表示成功)
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED
        ret = self.interf.FX_Interf_Init(
            self._handle,
            _to_char_p(cord_def),
            _to_char_p(cal_link_def),
            _to_char_p(input_def),
            _to_char_p(convex_def),
            _to_char_p(ic_def),
        )
        return ret

    def update_cord(self, cord_num, input_data):
        """刷新指定坐标组的实时坐标数据

        Args:
            cord_num:   目标坐标组索引标识数量
            input_data: 原始坐标 double 列表, 长度遵循已加载的配置定义

        Returns:
            int 返回码, 0 表示成功
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED
        arr = (ctypes.c_double * len(input_data))(*input_data)
        ret = self.interf.FX_Interf_UpdateCord(
            self._handle, ctypes.c_long(cord_num), arr)
        return ret

    def update_convex(self):
        """重建所有凸包碰撞几何模型 (需在 update_cord 之后调用)

        Returns:
            int 返回码, 0 表示成功
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED
        return self.interf.FX_Interf_UpdateConvex(self._handle)

    def calc_interf_distance(self):
        """计算所有刚体碰撞对的最小分离距离

        Returns:
            int 返回码, 0 表示成功
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED
        return self.interf.FX_Interf_CalcInterfDistance(self._handle)

    def get_com(self):
        """获取所有刚性连杆的加权平均质心 [X, Y, Z]

        Returns:
            tuple (ret, [X, Y, Z]); ret 为返回码, 失败时列表为 [0,0,0]
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED, [0.0, 0.0, 0.0]
        mcp = (ctypes.c_double * 3)()
        ret = self.interf.FX_Interf_GetCoM(self._handle, mcp)
        return ret, [mcp[0], mcp[1], mcp[2]]

    def get_cords_num(self):
        """获取已加载坐标组的总数

        Returns:
            tuple (ret, cords_num)
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED, 0
        num = ctypes.c_long(0)
        ret = self.interf.FX_Interf_GetCordsNum(self._handle, ctypes.byref(num))
        return ret, num.value

    def get_mass(self, count):
        """获取所有已定义刚体的质量标量数组

        Args:
            count: 质量数组长度, 通常等于坐标组总数 (可用 get_cords_num 获取)

        Returns:
            tuple (ret, [质量...])
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED, []
        quality = (ctypes.c_double * count)()
        ret = self.interf.FX_Interf_GetMass(self._handle, quality)
        return ret, [quality[i] for i in range(count)]

    def get_min_span(self):
        """获取所有碰撞对的全局最小几何分离距离

        Returns:
            tuple (ret, min_span)
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED, 0.0
        span = ctypes.c_double(0.0)
        ret = self.interf.FX_Interf_GetMinSpan(self._handle, ctypes.byref(span))
        return ret, span.value

    def get_interf_span(self, serial):
        """按碰撞对序号索引查询干涉距离值

        Args:
            serial: 目标碰撞对的唯一序号索引

        Returns:
            tuple (ret, min_span)
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED, 0.0
        span = ctypes.c_double(0.0)
        ret = self.interf.FX_Interf_GetInterfSpan(
            self._handle, ctypes.c_long(serial), ctypes.byref(span))
        return ret, span.value

    def on_is_interf(self, thresholds):
        """查询碰撞对是否存在超过预定义干涉阈值的情况

        Args:
            thresholds: 各碰撞对干涉阈值 double 列表

        Returns:
            int 返回码: FUNC_RET_ISInterference(-3) 表示发生干涉,
                       FUNC_RET_NOInterference(-4) 表示无干涉
        """
        if self._handle is None:
            return FUNC_RET_OPERATION_FAILED
        arr = (ctypes.c_double * len(thresholds))(*thresholds)
        return self.interf.FX_Interf_OnIsInterf(self._handle, arr)


def _to_char_p(value):
    """将 str/bytes 转为 c_char_p (UTF-8 编码)"""
    if value is None:
        return None
    if isinstance(value, bytes):
        return ctypes.c_char_p(value)
    return ctypes.c_char_p(value.encode('utf-8'))
