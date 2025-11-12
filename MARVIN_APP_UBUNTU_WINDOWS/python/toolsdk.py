import ctypes
import os
import logging
import sys
# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

current_path = os.getcwd()

# 定义 FX_DOUBLE 类型（假设为 double）
FX_DOUBLE = ctypes.c_double


# 定义 LoadDynamicPara 结构体
class LoadDynamicPara(ctypes.Structure):
    _fields_ = [
        ("m", FX_DOUBLE),
        ("r", FX_DOUBLE * 3),
        ("I", FX_DOUBLE * 6)
    ]

    def __str__(self):
        """自定义结构体的字符串表示，便于打印"""
        r_values = [self.r[i] for i in range(3)]
        i_values = [self.I[i] for i in range(6)]
        return f"m: {self.m}\nr: {r_values}\nI: {i_values}"

if sys.platform == 'win32':
    tool = ctypes.WinDLL(os.path.join(current_path, 'python/libToolSDK.dll'))
else:
    tool = ctypes.CDLL(os.path.join(current_path, 'python/libToolSDK.so'))


def format_vector(vector):
    return ", ".join([f"{v:.6f}" for v in vector])

def tool_dyn_identy(ccs:bool,data_dr:str):

    # 设置 OnCalLoadDyn 函数的原型
    tool.OnCalLoadDyn.argtypes = [
        ctypes.POINTER(LoadDynamicPara),  # LoadDynamicPara*
        ctypes.c_bool,  # bool
        ctypes.c_char_p  # const char*
    ]
    tool.OnCalLoadDyn.restype = ctypes.c_int  # 返回 int


    print("[FxRobot - FX_Robot_Iden_LoadDyn]")

    # 创建 LoadDynamicPara 结构体实例
    dyn_para = LoadDynamicPara()

    # 调用 C 函数
    result = tool.OnCalLoadDyn(
        ctypes.byref(dyn_para),  # 传递结构体指针
        ctypes.c_bool(ccs),  # 转换为 C 的 bool 类型
        data_dr.encode('utf-8')  # 转换为字节字符串
    )

    dyn_tool=[]
    logger.info(f'result:{result}')
    dyn_tool.append(dyn_para.m)
    for i in dyn_para.r:
        dyn_tool.append(i)
    for j in dyn_para.I:
        dyn_tool.append(j)

    logger.info(f"tool dynamic parameters={dyn_tool}")

    return result, format_vector(dyn_tool)

if __name__=="__main__":
    r,dyn=tool_dyn_identy(ccs=False, data_dr='./LoadData')



