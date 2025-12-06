from ctypes import *


# 定义StateCtr结构体
class StateCtr(Structure):
    _fields_ = [
        ("m_CurState", c_int),  # * 当前状态 */ ArmState
        ("m_CmdState", c_int),  # * 指令状态 */ DCSSCmdType 0
        ("m_ERRCode", c_int)  # * 错误码   */
    ]


# 定义RT_IN结构体
class RT_IN(Structure):
    _fields_ = [
        ("m_RtInSwitch", c_int),  # * 实时输入开关 用户实时数据 进行开关设置 0 -  close rt_in ;1- open rt_in*
        ("m_ImpType", c_int),  #阻抗类型
        ("m_InFrameSerial", c_int),  # short 输入帧序号   0 -  1000000 取模
        ("m_FrameMissCnt", c_short),  # short 丢帧计数
        ("m_MaxFrameMissCnt", c_short),  # short 开 启 后 最 大 丢 帧 计 数

        ("m_SysCyc", c_int),  # 0 -  1000000
        ("m_SysCycMissCnt", c_short),  # short 实 时 性  Miss 计 数
        ("m_MaxSysCycMissCnt", c_short),  # short开 启 后 最 大 实 时 性Miss 计 数

        ("m_ToolKine", c_float * 6),  # 工 具 运 动 学 参 数 1
        ("m_ToolDyn", c_float * 10),  # 工 具 动 力 学 参 数 1

        ("m_Joint_CMD_Pos", c_float * 7),  # 关 节 位 置 指 令
        ("m_Joint_Vel_Ratio", c_short),  # short 关 节 速 度 限 制 百分比 2
        ("m_Joint_Acc_Ratio", c_short),  # short 关 节 加 速 度 限 制  百分比 2

        ("m_Joint_K", c_float * 7),  # 关节阻抗刚度K指令 3
        ("m_Joint_D", c_float * 7),  # 关节阻抗刚度D指令 4

        ("m_DragSpType", c_int),  # 零空间类型 5
        ("m_DragSpPara", c_float * 6),  # 零空间参数类型 5

        ("m_Cart_KD_Type", c_int),  # 坐标阻抗类型
        ("m_Cart_K", c_float*6),  # 坐标阻抗刚度K指令 4
        ("m_Cart_D", c_float*6),  # 坐标阻抗阻尼D指令 4
        ("m_Cart_KN", c_float),  # 4
        ("m_Cart_DN", c_float),  # 4

        ("m_Force_FB_Type", c_int),  # 力控反馈源类型
        ("m_Force_Type", c_int),  # 力控类型 6
        ("m_Force_Dir", c_float * 6),  # 力控方向6维空间方向 6
        ("m_Force_PIDUL", c_float * 7),  # 力控pid 6
        ("m_Force_AdjLmt", c_float),  # 允许调节最大范围 6

        ("m_Force_Cmd", c_float),  # 力控指令 8

        ("m_SET_Tags", c_ubyte * 16),  # 零空间类型 5
        ("m_Update_Tags", c_ubyte * 16),  # 零空间类型 5

        ("m_PvtID", c_ubyte),  #设置的PVT号
        ("m_PvtID_Update", c_ubyte),  #PVT号更新情况
        ("m_Pvt_RunID", c_ubyte), #0: no pvt file; 1~99: 用户上传的PVT
        ("m_Pvt_RunState", c_ubyte),  #0: idle空闲; 1: loading正在加载 ; 2: running正在运行; 3: error出错啦

    ]


# 定义RT_OUT结构体
class RT_OUT(Structure):
    _fields_ = [
        ("m_OutFrameSerial", c_int),  # 输出帧序号   0 -  1000000 取模
        ("m_FB_Joint_Pos", c_float * 7),  # 关节位置反馈
        ("m_FB_Joint_Vel", c_float * 7),  # 关节速度反馈
        ("m_FB_Joint_PosE", c_float * 7),  # 关节位置(外编)
        ("m_FB_Joint_Cmd", c_float * 7),  # 位置关节指令
        ("m_FB_Joint_CToq", c_float * 7),  # 关节指令扭矩
        ("m_FB_Joint_SToq", c_float * 7),  # 关节实际扭矩
        ("m_FB_Joint_Them", c_float * 7),  # 关节温度
        ("m_EST_Joint_Firc", c_float * 7),  # 关节摩擦估计
        ("m_EST_Joint_Firc_Dot", c_float * 7),  # 关节力扰动估计值微分
        ("m_EST_Joint_Force", c_float * 7),  # 关节力扰动估计值
        ("m_EST_Cart_FN", c_float * 6),  # 末端笛卡尔空间力扰动估计值
        ("m_TipDI", c_char),  # 末端数字输入
        ("m_LowSpdFlag", c_char),  # 低速标志
        # ("m_pad", c_char * 2)  # 填充字节
    ]


# 定义DCSS结构体
class DCSS(Structure):
    _fields_ = [
        ("m_State", StateCtr * 2),  # 状态控制器数组
        ("m_In", RT_IN * 2),  # 输出数据数组
        ("m_Out", RT_OUT * 2),  # 输出数据数组

        ("m_ParaName", c_char * 30),  # 参数名称，结合配置机器人参数相关
        ("m_ParaType", c_ubyte),  # 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
        ("m_ParaIns", c_ubyte),  # DCSSCfgOperationType
        ("m_ParaValueI", c_int),  # FX_INT32 value
        ("m_ParaValueF", c_float),  # FX_FLOAT value
        ("m_ParaCmdSerial", c_short),  # short from PC
        ("m_ParaRetSerial", c_short),  # short working: 0; finish: cmd serial; error cmd_serial + 100
    ]





def call_on_get_buf():
    # 创建DCSS结构体实例
    dcss = DCSS()

    # 初始化结构体数据（可选）
    dcss.m_ParaName = b"DefaultParameters"  # 字节字符串

    # 调用C函数
    result = lib.OnGetBuf(byref(dcss))

    if not result:
        print("函数调用失败")
        return None

    # 处理返回的数据
    return process_dcss_data(dcss)


def process_dcss_data(dcss):
    """处理并提取DCSS结构体中的数据"""
    data = {}

    # 提取参数名
    data['para_name'] = dcss.m_ParaName.decode().strip('\x00')

    # 处理两个StateCtr实例
    states = []
    for i in range(2):
        state = {
            'cur_state': dcss.m_State[i].m_CurState,
            'cmd_state': dcss.m_State[i].m_CmdState,
            'err_code': dcss.m_State[i].m_ERRCode
        }
        states.append(state)
    data['states'] = states

    # 处理两个RT_OUT实例
    outputs = []
    for i in range(2):
        output = {
            'frame_serial': dcss.m_Out[i].m_OutFrameSerial,
            'joint_pos': list(dcss.m_Out[i].m_FB_Joint_Pos),
            'joint_vel': list(dcss.m_Out[i].m_FB_Joint_Vel),
            # 其他字段可根据需要添加...
            'tip_di': dcss.m_Out[i].m_TipDI,
            'low_speed_flag': dcss.m_Out[i].m_LowSpdFlag
        }
        outputs.append(output)
    data['outputs'] = outputs

    return data


# 示例调用
if __name__ == "__main__":
    '''
    ctypes      类型	C           等价类型	大小 (字节)	        取值范围
    c_int16	    int16_t     	    2	                    -32,768 到 32,767
    c_ubyte	    unsigned char	    1	                    0 到 255
    c_int8	    int8_t	            1	                    -128 到 127
    c_uint8	    uint8_t	            1	                    0 到 255
    c_int32	    int32_t	            4	                    -2,147,483,648 到 2,147,483,647
    c_uint32	uint32_t	        4	                    0 到 4,294,967,295
    c_float	    float	            4	                    约 ±3.4e38 (7位精度)
    c_double	double	            8	                    约 ±1.8e308 (15位精度)
    c_char	    char	            1	                    -128 到 127 或 0 到 255
    c_bool	    bool (C99+)	        1	                    0 或 1
    c_size_t	size_t	            4或8	                 平台相关
    '''
    lib = CDLL("MarvinLib/libMarvinSDK.so")  # 替换为实际库路径
    lib.OnGetBuf.argtypes = [POINTER(DCSS)]
    lib.OnGetBuf.restype = c_bool

    # 检查结构体大小是否匹配C端
    print(f"StateCtr size: {sizeof(StateCtr)} ")
    print(f"RT_IN size: {sizeof(RT_IN)} ")
    print(f"RT_OUT size: {sizeof(RT_OUT)} ")
    print(f"DCSS size: {sizeof(DCSS)} ")
    # result_data = call_on_get_buf()
    # if result_data:
    #     import pprint
    #
    #     pprint.pprint(result_data)
    # dcss = DCSS()
    # print(dcss.m_State)
