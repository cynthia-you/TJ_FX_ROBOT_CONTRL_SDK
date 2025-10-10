import ctypes
import inspect
from textwrap import dedent
import os
import re
from typing import Union

current_path=os.getcwd()


def decimal_to_hex(number, prefix=False, upper=True, float_precision=8):
    """
    将十进制数转换为十六进制表示

    参数:
        number: 要转换的十进制数，可以是整数或浮点数
        prefix: 是否添加"0x"前缀，默认为False
        upper: 是否使用大写字母，默认为True
        float_precision: 浮点数转换时的精度（小数位数），默认为8

    返回:
        str: 十六进制表示的字符串

    异常:
        TypeError: 当输入不是数字时抛出
    """
    # 检查输入是否为数字
    if not isinstance(number, (int, float)):
        raise TypeError("输入必须是整数或浮点数")

    # 处理整数
    if isinstance(number, int):
        hex_str = hex(number)
    # 处理浮点数
    else:
        # 使用float.hex()方法获取浮点数的十六进制表示
        hex_str = float(number).hex()

        # 如果需要，可以限制小数部分的精度
        if float_precision is not None:
            parts = hex_str.split('.')
            if len(parts) > 1:
                exponent_part = parts[1].split('p')
                if len(exponent_part) > 1:
                    hex_str = f"{parts[0]}.{exponent_part[0][:float_precision]}p{exponent_part[1]}"

    # 移除或保留前缀
    if not prefix and hex_str.startswith('0x'):
        hex_str = hex_str[2:]
    elif prefix and not hex_str.startswith('0x'):
        hex_str = '0x' + hex_str

    # 处理大小写
    if upper:
        hex_str = hex_str.upper()
    else:
        hex_str = hex_str.lower()

    return hex_str

def identify_and_calculate_length(input_data: Union[str, bytes]) -> dict:
    result = {
        "input": input_data,
        "type": None,
        "length_bytes": None,
        "bytes_representation": None
    }

    # 处理字节串输入
    if isinstance(input_data, bytes):
        result["type"] = "bytes"
        result["length_bytes"] = len(input_data)
        result["bytes_representation"] = input_data
        return result

    # 处理字符串输入
    if isinstance(input_data, str):
        # 检查是否是十六进制字符串（可能包含空格和0x前缀）
        # 移除所有空格和0x前缀
        clean_input = re.sub(r'\s+', '', input_data.lower())

        if clean_input.startswith('0x'):
            clean_input = clean_input[2:]

        # 检查是否为有效的十六进制字符串
        hex_pattern = re.compile(r'^[0-9a-f]+$')
        if hex_pattern.match(clean_input):
            # 确保长度为偶数
            if len(clean_input) % 2 != 0:
                clean_input = '0' + clean_input

            try:
                bytes_rep = bytes.fromhex(clean_input)
                result["type"] = "hex string"
                result["length_bytes"] = len(bytes_rep)
                result["bytes_representation"] = bytes_rep
                return result
            except ValueError:
                pass  # 如果不是有效的十六进制，继续尝试其他解释

        # 检查是否已经是字节串表示形式（如b"\x06\x01\xe3\x08"）
        if input_data.startswith('b"') and input_data.endswith('"'):
            try:
                # 使用eval安全地转换（注意：在实际应用中可能需要更安全的方法）
                bytes_rep = eval(input_data)
                if isinstance(bytes_rep, bytes):
                    result["type"] = "bytes representation string"
                    result["length_bytes"] = len(bytes_rep)
                    result["bytes_representation"] = bytes_rep
                    return result
            except:
                pass

        # 如果不是上述任何类型，将其视为普通字符串
        try:
            bytes_rep = input_data.encode('utf-8')
            result["type"] = "regular string"
            result["length_bytes"] = len(bytes_rep)
            result["bytes_representation"] = bytes_rep
            return result
        except UnicodeEncodeError:
            raise ValueError("输入不是有效的十六进制字符串，也无法编码为UTF-8字节串")

    # 如果既不是字符串也不是字节串，抛出异常
    raise TypeError("输入必须是字符串或字节串")

def structure2dict(dcss):
    result = {
        "para_name": ['Marvin_sub_data'],
        "states": [
            {
                "cur_state": dcss.m_State[0].m_CurState,
                "cmd_state": dcss.m_State[0].m_CmdState,
                "err_code": dcss.m_State[0].m_ERRCode
            },
            {
                "cur_state": dcss.m_State[1].m_CurState,
                "cmd_state": dcss.m_State[1].m_CmdState,
                "err_code": dcss.m_State[1].m_ERRCode
            }
        ]
    }
    # 3. 处理实时输出数组
    result["outputs"] = [
        {
            "frame_serial": rt_out.m_OutFrameSerial,
            "tip_di": rt_out.m_TipDI,
            "low_speed_flag": rt_out.m_LowSpdFlag,
            "fb_joint_pos": [round(rt_out.m_FB_Joint_Pos[j], 4) for j in range(7)],
            "fb_joint_vel": [round(rt_out.m_FB_Joint_Vel[j], 4) for j in range(7)],
            "fb_joint_posE": [round(rt_out.m_FB_Joint_PosE[j], 4) for j in range(7)],
            "fb_joint_cmd": [round(rt_out.m_FB_Joint_Cmd[j], 4) for j in range(7)],
            "fb_joint_cToq": [round(rt_out.m_FB_Joint_CToq[j], 4) for j in range(7)],
            "fb_joint_sToq": [round(rt_out.m_FB_Joint_SToq[j], 4) for j in range(7)],
            "fb_joint_them": [round(rt_out.m_FB_Joint_Them[j], 4) for j in range(7)],
            "est_joint_firc": [round(rt_out.m_EST_Joint_Firc[j], 4) for j in range(7)],
            "est_joint_firc_dot": [round(rt_out.m_EST_Joint_Firc_Dot[j], 4) for j in range(7)],
            "est_joint_force": [round(rt_out.m_EST_Joint_Force[j], 4) for j in range(7)],
            "est_cart_fn": [round(rt_out.m_EST_Cart_FN[j], 4) for j in range(6)]
        } for rt_out in dcss.m_Out
    ]

    # 4. 处理实时输入数组 (RT_IN)
    result["inputs"] = [
        {
            "rt_in_switch": rt_in.m_RtInSwitch,
            "imp_type": rt_in.m_ImpType,
            "in_frame_serial": rt_in.m_InFrameSerial,
            "frame_miss_cnt": rt_in.m_FrameMissCnt,
            "max_frame_miss_cnt": rt_in.m_MaxFrameMissCnt,
            "sys_cyc": rt_in.m_SysCyc,
            "sys_cyc_miss_cnt": rt_in.m_SysCycMissCnt,
            "max_sys_cyc_miss_cnt": rt_in.m_MaxSysCycMissCnt,
            "tool_kine": [round(rt_in.m_ToolKine[j], 4) for j in range(6)],
            "tool_dyn": [round(rt_in.m_ToolDyn[j], 4) for j in range(10)],
            "joint_cmd_pos": [round(rt_in.m_Joint_CMD_Pos[j], 4) for j in range(7)],
            "joint_vel_ratio": rt_in.m_Joint_Vel_Ratio,
            "joint_acc_ratio": rt_in.m_Joint_Acc_Ratio,
            "joint_k": [round(rt_in.m_Joint_K[j], 4) for j in range(7)],
            "joint_d": [round(rt_in.m_Joint_D[j], 4) for j in range(7)],
            "drag_sp_type": rt_in.m_DragSpType,
            "drag_sp_para": [round(rt_in.m_DragSpPara[j], 4) for j in range(6)],
            "cart_kd_type": rt_in.m_Cart_KD_Type,
            "cart_k": [round(rt_in.m_Cart_K[j], 4) for j in range(6)],
            "cart_d": [round(rt_in.m_Cart_D[j], 4) for j in range(6)],
            "cart_kn": round(rt_in.m_Cart_KN, 4),
            "cart_dn": round(rt_in.m_Cart_DN, 4),
            "force_fb_type": rt_in.m_Force_FB_Type,
            "force_type": rt_in.m_Force_Type,
            "force_dir": [round(rt_in.m_Force_Dir[j], 4) for j in range(6)],
            "force_pidul": [round(rt_in.m_Force_PIDUL[j], 4) for j in range(7)],
            "force_adj_lmt": round(rt_in.m_Force_AdjLmt, 4),
            "force_cmd": round(rt_in.m_Force_Cmd, 4),
            "set_tags": list(rt_in.m_SET_Tags),
            "update_tags": list(rt_in.m_Update_Tags),
            "pvt_id": rt_in.m_PvtID,
            "pvt_id_update": rt_in.m_PvtID_Update,
            "pvt_run_id": rt_in.m_Pvt_RunID,
            "pvt_run_state": rt_in.m_Pvt_RunState
        } for rt_in in dcss.m_In
    ]

    result["ParaName"]=[list(dcss.m_ParaName)]
    result["ParaType"]=[dcss.m_ParaType]
    result["ParaIns"]=[dcss.m_ParaIns]
    result["ParaValueI"]=[dcss.m_ParaValueI]
    result["ParaValueF"]=[dcss.m_ParaValueF]
    result["ParaCmdSerial"]=[dcss.m_ParaCmdSerial]
    result["ParaRetSerial"]=[dcss.m_ParaRetSerial]

    return result

class Marvin_Robot:
    def __init__(self):
        """初始化机器人控制类"""
        import sys
        print(f'user platform: {sys.platform}')
        if sys.platform=='win32':
            self.robot = ctypes.WinDLL(os.path.join(current_path,'libMarvinSDK.dll'))
        else:
            self.robot = ctypes.CDLL(os.path.join(current_path,'libMarvinSDK.so'))
        self.ErrorCode = None
        self.a_pvt_path=None
        self.b_pvt_path = None
        self.local_file_path=None
        self.remote_file_path=None
        self.save_csv_path=None
        self.save_data_path=None

    def _convert_ip(self, ip_str):
        """将IP字符串转换为ctypes数组"""
        ip1, ip2, ip3, ip4 = ip_str.split('.')
        ip_uchar = ctypes.c_ubyte
        return ip_uchar(int(ip1)), ip_uchar(int(ip2)), ip_uchar(int(ip3)), ip_uchar(int(ip4))

    def connect(self, robot_ip: str):
        '''连接机器人
        :param robot_ip: 器人IP地址,确保网线连接可以ping通。
        :return:
            int: 连接状态码 1: True; 0: Flase

        eg:
            connect(robot_ip='192.168.1.190')
        '''
        ip1, ip2, ip3, ip4 = self._convert_ip(robot_ip)
        return self.robot.OnLinkTo(ip1, ip2, ip3, ip4)


    def subscribe(self,dcss):
        '''订阅机器人状态数据
        :param dcss:  结构体，见structure_data.py
        :return:
            嵌套字典
        '''
        self.robot.OnGetBuf(ctypes.byref(dcss))
        result=structure2dict(dcss)
        return result

    def release_robot(self):
        ''' 断开机器人连接
        :return:
            int: 断开状态码 1: True; 0: Flase
        '''
        return self.robot.OnRelease()

    def SDK_version(self):
        '''查看SDK版本
        :return:
            long: SDK version
        '''
        return self.robot.OnGetSDKVersion()

    def update_SDK(self, sdk_path: str):
        '''更新系统SDK版本
        :param sdk_path: 本机存放SDK的绝对路径的SDK文件更新到控制柜上
        :return:
        '''
        sdk_char = ctypes.c_char_p(sdk_path.encode('utf-8'))
        self.robot.OnUpdateSystem(sdk_char)

    def download_sdk_log(self, log_path:str):
        '''下载SDK日志到本机
        :param log_path: 日志下载到本机的绝对路
        :return:
        '''
        log_char = ctypes.c_char_p(log_path.encode('utf-8'))
        return self.robot.OnDownloadLog(log_char)


    def get_param(self,type:str,paraName:str):
        '''获取参数信息
        :param type: float or int .参数类型
        :param paraName:  参数名见robot.ini
        :return:参数值
        eg:
         robot,ini:
            [R.A0.BASIC]
            BDRange=1.5
            BDToqR=1
            Dof=7
            GravityX=0
            GravityY=9.81
            GravityZ=0
            LoadOffsetSwitch=0
            TerminalPolar=1
            TerminalType=1
            Type=1007
            [R.A0.CTRL]
            CartJNTDampJ1=0.6
            ....
            #浮点类型参数获取：
            我想获取[R.A0.CTRL]这个参数组里CartJNTDampJ1的值:
            para=get_param('float','R.A0.CTRL.CartJNTDampJ1')

            #整数类型参数获取：
            我想获取[R.A0.BASIC]这个参数组里Type的值
            para=get_param('int','R.A0.BASIC.Type')
        '''
        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # 显式添加终止符

            if type=='float':
                result = ctypes.c_double(0)
                self.robot.OnGetFloatPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, float parameters={result.value}")
                return result.value
            elif type=='int':
                result = ctypes.c_int(0)
                self.robot.OnGetIntPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, int parameters={result.value}")
                return result.value
        except Exception as e:
            print("ERROR:",e)

    def set_param(self,type:str,paraName:str,value:float):
        '''设置参数信息
        :param type: float or int .参数类型
        :param paraName:  参数名见robot.ini
        :param value:
        :return:
        eg:
         robot,ini:
            [R.A0.BASIC]
            BDRange=1.5
            BDToqR=1
            Dof=7
            GravityX=0
            GravityY=9.81
            GravityZ=0
            LoadOffsetSwitch=0
            TerminalPolar=1
            TerminalType=1
            Type=1007
            [R.A0.CTRL]
            CartJNTDampJ1=0.6
            ....
            #设置浮点类型参数获取：
            我想设置[R.A0.CTRL]这个参数组里CartJNTDampJ1的值为0.0
            set_params('float','R.A0.CTRL.CartJNTDampJ1,0.0)

            #设置整数类型参数获取：
            我想设置[R.A0.BASIC]这个参数组里Type的值为0
            set_params('int','R.A0.BASIC.Type',0)
        '''

        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # 显式添加终止符

            if type=='float':
                result = ctypes.c_double(value)
                self.robot.OnSetFloatPara(param_buf, result)
                return True
            elif type=='int':
                result = ctypes.c_int(0)
                self.robot.OnSetIntPara(param_buf, result)
                return True
        except Exception as e:
            print("ERROR:",e)

    def clear_set(self):
        '''指令发送前清除
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnClearSet()

    def send_cmd(self):
        '''发送指令
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnSetSend()

    def collect_data(self,targetNum:int,targetID:list[int],recordNum:int):
        '''采集数据
        :param targetNum:targetNum采集列数 值最大35， 因为一次最多采集35个特征。
        :param targetID: list(35,1) 对应采集数据ID序号(见下)
        :param recordNum: 采集行数，小于1000会采集1000行，设置大于一百万行会采集一百万行。
        :return:
                    采集数据ID序号
                    左臂
                        0-6  	左臂关节位置
                        10-16 	左臂关节速度
                        20-26   左臂外编位置
                        30-36   左臂关节指令位置
                        40-46	左臂关节电流（千分比）
                        50-56   左臂关节传感器扭矩NM
                        60-66	左臂摩擦力估计值
                        70-76	左臂摩檫力速度估计值
                        80-85   左臂关节外力估计值
                        90-95	左臂末端点外力估计值
                    右臂对应 + 100

                    eg1: 采集左臂和右臂的关节位置，一共14列， 采集1000行：
                        cols=14
                        idx=[0,1,2,3,4,5,6,
                             100,101,102,103,104,105,106,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0]
                        rows=1000
                        robot.collect_date(targetNum=cols,targetID=idx,recordNum=rows)

                    eg2: 采集左臂第二关节的速度和电流一共2列， 采集500行：
                        cols=2
                        idx=[11,31,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0]
                        rows=500
                        robot.collect_date(targetNum=cols,targetID=idx,recordNum=rows)
        '''
        targetNum_int=ctypes.c_int(targetNum)
        targetID_int=(ctypes.c_long * len(targetID))(*targetID)
        recordNum_int=ctypes.c_int(recordNum)
        return self.robot.OnStartGather(targetNum_int,targetID_int,recordNum_int)

    def stop_collect_data(self):
        '''停止采集数据
        注： 在行数采集满后会自动停止采集,若需要中途停止采集调用本函数并等待1ms之后会停止采集。
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnStopGather()

    def save_collected_data_to_path(self,path:str):
        '''将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''
        self.save_data_path=path.encode('utf-8')
        path_char=ctypes.c_char_p(self.save_data_path)
        return self.robot.OnSaveGatherData(path_char)

    def save_collected_data_as_csv_to_path(self,path:str):
        '''以csv格式将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''
        self.save_csv_path=path
        path_char=ctypes.c_char_p(self.save_csv_path)
        # path_char = ctypes.c_wchar_p(self.save_csv_path)

        return self.robot.OnSaveGatherDataCSV(path_char)

    def soft_stop(self, arm:str):
        '''机械臂急停
        :param arm: ‘A’, 'B', 'AB', 可以让一条臂软急停，或者两条臂都软急停。
        :return:
        '''
        try:
            if arm=='A':
                return self.robot.OnEMG_A()
            elif arm=='B':
                return self.robot.OnEMG_B()
            elif arm=='AB':
                return self.robot.OnEMG_AB()
        except Exception as e:
            print("ERROR:", e)


    def get_servo_error_code(self, arm:str):
       '''获取机械臂伺服错误码
       :param self:
       :param arm:
       :return: (7,1)错误列表， 16进制
       '''
       try:
           err_code_value = (ctypes.c_long * 7)()
           if arm=='A':
               self.robot.OnGetServoErr_A.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_A(ctypes.byref(err_code_value))
               # print('err_code_value',err_code_value[-1])
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               return err_code
           elif arm=='B':
               self.robot.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_B(ctypes.byref(err_code_value))
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               return err_code

       except Exception as e:
           print("ERROR:", e)


    def clear_error(self,arm:str):
        '''清错
        :return:无
        '''
        try:
            if arm=='A':
                return self.robot.OnClearErr_A()
            elif arm=='B':
                return self.robot.OnClearErr_B()
        except Exception as e:
            print(f'ERROR:{e}')


    def set_state(self,arm:str,state:int):
        '''设置状态
        :param state:
                   ARM_STATE_IDLE = 0,            //////// 下伺服
                   ARM_STATE_POSITION = 1,		//////// 位置跟随
                   ARM_STATE_PVT = 2,			//////// PVT
                   ARM_STATE_TORQ = 3,			//////// 扭矩
        :return:
        '''
        try:
            state_int = ctypes.c_int(state)
            if arm=="A":
                return self.robot.OnSetTargetState_A(state_int)
            elif arm=='B':
                return self.robot.OnSetTargetState_B(state_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_impedance_type(self, arm:str,type: int):
        '''设置阻抗类型
        :param type:
            Type = 1 关节阻抗
            Type = 2 坐标阻抗
            Type = 3 力控
            注：需要在ARM_STATE_TORQ状态: set_state(arm='A',state=3)  才能以阻抗模式控制!!!
        :return:
            int : 1: True,  2: False
        '''
        try:
            type_int = ctypes.c_int(type)
            if arm=='A':
                return self.robot.OnSetImpType_A(type_int)
            elif arm == 'B':
                return self.robot.OnSetImpType_B(type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_vel_acc(self, arm:str, velRatio: int, AccRatio: int):
        '''设置速度和加速度百分比
        :param velRatio: 速度百分比
        :param AccRatio: 加速度百分比
        :return:
            int： 1: True; 0:Flase
        '''
        try:
            velRatio_int = ctypes.c_int(velRatio)
            AccRatio_int = ctypes.c_int(AccRatio)
            if arm=='A':
                return self.robot.OnSetJointLmt_A(velRatio_int, AccRatio_int)
            elif arm=='B':
                return self.robot.OnSetJointLmt_B(velRatio_int, AccRatio_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_tool(self,arm:str, kineParams: list, dynamicParams: list):
        '''设置工具信息
        :param kineParams: list(6,1). 运动学参数 XYZABC 单位毫米和度
        :param dynamicParams: list(10,1). 动力学参数分别为 质量M  质心[3]:mx,my,mz 惯量I[6]:XX,XY,XZ,YY,YZ,ZZ
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5 = kineParams
            d0, d1, d2, d3, d4, d5, d6, d7, d8, d9 = dynamicParams
            kp_double = ctypes.c_double * 6
            kineParams_value = kp_double(k0, k1, k2, k3, k4, k5)
            dp_double = ctypes.c_double * 10
            dynamicParams_value = dp_double(d0, d1, d2, d3, d4, d5, d6, d7, d8, d9)
            if arm=='A':
                return self.robot.OnSetTool_A(kineParams_value, dynamicParams_value)
            if arm=='B':
                return self.robot.OnSetTool_B(kineParams_value, dynamicParams_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_kd_params(self,arm:str, K: list, D: list):
        '''设置关节阻抗参数

        #关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
        1-7关节刚度不超过2
        1-7关节阻尼0-1之间
        :param K: list(7,1). 刚度 牛米 / 度 。 设置每个轴的的力为刚度系数。 如K=[2，2,2,1,1,1,1]，第1到3轴有2N作为刚度系数参与控制计算，第4到7轴有1N作为刚度系数参与控制计算。
        :param D: list(7,1). 阻尼 牛米 / (度 / 秒)。 设置每个轴的的阻尼系数。
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5, k6 = K
            d0, d1, d2, d3, d4, d5, d6 = D

            k_double = ctypes.c_double * 7
            k_value = k_double(k0, k1, k2, k3, k4, k5, k6)
            d_double = ctypes.c_double * 7
            d_value = d_double(d0, d1, d2, d3, d4, d5, d6)
            if arm=="A":
                return self.robot.OnSetJointKD_A(k_value, d_value)
            elif arm == "B":
                return self.robot.OnSetJointKD_B(k_value, d_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_cart_kd_params(self, arm:str, K: list, D: list, type: int):
        '''设置笛卡阻抗尔参数
            # 在笛卡尔阻抗模式下：
            刚度系数： 1-3平移方向刚度系数不超过3000, 4-6旋转方向不超过100。 零空间刚度系数不超过20
            阻尼系数： 平移和旋转阻尼系数0-1之间。 零空间阻尼系数不超过1
            零空间控制是保持末端固定不动，手臂角度运动的控制方式。接口未开放

        :param K: list(7,1). K[0]-k[2] N*m，x,y,z 平移方向每米的控制力; K[3]-k[5] N*m/rad, rx,ry,rz旋转弧度的控制力;K[6]N*m/rad,零空间总和刚度系数
        :param D: list(7,1). D[0]-D[5]  阻尼比例系数, D[6] 零空间总和阻尼比例系数
        :param type:int. set_A_arm_impedance_type设置的阻抗类型
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5, k6 = K
            d0, d1, d2, d3, d4, d5, d6 = D
            k_double = ctypes.c_double * 7
            k_value = k_double(k0, k1, k2, k3, k4, k5, k6)
            d_double = ctypes.c_double * 7
            d_value = d_double(d0, d1, d2, d3, d4, d5, d6)
            type_int = ctypes.c_int(type)
            if arm=="A":
                return self.robot.OnSetCartKD_A(k_value, d_value, type_int)
            if arm == "B":
                return self.robot.OnSetCartKD_B(k_value, d_value, type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_force_control_params(self,arm:str, fcType: int, fxDirection: list, fcCtrlpara: list, fcAdjLmt: float):
        '''设置力控参数
        :param fcType: 力控类型 0:坐标空间力控;1:工具空间力控(暂未实现)
        :param fxDirection: list(6,1). 力控方向 需要控制方向设1，目前只支持 X,Y,Z控制方向.如力控方向为z,fxDirection=[0,0,1,0,0,0]
        :param fcCtrlpara: list(7,1). 控制参数 目前全0
        :param fcAdjLmt:毫米，允许的调节范围
        :return:
            int : 1: True,  2: False
        '''
        try:
            fc_int=ctypes.c_int(fcType)
            k0, k1, k2, k3, k4, k5 = fxDirection
            d0, d1, d2, d3, d4, d5, d6 = fcCtrlpara
            fxDir_arr = (ctypes.c_double * 6)( k0, k1, k2, k3, k4, k5 )
            fcCtrlPara_arr = (ctypes.c_double * 7)(d0, d1, d2, d3, d4, d5, d6 )
            adj_double=ctypes.c_double(fcAdjLmt)
            if arm=='A':
                return self.robot.OnSetForceCtrPara_A(
                    fc_int,
                    fxDir_arr,
                    fcCtrlPara_arr,
                    adj_double)
            elif arm=='B':
                return self.robot.OnSetForceCtrPara_B(
                    fc_int,
                    fxDir_arr,
                    fcCtrlPara_arr,
                    adj_double)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_cmd_pose(self,arm:str, joints:list):
        '''设置关节跟踪指令值
        :param joints: list(7,1). 角度，非弧度，在位置跟随和扭矩模式下均有效
        :return:
            int : 1: True,  2: False
        '''
        try:
            j0, j1, j2, j3, j4, j5, j6= joints
            joints_double = ctypes.c_double * 7
            joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)
            if arm=='A':
                return self.robot.OnSetJointCmdPos_A(joints_value )
            elif arm == 'B':
                return self.robot.OnSetJointCmdPos_B(joints_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_force_cmd(self,arm:str, f:float):
        '''设置力控参数
        :param f: 目标力 单位牛或者牛米
        :return:
            int : 1: True,  2: False
        '''
        try:
            f_double=ctypes.c_double(f)
            if arm=='A':
                return self.robot.OnSetForceCmd_A(f_double)
            elif arm == 'B':
                return self.robot.OnSetForceCmd_B(f_double)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_pvt_id(self,arm:str,id:int):
        '''设置指定id号的pvt路径并运行
        :param id: 范围1-99. 需要在 ARM_STATE_PVT 状态，即： set_arm_state(arm='A',state=2)
        :return:
            int : 1: True,  2: False
        '''
        try:
            if arm=='B':
                id_int = ctypes.c_int(id)
                return self.robot.OnSetPVT_B(id_int)
            elif arm=='A':
                id_int = ctypes.c_int(id)
                return self.robot.OnSetPVT_A(id_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def send_pvt_file(self,arm:str, pvt_path: str, id: int):
        '''上传PVT文件给指定ID
        :param pvt_path: 本地pvt文件的绝对/相对路径
        :param id:
        :return:


            PVT文件格式见：DEMO_SRS_Left.fmv
            数据首行为行数和列数信息，“PoinType=9@9341 ”表示该PVT文件含9列数据，一共9341个点位。
            数据为什么是9列？ 首先前八列为关节角度， 为什么是8？ 我们预留了8关节，人形臂为7自由度，前7个有效值，第八列都填充0，
            好的，第九列，第九列是个标记列，全填0即可。
        '''
        try :
            if arm=='A':
                self.a_pvt_path = pvt_path.encode('utf-8')
                pvt_char = ctypes.c_char_p(self.a_pvt_path)
                id_int = ctypes.c_int(id)
                # print(f'send local pvt file:{pvt_path} to robot')
                return  self.robot.OnSendPVT_A(pvt_char, id_int)
            elif arm=='B':
                self.b_pvt_path = pvt_path.encode('utf-8')
                pvt_char = ctypes.c_char_p(self.b_pvt_path)
                id_int = ctypes.c_int(id)
                # print(f'send local pvt file:{pvt_path} to robot')
                return self.robot.OnSendPVT_B(pvt_char, id_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_drag_space(self,arm:str, dgType: int):
        '''设置拖动空间
        :param dgType:
                0 退出拖动模式
                1 关节空间拖动
                2 笛卡尔空间x方向拖动
                3 笛卡尔空间y方向拖动
                4 笛卡尔空间z方向拖动
                5 笛卡尔空间旋转方向拖动
        :return:
        '''
        try:
            type_int = ctypes.c_int(dgType)
            if arm=='A':
                return self.robot.OnSetDragSpace_A(type_int)
            elif arm=='B':
                return self.robot.OnSetDragSpace_B(type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def receive_file(self, local_path: str, remote_path: str):
        '''将械臂控制器文件发送到本地
        :param local_path: 本地绝对路径
        :param remote_path: 机械臂控制器绝对路径
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnRecvFile(local_char, remote_char)

    def save_para_file(self):
        '''保存配置文件
        :return:
        '''
        id = self.robot.OnSavePara()
        if id == -1 or id == 2:
            print("save parameter failed.")
            return id
        else:
            print(f'index of saved parameter is {id}')
            return id

    def log_switch(self,flag:str):
        try:
            if flag=='1':
                return self.robot.OnLogOn()
            elif flag=='0':
                return self.robot.OnLogOff()
        except Exception as e:
            print(f'ERROR:{e}')


    def local_log_switch(self,flag:str):
        try:
            if flag=='1':
                return self.robot.OnLocalLogOn()
            elif flag=='0':
                return self.robot.OnLocalLogOff()
        except Exception as e:
            print(f'ERROR:{e}')

    def clear_485_cache(self,arm:str):
        '''清空发送缓存

        :param arm: 机械手臂ID “A” OR “B”
        :return: bool
        '''
        try:
            if arm == 'A':
                return self.robot.OnClearChDataA()
            elif arm == 'B':
                return self.robot.OnClearChDataB()
        except Exception as e:
            print(f'ERROR:{e}')

    def set_485_data(self, arm: str, data:bytes, size_int:int,com:int):
        '''发送数据到485的指定来源， 每次长度不超过256字节，超过就切成多个包发。

        :param arm: 机械手臂ID “A” OR “B”
        :param data: 要传递的字节数据 (长度不超过2256)
        :param size_int: int, 发送的字节长度，不能超过256
        :param com: 信息来源， 1：‘C’端; 2：com1; 3:com2
        :return: bool
        '''

        try:
            # 定义函数原型
            self.robot.OnSetChDataA.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
            self.robot.OnSetChDataA.restype = ctypes.c_bool

            # 定义函数原型
            self.robot.OnSetChDataB.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
            self.robot.OnSetChDataB.restype = ctypes.c_long

            # 验证参数
            if len(data) >= 257:
                raise ValueError(f"数据长度({len(data)})超过256字节限制")
            if size_int >= 257:
                print(f"size_int({size_int})超过256，将被截断")
                size_int = 256

            result = identify_and_calculate_length(data)
            if result['type'] == "hex string" or result['type'] == 'bytes' or result[
                'type'] == "bytes representation string":
                print("-" * 50)
                print(f"输入: {data}")
                print(f"类型: {result['type']}")
                print(f"字节长度: {result['length_bytes']}")
                print(f"字节表示: {result['bytes_representation']}")
                print("-" * 50)
            else:
                print(f"ERROR: set_485_data input must be hex string of bytes string")
                return False, False

            size_int_long = ctypes.c_long(result['length_bytes'])
            com_long = ctypes.c_long(com)

            data_buffer = (ctypes.c_ubyte * 256)()
            # 复制数据到缓冲区
            data_length = min(len(result['bytes_representation']), size_int)
            for i in range(data_length):
                data_buffer[i] = result['bytes_representation'][i]
            if arm == 'A':
                return True, self.robot.OnSetChDataA(data_buffer, size_int_long, com_long)
            elif arm == 'B':
                return True, self.robot.OnSetChDataB(data_buffer, size_int_long, com_long)
        except Exception as e:
            print(f'ERROR:{e}')


    def get_485_data(self, arm: str,com:int):
        '''收指定来源的485数据
        :param arm: 机械手臂ID “A” OR “B”
        :param com: 信息来源， 1：‘C’端; 2：com1; 3:com2
        :return: int, 长度size
        '''
        try:
            # 创建 256 字节缓冲区
            data_buffer = (ctypes.c_ubyte * 256)()
            ret_ch = ctypes.c_long(com)
            if arm == 'A':
                result = self.robot.OnGetChDataA(data_buffer, ctypes.byref(ret_ch))
                # 提取字节数据
                byte_data = bytes(data_buffer)  # 或 bytearray(data_buffer)
                print(f'arm receive byte_data :{byte_data}')
                hex_list = []
                for byte in byte_data:
                    # 将每个字节转换为两位十六进制
                    hex_value = hex(byte)[2:].upper().zfill(2)
                    hex_list.append(hex_value)

                return result, ' '.join(hex_list)

            elif arm == 'B':
                result = self.robot.OnGetChDataB(data_buffer, ctypes.byref(ret_ch))
                # 提取字节数据
                byte_data = bytes(data_buffer)  # 或 bytearray(data_buffer)
                # print(f'B arm receive byte_data :{byte_data }')
                hex_list = []
                for byte in byte_data:
                    # 将每个字节转换为两位十六进制
                    hex_value = hex(byte)[2:].upper().zfill(2)
                    hex_list.append(hex_value)

                return result, ' '.join(hex_list)

        except Exception as e:
            print(f'ERROR:{e}')

    def identify_tool_dyn(self, robot_type: int, ipath: str):
        '''工具动力学参数辨识
        FX_BOOL  FX_Robot_Iden_LoadDyn(FX_INT32L Type,FX_CHAR* path,FX_DOUBLE mass, Vect3 mr, Vect6 I);
        :param robot_type: int ,机型，从CONFIG导入
        :param ipath: sting, 相对路径导入工具辨识轨迹数据。
        :return:
            m,mcp,i
        '''
        if type(robot_type) != int:
            raise ValueError("robot_type must be int type")

        if not os.path.exists(ipath):
            raise ValueError(f"no {ipath}, pls check!")

        robot_type_ = ctypes.c_int(robot_type)
        iden_path = ipath.encode('utf-8')
        path_char = ctypes.c_char_p(iden_path)

        # 创建指针变量而不是数组
        mm_ptr = ctypes.pointer(ctypes.c_double(0))
        mcp_ptr = (ctypes.c_double * 3)()
        ii_ptr = (ctypes.c_double * 6)()

        # 设置函数原型
        self.robot.FX_Robot_Iden_LoadDyn.argtypes = [
            ctypes.c_long,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double * 3),
            ctypes.POINTER(ctypes.c_double * 6)
        ]
        self.robot.FX_Robot_Iden_LoadDyn.restype = ctypes.c_bool

        # 调用函数
        success1 = self.robot.FX_Robot_Iden_LoadDyn(
            robot_type_,
            path_char,
            mm_ptr,
            mcp_ptr,
            ii_ptr
        )

        if success1:
            print('Identify tool dynamics successful')

            # 提取结果
            dyn_para = []
            m_val = mm_ptr.contents.value
            mcp_list = [mcp_ptr[i] for i in range(3)]
            ii_list = [ii_ptr[i] for i in range(6)]

            dyn_para.append(m_val)
            for i in mcp_list:
                dyn_para.append(i)
            for j in ii_list:
                dyn_para.append(j)

            print(f'tool dynamics: {dyn_para}')
            return dyn_para
        else:
            print('****error: Identify tool dynamics failed!')
            return False
        


    def help(self, method_name: str = None) -> None:
        """
        显示帮助信息

        参数:
            method_name (str): 可选的方法名，显示特定方法的帮助信息
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



if __name__ == "__main__":
    import time

    tj_robot = Marvin_Robot()
    tj_robot.help()
    tj_robot.help('collect_data')

