#!/usr/bin/env python3
"""
实时机器人控制节点
功能：连接机器人，执行实时控制，发布状态信息
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
import threading
import time

# 导入机器人控制库
import ctypes
import inspect
from textwrap import dedent
import os
import re
from typing import Union

# current_path = os.getcwd()

current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)


def update_text_file_simple(mode, data_list, filename):
    """
    简化版的文件更新函数
    """
    if mode not in ['A', 'B'] or len(data_list) != 16:
        return False
    try:
        # 如果文件存在，读取内容；否则创建默认内容
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                lines = file.readlines()
        # 更新对应行
        line_index = 0 if mode == 'A' else 1
        lines[line_index] = ','.join(str(x) for x in data_list) + '\n'

        # 写回文件
        with open(filename, 'w', encoding='utf-8') as file:
            file.writelines(lines)
        return True
    except Exception as e:
        print(f"更新文件时出错: {e}")
        return False

def read_csv_file_to_float_strict(filename, expected_columns=16):
    """
    读取CSV格式的文件内容并转换为float，严格验证每列数量

    参数:
        filename: 文件名
        expected_columns: 期望的列数（默认16）

    返回:
        如果文件为空: 返回0
        如果文件有一行: 返回 [float1, float2, ...]
        如果文件有两行: 返回 [[float1, float2, ...], [float1, float2, ...]]
        如果文件不存在或转换失败: 返回-1
    """

    if os.path.getsize(filename) == 0:
        return 0

    try:
        with open(filename, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        non_empty_lines = [line.strip() for line in lines if line.strip()]

        if len(non_empty_lines) == 0:
            return 0

        all_float_data = []
        for line_num, line in enumerate(non_empty_lines, 1):
            values = line.split(',')
            # 过滤空值并去除空格
            cleaned_values = [v.strip() for v in values if v.strip()]

            # 验证列数
            if len(cleaned_values) != expected_columns:
                print(f"第{line_num}行: 期望{expected_columns}列，实际找到{len(cleaned_values)}列")
                return -1

            float_values = []
            for value in cleaned_values:
                try:
                    float_value = float(value)
                    float_values.append(float_value)
                except ValueError:
                    print(f"第{line_num}行: 无法将内容 '{value}' 转换为float")
                    return -1

            all_float_data.append(float_values)

        # 根据行数返回不同格式
        if len(all_float_data) == 1:
            return all_float_data[0]
        elif len(all_float_data) == 2:
            return all_float_data
        else:
            print(f"文件包含{len(all_float_data)}行，只支持1-2行")
            return -1

    except Exception as e:
        print(f"读取文件时出错: {e}")
        return -1
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
            self.robot = ctypes.CDLL(current_path + '/src/robot_control/robot_control/MarvinLib/libMarvinSDK.so')
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
                result = ctypes.c_int(value)
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
        path1='tmp.txt'
        self.save_data_path = path1.encode('utf-8')
        path_char = ctypes.c_char_p(self.save_data_path)
        self.robot.OnSaveGatherData(path_char)
        import time
        time.sleep(0.2)

        import csv
        with open(path1, 'r') as file:
            lines = file.readlines()
        processed_data=[]
        lines = lines[1:]
        for i, line in enumerate(lines):
            parts = line.strip().split('$')
            numbers = []
            for part in parts:
                if part:
                    num_str = part.split()[-1]
                    numbers.append(num_str)
            if len(numbers) >= 2:
                numbers = numbers[2:]
            processed_data.append(numbers)

        try:
            with open(path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(processed_data)
            print(f"数据已成功保存到: {path}")
            if os.path.exists(path1):
                os.remove(path1)
            return True
        except Exception as e:
            print(f"保存失败: {e}")
            if os.path.exists(path1):
                os.remove(path1)
            return False


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
                   ARM_STATE_RELEASE = 4,		//////// 协作释放

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
        '''将机械臂控制器下载到上位机文件
        :param local_path: 本地绝对路径
        :param remote_path: 机械臂控制器绝对路径
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnRecvFile(local_char, remote_char)


    def send_file(self, local_path: str, remote_path: str):
        '''将上位机文件上传到机械臂控制器
        :param local_path: 本地绝对路径
        :param remote_path: 机械臂控制器绝对路径
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnSendFile(local_char, remote_char)



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




from ctypes import *
import ctypes
import inspect
from textwrap import dedent
import os
import math
import logging
from robot_structures import *

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


class Marvin_Kine:
    def __init__(self):
        """初始化机器人控制类"""
        import sys
        logger.info(f'user platform: {sys.platform}')
        if sys.platform == 'win32':
            self.kine = ctypes.WinDLL(os.path.join(current_path, 'libKine.dll'))
        else:
            self.kine = ctypes.CDLL(os.path.join(current_path, '/src/robot_control/robot_control/MarvinLib/libKine.so'))

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
            logger.info(f'fk result, matrix:{fk_mat}')
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
        特别提示:1 直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
            2 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
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


    def movL_KeepJ(self,robot_serial: int,start_joints:list, end_joints:list,vel:float,save_path):
        '''直线规划保持关节构型（MOVL KeepJ）

        :param robot_serial: int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param start_joints:
        :param end_joints:
        :param vel:
        :param save_path:
        :return: bool
        特别提示:1 直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
            2 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
        '''
        if robot_serial != 0 and robot_serial != 1:
            raise ValueError("robot_serial must be 0 or 1")

        Serial = ctypes.c_long(robot_serial)

        path = save_path.encode('utf-8')
        path_char = ctypes.c_char_p(path)

        s0,s1,s2,s3,s4,s5,s6=start_joints
        start= (ctypes.c_double * 7)( s0,s1,s2,s3,s4,s5,s6)

        e0,e1,e2,e3,e4,e5,e6=end_joints
        end= (ctypes.c_double * 7)(e0,e1,e2,e3,e4,e5,e6)

        vel_value=c_double(vel)

        self.kine.FX_Robot_PLN_MOVL_KeepJ.argtypes=[c_long,c_double*7,c_double*7,c_double,c_char_p]
        self.kine.FX_Robot_PLN_MOVL_KeepJ.restype=c_bool
        success1=self.kine.FX_Robot_PLN_MOVL_KeepJ(Serial,start,end,vel_value,path_char)
        if success1:
            logger.info(f'Plan MOVL KeepJ successful, PATH saved as :{save_path}')
            return  True

        else:
            logger.error(f'Plan MOVL KeepJ failed!')
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
        self.kine.FX_Robot_Iden_LoadDyn.restype = c_int32

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


from ctypes import *

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


# 示例使用
if __name__ == "__main__":
    # 创建结构体实例
    params = FX_InvKineSolvePara()

    # 设置输入值
    params.m_Input_IK_TargetTCP = Matrix4([i for i in range(16)])
    params.m_Input_IK_RefJoint = Vect7([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    params.m_Input_IK_ZSPType = 1
    params.set_input_ik_zsp_para([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    params.m_Input_ZSP_Angle = 45.0
    params.m_DGR1 = 10.0
    params.m_DGR2 = 20.0
    params.m_DGR3 = 30.0

    # 设置输出值（假设从C++函数返回）
    params.m_Output_RetJoint = Vect7([1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7])
    params.m_Output_IsOutRange = False
    params.set_output_is_deg([True, False, True, False, True, False, True])
    params.set_output_jnt_exd_tags([False, True, False, True, False, True, False])
    params.m_Output_IsJntExd = True
    params.m_Output_RunLmtP = Vect7([2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7])
    params.m_Output_RunLmtN = Vect7([3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7])

    # 打印一些值进行验证
    print("Input IK ZSP Para:", params.get_input_ik_zsp_para())
    print("Output IsDeg:", params.get_output_is_deg())
    print("Output JntExdTags:", params.get_output_jnt_exd_tags())
    print("Output RetJoint:", params.m_Output_RetJoint.to_list())

    # 创建 FX_Jacobi 实例
    jacobi = FX_Jacobi()

    # 设置轴数
    jacobi.m_AxisNum = 6

    # 创建一个示例雅可比矩阵 (6x7)
    example_matrix = [
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    ]

    # 设置雅可比矩阵
    jacobi.set_jcb(example_matrix)

    # 打印结构体内容
    print(jacobi)

    # 获取并打印雅可比矩阵
    matrix = jacobi.get_jcb()
    print("获取的雅可比矩阵:")
    for row in matrix:
        print(row)


class RealtimeRobotControl(Node):
    """实时机器人控制节点"""

    def __init__(self):
        super().__init__('realtime_robot_control')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', '192.168.1.190'),
                ('control_frequency', 10.0),  # Hz
                ('velocity_ratio', 50.0),  # %
                ('acceleration_ratio', 50.0),  # %
                ('home_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ('max_sine_amplitude', 10.0),  # 正弦运动最大幅度
                ('auto_reconnect', True),
            ]
        )

        # 获取参数值
        self.robot_ip = self.get_parameter('robot_ip').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.velocity_ratio = self.get_parameter('velocity_ratio').value
        self.acceleration_ratio = self.get_parameter('acceleration_ratio').value
        self.home_position = self.get_parameter('home_position').value
        self.max_sine_amplitude = self.get_parameter('max_sine_amplitude').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value

        # 初始化变量
        self.robot = None
        self.dcss = None
        self.is_connected = False
        self.is_running = False
        self.control_thread = None
        self.frame_serial = 0
        self.cycle_count = 0

        # 创建定时器用于状态检查
        self.status_timer = self.create_timer(1.0, self.check_status)

        # 初始化机器人
        self.init_robot()

        self.get_logger().info(f'机器人控制节点已启动，IP: {self.robot_ip}')

    def init_robot(self):
        """初始化机器人连接"""
        try:
            self.robot = Marvin_Robot()
            self.dcss = DCSS()

            # 连接到机器人
            if self.connect_robot():
                self.setup_robot()
                self.is_connected = True
                self.start_control_loop()
            else:
                self.get_logger().error('机器人连接失败')
                if self.auto_reconnect:
                    self.get_logger().info('5秒后尝试重新连接...')
                    self.create_timer(5.0, self.reconnect_robot, oneshot=True)

        except Exception as e:
            self.get_logger().error(f'初始化机器人失败: {str(e)}')

    def connect_robot(self):
        """连接到机器人"""
        try:
            self.get_logger().info(f'正在连接机器人 {self.robot_ip}...')

            init = self.robot.connect(self.robot_ip)
            if init == 0:
                self.get_logger().error('端口占用，连接失败!')
                return False

            # 防总线通信异常，先清错
            time.sleep(0.5)
            self.robot.clear_set()
            self.robot.clear_error('A')
            self.robot.clear_error('B')
            self.robot.send_cmd()
            time.sleep(0.5)

            # 检查连接状态
            motion_tag = 0
            frame_update = None

            for i in range(10):
                sub_data = self.robot.subscribe(self.dcss)
                if sub_data and 'outputs' in sub_data and len(sub_data['outputs']) > 0:
                    current_frame = sub_data['outputs'][0]['frame_serial']
                    self.get_logger().debug(f"连接帧: {current_frame}")

                    if current_frame != 0 and frame_update != current_frame:
                        motion_tag += 1
                        frame_update = current_frame

                time.sleep(0.001)

            if motion_tag > 0:
                self.get_logger().info('机器人连接成功!')
                return True
            else:
                self.get_logger().error('机器人连接失败!')
                return False

        except Exception as e:
            self.get_logger().error(f'连接过程中出错: {str(e)}')
            return False

    def reconnect_robot(self):
        """重新连接机器人"""
        if not self.is_connected and self.auto_reconnect:
            self.get_logger().info('尝试重新连接机器人...')
            self.init_robot()

    def setup_robot(self):
        """设置机器人参数"""
        try:
            # 设置机器人到位置跟随模式
            self.robot.clear_set()
            self.robot.set_state('A', 1)  # 1 = 位置跟随模式

            # 设置速度和加速度
            self.robot.set_vel_acc('A',
                                   int(self.velocity_ratio),
                                   int(self.acceleration_ratio))
            self.robot.send_cmd()
            time.sleep(0.5)

            self.get_logger().info(
                f"机器人设置为位置跟随模式，速度: {self.velocity_ratio}%，加速度: {self.acceleration_ratio}%")

            # 获取初始状态
            data = self.robot.subscribe(self.dcss)
            if data:
                current_states = data['states'][0]['cur_state']
                vel = data['inputs'][0]['joint_vel_ratio']
                acc = data['inputs'][0]['joint_acc_ratio']
                self.get_logger().info(f"当前状态: {current_states}, 速度: {vel}%, 加速度: {acc}%")

            # 移动到初始位置
            self.move_to_home()

        except Exception as e:
            self.get_logger().error(f'设置机器人参数失败: {str(e)}')

    def move_to_home(self):
        """移动到初始位置"""
        try:
            self.robot.clear_set()
            self.robot.set_joint_cmd_pose('A', self.home_position)
            self.robot.send_cmd()
            time.sleep(0.5)
            self.get_logger().info("已移动到初始位置")
        except Exception as e:
            self.get_logger().error(f'移动到初始位置失败: {str(e)}')

    def start_control_loop(self):
        """启动控制循环"""
        if not self.is_connected:
            self.get_logger().warning('机器人未连接，无法启动控制循环')
            return

        if self.is_running:
            self.get_logger().warning('控制循环已在运行中')
            return

        self.is_running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        self.get_logger().info('控制循环已启动')

    def stop_control_loop(self):
        """停止控制循环"""
        self.is_running = False
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
            self.control_thread = None
        self.get_logger().info('控制循环已停止')

    def control_loop(self):
        """控制循环主逻辑"""
        cycle_time = 1.0 / self.control_frequency
        start_time = time.time()

        try:
            while self.is_running and self.is_connected:
                cycle_start = time.time()

                # 获取当前关节位置
                data = self.robot.subscribe(self.dcss)
                if data:
                    joint_positions = data['outputs'][0]['fb_joint_pos']

                    # 记录帧序列号
                    current_frame = data['outputs'][0]['frame_serial']
                    if current_frame != self.frame_serial:
                        self.frame_serial = current_frame
                        self.cycle_count += 1

                    # 创建小幅度的正弦运动
                    # 使用不同的相位使每个关节运动不同
                    target_joints = []
                    for j in range(7):
                        phase = j * 0.5  # 每个关节相位差
                        amplitude = self.max_sine_amplitude * (1.0 - j * 0.1)  # 减小末端关节幅度
                        sine_value = np.sin(cycle_start * 1.0 + phase) * amplitude
                        target_joints.append(sine_value)

                    # 发送控制命令
                    self.robot.clear_set()
                    self.robot.set_joint_cmd_pose('A', target_joints)
                    self.robot.send_cmd()

                    # 发布状态信息（示例）
                    self.publish_status(joint_positions, target_joints)

                # 控制循环频率
                elapsed = time.time() - cycle_start
                sleep_time = max(0.0, cycle_time - elapsed)
                time.sleep(sleep_time)

        except Exception as e:
            self.get_logger().error(f'控制循环出错: {str(e)}')
            self.is_running = False

    def publish_status(self, current_positions, target_positions):
        """发布机器人状态（示例，需要根据实际需求实现）"""
        # 这里可以添加状态发布逻辑，例如：
        # 1. 发布到ROS话题
        # 2. 更新TF变换
        # 3. 记录日志

        # 示例：每100个周期记录一次
        if self.cycle_count % 100 == 0:
            self.get_logger().info(
                f"周期: {self.cycle_count}, "
                f"当前位置: {[f'{p:.2f}' for p in current_positions]}, "
                f"目标位置: {[f'{t:.2f}' for t in target_positions]}"
            )

    def check_status(self):
        """检查机器人状态"""
        if self.is_connected and self.robot:
            try:
                data = self.robot.subscribe(self.dcss)
                if data:
                    # 检查错误状态
                    if 'states' in data and len(data['states']) > 0:
                        error_code = data['states'][0].get('error_code', 0)
                        if error_code != 0:
                            self.get_logger().warning(f"机器人错误代码: {error_code}")

                    # 检查连接状态
                    frame_serial = data['outputs'][0]['frame_serial']
                    if frame_serial == 0:
                        self.get_logger().warning("机器人通信异常")

            except Exception as e:
                self.get_logger().error(f'状态检查失败: {str(e)}')
                self.is_connected = False

                if self.auto_reconnect:
                    self.get_logger().info('检测到连接断开，尝试重新连接...')
                    self.init_robot()

    def emergency_stop(self):
        """紧急停止"""
        self.get_logger().warn('执行紧急停止!')

        # 停止控制循环
        self.stop_control_loop()

        # 急停
        self.robot.clear_set()
        self.robot.soft_stop('A')
        self.robot.send_cmd()
        time.sleep(0.001)

        # 发送停止命令
        if self.robot and self.is_connected:
            try:
                # 回到初始位置
                self.move_to_home()
            except Exception as e:
                self.get_logger().error(f'紧急停止过程中出错: {str(e)}')

    def shutdown(self):
        """关闭节点"""
        self.get_logger().info('正在关闭机器人控制节点...')

        # 停止控制循环
        self.stop_control_loop()

        # 回到初始位置
        if self.is_connected and self.robot:
            try:
                self.move_to_home()
            except Exception as e:
                self.get_logger().error(f'返回初始位置失败: {str(e)}')

        # 断开机器人连接
        if self.robot:
            try:
                self.robot.release_robot()
                self.get_logger().info('机器人已断开连接')
            except Exception as e:
                self.get_logger().error(f'断开连接失败: {str(e)}')

        self.is_connected = False
        self.get_logger().info('机器人控制节点已关闭')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        # 创建节点
        node = RealtimeRobotControl()

        # 添加键盘中断处理
        def signal_handler(signum, frame):
            node.get_logger().info("接收到关闭信号")
            node.shutdown()
            rclpy.shutdown()

        import signal
        signal.signal(signal.SIGINT, signal_handler)

        # 运行节点
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"节点运行出错: {str(e)}")
    finally:
        # 确保清理
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()