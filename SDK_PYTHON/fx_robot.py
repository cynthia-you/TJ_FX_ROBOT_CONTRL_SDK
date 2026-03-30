import csv
import ctypes
import inspect
import os
import re
import time
from ctypes import *
from textwrap import dedent
from typing import Union

current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)

'''######################################## Original SDK ########################################################'''
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

    def check_error_and_clear(self,dcss):
        '''检查机械臂是否存在手臂或者伺服错误和状态切换错误'''
        sub_data=self.subscribe(dcss)
        a_state=sub_data["states"][0]["cur_state"]
        b_state=sub_data["states"][1]["cur_state"]
        a_arm_err=sub_data["states"][0]["err_code"]
        b_arm_err=sub_data["states"][1]["err_code"]

        err_code_value_a = (ctypes.c_long * 7)()
        self.robot.OnGetServoErr_A.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
        self.robot.OnGetServoErr_A(ctypes.byref(err_code_value_a))
        a_servo_err = [0] * 7
        err_code_value_b = (ctypes.c_long * 7)()
        self.robot.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
        self.robot.OnGetServoErr_B(ctypes.byref(err_code_value_b))
        b_servo_err = [0] * 7
        for i in range(7):
            a_servo_err[i] = err_code_value_a[i]
            b_servo_err[i] = err_code_value_b[i]

        if a_state==100 or a_arm_err!=0 :
            self.clear_error('A')
            print(f"Arm A exits error, clear error")
        elif b_state==100 or b_arm_err!=0:
            self.clear_error('B')
            print(f"Arm B exits error, clear error")
        elif set(a_servo_err) != {0}:
            self.clear_error('A')
            print(f"Arm A exits servo error, clear error")
        elif set(b_servo_err) != {0}:
            self.clear_error('B')
            print(f"Arm B exits servo error, clear error")
        else:
            print(f'Arms A&B exist no error')

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
        return self.robot.OnUpdateSystem(sdk_char)

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
            para=get_float_params('float','R.A0.CTRL.CartJNTDampJ1')

            #整数类型参数获取：
            我想获取[R.A0.BASIC]这个参数组里Type的值
            para=get_int_params('int','R.A0.BASIC.Type')
        '''
        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # 显式添加终止符
            if type=='float':
                result = ctypes.c_double(0)
                self.robot.OnGetFloatPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetFloatPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, float parameters={result.value}")
                return re_flag,result.value
            elif type=='int':
                result = ctypes.c_int(0)
                self.robot.OnGetIntPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetIntPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, int parameters={result.value}")
                return re_flag, result.value
        except Exception as e:
            print("ERROR:",e)

    def save_para_file(self):
        '''保存配置文件
        :return:
        '''
        self.robot.OnSavePara.restype = ctypes.c_long
        return self.robot.OnSavePara()

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
                self.robot.OnSetFloatPara.restype = ctypes.c_long
                return self.robot.OnSetFloatPara(param_buf, result)
            elif type=='int':
                result = ctypes.c_int(int(value))
                self.robot.OnSetIntPara.restype = ctypes.c_long
                return self.robot.OnSetIntPara(param_buf, result)
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

    def send_cmd_wait_response(self, timeout:int):
        '''发送指令等待回应
        :param timeout: 等待指令响应延时的时间， 单位：毫秒。 建议50-100毫秒
        :return: 延时时间
            0：超时
            -1：错误
        '''
        timeout_long = ctypes.c_long(timeout)
        self.robot.OnSetSendWaitResponse.restype = ctypes.c_long
        return self.robot.OnSetSendWaitResponse(timeout_long)

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

        time.sleep(0.2)
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

    def soft_stop(self, arm: str):
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

    def servo_reset(self,arm:str,axis:int):
        '''指定轴伺服软复位
        :param arm: 机械手臂ID “A” OR “B”
        :param axis: 指定关节：0-6
        :return:
        '''
        try:
            axis_int = ctypes.c_int(axis)
            if arm=='A':
                return self.robot.OnServoReset_A(axis_int)
            elif arm=='B':
                return self.robot.OnServoReset_B(axis_int)
        except Exception as e:
            print("ERROR:", e)

    def get_servo_error_code(self, arm:str,lang='CN'):
       '''获取机械臂伺服错误码
       :param self:
       :param arm: 机械手臂ID “A” OR “B”
       :param lang: 'CN' or 'EN'
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
               err_code=get_fault_descriptions(err_code,lang)
               return err_code
           elif arm=='B':
               self.robot.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_B(ctypes.byref(err_code_value))
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               err_code = get_fault_descriptions(err_code,lang)
               return err_code

       except Exception as e:
           print("ERROR:", e)

    def clear_error(self,arm:str):
        '''清错
        :param arm: 机械手臂ID “A” OR “B”
        :return:
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
        :param arm: 机械手臂ID “A” OR “B”
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
        :param arm: 机械手臂ID “A” OR “B”
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
        :param arm: 机械手臂ID “A” OR “B”
        :param velRatio: 速度百分比, 值范围： 0~100
        :param AccRatio: 加速度百分比， 值范围：0~100
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
        :param arm: 机械手臂ID “A” OR “B”
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
        1-7关节阻尼0-1之间
        :param arm: 机械手臂ID “A” OR “B”
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
        :param arm: 机械手臂ID “A” OR “B”
        :param K: list(7,1). K[0]-k[2] N*m，x,y,z 平移方向每米的控制力; K[3]-k[5] N*m/rad, rx,ry,rz旋转弧度的控制力;K[6]N*m/rad,零空间总和刚度系数
        :param D: list(7,1). D[0]-D[5]  阻尼比例系数, D[6] 零空间总和阻尼比例系数,范围0-1
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
        :param arm: 机械手臂ID “A” OR “B”
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

    def set_EefCart_control_params(self,arm:str, fcType: int, CartCtrlPara: list):
        '''设置末端笛卡尔阻抗参数
        :param arm: 机械手臂ID “A” OR “B”
        :param fcType:
              fcType=1，为自定义末端旋转方向。 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
              fcType=2，为系统自动计算末端笛卡尔旋转。 CartCtrlPara全填0
        :param CartCtrlPara: list(7,1). 控制参数前三个为旋转信息，基于基座的XYZ旋转。
        :return:
            int : 1: True,  2: False
        '''
        try:
            fc_int=ctypes.c_int(fcType)
            d0, d1, d2, d3, d4, d5, d6 = CartCtrlPara
            CartCtrlPara_arr = (ctypes.c_double * 7)(d0, d1, d2, d3, d4, d5, d6)

            if arm=='A':
                return self.robot.OnSetEefRot_A(
                    fc_int,
                    CartCtrlPara_arr)
            elif arm=='B':
                return self.robot.OnSetEefRot_B(
                    fc_int,
                    CartCtrlPara_arr)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_cmd_pose(self,arm:str, joints:list):
        '''设置关节跟踪指令值
        :param arm: 机械手臂ID “A” OR “B”
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
        '''设置力控指令
        :param arm: 机械手臂ID “A” OR “B”
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
        :param arm: 机械手臂ID “A” OR “B”
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
        :param arm: 机械手臂ID “A” OR “B”
        :param pvt_path: 本地pvt文件的绝对/相对路径
        :param id: 范围1-99. 需要在 ARM_STATE_PVT 状态，即： set_arm_state(arm='A',state=2)
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

    def pln_init(self,config_path):
        '''关节空间规划初始化
        :param config_path: 本地机械臂配置文件*.MvKDCfg, 可相对路径.
        :return:
            ture or false
        '''
        if not os.path.exists(config_path):
            raise ValueError("no config file")
        self.robot.OnInitPlnLmt.argtypes = [
            c_char_p,  # FX_CHAR* path
        ]
        self.robot.OnInitPlnLmt.restype = c_bool
        return self.robot.OnInitPlnLmt(config_path.encode('utf-8'))

    def stopRunPln_joint(self, arm: str):
        '''停止之前的规划运动，关节空间和笛卡尔空间都适用t
        :param arm: 机械手臂ID “A” OR “B”
        '''
        try:
            if arm not in ['A', 'B']:
                raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
            if arm == "A":
                return self.robot.OnStopPlnJoint_A()
            if arm == "B":
                return self.robot.OnStopPlnJoint_B()
        except Exception as e:
            print(f'ERROR:{e}')

    def setPln_joint(self,arm:str,start_joints:list, target_joints:list, velRatio:float,accRatio:float):
        '''位置模式下使用该接口传输目标关节点位，防止通信抖动
        :param arm: 机械手臂ID “A” OR “B”
        :param start_joints list(7, 1).起始点关节位置，单位角度
        :param target_joints list(7, 1).目标关节位置，单位角度
        :param velRatio float .规划插点的速度百分比， 范围0~1
        :param accRatio float .规划插点的加速度百分比，范围0~1
        :return:
            ture or false
        '''
        try:
            if arm not in ['A', 'B']:
                raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
            if len(start_joints) != 7:
                raise ValueError("shape error: start joints must be (7,)")
            if len(target_joints) != 7:
                raise ValueError("shape error: target joints must be (7,)")
            if type(velRatio)!=float:
                raise ValueError("value error: velRatio must be float, range:0~1 ")
            if type(accRatio)!=float:
                raise ValueError("value error: accRatio must be float, range:0~1 ")
            if velRatio>1 or velRatio<=0:
                raise ValueError("value error: velRatio range:0~1 ")
            if accRatio>1 or accRatio<=0:
                raise ValueError("value error: accRatio range:0~1 ")

            sj0,sj1,sj2,sj3,sj4,sj5,sj6 = start_joints
            tj0,tj1,tj2,tj3,tj4,tj5,tj6 = target_joints
            k_double = ctypes.c_double * 7
            start_value = k_double(sj0,sj1,sj2,sj3,sj4,sj5,sj6 )
            d_double = ctypes.c_double * 7
            target_value = d_double(tj0,tj1,tj2,tj3,tj4,tj5,tj6)
            vel = ctypes.c_double(velRatio)
            acc = ctypes.c_double(accRatio)

            if arm == "A":
                return self.robot.OnSetPlnJoint_A(start_value, target_value, vel,acc)
            if arm == "B":
                return self.robot.OnSetPlnJoint_B(start_value, target_value, vel,acc)
        except Exception as e:
            print(f'ERROR:{e}')

    def setPln_Cart(self,arm:str, pset: ctypes.c_void_p) -> bool:
        """位置模式下使用该接口传输目标笛卡尔坐标，防止通信抖动
        :param arm: 机械手臂ID “A” OR “B”
        :param pset: 由计算接口movLA(start_xyzabc: List[float], end_xyzabc: List[float],
              ref_joints: List[float], vel: float, acc: float,freq_hz:int,
              dimension: int = 7) -> tuple[List[List[float]], ctypes.c_void_p]计算得出
        :return:
            ture or false
        """
        if arm not in ['A', 'B']:
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        success=False
        if arm=='A':
            self.robot.OnSetPlnCart_A.argtypes = [ctypes.c_void_p]
            self.robot.OnSetPlnCart_A.restype = ctypes.c_bool
            success = self.robot.OnSetPlnCart_A(pset)
        elif arm=='B':
            self.robot.OnSetPlnCart_B.argtypes = [ctypes.c_void_p]
            self.robot.OnSetPlnCart_B.restype = ctypes.c_bool
            success = self.robot.OnSetPlnCart_B(pset)
        if not success:
            raise RuntimeError("setPln_Cart failed")
        return success

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
        :param com: 信息来源， 1:CAN端; 2：com1; 3:com2
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
        :param com: 信息来源， 1:CAN端; 2：com1; 3:com2
        :return: int, 长度size
        '''
        try:
            # 创建 256 字节缓冲区
            data_buffer = (ctypes.c_ubyte * 256)()
            ret_ch = ctypes.c_long(com)
            if arm == 'A':
                result = self.robot.OnGetChDataA(data_buffer, ctypes.byref(ret_ch))
                byte_data = bytes(data_buffer)  # 或 bytearray(data_buffer)
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

    def get_tool_info(self,):
        '''检查控制器是否已经保存工具信息
        :return:
           m,mcp,i
        '''

        '''tool '''
        local_path='tool_dyn_kine.txt'
        remote_path='/home/fusion/tool_dyn_kine.txt'
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        self.robot.OnRecvFile(local_char, remote_char)
        time.sleep(1)
        tool_result = read_csv_file_to_float_strict(local_path, expected_columns=16)
        return tool_result

    def help(self, method_name: str = None) -> None:
        """
        显示帮助信息

        参数:
            method_name (str): 可选的方法名，显示特定方法的帮助信息
        """
        print(f"\n{' API 帮助 ':=^50}\n")
        methods = [
            (name, func)
            for name, func in inspect.getmembers(self, inspect.ismethod)
            if not name.startswith('_') and name != 'help'
        ]
        if method_name is None:
            print("可用方法:")
            for name, func in methods:
                signature = inspect.signature(func)
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
        method_dict = dict(methods)
        if method_name in method_dict:
            func = method_dict[method_name]
            doc = inspect.getdoc(func) or "没有文档说明"
            signature = inspect.signature(func)
            print(f"方法: {method_name}{signature}")
            print("\n" + dedent(doc))
            print("\n参数详情:")
            for param in signature.parameters.values():
                param_info = f"  {param.name}: "
                if param.annotation is not param.empty:
                    param_info += f"类型: {param.annotation.__name__}, "
                if param.default is not param.empty:
                    param_info += f"默认值: {param.default!r}"
                print(param_info)
        else:
            print(f"错误: 没有找到方法 '{method_name}'")

        print(f"{'=' * 50}")

'''######################################## Concise SDK ########################################################'''
class Concise_Marvin_Robot:
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

        # Connect
        self.robot.Connect.argtypes = [
            ctypes.c_ubyte,  # ip1
            ctypes.c_ubyte,  # ip2
            ctypes.c_ubyte,  # ip3
            ctypes.c_ubyte,  # ip4
            ctypes.c_int  # log_switch
        ]
        self.robot.Connect.restype = ctypes.c_bool

        # OnGetBuf
        self.robot.OnGetBuf.argtypes = [ctypes.POINTER(DCSS)]
        self.robot.OnGetBuf.restype = ctypes.c_bool

        # ServoReset
        self.robot.ServoReset.argtypes = [ctypes.c_char, ctypes.c_int]
        self.robot.ServoReset.restype = None

        # EStop
        self.robot.EStop.argtypes = [ctypes.c_char_p]
        self.robot.EStop.restype = None

        # StartCollectData
        self.robot.StartCollectData.argtypes = [
            ctypes.c_long,  # targetNum
            ctypes.POINTER(ctypes.c_long),  # targetID (数组)
            ctypes.c_long  # recordNum
        ]
        self.robot.StartCollectData.restype = ctypes.c_bool

        # StopCollectData
        self.robot.StopCollectData.argtypes = []  # 无参数
        self.robot.StopCollectData.restype = ctypes.c_bool

        # OnSaveGatherData
        self.robot.OnSaveGatherData.argtypes = [ctypes.c_char_p]  # path
        self.robot.OnSaveGatherData.restype = ctypes.c_bool

        # SetJointMode
        self.robot.SetJointMode.argtypes = [ctypes.c_char, ctypes.c_int, ctypes.c_int]
        self.robot.SetJointMode.restype = ctypes.c_bool

        # SetImpJointMode
        self.robot.SetImpJointMode.argtypes = [
            ctypes.c_char,  # arm
            ctypes.c_int,  # velRatio
            ctypes.c_int,  # AccRatio
            ctypes.POINTER(ctypes.c_double),  # K[7]
            ctypes.POINTER(ctypes.c_double)  # D[7]
        ]
        self.robot.SetImpJointMode.restype = ctypes.c_bool

        # SetImpCartMode
        self.robot.SetImpCartMode.argtypes = [
            ctypes.c_char,  # arm
            ctypes.c_int,  # velRatio
            ctypes.c_int,  # AccRatio
            ctypes.POINTER(ctypes.c_double),  # K[7]
            ctypes.POINTER(ctypes.c_double),  # D[7]
            ctypes.c_int,  # fcType
            ctypes.POINTER(ctypes.c_double)
        ]
        self.robot.SetImpCartMode.restype = ctypes.c_bool


        # SetImpForceMode
        self.robot.SetImpForceMode.argtypes = [
            ctypes.c_char,  # arm
            ctypes.POINTER(ctypes.c_double),  # fxDir[6]
            ctypes.c_double  # fcAdjLmt
        ]
        self.robot.SetImpForceMode.restype = ctypes.c_bool

        # SetForceCmd
        self.robot.SetForceCmd.argtypes = [
            ctypes.c_char,  # arm
            ctypes.c_double  # force
        ]
        self.robot.SetForceCmd.restype = ctypes.c_bool

        # SetJointPostionCmd
        self.robot.SetJointPostionCmd.argtypes = [
            ctypes.c_char,  # arm
            ctypes.POINTER(ctypes.c_double)  # joint[7]
        ]
        self.robot.SetJointPostionCmd.restype = ctypes.c_bool

        # PlnInit
        self.robot.PlnInit.argtypes = [ctypes.c_char_p]  # path
        self.robot.PlnInit.restype = ctypes.c_bool

        # RunPlnJoint
        self.robot.RunPlnJoint.argtypes = [
            ctypes.c_char,  # arm
            ctypes.POINTER(ctypes.c_double),  # start_joints[7]
            ctypes.POINTER(ctypes.c_double),  # stop_joints[7]
            ctypes.c_double,  # vel_ratio
            ctypes.c_double  # acc_ratio
        ]
        self.robot.RunPlnJoint.restype = ctypes.c_bool

        # RunPlnCart
        self.robot.RunPlnCart.argtypes = [
            ctypes.c_char,  # arm
            ctypes.c_void_p  # pset (void*)
        ]
        self.robot.RunPlnCart.restype = ctypes.c_bool

        # StopPln
        self.robot.StopPln.argtypes = [ctypes.c_char]
        self.robot.StopPln.restype = ctypes.c_bool

        # SendPVT
        self.robot.SendPVT.argtypes = [
            ctypes.c_char,  # arm
            ctypes.c_char_p,  # local_file
            ctypes.c_long  # serial
        ]
        self.robot.SendPVT.restype = ctypes.c_bool

        # RunPVT
        self.robot.RunPVT.argtypes = [
            ctypes.c_char,  # arm
            ctypes.c_int  # id
        ]
        self.robot.RunPVT.restype = ctypes.c_bool

        # SetJointDrag
        self.robot.SetJointDrag.argtypes = [ctypes.c_char]
        self.robot.SetJointDrag.restype = ctypes.c_bool

        # SetCartDrag
        self.robot.SetCartDrag.argtypes = [ctypes.c_char, ctypes.c_char]
        self.robot.SetCartDrag.restype = ctypes.c_bool

        # ExitDrag
        self.robot.ExitDrag.argtypes = [ctypes.c_char]
        self.robot.ExitDrag.restype = ctypes.c_bool

        # ClearChData
        self.robot.ClearChData.argtypes = [ctypes.c_char]
        self.robot.ClearChData.restype = ctypes.c_bool

        # GetChData
        self.robot.GetChData.argtypes = [
            ctypes.c_char,  # arm
            ctypes.POINTER(ctypes.c_ubyte),  # data_ptr[256]
            ctypes.POINTER(ctypes.c_long)  # ret_ch (用于返回通道)
        ]
        self.robot.GetChData.restype = ctypes.c_long  # 返回实际接收数据长度

        # SetChData
        self.robot.SetChData.argtypes = [
            ctypes.c_char,  # arm
            ctypes.POINTER(ctypes.c_ubyte),  # data_ptr[256]
            ctypes.c_long,  # size_int
            ctypes.c_long  # set_ch
        ]
        self.robot.SetChData.restype = ctypes.c_long  # 返回发送数据长度

        # SetTool
        self.robot.SetTool.argtypes = [
            ctypes.c_char,  # arm
            ctypes.POINTER(ctypes.c_double),  # kinePara[6]
            ctypes.POINTER(ctypes.c_double)  # dynPara[10]
        ]
        self.robot.SetTool.restype = ctypes.c_bool

        # Disable
        self.robot.Disable.argtypes = [ctypes.c_char]  # arm
        self.robot.Disable.restype = ctypes.c_bool

    def _convert_ip(self, ip_str: str):
        """将IP字符串转换为四个整数的元组"""
        parts = ip_str.split('.')
        if len(parts) != 4:
            raise ValueError("IP address must have four octets")
        return tuple(int(p) for p in parts)

    def connect(self, robot_ip: str, log_switch: int = 0) -> bool:
        '''连接机器人
        :param robot_ip: 机器人IP地址，确保网线连接可以 ping 通。
        :param log_switch: 日志开关，0 关闭，1 开启（默认 0）
        :return: bool 连接成功返回 True，失败返回 False
        '''
        try:
            ip1, ip2, ip3, ip4 = self._convert_ip(robot_ip)
            result = self.robot.Connect(ip1, ip2, ip3, ip4, log_switch)
            return bool(result)
        except Exception as e:
            print(f"ConnectAndCkeck failed: {e}")
            return False

    def subscribe(self, dcss) -> dict | None:
        '''订阅机器人状态数据
        :param dcss: 结构体实例（由外部传入，会被填充）
        :return: 成功返回转换后的嵌套字典，失败返回 None
        '''
        if dcss is None:
            print("[ERROR] subscribe: dcss is None")
            return None
        try:
            success = self.robot.OnGetBuf(ctypes.byref(dcss))
            if not success:
                print("[ERROR] subscribe: OnGetBuf returned False")
                return None
            result = structure2dict(dcss)
            return result
        except Exception as e:
            print(f"[ERROR] subscribe failed: {e}")
            return None

    def check_error_and_clear(self):
        '''检查是否有机械臂错误和伺服错，有错则清错 ，一般用于连接机器人时检查使用，以及切换状态不成功时使用'''
        if not self.robot.CheckArmError() and not self.robot.CheckServoError():
            return False
        else:
            return False

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
        return self.robot.OnUpdateSystem(sdk_char)

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
            para=get_float_params('float','R.A0.CTRL.CartJNTDampJ1')

            #整数类型参数获取：
            我想获取[R.A0.BASIC]这个参数组里Type的值
            para=get_int_params('int','R.A0.BASIC.Type')
        '''
        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # 显式添加终止符
            if type=='float':
                result = ctypes.c_double(0)
                self.robot.OnGetFloatPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetFloatPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, float parameters={result.value}")
                return re_flag,result.value
            elif type=='int':
                result = ctypes.c_int(0)
                self.robot.OnGetIntPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetIntPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, int parameters={result.value}")
                return re_flag, result.value
        except Exception as e:
            print("ERROR:",e)

    def save_para_file(self):
        '''保存配置文件
        :return:
        '''
        self.robot.OnSavePara.restype = ctypes.c_long
        return self.robot.OnSavePara()

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
                self.robot.OnSetFloatPara.restype = ctypes.c_long
                return self.robot.OnSetFloatPara(param_buf, result)
            elif type=='int':
                result = ctypes.c_int(int(value))
                self.robot.OnSetIntPara.restype = ctypes.c_long
                return self.robot.OnSetIntPara(param_buf, result)
        except Exception as e:
            print("ERROR:",e)

    def start_collect_data(self, target_num: int, target_id: list, record_num: int) -> bool:
        """设置保存参数并开始采集数据，频率：1K hz

       :param target_num: 要采集的轴数量（0-35）
       :param target_id: 采集数据ID序号，
       :param record_num: 采集的数据点数最少1000行(1秒数据)，最大100万行（100秒数据）
       :return: bool 成功返回 True，失败返回 False

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
                       robot.start_collect_data(target_num=cols,target_id=idx,record_num=rows)

                   eg2: 采集左臂第二关节的速度和电流一共2列， 采集500行：
                       cols=2
                       idx=[11,31,0,0,0,0,0,
                            0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0]
                       rows=500
                       robot.start_collect_data(target_num=cols,target_id=idx,record_num=rows)
       """
        if target_num<0:
            target_num=1
        if target_num>35:
            target_num=35
        if record_num<1000:
            record_num=1000
        if record_num>1000000:
            record_num=1000000
        id_array = (ctypes.c_long * 35)(*target_id)
        try:
            result = self.robot.StartCollectData(target_num, id_array, record_num)
            return bool(result)
        except Exception as e:
            print(f"StartCollectData failed: {e}")
            return False

    def stop_collect_data(self) -> bool:
        """停止数据采集
        :return: bool 成功返回 True，失败返回 False
        """
        try:
            result = self.robot.StopCollectData()
            return bool(result)
        except Exception as e:
            print(f"StopCollectData failed: {e}")
            return False

    def save_gather_data(self, path: str) -> bool:
        """保存采集的数据
        :param path: 保存文件的路径（字符串）
        :return: bool 成功返回 True，失败返回 False
        """
        if not path:
            raise ValueError("path cannot be empty")
        try:
            path_bytes = path.encode('utf-8')
            result = self.robot.OnSaveGatherData(path_bytes)
            return bool(result)
        except Exception as e:
            print(f"OnSaveGatherData failed: {e}")
            return False

    def save_gather_data_as_csv_to_path(self,path:str) -> bool:
        '''以csv格式将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''
        path1='tmp.txt'
        self.save_data_path = path1.encode('utf-8')
        path_char = ctypes.c_char_p(self.save_data_path)
        self.robot.OnSaveGatherData(path_char)

        time.sleep(0.2)
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

    def soft_stop(self, arm: str):
        '''机械臂急停
        :param arm: ‘A’, 'B', 'AB', 可以让一条臂软急停，或者两条臂都软急停。
        :return: None
        '''
        try:
            arm_bytes = arm.encode('utf-8')
            self.robot.EStop(arm_bytes)
        except Exception as e:
            print("ERROR:", e)

    def servo_reset(self, arm: str, axis: int):
        """指定轴伺服软复位
        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param axis: 指定关节 0-6
        :return: None
        """
        if len(arm) != 1:
            raise ValueError("arm must be a single character, got '{}'".format(arm))
        if not isinstance(axis, int) or axis < 0 or axis > 6:
            raise ValueError("axis must be an integer between 0 and 6")
        arm_byte = arm.encode('ascii')
        try:
            self.robot.ServoReset(arm_byte, axis)
        except Exception as e:
            print("ServoReset failed:", e)

    def get_servo_error_code(self, arm:str,lang='CN'):
       '''获取机械臂伺服错误码
       :param self:
       :param arm: 机械手臂ID “A” OR “B”
       :param lang: 'CN' or 'EN'
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
               err_code=get_fault_descriptions(err_code,lang)
               return err_code
           elif arm=='B':
               self.robot.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_B(ctypes.byref(err_code_value))
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               err_code = get_fault_descriptions(err_code,lang)
               return err_code

       except Exception as e:
           print("ERROR:", e)

    def set_position_state(self, arm: str, velRatio: int, AccRatio: int) -> bool:
        """设置关节模式的速度和加速度百分比

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param velRatio: 速度百分比, 范围 0~100
        :param AccRatio: 加速度百分比, 范围 0~100
        :return: bool 设置成功返回 True，失败返回 False
        """
        # 参数校验
        if len(arm) != 1:
            raise ValueError(f"arm must be a single character, got '{arm}'")
        if velRatio < 0:
            velRatio = 10
        elif velRatio > 100:
            velRatio = 100
        if AccRatio < 0:
            AccRatio = 10
        elif AccRatio > 100:
            AccRatio = 100
        arm_byte = arm.encode('ascii')
        try:
            result = self.robot.SetJointMode(arm_byte, velRatio, AccRatio)
            return bool(result)
        except Exception as e:
            print(f"SetJointMode failed: {e}")
            return False

    def set_imp_joint_state(self, arm: str, velRatio: int, AccRatio: int, K, D) -> bool:
        """设置阻抗关节模式参数

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param velRatio: 速度百分比, 范围 0~100（超出会自动钳位）
        :param AccRatio: 加速度百分比, 范围 0~100（超出会自动钳位）
        :param K: 刚度系数列表/元组，长度必须为 7，值不能为负（负数会设为0）
        :param D: 阻尼系数列表/元组，长度必须为 7，值范围 0~1（超出会自动钳位）
        :return: bool 设置成功返回 True，失败返回 False
        """
        if velRatio < 0:
            velRatio = 10
        elif velRatio > 100:
            velRatio = 100
        if AccRatio < 0:
            AccRatio = 10
        elif AccRatio > 100:
            AccRatio = 100
        if len(K) != 7 or len(D) != 7:
            raise ValueError("K and D must have exactly 7 elements")
        K_clamped = []
        for val in K:
            if val < 0:
                val = 0.0
            K_clamped.append(float(val))
        D_clamped = []
        for val in D:
            if val < 0:
                val = 0.0
            elif val > 1:
                val = 1.0
            D_clamped.append(float(val))
        arm_byte = arm.encode('ascii')
        k_array = (ctypes.c_double * 7)(*K_clamped)
        d_array = (ctypes.c_double * 7)(*D_clamped)
        try:
            result = self.robot.SetImpJointMode(arm_byte, velRatio, AccRatio, k_array, d_array)
            return bool(result)
        except Exception as e:
            print(f"SetImpJointMode failed: {e}")
            return False

    def set_imp_cart_state(self, arm: str, velRatio: int, AccRatio: int, K, D, rot_type:int, cart_ctrl_para) -> bool:
        """设置指定手臂的速度、加速度和笛卡尔阻抗模式，并选择是否设置末端笛卡尔方向的旋转

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param velRatio: 速度百分比, 范围 0~100（超出自动钳位）
        :param AccRatio: 加速度百分比, 范围 0~100（超出自动钳位）
        :param K: 刚度系数列表/元组，长度必须为 7，值不能为负（负数设为0）
        :param D: 阻尼系数列表/元组，长度必须为 7，值范围 0~1（超出自动钳位）
        :param rot_type: 旋转模式。  0 不定义末端旋转， 1 用户自定义方向，2 系统自动计算
        :param cart_ctrl_para: 笛卡尔参数列表/元组，长度必须为 7（fcType=1 时前三个值为末端的旋转方向，fcType=2 时应全0）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        velRatio = max(0, min(velRatio, 100))
        AccRatio = max(0, min(AccRatio, 100))
        if len(K) != 7 or len(D) != 7:
            raise ValueError("K and D must have exactly 7 elements")

        if rot_type not in (0,1, 2):
            raise ValueError(f"rot_type must be 1 or 2, got {rot_type}")
        if len(cart_ctrl_para) != 7:
            raise ValueError("cart_ctrl_para must have exactly 7 elements")
        K_clamped = [max(0.0, float(v)) for v in K]
        D_clamped = [max(0.0, min(1.0, float(v))) for v in D]
        arm_byte = arm.encode('ascii')
        k_array = (ctypes.c_double * 7)(*K_clamped)
        d_array = (ctypes.c_double * 7)(*D_clamped)
        para_array = (ctypes.c_double * 7)(*cart_ctrl_para)
        try:
            result = self.robot.SetImpCartMode(arm_byte, velRatio, AccRatio, k_array, d_array,rot_type,para_array)
            return bool(result)
        except Exception as e:
            print(f"SetImpCartMode failed: {e}")
            return False

    def set_imp_force_state(self, arm: str, fx_dir, fc_adj_lmt: float) -> bool:
        """设置指定手臂的力控参数和力阻抗模式

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param fx_dir: 力方向向量，列表/元组，长度必须为 6
        :param fc_adj_lmt: 力的调节范围，单位毫米（应 >= 0）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if len(fx_dir) != 6:
            raise ValueError("fx_dir must have exactly 6 elements")
        if fc_adj_lmt < 0:
            raise ValueError(f"fc_adj_lmt must be >= 0, got {fc_adj_lmt}")
        arm_byte = arm.encode('ascii')
        fx_array = (ctypes.c_double * 6)(*fx_dir)
        try:
            result = self.robot.SetImpForceMode(arm_byte, fx_array, ctypes.c_double(fc_adj_lmt))
            return bool(result)
        except Exception as e:
            print(f"SetImpForceMode failed: {e}")
            return False

    def set_force_cmd(self, arm: str, force: float) -> bool:
        """设置指定手臂的力值

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param force: 力，单位：牛（可以是任意实数）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        arm_byte = arm.encode('ascii')
        try:
            result = self.robot.SetForceCmd(arm_byte, ctypes.c_double(force))
            return bool(result)
        except Exception as e:
            print(f"SetForceCmd failed: {e}")
            return False

    def set_joint_position_cmd(self, arm: str, joint) -> bool:
        """设置指定手臂的关节空间位置指令（位置模式扭矩模式下的关节指令）

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param joint: 七个关节的目标角度列表/元组，单位：度，长度必须为 7
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if len(joint) != 7:
            raise ValueError("joint must have exactly 7 elements")
        arm_byte = arm.encode('ascii')
        joint_array = (ctypes.c_double * 7)(*joint)
        try:
            result = self.robot.SetJointPostionCmd(arm_byte, joint_array)
            return bool(result)
        except Exception as e:
            print(f"SetJointPostionCmd failed: {e}")
            return False

    def pln_init(self, path: str) -> bool:
        """关节空间规划初始化（只需初始化一次）

        :param path: 规划文件路径（相对或绝对）
        :return: bool 成功返回 True，失败返回 False
        """
        if not path:
            raise ValueError("path cannot be empty")
        try:
            result = self.robot.PlnInit(path.encode('utf-8'))
            return bool(result)
        except Exception as e:
            print(f"PlnInit failed: {e}")
            return False

    def run_pln_joint(self, arm: str, start_joints, stop_joints, vel_ratio: float, acc_ratio: float) -> bool:
        """关节空间下从当前点规划方式运行到目标点

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param start_joints: 起始关节角度（7个，单位度）
        :param stop_joints: 目标关节角度（7个，单位度）
        :param vel_ratio: 速度比例（0~1 或百分比）
        :param acc_ratio: 加速度比例（0~1 或百分比）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if len(start_joints) != 7 or len(stop_joints) != 7:
            raise ValueError("start_joints and stop_joints must have exactly 7 elements")
        vel_ratio = max(0.0, min(1.0, vel_ratio))
        acc_ratio = max(0.0, min(1.0, acc_ratio))
        arm_byte = arm.encode('ascii')
        start_array = (ctypes.c_double * 7)(*start_joints)
        stop_array = (ctypes.c_double * 7)(*stop_joints)

        try:
            result = self.robot.RunPlnJoint(arm_byte, start_array, stop_array, vel_ratio, acc_ratio)
            return bool(result)
        except Exception as e:
            print(f"RunPlnJoint failed: {e}")
            return False

    def run_pln_cart(self, arm: str, pset) -> bool:
        """笛卡尔空间下从当前点规划方式运行到目标点（规划点位 pset 由 KinematicsSDK 计算得出）

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param pset: 规划点位数据（不透明指针，例如由结构体指针传入）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        arm_byte = arm.encode('ascii')
        try:
            if hasattr(pset, '_address_'):
                pset_ptr = ctypes.c_void_p(ctypes.addressof(pset))
            else:
                pset_ptr = ctypes.c_void_p(pset)
            result = self.robot.RunPlnCart(arm_byte, pset_ptr)
            return bool(result)
        except Exception as e:
            print(f"RunPlnCart failed: {e}")
            return False

    def stop_pln(self, arm: str) -> bool:
        """中断规划运行（笛卡尔空间和关节空间都适用）

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        try:
            result = self.robot.StopPln(arm.encode('ascii'))
            return bool(result)
        except Exception as e:
            print(f"StopPln failed: {e}")
            return False

    def send_pvt(self, arm: str, local_file: str, serial: int) -> bool:
        """上传本地 PVT 轨迹文件存为指定 ID

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param local_file: 轨迹文件路径（相对或绝对）
        :param serial: 轨迹 ID（0~99）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if not (0 <= serial <= 99):
            raise ValueError(f"serial must be between 0 and 99, got {serial}")
        if not local_file:
            raise ValueError("local_file cannot be empty")

        try:
            result = self.robot.SendPVT(arm.encode('ascii'), local_file.encode('utf-8'), serial)
            return bool(result)
        except Exception as e:
            print(f"SendPVT failed: {e}")
            return False

    def run_pvt(self, arm: str, id_: int) -> bool:
        """设置指定手臂的 PVT 号并立即运行该轨迹

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param id_: 轨迹 ID（0~99），与 SendPVT 的 serial 对应
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if not (0 <= id_ <= 99):
            raise ValueError(f"id must be between 0 and 99, got {id_}")
        try:
            result = self.robot.RunPVT(arm.encode('ascii'), id_)
            return bool(result)
        except Exception as e:
            print(f"RunPVT failed: {e}")
            return False

    def set_joint_drag(self, arm: str) -> bool:
        """设置指定手臂为关节拖动

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        try:
            result = self.robot.SetJointDrag(arm.encode('ascii'))
            return bool(result)
        except Exception as e:
            print(f"SetJointDrag failed: {e}")
            return False

    def set_cart_drag(self, arm: str, type_: str) -> bool:
        """设置指定手臂为笛卡尔拖动

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param type_: 拖动方向："X" "Y" "Z" "R" 之一
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if type_ not in ('X', 'Y', 'Z', 'R'):
            raise ValueError(f"type must be 'X', 'Y', 'Z' or 'R', got '{type_}'")
        try:
            result = self.robot.SetCartDrag(arm.encode('ascii'), type_.encode('ascii'))
            return bool(result)
        except Exception as e:
            print(f"SetCartDrag failed: {e}")
            return False

    def exit_drag(self, arm: str) -> bool:
        """设置指定手臂退出拖动

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        try:
            result = self.robot.ExitDrag(arm.encode('ascii'))
            return bool(result)
        except Exception as e:
            print(f"ExitDrag failed: {e}")
            return False

    def set_tool(self, arm: str, kine_para, dyn_para) -> bool:
        """设置指定手臂的工具参数（运动学和动力学）

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param kine_para: 工具相对于末端法兰的位置偏移（毫米）和姿态旋转（角度，XYZ顺序），长度必须为 6
        :param dyn_para: 工具动力学参数，长度必须为 10（由上位机软件计算）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if len(kine_para) != 6:
            raise ValueError("kine_para must have exactly 6 elements")
        if len(dyn_para) != 10:
            raise ValueError("dyn_para must have exactly 10 elements")
        arm_byte = arm.encode('ascii')
        kine_array = (ctypes.c_double * 6)(*kine_para)
        dyn_array = (ctypes.c_double * 10)(*dyn_para)
        try:
            result = self.robot.SetTool(arm_byte, kine_array, dyn_array)
            return bool(result)
        except Exception as e:
            print(f"SetTool failed: {e}")
            return False

    def disable(self, arm: str) -> bool:
        """设置指定手臂下使能/复位

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        try:
            result = self.robot.Disable(arm.encode('ascii'))
            return bool(result)
        except Exception as e:
            print(f"Disable failed: {e}")
            return False

    def clear_ch_data(self, arm: str) -> bool:
        """清缓存数据（手臂末端安装工具的通讯）

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :return: bool 成功返回 True，失败返回 False
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        try:
            result = self.robot.ClearChData(arm.encode('ascii'))
            return bool(result)
        except Exception as e:
            print(f"ClearChData failed: {e}")
            return False

    def get_ch_data(self, arm: str, channel:int) -> tuple[int, bytes, int]:
        """获取指定手臂指定通道的数据

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param channel: 通道号（1: CAN/CANFD, 2: COM1, 3: COM2）
        :return: (实际读取数据长度, 数据字节数组)
                 若失败返回 (0, b'')
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        data_buffer = (ctypes.c_ubyte * 256)()
        ret_ch = ctypes.c_long(channel)
        try:
            read_len = self.robot.GetChData(arm.encode('ascii'), data_buffer, ctypes.byref(ret_ch))
            if read_len <= 0:
                return 0, b''
            byte_data = bytes(data_buffer)[:read_len]
            hex_list = []
            for byte in byte_data:
                hex_value = hex(byte)[2:].upper().zfill(2)
                hex_list.append(hex_value)
            return read_len, ' '.join(hex_list)
        except Exception as e:
            print(f"GetChData failed: {e}")
            return 0, b''

    def set_ch_data(self, arm: str, data: bytes, size_int: int, set_ch: int) -> int:
        """给指定手臂指定通道发送数据

        :param arm: 机械手臂ID "A" 或 "B"（单字符）
        :param data: 要发送的数据字节串
        :param size_int: 数据长度（不应超过 256）
        :param set_ch: 通道号（1: CAN/CANFD, 2: COM1, 3: COM2）
        :return: 实际发送的数据长度（失败返回 -1）
        """
        if len(arm) != 1 or arm not in ('A', 'B'):
            raise ValueError(f"arm must be 'A' or 'B', got '{arm}'")
        if size_int > 256 or size_int < 0:
            raise ValueError(f"size_int must be between 0 and 256, got {size_int}")
        if set_ch not in (1, 2, 3):
            raise ValueError(f"set_ch must be 1, 2 or 3, got {set_ch}")
        data_buffer = (ctypes.c_ubyte * 256)()
        for i in range(min(size_int, len(data))):
            data_buffer[i] = data[i]
        try:
            sent_len = self.robot.SetChData(arm.encode('ascii'), data_buffer, size_int, set_ch)
            return sent_len
        except Exception as e:
            print(f"SetChData failed: {e}")
            return -1

    def get_tool_info(self,):
        '''检查控制器是否已经保存工具信息
        :return:
           m,mcp,i
        '''

        '''tool '''
        local_path='tool_dyn_kine.txt'
        remote_path='/home/fusion/tool_dyn_kine.txt'
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        self.robot.OnRecvFile(local_char, remote_char)
        time.sleep(1)
        tool_result = read_csv_file_to_float_strict(local_path, expected_columns=16)
        return tool_result

    def help(self, method_name: str = None) -> None:
        """
        显示帮助信息

        参数:
            method_name (str): 可选的方法名，显示特定方法的帮助信息
        """
        print(f"\n{' API 帮助 ':=^50}\n")
        methods = [
            (name, func)
            for name, func in inspect.getmembers(self, inspect.ismethod)
            if not name.startswith('_') and name != 'help'
        ]
        if method_name is None:
            print("可用方法:")
            for name, func in methods:
                signature = inspect.signature(func)
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
        method_dict = dict(methods)
        if method_name in method_dict:
            func = method_dict[method_name]
            doc = inspect.getdoc(func) or "没有文档说明"
            signature = inspect.signature(func)
            print(f"方法: {method_name}{signature}")
            print("\n" + dedent(doc))
            print("\n参数详情:")
            for param in signature.parameters.values():
                param_info = f"  {param.name}: "
                if param.annotation is not param.empty:
                    param_info += f"类型: {param.annotation.__name__}, "
                if param.default is not param.empty:
                    param_info += f"默认值: {param.default!r}"
                print(param_info)
        else:
            print(f"错误: 没有找到方法 '{method_name}'")
        print(f"{'=' * 50}")

'''######################################## Assistive Funcs ####################################################'''
class StateCtr(Structure):
    _fields_ = [
        ("m_CurState", c_int),  # * 当前状态 */ ArmState
        ("m_CmdState", c_int),  # * 指令状态 */ DCSSCmdType 0
        ("m_ERRCode", c_int)    # * 机械臂错误码*/
    ]

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

class RT_OUT(Structure):
    _fields_ = [
        ("m_OutFrameSerial", c_int),  # 输出帧序号   0 -  1000000 取模
        ("m_FB_Joint_Pos", c_float * 7),  # 关节位置反馈
        ("m_FB_Joint_Vel", c_float * 7),  # 关节速度反馈
        ("m_FB_Joint_PosE", c_float * 7),  # 关节位置(外编)
        ("m_FB_Joint_Cmd", c_float * 7),  # 位置关节指令
        ("m_FB_Joint_CToq", c_float * 7),  # 关节电流
        ("m_FB_Joint_SToq", c_float * 7),  # 关节扭矩
        ("m_FB_Joint_Them", c_float * 7),  # 关节温度
        ("m_EST_Joint_Firc", c_float * 7),  # 关节摩擦估计
        ("m_EST_Joint_Firc_Dot", c_float * 7),  # 关节力扰动估计值微分
        ("m_EST_Joint_Force", c_float * 7),  # 关节力扰动估计值
        ("m_EST_Cart_FN", c_float * 6),  # 末端笛卡尔空间力扰动估计值
        ("m_TipDI", c_char),  # 末端数字输入
        ("m_LowSpdFlag", c_char),  # 低速标志
        ("m_pad", c_char),  # 填充字节
        ("m_TrajState", c_char) #规划状态： 0: no traj; 1: receving; 2: recevied; >=3: running traj
    ]

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

def get_fault_descriptions(fault_codes_list,lang='CN'):
    """
    根据故障代码列表和故障字典，返回故障描述的字符串
    支持多个关节的错误信息显示
    Args:
        fault_codes_list: 各关节故障代码列表
        fault_dict: 故障代码与名称的字典

    Returns:
        包含所有关节故障描述的字符串
    """
    fault_dict=fault_code_dict_CN
    if lang=='EN':
        fault_dict=fault_code_dict_EN
    if not fault_codes_list:
        return "None fault codes list"
    all_descriptions = []
    for joint_idx, code in enumerate(fault_codes_list, 1):
        code_str = str(code)
        if isinstance(code, int):
            code_str = hex(code)
        code_str_lower = code_str.lower()
        if code_str in ["0x0000", "0000", "0", "0x0", "0X0", ""]:
            continue
        fault_name = None
        if code_str in fault_dict:
            fault_name = fault_dict[code_str]
        elif code_str_lower in fault_dict:
            fault_name = fault_dict[code_str_lower]
        else:
            fault_name = f"unknown error({code_str})"
        if fault_name:
            all_descriptions.append(f"joint {joint_idx}: {code_str} - {fault_name}")

    if not all_descriptions:
        return "None"
    elif len(all_descriptions) == 1:
        return all_descriptions[0]
    else:
        result = "Fault information for each joint:\n"
        for i, desc in enumerate(all_descriptions, 1):
            result += f"{i}. {desc}\n"
        return result.strip()

def update_text_file_simple(mode, data_list, filename):
    """
    简化版的文件更新函数
    """
    if mode not in ['A', 'B'] or len(data_list) != 16:
        return False
    try:
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                lines = file.readlines()
        line_index = 0 if mode == 'A' else 1
        lines[line_index] = ','.join(str(x) for x in data_list) + '\n'
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
        如果文件有一行: 返回0
        如果文件有两行且其中一行全为0:
            - 返回 ('line1', [第一行数据])  # 如果第二行全为0
            - 返回 ('line2', [第二行数据])  # 如果第一行全为0
        如果文件有两行且都不为0: 返回 [[第一行数据], [第二行数据]]
        如果文件有两行且都全为0: 返回0
        如果文件不存在或转换失败: 返回-1
    """
    if not os.path.exists(filename):
        print(f"文件不存在: {filename}")
        return -1
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
            cleaned_values = [v.strip() for v in values if v.strip()]
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

        if len(all_float_data) == 1:
            return 0

        elif len(all_float_data) == 2:
            line1_all_zero = all(x == 0.0 for x in all_float_data[0])
            line2_all_zero = all(x == 0.0 for x in all_float_data[1])
            if line1_all_zero and line2_all_zero:
                return 0
            elif line1_all_zero and not line2_all_zero:
                return ('line2', all_float_data[1])
            elif not line1_all_zero and line2_all_zero:
                return ('line1', all_float_data[0])
            else:
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
    if not isinstance(number, (int, float)):
        raise TypeError("输入必须是整数或浮点数")
    if isinstance(number, int):
        hex_str = hex(number)
    else:
        hex_str = float(number).hex()
        if float_precision is not None:
            parts = hex_str.split('.')
            if len(parts) > 1:
                exponent_part = parts[1].split('p')
                if len(exponent_part) > 1:
                    hex_str = f"{parts[0]}.{exponent_part[0][:float_precision]}p{exponent_part[1]}"
    if not prefix and hex_str.startswith('0x'):
        hex_str = hex_str[2:]
    elif prefix and not hex_str.startswith('0x'):
        hex_str = '0x' + hex_str
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

    if isinstance(input_data, bytes):
        result["type"] = "bytes"
        result["length_bytes"] = len(input_data)
        result["bytes_representation"] = input_data
        return result

    if isinstance(input_data, str):
        clean_input = re.sub(r'\s+', '', input_data.lower())

        if clean_input.startswith('0x'):
            clean_input = clean_input[2:]

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
                pass

        if input_data.startswith('b"') and input_data.endswith('"'):
            try:
                bytes_rep = eval(input_data)
                if isinstance(bytes_rep, bytes):
                    result["type"] = "bytes representation string"
                    result["length_bytes"] = len(bytes_rep)
                    result["bytes_representation"] = bytes_rep
                    return result
            except:
                pass
        try:
            bytes_rep = input_data.encode('utf-8')
            result["type"] = "regular string"
            result["length_bytes"] = len(bytes_rep)
            result["bytes_representation"] = bytes_rep
            return result
        except UnicodeEncodeError:
            raise ValueError("输入不是有效的十六进制字符串，也无法编码为UTF-8字节串")
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
            "pad": rt_out.m_pad,
            "traj_state":rt_out.m_TrajState,
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

arm_err_code={
    '1':"总线拓扑异常",
    '2':"伺服故障",
    '3':"PVT异常",
    '4':"请求进位置失败",
    '5':"进位置失败",
    '6':"请求进扭矩失败",
    '7':"进扭矩失败",
    '8':"请求上伺服失败",
    '9':"上伺服失败",
    '10':"请求下伺服失败",
    '11':"下伺服失败",
    '12':"内部错",
    '13':"急停",
    '14':"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开"
}

arm_err_code_EN={
'1':"Bus topology error",
'2':"Servo failure",
'3':"PVT error",
'4':"Failed to request position advance",
'5':"Failed to advance position",
'6':"Failed to request torque advance",
'7':"Failed to advance torque",
'8':"Failed to request servo up",
'9':"Failed to request servo up",
'10':"Failed to request servo down",
'11':"Failed to request servo down",
'12':"Internal error",
'13':"Emergency stop",
'14':"Floating base option selected in configuration file, but UMI setting not enabled in configuration file"
}

fault_code_dict_CN = {
    "0x2280": "驱动器短路",
    "0x2310": "U相输出电流过大",
    "0x2311": "V相输出电流过大",
    "0x2320": "驱动器硬件过流",
    "0x2330": "驱动器输出对地短路",
    "0x3130": "主电源输入异常",
    "0x3210": "直流母线过压",
    "0x3220": "直流母线欠压",
    "0x4210": "功率模块过热",
    "0x6010": "CPU1 看门狗溢出",
    "0x6011": "CPU2 看门狗溢出",
    "0x7112": "能耗制动电阻过载",
    "0x8311": "电机持续过载",
    "0x8611": "位置跟随误差过大",
    "0x8612": "正向软限位",
    "0x8613": "负向软限位",
    "0x8800": "编码器数据溢出",
    "0x8801": "保留",
    "0xFF00": "CPU1 工作异常",
    "0xFF01": "CPU2 工作异常",
    "0xFF02": "CPU1 内存异常",
    "0xFF03": "CPU2 内存异常",
    "0xFF04": "CPU 内存冲突",
    "0xFF05": "磁极定位错误",
    "0xFF06": "编码器数据异常",
    "0xFF07": "编码器通信异常",
    "0xFF08": "编码器通信超时",
    "0xFF09": "编码器内部异常1",
    "0xFF10": "驱动器其它轴异常",
    "0xFF11": "电机抱闸断线",
    "0xFF12": "保留",
    "0xFF13": "保留",
    "0xFF14": "控制编码器超速",
    "0xFF15": "驱动器持续过载",
    "0xFF16": "保留",
    "0xFF17": "驱动器输出缺相",
    "0xFF18": "电机失速",
    "0xFF19": "协处理器通讯异常",
    "0xFF20": "编码器AB信号变化异常",
    "0xFF21": "电流跟随误差过大",
    "0xFF22": "位置目标值异常",
    "0xFF23": "编码器上电位置异常",
    "0xFF24": "位置目标值溢出",
    "0xFF25": "电机抱闸异常",
    "0xFF26": "控制电源欠压",
    "0xFF27": "STO1 触发",
    "0xFF28": "STO2 触发",
    "0xFF29": "正向硬限位开关触发",
    "0xFF30": "负向硬限位开关触发",
    "0xFF31": "电机超速",
    "0xFF32": "急停输入开关触发",
    "0xFF33": "转矩饱和检测故障",
    "0xFF34": "速度跟随误差过大",
    "0xFF35": "驱动器过流2",
    "0xFF36": "寻原点失效",
    "0xFF37": "EtherCAT过程数据错误",
    "0xFF38": "EtherCAT总线指令非法",
    "0xFF39": "EtherCAT通讯周期错误",
    "0xFF40": "位置规划运行错误",
    "0xFF41": "EtherCAT非法同步模式",
    "0xFF42": "位置目标值超出设定范围",
    "0xFF43": "整流模块过热",
    "0xFF44": "散热器过热",
    "0xFF45": "电机U相持续过载",
    "0xFF46": "电机V相持续过载",
    "0xFF48": "保留",
    "0xFF49": "驱动器内部异常",
    "0xFF50": "限位开关异常",
    "0xFF51": "EtherCAT总线通讯异常",
    "0xFF52": "接口编码器分辨率变更",
    "0xFF53": "编码器过热",
    "0xFF54": "编码器电池欠电压故障",
    "0xFF55": "保留",
    "0xFF56": "保留",
    "0xFF57": "控制模式设定错误",
    "0xFF58": "上电位置偏差过大",
    "0xFF59": "编码器加速度异常故障",
    "0xFF60": "电机堵转",
    "0xFF61": "电机过热",
    "0xFF62": "增量式编码器Z信号异常",
    "0xFF63": "写EPROM数据异常",
    "0xFF64": "读EPROM数据异常",
    "0xFF65": "控制机功率异常",
    "0xFF66": "拖曳使能异常",
    "0xFF67": "CPU过热",
    "0xFF68": "CPU1过载",
    "0xFF69": "CPU2过载",
    "0xFF70": "CPU1握手失效",
    "0xFF71": "DriveMaster通讯超时",
    "0xFF72": "保留",
    "0xFF73": "力矩传感器异常",
    "0xFF74": "保留",
    "0xFF75": "ESC配置EEPROM异常",
    "0xFF76": "ESC内部访问错误",
    "0xFF77": "伺服使能未准备好",
    "0xFF78": "CPU2握手失败",
    "0xFF79": "CPU1主任务超时",
    "0xFF80": "主电源掉电",
    "0xFF81": "直流母线充电继电器异常",
    "0xFF82": "CPU内部错误",
    "0xFF83": "位置实际值溢出",
    "0xFF84": "保留",
    "0xFF85": "编码器内部异常2",
    "0xFF86": "保留",
    "0xFF87": "编码器内部异常3",
    "0xFF88": "保留",
    "0xFF89": "保留",
    "0xFF8A": "STO1电路诊断异常",
    "0xFF8B": "STO2电路诊断异常",
    "0xFF8C": "霍尔信号异常",
    "0xFF8D": "编码器霍尔-AB信号欠相异常",
    "0xFF8E": "第2位置跟随误差过大",
    "0xFF8F": "STO接线异常",
    "0xFF90": "第2速度跟随误差过大",
    "0xFF91": "驱动器内部异常2",
}

fault_code_dict_EN = {
    "0x2280": "Drive short circuit",
    "0x2310": "U-phase output overcurrent",
    "0x2311": "V-phase output overcurrent",
    "0x2320": "Drive hardware overcurrent",
    "0x2330": "Drive output short to ground",
    "0x3130": "Main power input abnormal",
    "0x3210": "DC bus overvoltage",
    "0x3220": "DC bus undervoltage",
    "0x4210": "Power module overtemperature",
    "0x6010": "CPU1 watchdog timeout",
    "0x6011": "CPU2 watchdog timeout",
    "0x7112": "Dynamic braking resistor overload",
    "0x8311": "Motor continuous overload",
    "0x8611": "Position following error too large",
    "0x8612": "Positive software limit",
    "0x8613": "Negative software limit",
    "0x8800": "Encoder data overflow",
    "0x8801": "Reserved",
    "0xFF00": "CPU1 operation abnormal",
    "0xFF01": "CPU2 operation abnormal",
    "0xFF02": "CPU1 memory abnormal",
    "0xFF03": "CPU2 memory abnormal",
    "0xFF04": "CPU memory conflict",
    "0xFF05": "Magnetic pole positioning error",
    "0xFF06": "Encoder data abnormal",
    "0xFF07": "Encoder communication abnormal",
    "0xFF08": "Encoder communication timeout",
    "0xFF09": "Encoder internal error 1",
    "0xFF10": "Other drive axis abnormal",
    "0xFF11": "Motor brake disconnection",
    "0xFF12": "Reserved",
    "0xFF13": "Reserved",
    "0xFF14": "Control encoder overspeed",
    "0xFF15": "Drive continuous overload",
    "0xFF16": "Reserved",
    "0xFF17": "Drive output missing phase",
    "0xFF18": "Motor stall",
    "0xFF19": "Coprocessor communication abnormal",
    "0xFF20": "Encoder AB signal change abnormal",
    "0xFF21": "Current following error too large",
    "0xFF22": "Position target value abnormal",
    "0xFF23": "Encoder power-on position abnormal",
    "0xFF24": "Position target value overflow",
    "0xFF25": "Motor brake abnormal",
    "0xFF26": "Control power undervoltage",
    "0xFF27": "STO1 triggered",
    "0xFF28": "STO2 triggered",
    "0xFF29": "Positive hardware limit switch triggered",
    "0xFF30": "Negative hardware limit switch triggered",
    "0xFF31": "Motor overspeed",
    "0xFF32": "Emergency stop input switch triggered",
    "0xFF33": "Torque saturation detection fault",
    "0xFF34": "Speed following error too large",
    "0xFF35": "Drive overcurrent 2",
    "0xFF36": "Homing failed",
    "0xFF37": "EtherCAT process data error",
    "0xFF38": "EtherCAT bus command illegal",
    "0xFF39": "EtherCAT communication cycle error",
    "0xFF40": "Position planning operation error",
    "0xFF41": "EtherCAT illegal sync mode",
    "0xFF42": "Position target value out of range",
    "0xFF43": "Rectifier module overtemperature",
    "0xFF44": "Heatsink overtemperature",
    "0xFF45": "Motor U-phase continuous overload",
    "0xFF46": "Motor V-phase continuous overload",
    "0xFF48": "Reserved",
    "0xFF49": "Drive internal error",
    "0xFF50": "Limit switch abnormal",
    "0xFF51": "EtherCAT bus communication abnormal",
    "0xFF52": "Interface encoder resolution changed",
    "0xFF53": "Encoder overtemperature",
    "0xFF54": "Encoder battery undervoltage fault",
    "0xFF55": "Reserved",
    "0xFF56": "Reserved",
    "0xFF57": "Control mode setting error",
    "0xFF58": "Power-on position deviation too large",
    "0xFF59": "Encoder acceleration abnormal fault",
    "0xFF60": "Motor locked rotor",
    "0xFF61": "Motor overtemperature",
    "0xFF62": "Incremental encoder Z signal abnormal",
    "0xFF63": "Write EPROM data abnormal",
    "0xFF64": "Read EPROM data abnormal",
    "0xFF65": "Control/power module mismatch",
    "0xFF66": "Drag enable abnormal",
    "0xFF67": "CPU overtemperature",
    "0xFF68": "CPU1 overload",
    "0xFF69": "CPU2 overload",
    "0xFF70": "CPU1 handshake failure",
    "0xFF71": "DriveMaster communication timeout",
    "0xFF72": "Reserved",
    "0xFF73": "Torque sensor abnormal",
    "0xFF74": "Reserved",
    "0xFF75": "ESC configuration EEPROM abnormal",
    "0xFF76": "ESC internal access error",
    "0xFF77": "Servo enable not ready",
    "0xFF78": "CPU2 handshake failure",
    "0xFF79": "CPU1 main task timeout",
    "0xFF80": "Main power loss",
    "0xFF81": "DC bus charging relay abnormal",
    "0xFF82": "CPU internal error",
    "0xFF83": "Actual position value overflow",
    "0xFF84": "Reserved",
    "0xFF85": "Encoder internal error 2",
    "0xFF86": "Reserved",
    "0xFF87": "Encoder internal error 3",
    "0xFF88": "Reserved",
    "0xFF89": "Reserved",
    "0xFF8A": "STO1 circuit diagnosis abnormal",
    "0xFF8B": "STO2 circuit diagnosis abnormal",
    "0xFF8C": "Hall signal abnormal",
    "0xFF8D": "Encoder Hall-AB signal missing phase abnormal",
    "0xFF8E": "2nd position following error too large",
    "0xFF8F": "STO wiring abnormal",
    "0xFF90": "2nd speed following error too large",
    "0xFF91": "Drive internal error 2",
}

tools_cfg={
    "arm0": {
        "tool-1": {
            "dyn": [0,0,0,0,0,0,0,0,0,0],
            "kine": [0,0,0,0,0,0]
        },
        "tool-2": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-3": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-4": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-5": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        }
    },
    "arm1": {
        "tool-1": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-2": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-3": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-4": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        },
        "tool-5": {
            "dyn": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "kine": [0, 0, 0, 0, 0, 0]
        }
    },
    "current_tool":{
        "arm0":None,
        "arm1":None
    }
}

if __name__ == "__main__":

    tj_robot = Marvin_Robot()
    tj_robot.help()
    tj_robot.help('collect_data')

    dcss=DCSS()
    sub_data=tj_robot.subscribe(dcss)
    print(sub_data['states'][0]['cur_state'])

    robot_concise=Concise_Marvin_Robot()
    robot_concise.help()
