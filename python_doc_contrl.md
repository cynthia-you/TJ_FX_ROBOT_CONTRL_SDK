# 天机-孚晞 机器人工具包 SDK_PYTHON
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1003
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2026-03


# ATTENTION
    1.  请先熟练使用MARVIN_APP或MarvinPlatform软件。操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
    2.  DEMO_C++/ 和 DEMO_PYTHON/ 下为接口使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
        这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
            比如:
                a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。

# 一 、SDK_PYTHON简要介绍
    为天机双臂机器人和人形机器人基于python开发的SDK，其分为控制SDK：SDK_PYTHON/fx_robot.py 和运动学计算的SDK:SDK_PYTHON/fx_kine.py

## 1.1 SDK文档
    SDK的主文档为master分支下的主文档：readme.md

    机器人控制SDK文档：
        c++_doc_contrl.md
        python_doc_contrl.md

    机器人计算SDK文档：
        c++_doc_kine.md
        python_doc_kine.md
    
## 1.2 SDK库文件夹下文件说明
    SDK_PYTHON文件下文件为：
    TJ_FX_ROBOT_CONTRL_SDK-master
    |————DEMO_PYTHON  #SDK在python下的使用案例
    |————SDK_PYTHON
            |————fx_kine.py #计算接口
            |————fx_robot.py #控制接口
            |————libKine.dll #windows下计算库动态库
            |————libKine.so #linux下计算库动态库
            |————libMarvinSDK.dll #windows下控制库动态库
            |————libMarvinSDK.so #linux下控制库动态库

    注意：请检查SDK_PYTHON下动态库是否为最新编译

## 1.3 SDK库文件编译

    使用自动化编译脚本：
        master分支下marvinSDK_windows.bat运行可自动编译C++和python调用的dll文件
        master分支下marvinSDK_ubuntu.sh运行可自动编译C++和python调用的so文件


  手动编译指令：

    windows下使用MinGW编译dll动态库：
			控制SDK（contrlSDK）：g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
			运动学SDK(kinematicsSDK)：g++ *.cpp -Wall -w -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下python使用

    linux设备编译:
    控制SDK(contrlSDK)，以下方法均可编译: 
        1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN
        2./contrlSDK/makefile 生成libMarvinSDK.so
    运动学SDK(kinematicsSDK)，以下方法均可编译: 
        1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt 
        2./kinematicsSDK/makefile 生成libKine.so
	编译的libKine.so 和 libMarvinSDK.so 供编译机器下的下C++和python使用


## 二、 控制SDK功能接口介绍
    SDK_PYTHON/fx_robot.py是基于机器人（双臂系统）C++开发的SDK的的二次开发工具包

## 2.1 提供功能大类有：
    (1) 1KHz 通信
        下发指令和订阅机器人数据是1KHz 通信， 采用UDP通信。
    
    (2) 控制状态切换
    
        ① 下使能
        ② 位置跟随模式
        ③ 位置PVT模式
        ④ 扭矩模式
            1) 关节阻抗控制/关节阻抗控制位置跟随
            2) 坐标阻抗控制/坐标阻抗控制位置跟随
            3) 力控制/力控制位置跟随
        ⑤ 协作释放
    
    (3) 控制状态参数（1KHz）
    
        ① 参数
            1) 目标跟随速度加速度设定，（百分比，值范围0-100）
            2) 关节阻抗参数设定
            3) 坐标阻抗参数设定
            4) 力控制参数设定
            5) 工具运动学/动力学参数设定
        ② 指令
            1) 位置跟随目标指令 
            2) 力控目标指令 
    
    (4) 数据反馈和采集（1KHz）
    
        ① 实时反馈
            1) 位置
            2) 速度
            3) 外编位置
            4) 电流
            5) 传感器扭矩
            6) 摩檫力
            7) 轴外力
        ② 数据采集
            1) 针对实时反馈数据可选择多达35项数据进行实时采集。
    
    (5) 参数获取和设置
    
        ① 统一接口以参数名方式获取和设置所有参数。

## 2.2 控制机器人的主要逻辑

    机器人控制的主逻辑为:
        UDP连接机器人,通过接收数的更新据确认为有效连接
        |
        设置预期控制状态下对应的参数（速度，加速度，刚度，阻尼等），再设置控制状态
        |
        下发关节指令/力指令
        |
        ...
        |
        任务完成,释放机器人以便别的程序或者用户连接机器人


## 2.3 接口详解
    
    API接口说明
        首先将fx_robot的类函数实例化，然后调用指定函数
        tj_robot = Marvin_Robot() #实例化

### (1) 连接机器人
    connect(robot_ip: str)

    '''UDP连接机器人
        :param robot_ip: 器人IP地址,确保网线连接可以ping通。
        :return:
            int: 连接状态码 1: True; 0: Flase
        eg:
            connect(robot_ip='192.168.1.190')
    '''

### (2) 释放机器人连接
    release_robot()

    '''释放机器人连接,释放后，要获取机器人的控制，需再次连接
        :return:
            int: 断开状态码 1: True; 0: Flase
    '''

### (3) 系统及系统更新相关
    SDK_version()

    '''查看当前SDK版本
    :return:
        long: SDK version
    '''

    update_SDK(sdk_path: str):
        '''更新系统
        :param sdk_path: 本机绝对路径存放的控制器系统文件更新到控制柜上
        '''

    获取当前控制器版本：
    ret, version = robot.get_param('int', 'VERSION')

    获取系统日志，下载到本机绝对路径
    download_sdk_log(self, log_path:str):
        '''下载SDK日志到本机
        :param log_path: 日志下载到本机的绝对路
        '''

### (4) 日志开关
    全局日志开， 日志信息将全部打印，包括1000HZ频率日志以及清空待发送数据缓冲区日志信息
    log_switch(flag:str)
    flag:string, "0"关； "1"开

    主要日志开，打印显示主要指令接口信息
    local_log_switch(flag:str)
    flag:string, "0"关； "1"开

### (5) 急停、获取错误码和清错

    soft_stop(arm: str):
        '''机械臂急停
        :param arm: ‘A’, 'B', 'AB', 可以让一条臂软急停，或者两条臂都软急停。
        '''
    get_servo_error_code(arm:str,lang='CN'):
       '''获取机械臂伺服错误码
       :param self:
       :param arm: ‘A’, 'B'
       :param lang: 'CN' or 'EN'
       :return: (7,1)错误列表， 16进制
       '''

    clear_error(arm:str):
        '''清错
        :param arm: ‘A’, 'B'
        '''

### (6) 实时订阅机器人数据
    subscribe(dcss):
    '''订阅机器人数据
    :param dcss:  结构体，见fx_robot.py 内的结构体定义class DCSS(Structure)
    :return:
        嵌套字典
    '''

      返回嵌套字典的结构
      result {
                'para_name': ['Marvin_sub_data'], 
                'states': [{'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}],
                'outputs': [
                            {'frame_serial': 0, 
                            'pad':b'\x00',
                            'traj_state:b'\x00',
                             'tip_di': b'\x00',
                            'low_speed_flag': b'\x00', 
                             'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                             'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                             'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                             'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                             'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                            'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, 
                        {'frame_serial': 0, 'tip_di': b'\x00', 'low_speed_flag': b'\x00', 'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
                ], 
                'inputs': [
                        {'rt_in_switch': 0, 'imp_type': 0, 'in_frame_serial': 0, 'frame_miss_cnt': 0, 'max_frame_miss_cnt': 0, 'sys_cyc': 0, 'sys_cyc_miss_cnt': 0, 'max_sys_cyc_miss_cnt': 0, 'tool_kine': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'tool_dyn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_vel_ratio': 0, 'joint_acc_ratio': 0, 'joint_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'drag_sp_type': 0, 'drag_sp_para': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kd_type': 0, 'cart_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kn': 0.0, 'cart_dn': 0.0, 'force_fb_type': 0, 'force_type': 0, 'force_dir': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_pidul': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_adj_lmt': 0.0, 'force_cmd': 0.0, 'set_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'update_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'pvt_id': 0, 'pvt_id_update': 0, 'pvt_run_id': 0, 'pvt_run_state': 0}, 
                        {'rt_in_switch': 0, 'imp_type': 0, 'in_frame_serial': 0, 'frame_miss_cnt': 0, 'max_frame_miss_cnt': 0, 'sys_cyc': 0, 'sys_cyc_miss_cnt': 0, 'max_sys_cyc_miss_cnt': 0, 'tool_kine': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'tool_dyn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_vel_ratio': 0, 'joint_acc_ratio': 0, 'joint_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'drag_sp_type': 0, 'drag_sp_para': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kd_type': 0, 'cart_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kn': 0.0, 'cart_dn': 0.0, 'force_fb_type': 0, 'force_type': 0, 'force_dir': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_pidul': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_adj_lmt': 0.0, 'force_cmd': 0.0, 'set_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'update_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'pvt_id': 0, 'pvt_id_update': 0, 'pvt_run_id': 0, 'pvt_run_state': 0}], 
                'ParaName': [[]], 
                'ParaType': [0], 
                'ParaIns': [0], 
                'ParaValueI': [0], 
                'ParaValueF': [0.0], 
                'ParaCmdSerial': [0], 
                'ParaRetSerial': [0]
                }

        注意，返回字典包括双臂的数据，A索引0，B索引1 
        如 读取当前双臂臂的状态和历史关节命令以及获取当前关节角度demo：
            from fx_robot import Marvin_Robot
            from structure_data import DCSS
            import time
            dcss=DCSS()
            robot=Marvin_Robot()
            robot.connect('192.168.1.190')
            robot.log_switch('1') #全局日志开关
            robot.local_log_switch('1') # 主要日志
            time.sleep(1)
        
            sub_data=robot.subscribe(dcss)
        
            a_state=sub_data["states"][0]["cur_state"]
            b_state=sub_data["states"][1]["cur_state"]
        
            a_joints_cmd=sub_data["inputs"][0]["joint_cmd_pos"]
            b_joints_cmd=sub_data["inputs"][1]["joint_cmd_pos"]
        
            a_current_joints=sub_data["outputs"][0]["fb_joint_pos"]
            b_current_joints=sub_data["outputs"][1]["fb_joint_pos"]


### (7) 配置机器人参数相关(参数名见robot.ini文件)
    get_param(type:str,paraName:str):
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

    set_param(type:str,paraName:str,value:float):
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

    save_para_file():
        '''保存配置文件
        :return:
        '''

### (8) 数据采集和保存相关
    collect_data(targetNum:int,targetID:list[int],recordNum:int):
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

    save_collected_data_to_path(path:str):
        '''将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''


    stop_collect_data(self):
        '''停止采集数据
        注： 在行数采集满后会自动停止采集,若需要中途停止采集调用本函数并等待1ms之后会停止采集。
        :return:
            int: 1: True; 0: Flase
        '''

    save_collected_data_as_csv_to_path(path:str):
        '''以csv格式将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''

### (9) 末端通信模组指令收发相关
    注意，之前无指令发送到末端模组，读取返回为0,使用逻辑为： 清缓存---发数据---读数据  或者： 清缓存---读数据---发数据---读数据
    建议连接机器人后，先清缓存，开启一个线程先读数据，再开另一个线程读数据。

    clear_485_cache(arm:str):
        '''清空发送缓存
        :param arm: 机械手臂ID “A” OR “B”
        :return: bool
        '''

    get_485_data(arm: str,com:int):
        '''收指定来源的末端数据
        :param arm: 机械手臂ID “A” OR “B”
        :param com: 信息来源， 1:CAN端; 2：com1; 3:com2
        :return: int, 长度size
        '''

    set_485_data(self, arm: str, data:bytes, size_int:int,com:int):
        '''发送数据到末端的指定来源， 每次长度不超过256字节，超过就切成多个包发。
        :param arm: 机械手臂ID “A” OR “B”
        :param data: 要传递的字节数据 (长度不超过2256)
        :param size_int: int, 发送的字节长度，不能超过256
        :param com: 信息来源， 1:CAN端; 2：com1; 3:com2
        :return: bool
        '''

### (10) 离线规划轨迹相关（PVT）

    send_pvt_file(arm:str, pvt_path: str, id: int):
        '''上传PVT文件给指定ID
        :param arm: 机械手臂ID “A” OR “B”
        :param pvt_path: 本地pvt文件的绝对/相对路径
        :param id: 范围1-99. 需要在 ARM_STATE_PVT 状态，即： set_arm_state(arm='A',state=2)

            PVT文件格式见：DEMO_SRS_Left.fmv
            数据首行为行数和列数信息，“PoinType=9@9341 ”表示该PVT文件含9列数据，一共9341个点位。
            数据为什么是9列？ 首先前八列为关节角度， 为什么是8？ 我们预留了8关节，人形臂为7自由度，前7个有效值，第八列都填充0，
            好的，第九列，第九列是个标记列，全填0即可。
        '''

    set_pvt_id(arm:str,id:int):
        '''设置指定id号的pvt路径并运行
        :param arm: 机械手臂ID “A” OR “B”
        :param id: 范围1-99. 需要在 ARM_STATE_PVT 状态，即： set_arm_state(arm='A',state=2)
        :return:
            int : 1: True,  2: False
        '''

### (11) 指令发送，以1000HZ频率进行发送
    clear_set():
        '''指令发送前清除
        :return:
            int: 1: True; 0: Flase
        '''

    send_cmd():
        '''发送指令
        :return:
            int: 1: True; 0: Flase
        '''

    BUFFER以1K HZ 频率刷新， 将发送机器人的数据或指令在clear_set()和send_cmd()中发送，以确保数据不受干扰，且下发频率高。

    注意：
        高频（IKHZ）收发的指令必须复合调用：
    
                以下指令设置必须在clear_set() 和send_cmd()之间才起效（忽略输入的测试值）：
                set_state(arm='A',state=3)
                set_drag_space(arm='A',dgType=1)
                set_impedance_type(arm='A',type=1)
                set_pvt_id(arm='A',id=1)
                set_card_kd_params(arm='A',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
                set_joint_kd_params(arm='A',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
                set_force_cmd(arm='A',f=1.)
                set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                fcAdjLmt=10.)
                set_vel_acc(arm='A',velRatio=1, AccRatio=1)
                set_tool(arm='A',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
                set_joint_cmd_pose(arm='A',joints=[0.,0.,0.,0.,0.,0.,6.])
                stop_collect_data()
                clear_error(arm: str)
                collect_data(targetNum7, targetID=[0,1,2,3,4,5,6,
                                                     0,0,0,0,0,0,0,
                                                     0,0,0,0,0,0,0,
                                                     0,0,0,0,0,0,0,
                                                     0,0,0,0,0,0,0], recordNum=1000)


### (12) 切换手臂目标状态
    set_state(arm:str,state:int):
        '''设置状态
        :param arm: 机械手臂ID “A” OR “B”
        :param state:
                   0 下伺服
                   1 位置跟随
                   2 PVT
                   3 扭矩
                   4 协作释放
        :return:
        '''
### (13) 设置指定手臂在扭矩模式下阻抗类型

    set_impedance_typearm:str,type: int):
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
### （14）设置指定手臂的关节跟随速度/加速度

    set_vel_acc(arm:str, velRatio: int, AccRatio: int):
        '''设置速度和加速度百分比
        :param arm: 机械手臂ID “A” OR “B”
        :param velRatio: 速度百分比, 值范围： 0~100
        :param AccRatio: 加速度百分比， 值范围：0~100
        :return:
            int： 1: True; 0:Flase
        '''

### （15）设置指定手臂的阻抗参数

    set_joint_kd_params(arm:str, K: list, D: list):
        '''设置关节阻抗参数

        #关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
        1-7关节阻尼0-1之间
        :param arm: 机械手臂ID “A” OR “B”
        :param K: list(7,1). 刚度 牛米 / 度 。 设置每个轴的的力为刚度系数。 如K=[2，2,2,1,1,1,1]，第1到3轴有2N作为刚度系数参与控制计算，第4到7轴有1N作为刚度系数参与控制计算。
        :param D: list(7,1). 阻尼 牛米 / (度 / 秒)。 设置每个轴的的阻尼系数。
        :return:
            int : 1: True,  2: False
        '''

    set_cart_kd_params(arm:str, K: list, D: list, type: int):
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

	注：1.参数在不同构型下的表现不同，请自行调节值到合适范围

### (16) 设置指定手臂的力控参数和力控指令

    set_force_control_params(arm:str, fcType: int, fxDirection: list, fcCtrlpara: list, fcAdjLmt: float):
        '''设置力控参数
        :param arm: 机械手臂ID “A” OR “B”
        :param fcType: 力控类型 0:坐标空间力控;1:工具空间力控(暂未实现)
        :param fxDirection: list(6,1). 力控方向 X,Y,Z,A,B,C控制方向.如力控方向为z,fxDirection=[0,0,1,0,0,0]
        :param fcCtrlpara: list(7,1). 控制参数 目前全0
        :param fcAdjLmt:毫米，允许的调节范围
        :return:
            int : 1: True,  2: False
        '''

    set_force_cmd(arm:str, f:float):
        '''设置力控指令
        :param arm: 机械手臂ID “A” OR “B”
        :param f: 目标力 单位牛或者牛米
        :return:
            int : 1: True,  2: False
        '''
### (17) 设置指定手臂的关节跟踪指令值
    set_joint_cmd_pose(arm:str, joints:list):
        '''设置关节跟踪指令值
        :param arm: 机械手臂ID “A” OR “B”
        :param joints: list(7,1). 角度，非弧度，在位置跟随和扭矩模式下均有效
        :return:
            int : 1: True,  2: False
        '''

### (18) 设置指定手臂的拖动空间
    set_drag_space(self,arm:str, dgType: int):
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
    注意：
        1.关节拖动需要在关节阻抗状态下使用
        2.笛卡尔拖动需要在笛卡尔阻抗下使用
        3.切换不同拖动模式前需要退出拖动模式再切换，否则控制效果是叠加混乱的。

    注意： 
        1.关节拖动需要在关节阻抗状态下使用
        2.笛卡尔拖动需要在笛卡尔阻抗下使用
        3.切换不同拖动模式前需要退出拖动模式再切换，否则控制效果是叠加混乱的。

### (19) 设置末端笛卡尔方向的旋转

    set_EefCart_control_params(,arm:str, fcType: int, CartCtrlPara: list):
        '''设置末端笛卡尔阻抗参数
        :param arm: 机械手臂ID “A” OR “B”
        :param fcType: 
              fcType=1，为自定义末端旋转方向。 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
              fcType=2，为系统自动计算末端笛卡尔旋转。 CartCtrlPara全填0
        :param CartCtrlPara: list(7,1). 控制参数前三个为旋转信息，基于基座的XYZ旋转。
        :return:
            int : 1: True,  2: False
        '''

### (20) 位置模式下规划当前点到目标点功能

    pln_init(self,config_path):
        '''关节空间规划初始化
        :param config_path: 本地机械臂配置文件*.MvKDCfg, 可相对路径.
        :return:
            ture or false
        '''

    setPln_joint(arm:str,start_joints:list, target_joints:list, velRatio:float,accRatio:float):
        '''位置模式下使用该接口传输目标关节点位，防止通信抖动
        :param arm: 机械手臂ID “A” OR “B”
        :param start_joints list(7, 1).起始点关节位置，单位角度
        :param target_joints list(7, 1).目标关节位置，单位角度
        :param velRatio float .规划插点的速度百分比， 范围0~1
        :param accRatio float .规划插点的加速度百分比，范围0~1
        :return:
            ture or false
        '''

    setPln_Cart(self,arm:str, pset: ctypes.c_void_p) -> bool:
        """位置模式下使用该接口传输目标笛卡尔坐标，防止通信抖动
        :param arm: 机械手臂ID “A” OR “B”
        :param pset: 由计算接口movLA(start_xyzabc: List[float], end_xyzabc: List[float],
              ref_joints: List[float], vel: float, acc: float,freq_hz:int,
              dimension: int = 7) -> tuple[List[List[float]], ctypes.c_void_p]计算得出
        :return:
            ture or false
        """   

    stopRunPln_joint(arm: str):
        '''停止之前的规划运动，关节空间和笛卡尔空间都适用t
        :param arm: 机械手臂ID “A” OR “B”
        '''



## 三、案例脚本
 见 DEMO_PYTHON/readme.md

### 扭矩模式下刚度和阻尼的建议：
    刚度用来衡量物体抗变形的能力。刚度越大，形变越小力的传导率高，运动时感觉很脆很硬；反之，刚度越小，形变大，形状恢复慢，传递力效率低，运动时感觉比较柔软富有韧性。
    阻尼用来衡量物体耗散振动能量的能力。阻尼越大，物体振幅减小越快，但对力、位移的响应迟缓，运动时感觉阻力大，有粘滞感； 阻尼越小，减震效果减弱，但运动阻力小，更流畅，停止到位置时有余震感。

    在精密定位、点无接触式操作的应用下，需要高刚度，中高阻尼的配合。高刚度确保消除擦产生大力，快速到达精确位置，足够的阻尼能够抑制震荡。
    在刚性表面打磨、装配应用下，需要低中刚度，高阻尼的配合。低刚度避免与环境强对抗导致不稳定和过大冲击力，高阻尼消耗能量，抑制接触震荡，稳定接触力。
    生物组织操作、海绵打磨等柔性环境接触应用下，需要中刚度中阻尼的配合。中等刚度提供一定的位置跟随能力同时避免压坏柔性物体，中度阻尼平衡响应速度和平稳性。
    在人机协作、示教编程等安全接触应用下，需要极低刚度和中度阻尼的配合。极低刚度使得机械臂非常的顺从，接触力很小也能感知，中等的阻尼提供基本稳定。

    # 协作机器人关节柔性显著，当使用纯关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
    1-7关节刚度系数不超过12
    1-7关节阻尼系数0-1之间

    # 在笛卡尔阻抗模式下：
    1-3平移方向刚度系数不超过1200, 4-6旋转方向不超过600。 零空间刚度系数不超过100
    平移和旋转阻尼系数0-1之间







        



    
