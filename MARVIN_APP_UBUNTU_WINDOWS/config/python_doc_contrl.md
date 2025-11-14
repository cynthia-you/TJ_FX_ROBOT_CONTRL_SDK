# 天机-孚晞 机器人工具包 MarvinSDK
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1003
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2025-09


机械臂错误码：

typedef enum
{
    ARM_ERR_BusPhysicAbnoraml = 1, //"总线拓扑异常"
    ARM_ERR_ServoError = 2,//"伺服故障"
    ARM_ERR_InvalidPVT = 3,//"PVT异常"
    ARM_ERR_RequestPositionMode = 4,//"请求进位置失败"
    ARM_ERR_PositionModeOK = 5,//"进位置失败"
    ARM_ERR_RequestSensorMode = 6,//"请求进扭矩失败"
    ARM_ERR_SensorModeOK = 7,//"进扭矩失败"
    ARM_ERR_RequestEnableServo = 8,//"请求上伺服失败"
    ARM_ERR_EnableServoOK = 9,//"上伺服失败"
    ARM_ERR_RequestDisableServo = 10, //"请求下伺服失败
    ARM_ERR_DisableServoOK = 11, //"下伺服失败"
    ARM_ERR_InvalidSubState = 12, //"内部错"
    ARM_ERR_Emcy = 13, //"急停"
    ARM_DYNA_FLOAT_NO_GYRO = 14,//"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开"S
}ArmErrorCode;


## 一、API列表 fx_robot.py
  获取SDK版本  
  - SDK_version()

  清除指定手臂的缓存数据
  - clear_485_cache(arm: str)

  清出指定手臂错误
  - clear_error(arm: str)

  清除发送指令的缓存
  - clear_set()

  采集数据
  - collect_data(targetNum: int, targetID: list, recordNum: int)

  连接机器人
  - connect(robot_ip: str)

  获取机器人SDK日志
  - download_sdk_log(log_path: str)

  读取指定手臂的末端通信模块（485/can）返回的数据
  - get_485_data(arm: str, com: int)

  获取机器人指定类型的配置参数
  - get_param(type: str, paraName: str)

  获取指定手臂的伺服错误十六进制
  - get_servo_error_code(arm: str)

  主要日志开关接口， 0关1开
  - local_log_switch(flag: str)

  全局日志开关接口，0关1开
  - log_switch(flag: str)

  释放机器人
  - release_robot()

  保存采集的数据到指定路径
  - save_collected_data_as_csv_to_path(path: str)

  保存采集的数据到指定路径
  - save_collected_data_to_path(path: str)

  保存参数文件
  - save_para_file()

  发送指令
  - send_cmd()

  发送PVT路径到指定ID给指定手臂
  - send_pvt_file(arm: str, pvt_path: str, id: int)

  指定手臂发送末端模组协议指令
  - set_485_data(arm: str, data: bytes, size_int: int, com: int)

  设置指定手臂的笛卡尔阻抗参数
  - set_cart_kd_params(arm: str, K: list, D: list, type: int)

  设置指定手臂的拖动空间
  - set_drag_space(arm: str, dgType: int)

  设置指定手臂的力控指令
  - set_force_cmd(arm: str, f: float)

  设置指定手臂的力控参数
  - set_force_control_params(arm: str, fcType: int, fxDirection: list, fcCtrlpara: list, fcAdjLmt: float)

  设置指定手臂的阻抗类型
  - set_impedance_type(arm: str, type: int)

  设置指定手臂的目标关节角度
  - set_joint_cmd_pose(arm: str, joints: list)

  设置指定手臂的关节阻抗参数
  - set_joint_kd_params(arm: str, K: list, D: list)

  设置机器人指定类型的配置参数
  - set_param(type: str, paraName: str, value: float)

  设置指定手臂的pvt ID
  - set_pvt_id(arm: str, id: int)

  设置指定手臂的控制模式
  - set_state(arm: str, state: int)

  设置指定手臂的工具参数：运动学和动力学
  - set_tool(arm: str, kineParams: list, dynamicParams: list)

  设置指定手臂的速度和加速度，百分比
  - set_vel_acc(arm: str, velRatio: int, AccRatio: int)

  指定手臂软急停
  - soft_stop(arm: str)

  停止采集数据
  - stop_collect_data()

  订阅机器人实时数据结构体
  - subscribe(dcss)

  更新SDK版本
  - update_SDK(sdk_path: str)


## 二、API用法
### 2.1 API接口说明
    首先将fx_robot的类函数实例化，然后调用help()方法可一览所有方法，help(方法名)可详细了解方法的输入和返回， 里面写的详细！

    tj_robot = Marvin_Robot() #实例化
    tj_robot.help() #一览所有方法
    tj_robot.help('collect_data') #查看说明
    '''下面是collect_data接口的说明
        ===================== API 帮助 =====================
        
        方法: collect_data(targetNum: int, targetID: list[int], recordNum: int)
        
        采集数据
        :param targetNum:targetNum采集列数 值最大35， 因为一次最多采集35个特征。
        :param targetID: list(35,1) 对应采集数据ID序号(见下)
        :param recordNum: 采集行数，小于1000会采集1000行，设置大于一百万行会采集一百万行。
        :return:
                    采集数据ID序号
                    左臂
                        0-6     左臂关节位置
                        10-16   左臂关节速度
                        20-26   左臂外编位置
                        30-36   左臂关节指令位置
                        40-46   左臂关节电流（千分比）
                        50-56   左臂关节传感器扭矩NM
                        60-66   左臂摩擦力估计值
                        70-76   左臂摩檫力速度估计值
                        80-85   左臂关节外力估计值
                        90-95   左臂末端点外力估计值
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
        
        参数详情:
          targetNum: 类型: int, 
          targetID: 类型: list, 
          recordNum: 类型: int, 
        ==================================================
    '''


## 2.2 绑定类方法
    一般的方法可以单独调用，但是部分控制指令需要有使用先后逻辑：
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


    可以单条指令设置:
    clear_set()
    set_state(state=3)
    send_cmd()

    也可以多个指令一起设置：
    ''' ####  A arm ###'''
    clear_set()
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
    send_cmd()

    ''' ####  B arm ###'''
    clear_set()
    set_state(arm='B',state=3)
    set_drag_space(arm='B',dgType=1)
    set_impedance_type(arm='B',type=1)
    set_pvt_id(arm='B',id=1)
    set_card_kd_params(arm='B',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
    set_joint_kd_params(arm='B',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
    set_force_cmd(arm='B',f=1.)
    set_force_control_params(arm='B',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                      fcAdjLmt=10.)
    set_vel_acc(arm='B',velRatio=1, AccRatio=1)
    set_tool(arm='B',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
    set_joint_cmd_pose(arm='B',joints=[0.,0.,0.,0.,0.,0.,6.])
    send_cmd()



## 2.3 扭矩模式下刚度和阻尼的建议：
    刚度用来衡量物体抗变形的能力。刚度越大，形变越小力的传导率高，运动时感觉很脆很硬；反之，刚度越小，形变大，形状恢复慢，传递力效率低，运动时感觉比较柔软富有韧性。
    阻尼用来衡量物体耗散振动能量的能力。阻尼越大，物体振幅减小越快，但对力、位移的响应迟缓，运动时感觉阻力大，有粘滞感； 阻尼越小，减震效果减弱，但运动阻力小，更流畅，停止到位置时有余震感。

    在精密定位、点无接触式操作的应用下，需要高刚度，中高阻尼的配合。高刚度确保消除擦产生大力，快速到达精确位置，足够的阻尼能够抑制震荡。
    在刚性表面打磨、装配应用下，需要低中刚度，高阻尼的配合。低刚度避免与环境强对抗导致不稳定和过大冲击力，高阻尼消耗能量，抑制接触震荡，稳定接触力。
    生物组织操作、海绵打磨等柔性环境接触应用下，需要中刚度中阻尼的配合。中等刚度提供一定的位置跟随能力同时避免压坏柔性物体，中度阻尼平衡响应速度和平稳性。
    在人机协作、示教编程等安全接触应用下，需要极低刚度和中度阻尼的配合。极低刚度使得机械臂非常的顺从，接触力很小也能感知，中等的阻尼提供基本稳定。

    # 协作机器人关节柔性显著，当使用纯关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
    1-7关节刚度系数不超过2
    1-7关节阻尼系数0-1之间

    # 在笛卡尔阻抗模式下：
    1-3平移方向刚度系数不超过3000, 4-6旋转方向不超过100。 零空间刚度系数不超过20
    平移和旋转阻尼系数0-1之间


    订阅机器人数据见subdata_demo.py, 数据含义见structure_data.py
        无连接字典返回全为0：
        
            result {
                'para_name': ['Marvin_sub_data'], 
                'states': [{'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}],
                'outputs': [
                            {'frame_serial': 0, 
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


# 三、案例脚本
请注意：案例仅为参考使用，实地生产和业务逻辑需要您加油写~~~


订阅数据：subdata_demo.py

阻抗模式：
    1. 关节阻抗：torque_joint_impedance_arm_A.py
    2. 笛卡尔阻抗：torque_card_impedance_arm_A.py
    3. 力控阻抗：torque_force_impedance_arm_A.py

位置模式：position_arm_A.py

拖动模式：drag_arm_A.py

末端485读取设定：demo_485_arm_A.py

末端CAN读取设定：demo_can_arm_A.py

PVT运行：pvt_demo.py

采集数据：collect_data_demo.py







        



    
