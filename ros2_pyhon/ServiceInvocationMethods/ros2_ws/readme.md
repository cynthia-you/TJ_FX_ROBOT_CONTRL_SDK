# 编译,terminal 窗口1:
# 在workspace根目录
```shell
colcon build --packages-select marvin_interfaces
colcon build --packages-select marvin_robot
source install/setup.bash
```
# 运行节点 
```shell
ros2 run marvin_robot robot_node
```


# 调用服务 terminal 窗口2:
```shell
source install/setup.bash
```

# 控制
    机器人控制的主逻辑为:
    UDP连接机器人,通过接收数的更新据确认为有效连接
    |
    选择模式,设置模式下对应的参数
    |
    下发关节指令/力指令
    |
    ...
    |
    任务完成,释放机器人以便别的程序或者用户连接机器人


    在机器人的控制状态目前提供以下:
        1)位置模式/关节跟随模式(该模式高刚度,高精度,碰撞有危险)
        2)PVT模式/离线轨迹复现模式(提前规划500HZ的轨迹,速度,加速度也要规划)
        3)扭矩模式/阻抗模式,阻抗模式又细化为关节阻抗,笛卡尔阻抗,力控三种
        4)协作释放模式,该模式用于机器人碰撞后扭开撞作一团的手臂,或者想要手动改变机器人构型的状态
        5)下始能/复位, 不同状态切换需要复位(安全起见),静止状态下可不复位切换(混合控制)
    
    位置模式和扭矩模式都需要先设置运行的参数:
        1)位置模式设置速度和加速度的百分比
        2)扭矩模式下除了速度加速度百分比要设置,还需要设置刚度和阻尼参数
        3)特殊的力控模式是设置力控的行程范围(毫米)
    
    1KHZ数据采集
        1)数据采集与机器人控制状态无关,无论什么模式都可采集数据
        2)数据采集可一次性采集35列数据,即35个特征, 一次性可采集100万行数据, 采集满可新建采集:
            左臂特征序号：
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
            右臂特征序号对应 + 100
    
    
    另外,机器人在扭矩模式下可以用末端的外部按钮实现拖动功能:
        1)关节阻抗模式下,选择关节拖动,可实现关节的柔顺拖动
        2)笛卡尔阻抗模式下,选择笛卡尔拖动中单一方向的拖动:X,Y,Z,旋转四种. 切换拖动方向需要先退出拖动,再切换为另一方向(否则控制效果是混乱的)

# 服务列表

## 1.连接服务
连接机器人 robot_ip: 机器人IP地址,确保网线连接可以ping通
```shell
ros2 service call /connect_robot marvin_interfaces/srv/Connect "{robot_ip: '192.168.1.190'}"
```

## 2.日志开关服务
日志开关：
global_or_local： 'global' or 'local'. 选择分级日志，全局，还是重要的连接信息。
flg: 0 or 1 . 开关信号，1开0关
```shell
ros2 service call /log_switch marvin_interfaces/srv/LogSwitch "{global_or_local: 'global', flag: '0'}"
ros2 service call /log_switch marvin_interfaces/srv/LogSwitch "{global_or_local: 'local', flag: '1'}"
```

## 3.获取当前机器人控制SDK版本
返回SDK版本号， 数字值
```shell
ros2 service call /sdk_version marvin_interfaces/srv/SdkVersion
```

## 4.伺服/驱动清错服务
指定手臂请错:
arm_id: 'A' or 'B'
```shell
ros2 service call /clear_arm_error marvin_interfaces/srv/ArmClearErr "{arm_id: 'A'}"
ros2 service call /clear_arm_error marvin_interfaces/srv/ArmClearErr "{arm_id: 'B'}"
```

## 5.机械臂软急停服务
指定手臂软急停:
arm_id: 'A' or 'B' or 'AB'
```shell
ros2 service call /arm_soft_stop marvin_interfaces/srv/ArmSoftStop "{arm_id: 'A'}"
ros2 service call /arm_soft_stop marvin_interfaces/srv/ArmSoftStop "{arm_id: 'B'}"
ros2 service call /arm_soft_stop marvin_interfaces/srv/ArmSoftStop "{arm_id: 'AB'}"
```

## 6.获取机械臂伺服错误码服务
指定手臂请错:
arm_id: 'A' or 'B'
```shell
ros2 service call /get_servo_error_code marvin_interfaces/srv/GetServoErrCode "{arm_id: 'A'}"
ros2 service call /get_servo_error_code marvin_interfaces/srv/GetServoErrCode "{arm_id: 'B'}"
```

## 7.设置机器人状态服务
设置指定手臂状态：
arm_id: 'A' or 'B'
state:
        0   #下伺服
        1   #位置跟随
        2   #PVT
        3   #扭矩
```shell
ros2 service call /set_arm_state marvin_interfaces/srv/SetArmState "{arm_id: 'A', state: 3}"
ros2 service call /set_arm_state marvin_interfaces/srv/SetArmState "{arm_id: 'B', state: 3}"
```

## 8.设置机器人阻抗模式服务
设置指定手臂阻抗模式：
arm_id: 'A' or 'B'
type:
        1 关节阻抗
        2 坐标阻抗
        3 力控
注：需要在state为3才能以阻抗模式控制!!!
```shell
ros2 service call /set_impedance_type marvin_interfaces/srv/SetImpedanceType "{arm_id: 'A', type: 1}"
ros2 service call /set_impedance_type marvin_interfaces/srv/SetImpedanceType "{arm_id: 'B', type: 1}"
```

## 9.设置机器人拖动空间服务
设置指定手臂拖动空间：
arm_id: 'A' or 'B'
dg_type:
        0 退出拖动模式
        1 关节空间拖动
        2 笛卡尔空间x方向拖动
        3 笛卡尔空间y方向拖动
        4 笛卡尔空间z方向拖动
        5 笛卡尔空间旋转方向拖动
```shell
ros2 service call /set_drag_space marvin_interfaces/srv/SetDragSpace "{arm_id: 'A', dg_type: 1}"
ros2 service call /set_drag_space marvin_interfaces/srv/SetDragSpace "{arm_id: 'B', dg_type: 1}"
```

## 10.设置机器人运动速度和加速度服务
设置指定手臂的速度和加速度百分比
arm_id: 'A' or 'B'
vel: 速度百分比
acc: 加速度百分比
```shell
ros2 service call /set_arm_vel_acc marvin_interfaces/srv/SetArmVelAcc "{arm_id: 'A', vel: 10, acc: 10}"
ros2 service call /set_arm_vel_acc marvin_interfaces/srv/SetArmVelAcc "{arm_id: 'B', vel: 5, acc: 5}"
```

## 11.设置机器人工具运动学和动力学参数服务
设置指定手臂的工具信息：
arm_id: 'A' or 'B'
kine_para: list(6,1). 运动学参数 XYZABC 单位毫米和度
dynamic_para: list(10,1). 动力学参数分别为 质量M  质心[3]:mx,my,mz 惯量I[6]:XX,XY,XZ,YY,YZ,ZZ
```shell
ros2 service call /set_tool_params marvin_interfaces/srv/SetToolPara "{arm_id: 'A', kine_para: [0.,0.,0.,0.,0.,0.], dynamic_para: [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]}"
ros2 service call /set_tool_params marvin_interfaces/srv/SetToolPara "{arm_id: 'B', kine_para: [0.,0.,0.,0.,0.,0.], dynamic_para: [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]}"
```

## 12.为指定手臂上传PVT轨迹为指定序号
    设置指定手臂的PVT的ID号并运行
    arm_id: 'A' or 'B'
    pvt_path: 绝对路径
    id: 指定序号,范围1-99.
```shell
ros2 service call /send_pvt_file marvin_interfaces/srv/SendPvt "{arm_id: 'A', pvt_path: '/home/fusion/projects/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/LoadData_ccs_right/LoadData/IdenTraj/LoadIdenTraj_MarvinCCS_Left.fmv', id: 2}"
ros2 service call /send_pvt_file marvin_interfaces/srv/SendPvt "{arm_id: 'B', pvt_path: '', id: 1}"
```
## 13.设置机器人PVT序号服务
    设置指定手臂的PVT的ID号并运行
    arm_id: 'A' or 'B'
    pvt_id: 范围1-99. 需要在手臂状态state为2下有效
    ```shell
    ros2 service call /set_pvt_id marvin_interfaces/srv/SetPvtId "{arm_id: 'A', pvt_id: 2}"
    ros2 service call /set_pvt_id marvin_interfaces/srv/SetPvtId "{arm_id: 'B', pvt_id: 1}"
    ```

## 14.设置机器人笛卡尔阻抗参数服务
设置指定手臂的笛卡尔阻抗参数：
arm_id: 'A' or 'B'
k: list(6,1). K[0]-k[2] 牛/毫米， K[3]-k[6] 牛米/度，K[6] 零空间总和K系数 牛米/度
d: list(6,1). D[0]-D[2] 牛/(毫米/秒 D[3]-D[6] 牛米/(度/秒）
type:int. 设置手臂的的阻抗类型：        1 关节阻抗 ，2 坐标阻抗 3 力控
```shell
ros2 service call /set_card_kd marvin_interfaces/srv/SetCardKD "{arm_id: 'A', k: [3000,3000,3000,60,60,60,0], d: [20,20,20,2,2,2,0], type: 2}"
ros2 service call /set_card_kd marvin_interfaces/srv/SetCardKD "{arm_id: 'B', k: [3000,3000,3000,60,60,60,0], d: [20,20,20,2,2,2,0], type: 2}"
```

## 15.设置机器人关节阻抗参数服务
设置指定手臂的关节阻抗参数：
arm_id: 'A' or 'B'
k:list(7,1). 刚度 牛米 / 度
d: list(7,1). 阻尼 牛米 / (度 / 秒)
```shell
ros2 service call /set_joint_kd marvin_interfaces/srv/SetJointKD "{arm_id: 'A', k: [3,3,3,1.6, 1, 1, 1], d: [0.6,0.6,0.6,0.4,0.2,0.2,0.2]}"
ros2 service call /set_joint_kd marvin_interfaces/srv/SetJointKD "{arm_id: 'B', k: [3,3,3,1.6, 1, 1, 1], d: [0.6,0.6,0.6,0.4,0.2,0.2,0.2]}"
```

## 16.设置机器人力控阻抗参数服务
设置指定手臂的力控阻抗参数：
arm_id: 'A' or 'B'
fc_type: 力控类型 0:坐标空间力控;1:工具空间力控(暂未实现)
directions: list(6,1). 力控方向 需要控制方向设1，目前只支持 X,Y,Z控制方向.如力控方向为z,fxDirection=[0,0,1,0,0,0]
fc_ctrl_para: list(7,1). 控制参数 目前全0
adjustment:毫米，允许的调节范围
```shell
ros2 service call /set_force_ctrl_para marvin_interfaces/srv/SetForceCtrlPara "{arm_id: 'A', fc_type: 0, directions: [0, 0, 1, 0, 0, 0], fc_ctrl_para: [0, 0, 0, 0, 0, 0, 0], adjustment: 10.}"
ros2 service call /set_force_ctrl_para marvin_interfaces/srv/SetForceCtrlPara "{arm_id: 'B', fc_type: 0, directions: [0, 0, 1, 0, 0, 0], fc_ctrl_para: [0, 0, 0, 0, 0, 0, 0], adjustment: 10.}"
```

## 17.设置机器人力指令服务
设置指定手臂的力控指令：
arm_id: 'A' or 'B'
f: 目标力 单位牛或者牛米
```shell
ros2 service call /set_force_cmd marvin_interfaces/srv/SetForceCmd "{arm_id: 'A', f: 1.}"
ros2 service call /set_force_cmd marvin_interfaces/srv/SetForceCmd "{arm_id: 'B', f: 1.}"
```

## 18.发送关节指令服务
设置指定手臂的目标关节指令：
arm_id: 'A' or 'B'
joints: list(7,1). 角度，非弧度，在位置跟随和扭矩模式下均有效
```shell
ros2 service call /set_joint_cmd_pos marvin_interfaces/srv/SetJointCmdPos "{arm_id: 'A', joints: [10.,10.,10.,10.,10.,10.,10.]}"
ros2 service call /set_joint_cmd_pos marvin_interfaces/srv/SetJointCmdPos "{arm_id: 'B', joints: [10.,10.,10.,10.,10.,10.,10.]}"
```

## 19.订阅数据服务
订阅双臂数据，返回值为嵌套字典：
states，outputs，inputs 均包含两个手臂信息
marvin_interfaces.srv.RobotData_Response(
    data=marvin_interfaces.msg.MarvinResponse(
        robot_name=['Marvin_sub_data'], 
        states=[marvin_interfaces.msg.State(cur_state=0, cmd_state=0, err_code=0), 
                marvin_interfaces.msg.State(cur_state=0, cmd_state=0, err_code=0)], 
        outputs=[
                marvin_interfaces.msg.Output(out_frame_serial=0, fb_joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_vel=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_cmd=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_c_toq=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_s_toq=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_them=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_joint_firc=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_joint_firc_dot=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_joint_force=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_cart_fn=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
                marvin_interfaces.msg.Output(out_frame_serial=0, fb_joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_vel=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_cmd=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_c_toq=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_s_toq=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], fb_joint_them=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_joint_firc=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_joint_firc_dot=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_joint_force=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], est_cart_fn=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])], 
        inputs=[
                marvin_interfaces.msg.Input(rt_in_switch=0, imp_type=0, in_frame_serial=0, tool_kine=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], tool_dyn=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_cmd_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_vel_ratio=0, joint_acc_ratio=0, joint_k=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_d=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], drag_sp_type=0, drag_sp_para=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cart_kd_type=0, cart_k=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cart_d=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cart_kn=0.0, cart_dn=0.0, force_fb_type=0, force_type=0, force_dir=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], force_pidul=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], force_adj_lmt=0.0, force_cmd=0.0, set_tags=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], update_tags=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], pvt_id=0, pvt_id_update=0, pvt_run_id=0, pvt_run_state=0), 
                marvin_interfaces.msg.Input(rt_in_switch=0, imp_type=0, in_frame_serial=0, tool_kine=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], tool_dyn=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_cmd_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_vel_ratio=0, joint_acc_ratio=0, joint_k=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_d=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], drag_sp_type=0, drag_sp_para=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cart_kd_type=0, cart_k=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cart_d=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cart_kn=0.0, cart_dn=0.0, force_fb_type=0, force_type=0, force_dir=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], force_pidul=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], force_adj_lmt=0.0, force_cmd=0.0, set_tags=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], update_tags=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], pvt_id=0, pvt_id_update=0, pvt_run_id=0, pvt_run_state=0)]))

```shell
ros2 service call /get_robot_data marvin_interfaces/srv/RobotData "{}"
```

## 20.获取机器人配置参数服务
type: float or int .参数类型
para_name:  参数名见robot.ini 
return:参数值
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

```shell
ros2 service call /get_param marvin_interfaces/srv/GetPara "{type: 'float', para_name: 'R.A0.BASIC.BDRange'}"
ros2 service call /get_param marvin_interfaces/srv/GetPara "{type: 'int', para_name: 'R.A0.BASIC.Dof'}"
```

## 21.设置机器人配置参数服务
type: float or int .参数类型
para_name:  参数名见robot.ini
value: 设置值
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

```shell
ros2 service call /set_param marvin_interfaces/srv/SetPara "{type: 'float', para_name: 'R.A0.CTRL.CartJNTDampJ1', value: 0.0}"
ros2 service call /set_param marvin_interfaces/srv/SetPara "{type: 'int', para_name: 'R.A0.BASIC.Type', value: 0}"
ros2 service call /set_param marvin_interfaces/srv/SetPara "{type: 'int', para_name: 'R.A0.BASIC.Dof', value: 7}"
```

## 22.保存参数服务
```shell
ros2 service call /save_param marvin_interfaces/srv/SaveParam
```

## 23.数据收集服务
target_num:采集特征的个数,最多35个.
target_ids:
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
record_num:最小1000行,最多100万行.
运行服务即可开始以1KHZ采集数据
```shell
ros2 service call /collect_data marvin_interfaces/srv/CollectData "{target_num: 2, target_ids: [11,31,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0], record_num: 1000000}"
```


## 24.数据采集停止服务
```shell
ros2 service call /stop_collect_data marvin_interfaces/srv/StopCollectData
```


## 25.保存的数据
local_save_path: 保存路径,可相对可绝对. 采集的数据的前两列为数据帧信息,第三列开始为采集特征.
```shell
ros2 service call /save_collected_data marvin_interfaces/srv/SaveCollectData "{local_save_path: 'a1223.txt'}"
```

## 26.释放机器人
```shell
ros2 service call /release_robot marvin_interfaces/srv/Release
```


# 简单使用演示
## 1.左臂位置模式控制
        1.1 在terminsal 1 运行节点 
        ```shell
        ros2 run marvin_robot robot_node
        ```
        1.2 在terminal 2 调用服务 :
        ```shell
        source install/setup.bash
        ```
        1.3 连接机器人:
    
        ```shell
        ros2 service call /connect_robot marvin_interfaces/srv/Connect "{robot_ip: '192.168.1.190'}"
        ```
        1.4 驱动清错
        ```shell
        ros2 service call /clear_arm_error marvin_interfaces/srv/ArmClearErr "{arm_id: 'A'}"
        ```
        1.5 设置控制状态为位置模式
        ```shell
        ros2 service call /set_arm_state marvin_interfaces/srv/SetArmState "{arm_id: 'A', state: 1}"
        ```
        
        1.6 设置速度加速度百分比为50,安全测试后可设为全速100
        ```shell
        ros2 service call /set_arm_vel_acc marvin_interfaces/srv/SetArmVelAcc "{arm_id: 'A', vel: 100, acc: 100}"
        ```
        1.7 设置目标点位
        ```shell
        ros2 service call /set_joint_cmd_pos marvin_interfaces/srv/SetJointCmdPos "{arm_id: 'A', joints: [10.,10.,10.,10.,10.,10.,10.]}"
        ```
        ...
        1.8 任务完成下,释放机器人,以便别的节点或用户调用
        ```shell
        ros2 service call /release_robot marvin_interfaces/srv/Release
        ```

## 1.2 扭矩模式下关节阻抗控制机器人
        1.1 在terminsal 1 运行节点 
        ```shell
        ros2 run marvin_robot robot_node
        ```
        1.2 在terminal 2 调用服务 :
        ```shell
        source install/setup.bash
        ```
        1.3 连接机器人:
        
        ```shell
        ros2 service call /connect_robot marvin_interfaces/srv/Connect "{robot_ip: '192.168.1.190'}"
        ```
        1.4 驱动清错
        ```shell
        ros2 service call /clear_arm_error marvin_interfaces/srv/ArmClearErr "{arm_id: 'A'}"
        ```
        1.5 设置控制状态为扭矩模式
        ```shell
        ros2 service call /set_arm_state marvin_interfaces/srv/SetArmState "{arm_id: 'A', state: 3}"
        ```
        1.6 选择阻抗类型为关节阻抗
        ```shell
        ros2 service call /set_impedance_type marvin_interfaces/srv/SetImpedanceType "{arm_id: 'A', type: 1}"
        ```
        1.7 设置速度加速度百分比为50,安全测试后可设为全速100
        ```shell
        ros2 service call /set_arm_vel_acc marvin_interfaces/srv/SetArmVelAcc "{arm_id: 'A', vel: 100, acc: 100}"
        ```
        1.8 设置关节阻抗的刚度参数和阻尼参数
        ```shell
        ros2 service call /set_joint_kd marvin_interfaces/srv/SetJointKD "{arm_id: 'A', k: [3,3,3,1.6, 1, 1, 1], d: [0.6,0.6,0.6,0.4,0.2,0.2,0.2]}"
        ```
        1.9 设置目标点位
        ```shell
        ros2 service call /set_joint_cmd_pos marvin_interfaces/srv/SetJointCmdPos "{arm_id: 'A', joints: [10.,10.,10.,10.,10.,10.,10.]}"
        ```
        ...
        1.10 任务完成下,释放机器人,以便别的节点或用户调用
        ```shell
        ros2 service call /release_robot marvin_interfaces/srv/Release
        ```
## 其余请参考DEMO_PYTHON下的示列使用逻辑
    
