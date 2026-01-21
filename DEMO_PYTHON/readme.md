# ATTENTION
    1.  请先熟练使用MARVIN_APP 或者https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/releases/ 下各个版本里的FxStation.exe， 操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
    2.  C++_DEMO/ 和 DEMO_PYTHON/ 下为接口使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
        这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
            比如:
                a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。


# 请检查本文件夹下  *.so *.dll 是否为最新编译
    
    *.so  LINUX下使用

    *.dll WINDOWS下使用



## 一. 控制showcases

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

### 1.双臂关节位置跟随控制演示
        showcase_position.py
### 2. 单臂执行PVT轨迹并保存数据的演示
        showcase_pvt_arm_A.py
### 3. 单臂扭矩模式关节阻抗控制演示
        showcase_torque_joint_impedance_arm_A.py
### 4. 单臂扭矩模式笛卡尔阻抗控制演示
        showcase_torque_cart_impedance_arm_A.py
### 5. 单臂扭矩模式力控演示
        showcase_torque_force_impedance_arm_A.py
### 6. 单臂关节阻抗模式拖动手臂演示
        showcase_joint_drag_arm_A.py
### 7. 单臂关节阻抗模式拖动手臂并保存数据演示
        showcase_drag_JointImpedance_and_save_data_arm_A.py
### 8. 单臂笛卡尔阻抗模式拖动手臂演示
        showcase_cart_drag_arm_A.py
### 9. 单臂笛卡尔阻抗模式拖动手臂并保存数据演示
        showcase_drag_CartImpedance_and_save_data_arm_A.py
### 10. 保存数据演示
        showcase_collect_data.py
### 11. 保存数据为CSV演示
        showcase_collect_data_as_csv.py
### 12. 保存工具动力学和运动学信息演示
        showcase_set_save_tool.py
### 13. 获取和设置机器人配置参数演示
        showcase_get_set_param.py
### 14. 单臂末端485通讯演示
        showcase_485_arm_A.py
### 15. 单臂末端CAN/CANFD通讯演示
        showcase_CAN_arm_A.py
### 16. 电机清错清零演示
        showcase_motor_encoder_clear.py
### 17. 协作释放演示
        showcase_collaborative_release.py
### 18. 松闸抱闸演示
        showcase_apply-brake_release-brake.py
### 19. 获取伺服错误码及对应错误原因
        showcase_servo_error.py
### 20. 拖动保存的轨迹转为PVT文件
        showcase_process_collect_data_to_pvt_format.py

## 二. 计算showcases

### 1. 计算SDK 功能模块完整演示
            showcase_kinematics_all_functions.py

### 2. 计算逆解失败总结
            showcase_ik_failed_conclusion.py

### 3. 两条手臂同时计算
            showcase_kine_two_arms.py

### 4. CCS右臂工具动力学辨识演示脚本
            showcase_identy_tool_dynamic_CCS_B.py
    
### 5. SRS右臂工具动力学辨识演示脚本
            showcase_identy_tool_dynamic_SRS_B.py

### 6. 设置工具校验正解
            showcase_set_tool.py

### 7. 逆解参考基准
            showcase_ik_nsp_two_arms.py
            
### 8.在线直线规划并以关节阻抗模式以50HZ频率执行点位
            showcase_online_pln_movl.py
            
### 9.在线直线规划，约束构型并以关节阻抗模式以50HZ频率执行点位
            showcase_online_pln_movl_keepj.py
