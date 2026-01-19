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



## 一. 计算showcases

### 1. 计算SDK 功能模块完整演示
            showcase_kinematics_all_functions.cpp

### 2. 计算逆解失败总结
            showcase_ik_failed_conclusion.cpp

### 3. 两条手臂同时计算
            showcase_kine_two_arms.cpp

### 5. 逆解参考基准
            showcase_ik_nsp_two_arms.cpp
            
### 6. 演示左臂离线和在线规划功能接口：
            showcase_online_and_offline_pln_all_function.cpp


### 7. 左臂关节阻抗50HZ执行离线直线规划文件：
            showcase_offline_movl_execution.cpp

### 8.左臂关节阻抗50HZ执行在线直线规划点：
            showcase_online_movla_execution.cpp

### 9.左臂关节阻抗50HZ执行约束构型的离线直线规划文件：
            showcase_offline_movl_keepj_execution.cpp

### 10. 左臂关节阻抗50HZ执行约束构型的在线直线规划点位：
            showcase_online_movl_keepja_execution.cpp


## 二. 控制showcases

### 1. 强制抱闸和强制松闸案例
            showcase_apply_brake_release_brake.cpp

### 2. 机器人进入协作释放案列
            showcase_collaborative_release.cpp

### 3. 在迪卡尔阻抗模式下,进去迪卡尔Y方向拖动,拖动并保存数据的控制案列
            showcase_drag_CartImpedance_save_data.cpp

### 4. 拖动控制案例
            showcase_drag_joint.cpp

### 5. 在关节阻抗模式下,进去关节拖动,拖动并保存数据的控制案列
            showcase_drag_JointImpedance_save_data.cpp

### 6. 获取和设置参数案列
           showcase_get_set_param_demo.cpp

### 7. 连接检查案列
            showcase_link_check.cpp

### 8. 关节位置跟随控制案列
            showcase_position_two_arms.cpp

### 9. 跑PVT轨迹并保存数据的案列
            showcase_pvt.cpp

### 10. 为笛卡尔阻抗控制案列
            torque_cart_impedance_demo.cpp

### 11. 为力控案列
            torque_force_impedance_demo.cpp

### 12. 关节阻抗控制案列
            torque_joint_impedance_demo.cpp

### 13. 在关节阻抗模式下,进去关节拖动,拖动并保存数据的控制案列
            showcase_drag_JointImpedance_save_data.cpp
            
