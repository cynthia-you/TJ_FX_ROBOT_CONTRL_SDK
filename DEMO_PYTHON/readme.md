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


## 计算showcases

### 1. 计算SDK 功能模块完整演示
            showcase_kinematics_all_functions.py

### 2. 计算逆解失败总结
            showcase_ik_failed_conclusion.py

### 3. 两条手臂同时计算
            showcase_two_arms.py

### 4. CCS右臂工具动力学辨识演示脚本
            showcase_identy_tool_dynamic_CCS_B.py
    
### 5. SRS右臂工具动力学辨识演示脚本
            showcase_identy_tool_dynamic_SRS_B.py