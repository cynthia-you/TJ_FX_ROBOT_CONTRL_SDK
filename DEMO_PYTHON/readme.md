# ATTENTION
    1.  请先熟练使用MARVIN_APP或MarvinPlatform软件。操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
    2.  DEMO_C++/ 和 DEMO_PYTHON/ 下为接口使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
        这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
            比如:
                a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。



## 一、SDK文档
    请阅读SDK文档，了解接口的功能和输入参数类型数值含义。

    SDK的主文档为master分支下的主文档：readme.md

    机器人控制SDK文档：
        c++_doc_contrl.md
        python_doc_contrl.md

    机器人计算SDK文档：
        c++_doc_kine.md
        python_doc_kine.md

## 二、SDK库文件夹下文件说明
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

## 三、 SDK库
    SDK_PYTHON为天机双臂机器人和人形机器人基于python开发的SDK，其分为:
        控制SDK：SDK_PYTHON/fx_robot.py 
        和运动学计算的SDK:SDK_PYTHON/fx_kine.py


### 3.1使用自动化编译脚本：
        master分支下marvinSDK_windows.bat运行可自动编译C++和python调用的dll文件
        master分支下marvinSDK_ubuntu.sh运行可自动编译C++和python调用的so文件

### 3.2.1 编译so动态库:
    linux设备编译:
        控制SDK(contrlSDK)，以下方法均可编译: 
			1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN
			2./contrlSDK/makefile 生成libMarvinSDK.so
        运动学SDK(kinematicsSDK)，以下方法均可编译: 
			1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt 
			2./kinematicsSDK/makefile 生成libKine.so
	编译的libKine.so 和 libMarvinSDK.so 供编译机器下的下C++和python使用

### 3.2.2 编译c++调用的dll动态库:
    1)windows下使用MinGW编译dll动态库:
			控制SDK(contrlSDK): g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN
            运动学SDK(kinematicsSDK): g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.dll
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下C++使用

			
### 3.2.3 编译python调用的dll动态库
    1)linux下编译dll动态库:
        控制SDK(contrlSDK):  x86_64-w64-mingw32-g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -DCMPL_WIN -static -static-libgcc -static-libstdc++ -lws2_32 -lpthread -lwinmm
        运动学SDK(kinematicsSDK): g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.dll 

	2）windows下使用MinGW编译dll动态库：
			控制SDK（contrlSDK）：g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
			运动学SDK(kinematicsSDK)：g++ *.cpp -Wall -w -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下python使用

## 四、 控制showcases
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
### 21. 工具笛卡尔阻抗
        showcase_torque_EefCart_impedance_arm_A.py
### 22. 指定关节伺服软复位
        showcase_servo_reset.py
### 23. 检查指令下发的延迟时间
        showcase_check_cmd_delay.py
### 24. 关节空间以规划方式下发指令，消除抖动
        showcase_pln_joint_positionMode.py
### 25. 笛卡尔空间以规划方式下发指令，实现直线约束
        showcase_pln_cart_positionMode.py
### 26. 关节空间以规划方式下发指令，并中断运行
        showcase_pln_joint_positionMode_with_break.py
### 27. 笛卡尔空间以规划方式下发指令，并中断运行
        showcase_pln_cart_positionMode_with_break.py

## 五. 计算showcases

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

### 6. 逆解参考基准
            showcase_ik_nsp_two_arms.py
            
### 7.在线直线规划并以笛卡尔阻抗模式以50HZ频率执行点位
            showcase_online_pln_movl.py
            
### 8.在线直线规划，约束构型并以笛卡尔阻抗模式以50HZ频率执行点位
            showcase_online_pln_movl_keepj.py

### 9.在线直线规划，约束构型并以笛卡尔阻抗模式以50HZ频率执行点位，定于旋转
            showcase_online_pln_movl_with_specific_rot.py
