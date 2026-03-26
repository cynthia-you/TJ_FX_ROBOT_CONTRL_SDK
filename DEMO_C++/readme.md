# ATTENTION
    1.  请先熟练使用MARVIN_APP或MarvinPlatform软件。操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
    2.  DEMO_C++/ 和 DEMO_PYTHON/ 下为接口使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
        这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
            比如:
                a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。

# 一、SDK文档
    请阅读SDK文档，了解接口的功能和输入参数类型数值含义。

    SDK的主文档为master分支下的主文档：readme.md

    机器人控制SDK文档：
        c++_doc_contrl.md
        python_doc_contrl.md

    机器人计算SDK文档：
        c++_doc_kine.md
        python_doc_kine.md

# 二、 DEMO_C++/下必要文件
    
    请检查以下文件是否为为本机编译：
        libKine.dll
        libMarvinSDK.dll

        libKine.so
        libMarvinSDK.so

    请检查以下头文件是否存在：
        FXDG.h
        FxRobot.h
        MarvinSDK.h
        PointSet.h

## 2.1 编译控制库和运动计算库方法
## 2.1.1使用自动化编译脚本：
        master分支下marvinSDK_windows.bat运行可自动编译C++和python调用的dll文件
        master分支下marvinSDK_ubuntu.sh运行可自动编译C++和python调用的so文件


### 2.1.2 编译so动态库:
    linux设备编译:
        控制SDK(contrlSDK)，以下方法均可编译: 
            1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN
            2./contrlSDK/makefile 生成libMarvinSDK.so
        运动学SDK(kinematicsSDK)，以下方法均可编译: 
            1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt 
            2./kinematicsSDK/makefile 生成libKine.so
    编译的libKine.so 和 libMarvinSDK.so 供编译机器下的下C++和python使用

### 2.1.3 编译c++调用的dll动态库:
    1)windows下使用MinGW编译dll动态库:
            控制SDK(contrlSDK): g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN
            运动学SDK(kinematicsSDK): g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.dll
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下C++使用

            
### 2.1.4 编译python调用的dll动态库
    1)linux下编译dll动态库:
        控制SDK(contrlSDK):  x86_64-w64-mingw32-g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -DCMPL_WIN -static -static-libgcc -static-libstdc++ -lws2_32 -lpthread -lwinmm
        运动学SDK(kinematicsSDK): g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.dll 

    2）windows下使用MinGW编译dll动态库：
            控制SDK（contrlSDK）：g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
            运动学SDK(kinematicsSDK)：g++ *.cpp -Wall -w -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下python使用
    

# 三、 案列使用编译
## 3.1 linux示例
    只用到运动学库
    g++ showcase_kinematics_all_functions.cpp -o kine -L. -lKine -Wl,-rpath=.
    ./kine
    
    控制库和运动学库同时使用
    g++ showcase_offline_movl_keepj_execution.cpp -o offline_movl_keepj -L. -lKine -lMarvinSDK -Wl,-rpath=.
    ./offline_movl_keepj

## 3.2 windows示例
    只用到运动学库
    g++ showcase_kinematics_all_functions.cpp -o kine.exe -L. -lKine
    kine.exe
    
    控制库和运动学库同时使用
    g++ showcase_offline_movl_keepj_execution.cpp -o offline_movl_keepj.exe -L. -lKine -lMarvinSDK -Wl,-rpath=.
    offline_movl_keepj.exe 

# 四. 计算showcases

## 1. 计算SDK 功能模块完整演示
            showcase_kinematics_all_functions.cpp

## 2. 计算逆解失败总结
            showcase_ik_failed_conclusion.cpp

## 3. 两条手臂同时计算
            showcase_kine_two_arms.cpp

## 5. 逆解参考基准
            showcase_ik_nsp_two_arms.cpp
            
## 6. 演示左臂离线和在线规划功能接口：
            showcase_online_and_offline_pln_all_function.cpp


## 7. 左臂关节阻抗50HZ执行离线直线规划文件：
            showcase_offline_movl_execution.cpp

## 8.左臂关节阻抗50HZ执行在线直线规划点：
            showcase_online_movla_execution.cpp

## 9.左臂关节阻抗50HZ执行约束构型的离线直线规划文件：
            showcase_offline_movl_keepj_execution.cpp

## 10. 左臂关节阻抗50HZ执行约束构型的在线直线规划点位：
            showcase_online_movl_keepja_execution.cpp


# 五. 控制showcases

## 1. 强制抱闸和强制松闸案例
            showcase_apply_brake_release_brake.cpp

## 2. 机器人进入协作释放案列
            showcase_collaborative_release.cpp

## 3. 在迪卡尔阻抗模式下,进去迪卡尔Y方向拖动,拖动并保存数据的控制案列
            showcase_drag_CartImpedance_save_data.cpp

## 4. 拖动控制案例
            showcase_drag_joint.cpp

## 5. 在关节阻抗模式下,进去关节拖动,拖动并保存数据的控制案列
            showcase_drag_JointImpedance_save_data.cpp

## 6. 获取和设置参数案列
           showcase_get_set_param_demo.cpp

## 7. 连接检查案列
            showcase_link_check.cpp

## 8. 关节位置跟随控制案列
            showcase_position_two_arms.cpp

## 9. 跑PVT轨迹并保存数据的案列
            showcase_pvt.cpp

## 10. 为笛卡尔阻抗控制案列
            torque_cart_impedance_demo.cpp

## 11. 为力控案列
            torque_force_impedance_demo.cpp

## 12. 关节阻抗控制案列
            torque_joint_impedance_demo.cpp

## 13. 在关节阻抗模式下,进去关节拖动,拖动并保存数据的控制案列
            showcase_drag_JointImpedance_save_data.cpp

## 14. 检查指令下发的延迟时间
            showcase_cmd_delay.cpp

## 15. 关节空间以规划方式下发指令，消除抖动
            showcase_pln_jointSpace_PositionMode.cpp

## 16. 笛卡尔空间以规划方式下发指令，实现直线约束
            showcase_pln_cartSpace_PositionMode.cpp
            
## 17. 关节空间以规划方式下发指令，中断规划运行
            showcase_pln_jointSpace_PositionMode_with_break.cpp

## 18. 笛卡尔空间以规划方式下发指令，中断规划运行
            showcase_pln_cartSpace_PositionMode_with_break.cpp
    
# 六、 控制SDK简明式接口案例
            showcase_new_control_sdk_usage.cpp
