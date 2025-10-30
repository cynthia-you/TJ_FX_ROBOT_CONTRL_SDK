# MARVIN_SDK说明

##更新
### 1.机器人飞车后调整手臂的示例:

    协作释放和松闸调整的方案二选一
    C++:
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/c%2B%2B_linux/apply-brake_release-brake_demo.cpp
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/c%2B%2B_linux/collaborative_release_demo.cpp

    PYTHON:
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/python/apply-brake_relase-brake_demo.py
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/python/collaborative_release.py

### 2.拖动并保存数据示例
    C++:
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/c%2B%2B_linux/drag_JointImdedance_save_data_arm_A.cpp
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/c%2B%2B_linux/drag_CartImdedance_save_data_arm_A.cpp

    PYTHON:
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/python/drag_CartImpedance_and_save_data_arm_A.py
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/demo_linux_win/python/drag_JointImpedance_and_save_data_arm_A.py
    

### 3.升级版本和参数都发布在releases下
    https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/Release

    
## 功能同步更新到MARVIN_APP
    
## 一、天机协作机器人为7自由度协作机器人

    MARVIN SDK说明：
    1. MARVIN系列机器人的SDK分为控制SDK和机器人计算SDK
    2. 控制SDK支持win/linux平台下C++/python的使用和开发（已开源SDK代码）
    3. 计算SDK支持win/linux下的C++/python的使用（开源运动学SDK代码:正解,逆解,逆解零空间,雅可比矩阵,直线规划movL,工具负载的动力学辨识. 动力学计算接口及浮动机座接口请商询）
    4. 我司linux下仅有x_86架构机器开发和测试，特殊架构请编译测试
    5. 提供ubuntu-x_86/Windows 上位机控制软件APP(开源软件代码)

    特别说明：为了您更流畅操控我们的机器人，请您务必先查阅文档和案列，使用操作APP后再根据您的控制需求开发业务和生产脚本。


## 二、机器人控制SDK文档：
    c++_doc_contrl.md
    python_doc_contrl.md
    文档内含DEMO说明

## 三、机器人计算SDK文档：
    c++_doc_kine.md
    python_doc_kine.md
    文档内含DEMO说明


## 四、编译在目标机器
    demo_linux_win下SO的动态库是ubuntu24.04 x_86  glibc2.39机器编译的,如果你设备环境相同,可跳过4.1直接使用.

### 4.1 编译
    4.1.1编译SO动态库:
    INUX设备编译:
        控制SDK:  ./contrlSDK/makefile 生成libMarvinSDK.so
        运动学SDK: ./kinematicsSDK/makefile 生成libKine.so

    4.1.2编译DLL动态库:
    1)在WINDOWS下使用MINGW编译:
            控制SDK:  g++ MarvinSDK.cpp Robot.cpp ACB.cpp FXDG.cpp PointSet.cpp FileOP.cpp FilePortal.cpp Parser.cpp TCPAgent.cpp TCPFileClient.cpp  -Wall -w -O2 -fPIC -shared -o libMarvinSDK.dll
            运动学SDK: g++ *.cpp *.c -Wall -w -O2 -fPIC -shared -o libKine.dll    
            WINDOWS下C++使用


    2)LINUX下编译神DLL动态库:
        控制SDK: x86_64-w64-mingw32-g++ MarvinSDK.cpp Robot.cpp FXDG.cpp PointSet.cpp FileOP.cpp FilePortal.cpp Parser.cpp TCPAgent.cpp TCPFileClient.cpp -Wall -O2 -shared -o libMarvinSDK.dll \
                -DBUILDING_DLL \
                -static -static-libgcc -static-libstdc++ \
                -lws2_32 -lpthread \
                -lwinmm

                该指令生成的DLL PYTHON可调用, 但WINDOWS下C++使用不可

        运动学SDK: g++ *.cpp *.c -Wall -w -O2 -fPIC -shared -o libKine.dll  

### 4.2 使用
    LINUX:
        C++: 
            ./demo_linux_win/c++_linux/API_USAGE_KINEMATICS.txt
            ./demo_linux_win/c++_linux/API_USAGE_MarvinSDK.txt

        PYTHON 代码跨平台, 参考python_doc_contrl.md 和python_doc_kine.md 内的DENO

    WINDOWS:

            C++: 
            ./demo_linux_win/c++_win/API_USAGE_KINEMATICS.txt
            ./demo_linux_win/c++_win/API_USAGE_MarvinSDK.txt

        PYTHON 代码跨平台, 参考python_doc_contrl.md 和python_doc_kine.md 内的DENO


## 五、注意事项
    1.机器人连接通信，通信成功不代表数据已经开始发送和接受。只有在控制器接收到发送数据之后才会向上位机开始1000HZ的周期性状态数据发送。

    2.不可将软件和SDK混用，不可将软件和SDK混用，防止端口占用，收发数据失败。

    3.使用前设置网口网段和控制器在同一网段。

    4.机器人释放后，将失去对机器人的连接和控制，需要重新连接机器人

    5.我们的机器有伺服驱动器和控制器两部分，建议您将两个电源连在一个插排上，方便同时上下电和重启， 重启后有30-60秒的热机时间，请等待再操控机器人，以免伺服不响应。

    6.机器人使用结束必须在代码或者软件释放机器人(代码接口:release, 软件断开机器人按钮或者关闭软件均会释放)，以免在一个进程中，未释放，其他进程连接订阅不生效。

    7.在控制SDKc++接口中后缀_A或_B表示， _A 为左臂 _B 为右臂；如果您这只有一条臂则为_A左臂

    8.请常用清错：连接机器人小睡半秒后，应清错；获取错误码不为0时，应清错；订阅回来的机器人当前状态有错时候，应清错

    9.末端模组（485/can）的控制：务必使用末端模组供应商提供的说明书和测试软件，测试号控制指令以后再使用我司提供的SDK下发控制协议指令。



## 六、主要问题和解决
    1 连接相关
    甲方：“诶，我怎么ping不通啊”
    乙方：“请看看网线插上了吗” “有无其他设备和进程占用了” “设置成和机器人控制器同一网段的静态IP了吗”

    2 订阅相关
    甲方：“你们机器人订阅接口使用了怎么订阅不到数据，全是0？”
    乙方：“订阅前要连接机器人，小睡半秒可以实时订阅” “是否有其他进程如ROS在占用订阅进程” “防火墙是否关闭”

    3 多次回调
    甲方：“我一直CALLBACK怎么不奏效，只有第一次能动作”
    乙方：“连接和释放机器人不需要一直回调，高频伺服响应来不及，会报错。运动的指令可以低于1KHz频率发送”

    4 运动信息判断
    甲方：“我怎么通过代码判定你们机器人是否走到我指定的点位”
    乙方：“C++代码：订阅数据接口，通过订阅数据结构体里的’m_FB_Joint_Pos‘可判断是否到位，或者机器人低速标志’m_LowSpdFlag‘判定，
        当各个关节速度都小于0.5度/秒时，m_LowSpdFlag=1    ”

        “python代码：通过订阅数据结构体里的sub_data["outputs"][0]["fb_joint_pos"]可判断是否到位，
    或者机器人低速标志sub_data["outputs"][0]["low_speed_flag"]判定，当各个关节速度都小于0.5度/秒时，low_speed_flag=1”

    5 机器人状态和错误判定
    c++: 订阅数据’m_CurState‘的值可以看到当前伺服状态：
        0,             //////// 下伺服
        1,			//////// 位置跟随
        2,				//////// PVT
        3,				//////// 扭矩
        4,              ////////协作释放

        100, //报错了，清错
        ARM_STATE_TRANS_TO_POSITION = 101, //正常，切换瞬间
        ARM_STATE_TRANS_TO_PVT = 102,//正常，切换瞬间
        ARM_STATE_TRANS_TO_TORQ = 103,//正常，切换瞬间
        ARM_STATE_TRANS_TO_TORQ = 104,//正常，切换瞬间

        订阅数据’m_ERRCode‘是7个长度的double, 十进制，
        需要转换为16进制，对照伺服报错的excel看啥错
        软件已经转了16进制，C++代码接口出来的是原始数据。

    python：订阅数据a_state=sub_data["states"][0]["cur_state"]的值可以看到当前伺服状态：
        0,             //////// 下伺服
        1,			//////// 位置跟随
        2,				//////// PVT
        3,				//////// 扭矩

        ARM_STATE_ERROR = 100, //报错了，清错
        ARM_STATE_TRANS_TO_POSITION = 101, //正常，切换瞬间
        ARM_STATE_TRANS_TO_PVT = 102,//正常，切换瞬间
        ARM_STATE_TRANS_TO_TORQ = 103,//正常，切换瞬间

        获取错误用error_codes=get_servo_error_code('A')
        对照伺服报错的excel看啥错
        软件和python已经转了16进制，C++代码接口出来的是原始数据。

    6 急停后指令不响应
    急停后是自动下伺服的，需要清错再重新上伺服状态

    7 末端夹爪通信
    目前仅支持modbus485通信和CAN/CANFD。
    ！！！请不要直接把demo的指令直接发给末端夹爪或者灵巧手，协议不一致可能导致模组死机，务必使用末端模组供应商提供的说明书和测试软件，明确控制指令以后发送。
    需要注意：
        发送HEX数据到CAN
        注意看控制模组提供的指令协议：
            32位 CANID 如果为0x01, 按HEX发送为：01 00 00 00 00
            64位 CANID 如果为0x01, 按HEX发送为：01 00

## 📄 许可证

本项目基于 Apache License 2.0 许可证开源。详见 [LICENSE](LICENSE) 文件。
