# 本项目为天机MARVIN系列机器人的开源仓库

# 本文档包含:SDK更新， 控制器版本更新, APP更新， 简要说明， 编译方法， 使用注意， 机器人报错及处理措施

# ATTENTION
     1.  请先熟练使用MARVIN_APP 或者https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/releases/ 下各个版本里的FxStation.exe， 操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
     2.  DEMO_C++ 和 DEMO_PYTHON 为SDK接口在C++和PYTHON下的使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
         这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
             比如:
                 a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                 b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                 c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                 d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。
                 
## 一、 SDK更新

### 1.1 PYTHON
     1. 更新了计算的SDK:https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/SDK_PYTHON/fx_kine.py
     2. 更新了SDK的DEMO:https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/master/DEMO_PYTHON/readme.md

### 1.2 代码获取控制器版本号
     C++:
          char paraName[30]="VERSION";
          long retValue=0;
          OnGetIntPara(paraName,&retValue);
          printf("CONTRL VERSION: %ld\n", retValue);

     PYTHON:
          ret,version=robot.get_param('int','VERSION')
          print(f'controller version:{version}')

     显示为1003xx, 如100335, 即大版本号:1003,子版本35



## 二、 控制器版本更新

     1003_37版本添加功能::
     1. 新增任意状态下的轴外力检测,该轴外力可用于计算末端所受外力.

     
    1003_35版本添加功能:
    1 增加内外编码器检测功能
    2 修复伺服出错后所有轴全部下使能
    https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/releases/tag/marvin_tool_1003_35

    
    1003_34版本添加功能:
    1 内编外编清0，编码器清错。
    2 支持仅位置模式控制 增加了参数R.A0.BASIC.CtrlType和R.A1.BASIC.CtrlType。0表示控制模式都开放，1表示只有位置控制 (修改在机器人配置文件 *.ini)
    
    更能已同步更新到MARVIN_APP和FX-STATION

    1003_34地址：
        https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/releases/tag/marvin_tool_1003_34
        
### 2.1 更新功能同步到MARVIN_APP

### 2.2 机器人电机内外编清零和内编清错示例
    控制器需要升级到1003_34版本
       
### 2.3 升级版本和参数都发布在releases下
    https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/releases

## 三、APP更新

### 3.1 添加FXSTAION软件源码
     https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/%E4%B8%8A%E4%BD%8D%E6%9C%BA%E8%BD%AF%E4%BB%B6MARVIN_APP/FxStation_1217.zip

### 3.2 末端CAN/485 更新
     1.机器人连接后点击接收按钮后开启读末端返回报文线程
     2.可加入多个协议指令在列表
     3.可删除下拉列表中的协议指令
     MARVIN_APP_1128

### 3.3 增加浮机参数计算功能
     MARVIN_APP_1125及以上

    
    
## 四、简要说明

    MARVIN SDK说明：
         1. MARVIN系列机器人的SDK分为控制SDK和机器人计算SDK
         2. 控制SDK支持win/linux平台下C++/python的使用和开发（已开源SDK代码）
         3. 计算SDK支持win/linux下的C++/python的使用（开源运动学SDK代码:正解,逆解,逆解零空间,雅可比矩阵,直线规划movL,工具负载的动力学辨识. 动力学计算接口及浮动机座接口请商询）
         4. 我司linux下仅有x_86架构机器开发和测试，特殊架构请编译测试
         5. 提供ubuntu-x_86/Windows 上位机控制软件APP(开源软件代码)

    特别说明：为了您更流畅操控我们的机器人，请您务必先查阅文档和案列，使用操作APP后再根据您的控制需求开发业务和生产脚本。

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


## 1.1 机器人控制SDK文档：
    c++_doc_contrl.md
    python_doc_contrl.md
    文档内含DEMO说明

## 4.2 机器人计算SDK文档：
    c++_doc_kine.md
    python_doc_kine.md
    文档内含DEMO说明


## 五、编译方法

### 5.1 编译
    5.1.1编译SO动态库:
    INUX设备编译:
        控制SDK:  ./contrlSDK/makefile 生成libMarvinSDK.so
        
        运动学SDK: ./kinematicsSDK/makefile 生成libKine.so

    5.1.2编译DLL动态库:
    1)在WINDOWS下使用MINGW编译:
            控制SDK:
                 源码controlSDK_win: g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm
                 
            运动学SDK(kinematicsSDK): g++ *.cpp *.c -Wall -w -O2 -fPIC -shared -o libKine.dll    
            
            编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下C++使用


    2)LINUX下编译神DLL动态库:
        控制SDK: x86_64-w64-mingw32-g++ MarvinSDK.cpp Robot.cpp FXDG.cpp PointSet.cpp FileOP.cpp FilePortal.cpp Parser.cpp TCPAgent.cpp TCPFileClient.cpp -Wall -O2 -shared -o libMarvinSDK.dll \
                -DBUILDING_DLL \
                -static -static-libgcc -static-libstdc++ \
                -lws2_32 -lpthread \
                -lwinmm

        运动学SDK: g++ *.cpp *.c -Wall -w -O2 -fPIC -shared -o libKine.dll  

        该指令生成的DLL PYTHON可调用, 但WINDOWS下C++使用不可

### 5.2 使用
    LINUX:
        C++: 
            ./DEMO_C++/readme.md

        PYTHON 代码跨平台, 参考DEMO_PYTHON/readme.md

    WINDOWS:

        C++: 
            ./DEMO_C++/readme.md
            双击执行自动化demo生成脚本： ./DEMO_C++/auto_complie.bat
            
        PYTHON 代码跨平台, 参考DEMO_PYTHON/readme.md


## 六、注意事项
    1.机器人连接通信，通信成功不代表数据已经开始发送和接受。只有在控制器接收到发送数据之后才会向上位机开始1000HZ的周期性状态数据发送。

    2.不可将软件和SDK混用，不可将软件和SDK混用，防止端口占用，收发数据失败。

    3.使用前设置网口网段和控制器在同一网段。

    4.机器人释放后，将失去对机器人的连接和控制，需要重新连接机器人

    5.我们的机器有伺服驱动器和控制器两部分，建议您将两个电源连在一个插排上，方便同时上下电和重启， 重启后有30-60秒的热机时间，请等待再操控机器人，以免伺服不响应。

    6.机器人使用结束必须在代码或者软件释放机器人(代码接口:release, 软件断开机器人按钮或者关闭软件均会释放)，以免在一个进程中，未释放，其他进程连接订阅不生效。

    7.在控制SDKc++接口中后缀_A或_B表示， _A 为左臂 _B 为右臂；如果您这只有一条臂则为_A左臂

    8.请常用清错：连接机器人小睡半秒后，应清错；获取错误码不为0时，应清错；订阅回来的机器人当前状态有错时候，应清错

    9.末端模组（485/can）的控制：务必使用末端模组供应商提供的说明书和测试软件，测试号控制指令以后再使用我司提供的SDK下发控制协议指令。



## 七、主要问题和解决
### 7.1 marvin sdk&app 问题与解决
     【腾讯文档】MARVIN SDK&APP 问题收集与解决
     https://docs.qq.com/sheet/DUmdJck1zQkJVT0tw

### 7.2其他常见问题
    1 连接相关
    Q：“诶，我怎么ping不通啊”
    A：“请看看网线插上了吗” “有无其他设备和进程占用了” “设置成和机器人控制器同一网段的静态IP了吗”

    2 订阅相关
    Q：“你们机器人订阅接口使用了怎么订阅不到数据，全是0？”
    A：“订阅前要连接机器人，小睡半秒可以实时订阅” “是否有其他进程如ROS在占用订阅进程” “防火墙是否关闭”

    3 多次回调
    Q：“我一直CALLBACK怎么不奏效，只有第一次能动作”
    A：“连接和释放机器人不需要一直回调，高频伺服响应来不及，会报错。运动的指令可以低于1KHz频率发送”

    4 运动信息判断
    Q：“我怎么通过代码判定你们机器人是否走到我指定的点位”
    A：“C++代码：订阅数据接口，通过订阅数据结构体里的’m_FB_Joint_Pos‘可判断是否到位，或者机器人低速标志’m_LowSpdFlag‘判定，
        当各个关节速度都小于0.5度/秒时，m_LowSpdFlag=1    ”

        “python代码：通过订阅数据结构体里的sub_data["outputs"][0]["fb_joint_pos"]可判断是否到位，
    或者机器人低速标志sub_data["outputs"][0]["low_speed_flag"]判定，当各个关节速度都小于0.5度/秒时，low_speed_flag=1”

    5 机器人状态和错误判定
    c++: 订阅数据’m_CurState‘的值(int)可以看到当前机械臂状态：
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

        订阅数据’m_ERRCode‘的值(int)可以看到当前机械臂的错误状态：
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
             ARM_DYNA_FLOAT_NO_GYRO = 14,//"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开"


    python：订阅数据a_state=sub_data["states"][0]["cur_state"]的值可以看到当前伺服状态：
        0,             //////// 下伺服
        1,			//////// 位置跟随
        2,				//////// PVT
        3,				//////// 扭矩

        ARM_STATE_ERROR = 100, //报错了，清错
        ARM_STATE_TRANS_TO_POSITION = 101, //正常，切换瞬间
        ARM_STATE_TRANS_TO_PVT = 102,//正常，切换瞬间
        ARM_STATE_TRANS_TO_TORQ = 103,//正常，切换瞬间



        订阅数据a_state=sub_data["states"][0]["err_code"]的值可以看到当前机械臂的错误状态：
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
             ARM_DYNA_FLOAT_NO_GYRO = 14,//"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开"

        获取错误用error_codes=get_servo_error_code('A')
        对照伺服报错的PDF看啥错
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
