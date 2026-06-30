# 本项目为天机MARVIN系列机器人的开源仓库

# 本文档包含:一、SDK简要说明，二、编译方法，三、SDK更新，四、控制器版本更新，五、APP更新，六、使用注意，七、机器人报错及处理措施

# ATTENTION
    1.  请先熟练使用MarvinPlatform软件。操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
    2.  DEMO_C++/ 和 DEMO_PYTHON/ 下为接口使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
        这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
            比如:
                a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。

## 一、SDK简要说明

    MARVIN SDK说明：
         1. MARVIN系列机器人的SDK分为控制SDK和机器人计算SDK
         2. 控制SDK支持win/linux平台下C++/python的使用和开发
         3. 计算SDK支持win/linux下的C++/python的使用（开源运动学SDK代码:正解,逆解,逆解零空间,雅可比矩阵,直线规划movL,工具负载的动力学辨识. 动力学计算接口及浮动机座接口请商询）
         4. 我司linux下仅有x_86架构机器开发和测试，特殊架构请编译测试
         5. 提供ubuntu-x_86/Windows 上位机控制软件APP(开源软件代码)

    特别说明：
            1.为了您更流畅操控我们的机器人，请您务必先查阅文档和案列，
            2.使用操作上位机软件后再根据您的控制需求开发业务和生产脚本。

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
                        20-26   左臂外部编码器位置
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


## 机器人运动控制模式说明
    MarvinProx机器人（人形臂）的运动控制支持多种运控模式：
        -位置模式
        -扭矩/阻抗模式，细分为关节阻抗，笛卡尔阻抗，力控阻抗三种
        -PD模式
        -协作释放（零力拖动模式）
        -轨迹复现模式


### 位置模式（Position Mode）
    基于伺服闭环控制，机器人各关节按照预设的目标位置、速度和加速度轨迹运动，实现高精度的点到点或连续路径跟踪。该模式适用于对轨迹重复性要求高的场景（如搬运、涂胶、焊接），不对外部接触力做主动调节，位置偏差主要由刚度决定。

    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
//设置速度加速度百分比
OnClearSet();
OnSetJointLmt_A(10, 10) ;
OnSetJointLmt_B(10, 10) ;
OnSetSend();
SLEEP(200);
//切换为位置模式
OnClearSet();
OnSetTargetState_A(1) ;
OnSetTargetState_B(1) ;
OnSetSend();
SLEEP(1000);
```
    python(省略连接仅展示切换代码)：
```python
'''设置速度加速度百分比'''
robot.clear_set()
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.set_vel_acc(arm='B',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.2)
'''切换为位置模式'''
robot.clear_set()
robot.set_state(arm='A',state=1)
robot.set_state(arm='B',state=1)
robot.send_cmd()
time.sleep(1)
```


### 关节阻抗模式（Joint Impedance Mode）
    在关节空间内建立力矩与位置偏差的动态关系，表现为“弹簧-阻尼”特性。用户可分别设置每个关节的刚度（范围 0~22， 单位N*m/rad）和阻尼（>0，单位N*m/（rad/s)），刚度越高关节“越硬”，阻尼越大，物体振幅减小越快，但对力、位移的响应迟缓，运动时感觉阻力大，有粘滞感； 阻尼越小，减震效果减弱，但运动阻力小，更流畅，停止到位置时有余震感。该模式适用于需要关节级柔顺性的装配、抛光和避障任务，可吸收冲击并适应不规则表面。
    
    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
// 设置关节阻抗关键参数
double k[7] = {12, 12, 12, 10, 9, 9, 7};
double d[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1};
OnClearSet();
OnSetJointLmt_A(10, 10);
OnSetJointKD_A(k, d);
OnSetJointLmt_B(10, 10);
OnSetJointKD_B(k, d);
OnSetSend();
SLEEP(200);
// 切换到关节阻抗控制模式
OnClearSet();
OnSetTargetState_A(3); 
OnSetImpType_A(1);
OnSetTargetState_B(3); 
OnSetImpType_B(1);
OnSetSend();
SLEEP(1000);
```
    python(省略连接仅展示切换代码)：
```python
'''设置关节阻抗关键参数'''
robot.clear_set()
robot.set_joint_kd_params(arm='A',K=[12, 12, 12, 10, 9, 9, 7], D=[0.3,0.3,0.3,0.2,0.2,0.2,0.2]）
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.set_joint_kd_params(arm='B',K=[12, 12, 12, 10, 9, 9, 7], D=[0.3,0.3,0.3,0.2,0.2,0.2,0.2]）
robot.set_vel_acc(arm='B',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.2)
'''切换关节阻抗模式'''
robot.clear_set()
robot.set_state(arm='A',state=3)
robot.set_impedance_type(arm='A',type=1) 
robot.set_state(arm='B',state=3)
robot.set_impedance_type(arm='B',type=1) 
robot.send_cmd()
time.sleep(1)
```

### 笛卡尔阻抗模式（Cartesian Impedance Mode）
    在末端笛卡尔空间（X/Y/Z方向及旋转轴）构建柔顺控制模型，使末端对外力呈现可调的刚度和阻尼特性。平移刚度范围 0~1200 N*m ，旋转 N*m/rad，；阻尼值需 >0，单位N*m/（rad/s)。该模式适用于末端与外部环境交互（如打磨、去毛刺、力控装配），可在保持轨迹精度的同时主动顺应外力，提高接触安全性。
     
    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
// 设置笛卡尔阻抗关键参数
double k[7] = {10000, 10000, 10000, 600, 600, 600, 20};
double d[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1};
OnClearSet();
OnSetJointLmt_A(10, 10);
OnSetJointKD_A(k, d);
OnSetJointLmt_B(10, 10);
OnSetJointKD_B(k, d);
OnSetSend();
SLEEP(200);
// 切换到笛卡尔控制模式
OnClearSet();
OnSetTargetState_A(3); 
OnSetImpType_A(2);
OnSetTargetState_B(3); 
OnSetImpType_B(2);
OnSetSend();
SLEEP(1000);
```
    python(省略连接仅展示切换代码)：
```python
'''设置笛卡尔阻抗关键参数'''
robot.clear_set()
robot.set_joint_kd_params(arm='A',K=[10000, 10000, 10000, 600, 600, 600, 20], D=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1]）
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.set_joint_kd_params(arm='B',K=[10000, 10000, 10000, 600, 600, 600, 20], D=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1]）
robot.set_vel_acc(arm='B',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.2)

'''切换笛卡尔阻抗模式'''
robot.clear_set()
robot.set_state(arm='A',state=3)
robot.set_impedance_type(arm='A',type=2) 
robot.set_state(arm='B',state=3)
robot.set_impedance_type(arm='B',type=2) 
robot.send_cmd()
time.sleep(1)
```

### 力控阻抗模式（Force-controlled Impedance Mode）
    在指定笛卡尔方向（X、Y、Z 三轴）上直接以期望接触力为目标进行闭环控制，同时保留阻抗柔顺特性。力控制范围 0~50 N，力作用距离（即允许的位移偏差窗口）为 -50 mm ~ +50 mm。该模式适用于恒定力跟踪应用，通过调整参数适应接触表面起伏，确保接触力稳定可控。
     
    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
// 设置末端力控参数：Z方向力控
int fcType=0;// 基于基座的力控
double fcCtrlPara[7] = {0.0};
double fxDir[6] = {0, 0, 1, 0, 0, 0};
double fcAdjLmt = 50;
double force = 10;
OnClearSet();
OnSetForceCtrPara_A(0, fxDir, fcCtrlPara, fcAdjLmt);
OnSetForceCmd_A(force);
OnSetSend();
SLEEP(200);
// 切换力控模式
OnClearSet();
OnSetTargetState_A(3); 
OnSetImpType_A(3);
OnSetTargetState_B(3); 
OnSetImpType_B(3);
OnSetSend();
SLEEP(1000);

```
    python(省略连接仅展示切换代码)：
```python
'''设置力控参数'''
robot.clear_set()
# 设置是在Y轴方向有5厘米的调节范围
robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=5.)
time.sleep(0.5)
'''切换力控模式'''
robot.clear_set()
robot.set_state(arm='A',state=3)
robot.set_impedance_type(arm='A',type=3) 
robot.set_state(arm='B',state=3)
robot.set_impedance_type(arm='B',type=3) 
robot.send_cmd()
time.sleep(1)
```
### PD前馈模式
    PD模式时一种极低延时跟踪模式兼具关节阻抗模式的柔顺性能，主要适用于遥操场景。
    用户的关节轨迹每个关节速度不能超过180度/秒
    开启条件:
        - 配置参数robot.ini文件中 JointPIDCtlType=1; 
        - 在关节阻抗模式下使用，速度加速大设置为最大，以免限制轨迹；
        - 发送轨迹前，通过FX_OnSetVelEstStep()开启前馈控制
     
    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
//切换关节阻抗，速度加速度设置为最大，设置刚度阻尼参数
//推荐三组刚度参数，按需选择使用
double k_max[7] = {20, 20, 20, 15, 8, 8, 8};//max
double k_min[7] = {2, 2, 2, 1.5, 0.8, 0.8, 0.8 };//min
double k_normal[7]={ 14, 14, 14, 10.5, 5.6, 5.6, 5.6}
double d[7] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
OnClearSet();
OnSetJointLmt_A(100,100)
OnSetJointKD_A(k_normal, d)
OnSetJointLmt_B(100, 100);
OnSetJointKD_B (k_normal, d)
OnSetTargetState_A(3)
OnSetImpType_A(1)
OnSetTargetState_B(3)
OnSetImpType_B(1)
OnSetSend();
SLEEP(200);
//开启PD前馈
//控制周期ControlPeriod参数范围0~20ms, 0表示不开启PD前馈。建议设置为5ms，并且发送的轨迹速度经可能不要超过最大速度限制。
int ControlPeriod = 5;
OnClearSet();
FX_OnSetVelEstStep("A"，ControlPeriod);
FX_OnSetVelEstStep("B"，ControlPeriod);
OnSetSend();
SLEEP(1000);
```
    python(省略连接仅展示切换代码)：
```python
'''切换关节阻抗，速度加速度设置为最大，设置刚度阻尼参数'''
#推荐三组刚度参数，按需选择使用
k_max=[20, 20, 20, 15, 8, 8, 8]
k_min=[2, 2, 2, 1.5, 0.8, 0.8, 0.8]
k_normal=[ 14, 14, 14, 10.5, 5.6, 5.6, 5.6]
d=[0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
robot.clear_set()
robot.set_joint_kd_params(arm='A',K=k_normal, D=d）
robot.set_vel_acc(arm='A',velRatio=100, AccRatio=100)
robot.set_joint_kd_params(arm='B',K=k_normal, D=d）
robot.set_vel_acc(arm='B',velRatio=100, AccRatio=100)
robot.send_cmd()
time.sleep(0.2)
robot.clear_set()
robot.set_state(arm='A',state=3)
robot.set_impedance_type(arm='A',type=1) 
robot.set_state(arm='B',state=3)
robot.set_impedance_type(arm='B',type=1) 
robot.send_cmd()
time.sleep(1)
'''开启PD前馈
控制周期ControlPeriod参数范围0~20ms, 0表示不开启PD前馈。建议设置为5ms，并且发送的轨迹速度经可能不要超过最大速度限制。'''
ControlPeriod = 5
robot.clear_set()
robot.set_PD_vel_est_step(arm='A',step=ControlPeriod)
robot.set_PD_vel_est_step(arm='B',step=ControlPeriod)
robot.send_cmd()
time.sleep(1)
```
    
### 协作释放模式（Collaborative Release Mode）
    专为人机协作安全设计的安全响应模式。当检测到碰撞或外力超过阈值时，机器人立即停止运动并主动释放所有关节制动力矩，使各轴处于“零力漂浮”状态，从而最大限度降低碰撞冲击能量。该模式也可由操作员手动触发，用于紧急脱离或手动拖拽示教，恢复后需重新上使能方可继续运行。
     
    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
//开启协作释放模式
OnClearSet();
OnSetTargetState_A(4) ;
OnSetTargetState_B(4) ;
OnSetSend();
SLEEP(1000);
//开启后可拖拽手臂调制位置，调整结束复位
OnClearSet();
OnSetTargetState_A(0) ;
OnSetTargetState_B(0) ;
OnSetSend();
SLEEP(1000);
```
    python(省略连接仅展示切换代码)：
```python
'''开启协作释放模式'''
robot.clear_set()
robot.set_state(arm='A',state=1)
robot.set_state(arm='B',state=1)
robot.send_cmd()
time.sleep(1)
'''开启后可拖拽手臂调制位置，调整结束复位'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.set_state(arm='B',state=0)
robot.send_cmd()
time.sleep(1)
```

### 轨迹复现PVT模式（Position-Velocity-Time Replay Mode）
    该模式基于示教或离线编程记录的轨迹点（每个点包含位置、速度及时间戳信息），通过高精度插补算法在关节或笛卡尔空间内按原始时间序列复现完整运动路径。PVT模式保证了轨迹的连续性与速度平滑性，适合喷涂、打磨、点焊等需要严格遵循示教路径的重复作业。参数设置包括插补周期，速度和加速度限制，确保复现精度与动态性能。
    注意，机械臂先执行到轨迹的起始点位。
         
    开启条件：
    C/C++(省略连接仅展示切换代码)：
```c
//设置PVT模式
OnClearSet();
OnSetTargetState_A(2) ;
OnSetSend();
SLEEP(200);
//选择PVT轨迹文件和设置PVT号
char path[] = "LoadData_ccs_right/LoadData/IdenTraj/LoadIdenTraj_MarvinCCS_Left.fmv"; //改成你的绝对路径
long serial=27;
bool re=false;
re=OnSendPVT_A(path,serial);
printf("send pvt return =%d\n",re);
SLEEP(200);
//执行指定的PVT号
int id=27;
OnClearSet();
OnSetPVT_A(id);
OnSetSend();
//等待轨迹执行完毕
```
    python(省略连接仅展示切换代码)：
```python
'''设置PVT模式'''
robot.clear_set()
robot.set_state(arm='A',state=2)#PVT， 自己的速度和加速度，不受外部控制。
robot.send_cmd()
time.sleep(0.5)
'''设置PVT 轨迹本机路径 和PVT号'''
pvt_file='/LoadData_ccs_right/LoadData/IdenTraj/LoadIdenTraj_MarvinCCS_Left.fmv'
robot.send_pvt_file('A',pvt_file, 2)
time.sleep(1)
'''设置运行的PVT号'''
robot.clear_set()
robot.set_pvt_id('A',2)
robot.send_cmd()
#等待轨迹执行完毕
```



## 1.1 机器人控制SDK文档：
[C++ 控制SDK 文档](c++_doc_contrl.md)

[PYTHON 控制SDK 文档](python_doc_contrl.md)

    文档内含DEMO说明

## 1.2 机器人计算SDK文档：
[C++ 运动计算SDK 文档]( c++_doc_kine.md)

[PYTHON 运动计算SDK 文档](python_doc_kine.md)

    文档内含DEMO说明


## 二、编译方法

## 注：最新contrlSDK100343/contrlSDK  和 kinematicsSDK 代码兼容windows 和linux平台。

### 2.1 编译
    2.1.1 编译so动态库:
    linux设备编译:
        控制SDK(contrlSDK100343)，以下方法均可编译: 
			1. g++ *.cpp -Wall -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN
			2./contrlSDK100343/makefile 生成libMarvinSDK.so
        运动学SDK(kinematicsSDK)，以下方法均可编译: 
			1. g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.so -lpthread -lrt 
			2./kinematicsSDK/makefile 生成libKine.so
	编译的libKine.so 和 libMarvinSDK.so 供编译机器下的下C++和python使用

    2.1.2 编译c++调用的dll动态库:
    1)windows下使用MinGW编译dll动态库:
			控制SDK(contrlSDK100343): g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN
            运动学SDK(kinematicsSDK): g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.dll
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下C++使用

			
	2.1.3 编译python调用的dll动态库
    1)linux下编译dll动态库:
        控制SDK(contrlSDK100343):  x86_64-w64-mingw32-g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -DCMPL_WIN -static -static-libgcc -static-libstdc++ -lws2_32 -lpthread -lwinmm
        运动学SDK(kinematicsSDK): g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.dll 

	2）windows下使用MinGW编译dll动态库：
			控制SDK（contrlSDK100343）：g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
			运动学SDK(kinematicsSDK)：g++ *.cpp -Wall -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下python使用

### 2.2 自动化编译动态链库
    以contrlSDK100343和运动学库为例：
	1）linux下可使用marvinSDK_ubuntu_100343.sh 自动编译替换.so
			# 赋予脚本执行权限
            chmod +xmarvinSDK_ubuntu_100343.sh
			# 运行自动化编译脚本
			./marvinSDK_ubuntu_100343.sh

	2）windows下可使用marvinSDK_windows_100343.bat 自动编译替换.dll
			# 直接运行批处理脚本
			./marvinSDK_windows_100343.bat
        
### 2.3 使用案列
    LINUX:
        C++: 
            ./DEMO_C++/readme.md
        PYTHON 代码跨平台, 参考DEMO_PYTHON/readme.md

    WINDOWS:

        C++: 
            ./DEMO_C++/readme.md
        PYTHON 代码跨平台, 参考DEMO_PYTHON/readme.md


### 2.4 不使用动态库，源码调用

    使用contrlSDK100343为例, 假设调用代码文件main.cpp在和 contrlSDK100343 同级目录文件夹workspace下， 
    目录树为：
    ...
    |---contrlSDK100343
    |---workspace
        |--- main.cpp
    ...


    则编译指令为：
    1）windows下编译指令：
    g++ -Wall main.cpp ../contrlSDK100343/*.cpp -I../contrlSDK100343 -o main.exe -lws2_32 -lwinmm -DCMPL_WIN

    2）linux下编译指令：
    g++ -Wall main.cpp ../contrlSDK100343/*.cpp -I../contrlSDK100343 -o main -lpthread -lrt -DCMPL_LIN

    编译完成后，会生成
    ...
    |---contrlSDK100343
    |---workspace
        |--- main.cpp
        |--- main.exe or main
    ...


                 
## 三、 SDK更新
## contrlSDK不再维护，持续维护开发100343版本。

### contrlSDK100343版本
    1 contrlSDK100343版本必须匹配底层控制系统100343及以上版本，不支持控制系统100343以前的版本（如100341）。
    2 contrlSDK100343版本主要修改了通讯协议，增加校验功能，防止多线程调用接口导致发送指令混乱。
    3 本分支原有DEMO以及用户已经存在的脚本无需修改，只需将SDK100342重新编译替换原有动态库即可，编译指令不变。
    4 特别注意控制系统100343版本不向下兼容，只能使用100343版本SDK。

## 3.1 案例更新
### 关节力矩转末端六维力
[PYTHON案例](DEMO_PYTHON/showcase_jointsTorque2EefTorque.py)

## 3.2 控制SDK
### 控制SDK新增简明接口
# 为了更简明地使用控制SDK，我们特别提供了简明式接口，
[原SDK接口介绍](c++_doc_contrl.md#L118)

[简明式接口介绍](c++_doc_contrl.md#L840)

[控制SDK MarvinSDK.h](contrlSDK/MarvinSDK.h)

[简明式控制案例C++](DEMO_C++/showcase_new_control_sdk_usage.cpp)
[简明式控制案例PYTHON](DEMO_PYTHON/showcases_new_control_sdk.py)
    
### 以规划方式下发关节指令消除抖动：
    //关节空间PLN方式发送指令
    FX_DLL_EXPORT bool OnInitPlnLmt(char * path);
	FX_DLL_EXPORT bool OnSetPlnJoint_A(double start_joints[7], double stop_joints[7],double vel_ratio,double acc_ratio);
	FX_DLL_EXPORT bool OnSetPlnJoint_B(double start_joints[7], double stop_joints[7],double vel_ratio,double acc_ratio);

### 以规划方式下发指令实现走直线
    // 笛卡尔空间PLN方式发送指令
	FX_DLL_EXPORT void* FX_CPointSet_Create();
	FX_DLL_EXPORT void FX_CPointSet_Destroy(void* pset);
	FX_DLL_EXPORT bool OnSetPlnCart_A(void* pset);
	FX_DLL_EXPORT bool OnSetPlnCart_B(void* pset);

### 规划中断运行
	FX_DLL_EXPORT bool OnStopPlnJoint_A();
	FX_DLL_EXPORT bool OnStopPlnJoint_B();

### 设置末端力控类型和笛卡尔方向的旋转
	//设置左臂力控类型fcType=1。 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
	FX_DLL_EXPORT bool OnSetEefRot_A(int fcType, double CartCtrlPara[7]);
	//设置右臂力控类型fcType=1。 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
	FX_DLL_EXPORT bool OnSetEefRot_B(int fcType, double CartCtrlPara[7]);

### 指定关节伺服软复位
	// 左臂指定关节伺服软复位
	FX_DLL_EXPORT void OnServoReset_A(int axis);
	// 右臂指定关节伺服软复位
	FX_DLL_EXPORT void OnServoReset_B(int axis);

## 3.3运动计算SDK
### 更新在线规划功能

     C++接口：
        FX_BOOL  FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32L Freq, FX_CHAR* OutPutPath);
        FX_BOOL  FX_Robot_PLN_MOVL_KeepJ(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32L Freq, FX_CHAR* OutPutPath);
        FX_BOOL FX_Robot_PLN_MOVLA(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC,Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32L Freq, CPointSet* ret_pset);
        FX_BOOL  FX_Robot_PLN_MOVL_KeepJA(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints,FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32L Freq, CPointSet* ret_pset);

     c++ demo: 
          1.演示左臂离线和在线规划功能接口：showcase_online_and_offline_pln_all_function.cpp
          2.左臂关节阻抗50HZ执行离线直线规划文件：showcase_offline_movl_execution.cpp
          3.左臂关节阻抗50HZ执行在线直线规划点：showcase_online_movla_execution.cpp
          4.臂关节阻抗50HZ执行约束构型的离线直线规划文件：showcase_offline_movl_keepj_execution.cpp
          5.左臂关节阻抗50HZ执行约束构型的在线直线规划点位：showcase_online_movl_keepja_execution.cpp

     PY接口：
        直线插值规划
       - movL(start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float, freq_hz:int, save_path)
    
        直线插值规划，约束起始结束关节构型
        - movL_KeepJ(start_joints:list, end_joints:list,vel:float,acc: float,freq_hz:int, save_path)
    
          在线直线插值规划
        - movLA(start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float,freq_hz:int )
    
          在线直线插值规划，约束起始结束关节构型
        - movL_KeepJA(start_joints:list, end_joints:list,vel:float,acc: float,freq_hz:int)

       py demo:
            showcase_online_pln_movl.py
            showcase_online_pln_movl_keepj.py
            showcase_online_pln_movl_with_specific_rot.py
          

### 代码获取控制器版本号
     C++:
          char paraName[30]="VERSION";
          long retValue=0;
          OnGetIntPara(paraName,&retValue);
          printf("CONTRL VERSION: %ld\n", retValue);

     PYTHON:
          ret,version=robot.get_param('int','VERSION')
          print(f'controller version:{version}')

     显示为1003xx, 如100335, 即大版本号:1003,子版本35



## 四、 控制器版本更新

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
        

### 4.1 机器人电机内外编清零和内编清错示例
    控制器需要升级到1003_34版本
       
### 4.2 升级版本和参数都发布在releases下
    https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/releases


## 五、APP更新
[MarvinPlatform源码](MarvinPlatform_EN/ui_EN.py)
[MarvinPlatform windows 上位机](MarvinPlatform_EN\MarvinPlatform_win_100343.exe)
[MarvinPlatform linux 上位机](MarvinPlatform_EN\MarvinPlatform_linux_100343)
[MarvinPlatform使用说明](MarvinPlatform_EN/天机Marvin系列MarvinPlatform软件使用说明2601.pptx)


## 六、注意事项
    1.机器人连接通信，通信成功不代表数据已经开始发送和接受。只有在控制器接收到发送数据之后才会向上位机开始1000HZ的周期性状态数据发送。

    2.不可将软件和SDK混用，不可将软件和SDK混用，防止端口占用，收发数据失败。

    3.使用前设置网口网段和控制器在同一网段。

    4.机器人释放后，将失去对机器人的连接和控制，需要重新连接机器人

    5.我们的机器有伺服驱动器和控制器两部分，建议您将两个电源连在一个插排上，方便同时上下电和重启， 重启后有30-60秒的热机时间，请等待再操控机器人，以免伺服不响应。

    6.机器人使用结束必须在代码或者软件释放机器人(代码接口:release, 软件断开机器人按钮或者关闭软件均会释放)，以免在一个进程中，未释放，其他进程连接订阅不生效。

    7.在控制SDKc++接口中后缀_A或_B表示， _A 为左臂 _B 为右臂；如果您这只有一条臂则为_A左臂

    8.当订阅到机器人状态值为100 或者订阅到机器人错误 伺服发生错误时， 请清错

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
             ARM_ERR_BusPhysicAbnoraml = 1, //"总线拓扑异常" 	EtherCAT通讯处于断开状态等错误状态
             ARM_ERR_ServoError = 2,//"伺服故障"  				1) 某个轴处于故障状态, 2) 轴参数配置错误, 3) 轴通讯错误
             ARM_ERR_InvalidPVT = 3,//"PVT异常"  				1) PVT模式内部读取数据错误，长度不符, 2) 位置模式时linux系统调度导致PSI进程和SI进程数据交互错误
             ARM_ERR_RequestPositionMode = 4,//"请求进位置失败" 	1) 伺服初始化状态错误，2) 伺服状态正在切换，3) 编码器状态错误，4) 伺服反馈状态切换失败，5) 处于急停状态，6) 100341版本以前原因同6
             ARM_ERR_PositionModeOK = 5,//"进位置失败" 			1) 伺服反馈切换运行模式失败，2) 电机状态错误，3) 控制器系统内存状态错误，4) 控制器内部手臂数量设置错误
             ARM_ERR_RequestSensorMode = 6,//"请求进扭矩失败" 	1) 未设置工具动力学参数，2) 手臂与外界环境处于硬接触状态，3) 其他原因同4、5
             ARM_ERR_SensorModeOK = 7,//"进扭矩失败" 			原因同4、5
             ARM_ERR_RequestEnableServo = 8,//"请求上伺服失败" 	原因同4、5
             ARM_ERR_EnableServoOK = 9,//"上伺服失败" 			原因同4、5

             ARM_ERR_RequestDisableServo = 10, //"请求下伺服失败 原因同4、5
             ARM_ERR_DisableServoOK = 11, //"下伺服失败" 		原因同4、5
             ARM_ERR_InvalidSubState = 12, //"内部错" 			1) 操作系统内存错误，调度错误，2) 变量值计算错误，3) 某些内存指针为空
             ARM_ERR_Emcy = 13, //"急停"
             ARM_DYNA_FLOAT_NO_GYRO = 14,//"配置文件选择了浮动基座选项，但是实际没有IMU硬件接入控制器"
             ARM_ERR_PdoAbnormal = 15, //"PDO工作不正常"


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
            32位 CANID 如果为0x01, 按HEX发送为：01 00 00 00
            64位 CANID 如果为0x01, 按HEX发送为：01 00

## 📄 许可证

本项目基于 Apache License 2.0 许可证开源。详见 [LICENSE](LICENSE) 文件。
