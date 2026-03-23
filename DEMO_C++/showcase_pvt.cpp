#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#ifdef _WIN32
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif
// '''#################################################################
// 该DEMO 为跑PVT轨迹并保存数据的案列

// 使用逻辑
//      初始化订阅数据的结构体
//      查验连接是否成功,失败程序直接退出
//      设置PVT模式
//      订阅查看设置是否成功
//      设置PVT 轨迹本机路径 和PVT号
//      机器人运动前开始设置保存数据并开始采集数据
//      设置运行的PVT号并立即执行PVT轨迹
//      保存采集数据
//      任务完成,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################

int main()
{
      // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    //检查伺服和手臂是否有错，有错误清错
    //订阅最新数据获取机械臂的错误和状态，有错误清错
    OnGetBuf(&dcss);
    int arm_error_a=dcss.m_State[0].m_ERRCode;
    int arm_error_b=dcss.m_State[1].m_ERRCode;
    int arm_state_a=dcss.m_State[0].m_CurState;
    int arm_state_b=dcss.m_State[1].m_CurState;
   if (arm_error_a!=0 || arm_state_a==100)
    {
        std::cout << "arm A: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (arm_error_b!=0 || arm_state_b==100)
    {
        std::cout << "arm B: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    } 

    //获取伺服错误，有错误清错
    long ErrCode_A[7]={};
    long ErrCode_B[7]={};
    OnGetServoErr_A(ErrCode_A);
    OnGetServoErr_B(ErrCode_B);
    bool allZero_a = true;
    bool allZero_b = true;
    for (int i = 0; i < 7; ++i) 
    {
        if (ErrCode_A[i] != 0) {
            allZero_a = false;
            break;
        }
    }
    for (int i = 0; i < 7; ++i) 
    {
        if (ErrCode_B[i] != 0) {
            allZero_b = false;
            break;
        }
    }
    if (allZero_a)
    {
        std::cout << "arm A: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    } 
    if (allZero_b)
    {
        std::cout << "arm B: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }
    
    //通过确认freame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）
    int motion_tag = 0;
    int frame_update = 0;

    for (int i = 0; i < 5; i++) {
        OnGetBuf(&dcss);
        std::cout << "connect frames:" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[0].m_OutFrameSerial) {
            motion_tag++;
            frame_update = dcss.m_Out[0].m_OutFrameSerial;
        }
        SLEEP(1);
    }
    if (motion_tag > 0) {
        std::cout << "success:robot connected\n" << std::endl;
    } else {
        std::cerr << "failed:robot connection failed\n"<< std::endl;
        OnRelease();
        return -1;
    }

    //控制日志开
    OnLogOn();
	OnLocalLogOn();


    //设置PVT模式
    OnClearSet();
    OnSetTargetState_A(2) ; //3:torque mode; 1:position mode；2：pvt
    OnSetSend();
    SLEEP(200);


    //选择PVT轨迹文件和设置PVT号
    char path[] = "LoadData_ccs_right/LoadData/IdenTraj/LoadIdenTraj_MarvinCCS_Left.fmv"; //改成你的绝对路径
    long serial=27;
    bool re=false;
    re=OnSendPVT_A(path,serial);
    printf("send pvt return =%d\n",re);
    SLEEP(200);

    //设置保存数据的n参数并开始采集数据
                    // targetNum采集列数 （1-35列）
                    // targetID[35] 对应采集数据ID序号  
                    //           左臂序号：
                    //               0-6  	左臂关节位置 
                    //               10-16 	左臂关节速度
                    //               20-26   左臂外编位置
                    //               30-36   左臂关节指令位置
                    //               40-46	左臂关节电流（千分比）
                    //               50-56   左臂关节传感器扭矩NM
                    //               60-66	左臂摩擦力估计值
                    //               70-76	左臂摩檫力速度估计值
                    //               80-85   左臂关节外力估计值
                    //               90-95	左臂末端点外力估计值
                    //           右臂对应 + 100
                    // recordNum  采集行数 ，小于1000会采集1000行，设置大于一百万行会采集一百万行

    long targetNum=7;
    long targetID[35]={0,1,2,3,4,5,6,
        0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,
        0,0,0,0,0,0,0};
    long recordNum=1000;
    OnClearSet();
    OnStartGather(targetNum, targetID, recordNum);
    OnSetSend();
    SLEEP(200);


    //执行指定的PVT号
    int id=27;
    OnClearSet();
    OnSetPVT_A(id);
    OnSetSend();
    SLEEP(10000);//模拟执行时长


    //保存数据为TXT
    char save_path[]="aaa.txt"; //改成你的绝对路径
    OnSaveGatherData(save_path);
    SLEEP(1000);

    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}
