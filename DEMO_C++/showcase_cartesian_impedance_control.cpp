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
//'''#################################################################
// 该DEMO 为在笛卡尔阻抗模式下运动控制案列
//
// 使用逻辑
//    初始化订阅数据的结构体
//    初始化机器人接口
//    查验连接是否成功,失败程序直接退出
//    开启日志以便检查
//    设置笛卡尔控制参数并进笛卡尔阻抗模式
//    订阅查看笛卡尔阻抗模式和参数是否设置成功
//    运动到指定位置
//    任务完成，下使能
//    释放内存使别的程序或者用户可以连接机器人
//'''################################################################

bool checkJointsReached(double target_joints[7], double current_joints[7], double tolerance = 0.05)
{
    for (int i = 0; i < 7; i++)
    {
        double error = std::abs(target_joints[i] - current_joints[i]);
        if (error >= tolerance)
        {
            return false;
        }
    }
    return true;
}

int main()
{
    // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init)
    {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    // 检查伺服和手臂是否有错，有错误清错
    // 订阅最新数据获取机械臂的错误和状态，有错误清错
    OnGetBuf(&dcss);
    int arm_error_a = dcss.m_State[0].m_ERRCode;
    int arm_error_b = dcss.m_State[1].m_ERRCode;
    int arm_state_a = dcss.m_State[0].m_CurState;
    int arm_state_b = dcss.m_State[1].m_CurState;
    if (arm_error_a != 0 || arm_state_a == 100)
    {
        std::cout << "arm A: exits error, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (arm_error_b != 0 || arm_state_b == 100)
    {
        std::cout << "arm B: exits error, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }

    // 获取伺服错误，有错误清错
    long ErrCode_A[7] = {};
    long ErrCode_B[7] = {};
    OnGetServoErr_A(ErrCode_A);
    OnGetServoErr_B(ErrCode_B);
    bool allZero_a = true;
    bool allZero_b = true;
    for (int i = 0; i < 7; ++i)
    {
        if (ErrCode_A[i] != 0)
        {
            allZero_a = false;
            break;
        }
    }
    for (int i = 0; i < 7; ++i)
    {
        if (ErrCode_B[i] != 0)
        {
            allZero_b = false;
            break;
        }
    }
    if (!allZero_a)
    {
        std::cout << "arm A: srvo error exists, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (!allZero_b)
    {
        std::cout << "arm B: srvo error exists, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }

    // 通过确认freame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）
    int motion_tag = 0;
    int frame_update = 0;

    for (int i = 0; i < 5; i++)
    {
        OnGetBuf(&dcss);
        std::cout << "connect frames:" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[0].m_OutFrameSerial)
        {
            motion_tag++;
            frame_update = dcss.m_Out[0].m_OutFrameSerial;
        }
        SLEEP(1);
    }
    if (motion_tag > 0)
    {
        std::cout << "success:robot connected\n"
                  << std::endl;
    }
    else
    {
        std::cerr << "failed:robot connection failed\n"
                  << std::endl;
        OnRelease();
        return -1;
    }

    // 开启日志
    OnLogOn();
    OnLocalLogOn();

    // 笛卡尔阻抗参数设置
    double k[7] = {3000, 3000, 3000, 100, 100, 100, 20}; // 预设参考。
    double d[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};   // 预设参考。
    OnClearSet();
    OnSetJointLmt_A(20, 20);
    OnSetCartKD_A(k, d);
    OnSetSend();
    SLEEP(200);

    // 切换到笛卡尔阻抗控制模式
    OnClearSet();
    OnSetTargetState_A(3); // 3:torque mode; 1:position mode
    OnSetImpType_A(2);     // type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(1000);

    // 订阅数据，查看是否参数设置成功
    OnGetBuf(&dcss);
    printf("CMD state of A arm:%d\n", dcss.m_State[0].m_CmdState);
    printf("CMD of imdepancd:%d\n", dcss.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[0].m_Joint_Vel_Ratio, dcss.m_In[1].m_Joint_Acc_Ratio);
    printf("CMD of cart K=[%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_In[0].m_Cart_K[0],
           dcss.m_In[0].m_Cart_K[1],
           dcss.m_In[0].m_Cart_K[2],
           dcss.m_In[0].m_Cart_K[3],
           dcss.m_In[0].m_Cart_K[4],
           dcss.m_In[0].m_Cart_K[5],
           dcss.m_In[0].m_Cart_K[6]);
    printf("CMD of cart D=[%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_In[0].m_Cart_D[0],
           dcss.m_In[0].m_Cart_D[1],
           dcss.m_In[0].m_Cart_D[2],
           dcss.m_In[0].m_Cart_D[3],
           dcss.m_In[0].m_Cart_D[4],
           dcss.m_In[0].m_Cart_D[5],
           dcss.m_In[0].m_Cart_D[6]);

    // 设置两个运控点位
    long timeout = 0;
    double fb_joints[7] = {0.0};
    double target_joint[7] = {0, 0, 0, 0, 0, 0, 0};
    double target_joint1[7] = {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};

    // 运动到点位1
    OnClearSet();
    OnSetJointCmdPos_A(target_joint);
    OnSetSend();
    SLEEP(1000); // 预留机器人加速时间
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[0].m_FB_Joint_Pos[joint];
        }
        SLEEP(1);
    } while (!(checkJointsReached(target_joint, fb_joints)));

    // 运动到点位2
    OnClearSet();
    OnSetJointCmdPos_A(target_joint1);
    timeout = OnSetSendWaitResponse(50);
    if (timeout < 0)
    {
        printf("send cmd time out\n");
        return -1;
    }
    SLEEP(1000); // 预留机器人加速时间
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[0].m_FB_Joint_Pos[joint];
        }
        SLEEP(1);
    } while (!(checkJointsReached(target_joint1, fb_joints)));

    // 任务完成，下使能
    OnClearSet();
    OnSetTargetState_A(0);
    timeout = OnSetSendWaitResponse(50);
    if (timeout < 0)
    {
        printf("send cmd time out\n");
        return -1;
    }
    SLEEP(200);

    // 释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 0;
}
