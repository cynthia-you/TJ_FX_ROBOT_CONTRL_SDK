#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif

bool checkJointsReached(double target_joints[7],
                        double current_joints[7],
                        double tolerance = 0.05)
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
    ////'''#################################################################
    ////该DEMO 为位置模式下解决指令抖动/通讯抖动问题的用例位置和到位规划功能的混合使用案例
    /// 指定目标位置下发执行存在指令抖动/通讯抖动问题
    /// 因此使用起点A到目标点B的规划功能下发，控制器内部以50HZ执行规划点位，解决直接接收目标点B的通讯抖动问题。
    ////
    ////使用逻辑
    ////    初始化订阅数据的结构体
    ////    查验连接是否成功
    ////    为了防止伺服有错，先清错
    ////    设置位置模式和速度加速度
    ////    运动到初始位置
    ////    完成4次循环：规划点位下发+位置指令下发
    ////    任务完成，释放内存使别的程序或者用户可以连接机器人
    ////'''#################################################################
    auto print_matrix = [](auto *mat, size_t rows, size_t cols, const char *name = "", int precision = 2)
    {
        if (name[0] != '\0')
            printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i)
        {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j)
            {
                printf("%.*lf%s", precision, mat[i][j], j < cols - 1 ? "," : "");
            }
            printf("]%s\n", i < rows - 1 ? "," : "]");
        }
    };

    auto print_array = [](auto *arr, size_t n, const char *name = "", int precision = 2)
    {
        if (name[0] != '\0')
            printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i)
        {
            printf("%.*lf%s", precision, arr[i], i < n - 1 ? "," : "");
        }
        printf("]\n");
    };

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
    if (allZero_a)
    {
        std::cout << "arm A: srvo error exists, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (allZero_b)
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

    // 控制日志开
    OnLogOn();
    OnLocalLogOn();

    // 设置速度加速度百分比
    long return_delay = 0;
    long wait_respond_time = 100;
    int vel = 100;
    int acc = 100;
    OnClearSet();
    OnSetJointLmt_A(vel, acc);
    OnSetJointLmt_B(vel, acc);
    return_delay = OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay);
    SLEEP(100);

    // 设置position
    long return_delay1 = 0;
    OnClearSet();
    ;
    OnSetTargetState_A(1);
    OnSetTargetState_B(1);
    return_delay1 = OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);
    SLEEP(100);

    if (OnInitPlnLmt((char *)"ccs_m6_40.MvKDCfg") != true)
    {
        printf("load cfg failed!\n");
    }
    else
    {
        printf("load NPVA success!\n");
    }

    // 走到初始零位
    double initial_pos[7] = {0.0};
    OnClearSet();
    OnSetJointCmdPos_A(initial_pos);
    OnSetJointCmdPos_B(initial_pos);
    return_delay1 = OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);
    SLEEP(3000);

    double fb_joints0[7] = {0.0};
    double fb_joints1[7] = {0.0};
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints0[joint] = dcss.m_Out[0].m_FB_Joint_Pos[joint];
            fb_joints0[joint] = dcss.m_Out[1].m_FB_Joint_Pos[joint];
        }
        SLEEP(1);
    } while (!checkJointsReached(initial_pos, fb_joints0) and !checkJointsReached(initial_pos, fb_joints1));

    // 定义规划器的速度和加速度比例：范围0~1.
    double vel_ratio = 0.2;
    double acc_ratio = 0.2;

    long j = 0;
    double start_joints0[7] = {0};
    double start_joints1[7] = {0};
    double stop_joints0[7] = {17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134};
    double stop_joints1[7] = {-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134};

    // 刷新直到轨迹状态为0
    do
    {
        OnGetBuf(&dcss);
    } while (dcss.m_Out[0].m_TrajState != 0 and dcss.m_Out[1].m_TrajState != 0);

    // 打印当前关节位置
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "current joints of arm B");

    print_array(start_joints0, 7, "start joints of arm A");
    print_array(start_joints1, 7, "start joints of arm B");
    print_array(stop_joints0, 7, "end joints of arm A");
    print_array(stop_joints1, 7, "end joints of arm B");

    // 调用轨迹规划函数下发点位，解决通讯抖动
    if (OnSetPlnJoint_AB(start_joints0, stop_joints0, start_joints1, stop_joints1, vel_ratio, acc_ratio) != true)
    {
        printf("AB arm pln failed at iteration %ld!\n", j);
        return -1;
    }

    // 等待轨迹规划完成
    do
    {
        OnGetBuf(&dcss);
    } while (dcss.m_Out[0].m_TrajState != 0 and dcss.m_Out[1].m_TrajState != 0);

    OnGetBuf(&dcss);
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "current joints of arm B");

    // 任务完成,下使能，释放内存使别的程序或者用户可以连接机器人
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(50);
    OnRelease();
    return 1;
}
