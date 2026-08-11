#include "MarvinSDK.h"
#include "FxRobot.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>

#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif

// =============================================================================
// 示例说明：在位置模式下规划笛卡尔直线轨迹，并演示运动中断
//
// 整体流程：
//   阶段一：连接与控制模式配置
//     1. 初始化状态数据结构并连接机器人。
//     2. 读取机械臂状态和伺服错误码，发现错误时执行清错。
//     3. 检查 UDP 帧序号是否持续更新，确认数据通道正常。
//     4. 开启控制日志。
//     5. 设置速度、加速度及位置模式并确认配置结果。
//   阶段二：起点与运动学初始化
//     6. 下发起始关节角，使 A 臂运动至规划起点。
//     7. 加载机型配置并初始化运动学计算接口。
//     8. 通过正运动学计算起点末端位姿。
//     9. 将起点位姿矩阵转换为 XYZABC。
//   阶段三：直线规划与中断
//    10. 定义 Z 方向直线运动终点。
//    11. 生成 50 Hz 在线直线轨迹。
//    12. 调用控制接口下发规划点并执行轨迹。
//    13. 轨迹执行 1 s 后主动中断当前规划。
//   阶段四：资源释放
//    14. 循环结束后下使能并释放机器人连接。
// =============================================================================

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

    // [阶段一｜步骤 1] 初始化状态数据结构并连接机器人
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init)
    {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    // [阶段一｜步骤 2] 读取状态并清除机械臂错误
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

    // 读取伺服错误码并清错
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

    // [阶段一｜步骤 3] 检查帧序号更新，确认 UDP 数据通道正常
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

    // [阶段一｜步骤 4] 开启控制日志
    OnLogOn();
    OnLocalLogOn();

    // 如需关闭日志，可调用以下接口：
    //  OnLogOff();
    //  OnLocalLogOff();

    // [阶段一｜步骤 5] 设置速度百分比、加速度百分比及位置模式
    OnClearSet();
    OnSetJointLmt_A(100, 100);
    OnSetSend();
    SLEEP(50);

    // 设置控制模式为位置模式
    OnClearSet();
    OnSetTargetState_A(1);
    OnSetSend();
    SLEEP(50);

    // 读取并打印控制参数，查看设置是否成功
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n", dcss.m_State[0].m_CurState);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[0].m_Joint_Vel_Ratio, dcss.m_In[0].m_Joint_Acc_Ratio);

    // [阶段二｜步骤 6] 运动至规划起点
    double joints_a[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    SLEEP(3000);

    double fb_joints[7] = {0.0};
    // 检查指令位置与反馈位置，查看是否运动到位
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    SLEEP(50);

    // [阶段二｜步骤 7] 加载配置并初始化运动学计算接口
    int i = 0;
    int j = 0;
    // 关闭打印日志
    bool log_switch = false;
    FX_LOG_SWITCH(log_switch);
    // 导入运动学参数
    int TYPE[2];
    double GRV[2][3];
    double DH[2][8][4];
    double PNVA[2][7][4];
    double BD[2][4][3];

    double Mass[2][7];
    double MCP[2][7][3];
    double I[2][7][6];

    // 注意：配置文件必须与实际机器人型号和版本一致，否则计算结果可能错误。
    // 确认arm_type是左臂0 还是右臂1
    // ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    // ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
    // srs机型为srs.MvKDCfg.
    if (LOADMvCfg((char *)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == false)
    {
        printf("Load CFG Error\n");
        return -1;
    }
    // 初始化运动学参数
    if (FX_Robot_Init_Type(0, TYPE[0]) == false)
    {
        printf("Robot Init Type Error\n");
        return -1;
    }
    if (FX_Robot_Init_Kine(0, DH[0]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
        return -1;
    }
    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
        return -1;
    }
    // [阶段二｜步骤 8] 通过正运动学计算起点末端位姿
    double jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    for (int i = 0; i < 7; i++)
    {
        jv[i] = joints_a[i];
    }
    double kine_pg[4][4];
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }

    // [阶段二｜步骤 9] 将起点位姿转换为 XYZABC
    double xyzabc[6] = {0};
    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == false)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }
    double start[6] = {0.0};
    for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
    }

    // [阶段三｜步骤 10] 定义 YZ 方向、正方形边长为200mm的直线运动轨迹
    double end[6] = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }
    end[2] += 200;

    // [阶段三｜步骤 11] 生成 50 Hz 在线直线轨迹
    CPointSet pset_movla;
    long freq = 50;
    if (FX_Robot_PLN_MOVLA(0, start, end, jv, 1000, 1000, freq, &pset_movla) == false)
    {
        printf("MOVLA Error\n");
        return -1;
    }

    for (i = 0; i < 10; i++)
    {
        // [阶段三｜步骤 12] 调用控制接口下发规划点
        if (!OnSetPlnCart_A(&pset_movla))
        {
            printf("Failed to run MOVLA plan\n");
            return -1;
        }

        // 轨迹执行 1 s 后进入中断流程
        SLEEP(1000);

        // [阶段三｜步骤 13] 中断当前规划执行
        OnStopPlnJoint_A();
        SLEEP(200);
    }

    // [阶段四｜步骤 14] 任务结束：下使能并释放连接
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetSend();
    SLEEP(50);

    OnRelease();
    return 1;
}
