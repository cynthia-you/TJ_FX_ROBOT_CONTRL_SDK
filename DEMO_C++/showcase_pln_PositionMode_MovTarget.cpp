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
// 示例说明：在位置模式下执行“直线优先、关节兜底”的目标规划
//
// 整体流程：
//   阶段一：连接与控制模式配置
//     1. 初始化状态数据结构并连接机器人。
//     2. 读取机械臂状态和伺服错误码，发现错误时执行清错。
//     3. 检查 UDP 帧序号是否持续更新，确认数据通道正常。
//     4. 开启控制日志。
//     5. 设置速度、加速度及位置模式并确认配置结果。
//   阶段二：起点与运动学初始化
//     6. 下发起始关节角，使机械臂运动至规划起点。
//     7. 加载右臂机型配置并初始化运动学计算接口。
//     8. 通过正运动学计算起点末端位姿。
//     9. 将起点位姿矩阵转换为 XYZABC。
//   阶段三：目标规划
//    10. 定义目标末端位姿。
//    11. 执行直线优先、关节规划兜底的目标规划。
//   阶段四：轨迹执行与资源释放
//    12. 调用控制接口下发规划点并等待运动完成。
//    13. 任务结束后下使能并释放机器人连接。
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
        std::cout << "connect frames:" << dcss.m_Out[1].m_OutFrameSerial << std::endl;

        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[1].m_OutFrameSerial)
        {
            motion_tag++;
            frame_update = dcss.m_Out[1].m_OutFrameSerial;
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

    // [阶段一｜步骤 5] 设置速度、加速度及位置模式
    OnClearSet();
    OnSetJointLmt_B(100, 100);
    OnSetSend();
    SLEEP(50);

    // 设置控制模式为位置模式
    OnClearSet();
    OnSetTargetState_B(1);
    OnSetSend();
    SLEEP(1000);

    // 读取并打印控制参数
    OnGetBuf(&dcss);
    printf("B arm\n");
    printf("current state:%d\n", dcss.m_State[1].m_CurState);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[1].m_Joint_Vel_Ratio, dcss.m_In[1].m_Joint_Acc_Ratio);

    // [阶段二｜步骤 6] 运动至规划起点
    OnClearSet();
    double fb_joints[7] = {0.0};
    double joints_a[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    OnSetJointCmdPos_B(joints_a);
    OnSetSend();
    SLEEP(5000);

    // 检查指令位置与反馈位置
    OnGetBuf(&dcss);
    print_array(dcss.m_In[1].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "current joints of arm A");
    SLEEP(50);

    // [阶段二｜步骤 7] 加载右臂配置并初始化运动学计算接口
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
    if (FX_Robot_Init_Type(1, TYPE[1]) == false)
    {
        printf("Robot Init Type Error\n");
        return -1;
    }
    if (FX_Robot_Init_Kine(1, DH[1]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
        return -1;
    }
    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
        return -1;
    }
    // [阶段二｜步骤 8] 通过正运动学计算起点末端位姿
    double jv[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    double kine_pg[4][4];
    if (FX_Robot_Kine_FK(1, jv, kine_pg) == false)
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

    // [阶段三｜步骤 10] 定义目标末端位姿
    double end[6] = {0.0};
    double pg_end[4][4] = {
        {0.67886967, -0.71212705, -0.17891636, 11.97283},
        {0.19112802, 0.40665334, -0.89336618, -383.75744},
        {0.70894716, 0.57228328, 0.41217207, 703.32845},
        {0, 0, 0, 1}};

    if (FX_Matrix42XYZABCDEG(pg_end, end) == false)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }

    // [阶段三｜步骤 11] 执行直线优先、关节兜底的目标规划
    CPointSet pset_movt;
    pset_movt.OnEmpty();
    long freq = 50;
    if (FX_Robot_PLN_MOV_Target(1, start, end, jv, 100, 100, freq, &pset_movt) == false)
    {
        printf("MOVL Target Error\n");
        return -1;
    }

    long num1 = pset_movt.OnGetPointNum();
    double *joint1 = pset_movt.OnGetPoint(num1 - 1);
    jv[0] = joint1[0];
    jv[1] = joint1[1];
    jv[2] = joint1[2];
    jv[3] = joint1[3];
    jv[4] = joint1[4];
    jv[5] = joint1[5];
    jv[6] = joint1[6];

    SLEEP(50);

    // [阶段四｜步骤 12] 以 50 Hz 下发规划点
    int tim2 = 0;
    if (!OnSetPlnCart_B(&pset_movt))
    {
        printf("Failed to run MOV Targetplan\n");
        goto EAIT;
    }
    SLEEP(20);

    // 等待目标运动完成
    do
    {
        OnGetBuf(&dcss);
        SLEEP(200);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[1].m_FB_Joint_Pos[joint];
        }
        tim2++;
        if (tim2 > 30)
        {
            break;
        }
    } while (!checkJointsReached(jv, fb_joints) || dcss.m_Out[1].m_TrajState != 0);

EAIT:
    // [阶段四｜步骤 13] 任务结束：下使能并释放连接
    SLEEP(200);
    OnClearSet();
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(200);

    OnRelease();
    return 1;
}
