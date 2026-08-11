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
// 示例说明：以笛卡尔阻抗模式执行 MOVJ 在线关节轨迹
//
// 整体流程：
//   阶段一：连接与状态检查
//     1. 初始化状态数据结构并连接机器人。
//     2. 读取机械臂状态和伺服错误码，发现错误时执行清错。
//     3. 检查 UDP 帧序号是否持续更新，确认数据通道正常。
//   阶段二：控制参数配置
//     4. 开启控制日志。
//     5. 设置笛卡尔阻抗、关节速度和加速度参数。
//     6. 将 A 臂切换为力矩模式和笛卡尔阻抗模式。
//     7. 读取并打印控制参数，确认配置结果。
//     8. 下发起始关节角，使 A 臂运动至规划起点。
//   阶段三：MOVJ 在线规划
//     9. 加载机型配置并初始化在线规划接口。
//    10. 设置关节空间起点和终点，以500Hz进行关节空间运动规划。
//   阶段四：轨迹执行与资源释放
//    11. 将 500 Hz 在线轨迹下采样为 50 Hz 并逐点执行。
//    12. 轨迹结束后下使能并释放机器人连接。
// =============================================================================

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

    // 检查机器人接口连接结果
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

    // [阶段二｜步骤 4] 开启控制日志
    OnLogOn();
    OnLocalLogOn();

    // 如需关闭日志，可调用 OnLogOff() 和 OnLocalLogOff()。

    // [阶段二｜步骤 5] 设置笛卡尔阻抗参数
    double K[7] = {3000, 3000, 3000, 100, 100, 100, 20}; // 预设参考。
    double D[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};   // 预设参考。
    OnClearSet();
    OnSetCartKD_A(K, D);
    OnSetSend();
    SLEEP(200);

    // 设置关节的速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_A(30, 30);
    OnSetSend();
    SLEEP(200);

    // [阶段二｜步骤 6] 设置力矩模式和笛卡尔阻抗模式
    OnClearSet();
    OnSetTargetState_A(3); // 3:torque mode; 1:position mode
    OnSetImpType_A(2);     // type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(1000);

    // [阶段二｜步骤 7] 读取并打印控制参数
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n", dcss.m_State[0].m_CurState);
    printf("CMD of impedance:%d\n", dcss.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[0].m_Joint_Vel_Ratio, dcss.m_In[0].m_Joint_Acc_Ratio);
    print_array(dcss.m_In[0].m_Joint_K, 7, "CMD of joint K");
    print_array(dcss.m_In[0].m_Joint_D, 7, "CMD of joint D");

    // [阶段二｜步骤 8] 运动至在线规划起点
    double joints_a[7] = {10.0, 20.0, -30.0, 10.0, 5.0, 10.0, -10.0};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    SLEEP(3000); // 预留运动时间

    // 检查指令位置与反馈位置
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    SLEEP(200);

    // [阶段三｜步骤 9] 加载配置并初始化在线规划接口
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
    // [阶段三｜步骤 10] 设置关节空间起点和终点
    double start_joints[7] = {10.0, 20.0, -30.0, 10.0, 5.0, 10.0, -10.0};
    double end_joints[7] = {15.0, 50.0, 10.0, 15.0, 5.0, 10.0, -10.0};

    // [阶段四｜步骤 11] 以500Hz生成在线轨迹，并以 50 Hz 下发
    CPointSet pset_movj;
    long freq = 500;
    if (FX_Robot_PLN_MOVJ(0, start_joints, end_joints, 30, 30, freq, &pset_movj) == false)
    {
        printf("MOVJ Error\n");
        return -1;
    }

    int point_num = 0;
    point_num = pset_movj.OnGetPointNum();
    printf("[ONLINE] MOVJ number of online pvt points:%d\n", point_num);
    double joints_[7] = {0.0};
    for (long tag = 0; tag < point_num; tag += 10) // 500 Hz 轨迹下采样为 50 Hz
    {
        double *pvv = pset_movj.OnGetPoint(tag);
        print_array(pvv, 7, "MOVJ online pvt point");
        joints_[0] = pvv[0];
        joints_[1] = pvv[1];
        joints_[2] = pvv[2];
        joints_[3] = pvv[3];
        joints_[4] = pvv[4];
        joints_[5] = pvv[5];
        joints_[6] = pvv[6];
        if (pvv == NULL)
        {
            printf("MOVJ online pln Error\n");
            return -1;
        }
        else
        {
            OnClearSet();
            OnSetJointCmdPos_A(joints_);
            OnSetSend();
            SLEEP(20);
        }
    }

    // [阶段四｜步骤 12] 任务结束：下使能并释放连接
    SLEEP(200);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetSend();
    SLEEP(200);

    OnRelease();
    return 1;
}
