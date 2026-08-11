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
// 示例说明：以 50 Hz 控制频率执行 MOVL 离线直线规划轨迹
//
// 整体流程：
//   阶段一：连接与状态检查
//     1. 初始化机器人状态数据结构并连接机器人。
//     2. 读取机械臂状态，清除机械臂及伺服错误。
//     3. 检查 UDP 数据帧是否持续更新，确认通信正常。
//
//   阶段二：控制参数配置
//     4. 开启控制日志。
//     5. 设置笛卡尔阻抗参数、关节速度及加速度比例。
//     6. 将 A 臂切换为力矩模式和笛卡尔阻抗模式。
//     7. 读取并打印配置结果，确认参数已生效。
//
//   阶段三：运动至规划起点
//     8. 下发起始关节角，使 A 臂运动到离线规划的起点。
//     9. 读取指令位置和反馈位置，检查机械臂是否到位。
//
//   阶段四：MOVL 离线规划
//    10. 加载对应机型配置并初始化运动学计算接口。
//    11. 通过正运动学计算起点的末端位姿矩阵。
//    12. 将起点位姿矩阵转换为 XYZABC。
//    13. 以起始位姿为基准，将末端沿 X 方向移动 50 mm。
//    14. 生成 500 Hz 的 MOVL 轨迹文件 movl.txt。
//
//   阶段五：轨迹执行与资源释放
//    15. 每隔 10 个规划点采样一次，以 50 Hz 下发关节位置。
//    16. 轨迹执行结束后下使能并释放机器人连接。
//
// 注意：执行前必须确认配置文件与实际机器人型号、版本及机械臂一致。
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

    // 连接机器人并检查接口初始化结果
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init)
    {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    // [阶段一｜步骤 2] 读取双臂状态，发现机械臂错误时执行清错
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

    // 读取双臂伺服错误码，发现伺服错误时执行清错
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

    // [阶段一｜步骤 3] 检查帧序号是否更新，确认 UDP 数据通道正常
    // 说明：防火墙等因素可能导致接口已连接但无法正常接收数据。
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

    // [阶段二｜步骤 5] 设置笛卡尔阻抗参数
    double K[7] = {3000, 3000, 3000, 100, 100, 100, 20}; // 预设参考值
    double D[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};   // 预设参考值
    OnClearSet();
    OnSetCartKD_A(K, D);
    OnSetSend();
    SLEEP(200);

    // 设置关节速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_A(100, 100);
    OnSetSend();
    SLEEP(200);

    // [阶段二｜步骤 6] 设置力矩模式和笛卡尔阻抗模式
    OnClearSet();
    OnSetTargetState_A(3); // 3：力矩模式；1：位置模式
    OnSetImpType_A(2);     // 1：关节阻抗；2：笛卡尔阻抗；3：力控
    OnSetSend();
    SLEEP(1000);

    // [阶段二｜步骤 7] 读取并打印控制参数，确认配置结果
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n", dcss.m_State[0].m_CurState);
    printf("CMD of impedance:%d\n", dcss.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[0].m_Joint_Vel_Ratio, dcss.m_In[0].m_Joint_Acc_Ratio);
    print_array(dcss.m_In[0].m_Joint_K, 7, "CMD of joint K");
    print_array(dcss.m_In[0].m_Joint_D, 7, "CMD of joint D");

    // [阶段三｜步骤 8] 下发起始关节角，使 A 臂运动至规划起点
    double joints_a[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    SLEEP(3000);
    ; // 预留运动时间

    // [阶段三｜步骤 9] 对比指令位置与反馈位置，检查是否到位
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");

    // -------------------------------------------------------------------------
    // [阶段四｜步骤 10] 加载机型配置并初始化运动学计算接口
    // -------------------------------------------------------------------------
    int i = 0;
    int j = 0;
    // 关闭运动学打印日志
    bool log_switch = false;
    FX_LOG_SWITCH(log_switch);
    // 定义运动学参数存储区
    int TYPE[2];
    double GRV[2][3];
    double DH[2][8][4];
    double PNVA[2][7][4];
    double BD[2][4][3];

    double Mass[2][7];
    double MCP[2][7][3];
    double I[2][7][6];

    // 注意：配置文件与实际机型不匹配时，程序可能正常运行但计算结果错误。
    // 配置文件参考：
    //   - CCS 6 kg 3.1 版本：ccs_m6_31.MvKDCfg
    //   - CCS 6 kg 4.0 版本：ccs_m6_40.MvKDCfg
    //   - CCS 3 kg 版本：ccs_m3.MvKDCfg
    //   - SRS 机型：srs.MvKDCfg
    // 同时需要确认 arm_type 对应左臂（0）还是右臂（1）。
    if (LOADMvCfg((char *)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == false)
    {
        printf("Load CFG Error\n");
        return -1;
    }
    // 初始化机器人类型、DH 参数及运动限制参数
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

    // [阶段四｜步骤 11] 通过正运动学计算起点的末端位姿矩阵
    double jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    double kine_pg[4][4] = {0};
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }

    // [阶段四｜步骤 12] 将起点位姿矩阵转换为 XYZABC
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

    // [阶段四｜步骤 13] 定义直线终点：末端沿 X 方向移动 50 mm
    double end[6] = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }
    end[0] += 50; // 末端沿 X 方向移动 50 mm

    // [阶段四｜步骤 14] 执行 MOVL 离线规划并生成 movl.txt
    char *path = (char *)"movl.txt";
    long freq = 500;
    if (FX_Robot_PLN_MOVL(0, start, end, jv, 100, 100, freq, path) == false)
    {
        printf("MOVL offline Error\n");
        return -1;
    }

    // -------------------------------------------------------------------------
    // [阶段五｜步骤 15] 加载规划文件并以 50 Hz 下发轨迹点
    // 说明：规划文件频率为 500 Hz，每隔 10 个点采样一次。
    // -------------------------------------------------------------------------
    CPointSet pset_movl;
    pset_movl.OnLoadFast(path);
    int point_num = 0;
    point_num = pset_movl.OnGetPointNum();
    printf("[OFFLINE] MOVL number of pvt points:%d\n", point_num);
    double joints_[7] = {0.0};
    for (long tag = 0; tag < point_num; tag += 10) // 500 Hz 轨迹下采样为 50 Hz
    {
        double *pvv = pset_movl.OnGetPoint(tag);
        print_array(pvv, 7, "MOVL offline pvt point");
        joints_[0] = pvv[0];
        joints_[1] = pvv[1];
        joints_[2] = pvv[2];
        joints_[3] = pvv[3];
        joints_[4] = pvv[4];
        joints_[5] = pvv[5];
        joints_[6] = pvv[6];
        if (pvv = NULL)
        {
            printf("MOVL offline pln Error\n");
            return -1;
        }
        else
        {
            OnClearSet();
            OnSetJointCmdPos_A(joints_);
            OnSetSend();
            SLEEP(20); // 控制周期：20 ms
        }
    }

    // [阶段五｜步骤 16] 任务结束：下使能并释放连接
    SLEEP(2000);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetSend();
    SLEEP(200);

    OnRelease();
    return 1;
}
