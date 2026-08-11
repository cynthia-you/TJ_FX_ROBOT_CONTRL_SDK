#include "FxRobot.h"
#include "MarvinSDK.h"
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
// 示例说明：在位置模式下使用控制接口下发多点笛卡尔规划点位，降低直接下发目标的通信抖动
//
// 整体流程：
//   阶段一：连接与控制模式配置
//     1. 初始化状态数据结构、连接机器人并执行清错。
//     2. 开启控制日志。
//     3. 设置关节速度和加速度参数。
//     4. 将 A 臂切换为位置模式并确认配置结果。
//   阶段二：起点与运动学初始化
//     5. 下发起始关节角，使 A 臂运动至规划起点。
//     6. 加载机型配置并初始化运动学计算接口。
//     7. 通过正运动学计算起点末端位姿。
//     8. 将起点位姿矩阵转换为 XYZABC。
//   阶段三：多点笛卡尔规划
//     9. 定义 YZ 平面矩形的多个目标点。
//    10. 依次输入目标点并生成 50 Hz 多段直线轨迹。
//   阶段四：轨迹执行与资源释放
//    11. 使用控制接口下发规划点并开始运动。
//    12. 等待整条轨迹执行完成。
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
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    }
    else
    {
        // 为避免总线通信异常，连接后先执行清错
        SLEEP(50);
        OnClearSet();
        OnClearErr_A();
        OnClearErr_B();
        OnSetSend();
        SLEEP(50);

        int motion_tag = 0;
        int frame_update = 0;

        for (int i = 0; i < 5; i++)
        {
            OnGetBuf(&dcss);
            std::cout << "connect frames :" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

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
            std::cout << "success:机器人连接成功!" << std::endl;
        }
        else
        {
            std::cerr << "failed:机器人连接失败!" << std::endl;
            return -1;
        }
    }

    // [阶段一｜步骤 2] 开启控制日志
    OnLogOn();
    OnLocalLogOn();

    // 如需关闭日志，可调用以下接口：
    //  OnLogOff();
    //  OnLocalLogOff();

    // [阶段一｜步骤 3] 设置关节速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_A(100, 100);
    OnSetSend();
    SLEEP(50);

    // [阶段一｜步骤 4] 设置位置模式
    OnClearSet();
    OnSetTargetState_A(1);
    OnSetSend();
    SLEEP(50);

    // 读取并打印控制参数
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n", dcss.m_State[0].m_CurState);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[0].m_Joint_Vel_Ratio, dcss.m_In[0].m_Joint_Acc_Ratio);

    // [阶段二｜步骤 5] 运动至规划起点
    double joints_a[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    SLEEP(3000);

    double fb_joints[7] = {0.0};
    // 检查指令位置与反馈位置
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    SLEEP(50);

    // return 0;

    // [阶段二｜步骤 6] 加载配置并初始化运动学计算接口
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

    // [阶段二｜步骤 7] 通过正运动学计算起点末端位姿
    double jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    double kine_pg[4][4];
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }

    // [阶段二｜步骤 8] 将起点位姿转换为 XYZABC
    double xyzabc[6] = {0};
    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == false)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }

    // [阶段三｜步骤 9] 定义 YZ 平面矩形的目标点
    double start[6] = {0.0};
    double end[6] = {0.0};
    for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
        end[i] = xyzabc[i];
    }

    end[2] += 200; // 末端沿 Z 轴正方向移动 200 mm

    double zsp_p[6] = {0};
    zsp_p[2] = -1;

    // [阶段三｜步骤 10] 依次输入多个目标点并生成 50 Hz 轨迹
    CPointSet ret_pset1;
    ret_pset1.OnEmpty();
    long freq = 50;
    // 设置第一段起点和终点
    if (FX_Robot_PLN_Set_MOVL_Start(0, jv, start, end, 5.0, 1, zsp_p, 1000, 2000, freq) == false)
    {
        printf("MOVL Start Error\n");
        return -1;
    }

    // 输入第二个目标点
    end[1] -= 200;
    if (FX_Robot_PLN_Set_MOVL_Next_Point(0, end, 5.0, 1, zsp_p, 1000, 2000) == false)
    {
        printf("----------------------------\n");
    }

    // 输入第三个目标点
    end[2] -= 200;
    if (FX_Robot_PLN_Set_MOVL_Next_Point(0, end, 5.0, 1, zsp_p, 1000, 2000) == false)
    {
        printf("----------------------------\n");
    }

    // 输入第n个目标点
    end[1] += 200;
    if (FX_Robot_PLN_Set_MOVL_Next_Point(0, end, 5.0, 1, zsp_p, 1000, 2000) == false)
    {
        printf("----------------------------\n");
    }

    //
    if (FX_Robot_PLN_Get_MOVL_Path(0, &ret_pset1) == false)
    {
        printf("----------------------------\n");
    }

    long num = ret_pset1.OnGetPointNum();
    double *p = ret_pset1.OnGetPoint(num - 1);
    jv[0] = p[0];
    jv[1] = p[1];
    jv[2] = p[2];
    jv[3] = p[3];
    jv[4] = p[4];
    jv[5] = p[5];
    jv[6] = p[6];

    // [阶段四｜步骤 11] 调用控制接口，下发规划点并开始运动
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
    } while (dcss.m_Out[0].m_TrajState != 0);

    if (!OnSetPlnCart_A(&ret_pset1))
    {
        printf("Failed to run MOVLA plan\n");
        goto FAIL_STEP;
    }

    // return 0;
    // [阶段四｜步骤 12] 等待轨迹执行完成
    SLEEP(200);
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[0].m_FB_Joint_Pos[joint];
        }
    } while (!(dcss.m_Out[0].m_LowSpdFlag == 1 || checkJointsReached(jv, fb_joints)));

    // [阶段四｜步骤 13] 任务结束：下使能并释放连接
FAIL_STEP:
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetSend();
    SLEEP(50);

    OnRelease();
    return 1;
}
