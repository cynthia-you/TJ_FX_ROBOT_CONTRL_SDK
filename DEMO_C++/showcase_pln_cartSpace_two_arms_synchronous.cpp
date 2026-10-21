#include "MarvinSDK.h"
#include "FxRobot.h"
#include "stdio.h"
#include "stdlib.h"
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
// 示例说明：双臂同步直线规划并在位置模式下执行
//
// 整体流程：
//   阶段一：连接与双臂状态检查
//     1. 初始化状态数据结构并连接机器人。
//     2. 读取双臂状态和伺服错误码，发现错误时执行清错。
//     3. 检查 UDP 帧序号是否持续更新，确认数据通道正常。
//     4. 开启控制日志。
//   阶段二：双臂控制模式配置与运动学初始化
//     5. 设置双臂速度、加速度百分比并切换为位置模式。
//     6. 加载机型配置并初始化双臂运动学计算接口。
//     7. 下发初始关节角，使双臂运动至规划起点。
//     8. 通过正运动学计算双臂起点末端位姿并转换为 XYZABC。
//   阶段三：双臂同步直线规划
//     9. 填写双臂同步规划参数结构体。
//    10. 调用 FX_Robot_PLN_MOVL_Arms_Synchronous 生成双臂同步轨迹。
//   阶段四：轨迹执行与资源释放
//    11. 使用 CoRunPlnCart 同时下发双臂规划点并开始运动。
//    12. 等待双臂轨迹执行完成。
//    13. 任务结束后下使能并释放机器人连接。
//
// 注意：
// 1. FX_Robot_PLN_MOVL_Arms_Synchronous 以较长一侧手臂的路径长度为主轴，
//    双臂按相同的时间比例同步运动，运动过程中躯干（机体）保持固定。
// 2. World_Co_Flag 为 false 时，双臂位姿分别基于各自机械臂基坐标系给定；
//    为 true 时，双臂位姿基于机体坐标系给定，需同时提供 Skye_Body_Start_PRR。
// 3. 本示例的 libKine 库需为包含 FX_Robot_PLN_MOVL_Arms_Synchronous 接口的
//    新编译版本，详见 readme.md 中 2.1 编译方法。
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
    // [阶段一｜步骤 2] 读取双臂状态并清除机械臂错误
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

    // 读取双臂伺服错误码并清错
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

        if (frame_update != dcss.m_Out[0].m_OutFrameSerial)
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

    // [阶段二｜步骤 5] 设置双臂速度、加速度百分比并切换为位置模式
    long wait_respond_time = 100;
    int vel = 10;
    int acc = 10;
    OnClearSet();
    OnSetJointLmt_A(vel, acc);
    OnSetJointLmt_B(vel, acc);
    //long return_delay = OnSetSendWaitResponse(wait_respond_time);
    //printf(" cmd delay in 100ms is:%ld\n", return_delay);
    OnSetSend();
    SLEEP(100);

    // 设置控制模式为位置模式
    OnClearSet();
    OnSetTargetState_A(1);
    OnSetTargetState_B(1);
    // return_delay = OnSetSendWaitResponse(wait_respond_time);
    // printf(" cmd delay in 100ms is:%ld\n", return_delay);
    OnSetSend();
    SLEEP(1000);

    // [阶段二｜步骤 6] 加载机型配置并初始化双臂运动学计算接口
    int i = 0;
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
        OnRelease();
        return -1;
    }
    // 初始化双臂运动学参数：0表示左臂(A臂)，1表示右臂(B臂)
    if (FX_Robot_Init_Type(0, TYPE[0]) == false)
    {
        printf("Robot Init Type Error\n");
        OnRelease();
        return -1;
    }
    if (FX_Robot_Init_Kine(0, DH[0]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
        OnRelease();
        return -1;
    }
    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
        OnRelease();
        return -1;
    }
    if (FX_Robot_Init_Type(1, TYPE[1]) == false)
    {
        printf("Robot Init Type Error\n");
        OnRelease();
        return -1;
    }
    if (FX_Robot_Init_Kine(1, DH[1]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
        OnRelease();
        return -1;
    }
    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
        OnRelease();
        return -1;
    }

    // [阶段二｜步骤 7] 下发初始关节角，使双臂运动至规划起点（左右臂镜像对称）
    double joints_a[7] = {17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134};
    double joints_b[7] = {-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetJointCmdPos_B(joints_b);
    OnSetSend();
    SLEEP(3000);

    double fb_joints_a[7] = {0.0};
    double fb_joints_b[7] = {0.0};
    // 等待双臂运动到起始关节角，带超时保护
    int wait_ms = 0;
    bool reached = false;
    do
    {
        OnGetBuf(&dcss);
        for (i = 0; i < 7; i++)
        {
            fb_joints_a[i] = dcss.m_Out[0].m_FB_Joint_Pos[i];
            fb_joints_b[i] = dcss.m_Out[1].m_FB_Joint_Pos[i];
        }
        SLEEP(1);
        wait_ms++;
        reached = checkJointsReached(joints_a, fb_joints_a) &&
                  checkJointsReached(joints_b, fb_joints_b);
    } while (!reached && wait_ms < 60000);
    if (!reached)
    {
        printf("wait arms reach start joints timeout\n");
        OnRelease();
        return -1;
    }

    print_array(fb_joints_a, 7, "current joints of arm A");
    print_array(fb_joints_b, 7, "current joints of arm B");

    // [阶段二｜步骤 8] 通过正运动学计算双臂起点末端位姿并转换为 XYZABC
    double kine_pg_a[4][4];
    if (FX_Robot_Kine_FK(0, joints_a, kine_pg_a) == false)
    {
        printf("Forward Kinematics Error of arm A\n");
        OnRelease();
        return -1;
    }
    double kine_pg_b[4][4];
    if (FX_Robot_Kine_FK(1, joints_b, kine_pg_b) == false)
    {
        printf("Forward Kinematics Error of arm B\n");
        OnRelease();
        return -1;
    }

    double xyzabc_a[6] = {0};
    if (FX_Matrix42XYZABCDEG(kine_pg_a, xyzabc_a) == false)
    {
        printf("matrix to xyzabc failed of arm A.");
        OnRelease();
        return -1;
    }
    double xyzabc_b[6] = {0};
    if (FX_Matrix42XYZABCDEG(kine_pg_b, xyzabc_b) == false)
    {
        printf("matrix to xyzabc failed of arm B.");
        OnRelease();
        return -1;
    }
    print_array(xyzabc_a, 6, "start XYZABC of arm A");
    print_array(xyzabc_b, 6, "start XYZABC of arm B");

    // [阶段三｜步骤 9] 填写双臂同步规划参数结构体
    // 目标：双臂末端各沿自身基坐标系 Z 轴正方向下探 100 mm
    ArmsSynchronousPlanningParams aspp;
    memset(&aspp, 0, sizeof(aspp));

    // World_Co_Flag 为 false 时，位姿基于各自机械臂基坐标系给定，Skye_Body_Start_PRR 不生效；
    // 若需基于机体坐标系给定双臂位姿，请将 World_Co_Flag 置 true，
    // 并填写 Skye_Body_Start_PRR（机体俯仰、横滚及高度，单位：度、度、毫米）。
    aspp.World_Co_Flag = false;

    // Arm0（左臂 A 臂）
    for (i = 0; i < 6; i++)
    {
        aspp.Arm0_Start_XYZABC[i] = xyzabc_a[i];
        aspp.Arm0_End_XYZABC[i] = xyzabc_a[i];
    }
    aspp.Arm0_End_XYZABC[2] += 100;
    for (i = 0; i < 7; i++)
    {
        aspp.Arm0_Ref_Joints[i] = joints_a[i];
    }
    aspp.Arm0_ZSP_Type = 0; // 逆解零空间约束类型：0 与参考关节角欧式距离最近

    // Arm1（右臂 B 臂）
    for (i = 0; i < 6; i++)
    {
        aspp.Arm1_Start_XYZABC[i] = xyzabc_b[i];
        aspp.Arm1_End_XYZABC[i] = xyzabc_b[i];
    }
    aspp.Arm1_End_XYZABC[2] += 100;
    for (i = 0; i < 7; i++)
    {
        aspp.Arm1_Ref_Joints[i] = joints_b[i];
    }
    aspp.Arm1_ZSP_Type = 0;

    // 公共参数
    aspp.Freq = 50;      // 规划频率 50 Hz，与 CoRunPlnCart 下发频率匹配
    aspp.Vel = 50;       // 直线规划速度，单位 mm/s（范围 0.1 ~ 500）
    aspp.Acc = 100;      // 直线规划加速度，单位 mm/s^2（范围 0.1 ~ 500）

    // [阶段三｜步骤 10] 生成双臂同步直线轨迹
    CPointSet pset_arm0;
    CPointSet pset_arm1;
    if (FX_Robot_PLN_MOVL_Arms_Synchronous(&aspp, &pset_arm0, &pset_arm1) == false)
    {
        printf("FX_Robot_PLN_MOVL_Arms_Synchronous Error\n");
        OnRelease();
        return -1;
    }
    printf("pln points of arm A:%ld, arm B:%ld\n",
           pset_arm0.OnGetPointNum(), pset_arm1.OnGetPointNum());
    
    char* a = (char*)"arm0.txt";
    pset_arm0.OnSave(a);

    // [阶段四｜步骤 11] 同时下发双臂规划点并开始运动
    // 等待双臂上一段轨迹结束（双臂都结束才继续），带超时保护
    wait_ms = 0;
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
        wait_ms++;
    } while ((dcss.m_Out[0].m_TrajState != 0 || dcss.m_Out[1].m_TrajState != 0) && wait_ms < 60000);
    if (wait_ms >= 60000)
    {
        printf("wait previous dual arm traj finish timeout\n");
        OnRelease();
        return -1;
    }

    if (!CoRunPlnCart(&pset_arm0, &pset_arm1))
    {
        printf("Failed to run dual arm plan\n");
        OnRelease();
        return -1;
    }

    // [阶段四｜步骤 12] 等待双臂轨迹执行完成，带超时保护
    SLEEP(200);
    wait_ms = 0;
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
        wait_ms++;
    } while ((dcss.m_Out[0].m_TrajState != 0 || dcss.m_Out[1].m_TrajState != 0) && wait_ms < 60000);
    if (wait_ms >= 60000)
    {
        // 超时不得直接下使能（避免运动中断使能），释放连接并退出
        printf("wait dual arm traj finish timeout\n");
        OnRelease();
        return -1;
    }

    OnGetBuf(&dcss);
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "final joints of arm A");
    print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "final joints of arm B");

    // [阶段四｜步骤 13] 任务结束：下使能并释放连接
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(50);

    OnRelease();
    return 0;
}
