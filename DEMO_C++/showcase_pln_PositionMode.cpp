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

//'''#################################################################
// 该DEMO 为在位置模式下,避免通讯抖动，使用规划方式将目标点发送至机器人
//
// 使用逻辑
//   初始化订阅数据的结构体
//   初始化机器人接口
//   查验连接是否成功,失败程序直接退出
//   开启日志以便检查
//   设置速度加速度和位置模式
//   走到初始运动点
//   计算配置初始化（已右臂为例）
//   进行直线优先、关节兜底的规划运动，优先保证能够到达目标笛卡尔位置
//   下使能释放内存使别的程序或者用户可以连接机器人
//'''#################################################################

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

    // 控制日志开
    OnLogOn();
    OnLocalLogOn();

    // 控制日志关
    //  OnLogOff();
    //  OnLocalLogOff();

    // 设置关节的速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_B(100, 100);
    OnSetSend();
    SLEEP(50);

    // 设置控制模式为位置模式
    OnClearSet();
    OnSetTargetState_B(1);
    OnSetSend();
    SLEEP(1000);

    // 订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("B arm\n");
    printf("current state:%d\n", dcss.m_State[1].m_CurState);
    printf("CMD of vel and acc:%d %d\n", dcss.m_In[1].m_Joint_Vel_Ratio, dcss.m_In[1].m_Joint_Acc_Ratio);

    // 下发运动点位
    OnClearSet();
    double fb_joints[7] = {0.0};
    double joints_a[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    OnSetJointCmdPos_B(joints_a);
    OnSetSend();
    SLEEP(5000);

    // 订阅查看是否运动到位
    OnGetBuf(&dcss);
    print_array(dcss.m_In[1].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "current joints of arm A");
    SLEEP(50);

    // MOV_Target在线直线规划步骤：
    // 1. 计算初始化
    // 2. 将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    // 3. 起点的末端位置姿态矩阵转为XYZABC
    // 4. 定义直线结束点的XYZABC
    // 5. 运行在线规划,规划文件为50HZ,点位下发频率50Hz

    // 1 计算初始化
    FX_INT32L i = 0;
    FX_INT32L j = 0;
    // 关闭打印日志
    bool log_switch = false;
    FX_LOG_SWITCH(log_switch);
    // 导入运动学参数
    FX_INT32L TYPE[2];
    FX_DOUBLE GRV[2][3];
    FX_DOUBLE DH[2][8][4];
    FX_DOUBLE PNVA[2][7][4];
    FX_DOUBLE BD[2][4][3];

    FX_DOUBLE Mass[2][7];
    FX_DOUBLE MCP[2][7][3];
    FX_DOUBLE I[2][7][6];

    // 配置导入 !!! 非常重要！！！ 使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，看起来运行正常，但是值错误！！！
    // 确认arm_type是左臂0 还是右臂1
    // ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    // ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
    // srs机型为srs.MvKDCfg.
    if (LOADMvCfg((char *)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_FALSE)
    {
        printf("Load CFG Error\n");
        return -1;
    }
    // 初始化运动学参数
    if (FX_Robot_Init_Type(1, TYPE[1]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
        return -1;
    }
    if (FX_Robot_Init_Kine(1, DH[1]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
        return -1;
    }
    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
        return -1;
    }
    // 2.将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    FX_DOUBLE jv[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    Matrix4 kine_pg;
    if (FX_Robot_Kine_FK(1, jv, kine_pg) == FX_FALSE)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }

    // 3. 起点的末端位置姿态矩阵转为XYZABC
    Vect6 xyzabc = {0};
    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == FX_FALSE)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }
    Vect6 start = {0.0};
    for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
    }

    // 4. 定义直线结束点的XYZABC
    Vect6 end = {0.0};
    Matrix4 pg_end = {
        {0.67886967, -0.71212705, -0.17891636, 11.97283},
        {0.19112802, 0.40665334, -0.89336618, -383.75744},
        {0.70894716, 0.57228328, 0.41217207, 703.32845},
        {0, 0, 0, 1}};

    if (FX_Matrix42XYZABCDEG(pg_end, end) == FX_FALSE)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }

    // 5. 运行在线规划,规划文件为50HZ
    CPointSet pset_movt;
    pset_movt.OnEmpty();
    long freq = 50;
    if (FX_Robot_PLN_MOV_Target(1, start, end, jv, 100, 100, freq, &pset_movt) == FX_FALSE)
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

    // 6. 下发点位
    int tim2 = 0;
    if (!OnSetPlnCart_B(&pset_movt))
    {
        printf("Failed to run MOV Targetplan\n");
        goto EAIT;
    }
    SLEEP(20);

    // 等待运动完成
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
    // 任务完成,下使能，释放内存使别的程序或者用户可以连接机器人
    SLEEP(200);
    OnClearSet();
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(200);

    OnRelease();
    return 1;
}
