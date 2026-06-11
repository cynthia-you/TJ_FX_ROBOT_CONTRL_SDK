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
//   计算配置初始化
//   在笛卡尔空间YZ平面执行一个矩形框，分别规划和执行四条边
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
    bool is_connect = true;
    int arm = 1;
    double fb_joints[7] = {0.0};
    double joints_a[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};

    if (is_connect)
    {
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
            OnClearErr_B();
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

            if (dcss.m_Out[1].m_OutFrameSerial != 0 &&
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
        OnSetJointCmdPos_B(joints_a);
        OnSetSend();
        SLEEP(5000);

        // 订阅查看是否运动到位
        OnGetBuf(&dcss);
        print_array(dcss.m_In[1].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
        print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "current joints of arm A");
        SLEEP(50);
    }

    // MOVLA在线直线规划步骤：
    // 1. 计算初始化
    // 2. 将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    // 3. 起点的末端位置姿态矩阵转为XYZABC
    // 4. 定义直线结束点的XYZABC，
    // 5. 运行在线规划,规划文件为50HZ执行

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
    if (LOADMvCfg((char *)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_FALSE)
    {
        printf("Load CFG Error\n");
        return -1;
    }
    // 初始化运动学参数
    if (FX_Robot_Init_Type(0, TYPE[0]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
        return -1;
    }
    if (FX_Robot_Init_Kine(0, DH[0]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
        return -1;
    }
    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
        return -1;
    }
    // 2.将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    FX_DOUBLE jv[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    for (FX_INT32 i = 0; i < 7; i++)
    {
        jv[i] = joints_a[i];
    }
    Matrix4 kine_pg;
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == FX_FALSE)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }
    print_matrix(kine_pg, 4, 4, "forward kinematics result - pose matrix of the end effector");

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

    // 4. 定义直线结束点的XYZABC， showcase是规划了一个YZ平面的边长200mm的矩形
    Vect6 end = {0.0};
    // for (i = 0; i < 6; i++)
    // {
    //     end[i] = xyzabc[i];
    // }
    // end[0]+=50;//末端X方向移动50毫米//11.97283
    Matrix4 pg_end = {
        {0.67886967, -0.71212705, -0.17891636, 11.97283},
        {0.19112802, 0.40665334, -0.89336618, -383.75744},
        {0.70894716, 0.57228328, 0.41217207, 403.32845},
        {0, 0, 0, 1}};

    if (FX_Matrix42XYZABCDEG(pg_end, end) == FX_FALSE)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }

    // 5. 运行在线规划,规划文件为50HZ
    CPointSet pset_movla;
    pset_movla.OnEmpty();
    long freq = 50;
    printf("start movla planning\n");
    FX_DOUBLE angle1[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    FX_DOUBLE angle2[7] = {-60.19359403635502, -10.521529304136436, -34.42557579127981, -42.84602173277278, 142.16525695856654, 17.041249001464394, 1.756722024828207};

    if (1)
    {
        if (FX_Robot_PLN_MOVL_TargetA(0, start, end, jv, 100, 100, freq, &pset_movla) == FX_FALSE)
        {
            printf("MOVLA Error\n");
            return -1;
        }
        // if (FX_Robot_PLN_MOVLA_Redundency_ReconfigA(0, start, end, jv, 80, 100, freq, &pset_movla) == FX_FALSE)
        // {
        //     printf("MOVLA Error\n");
        //     return -1;
        // }
    }
    else
    {
        if (FX_Robot_PLN_MOVL_KeepJA(0, angle1, angle2, 100, 100, freq, &pset_movla) == FX_FALSE)
        {
            printf("MOVL KeepJ Error\n");
        }
    }

    char *path = (char *)"movl_cart_111.txt";
    pset_movla.OnSave(path);

    long num1 = pset_movla.OnGetPointNum();
    double *joint1 = pset_movla.OnGetPoint(num1 - 1);
    jv[0] = joint1[0];
    jv[1] = joint1[1];
    jv[2] = joint1[2];
    jv[3] = joint1[3];
    jv[4] = joint1[4];
    jv[5] = joint1[5];
    jv[6] = joint1[6];

    printf("------------\nstart moving\n-------------\n");
    SLEEP(50);
    // do
    // {
    //     OnGetBuf(&dcss);
    //     SLEEP(1);
    // } while (dcss.m_Out[1].m_TrajState != 0);

    if (is_connect)
    {
        // 6. 下发点位
        if (!OnSetPlnCart_B(&pset_movla))
        {
            printf("Failed to run MOVLA plan\n");
            goto EAIT;
        }
        SLEEP(20);

        // 等待运动完成
        do
        {
            OnGetBuf(&dcss);
            SLEEP(1);
            for (long joint = 0; joint < 7; joint++)
            {
                fb_joints[joint] = dcss.m_Out[1].m_FB_Joint_Pos[joint];
            }
        } while (!checkJointsReached(jv, fb_joints));
    }

EAIT:
    // 任务完成,下使能，释放内存使别的程序或者用户可以连接机器人
    printf("-------------\nmotion completed, start to release\n------------------\n");
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(50);
    OnRelease();
    return 1;
}