#include "MarvinSDK.h"
#include "FxRobot.h"
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

int main()
{
    ////'''#################################################################
    ////   该Demo仅用于关节力矩到末端六维力矩的转换计算
    ////使用逻辑
    ////    初始化订阅数据的结构体
    ////    查验连接是否成功
    ////    为了防止伺服有错，先清错
    ////    订阅当前关节角度及当前关节力矩
    ////    调用接口，计算末端六维力矩
    ////    任务完成，释放内存使别的程序或者用户可以连接机器人
    ////'''#################################################################
     auto print_matrix = [](auto* mat, size_t rows, size_t cols, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i) {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j) {
                printf("%.*lf%s", precision, mat[i][j], j < cols-1 ? "," : "");
            }
            printf("]%s\n", i < rows-1 ? "," : "]");
        }
    };

    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
    if (name[0] != '\0') printf("%s=", name);
    printf("[");
    for (size_t i = 0; i < n; ++i) {
        printf("%.*lf%s", precision, arr[i], i < n-1 ? "," : "");
    }
    printf("]\n");
    };

       // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    //检查伺服和手臂是否有错，有错误清错
    //订阅最新数据获取机械臂的错误和状态，有错误清错
    OnGetBuf(&dcss);
    int arm_error_a=dcss.m_State[0].m_ERRCode;
    int arm_error_b=dcss.m_State[1].m_ERRCode;
    int arm_state_a=dcss.m_State[0].m_CurState;
    int arm_state_b=dcss.m_State[1].m_CurState;
   if (arm_error_a!=0 || arm_state_a==100)
    {
        std::cout << "arm A: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (arm_error_b!=0 || arm_state_b==100)
    {
        std::cout << "arm B: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    } 

    //获取伺服错误，有错误清错
    long ErrCode_A[7]={};
    long ErrCode_B[7]={};
    OnGetServoErr_A(ErrCode_A);
    OnGetServoErr_B(ErrCode_B);
    bool allZero_a = true;
    bool allZero_b = true;
    for (int i = 0; i < 7; ++i) 
    {
        if (ErrCode_A[i] != 0) {
            allZero_a = false;
            break;
        }
    }
    for (int i = 0; i < 7; ++i) 
    {
        if (ErrCode_B[i] != 0) {
            allZero_b = false;
            break;
        }
    }
    if (allZero_a)
    {
        std::cout << "arm A: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    } 
    if (allZero_b)
    {
        std::cout << "arm B: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }
    
    //通过确认freame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）
    int motion_tag = 0;
    int frame_update = 0;

    for (int i = 0; i < 5; i++) {
        OnGetBuf(&dcss);
        std::cout << "connect frames:" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[0].m_OutFrameSerial) {
            motion_tag++;
            frame_update = dcss.m_Out[0].m_OutFrameSerial;
        }
        SLEEP(1);
    }
    if (motion_tag > 0) {
        std::cout << "success:robot connected\n" << std::endl;
    } else {
        std::cerr << "failed:robot connection failed\n"<< std::endl;
        OnRelease();
        return -1;
    }

    //控制日志开
    OnLogOn();
	OnLocalLogOn();

    //设置位置模式及关节的速度和加速度百分比
    OnClearSet();
    OnSetTargetState_A(1) ; //3:torque mode; 1:position mode
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    SLEEP(200);

    // 订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("current state of A arm:%d\n", dcss.m_State[0].m_CurState);
    printf("cmd state of A arm:%d\n", dcss.m_State[0].m_CmdState);
    printf("error code of A arms:%d\n", dcss.m_State[0].m_ERRCode);
    printf("cmd of vel and acc:%d %d\n", dcss.m_In[0].m_Joint_Vel_Ratio, dcss.m_In[0].m_Joint_Acc_Ratio);
    printf("------------------------------\n");

    // ===================== 导入 & 初始化运动学参数 =====================
    FX_INT32L i = 0;
    FX_INT32L j = 0;

    FX_INT32L TYPE[2] = {0};
    FX_DOUBLE GRV[2][3] = {0};
    FX_DOUBLE DH[2][8][4] = {0};
    FX_DOUBLE PNVA[2][7][4] = {0};
    FX_DOUBLE BD[2][4][3] = {0};

    FX_DOUBLE Mass[2][7] = {0};
    FX_DOUBLE MCP[2][7][3] = {0};
    FX_DOUBLE I[2][7][6] = {0};

    if (LOADMvCfg((char*)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_TRUE) {
        printf("Robot Load CFG Success\n");
    } else {
        printf("Robot Load CFG Error\n");
        printf(">>> 请确认 ccs_m6.MvKDCfg 是否在可执行程序同目录，或写绝对路径\n");
        return 0;
    }
    printf("------------------------------\n");

    if (FX_Robot_Init_Type(0, TYPE[0]) == FX_FALSE) {
        printf("Robot Init Type Error\n");
        return 0;
    } else {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(0, DH[0]) == FX_FALSE) {
        printf("Robot Init DH Parameters Error\n");
        return 0;
    } else {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == FX_FALSE) {
        printf("Robot Init Limit Parameters Error\n");
        return 0;
    } else {
        printf("Robot Init Limit Parameters Success\n");
    }
    printf("------------------------------\n");

    // ===================== 工具设置（单位阵） =====================
    Matrix4 tool;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tool[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    if (FX_Robot_Tool_Set(0, tool) == FX_FALSE) {
        printf("Robot Set Tool Error\n");
    } else {
        printf("Robot Set Tool Success\n");
    }
    if (FX_Robot_Tool_Rmv(0) == FX_FALSE)
    {
        printf("Robot Remove Tool Error\n");
    }
    else
    {
        printf("Robot Remove Tool Success\n");
    }


    printf("------------------------------\n");

    // ===================== 从订阅获取“当前关节角 + 当前关节力矩” =====================
    // 先刷新一次订阅
    OnGetBuf(&dcss);

    FX_DOUBLE jv[7] = {0};  // 当前关节角
    FX_DOUBLE tau[7] = {0}; // 当前关节力矩/扭矩（你的 Python 里叫 m_EST_Joint_Force）

    for (int k = 0; k < 7; ++k) {
        jv[k] = dcss.m_Out[0].m_FB_Joint_Pos[k];
    }

    // 注意：这里字段名可能因版本不同而不同
    for (int k = 0; k < 7; ++k) {
        tau[k] = dcss.m_Out[0].m_EST_Joint_Force[k];
    }

    printf("current joints(A): ");
    for (int k = 0; k < 7; ++k) printf("%lf ", jv[k]);
    printf("\n");

    printf("current tau(A):    ");
    for (int k = 0; k < 7; ++k) printf("%lf ", tau[k]);
    printf("\n");
    printf("------------------------------\n");

    // ===================== 力矩 -> 末端六维力/力矩 =====================
    Vect6 EE_Torque; // 输出的末端六维力矩
    if (FX_Robot_JntTau2EETau(0, jv, tau, EE_Torque) == FX_FALSE) {
        printf("Robot JntTau2EETau Error, singularity exists\n");
    } else {
        printf("Robot JntTau2EETau Success\n");
    }
    
    printf("Fx=%f Fy=%f Fz=%f\n", EE_Torque[0], EE_Torque[1], EE_Torque[2]);
    printf("Nx=%f Ny=%f Nz=%f\n", EE_Torque[3], EE_Torque[4], EE_Torque[5]);
    printf("------------------------------\n");

    //任务完成,下使能，释放内存使别的程序或者用户可以连接机器人
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetSend();
    SLEEP(50);
    OnRelease();
    return 1;
}
