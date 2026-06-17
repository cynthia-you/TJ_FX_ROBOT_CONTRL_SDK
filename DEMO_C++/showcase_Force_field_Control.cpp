#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#ifdef _WIN32
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif
// '''#################################################################
// 该DEMO 让机械臂末端以给定的力和扭矩运动到给定的位置距离和姿态距离。
// 可实时触发调整力的方向和大小。针对机器人拉门和拉抽屉任务，
// 需要实时调整力方向和大小的场景十分适用。 此功能无法进行力位混合控制  但可以实时切换控制模式
//注：此时零空间刚度为0,力场模式下，限制零空间运动可以增大零空间阻尼
// 使用逻辑
//      初始化订阅数据的结构体
//      查验连接是否成功,失败程序直接退出
//      设置关节阻抗模式
//      关节阻抗模式运动到给定位置
//      切换笛卡尔力场模式进行力控
//      任务完成,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################

int main()
{
      // 初始化订阅数据的结构体

    // 打印数组的lambda - 保留两位小数
    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i) {
            printf("%.*lf%s", precision, arr[i], i < n - 1 ? "," : "");
        }
        printf("]\n");
        };

    // 打印矩阵的lambda - 保留两位小数
    auto print_matrix = [](auto* mat, size_t rows, size_t cols, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i) {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j) {
                printf("%.*lf%s", precision, mat[i][j], j < cols - 1 ? "," : "");
            }
            printf("]%s\n", i < rows - 1 ? "," : "]");
        }
        };

    // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init) {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    //检查伺服和手臂是否有错，有错误清错
    //订阅最新数据获取机械臂的错误和状态，有错误清错
    OnGetBuf(&dcss);
    int arm_error_a = dcss.m_State[0].m_ERRCode;
    int arm_error_b = dcss.m_State[1].m_ERRCode;
    int arm_state_a = dcss.m_State[0].m_CurState;
    int arm_state_b = dcss.m_State[1].m_CurState;
    if (arm_error_a != 0 || arm_state_a == 100)
    {
        std::cout << "arm A: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (arm_error_b != 0 || arm_state_b == 100)
    {
        std::cout << "arm B: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }

    //获取伺服错误，有错误清错
    long ErrCode_A[7] = {};
    long ErrCode_B[7] = {};
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
    }
    else {
        std::cerr << "failed:robot connection failed\n" << std::endl;
        OnRelease();
        return -1;
    }

    //控制日志开
    OnLogOn();
    OnLocalLogOn();



    

   double K[7] = { 5000,5000,5000,10,10,10,50 };  //设置刚度和阻尼参数
   double D[7] = { 0.5,0.5,0.5,0.3,0.3,0.3,1 };
   double k[7] = { 10, 10, 10,8,5, 4,4 };
   //double d[7] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };

   double d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };
   int run_vel = 20;
   int run_acc = 20;
   OnClearSet();
   OnSetJointLmt_B(run_vel, run_acc);
   OnSetCartKD_B(K, D, 2);
   OnSetJointKD_B(k, d);
   OnSetTargetState_B(ARM_STATE_TORQ);
   OnSetImpType_B(1);
   OnSetSend();
   SLEEP(1000);
   //订阅查看设置是否成功


   double joints_b[7] = { 90, 80, -90, -90, 0, 0, 0 };
   OnClearSet();
   OnSetJointCmdPos_B(joints_b);
   OnSetSend();
   SLEEP(5000);//预留运动时间




   SLEEP(500);
   FTCmd FTCmds;
   FTCmds.fxDir[0] = 1; FTCmds.fxDir[1] = 0; FTCmds.fxDir[2] = 0;
   FTCmds.fxDir[3] = 0; FTCmds.fxDir[4] = 0; FTCmds.fxDir[5] = -1;
   FTCmds.F = 10;
   FTCmds.K = 6000;
   FTCmds.Dis = 50;
   FTCmds.FreeDis = 1;
   FTCmds.NFreeDis = 1;
   FTCmds.Tn = 2;
   FTCmds.Ndis = 100;
   FTCmds.Kn = 80;
   OnClearSet();
   FTArmControl('B', FTCmds);
   OnSetSend();
   SLEEP(8000);

   OnClearSet();
   OnSetTargetState_B(ARM_STATE_IDLE);
   OnSetSend();

    OnRelease();
    return 1;
}
