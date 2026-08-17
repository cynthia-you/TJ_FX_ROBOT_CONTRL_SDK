#include "MarvinSDK.h"
#include "FxRtCSDef.h"
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
// 示例说明：该 DEMO 用于设置左右臂关节/笛卡尔阻抗刚度阻尼参数 ，并从实际控制流程中读回校验。
// 整体流程：
//   阶段一：连接与状态检查
//     1. 初始化状态数据结构并连接机器人。
//     2. 读取机械臂状态和伺服错误码，发现错误时执行清错。
//     3. 检查 UDP 帧序号是否持续更新，确认数据通道正常。
//   阶段二：设置刚度阻尼参数并读回校验
//     4. 分别设置左右臂的笛卡尔刚度阻尼参数
//     5. 左右臂进笛卡尔阻抗模式，读取并打印笛卡尔刚度阻尼参数，确认设置结果。
//     6. 分别设置左右臂的关节刚度阻尼参数
//     7. 左右臂进关节阻抗模式，读取并打印关节刚度阻尼参数，确认设置结果。
//   阶段三：资源释放
//     8. 验证结束后下使能并释放机器人连接。
// =============================================================================
int main()
{
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

    // 控制日志开
    OnLogOn();
    OnLocalLogOn();


    SLEEP(500);
    double K[7] = {3000, 3000, 3000, 600, 300, 300, 20}; // 设置刚度和阻尼参数
    double D[7] = {0.3, 0.1, 0.1, 0.3, 0.3, 0.3, 1};
    OnClearSet();
    OnSetCartKD_A(K, D);
    OnSetSend();
    SLEEP(100);

    OnClearSet();
    OnSetUserSpcfData_A(DCSS_CMD_ARM0_GET_DATA_CARTSTIFF);
    OnSetSend();
    SLEEP(100);
            
    SLEEP(200);
    //设置力矩模式和笛卡尔阻抗模式
    OnClearSet();
    OnSetTargetState_A(3); // 3:torque mode; 1:position mode
    OnSetImpType_A(2);     // type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(1000);

    OnGetBuf(&dcss);
    SLEEP(100);

    printf("A_joint set specify Data = [%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_In[0].m_Cart_K[0],
        dcss.m_In[0].m_Cart_K[1],
        dcss.m_In[0].m_Cart_K[2],
        dcss.m_In[0].m_Cart_K[3],
        dcss.m_In[0].m_Cart_K[4],
        dcss.m_In[0].m_Cart_K[5],
        dcss.m_In[0].m_Cart_KN);
    printf("DCSS_CMD_ARM0_GET_DATA_CARTSTIFF [%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_Out[0].m_EST_Joint_Firc_Dot[0],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[1],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[2],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[3],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[4],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[5],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[6]);

    OnClearSet();
    OnSetTargetState_A(0);
    OnSetSend();
    SLEEP(1000);

    SLEEP(500);
    OnClearSet();
    OnSetCartKD_B(K, D);
    OnSetSend();

    SLEEP(500);
    OnSetUserSpcfData_B(DCSS_CMD_ARM1_GET_DATA_CARTSTIFF);
    OnSetSend();
    SLEEP(100);


    // [阶段二｜步骤 7] 设置力矩模式和笛卡尔阻抗模式
    OnClearSet();
    OnSetTargetState_B(3); // 3:torque mode; 1:position mode
    OnSetImpType_B(2);     // type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(1000);


    OnGetBuf(&dcss);
    SLEEP(100);

    printf("B_joint set specify Data  = [%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_In[1].m_Cart_K[0],
        dcss.m_In[1].m_Cart_K[1],
        dcss.m_In[1].m_Cart_K[2],
        dcss.m_In[1].m_Cart_K[3],
        dcss.m_In[1].m_Cart_K[4],
        dcss.m_In[1].m_Cart_K[5],
        dcss.m_In[1].m_Cart_KN);
    printf("DCSS_CMD_ARM1_GET_DATA_CARTSTIFF[%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_Out[1].m_EST_Joint_Firc_Dot[0],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[1],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[2],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[3],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[4],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[5],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[6]);
    
    SLEEP(100);
    OnClearSet();
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(1000);


    SLEEP(500);
    double k[7] = {2.5, 2, 2.5, 1, 0.9, 0.9, 0.7};
    double d[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1};
    OnClearSet(); 
    OnSetJointKD_A(k, d);
    OnSetSend();
    SLEEP(100);

    OnClearSet(); 
    OnSetUserSpcfData_A(DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF);
    OnSetSend();
    SLEEP(100);

    OnClearSet();
    OnSetTargetState_A(3); // 3:torque mode; 1:position mode
    OnSetImpType_A(1);     // type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(500);

    OnGetBuf(&dcss);
    SLEEP(100);
    printf("A_joint set specify Data= [%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_In[0].m_Joint_K[0],
           dcss.m_In[0].m_Joint_K[1],
           dcss.m_In[0].m_Joint_K[2],
           dcss.m_In[0].m_Joint_K[3],
           dcss.m_In[0].m_Joint_K[4],
           dcss.m_In[0].m_Joint_K[5],
           dcss.m_In[0].m_Joint_K[6]);
    printf("DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF [%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_Out[0].m_EST_Joint_Firc_Dot[0],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[1],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[2],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[3],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[4],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[5],
    dcss.m_Out[0].m_EST_Joint_Firc_Dot[6]);
    SLEEP(1000);
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetSend();


    SLEEP(1000);
    OnClearSet(); 
    OnSetJointKD_B(k, d);
    OnSetSend();
    SLEEP(100);

    OnClearSet(); 
    OnSetUserSpcfData_B(DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF);//实测k、d参数和取值标志位不能一起下发
    OnSetSend();
    SLEEP(100);

    OnClearSet();
    OnSetTargetState_B(3); // 3:torque mode; 1:position mode
    OnSetImpType_B(1);     // type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(1000);

    OnGetBuf(&dcss);
    SLEEP(100);
    printf("B_joint set specify Data= [%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_In[1].m_Joint_K[0],
           dcss.m_In[1].m_Joint_K[1],
           dcss.m_In[1].m_Joint_K[2],
           dcss.m_In[1].m_Joint_K[3],
           dcss.m_In[1].m_Joint_K[4],
           dcss.m_In[1].m_Joint_K[5],
           dcss.m_In[1].m_Joint_K[6]);
    printf("DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF [%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_Out[1].m_EST_Joint_Firc_Dot[0],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[1],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[2],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[3],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[4],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[5],
    dcss.m_Out[1].m_EST_Joint_Firc_Dot[6]);
    
    SLEEP(100);
    OnClearSet();
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(1000);
    OnRelease();
    return 1;
}
