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

// '''#################################################################
// 该 DEMO 用于【左臂：设置笛卡尔阻抗/力控相关参数 + 切换扭矩模式 + 读回校验】的示例。

// '''#################################################################
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

    int i = 5;
    OnSetUserSpcfData_A(DCSS_CMD_ARM0_GET_DATA_6FT);
    i = 5;

    while (i-- > 0)
    {

        OnGetBuf(&dcss);
        printf("DCSS_CMD_ARM0_GET_DATA_6FT [%lf %lf %lf %lf %lf %lf]\n", dcss.m_Out[0].m_EST_Joint_Firc_Dot[0],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[1],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[2],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[3],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[4],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[5]);
        printf("current specify Data = %f \n", dcss.m_Out[0].m_EST_Joint_Firc[0]);
        SLEEP(500);
    }

    OnSetUserSpcfData_A(DCSS_CMD_ARM0_GET_DATA_GRAVITY);
    i = 5;
    while (i-- > 0)
    {
        OnGetBuf(&dcss);
        printf("DCSS_CMD_ARM0_GET_DATA_GRAVITY [%lf %lf %lf %lf %lf %lf %lf]\n", dcss.m_Out[0].m_EST_Joint_Firc_Dot[0],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[1],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[2],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[3],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[4],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[5],
               dcss.m_Out[0].m_EST_Joint_Firc_Dot[6]);
        printf("current specify Data = %f \n", dcss.m_Out[0].m_EST_Joint_Firc[0]);
        SLEEP(500);
    }

    OnSetUserSpcfData(DCSS_CMD_GET_DATA_GYRO_ACC);
    i = 5;
    while (i-- > 0)
    {
        OnGetBuf(&dcss);
        printf("DCSS_CMD_GET_DATA_GYRO_ACC [%lf %lf %lf]\n", dcss.m_Out[0].m_EST_Cart_FN[0],
               dcss.m_Out[0].m_EST_Cart_FN[1],
               dcss.m_Out[0].m_EST_Cart_FN[2]);
        printf("current specify Data = %f \n", dcss.m_Out[0].m_EST_Joint_Firc[1]);
        printf("current specify Data = %f \n", dcss.m_Out[1].m_EST_Joint_Firc[1]);
        SLEEP(500);
    }

    OnSetUserSpcfData(DCSS_CMD_GET_DATA_GYRO_ANGLE);
    i = 5;
    while (i-- > 0)
    {
        OnGetBuf(&dcss);
        printf("DCSS_CMD_GET_DATA_GYRO_ANGLE [%lf %lf %lf]\n", dcss.m_Out[0].m_EST_Cart_FN[0],
               dcss.m_Out[0].m_EST_Cart_FN[1],
               dcss.m_Out[0].m_EST_Cart_FN[2]);
        printf("current specify Data = %f \n", dcss.m_Out[0].m_EST_Joint_Firc[1]);
        printf("current specify Data = %f \n", dcss.m_Out[1].m_EST_Joint_Firc[1]);
        SLEEP(500);
    }

    OnSetUserSpcfData(DCSS_CMD_GET_DATA_GYRO_OMEG);
    i = 5;
    while (i-- > 0)
    {
        OnGetBuf(&dcss);
        printf("DCSS_CMD_GET_DATA_GYRO_OMEG [%lf %lf %lf]\n", dcss.m_Out[0].m_EST_Cart_FN[0],
               dcss.m_Out[0].m_EST_Cart_FN[1],
               dcss.m_Out[0].m_EST_Cart_FN[2]);
        printf("current specify Data = %f \n", dcss.m_Out[0].m_EST_Joint_Firc[1]);
        printf("current specify Data = %f \n", dcss.m_Out[1].m_EST_Joint_Firc[1]);
        SLEEP(500);
    }

    OnRelease();
    return 1;
}
