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
// 目的：
//     1) 下发右臂笛卡尔刚度/阻尼参数（K/D）与控制类型（type）
//     2) 下发末端笛卡尔旋转方向/参数（Dir）
//     3) 设置关节速度/加速度比例限制
//     4) 切换右臂目标状态到扭矩相关模式，并设置阻抗类型
//     5) 通过 OnGetBuf 读取 DCSS，打印当前状态/命令状态/错误码，以及下发参数是否生效
//

// 参数说明（按当前示例写法）：
//     - K[7] / D[7]：右臂笛卡尔阻抗刚度/阻尼（7维数组，按 SDK 定义）
//     - type：力控/阻抗类型选择（示例中 type=1；具体含义以 SDK 文档为准）
//     - Dir[7]：笛卡尔方向/旋转参数：前三个通常为基于基座 X/Y/Z 的旋转量，后四个保留填 0
//
// '''#################################################################



int main()
{
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

    //设置笛卡尔阻抗参数
    double K[7] = {100,8000,8000,600,300,300,20};   //设置刚度和阻尼参数
    double D[7] = {0.1,0.1,0.1,0.3,0.3,0.3,1};
   
    int type = 1;  //设置右臂力控类型fcType=1。
    double Dir[7] = {0,0,0,0,0,0};  //笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
 

    // 设置右臂参数并发送指令
    OnClearSet();
    OnSetCartKD_A(K, D,type) ;
    OnSetEefRot_A(type, Dir);
    OnSetSend();
    SLEEP(200);

    // 设置右臂速度和加速度限制并发送指令
    OnClearSet();
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    SLEEP(200);

    // 设置右臂为扭矩模式和坐标阻抗模式并发送指令
    OnClearSet();
    OnSetTargetState_A(3) ; 
    OnSetImpType_A(2) ;
    OnSetSend();
    SLEEP(200);

    // 获取右臂状态信息并打印
    OnGetBuf(&dcss);
    printf("current state of A arm:%d\n",dcss.m_State[0].m_CurState);
    printf("cmd state of A arm:%d\n",dcss.m_State[0].m_CmdState);
    printf("error code of A arms:%d\n",dcss.m_State[0].m_ERRCode);

    printf("CMD of imdepancd:%d\n",dcss.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[1].m_Joint_Acc_Ratio);

    printf("CMD of cart K=[%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_In[0].m_Cart_K[0],
                                                            dcss.m_In[0].m_Cart_K[1],
                                                            dcss.m_In[0].m_Cart_K[2],
                                                            dcss.m_In[0].m_Cart_K[3],
                                                            dcss.m_In[0].m_Cart_K[4],
                                                            dcss.m_In[0].m_Cart_K[5],
                                                            dcss.m_In[0].m_Cart_K[6]);

    printf("CMD of cart D=[%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_In[0].m_Cart_D[0],
                                                            dcss.m_In[0].m_Cart_D[1],
                                                            dcss.m_In[0].m_Cart_D[2],
                                                            dcss.m_In[0].m_Cart_D[3],
                                                            dcss.m_In[0].m_Cart_D[4],
                                                            dcss.m_In[0].m_Cart_D[5],
                                                            dcss.m_In[0].m_Cart_D[6]);

    printf("CMD of cart type=%d\n",dcss.m_In[1].m_Cart_KD_Type);
    //打印新的笛卡尔参数
    printf("CMD of NEW cart=[%lf %lf %lf %lf %lf %lf %lf]\n",dcss.m_In[0].m_Force_PIDUL[0],
                                                            dcss.m_In[0].m_Force_PIDUL[1],
                                                            dcss.m_In[0].m_Force_PIDUL[2],
                                                            dcss.m_In[0].m_Force_PIDUL[3],
                                                            dcss.m_In[0].m_Force_PIDUL[4],
                                                            dcss.m_In[0].m_Force_PIDUL[5],
                                                            dcss.m_In[0].m_Force_PIDUL[6]);


    OnGetBuf(&dcss);
  
    printf("CMD joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",dcss.m_In[0].m_Joint_CMD_Pos[0],
                                                                dcss.m_In[0].m_Joint_CMD_Pos[1],
                                                                dcss.m_In[0].m_Joint_CMD_Pos[2],
                                                                dcss.m_In[0].m_Joint_CMD_Pos[3],
                                                                dcss.m_In[0].m_Joint_CMD_Pos[4],
                                                                dcss.m_In[0].m_Joint_CMD_Pos[5],
                                                                dcss.m_In[0].m_Joint_CMD_Pos[6]);
                                                                
    printf("current joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",dcss.m_Out[0].m_FB_Joint_Pos[0],
                                                                      dcss.m_Out[0].m_FB_Joint_Pos[1],
                                                                      dcss.m_Out[0].m_FB_Joint_Pos[2],
                                                                      dcss.m_Out[0].m_FB_Joint_Pos[3],
                                                                      dcss.m_Out[0].m_FB_Joint_Pos[4],
                                                                      dcss.m_Out[0].m_FB_Joint_Pos[5],
                                                                      dcss.m_Out[0].m_FB_Joint_Pos[6]);
    SLEEP(200);


    OnRelease();
    return 1;
}
