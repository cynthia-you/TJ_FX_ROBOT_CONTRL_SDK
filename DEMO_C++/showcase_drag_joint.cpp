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
// 该DEMO 为拖动控制案列

// 使用逻辑
//    初始化订阅数据的结构体
//    查验连接是否成功,失败程序直接退出
//    设置拖动类型
//    订阅查看设置是否成功
//    拖动,拖动结束,退出拖动
//    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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

    //进关节拖动前先设置机器人运动控制模式为关节阻抗
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetImpType_A(1) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(200);



  //设置拖动类型
  int dgType = 1;
  //   gType
  // # 0 退出拖动模式
  // # 1 关节空间拖动
  // # 2 笛卡尔空间x方向拖动
  // # 3 笛卡尔空间y方向拖动
  // # 4 笛卡尔空间z方向拖动
  // # 5 笛卡尔空间旋转方向拖动

  OnClearSet();
  OnSetDragSpace_A(dgType);
  OnSetSend();
  SLEEP(200);

  //刷新订阅数据查看设置是否成功
  OnGetBuf(&dcss);
  printf("cmd of drag spcae type:%d\n",dcss.m_In[0].m_DragSpType);

  SLEEP(30000);//模拟拖动时长

  //拖动结束,退出拖动
  OnClearSet();
  OnSetDragSpace_A(0);
  OnSetSend();
  SLEEP(200);

    //  请注意如果是使用坐标拖动的其中一种拖动方式,流程为:
    //  先进笛卡尔阻抗模式-->选择一种迪卡尔拖动-->拖动-->拖动结束退出拖动.
    //  再进别的迪卡尔拖动,使用完毕也要退出拖动.

 //任务完成,释放内存使别的程序或者用户可以连接机器人
  OnRelease();
  return 1;
}
