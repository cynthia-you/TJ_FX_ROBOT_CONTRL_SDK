#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>

////'''#################################################################
////该DEMO 为检查上位机下发指令到机器人控制器接收到的延迟时间
////
////使用逻辑
////    1 初始化订阅数据的结构体
////    2 查验连接是否成功
////    3 为了防止伺服有错，先清错
////    4 多次设置速度加速度百分比查看在100毫秒内的延迟时间
////    5 任务完成，释放内存使别的程序或者用户可以连接机器人
////'''#################################################################
int main()
{
    DCSS dcss;

    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    } else {

        //防总线通信异常,先清错
        usleep(100000);
        OnClearSet();
        OnClearErr_A();
        OnClearErr_B();
        OnSetSend();
        usleep(100000);
  

        int motion_tag = 0;
        int frame_update = 0;

        for (int i = 0; i < 5; i++) {
            OnGetBuf(&dcss);
            std::cout << "connect frames :" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

            if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
                frame_update != dcss.m_Out[0].m_OutFrameSerial) {
                motion_tag++;
                frame_update = dcss.m_Out[0].m_OutFrameSerial;
            }
            usleep(100000);
        }

        if (motion_tag > 0) {
            std::cout << "success:机器人连接成功!" << std::endl;
        } else {
            std::cerr << "failed:机器人连接失败!" << std::endl;
            return -1;
        }
    }


    //设置速度加速度百分比
    long return_delay=0;
    long wait_respond_time=100;
    OnClearSet();
    OnSetJointLmt_A(10, 10);
    return_delay=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay);
    sleep(1);

    //订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("cmd of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);


    long return_delay1=0;
    OnClearSet();
    OnSetJointLmt_A(20, 20);
    return_delay1=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);
    sleep(1);

    OnGetBuf(&dcss);
    printf("cmd of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);

    long return_delay2=0;
    OnClearSet();
    OnSetJointLmt_A(30, 30);
    return_delay2=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay2);
    sleep(1);

    OnGetBuf(&dcss);
    printf("cmd of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);
        


    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}

