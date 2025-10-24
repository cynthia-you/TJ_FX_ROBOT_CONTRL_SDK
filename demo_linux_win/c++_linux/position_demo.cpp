#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>


////'''#################################################################
////该DEMO 为关节位置跟随控制案列
////
////使用逻辑
////    1 初始化订阅数据的结构体
////    2 查验连接是否成功
////    3 为了防止伺服有错，先清错
////    4 设置位置模式和速度加速度百分比
////    6 订阅查看设置是否成功
////    7 下发运动点位
////    8 订阅查看是否运动到位
////    9 任务完成，释放内存使别的程序或者用户可以连接机器人
////'''#################################################################


int main()
{
  // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    int init = OnLinkTo(192,168,1,190);
    if (init == -1) {
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    } else {
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


    //为了防止伺服有错，先清错
    OnClearSet();
    OnClearErr_A();
    OnSetSend();
    usleep(100000);

    //设置设置位置模式和速度加速度百分比
    OnClearSet();
    OnSetTargetState_A(1) ;
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    usleep(100000);


    //订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("current state of A arm:%d\n",dcss.m_State[0].m_CurState);
    printf("cmd state of A arm:%d\n",dcss.m_State[0].m_CmdState);
    printf("error code of A arms:%d\n",dcss.m_State[0].m_ERRCode);
    printf("cmd of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);
        

    //下发运动点位
    double joints[7] = {0,0,0,0,0,0,10};
    OnClearSet();
    OnSetJointCmdPos_A(joints);
    OnSetSend();
    sleep(10);//预留运动时间

    //订阅查看是否运动到位
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
    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}

