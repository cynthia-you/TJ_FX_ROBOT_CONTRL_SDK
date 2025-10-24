#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>

// '''#################################################################
// 该DEMO 为关节阻抗控制案列
// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 设置阻抗参数
//     5 设置扭矩模式和速度加速度百分比
//     6 选择阻抗模式
//     7 订阅数据查看是否设置
//     8 下发运动点位1
//     9 订阅查看是否运动到位
//     10 任务完成,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################

int main()
{
    // 初始化订阅数据的结构体
    DCSS t;

    // 查验连接是否成功
    int init = OnLinkTo(192,168,1,190);
    if (init == -1) {
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    } else {
        int motion_tag = 0;
        int frame_update = 0;

        for (int i = 0; i < 5; i++) {
            OnGetBuf(&t);
            std::cout << "connect frames :" << t.m_Out[0].m_OutFrameSerial << std::endl;

            if (t.m_Out[0].m_OutFrameSerial != 0 &&
                frame_update != t.m_Out[0].m_OutFrameSerial) {
                motion_tag++;
                frame_update = t.m_Out[0].m_OutFrameSerial;
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

    //设置关节阻抗参数
    double K[7] = {2,2,2,1.6,1,1,1};//预设为参数最大上限，供参考。
    double D[7] = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};//预设为参数最大上限，供参考。
    OnClearSet();
    OnSetJointKD_A(K, D) ;
    OnSetSend();
    usleep(100000);

    //设置关节的速度和加速度
    OnClearSet();
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    usleep(100000);

    //设置机器人运动控制模式为关节阻抗
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetImpType_A(1) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    usleep(100000);

    //刷新订阅数据查看设置是否成功
    OnGetBuf(&t);
    printf("current state of A arm:%d\n",t.m_State[0].m_CurState);
    printf("cmd state of A arm:%d\n",t.m_State[0].m_CmdState);
    printf("error code of A arms:%d\n",t.m_State[0].m_ERRCode);

    printf("CMD of imdepancd:%d\n",t.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n",t.m_In[0].m_Joint_Vel_Ratio,t.m_In[0].m_Joint_Acc_Ratio);
    
    printf("CMD of joint D=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Joint_K[0],
                                                            t.m_In[0].m_Joint_K[1],
                                                            t.m_In[0].m_Joint_K[2],
                                                            t.m_In[0].m_Joint_K[3],
                                                            t.m_In[0].m_Joint_K[4],
                                                            t.m_In[0].m_Joint_K[5],
                                                            t.m_In[0].m_Joint_K[6]);

    printf("CMD of joint D=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Joint_D[0],
                                                            t.m_In[0].m_Joint_D[1],
                                                            t.m_In[0].m_Joint_D[2],
                                                            t.m_In[0].m_Joint_D[3],
                                                            t.m_In[0].m_Joint_D[4],
                                                            t.m_In[0].m_Joint_D[5],
                                                            t.m_In[0].m_Joint_D[6]);




    // 设置关节点位
    double joints[7] = {10,20,30,40,50,30,40};
    OnClearSet();
    OnSetJointCmdPos_A(joints);
    OnSetSend();
    usleep(100000);

    //刷新订阅数据查看是否到位
    OnGetBuf(&t);
    printf("CMD joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",t.m_In[0].m_Joint_CMD_Pos[0],
                                                                t.m_In[0].m_Joint_CMD_Pos[1],
                                                                t.m_In[0].m_Joint_CMD_Pos[2],
                                                                t.m_In[0].m_Joint_CMD_Pos[3],
                                                                t.m_In[0].m_Joint_CMD_Pos[4],
                                                                t.m_In[0].m_Joint_CMD_Pos[5],
                                                                t.m_In[0].m_Joint_CMD_Pos[6]);
                                                                
    printf("current joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",t.m_Out[0].m_FB_Joint_Pos[0],
                                                                        t.m_Out[0].m_FB_Joint_Pos[1],
                                                                        t.m_Out[0].m_FB_Joint_Pos[2],
                                                                        t.m_Out[0].m_FB_Joint_Pos[3],
                                                                        t.m_Out[0].m_FB_Joint_Pos[4],
                                                                        t.m_Out[0].m_FB_Joint_Pos[5],
                                                                        t.m_Out[0].m_FB_Joint_Pos[6]);
    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}
