#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>



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

    // 打印数组的lambda - 保留两位小数
    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i) {
            printf("%.*lf%s", precision, arr[i], i < n-1 ? "," : "");
        }
        printf("]\n");
    };

    // 打印矩阵的lambda - 保留两位小数
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

  // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
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

    //为了防止伺服有错，先清错
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    usleep(100000);



    //设置设置位置模式和速度加速度百分比
    OnClearSet();
    OnSetTargetState_A(1) ;
    OnSetJointLmt_A(10, 10) ;
    OnSetTargetState_B(1) ;
    OnSetJointLmt_B(10, 10) ;
    OnSetSend();
    usleep(100000);


    //订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n",dcss.m_State[0].m_CurState);
    printf("error code:%d\n",dcss.m_State[0].m_ERRCode);
    printf("vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);
    printf("B arm\n");
    printf("current state:%d\n",dcss.m_State[1].m_CurState);
    printf("error code:%d\n",dcss.m_State[1].m_ERRCode);
    printf("vel and acc:%d %d\n",dcss.m_In[1].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);
        

    //下发运动点位
    double joints_a[7] = {21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68};
    double joints_b[7] = {-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetJointCmdPos_B(joints_b);
    OnSetSend();
    sleep(3);//预留运动时间

    //订阅查看是否运动到位
    OnGetBuf(&dcss);

    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");

    print_array(dcss.m_In[1].m_Joint_CMD_Pos, 7, "CMD joints of arm B");
    print_array(dcss.m_Out[1].m_FB_Joint_Pos, 7, "current joints of arm B");


    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人    
    OnRelease();
    return 1;
}

