#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <math.h>

int main()
{
    ////'''#################################################################
    ////该DEMO 为位置模式下解决指令抖动/通讯抖动问题的用例位置和到位规划功能的混合使用案例
    ///指定目标位置下发执行存在指令抖动/通讯抖动问题
    ///因此使用起点A到目标点B的规划功能下发，控制器内部以50HZ执行规划点位，解决直接接收目标点B的通讯抖动问题。
    ////
    ////使用逻辑
    ////    1 初始化订阅数据的结构体
    ////    2 查验连接是否成功
    ////    3 为了防止伺服有错，先清错
    ////    4 设置位置模式和速度加速度
    ////    5 运动到初始位置
    ////    4 完成4次循环：规划点位下发+位置指令下发
    ////    5 任务完成，释放内存使别的程序或者用户可以连接机器人
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

    DCSS dcss;

    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    }else {

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
            std::cout << "success:robot connected\n" << std::endl;
        } else {
            std::cerr << "failed:robot connection failed\n" << std::endl;
            return -1;
        }
    }


    //设置速度加速度百分比
    long return_delay=0;
    long wait_respond_time=100;
    int vel=100;
    int acc=100;
    OnClearSet();
    OnSetJointLmt_A(vel, acc);
    return_delay=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay);
    sleep(1);


    //设置position
    long return_delay1=0;
    OnClearSet();;
    OnSetTargetState_A(1);
    return_delay1=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);
    sleep(1);

    if (OnInitPlnLmt((char*)"ccs_m6_40.MvKDCfg")!=true)
    {
        printf("load cfg failed!\n");
    }

    //走到初始零位
    double initial_pos[7]={0.0};
    OnClearSet();
    OnSetJointCmdPos_A(initial_pos);
    return_delay1 = OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);

    // 等待运动完成
    do {
        OnGetBuf(&dcss);
    } while (dcss.m_Out[0].m_LowSpdFlag == 1);
    sleep(2);

    //定义规划器的速度和加速度比例：范围0~1.
    double vel_ratio=0.2;
    double acc_ratio=0.2;


    long j=0;
    double start_joints[7]={0};
    double stop_joints[7]={0};

    for (j = 0; j < 5; j++)
    {
        // 刷新直到轨迹状态为0
        do {
            OnGetBuf(&dcss);
        } while (dcss.m_Out[0].m_TrajState != 0);

        // 打印当前关节位置
        print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");

        // 设置起始关节位置
        for (long i = 0; i < 7; i++)
        {
            start_joints[i] = dcss.m_Out[0].m_FB_Joint_Pos[i];
        }

        print_array(start_joints, 7, "start joints of arm A");

        // 更新停止关节位置
        stop_joints[3] -= 20;
        print_array(stop_joints, 7, "stop joints of arm A");

        // 调用轨迹规划函数下发点位，解决通讯抖动
        if (OnSetPln_A(start_joints, stop_joints, vel_ratio, acc_ratio) != true)
        {
            printf("A arm pln failed at iteration %ld!\n", j);
            return -1;
        }

         // 等待轨迹规划完成
        do {
            OnGetBuf(&dcss);
        } while (dcss.m_Out[0].m_LowSpdFlag == 1);
        sleep(1);


        // 直接下发关节命令，有通讯抖动
        stop_joints[3] += 20;
        OnClearSet();
        OnSetJointCmdPos_A(stop_joints);
        return_delay1 = OnSetSendWaitResponse(wait_respond_time);
        printf(" cmd delay in 100ms is:%d\n", return_delay1);

        // 等待运动完成
        do {
            OnGetBuf(&dcss);
        } while (dcss.m_Out[0].m_LowSpdFlag == 1);
        sleep(2);
    }

    sleep(5);
    OnRelease();
    return 1;
}