#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <math.h>

#ifdef _WIN32
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif

bool checkJointsReached(double target_joints[7],
                    double current_joints[7],
                    double tolerance = 0.05)
{
    for (int i = 0; i < 7; i++) {
        double error = std::abs(target_joints[i] - current_joints[i]);
        if (error >= tolerance) {
            return false;
        }
    }
    return true;
}

int main()
{
    // =========================================================================
    // 示例说明：在位置模式下执行关节空间规划，并演示规划中断
    //
    // 整体流程：
    //   阶段一：连接与控制模式配置
    //     1. 初始化状态数据结构并连接机器人。
    //     2. 读取机械臂状态和伺服错误码，发现错误时执行清错。
    //     3. 检查 UDP 帧序号是否持续更新，确认数据通道正常。
    //     4. 开启控制日志。
    //     5. 设置速度和加速度百分比。
    //     6. 将 A 臂切换为位置模式。
    //   阶段二：起点与规划器配置
    //     7. 下发初始关节角并等待机械臂稳定到达零位。
    //     8. 设置规划器速度和加速度比例。
    //   阶段三：关节规划与中断
    //     9. 使用规划接口下发关节目标。
    //    10. 规划执行 1 s 后主动中断，并读取当前关节位置。
    //   阶段四：资源释放
    //    11. 循环结束后下使能并释放机器人连接。
    // =========================================================================
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

    // [阶段一｜步骤 1] 初始化状态数据结构并连接机器人
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    // [阶段一｜步骤 2] 读取状态并清除机械臂错误
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

    // 读取伺服错误码并清错
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
    if (!allZero_a)
    {
        std::cout << "arm A: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    } 
    if (!allZero_b)
    {
        std::cout << "arm B: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }
    
    // [阶段一｜步骤 3] 检查帧序号更新，确认 UDP 数据通道正常
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

    // [阶段一｜步骤 4] 开启控制日志
    OnLogOn();
	OnLocalLogOn();

    // [阶段一｜步骤 5] 设置速度和加速度百分比
    long return_delay=0;
    long wait_respond_time=100;
    int vel=100;
    int acc=100;
    OnClearSet();
    OnSetJointLmt_A(vel, acc);
    return_delay=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay);
    SLEEP(100);


    // [阶段一｜步骤 6] 设置位置模式
    long return_delay1=0;
    OnClearSet();;
    OnSetTargetState_A(1);
    return_delay1=OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);

    if (OnInitPlnLmt((char*)"ccs_m6_40.MvKDCfg")!=true)
    {
        printf("load cfg failed!\n");
    }else{
        printf("load NPVA success!\n");
    }

    // [阶段二｜步骤 7] 运动至初始零位
    double initial_pos[7]={0.0};
    OnClearSet();
    OnSetJointCmdPos_A(initial_pos);
    return_delay1 = OnSetSendWaitResponse(wait_respond_time);
    printf(" cmd delay in 100ms is:%d\n", return_delay1);
    // 等待到达工作起点并进入稳定状态
    SLEEP(3000);

    double fb_joints[7]={0.0};
    do {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++) {
            fb_joints[joint] = dcss.m_Out[0].m_FB_Joint_Pos[joint];
        }
        SLEEP(1);
    } while (!checkJointsReached(initial_pos, fb_joints));


    // [阶段二｜步骤 8] 设置规划器速度和加速度比例，取值范围为 0～1
    double vel_ratio=0.2;
    double acc_ratio=0.2;

    long j=0;
    double start_joints[7]={0};
    double stop_joints[7]={0,0,30,0,0,0,0};
    print_array(start_joints, 7, "start joints of arm A");
    print_array(stop_joints, 7, "stop joints of arm A");

    for (j = 0; j < 5; j++)
    {
        printf("---iter---:%ld\n",j);
        // 刷新直到轨迹状态为0
        do {
            OnGetBuf(&dcss);
        } while (dcss.m_Out[0].m_TrajState != 0);

        // [阶段三｜步骤 9] 使用规划接口下发关节目标
        if (OnSetPlnJoint_A(start_joints, stop_joints, vel_ratio, acc_ratio) != true)
        {
            printf("A arm pln failed at iteration %ld!\n", j);
            return -1;
        }

        // 规划轨迹执行 1 s
        SLEEP(1000);

        // [阶段三｜步骤 10] 中断当前规划执行
        OnStopPlnJoint_A();
        SLEEP(200);
        // 打印当前关节位置
        print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "arm A break at:");
    }



    // [阶段四｜步骤 11] 任务结束：下使能并释放连接
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetSend();
    SLEEP(50);
    OnRelease();
    return 1;
}
