#include "MarvinSDK.h" 
#include "FxRobot.h"
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
// 该DEMO 为机器人以关节阻抗50HZ频率执行离线规划的完整演示脚本
// MOVL_KEEPJ离线直线规划步骤：
//     初始化订阅数据的结构体
//     初始化机器人接口
//     查验连接是否成功
//     开启日志以便检查
//     设置阻抗参数
//     设置扭矩模式,关节阻抗模式,速度加速度百分比
//     订阅数据查看是否设置
//     运行到起始点位
//     实列化计算并导入机型DH数据并初始化计算接口
//     定义起点和终点的构型
//     运行离线规划，执行规划文件：规划点位为500HZ， 下采样为50HZ执行
// '''  #################################################################


int main()
{
    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i) {
            printf("%.*lf%s", precision, arr[i], i < n-1 ? "," : "");
        }
        printf("]\n");
    };

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
    
    //控制日志关
    // OnLogOff();
    // OnLocalLogOff();

    //设置关节阻抗参数
    double K[7] = {2,2,2,1.6,1,1,1};//预设参考。
    double D[7] = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};//预设参考。
    OnClearSet();
    OnSetJointKD_A(K, D) ;
    OnSetSend();
    SLEEP(200);

    //设置关节的速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_A(100, 100) ;
    OnSetSend();
    usleep(100000);
    SLEEP(200);

    //设置控制模式为关节阻抗
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetImpType_A(1) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    SLEEP(200);


    //订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n",dcss.m_State[0].m_CurState);
    printf("CMD of impedance:%d\n",dcss.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);
    print_array(dcss.m_In[0].m_Joint_K,7,"CMD of joint K");
    print_array(dcss.m_In[0].m_Joint_D,7,"CMD of joint D");


    //下发运动点位
    double joints_a[7] = {-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    SLEEP(3000);//预留运动时间

    //订阅查看是否运动到位
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    

    //MOVL_KEEPJ约束起点和终点构型的离线直线规划步骤：
    //1. 计算初始化
    //2. 定义起点和终点的构型
    //3. 运行离线规划得到规划文件movl_keepj.txt
    //4. 用关节阻抗执行规划文件：规划文件为500HZ， 下采样为50HZ执行

    //1. 计算初始化
    FX_INT32L i = 0;
    FX_INT32L j = 0;
    //关闭运动学打印日志
    bool log_switch=false;
    FX_LOG_SWITCH(log_switch);
   //导入运动学参数
    FX_INT32L TYPE[2];
    FX_DOUBLE GRV[2][3];
    FX_DOUBLE DH[2][8][4];
    FX_DOUBLE PNVA[2][7][4];
    FX_DOUBLE BD[2][4][3];

    FX_DOUBLE Mass[2][7];
    FX_DOUBLE MCP[2][7][3];
    FX_DOUBLE I[2][7][6];
    if (LOADMvCfg((char*)"ccs_m6.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_FALSE)
    {
        printf("Load CFG Error\n");
        return -1;
    }
    //初始化运动学参数
    if (FX_Robot_Init_Type(0, TYPE[0]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
        return -1;
    }
    if (FX_Robot_Init_Kine(0, DH[0]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
        return -1;
    }
    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
        return -1;
    }

    
    //2. 定义起点和终点的构型
    FX_DOUBLE angle1[7] = { -5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995 };
    FX_DOUBLE angle2[7] = { -26.908 ,-91.109, 74.502 ,-88.083, -93.599 ,17.151, -13.602 };


    //3. 运行离线规划得到规划文件movl_keepj.txt
    char op[] = "movl_keepj.txt";
    char* path = op;
    long freq=500;
    if (FX_Robot_PLN_MOVL_KeepJ(0, angle1, angle2, 100, 100, freq, path)== FX_FALSE)
    {
        printf("MOVL KeepJ Error\n");
    }

    //4. 用关节阻抗执行规划文件：规划文件为500HZ， 下采样为50HZ执行
    CPointSet pset_movl_keepj;
    pset_movl_keepj.OnLoadFast(path);
    int point_num=0;
    point_num=pset_movl_keepj.OnGetPointNum();
    printf("[OFFLINE] MOVL_KEEPJ number of pvt points:%d\n",point_num);
    double joints_[7]={0.0};
    for (long tag=0; tag<point_num;tag+=10)//规划文件为500HZ， 下采样为50HZ
    {
        double* pvv=pset_movl_keepj.OnGetPoint(tag);
        print_array(pvv,7,"MOVL_KEEPJ offline pvt point");
        joints_[0]=pvv[0];
        joints_[1]=pvv[1];
        joints_[2]=pvv[2];
        joints_[3]=pvv[3];
        joints_[4]=pvv[4];
        joints_[5]=pvv[5];
        joints_[6]=pvv[6];
        if (pvv=NULL)
        {
            printf("MOVL_KEEPJ offline pln Error\n");
            return -1;
        }
        else
        {
            OnClearSet();
            OnSetJointCmdPos_A(joints_);
            OnSetSend();
            SLEEP(2);//2ms
        }
    }


    //任务完成,下使能，释放内存使别的程序或者用户可以连接机器人   
    SLEEP(2000);
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetSend();
    SLEEP(200);

    OnRelease();
    return 1;
}


