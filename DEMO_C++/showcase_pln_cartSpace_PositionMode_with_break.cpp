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

//'''#################################################################
//该DEMO 为在位置模式下,避免通讯抖动，使用规划方式将目标点发送至机器人并中断执行的案列
//
//使用逻辑
//    1 初始化订阅数据的结构体
//    2 初始化机器人接口
//    3 查验连接是否成功,失败程序直接退出
//    4 开启日志以便检查
//    5 设置速度加速度和位置模式
//    6 走到初始运动点
//    7 计算配置初始化
//    8 多次循环：在笛卡尔空间Z方向运动并中断
//    9 下使能释放内存使别的程序或者用户可以连接机器人
//'''#################################################################

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
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
   } 
    else 
    {

       //防总线通信异常,先清错
       SLEEP(50);
       OnClearSet();
       OnClearErr_A();
       OnClearErr_B();
       OnSetSend();
       SLEEP(50);

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
           SLEEP(1);
       }

       if (motion_tag > 0) {
           std::cout << "success:机器人连接成功!" << std::endl;
       } else {
           std::cerr << "failed:机器人连接失败!" << std::endl;
           return -1;
       }
   }


    
    //控制日志开
    OnLogOn();
	OnLocalLogOn();
    
    //控制日志关
    // OnLogOff();
    // OnLocalLogOff();


    //设置关节的速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_A(100,100) ;
    OnSetSend();
    SLEEP(50);


    //设置控制模式为位置模式
    OnClearSet();
    OnSetTargetState_A(1) ;
    OnSetSend();
    SLEEP(50);


    //订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n",dcss.m_State[0].m_CurState);
    printf("CMD of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);



    //下发运动点位
    double joints_a[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    SLEEP(3000);


    double fb_joints[7] = {0.0};
    //订阅查看是否运动到位
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    SLEEP(50);


    //MOVLA在线直线规划步骤：
    //1. 计算初始化
    //2. 将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    //3. 起点的末端位置姿态矩阵转为XYZABC
    //4. 定义直线结束点的XYZABC， 
    //5. 运行在线规划,规划文件为50HZ执行

    //1 计算初始化
    FX_INT32L i = 0;
    FX_INT32L j = 0;
    //关闭打印日志
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
    if (LOADMvCfg((char*)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_FALSE)
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
    //2.将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    FX_DOUBLE jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    for(FX_INT32 i=0;i<7;i++)
    {
        jv[i]=joints_a[i];
    }
    Matrix4 kine_pg;
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == FX_FALSE)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }

    //3. 起点的末端位置姿态矩阵转为XYZABC
    Vect6 xyzabc={0};
    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == FX_FALSE)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }
    Vect6 start = {0.0};
    for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
    }

    //4. 定义直线结束点的XYZABC， showcase是规划了一个YZ平面的边长200mm的矩形
    Vect6 end = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }
    end[2]+=200;

    //5. 运行在线规划,规划文件为50HZ
    CPointSet pset_movla;
    if (FX_Robot_PLN_MOVLA(0, start, end, jv, 1000, 1000, &pset_movla) == FX_FALSE)
    {
        printf("MOVLA Error\n");
        return -1;
    }

    for (i = 0; i < 10; i++)
    {
        //6. 下发点位
        if (!OnSetPlnCart_A(&pset_movla))
        {
            printf("Failed to run MOVLA plan\n");
            return -1;
        }

        // 等待轨迹规划执行1s
        SLEEP(1000);

        //中断规划执行
        OnStopPlnJoint_A();
        SLEEP(200);


    }


    //任务完成,下使能，释放内存使别的程序或者用户可以连接机器人   
    SLEEP(50);
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetSend();
    SLEEP(50);

    OnRelease();
    return 1;
}


