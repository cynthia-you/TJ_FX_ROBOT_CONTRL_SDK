#include "MarvinSDK.h" 
#include "FxRobot.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>


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
//     else 
//     {

//        //防总线通信异常,先清错
//        usleep(100000);
//        OnClearSet();
//        OnClearErr_A();
//        OnClearErr_B();
//        OnSetSend();
//        usleep(100000);

//        int motion_tag = 0;
//        int frame_update = 0;

//        for (int i = 0; i < 5; i++) {
//            OnGetBuf(&dcss);
//            std::cout << "connect frames :" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

//            if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
//                frame_update != dcss.m_Out[0].m_OutFrameSerial) {
//                motion_tag++;
//                frame_update = dcss.m_Out[0].m_OutFrameSerial;
//            }
//            usleep(100000);
//        }

//        if (motion_tag > 0) {
//            std::cout << "success:机器人连接成功!" << std::endl;
//        } else {
//            std::cerr << "failed:机器人连接失败!" << std::endl;
//            return -1;
//        }
//    }

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
    usleep(100000);

    //设置关节的速度和加速度百分比
    OnClearSet();
    OnSetJointLmt_A(100, 100) ;
    OnSetSend();
    usleep(100000);


    //设置控制模式为关节阻抗
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetImpType_A(1) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    usleep(100000);


    //订阅查看设置是否成功
    OnGetBuf(&dcss);
    printf("A arm\n");
    printf("current state:%d\n",dcss.m_State[0].m_CurState);
    printf("CMD of impedance:%d\n",dcss.m_In[0].m_ImpType);
    printf("CMD of vel and acc:%d %d\n",dcss.m_In[0].m_Joint_Vel_Ratio,dcss.m_In[0].m_Joint_Acc_Ratio);
    print_array(dcss.m_In[0].m_Joint_K,7,"CMD of joint K");
    print_array(dcss.m_In[0].m_Joint_D,7,"CMD of joint D");


    //下发运动点位
    double joints_a[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    OnClearSet();
    OnSetJointCmdPos_A(joints_a);
    OnSetSend();
    sleep(3);//预留运动时间

    //订阅查看是否运动到位
    OnGetBuf(&dcss);
    print_array(dcss.m_In[0].m_Joint_CMD_Pos, 7, "CMD joints of arm A");
    print_array(dcss.m_Out[0].m_FB_Joint_Pos, 7, "current joints of arm A");
    usleep(100000);




    //MOVL离线直线规划步骤：
    //1. 计算初始化
    //2. 将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    //3. 起点的末端位置姿态矩阵转为XYZABC
    //4. 定义直线结束点的XYZABC， showcase是规划了末端X方向移动50毫米
    //5. 运行离线规划得到规划文件movl.txt
    //6. 用关节阻抗执行规划文件：规划文件为500HZ， 下采样为50HZ执行

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

    //2.将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    FX_DOUBLE jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
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

    //4. 定义直线结束点的XYZABC， showcase是规划了末端X方向移动50毫米
    Vect6 end = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }
    end[0]+=50;//末端X方向移动50毫米

    //5. 运行离线规划得到规划文件movl.txt
    char op[] = "movl.txt";
    char* path = op;
    if (FX_Robot_PLN_MOVL(0, start, end, jv, 100, 100, path) == FX_FALSE)
    {
        printf("MOVL offline Error\n");
        return -1;
    }

    //6. 用关节阻抗执行规划文件：规划文件为500HZ， 下采样为50HZ执行
    CPointSet pset_movl;
    pset_movl.OnLoadFast(path);
    int point_num=0;
    point_num=pset_movl.OnGetPointNum();
    printf("[OFFLINE] MOVL number of pvt points:%d\n",point_num);
    double joints_[7]={0.0};
    for (long tag=0; tag<point_num;tag+=10)//规划文件为500HZ， 下采样为50HZ
    {
        double* pvv=pset_movl.OnGetPoint(tag);
        print_array(pvv,7,"MOVL offline pvt point");
        joints_[0]=pvv[0];
        joints_[1]=pvv[1];
        joints_[2]=pvv[2];
        joints_[3]=pvv[3];
        joints_[4]=pvv[4];
        joints_[5]=pvv[5];
        joints_[6]=pvv[6];
        if (pvv=NULL)
        {
            printf("MOVL offline pln Error\n");
            return -1;
        }
        else
        {
            OnClearSet();
            OnSetJointCmdPos_A(joints_);
            OnSetSend();
            usleep(200000);//microseconds,200ms
        }
    }


    //任务完成,下使能，释放内存使别的程序或者用户可以连接机器人   
    sleep(2);
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetSend();
    usleep(100000);

    OnRelease();
    return 1;
}

