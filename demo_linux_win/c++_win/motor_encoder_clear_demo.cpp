#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>

//'''#################################################################
//该DEMO 为电机内外编码器清零和编码器清错参数案列
//
//#注意控制器需升级到1003_34方可解锁编码器清零和清错的功能。
//
//使用逻辑
//    1 初始化订阅数据的结构体
//    2 初始化机器人接口
//    3 查验连接是否成功,失败程序直接退出
//    4 开启日志以便检查
//    5 为了防止伺服有错，先清错
//    6 确保下始能
//    7 第一轴/关节电机内编清零
//    8 第一轴/关节电机外编清零
//    9 第一轴/关节电机编码器清错
//    10 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
//'''#################################################################

int main()
{
    // 初始化订阅数据的结构体
    DCSS t;

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

    //下使能
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetTargetState_B(0) ;
    OnSetSend();
    usleep(100000);

    //第一轴/关节
    long axis=0;

    //订阅数据确保下使能后第一轴/关节的电机内编值清零
    //订阅查看双臂是否都下使能
    OnGetBuf(&dcss);
    printf("current state of A arm:%d\n",dcss.m_State[0].m_CurState);
    printf("current state of B arm:%d\n",dcss.m_State[1].m_CurState);
    if (dcss.m_State[0].m_CurState==0 && dcss.m_State[1].m_CurState==0)
    {
        //左臂电机内编
        char paraName1[30]="RESETMOTENC0";
        //右臂电机内编
        char paraName11[30]="RESETMOTENC1";
        OnSetIntPara(paraName1,axis);
        OnSetIntPara(paraName11,axis);
        usleep(100000);
    } else{
        return -1;
    }

    usleep(100000);


    //订阅数据确保下使能后第一轴/关节的电机外编值清零
    //订阅查看双臂是否都下使能
    OnGetBuf(&dcss);
    printf("current state of A arm:%d\n",dcss.m_State[0].m_CurState);
    printf("current state of B arm:%d\n",dcss.m_State[1].m_CurState);
    if (dcss.m_State[0].m_CurState==0 && dcss.m_State[1].m_CurState==0)
    {
        //左臂电机外编
        char paraName2[30]="RESETEXTENC0";
        //右臂电机外编
        char paraName22[30]="RESETEXTENC1";
        OnSetIntPara(paraName2,axis);
        OnSetIntPara(paraName22,axis);
        usleep(100000);
    } else{
        return -1;
    }

    usleep(100000);



    //订阅数据确保下使能后第一轴/关节的电机编码器清错
    //订阅查看双臂是否都下使能
    OnGetBuf(&dcss);
    printf("current state of A arm:%d\n",dcss.m_State[0].m_CurState);
    printf("current state of B arm:%d\n",dcss.m_State[1].m_CurState);
    if (dcss.m_State[0].m_CurState==0 && dcss.m_State[1].m_CurState==0)
    {
        //左臂电机编码器
        char paraName3[30]="CLEARMOTENC0";
        //右臂电机编码器
        char paraName33[30]="CLEARMOTENC1";
        OnSetIntPara(paraName3,axis);
        OnSetIntPara(paraName33,axis);
        usleep(100000);
    } else{
        return -1;
    }

    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}
