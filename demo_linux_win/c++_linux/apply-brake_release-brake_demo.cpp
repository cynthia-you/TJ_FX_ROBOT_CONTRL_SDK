#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <cstring>


// '''#################################################################
// 该DEMO 为强制抱闸和强制松闸案例,应对手臂飞车或者撞机急停后扭到一团无法上使能情况,先松闸调整,调整完毕后抱闸再切换成想要的控制模式.

// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 左臂强制松闸
//     5 调整完毕,左臂强制抱闸
//     6 右臂强制抱闸
//     7 调整完毕,右臂强制松闸
//     8 任务完成,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################

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
    OnClearErr_B();
    OnSetSend();
    usleep(100000);

    //左臂强制松闸
    char paraName[30]="BRAK0";
    // long retValue=2;
    // char paraName[30];
    // memset(paraName,0,30);
    // sprintf(paraName,"BRAK0");
    OnSetIntPara(paraName,2);
    sleep(30); //预留时间去调整手臂的姿态


    //调整完毕,左臂强制抱闸
    // long retValue1=1;
    OnSetIntPara(paraName,1);
    usleep(100000);


    //右臂强制松闸
    char paraName1[30]="BRAK1";
    OnSetIntPara(paraName1,2);
    sleep(30); //预留时间去调整手臂的姿态


    //调整完毕,右臂强制抱闸
    OnSetIntPara(paraName1,1);
    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}

