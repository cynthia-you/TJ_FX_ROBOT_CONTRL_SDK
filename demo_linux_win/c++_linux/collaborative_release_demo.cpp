#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>


// '''#################################################################
// 该DEMO 为机器人进入协作释放案列(即关节不为抱死状态,有重力补偿,可以手轻松的扭/拽/拖机器人,用于紧急情况:如机器人卡死抱在一起的姿态,把手臂扭开)

// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 开启日志以便检查
//     4 为了防止伺服有错，先清错
//     5 设置协作释放模式
//     6 复位以取消协作释放模式
//     7 任务完成，释放内存使别的程序或者用户可以连接机器人
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
    OnSetSend();
    usleep(100000);

    //设置协作释放模式
    OnClearSet();
    OnSetTargetState_A(4) ;
    OnSetSend();
    sleep(30);//预留手拖动调整手臂构型时间


    //复位以取消协作释放模式
    OnClearSet();
    OnSetTargetState_A(0) ;
    OnSetSend();
    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}

