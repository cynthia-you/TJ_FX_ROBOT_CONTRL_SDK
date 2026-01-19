#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>

////'''#################################################################
////该DEMO 为连接n检查案列
////
////使用逻辑
////    1 初始化订阅数据的结构体
////    2 查验连接是否成功
////    3 任务完成，释放内存使别的程序或者用户可以连接机器人
////'''#################################################################
int main() {
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
            OnRelease();
            return -1;
        }
    }

    //任务完成,释放内存使别的程序或者用户可以连接机器人
    usleep(100000);
    OnRelease();
    usleep(100000);
    return 1;
}
