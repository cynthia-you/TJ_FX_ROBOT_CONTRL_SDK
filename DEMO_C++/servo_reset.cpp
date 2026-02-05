#include "MarvinSDK.h" 
#include <iostream>
#include <cstring>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
    void Sleep(unsigned int milliseconds) {
        usleep(milliseconds * 1000);
    }
#endif

int main()
{
    // 初始化订阅数据的结构体
    DCSS t;
    memset(&t, 0, sizeof(DCSS));

    // 查验连接是否成功
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init) {
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    }

    // 防总线通信异常,先清错
    Sleep(2);
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    Sleep(2);

    int motion_tag = 0;
    int frame_update = 0;

    for (int i = 0; i < 5; i++) {
        bool ret = OnGetBuf(&t);
        if (!ret) {
            std::cerr << "获取数据失败!" << std::endl;
            break;
        }

        std::cout << "connect frames :" << t.m_Out[0].m_OutFrameSerial << std::endl;

        if (t.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != t.m_Out[0].m_OutFrameSerial) {
            motion_tag++;
            frame_update = t.m_Out[0].m_OutFrameSerial;
        }
        Sleep(2);
    }

    if (motion_tag > 0) {
        std::cout << "success:机器人连接成功!" << std::endl;
    }
    else {
        std::cerr << "failed:机器人连接失败!" << std::endl;
        OnRelease();  // 释放资源
        return -1;
    }

    int axis = 1;
    OnServoReset_A(axis);// 所有关节都可软复位，demo仅对第二关节复位

    // 任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 0;
}