#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <cstdlib>
#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif
// '''#################################################################
// 该DEMO 为SDK类型兼容性检查案列
//
// 使用逻辑
//    1 调用CheckSDKTypeCompat检查调用方字节大小是否与SDK定义(FxType.h)一致
//    2 检查#pragma pack(4)对齐是否生效
//    3 检测当前平台大小端
//    4 以上检查全部通过后，再连接机器人进行通信
//    5 任务完成，释放内存使别的程序或者用户可以连接机器人
//
// 注意：该检查应在连接机器人之前执行，若类型大小或对齐不一致，
//       会导致通信结构体(DCSS/DDSS等)内存布局与控制器不匹配，
//       造成数据解析错位，引发不可预期的行为。
// '''#################################################################

int main()
{
    // ====== 第一步：SDK类型兼容性检查（连接机器人之前） ======
    std::cout << "===== SDK Type Compatibility Check =====" << std::endl;

    int byte_order = -1;
    int ret = CheckSDKTypeCompat(&byte_order);

    if (ret < 0)
    {
        std::cerr << "[FAILED] SDK type compatibility check failed, error mask = 0x"
                  << std::hex << (-ret) << std::dec << std::endl;
        std::cerr << "Please check compiler settings and FxType.h definitions." << std::endl;
        return -1;
    }

    std::cout << "[PASSED] All type sizes match FxType.h definitions" << std::endl;
    std::cout << "[PASSED] #pragma pack(4) alignment is effective" << std::endl;
    std::cout << "[INFO]  Byte order: " << (byte_order == 0 ? "little-endian" : "big-endian") << std::endl;
    std::cout << std::endl;

    // ====== 第二步：连接机器人 ======
    DCSS dcss;

    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init)
    {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);

    // 检查伺服和手臂是否有错，有错误清错
    OnGetBuf(&dcss);
    int arm_error_a = dcss.m_State[0].m_ERRCode;
    int arm_error_b = dcss.m_State[1].m_ERRCode;
    int arm_state_a = dcss.m_State[0].m_CurState;
    int arm_state_b = dcss.m_State[1].m_CurState;
    if (arm_error_a != 0 || arm_state_a == 100)
    {
        std::cout << "arm A: exits error, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (arm_error_b != 0 || arm_state_b == 100)
    {
        std::cout << "arm B: exits error, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }

    // 获取伺服错误，有错误清错
    long ErrCode_A[7] = {};
    long ErrCode_B[7] = {};
    OnGetServoErr_A(ErrCode_A);
    OnGetServoErr_B(ErrCode_B);
    bool allZero_a = true;
    bool allZero_b = true;
    for (int i = 0; i < 7; ++i)
    {
        if (ErrCode_A[i] != 0)
        {
            allZero_a = false;
            break;
        }
    }
    for (int i = 0; i < 7; ++i)
    {
        if (ErrCode_B[i] != 0)
        {
            allZero_b = false;
            break;
        }
    }
    if (!allZero_a)
    {
        std::cout << "arm A: servo error exists, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (!allZero_b)
    {
        std::cout << "arm B: servo error exists, clear error\n"
                  << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }

    // 通过确认frame数据的刷新，确认UDP数据通道连接成功
    int motion_tag = 0;
    int frame_update = 0;

    for (int i = 0; i < 5; i++)
    {
        OnGetBuf(&dcss);
        std::cout << "connect frames:" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[0].m_OutFrameSerial)
        {
            motion_tag++;
            frame_update = dcss.m_Out[0].m_OutFrameSerial;
        }
        SLEEP(1);
    }
    if (motion_tag > 0)
    {
        std::cout << "success: robot connected\n"
                  << std::endl;
    }
    else
    {
        std::cerr << "failed: robot connection failed\n"
                  << std::endl;
        OnRelease();
        return -1;
    }

    // ====== 第三步：任务完成，释放连接 ======
    SLEEP(20);
    OnRelease();
    std::cout << "robot released" << std::endl;
    return 1;
}
