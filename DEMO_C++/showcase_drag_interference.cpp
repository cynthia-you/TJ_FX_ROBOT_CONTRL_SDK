#include "MarvinSDK.h"
#include "Interference.h"
#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#ifdef _WIN32
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif

////'''#################################################################
////该DEMO 为: 双臂进入关节拖动模式, 获取当前关节做碰撞检测
////
////使用逻辑
////   连接机器人
////   清错
////   初始化碰撞检测，必须根据机型选择配置文件
////   设置关节阻抗KD参数
////   进入关节阻抗模式(扭矩模式 + 关节阻抗类型)
////   进入关节拖动模式(进拖动前必须先进关节阻抗)
////   循环约30s: 持续获取左右臂关节角 -> 碰撞检测, 检测到碰撞只打印不退出
////   退出拖动, 释放机器人
////'''#################################################################

/******* 编译指令
  cd .\DEMO_C++\
  g++ showcase_drag_interference.cpp -o showcase_drag_interference.exe -L. -lKine -lMarvinSDK -lInterfCheck
*/

// 碰撞检测阈值(单位mm), 对应15个碰撞对
double interf_threshold[15] = { 10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                                10, 10,  10, 10, 10};

int main()
{
    // ===== 1. 连接机器人 =====
    bool link_ok = OnLinkTo(192, 168, 1, 190);
    if (!link_ok) {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }
    SLEEP(200);

    // ===== 2. 清错(借鉴 showcase_position_two_arms.cpp) =====
    DCSS dcss;
    OnGetBuf(&dcss);

    // 2.1 手臂错误/状态检查, 有错误清错
    for (int arm_idx = 0; arm_idx < 2; arm_idx++) {
        int arm_err = dcss.m_State[arm_idx].m_ERRCode;
        int arm_sta = dcss.m_State[arm_idx].m_CurState;
        if (arm_err != 0 || arm_sta == 100) {
            std::cout << "arm " << (arm_idx == 0 ? 'A' : 'B')
                      << ": error exists, clearing" << std::endl;
            SLEEP(20);
            OnClearSet();
            if (arm_idx == 0) { OnClearErr_A(); } else { OnClearErr_B(); }
            OnSetSend();
            SLEEP(20);
        }
    }

    // 2.2 伺服错误检查, 有错误清错
    long servo_err_a[7] = {0};
    long servo_err_b[7] = {0};
    OnGetServoErr_A(servo_err_a);
    OnGetServoErr_B(servo_err_b);
    bool servo_err_a_exist = false;
    bool servo_err_b_exist = false;
    for (int idx = 0; idx < 7; idx++) {
        if (servo_err_a[idx] != 0) { servo_err_a_exist = true; }
        if (servo_err_b[idx] != 0) { servo_err_b_exist = true; }
    }
    if (servo_err_a_exist) {
        std::cout << "arm A: servo error exists, clearing" << std::endl;
        SLEEP(20); OnClearSet(); OnClearErr_A(); OnSetSend(); SLEEP(20);
    }
    if (servo_err_b_exist) {
        std::cout << "arm B: servo error exists, clearing" << std::endl;
        SLEEP(20); OnClearSet(); OnClearErr_B(); OnSetSend(); SLEEP(20);
    }

    // ===== 3. 初始化碰撞检测 =====
    //必须根据机型选择配置文件
    FX_InterfHandle interf_handle = FX_Interf_Create();
    int interf_ret = FX_Interf_Init(interf_handle,
        (char*)"../interferenceCheck/InterferenceCfg/CordDef_CCSM6.Cord",
        (char*)"../interferenceCheck/InterferenceCfg/CalLinkDef.Links",
        (char*)"../interferenceCheck/InterferenceCfg/CalInputDef.Joints",
        (char*)"../interferenceCheck/InterferenceCfg/ConvexDef_CCSM6.Convex",
        (char*)"../interferenceCheck/InterferenceCfg/InterfDef.Interf");
    if (interf_ret != FUNC_RET_SUCCESS) {
        printf("interference init failed: %d\n", interf_ret);
        OnRelease();
        return -1;
    }

    // ===== 4. 设置关节阻抗KD参数(K非负, D取值0~1) =====
    double joint_stiffness_K[7] = {3, 3, 3, 3, 2, 2, 2};
    double joint_damping_D[7]   = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

    OnClearSet();
    OnSetJointKD_A(joint_stiffness_K, joint_damping_D);
    OnSetJointKD_B(joint_stiffness_K, joint_damping_D);
    OnSetSend();
    SLEEP(200);

    // ===== 5. 进入关节阻抗模式(扭矩模式 + 关节阻抗类型) =====
    OnClearSet();
    OnSetTargetState_A(3);  // 3=扭矩模式
    OnSetImpType_A(1);      // 1=关节阻抗
    OnSetTargetState_B(3);
    OnSetImpType_B(1);
    OnSetSend();
    SLEEP(500);

    // ===== 6. 进入关节拖动模式(进拖动前必须先进关节阻抗) =====
    OnClearSet();
    OnSetDragSpace_A(1);    // 1=关节拖动
    OnSetDragSpace_B(1);
    OnSetSend();
    SLEEP(500);

    // ===== 7. 循环约30s: 持续获取左右臂关节角并做碰撞检测 =====
    std::cout << "enter 30s drag + interference check loop" << std::endl;
    auto loop_start = std::chrono::high_resolution_clock::now();
    double run_seconds = 100.0;
    int detect_count = 0;

    while (true) {
        auto now_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now_time - loop_start).count();
        if (elapsed >= run_seconds) { break; }

        // 获取左右臂实时关节角, 拼成14维
        OnGetBuf(&dcss);
        double joint_angles[14] = {0};
        for (int idx = 0; idx < 7; idx++) {
            joint_angles[idx]     = dcss.m_Out[0].m_FB_Joint_Pos[idx];  // 左臂
            joint_angles[idx + 7] = dcss.m_Out[1].m_FB_Joint_Pos[idx];  // 右臂
        }

        // 更新构型 -> 更新包络体 -> 计算碰撞距离
        FX_Interf_UpdateCord(interf_handle, 14, joint_angles);
        FX_Interf_UpdateConvex(interf_handle);
        FX_Interf_CalcInterfDistance(interf_handle);

        // 判断是否碰撞: 返回 FUNC_RET_ISInterference 表示发生碰撞
        int interf_state = FX_Interf_OnIsInterf(interf_handle, interf_threshold);
        if (interf_state == FUNC_RET_ISInterference) {
            detect_count++;
            printf("[%.2fs] interference detected! count=%d\n", elapsed, detect_count);
        }

        SLEEP(10);  // 控制循环频率, 约100Hz
    }

    std::cout << "loop finished, total interference detected: "
              << detect_count << std::endl;


    std::cout << "loop finished, total interference detected: "
              << detect_count << std::endl;
    SLEEP(500);

    // ===== 8. 退出拖动并释放资源 =====
    OnClearSet();
    OnSetTargetState_A(0);
    OnSetTargetState_B(0);
    OnSetSend();
    SLEEP(500);

    FX_Interf_Destroy(interf_handle);
    OnRelease();
    return 0;
}
