/**
 * @file showcase_update_system.cpp
 * @brief 展示如何通过 OnUpdateSystem 接口将升级包传输到控制器并触发系统升级。
 *
 * 前置条件：
 *   1. 准备好加密的 .ota 升级包（如通过 host_pack_ota 工具制作）
 *   2. PC 与控制器在同一网段，能 ping 通
 *   3. 已编译 libMarvinSDK.dll / .so
 *
 * OnUpdateSystem 内部流程：
 *   阶段一：通过 TCP 10240 端口将本地文件推送到控制器
 *           /home/FUSION/Tmp/ctrl_package.tar（远端路径硬编码）
 *   阶段二：通过 UDP 4729 端口发送 UPDATES 指令
 *           控制器端 FxProducer 收到后执行：
 *             touch /home/FUSION/Tmp/UpdateFlag
 *           返回成功
 *
 * 升级生效时机：
 *   控制器重启后，run.sh 检测到 UpdateFlag，
 *   解压 ctrl_package.tar 并执行 FXAutoRun.sh，完成系统升级。
 *
 * 对比 OnSendFile（OTA 加密路径）：
 *   - OnSendFile:    灵活指定远程路径，传 .ota 加密包，重启后自动解密 + 升级
 *   - OnUpdateSystem: 固定远端路径 ctrl_package.tar，在线触发标志位，重启后升级
 *
 * 使用逻辑：
 *   1. 查验连接是否成功
 *   2. OnUpdateSystem 发送升级包并触发 UPDATES 指令
 *   3. 任务完成，释放连接
 *   4. 手动重启控制器使升级生效
 */

#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif

////'''#################################################################
//// 该 DEMO 为 OnUpdateSystem 在线升级案列
////
//// 使用逻辑
////    1 查验连接是否成功
////    2 调用 OnUpdateSystem 传输升级包并触发 UPDATES 指令
////    3 任务完成，释放连接
////    4 手动重启控制器使升级生效
////'''#################################################################

int main(int argc, char** argv)
{
    long sdk_version = 0;               ///< SDK 版本号

    /* 解析本地升级包路径（默认或命令行参数） */
    const char* local_ota = "./update_package.ota";
    if (argc >= 2)
    {
        local_ota = argv[1];
    }

    /* 获取 SDK 版本号 */
    sdk_version = OnGetSDKVersion();
    printf("SDK version is 0x%08lx\n", sdk_version);

    /* 建立与控制器之间的通讯连接 */
    printf("[INFO] Connecting to robot (192.168.1.190)...\n");
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init)
    {
        std::cerr << "[ERROR] Failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }
    printf("[INFO] Connected successfully.\n");

    SLEEP(200);

    /*
     * 调用 OnUpdateSystem
     *
     * 内部执行两步操作：
     *   1. TCP 传文件: local_tar → /home/FUSION/Tmp/ctrl_package.tar
     *   2. UDP 发指令: OnSetIntPara("UPDATES", 0)
     *      → 控制器 FxProducer 收到 → touch /home/FUSION/Tmp/UpdateFlag
     *      → 返回成功
     *
     * 注意：
     *   - 远端路径固定为 /home/FUSION/Tmp/ctrl_package.tar，不可自定义
     *   - OnUpdateSystem 是同步阻塞调用，内部会等待文件传输完成 + UPDATES 响应
     *   - 升级不会立即生效，需重启控制器后 run.sh 检测 UpdateFlag 执行升级
     */
    printf("[INFO] Press any key to send OTA package [%s] via OnUpdateSystem...\n", local_ota);
    getchar();

    printf("[INFO] Sending OTA package and triggering UPDATES command...\n");

    clock_t start = clock();

    if (!OnUpdateSystem((char*)local_ota))
    {
        printf("[ERROR] OnUpdateSystem failed!\n");
        printf("  Possible reasons:\n");
        printf("    - File [%s] does not exist\n", local_ota);
        printf("    - TCP file transfer to /home/FUSION/Tmp/ctrl_package.ota failed\n");
        printf("    - UPDATES command response timeout (>100ms)\n");
        OnRelease();
        return -1;
    }

    clock_t end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;

    printf("[INFO] OnUpdateSystem succeeded! (elapsed: %.2f s)\n", elapsed);
    printf("[INFO] Upgrade package sent to: /home/FUSION/Tmp/ctrl_package.ota\n");
    printf("[INFO] UPDATES command sent, UpdateFlag created on controller.\n");
    printf("\n");
    printf("[NEXT] Reboot the controller to apply the upgrade:\n");
    printf("       On the controller, run: reboot\n");
    printf("       Or use SDK: OnSetIntPara(\"REBOOT\", 0)\n");
    printf("\n");
    printf("       On reboot, run.sh will detect UpdateFlag and:\n");
    printf("         tar xf /home/FUSION/Tmp/ctrl_package.tar\n");
    printf("         execute FXAutoRun.sh → upgrade bin/lib/Shell\n");
    printf("         rm UpdateFlag → mount ro /\n");

    /* 任务完成，释放连接 */
    SLEEP(20);
    OnRelease();
    printf("[INFO] Connection released.\n");

    printf("Press any key to exit\n");
    getchar();
    return 0;
}
