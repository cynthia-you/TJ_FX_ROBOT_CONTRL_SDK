/**
 * @file showcase_ota_transfer.cpp
 * @brief 展示如何通过文件传输接口将加密的 AES-256-GCM OTA 升级包
 *        传输到 机器人控制器。
 *
 * 前置条件：
 *   1. 在运行此示例之前，先在主机 PC 上加密升级文件：
 *      @code
 *      ./host_pack_ota update_package.UPDATE <key_hex> update_package.ota
 *      @endcode
 *
 *   2. 控制器必须在以下路径部署 AES 密钥：
 *      /home/FUSION/Config/aes_key.hex
 *
 * 流程：
 *   1. 建立与控制器之间的通讯连接
 *   2. 通过 TCP 文件传输（端口 10240）发送 .ota 包
 *   3. 控制器接收文件后，自动检测 ".ota" 后缀，
 *      解密、校验 GCM Tag + 防重放，并输出：
 *      /home/FUSION/Tmp/update_package.UPDATE
 *
 * @note .ota 文件的远端路径必须以 ".ota" 结尾，控制器侧的
 *       FxOtaTryDecrypt() 才会触发自动解密。
 */

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

////'''#################################################################
//// 该 DEMO 为 OTA 加密包传输案列
////
//// 使用逻辑
////    1 查验连接是否成功
////    2 发送加密的 .ota 升级包到控制器
////    3 任务完成，释放连接
////'''#################################################################

int main(int argc, char** argv)
{
    long sdk_version = 0;               ///< SDK 版本号

    /* 解析本地 .ota 文件路径（默认或命令行参数） */
    const char* local_ota = "./update_package.ota";
    if (argc >= 2)
    {
        local_ota = argv[1];
    }

    /* 获取 SDK 版本号 */
    sdk_version = OnGetSDKVersion();
    printf("SDK version is 0x%08lx\n", sdk_version);

    /* 建立与控制器之间的通讯连接 */
    bool init = OnLinkTo(192, 168, 1, 190);
    if (!init)
    {
        std::cerr << "Failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);

    /* 发送加密 OTA 包到控制器。
     *
     * 远端路径必须以 ".ota" 结尾，控制器侧的 FXFileServer
     * 才会在 fclose() 之后触发 FxOtaTryDecrypt()。
     * 解密后的固件将写入同一目录下的 "update_package.UPDATE"
     *（路径在 ota_verify.cpp 中硬编码）。
     */
    printf("Press any key to send OTA package [%s] to controller\n", local_ota);
    getchar();

    if (!OnSendFile((char*)local_ota,(char*)"/home/FUSION/Tmp/update_package.ota"))
    {
        printf("Failed to send OTA package to controller\n");
        OnRelease();
        return -1;
    }

    printf("OTA package sent successfully.\n");
    printf("Controller will auto-decrypt to /home/FUSION/Tmp/ctrl_package.tar\n");

    /* 任务完成，释放连接使别的程序或者用户可以连接机器人 */
    SLEEP(20);
    OnRelease();

    printf("Press any key to exit\n");
    getchar();
    return 0;
}
