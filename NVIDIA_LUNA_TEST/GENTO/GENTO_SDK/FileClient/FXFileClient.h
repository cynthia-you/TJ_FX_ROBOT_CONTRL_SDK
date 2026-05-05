#ifndef FX_FXFILECLIENT_H_
#define FX_FXFILECLIENT_H_
#include "FileClient.h"

#if defined(_WIN32) || defined(_WIN64)
#define FX_FileClient_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define FX_FileClient_SDK_API
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 发送文件到控制器. Send file to controller.
     * @param local_file  local path (absolute path or relative path)
     * @param remote_file remote path
     * @return FX_TRUE, FX_FALSE
     */
    FX_FileClient_SDK_API FX_BOOL FX_FileClient_SendFile(FX_CHAR *local_file, FX_CHAR *remote_file);

    /**
     * @brief 从控制器接收文件. Receive file from controller.
     * @param local_file  local path (absolute path or relative path)
     * @param remote_file remote path
     * @return FX_TRUE, FX_FALSE
     */
    FX_FileClient_SDK_API FX_BOOL FX_FileClient_RecvFile(FX_CHAR *local_file, FX_CHAR *remote_file);
};
#endif