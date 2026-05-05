#include "FXFileClient.h"

/*=============================================================================
 * 文件传输
 *============================================================================*/
FX_BOOL FX_FileClient_SendFile(FX_CHAR *local_file, FX_CHAR *remote_file)
{
    return FXFileClient::OnSendFile(local_file, remote_file) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_FileClient_RecvFile(FX_CHAR *local_file, FX_CHAR *remote_file)
{
    return FXFileClient::OnRecvFile(local_file, remote_file) ? FX_TRUE : FX_FALSE;
}
