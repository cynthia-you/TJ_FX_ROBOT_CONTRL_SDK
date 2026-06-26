#include "FXFileClient.h"
#include "TCPFileClient.h"

int FX_FileClient_SendFile(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, char* local_file, char* remote_file)
{
    CTCPFileClient cln;
    if (!cln.OnLinkTo(ip1, ip2, ip3, ip4, 10240))
    {
        return -1;
    }
    bool ret = cln.OnSendFile(local_file, remote_file);
    cln.OnQuit();
    if (!ret)
    {
        return -2;
    }
    return 0;
}

int FX_FileClient_RecvFile(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, char* local_file, char* remote_file)
{
    CTCPFileClient cln;
    if (!cln.OnLinkTo(ip1, ip2, ip3, ip4, 10240))
    {
        return -1;
    }
    bool ret = cln.OnRecvFile(local_file, remote_file);
    cln.OnQuit();
    if (!ret)
    {
        return -2;
    }
    return 0;
}

