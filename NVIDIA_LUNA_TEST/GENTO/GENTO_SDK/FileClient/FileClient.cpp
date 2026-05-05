
#include "FileClient.h"

static FXFileClient *FileO = NULL;

FX_BOOL FXFileClient::SendFile(FX_CHAR *local_file, FX_CHAR *remote_file)
{
    CTCPFileClient cln;
    if (cln.OnLinkTo(m_ip1, m_ip2, m_ip3, m_ip4, 10240) == FX_FALSE)
    {
        return FX_FALSE;
    }
    FX_BOOL ret = cln.OnSendFile(local_file, remote_file);
    cln.OnQuit();
    if (!ret)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FXFileClient::RecvFile(FX_CHAR *local_file, FX_CHAR *remote_file)
{

    CTCPFileClient cln;
    if (cln.OnLinkTo(m_ip1, m_ip2, m_ip3, m_ip4, 10240) == FX_FALSE)
    {
        return FX_FALSE;
    }
    FX_BOOL ret = cln.OnGetFile(local_file, remote_file);
    cln.OnQuit();
    if (!ret)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FXFileClient::OnSendFile(FX_CHAR *local_file, FX_CHAR *remote_file)
{
    if (FileO == NULL)
        return FX_FALSE;
    if (FileO->SendFile(local_file, remote_file) == FX_TRUE)
    {
        printf("[INFO]: send file local file:%s, remote file: %s\n ", local_file, remote_file);
        return FX_TRUE;
    }
    return FX_FALSE;
}

FX_BOOL FXFileClient::OnRecvFile(FX_CHAR *local_file, FX_CHAR *remote_file)
{
    if (FileO == NULL)
        return FX_FALSE;
    if (FileO->RecvFile(local_file, remote_file) == FX_TRUE)
    {
        printf("[INFO]: receve file from remote file: %s to local file:%s, \n ", remote_file, local_file);
        return FX_TRUE;
    }
    return FX_FALSE;
}
