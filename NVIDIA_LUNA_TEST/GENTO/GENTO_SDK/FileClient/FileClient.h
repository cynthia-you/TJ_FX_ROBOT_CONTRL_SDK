#ifndef FX_FILECLIENT_H_
#define FX_FILECLIENT_H_
#include "TCPFileClient.h"
#include "FxType.h"

class FXFileClient
{
public:
    virtual ~FXFileClient();
    static FX_BOOL OnSendFile(FX_CHAR *local_file, FX_CHAR *remote_file);
    static FX_BOOL OnRecvFile(FX_CHAR *local_file, FX_CHAR *remote_file);
    FX_BOOL SendFile(FX_CHAR *local_file, FX_CHAR *remote_file);
    FX_BOOL RecvFile(FX_CHAR *local_file, FX_CHAR *remote_file);

private:
    unsigned char m_ip1;
    unsigned char m_ip2;
    unsigned char m_ip3;
    unsigned char m_ip4;
};
#endif