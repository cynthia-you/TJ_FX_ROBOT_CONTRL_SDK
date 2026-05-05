#ifndef FX_ULT_H_
#define FX_ULT_H_
#include "CmplOpt.h"
#include "FxType.h"

class CFXULT
{
public:
	static void UniMilliSleep(long millisecond);
	virtual ~CFXULT();

protected:
	CFXULT();
};

//////////////////////////////////////////////////////////////////////////////////////
/////////////////
///////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	SOCKET _local_sock;
	SOCKET _tosock_;
	struct sockaddr_in _to;
	FX_INT32 _toLen;
	struct sockaddr_in _local;
	FX_INT32 _localLen;
	FX_INT32 _from_valid;
} NetCell;

typedef struct
{
	FX_CHAR m_Recvbuf[2000];
	FX_CHAR m_SendBuf[1500];
	FX_INT32 m_Rlen;
	FX_INT32 m_Slen;
	FX_INT32 m_STag;
} NC_BUF;

class CMarvNetAgent
{
public:
	CMarvNetAgent();
	virtual ~CMarvNetAgent();
	FX_BOOL OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4, FX_INT32L port);
	FX_BOOL OnSend();
	FX_BOOL OnSendRaw();
	FX_BOOL OnSendLink();
	FX_BOOL OnRecv();
	NetCell m_nc;
	NC_BUF m_buf;
	FX_BOOL m_LinkTag;
};

//////////////////////////////////////////////////////////////////////////////////////
/////////////////
///////////////////////////////////////////////////////////////////////////////////////

class CACB
{
public:
	CACB();
	virtual ~CACB();
	FX_INT32 OnGetStoreNum();
	FX_BOOL WriteBuf(FX_UCHAR *data_ptr, FX_INT32 size_int);
	FX_INT32 ReadBuf(FX_UCHAR *data_ptr, FX_INT32 size_int);
	FX_BOOL Empty();

protected:
	FX_BOOL init_tag_;
	FX_INT32 write_pos_;
	FX_INT32 read_pos_;
	FX_UCHAR write_lock_;
	FX_UCHAR read_lock_;
	FX_UINT32 buf_serial_;
	FX_UCHAR *base_;
	FX_INT32 size_;
	FX_INT32 item_num_;
};

//////////////////////////////////////////////////////////////////////////////////////
/////////////////
///////////////////////////////////////////////////////////////////////////////////////

#endif
