
#include "TCPFileClient.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CTCPFileClient::CTCPFileClient()
{
	m_inslen = sizeof(FileIns);
}

CTCPFileClient::~CTCPFileClient()
{

}

void CTCPFileClient::OnDisconnect()
{
	m_fop.SetErr();
	m_fop.Empty();
}

void CTCPFileClient::OnRecvData(char * data,long size)
{
	if(size != m_inslen)
	{
		printf("size = %ld != %ld\n",size,m_inslen);
		return;
	}
	if( m_fop.OnIns((pFileIns)data) == true)
	{
		OnSend(data,size);
	}
}



bool CTCPFileClient::OnGetFile(char * lpath,char * rpath)
{
	pFileIns ins = m_fop.OnRecvFile(lpath,rpath);
	if(ins == NULL)
	{
		return false;
	}
	if( OnSend((char *)ins,sizeof(FileIns)) == false)
	{
		free(ins);
		return false;
	}
	free(ins);
	while(m_fop.OnCheckStateOK() == false)
	{
		
		UninetSleep(50);
	}
	
	bool err = m_fop.OnCheckErrorTag();
	m_fop.OnReSetErrorTag();
	return (err == false);
}

bool CTCPFileClient::OnSendFile(char * lpath,char * rpath)
{
	pFileIns ins = m_fop.OnSendFile(lpath,rpath);
	if(ins == NULL)
	{
		return false;
	}
	if( OnSend((char *)ins,sizeof(FileIns)) == false)
	{
		free(ins);
		return false;
	}
	free(ins);
	while(m_fop.OnCheckStateOK() == false)
	{
		UninetSleep(50);
	}

	bool err = m_fop.OnCheckErrorTag();
	m_fop.OnReSetErrorTag();
	return (err == false);
}
