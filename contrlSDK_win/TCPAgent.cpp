
#include "TCPAgent.h"


void UninetSleep(long usecond)
{
	
#ifdef CMPL_WIN
	Sleep(usecond);
#endif
#ifdef CMPL_LIN
	timeval tm;
	long sec;
	long usec;
	sec = usecond/1000;
	usec = (usecond%1000)*1000;
	tm.tv_sec = sec;
	
	if(usec==0)
	{
		tm.tv_usec  = 2;
	}
	else
	{
		tm.tv_usec =usec;
	}
	select(0,NULL,NULL,NULL,&tm);
#endif
}


CTCPAgent::CTCPAgent()
{
	m_bLinkTag = false;
	m_iSocket = INVALID_SOCKET;
	m_bNetInitTag = false;
	m_LoopThreadHandle = INVALID_HANDLE_VALUE;
	SrvUtText = (char * )malloc(MAX_TCP_BUFSIZE);
	m_parser = new CParser;

}

CTCPAgent::~CTCPAgent()
{
	OnQuit();
	free(SrvUtText);
	delete m_parser;
}

bool CTCPAgent::OnCheckLink()
{
	return m_bLinkTag;
}

bool CTCPAgent::OnQuit()
{
#ifdef CMPL_WIN
		if(m_iSocket != INVALID_SOCKET)
	{
		m_quit_tag = true;
		closesocket(m_iSocket);
		m_iSocket = INVALID_SOCKET;
		m_bLinkTag = false;
		while(m_quit_tag )
		{
			UninetSleep(10);
		}
	}
				
#endif
				
#ifdef CMPL_LIN
	if(m_iSocket != INVALID_SOCKET )
	{

		m_quit_tag = true;
		shutdown(m_iSocket, SHUT_RDWR);
		close(m_iSocket);
		m_iSocket = INVALID_SOCKET;
		m_bLinkTag = false;
	//	while(m_quit_tag )
		{
			UninetSleep(10);
		}
	}
#endif
	
	return true;
}

void  CTCPAgent::NetLoop(void * p)
{
	CTCPAgent * sunit = (CTCPAgent *)p;
	
	char * r;
	long   rlen;
	while(1)
	{

		long  rcvn = recv(sunit->m_iSocket,&sunit->SrvUtText[0],MAX_TCP_BUFSIZE,0); 
		
		if(rcvn <= 0)
		{//Á¬œÓ¶Ïµô
			sunit->OnDisconnect();
			sunit->m_iSocket = INVALID_SOCKET;
			sunit->m_LoopThreadHandle = INVALID_HANDLE_VALUE;
			sunit->m_bLinkTag = false;
			sunit->m_quit_tag = false;
			return;
		}
		else
		{
			sunit->m_parser->OnAddRawData(sunit->SrvUtText,rcvn);
			while(sunit->m_parser->OnUnPack() == true)
			{
				r = sunit->m_parser->OnGetContent(rlen);
				sunit->OnRecvData(r,rlen);
			}
			
		}
	}
}


void CTCPAgent::OnRecvData(char * data,long size)
{
	printf("OnRecvData %ld\n",size);
	long len=10;
	char buf[11];
	if(size < 10)
	{
		len = size;
	}
	for(long i =0 ; i < 11 ; i ++)
	{
		if(i < len)
		{
			buf[i] = data[i];
		}
		else
		{
			buf[i] = '\0';
		}
	}
	
	printf("%s\n", buf);
}



bool CTCPAgent::OnInitNet()
{
#ifdef CMPL_LIN
	m_bNetInitTag = true;
	return true;
#endif
#ifdef CMPL_WIN
	
	if(m_bNetInitTag == true)
	{
		return true;
	}
	
	WORD    wVersionRequested;
	WSADATA wsaData;
	int     err;
	
	wVersionRequested = MAKEWORD(1,1);
	err = WSAStartup( wVersionRequested, &wsaData );
	if ( err != 0 ) 
	{
		return false;
	}
	
	if ( LOBYTE( wsaData.wVersion ) != 1 ||
		HIBYTE( wsaData.wVersion ) != 1 ) 
	{
		WSACleanup();
		return false; 
	}
	
	m_bNetInitTag = true;
	return true;
#endif
}


void CTCPAgent::OnDisconnect()
{
#ifdef CMPL_LIN
	if(m_iSocket != INVALID_SOCKET )
    {
		close(m_iSocket);
		m_iSocket = INVALID_SOCKET;
		m_bLinkTag = false;	
	}
#endif
#ifdef CMPL_WIN
	if(m_iSocket != INVALID_SOCKET )
	{
		closesocket(m_iSocket);
		m_iSocket = INVALID_SOCKET;
		m_bLinkTag = false;
	}

#endif
}

bool CTCPAgent::OnLinkTo(unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4,unsigned long port)
{
	if(m_bLinkTag == true)
	{
		OnDisconnect();
	}
	
	OnInitNet();

	char ip[100];
	sprintf(ip,"%d.%d.%d.%d",ip1,ip2,ip3,ip4);


	struct sockaddr_in addrSrv;
    memset(&addrSrv, 0, sizeof(addrSrv));
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(port);  ///服务器端口
    addrSrv.sin_addr.s_addr = inet_addr(ip);  ///服务器ip
	
	//创建套接字
	m_iSocket = socket(AF_INET, SOCK_STREAM, 0);
	if(SOCKET_ERROR == m_iSocket)
	{
        printf("OnLinkTo ERR1\n");
		m_iSocket = INVALID_SOCKET;
		return false;
	}
	/*/
	struct ifreq interface;
	strncpy(interface.ifr_ifrn.ifrn_name, "eth0",strlen("eth0"));
	long tret = setsockopt(m_iSocket, SOL_SOCKET, SO_BINDTODEVICE,(char *)&interface, sizeof(interface));
    if ( tret != 0 ) 
    {
        printf("OnLinkTo ERR2  %d\n",tret);
        m_iSocket = INVALID_SOCKET;
		return false;
    }
    /*/
	//向服务器发出连接请求
	if(connect(m_iSocket, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET){
    
        printf("OnLinkTo ERR3\n");
		m_iSocket = INVALID_SOCKET;
		return false;
	}
	
	
#ifdef CMPL_WIN
	
	DWORD id;
	HANDLE reth = CreateThread(NULL,0,(unsigned long (__stdcall *)(void *))NetLoop,this,0,&id);
	if(reth == INVALID_HANDLE_VALUE)
	{
		m_iSocket = INVALID_SOCKET;
		
        printf("OnLinkTo ERR4\n");
		return false;
	}
	m_LoopThreadHandle = reth;
#endif

	
	
#ifdef CMPL_LIN
	
	
	pthread_t pthread_t_Listen;
	pthread_attr_t attrtNet;
	pthread_attr_init(&attrtNet);
	pthread_attr_setdetachstate(&attrtNet,PTHREAD_CREATE_DETACHED);
	m_LoopThreadHandle = pthread_create(&pthread_t_Listen,&attrtNet,(void *(*)(void *))NetLoop,this);
	if (m_LoopThreadHandle != 0)
	{
		pthread_detach(pthread_t_Listen);
		close(m_iSocket);
		m_iSocket = INVALID_SOCKET;
		
        printf("OnLinkTo ERR4\n");
		return false;
	}

#endif


	m_bLinkTag = true;

	return true;
}

bool CTCPAgent::OnSend(char * buf,long size)
{
	if(m_bLinkTag == false || buf == NULL || size <=0 || m_iSocket == INVALID_SOCKET)
	{
		return false;
	}
	long slen;
	char * s;
	s = m_parser->OnPack(buf,size,slen);
	if(s == NULL)
	{
		return false;
	}
	return (send(m_iSocket, s, slen, 0) != SOCKET_ERROR);

}


