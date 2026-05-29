#include "Robot.h"

static CRobot *m_InsRobot = NULL;

static bool inRelease = false;

CAxisSpPln pln_A;
CAxisSpPln pln_B;

bool CRobot::OnClearChDataA()
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB1.ReadBuf((unsigned char *)&t, si);
	while (num > 0)
	{
		num = m_InsRobot->m_ACB1.ReadBuf((unsigned char *)&t, si);
	}
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: Clear 485 cache of A arm\n");
	return true;
}

bool CRobot::OnClearChDataB()
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB2.ReadBuf((unsigned char *)&t, si);
	while (num > 0)
	{
		num = m_InsRobot->m_ACB2.ReadBuf((unsigned char *)&t, si);
	}
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: Clear 485 cache of B arm\n");
	return true;
}

long CRobot::OnGetChDataA(unsigned char data_ptr[256], long *ret_ch)
{
	if (m_InsRobot == NULL)
	{
		return 0;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB1.ReadBuf((unsigned char *)&t, si);
	if (num == 0)
	{
		return num;
	}
	memset(data_ptr, 0, 256);
	*ret_ch = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Get 485 of A arm: \nchannel =%ld\n", *ret_ch);
		printf("data:\n");
		for (int i = 0; i < 256; ++i)
		{
			printf("%02x ", data_ptr[i]);

			if ((i + 1) % 16 == 0)
			{
				printf("  ");
				for (int j = i - 15; j <= i; j++)
				{
					printf("%c", (data_ptr[j] >= 32 && data_ptr[j] <= 126) ? data_ptr[j] : '.');
				}
				printf("\n");
			}
		}
		printf("\ndata size=%d \n", t.m_Size);
	}
	return t.m_Size;
}

bool CRobot::OnSetChDataA(unsigned char *data_ptr, long size_int, long set_ch)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	if (m_InsRobot->m_ch_send_a_tag == true)
	{
		return false;
	}
	long serial = m_InsRobot->pDDSS1->m_Serial + 1;
	if (serial > 1000000)
	{
		serial = 1;
	}
	m_InsRobot->pDDSS1->m_Serial = serial;
	m_InsRobot->pDDSS1->m_Size = size_int;
	m_InsRobot->pDDSS1->m_SUB_CH = set_ch;
	memcpy(m_InsRobot->pDDSS1->m_Data, data_ptr, size_int);
	m_InsRobot->m_ch_send_a_tag = true;

	return true;
}

long CRobot::OnGetChDataB(unsigned char data_ptr[256], long *ret_ch)
{

	if (m_InsRobot == NULL)
	{
		return 0;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB2.ReadBuf((unsigned char *)&t, si);
	if (num == 0)
	{
		return num;
	}
	memset(data_ptr, 0, 256);
	*ret_ch = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Get 485 of B arm: \nchannel =%ld\n", *ret_ch);
		printf("data:\n");
		for (int i = 0; i < 256; ++i)
		{
			printf("%02x ", data_ptr[i]);
			if ((i + 1) % 16 == 0)
			{
				printf("  ");
				for (int j = i - 15; j <= i; j++)
				{
					printf("%c", (data_ptr[j] >= 32 && data_ptr[j] <= 126) ? data_ptr[j] : '.');
				}
				printf("\n");
			}
		}
		printf("\ndata size=%d \n", t.m_Size);
	}
	return t.m_Size;
}

bool CRobot::OnSetChDataB(unsigned char *data_ptr, long size_int, long set_ch)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	if (m_InsRobot->m_ch_send_b_tag == true)
	{
		return false;
	}
	long serial = m_InsRobot->pDDSS2->m_Serial + 1;
	if (serial > 1000000)
	{
		serial = 1;
	}
	m_InsRobot->pDDSS2->m_Serial = serial;
	m_InsRobot->pDDSS2->m_Size = size_int;
	m_InsRobot->pDDSS2->m_SUB_CH = set_ch;
	memcpy(m_InsRobot->pDDSS2->m_Data, data_ptr, size_int);
	m_InsRobot->m_ch_send_b_tag = true;
	return true;
}

long CRobot::OnGetSDKVersion()
{
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: SDK version %d\n", SDK_VERSION);
	return SDK_VERSION;
}

bool CRobot::OnSendPVT_A(char *local_file, long serial)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	if (serial < 0 || serial >= 100)
	{
		return false;
	}
	char remote[256];
	memset(remote, 0, 256);
	sprintf(remote, "/home/FUSION/Config/pvt/user0/%ld.txt", serial);
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: Send A arm pvt of serial=%ld to local=%s\n ", serial, local_file);
	return OnSendFile(local_file, remote);
}

bool CRobot::OnSendPVT_B(char *local_file, long serial)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	if (serial < 0 || serial >= 100)
	{
		return false;
	}
	char remote[256];
	memset(remote, 0, 256);
	sprintf(remote, "/home/FUSION/Config/pvt/user1/%ld.txt", serial);
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: Send B arm pvt of serial=%ld to local=%s\n ", serial, local_file);
	return OnSendFile(local_file, remote);
}

bool CRobot::OnSendFile(char *local_file, char *remote_file)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	if (m_InsRobot->SendFile(local_file, remote_file) == FX_TRUE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
			printf("[Marvin SDK]: send file local file:%s, remote file: %s\n ", local_file, remote_file);
		return true;
	}
	return false;
}

bool CRobot::OnRecvFile(char *local_file, char *remote_file)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	if (m_InsRobot->RecvFile(local_file, remote_file) == FX_TRUE)
	{
		return true;
	}
	return false;
}

#ifdef CMPL_WIN
void CALLBACK CallBackFunc2(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
	m_InsRobot->DoRecv();
	m_InsRobot->DoSend();
	m_InsRobot->DoCnt();
}
#endif
#ifdef CMPL_LIN
void CallBackFunc(union sigval v)
{
	m_InsRobot->DoRecv();
	m_InsRobot->DoSend();
	m_InsRobot->DoCnt();
}
#endif
bool CRobot::OnOpenShareCh(char *shm_name)
{
	GetIns();
	if (m_InsRobot->m_share_ch_tag == true)
	{
		return true;
	}

	ShmOnInit(&m_InsRobot->m_ShMem);

	int master_result = m_InsRobot->m_ShMem.OnMapMster(&m_InsRobot->m_ShMem, shm_name, 102400);
	if (FX_TRUE == master_result)
	{
		m_InsRobot->m_psm = m_InsRobot->m_ShMem.OnGetMem(&m_InsRobot->m_ShMem);
		m_InsRobot->m_ACB_ShMem.OnSetBuf(m_InsRobot->m_psm, 102400);
		m_InsRobot->m_ACB_ShMem.Empty();
		m_InsRobot->m_share_ch_tag = true;
		return true;
	}
	else
	{
		master_result = m_InsRobot->m_ShMem.OnMapSlave(&m_InsRobot->m_ShMem, shm_name);
		if (FX_TRUE == master_result)
		{
			m_InsRobot->m_psm = m_InsRobot->m_ShMem.OnGetMem(&m_InsRobot->m_ShMem);
			m_InsRobot->m_ACB_ShMem.OnSetBuf(m_InsRobot->m_psm, 102400);
			m_InsRobot->m_ACB_ShMem.Empty();
			m_InsRobot->m_share_ch_tag = true;
			return true;
		}
		else
		{
			return false;
		}
	}
}
CRobot::CRobot()
{
	m_Arm0PosCmdSendSerial = 7;
	m_Arm1PosCmdSendSerial = 7;
	m_share_ch_tag = false;
	m_send_lock = false;
	m_ch_send_a_tag = false;
	m_ch_send_b_tag = false;
	m_send_response_recv_tag = 0;
	m_send_response_timeout_cnt = 0;
	m_last_response_timeout_cnt = 0;
	m_respones_time_tag = 0;
	m_respones_time_cnt = 0;

	m_LocalLogTag = true;
	m_ParaSerial = 1;
	m_GatherTag = 0;
	m_SendTag = 0;
	miss_cnt = 0;
	old_serial_tag = FX_FALSE;
#ifdef CMPL_WIN
	m_TimeEventID = 0;
#endif

#ifdef CMPL_LIN
	robot_timer = 0;
#endif
	memset(&m_DCSS, 0, sizeof(DCSS));
	m_LastGatherTag = FX_FALSE;
	m_GatherTag = FX_FALSE;
	m_RunState = 0;
	m_LinkTag = FX_FALSE;
	_local_sock = INVALID_SOCKET;
	_tosock_ = INVALID_SOCKET;
	pDDSS1 = (DDSS *)&m_SendBuf1[2];
	memset(pDDSS1, 0, sizeof(DDSS));
	m_SendBuf1[0] = 'C';
	m_SendBuf1[1] = 'H';
	pDDSS1->m_Serial = 1;
	pDDSS1->m_CH = 1;
	pDDSS2 = (DDSS *)&m_SendBuf2[2];
	memset(pDDSS2, 0, sizeof(DDSS));
	m_SendBuf2[0] = 'C';
	m_SendBuf2[1] = 'H';
	pDDSS2->m_Serial = 1;
	pDDSS2->m_CH = 2;

	m_VersionMatchTag = FX_FALSE;
	m_ctrlSysVersion = 0;
}

bool CRobot::OnRelease()
{
	if (inRelease)
	{
		return true;
	}
	inRelease = true;
#ifdef CMPL_WIN

	if (m_InsRobot->m_TimeEventID != 0)
	{
		timeKillEvent(m_InsRobot->m_TimeEventID);
		SLEEP(10);
		m_InsRobot->m_TimeEventID = 0;
	}

	if (m_InsRobot->m_LinkTag == FX_TRUE)
	{
		closesocket(m_InsRobot->_local_sock);
		m_InsRobot->_local_sock = 0;

		closesocket(m_InsRobot->_tosock_);
		m_InsRobot->_tosock_ = 0;
	}

#endif
#ifdef CMPL_LIN
	if (m_InsRobot->robot_timer != 0)
	{
		timer_delete(m_InsRobot->robot_timer);
		SLEEP(10);
		m_InsRobot->robot_timer = 0;
	}
	if (m_InsRobot->m_LinkTag == FX_TRUE)
	{
		close(m_InsRobot->_local_sock);
		m_InsRobot->_local_sock = 0;

		close(m_InsRobot->_tosock_);
		m_InsRobot->_tosock_ = 0;
	}
#endif

	SLEEP(10);
	if (m_InsRobot->m_share_ch_tag == true)
	{
		m_InsRobot->m_ShMem.OnDest(&m_InsRobot->m_ShMem);
	}

	delete m_InsRobot;
	m_InsRobot = NULL;
	printf("[Marvin SDK]: Robot released\n");
	return true;
}

CRobot::~CRobot()
{
	OnRelease();
}

CRobot *CRobot::GetIns()
{
	if (m_InsRobot == NULL)
	{
		m_InsRobot = new CRobot();
	}
	inRelease = false;
	return m_InsRobot;
}

void CRobot::OnLocalLogOn()
{
	GetIns();
	m_InsRobot->m_LocalLogTag = true;
}

void CRobot::OnLocalLogOff()
{
	GetIns();
	m_InsRobot->m_LocalLogTag = false;
}

bool CRobot::OnGetBuf(DCSS *ret)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	memcpy(ret, &m_InsRobot->m_DCSS, sizeof(m_DCSS));
	return true;
}

bool CRobot::OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_TRUE)
	{
		return false;
	}
#ifdef CMPL_WIN
	WSADATA wsadata;
	int ret;
	ret = WSAStartup(0x101, &wsadata);
	if (ret != 0)
	{
		return false;
	}
#endif
	memset(&m_InsRobot->_local, 0, sizeof(m_InsRobot->_local));
	m_InsRobot->_localLen = sizeof(sockaddr_in);
	m_InsRobot->_local.sin_family = AF_INET;
	m_InsRobot->_local.sin_port = htons(4730);
	m_InsRobot->_local.sin_addr.s_addr = INADDR_ANY;
	m_InsRobot->_local_sock = socket(AF_INET, SOCK_DGRAM, 0);
	unsigned long on = 1;
#ifdef CMPL_WIN
	if (0 != ioctlsocket(m_InsRobot->_local_sock, FIONBIO, &on))
	{
		return false;
	}

#endif
#ifdef CMPL_LIN

	if (0 != ioctl(m_InsRobot->_local_sock, FIONBIO, &on))
	{
		return false;
	}
#endif
	if (bind(m_InsRobot->_local_sock, (struct sockaddr *)&m_InsRobot->_local, sizeof(m_InsRobot->_local)) != 0)
	{
		if (m_InsRobot->m_LocalLogTag == true)
			printf("port bind failure, possibly occupied by another program\n");
#ifdef CMPL_WIN
		closesocket(m_InsRobot->_local_sock);
#endif

#ifdef CMPL_LIN
		close(m_InsRobot->_local_sock);
#endif
		m_InsRobot->_local_sock = 0;
		m_InsRobot->m_LinkTag = FX_FALSE;
		return false;
		return false;
	}
	memset(&m_InsRobot->_to, 0, sizeof(_to));
	char ip_str[100];
	sprintf(ip_str, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
	m_InsRobot->_toLen = sizeof(sockaddr_in);
	m_InsRobot->_to.sin_family = AF_INET;
	m_InsRobot->_to.sin_port = htons(4729);
	m_InsRobot->_to.sin_addr.s_addr = inet_addr(ip_str);
	m_InsRobot->_tosock_ = socket(AF_INET, SOCK_DGRAM, 0);
	m_InsRobot->m_LinkTag = FX_TRUE;
	{
		int _localLen = sizeof(m_InsRobot->_local);
#ifdef CMPL_WIN
		int Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr *)&m_InsRobot->_local, &_localLen);
#endif
#ifdef CMPL_LIN
		int Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr *)&m_InsRobot->_local, (socklen_t *)&_localLen);
#endif
		while (Len > 0)
		{
#ifdef CMPL_WIN
			Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr *)&m_InsRobot->_local, &_localLen);
#endif
#ifdef CMPL_LIN
			Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr *)&m_InsRobot->_local, (socklen_t *)&_localLen);
#endif
		}
	}

#ifdef CMPL_WIN
	m_InsRobot->m_TimeEventID = timeSetEvent(1, 1, CallBackFunc2, (DWORD)NULL, TIME_PERIODIC);
#endif
#ifdef CMPL_LIN
	{
		struct sigevent evp;
		struct itimerspec ts;
		int ret;
		memset(&evp, 0, sizeof(evp));
		evp.sigev_value.sival_ptr = &m_InsRobot->robot_timer;
		evp.sigev_notify = SIGEV_THREAD;
		evp.sigev_notify_function = CallBackFunc;
		evp.sigev_value.sival_int = 0;
		ret = timer_create(CLOCK_REALTIME, &evp, &m_InsRobot->robot_timer);
		if (ret)
		{
			return false;
		}
		ts.it_interval.tv_sec = 0;
		ts.it_interval.tv_nsec = 1000000;
		ts.it_value.tv_sec = 0;
		ts.it_value.tv_nsec = 1000000;
		ret = timer_settime(m_InsRobot->robot_timer, TIMER_ABSTIME, &ts, NULL);
		if (ret)
		{
			return false;
		}
	}
#endif
	// m_InsRobot->ReadPendingData();
	m_InsRobot->m_RunState = 0;
	m_InsRobot->m_ip1 = ip1;
	m_InsRobot->m_ip2 = ip2;
	m_InsRobot->m_ip3 = ip3;
	m_InsRobot->m_ip4 = ip4;

	char name[30];
	memset(name, 0, 30);
	sprintf(name, "VERSION");
	long ctrlSysVers = 0;
    long int cnt = 5;
	while (cnt > 0)
	{
		if(0 != OnGetIntPara(name, &ctrlSysVers) )
		{
			cnt--;
			continue;
		}
		else
		{
			break;
		}
	}
	OnGetIntPara(name, &ctrlSysVers);
	m_InsRobot->m_ctrlSysVersion = ctrlSysVers;
	int SDKVer = SDK_VERSION;
	if ((ctrlSysVers / 1000) != (SDKVer / 1000))
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n"
				   "                      Some functions may not work properly\n",
				   __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		m_InsRobot->m_VersionMatchTag = FX_FALSE;
	}
	else
	{
		m_InsRobot->m_VersionMatchTag = FX_TRUE;
	}

	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Robot connected  IP=%d.%d.%d.%d\n", ip1, ip2, ip3, ip4);
	}
	return true;
}

void CRobot::ReadPendingData()
{
	if (!m_InsRobot)
	{
		return;
	}
	char buffer[2048];
	sockaddr_in fromAddr;
	socklen_t fromLen = sizeof(fromAddr);
	int dataCount = 0;
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Checking for pending data...\n");
	}
	while (true)
	{
		memset(buffer, 0, sizeof(buffer));
		memset(&fromAddr, 0, sizeof(fromAddr));
		fromLen = sizeof(fromAddr);

		int len = recvfrom(m_InsRobot->_local_sock, buffer, sizeof(buffer) - 1, 0,
						   (struct sockaddr *)&fromAddr, &fromLen);
		if (len <= 0)
		{
			break;
		}
		dataCount++;
		buffer[len] = '\0';
		if (m_InsRobot->m_LocalLogTag == true)
		{
			char fromIp[32];
#ifdef CMPL_WIN
			sprintf(fromIp, "%s", inet_ntoa(fromAddr.sin_addr));
#endif

#ifdef CMPL_LIN
			inet_ntop(AF_INET, &(fromAddr.sin_addr), fromIp, sizeof(fromIp));
#endif
			printf("[Marvin SDK]: Read pending data %d bytes from %s:%d\n",
				   len, fromIp, ntohs(fromAddr.sin_port));
		}
	}
	if (m_InsRobot->m_LocalLogTag == true && dataCount > 0)
	{
		printf("[Marvin SDK]: Read %d pending packets\n", dataCount);
	}
}

long CRobot::OnWriteRaw(unsigned char ins, int size, unsigned char *buf)
{
	// INS LEN CRC CRC DATA
	if (size < 0)
	{
		return -2;
	}
	long data_size = size;
	long add_size = 5 + data_size;
	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}
	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;
	for (long i = 0; i < size; i++)
	{
		m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i] = buf[i];
		crc += buf[i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnWriteInt16(unsigned char ins, int size, int *data)
{
	if (size < 0)
	{
		return -2;
	}
	long data_size = sizeof(FX_INT16) * size;

	long add_size = 5 + data_size;

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;

	FX_INT16 *pdata = (FX_INT16 *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5];

	long i;
	for (i = 0; i < size; i++)
	{
		pdata[i] = data[i];
	}

	for (long i = 0; i < data_size; i++)
	{
		crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnWriteInt32(unsigned char ins, int size, int *data)
{
	if (size < 0)
	{
		return -2;
	}
	long data_size = sizeof(FX_INT32) * size;

	long add_size = 5 + data_size;

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;

	FX_INT32 *pdata = (FX_INT32 *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5];

	long i;
	for (i = 0; i < size; i++)
	{
		pdata[i] = data[i];
	}

	for (long i = 0; i < data_size; i++)
	{
		crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnWriteFloat(unsigned char ins, int size, double *data)
{
	if (size < 0)
	{
		return -2;
	}
	long data_size = sizeof(FX_FLOAT) * size;

	long add_size = 5 + data_size;

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;

	FX_FLOAT *pdata = (FX_FLOAT *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5];

	long i;
	for (i = 0; i < size; i++)
	{
		pdata[i] = data[i];
	}

	for (i = 0; i < data_size; i++)
	{
		crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnWriteIntFloat(unsigned char ins, int intvalue, int size, double *data)
{
	if (size < 0)
	{
		return -2;
	}
	long data_size = sizeof(FX_FLOAT) * size + sizeof(FX_INT32);

	long add_size = 5 + data_size;

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;

	FX_INT32 *ipdata = (FX_INT32 *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5];
	*ipdata = intvalue;
	FX_FLOAT *pdata = (FX_FLOAT *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + sizeof(FX_INT32)];

	long i;
	for (i = 0; i < size; i++)
	{
		pdata[i] = data[i];
	}

	for (i = 0; i < data_size; i++)
	{
		crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnWriteInt32Float(unsigned char ins, int size_i, int *idata, int size_f, double *fdata)
{
	if (idata == NULL)
	{
		return -2;
	}
	if (fdata == NULL)
	{
		return -2;
	}
	long data_size = sizeof(FX_FLOAT) * size_f + sizeof(FX_INT32) * size_i;

	long add_size = 5 + data_size;

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;

	FX_INT32 *ipdata = (FX_INT32 *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5];
	FX_FLOAT *fpdata = (FX_FLOAT *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + sizeof(FX_INT32) * size_i];

	long i;
	for (i = 0; i < size_i; i++)
	{
		ipdata[i] = idata[i];
	}
	for (i = 0; i < size_f; i++)
	{
		fpdata[i] = fdata[i];
	}

	for (long i = 0; i < data_size; i++)
	{
		crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnWriteFloatInt32(unsigned char ins, int size_f, double *fdata, int size_i, int *idata)
{
	if (idata == NULL)
	{
		return -2;
	}
	if (fdata == NULL)
	{
		return -2;
	}
	long data_size = sizeof(FX_FLOAT) * size_f + sizeof(FX_INT32) * size_i;

	long add_size = 5 + data_size;

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	unsigned char crc = 0;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = ins;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1] = data_size / 256;
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2] = data_size % 256;
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 1];
	crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 2];
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 3] = 256 - crc;
	crc = 0;

	FX_INT32 *ipdata = (FX_INT32 *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + sizeof(FX_FLOAT) * size_f];
	FX_FLOAT *fpdata = (FX_FLOAT *)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5];

	long i;
	for (i = 0; i < size_i; i++)
	{
		ipdata[i] = idata[i];
	}
	for (i = 0; i < size_f; i++)
	{
		fpdata[i] = fdata[i];
	}

	for (long i = 0; i < data_size; i++)
	{
		crc += m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 5 + i];
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen + 4] = 256 - crc;
	m_InsRobot->m_Slen += (5 + data_size);
	{
		m_InsRobot->m_SendBuf[2] += 1;
		long frm_data_len = m_InsRobot->m_Slen - 7;
		m_InsRobot->m_SendBuf[3] = frm_data_len % 256;
		m_InsRobot->m_SendBuf[4] = frm_data_len / 256;

		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	return 0;
}

long CRobot::OnDealData(char paraName[30], unsigned char type, unsigned char dins, float *fdata, int *idata, long &serial)
{
	if (paraName[29] != 0)
	{
		return -1;
	}

	/*/

	if(type != 1 && type != 2)
	{
		return -2;
	}

	if(dins != 101 && dins != 102 && dins != 103 && dins != 104)
	{
		return -3;
	}

	if((dins == 101 || dins = 103 ) && type ==2)
	{
		return -4;
	}

	if((dins == 103 || dins = 104 ) && type ==1)
	{
		return -5;
	}
	/*/

	FX_UCHAR m_SendBuf[100];
	long m_Slen = 0;

	FX_CHAR *pName = (FX_CHAR *)&m_SendBuf[m_Slen];
	m_Slen += 30 * sizeof(FX_CHAR);
	FX_CHAR *pType = (FX_CHAR *)&m_SendBuf[m_Slen];
	m_Slen += sizeof(FX_CHAR);
	FX_CHAR *pIns = (FX_CHAR *)&m_SendBuf[m_Slen];
	m_Slen += sizeof(FX_CHAR);
	FX_INT32 *pValueInt = (FX_INT32 *)&m_SendBuf[m_Slen];
	m_Slen += sizeof(FX_INT32);
	FX_FLOAT *pValueFloat = (FX_FLOAT *)&m_SendBuf[m_Slen];
	m_Slen += sizeof(FX_FLOAT);
	FX_INT16 *pSerial = (FX_INT16 *)&m_SendBuf[m_Slen];
	m_Slen += sizeof(FX_INT16);
	long i;
	for (i = 0; i < 30; i++)
	{
		pName[i] = 0;
		if (paraName[i] != 0)
		{
			pName[i] = paraName[i];
		}
	}
	m_InsRobot->m_ParaSerial++;
	if (m_InsRobot->m_ParaSerial >= 99)
	{
		m_InsRobot->m_ParaSerial = 1;
	}
	serial = m_InsRobot->m_ParaSerial;
	*pType = type;
	*pIns = dins;
	*pValueInt = 0;
	*pValueFloat = 0;

	*pSerial = serial;
	if (dins == 101 || dins == 102)
	{
		if (dins == 101)
		{
			*pValueInt = *idata;
		}
		else
		{
			*pValueFloat = *fdata;
		}
	}

	return OnWriteRaw(150, m_Slen, m_SendBuf);
}

long CRobot::OnSetIntPara(char paraName[30], long setValue)
{
	if (paraName[29] != 0)
	{
		return -1;
	}
	if (OnClearSet() == false)
	{
		return -1;
	}
	long serial = 0;
	long i;
	FX_FLOAT fvalue = 0;
	FX_INT32 ivalue = setValue;
	long ret = OnDealData(paraName, 1, 101, &fvalue, &ivalue, serial);
	if (ret != 0)
	{
		return ret;
	}

	CRobot::OnSetSend();
	for (i = 0; i < 50; i++)
	{
		SLEEP(2);
		long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
		if (ret_s % 100 == serial)
		{
			long ret_v = ret_s / 100;
			return ret_v;
		}
	}
	return -2;
}

long CRobot::OnSetFloatPara(char paraName[30], double setValue)
{
	if (paraName[29] != 0)
	{
		return -1;
	}
	if (OnClearSet() == false)
	{
		return -1;
	}
	long serial = 0;
	long i;
	FX_FLOAT fvalue = setValue;
	FX_INT32 ivalue = 0;
	long ret = OnDealData(paraName, 2, 102, &fvalue, &ivalue, serial);
	if (ret != 0)
	{
		return ret;
	}

	CRobot::OnSetSend();
	for (i = 0; i < 50; i++)
	{
		SLEEP(2);
		long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
		if (ret_s % 100 == serial)
		{
			long ret_v = ret_s / 100;
			return ret_v;
		}
	}
	return -2;
}

long CRobot::OnGetIntPara(char paraName[30], long *retValue)
{
	if (paraName[29] != 0)
	{
		return -1;
	}

	if (OnClearSet() == false)
	{
		return -1;
	}
	long serial = 0;
	long i;
	FX_FLOAT fvalue = 0;
	FX_INT32 ivalue = 0;
	long ret = OnDealData(paraName, 1, 103, &fvalue, &ivalue, serial);
	if (ret != 0)
	{
		return ret;
	}
	CRobot::OnSetSend();
	for (i = 0; i < 50; i++)
	{
		SLEEP(2);
		long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
		if (ret_s % 100 == serial)
		{
			long ret_v = ret_s / 100;
			if (ret_v == 0)
			{
				*retValue = m_InsRobot->m_DCSS.m_ParaValueI;
			}
			if (m_InsRobot->m_LocalLogTag == true)
			{
				if (retValue != nullptr)
				{
					printf("[Marvin SDK]: Get int parameter: %s, value=%ld\n", paraName, *retValue);
				}
				else
				{
					printf("[Marvin SDK] retValue is null pointer.\n");
				}
			}
			return ret_v;
		}
	}
	return -2;
}

long CRobot::OnGetFloatPara(char paraName[30], double *retValue)
{
	if (paraName[29] != 0)
	{
		return -1;
	}
	if (OnClearSet() == false)
	{
		return -1;
	}

	long serial = 0;
	FX_FLOAT fvalue = 0;
	FX_INT32 ivalue = 0;
	long ret = OnDealData(paraName, 2, 104, &fvalue, &ivalue, serial);
	if (ret != 0)
	{
		return ret;
	}

	long i;
	CRobot::OnSetSend();
	for (i = 0; i < 50; i++)
	{
		SLEEP(2);
		long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
		if (ret_s % 100 == serial)
		{
			long ret_v = ret_s / 100;
			if (ret_v == 0)
			{
				*retValue = m_InsRobot->m_DCSS.m_ParaValueF;
			}
			if (m_InsRobot->m_LocalLogTag == true)
			{
				if (retValue != nullptr)
				{
					printf("[Marvin SDK]: Get float parameter: %s, value=%lf\n", paraName, *retValue);
				}
				else
				{
					printf("[Marvin SDK] retValue is null pointer.\n");
				}
			}
			return ret_v;
		}
	}
	return -2;
}

long CRobot::OnSavePara()
{
	if (OnClearSet() == false)
	{
		return -1;
	}
	long add_size = 1 + sizeof(FX_CHAR) * 32 + sizeof(FX_INT32) + sizeof(FX_FLOAT) + sizeof(FX_INT16);
	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return -1;
	}

	long serial = 0;
	FX_FLOAT fvalue = 0;
	FX_INT32 ivalue = 0;
	char paraName[30] = {0};
	long ret = OnDealData(paraName, 0, 105, &fvalue, &ivalue, serial);
	if (ret != 0)
	{
		return ret;
	}

	long i;
	CRobot::OnSetSend();
	for (i = 0; i < 50; i++)
	{
		SLEEP(2);
		long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
		if (ret_s % 100 == serial)
		{
			long ret_v = ret_s / 100;
			return ret_v;
		}
	}
	return -2;
}

void CRobot::DoCnt()
{
	if (m_send_response_timeout_cnt > 0)
	{
		if (m_last_response_timeout_cnt == 0)
		{
			m_respones_time_tag = 0;
			m_respones_time_cnt = 0;
		}
		m_send_response_timeout_cnt--;
		m_last_response_timeout_cnt = m_send_response_timeout_cnt;
		if (m_send_response_local_tag != m_send_response_recv_tag)
		{
			m_respones_time_cnt++;
		}
		else
		{
			m_respones_time_tag = 1;
			m_last_response_timeout_cnt = 0;
			m_send_response_timeout_cnt = 0;
		}
	}
}

void CRobot::DoRecv()
{
	if (m_send_response_timeout_cnt > 0)
	{
		m_send_response_timeout_cnt--;
	}
	if (m_LinkTag == FX_FALSE)
	{
		return;
	}
	_localLen = sizeof(_local);
#ifdef CMPL_WIN
	int Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr *)&_local, &_localLen);
#endif
#ifdef CMPL_LIN
	int Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr *)&_local, (socklen_t *)&_localLen);
#endif
	while (Len > 0)
	{
		if (Len == sizeof(DCSS) + 2)
		{
			if (recvbuf[0] == 'F' && recvbuf[1] == 'X')
			{
				if (m_InsRobot->m_share_ch_tag == true)
				{
					m_ACB_ShMem.WriteBuf((unsigned char *)recvbuf, Len);
				}

				DCSS *p = (DCSS *)&recvbuf[2];
				memcpy(&m_DCSS, p, sizeof(m_DCSS));
				m_send_response_recv_tag = m_DCSS.m_Out[0].m_pad[0];
				if (m_InsRobot->m_GatherTag == 1)
				{
					if (m_GatherRecordNum >= m_GatherRecordMaxNum)
					{
						m_GatherTag = 4;
					}
					else
					{
						double v[40];
						for (long i = 0; i < m_GatherItemSize; i++)
						{
							v[i + 2] = *m_GatherItem[i];
						}
						v[0] = m_DCSS.m_Out[0].m_OutFrameSerial;
						v[1] = m_DCSS.m_Out[1].m_OutFrameSerial;
						m_GatherSet.OnSetPoint(v);
						m_GatherRecordNum++;
					}
				}
				if (m_InsRobot->m_GatherTag == 2)
				{
					m_InsRobot->m_GatherTag = 4;
				}
				if (old_serial_tag == FX_FALSE)
				{
					old_serial_tag = true;
					old_serial = m_DCSS.m_Out[0].m_OutFrameSerial;
				}
				else
				{
					old_serial += 1;
					old_serial %= 1000000;
					if (old_serial != m_DCSS.m_Out[0].m_OutFrameSerial)
					{
						miss_cnt++;
						old_serial = m_DCSS.m_Out[0].m_OutFrameSerial;
					}
					else
					{
						miss_cnt = 0;
					}
				}
			}
		}
		else if (Len == sizeof(DDSS) + 2)
		{
			if (recvbuf[0] == 'C' && recvbuf[1] == 'H')
			{
				DDSS *p = (DDSS *)&recvbuf[2];
				if (p->m_CH == 1)
				{
					m_ACB1.WriteBuf((unsigned char *)p, sizeof(DDSS));
				}
				if (p->m_CH == 2)
				{
					m_ACB2.WriteBuf((unsigned char *)p, sizeof(DDSS));
				}
			}
		}
#ifdef CMPL_WIN
		Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr *)&_local, &_localLen);
#endif
#ifdef CMPL_LIN
		Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr *)&_local, (socklen_t *)&_localLen);
#endif
	}
}

void CRobot::DoSend()
{
	if (m_send_lock == true)
	{
		return;
	}

	m_send_lock = true;
	if (m_SendTag == 100)
	{

		sendto(_tosock_, (char *)m_SendBuf, m_Slen, 0, (struct sockaddr *)&_to, sizeof(_to));
		m_SendTag = 0;
		m_Slen = 0;
	}

	if (m_ch_send_a_tag == true)
	{
		sendto(m_InsRobot->_tosock_, (char *)m_InsRobot->m_SendBuf1, sizeof(DDSS) + 2, 0, (struct sockaddr *)&m_InsRobot->_to, sizeof(m_InsRobot->_to));
		m_ch_send_a_tag = false;
	}

	if (m_ch_send_b_tag == true)
	{
		sendto(m_InsRobot->_tosock_, (char *)m_InsRobot->m_SendBuf2, sizeof(DDSS) + 2, 0, (struct sockaddr *)&m_InsRobot->_to, sizeof(m_InsRobot->_to));
		m_ch_send_b_tag = false;
	}

	m_send_lock = false;
}

bool CRobot::OnStopGather()
{
	GetIns();
	if (m_InsRobot->m_LinkTag == false)
	{
		return false;
	}
	if (m_InsRobot->m_GatherTag != 1)
	{
		return false;
	}
	m_InsRobot->m_GatherTag = 2;
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Stop collect data\n");
	}
	return true;
}

bool CRobot::OnSaveGatherData(char *path)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == false)
	{
		return false;
	}
	if (m_InsRobot->m_GatherTag != 4)
	{
		return false;
	}
	bool ret = m_InsRobot->m_GatherSet.OnSave(path);
	m_InsRobot->m_GatherTag = 0;
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Save collected data to %s\n", path);
	}
	return ret;
}

bool CRobot::OnSaveGatherDataCSV(char *path)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == false)
	{
		return false;
	}
	if (m_InsRobot->m_GatherTag != 4)
	{
		return false;
	}
	bool ret = m_InsRobot->m_GatherSet.OnSaveCSV(path);
	m_InsRobot->m_GatherTag = 0;
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Collected csv saved path=%s\n", path);
	}
	return ret;
}

bool CRobot::OnStartGather(long targetNum, long targetID[35], long recordNum)
{
	GetIns();
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Collect data settings\n");
		printf("targetNum=%ld\n", targetNum);
		printf("targetID= [%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]\n",
			   targetID[0], targetID[1], targetID[2], targetID[3], targetID[4], targetID[5], targetID[6],
			   targetID[7], targetID[8], targetID[9], targetID[10], targetID[11], targetID[12], targetID[13],
			   targetID[14], targetID[15], targetID[16], targetID[17], targetID[18], targetID[19], targetID[20],
			   targetID[21], targetID[22], targetID[23], targetID[24], targetID[25], targetID[26], targetID[27],
			   targetID[28], targetID[29], targetID[30], targetID[31], targetID[32], targetID[33], targetID[34]);
		printf("recordNum=%ld\n", recordNum);
	}
	if (m_InsRobot->m_LinkTag == false)
	{
		return false;
	}
	if (targetNum <= 0)
	{
		return false;
	}
	if (targetNum >= 35)
	{
		targetNum = 35;
	}
	if (m_InsRobot->m_GatherTag == 1)
	{
		return false;
	}
	long i;
	for (i = 0; i < targetNum; i++)
	{
		long v = targetID[i];
		long rob = v / 100;
		if (rob > 1)
		{
			return false;
		}
		long grp = (v % 100) / 10;
		long pos = v % 10;
		if (pos > 7)
		{
			return false;
		}
		if (grp == 9 && pos > 6)
		{
			return false;
		}
		if (rob == 0)
		{
			if (grp == 0)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_Pos[pos];
			}
			else if (grp == 1)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_Vel[pos];
			}
			else if (grp == 2)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_PosE[pos];
			}
			else if (grp == 3)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_Cmd[pos];
			}
			else if (grp == 4)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_CToq[pos];
			}
			else if (grp == 5)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_SToq[pos];
			}
			else if (grp == 6)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_EST_Joint_Firc[pos];
			}
			else if (grp == 7)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_EST_Joint_Firc_Dot[pos];
			}
			else if (grp == 8)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_EST_Joint_Force[pos];
			}
			else if (grp == 9)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[0].m_EST_Cart_FN[pos];
			}
		}
		else
		{
			if (grp == 0)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_FB_Joint_Pos[pos];
			}
			else if (grp == 1)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_FB_Joint_Vel[pos];
			}
			else if (grp == 2)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_FB_Joint_PosE[pos];
			}
			else if (grp == 3)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_FB_Joint_Cmd[pos];
			}
			else if (grp == 4)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_FB_Joint_CToq[pos];
			}
			else if (grp == 5)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_FB_Joint_SToq[pos];
			}
			else if (grp == 6)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_EST_Joint_Firc[pos];
			}
			else if (grp == 7)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_EST_Joint_Firc_Dot[pos];
			}
			else if (grp == 8)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_EST_Joint_Force[pos];
			}
			else if (grp == 9)
			{
				m_InsRobot->m_GatherItem[i] = &m_InsRobot->m_DCSS.m_Out[1].m_EST_Cart_FN[pos];
			}
		}
	}
	m_InsRobot->m_GatherItemSize = targetNum;
	m_InsRobot->m_GatherRecordNum = 0;
	if (recordNum < 1000)
	{
		recordNum = 1000;
	}
	if (recordNum > 5000000 / targetNum)
	{
		recordNum = 5000000 / targetNum;
	}
	if (recordNum > 1000000)
	{
		recordNum = 1000000;
	}
	m_InsRobot->m_GatherRecordMaxNum = recordNum;
	int t = (targetNum + 2);
	m_InsRobot->m_GatherSet.OnInit((PoinType)t, recordNum);
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Data collected, targetNum=%ld, recordNum=%ld\n", targetNum, recordNum);
		printf("targetID= [%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]\n",
			   targetID[0], targetID[1], targetID[2], targetID[3], targetID[4], targetID[5], targetID[6],
			   targetID[7], targetID[8], targetID[9], targetID[10], targetID[11], targetID[12], targetID[13],
			   targetID[14], targetID[15], targetID[16], targetID[17], targetID[18], targetID[19], targetID[20],
			   targetID[21], targetID[22], targetID[23], targetID[24], targetID[25], targetID[26], targetID[27],
			   targetID[28], targetID[29], targetID[30], targetID[31], targetID[32], targetID[33], targetID[34]);
	}
	m_InsRobot->m_GatherTag = true;
	return true;
}

bool CRobot::OnClearSet()
{
	if (m_InsRobot->m_SendTag == 100 || m_InsRobot->m_SendTag == 50)
	{
		return false;
	}
	m_InsRobot->m_SendTag = 0;
	m_InsRobot->m_SendBuf[0] = 'F';
	m_InsRobot->m_SendBuf[1] = 'Y';
	m_InsRobot->m_SendBuf[2] = 0; // INS number
	m_InsRobot->m_SendBuf[3] = 0; // LSB of data len
	m_InsRobot->m_SendBuf[4] = 0; // MSB of data len
	m_InsRobot->m_SendBuf[5] = 0; // CRC of Head
	m_InsRobot->m_SendBuf[6] = 0; // CRC of data

	{
		unsigned char addv = 0;
		addv += m_InsRobot->m_SendBuf[0];
		addv += m_InsRobot->m_SendBuf[1];
		addv += m_InsRobot->m_SendBuf[2];
		addv += m_InsRobot->m_SendBuf[3];
		addv += m_InsRobot->m_SendBuf[4];
		m_InsRobot->m_SendBuf[5] = 256 - addv;
	}

	m_InsRobot->m_Slen = 7;

	return true;
}

bool CRobot::OnSetJointLmt_A(int velRatio, int AccRatio)
{
	int data[10] = {0};
	int v = velRatio;
	int a = AccRatio;
	if (v < 1)
		v = 1;
	if (v > 100)
		v = 100;
	if (a < 1)
		a = 1;
	if (a > 100)
		a = 100;
	data[0] = v;
	data[1] = a;
	long ret = OnWriteInt16(103, 2, data);
	if (ret != 0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetPVT_A(int id)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}

	if (id < 0 || id > 255)
	{
		return false;
	}

	FX_UCHAR buf[10] = {0};

	buf[0] = id;

	long ret = OnWriteRaw(110, 1, buf);

	if (ret != 0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetPVT_B(int id)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	if (id < 0 || id > 255)
	{
		return false;
	}

	FX_UCHAR buf[10] = {0};

	buf[0] = id;

	long ret = OnWriteRaw(210, 1, buf);

	if (ret != 0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetForceCmd_A(double force)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	double data[10] = {0};
	data[0] = force;
	long ret = OnWriteFloat(109, 1, data);
	return (ret == 0);
}

bool CRobot::OnSetJointCmdPos_A(double joint[7])
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}

	double data[10] = {0};
	data[0] = joint[0];
	data[1] = joint[1];
	data[2] = joint[2];
	data[3] = joint[3];
	data[4] = joint[4];
	data[5] = joint[5];
	data[6] = joint[6];

	m_InsRobot->m_Arm0PosCmdSendSerial++;
	if (m_InsRobot->m_Arm0PosCmdSendSerial > 9999)
	{
		m_InsRobot->m_Arm0PosCmdSendSerial = 7;
	}
	// long ret = OnWriteFloat(108, 7, data);
	long ret = OnWriteIntFloat(108, m_InsRobot->m_Arm0PosCmdSendSerial, 7, data);
	return (ret == 0);
}

bool CRobot::OnInitPlnLmt(char *path)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	double vel[8] = {0.0};
	double acc[8] = {0.0};
	double npos[8] = {0.0};
	double ppos[8] = {0.0};
	long TYPE[2];
	double GRV[2][3];
	double DH[2][8][4];
	double NPVA[2][7][4];
	double BD[2][4][3];
	double Mass[2][7];
	double MCP[2][7][3];
	double I[2][7][6];
	char c;
	long i = 0;
	long j = 0;
	FILE *fp = fopen(path, "rb");
	if (fp == NULL)
	{
		return false;
	}
	for (i = 0; i < 2; i++)
	{
		if (fscanf(fp, "%ld,%lf,%lf,%lf,%c", &TYPE[i], &GRV[i][0], &GRV[i][1], &GRV[i][2], &c) != 5)
		{
			fclose(fp);
			return false;
		}
		if (c != 0x0a)
		{
			fclose(fp);
			return false;
		}
		for (j = 0; j < 7; j++)
		{
			if (fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%c",
					   &DH[i][j][0], &DH[i][j][1], &DH[i][j][2], &DH[i][j][3],
					   &NPVA[i][j][0], &NPVA[i][j][1], &NPVA[i][j][2], &NPVA[i][j][3],
					   &Mass[i][j], &MCP[i][j][0], &MCP[i][j][1], &MCP[i][j][2],
					   &I[i][j][0], &I[i][j][1], &I[i][j][2], &I[i][j][3], &I[i][j][4], &I[i][j][5],
					   &c) != 19)
			{
				fclose(fp);
				return false;
			}
			if (c != 0x0a)
			{
				fclose(fp);
				return false;
			}
		}
		if (fscanf(fp, "%lf,%lf,%lf,%lf,%c", &DH[i][7][0], &DH[i][7][1], &DH[i][7][2], &DH[i][7][3],
				   &c) != 5)
		{
			fclose(fp);
			return false;
		}
		if (c != 0x0a)
		{
			fclose(fp);
			return false;
		}
		for (j = 0; j < 4; j++)
		{
			if (fscanf(fp, "%lf,%lf,%lf,%c",
					   &BD[i][j][0], &BD[i][j][1], &BD[i][j][2], &c) != 4)
			{
				fclose(fp);
				return false;
			}
			if (c != 0x0a)
			{
				fclose(fp);
				return false;
			}
		}
	}
	fclose(fp);

	for (j = 0; j < 7; j++)
	{
		npos[j] = NPVA[0][j][0];
		ppos[j] = NPVA[0][j][1];
		vel[j] = NPVA[0][j][2];
		acc[j] = NPVA[0][j][3];
	}
	{
		pln_A.OnSetLmt(7, npos, ppos, vel, acc);
	}

	for (j = 0; j < 7; j++)
	{
		npos[j] = NPVA[1][j][0];
		ppos[j] = NPVA[1][j][1];
		vel[j] = NPVA[1][j][2];
		acc[j] = NPVA[1][j][3];
	}
	{
		pln_B.OnSetLmt(7, npos, ppos, vel, acc);
	}
	return true;
}

bool CRobot::OnSetPlnCart_A(CPointSet *pset)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	DCSS t;
	long num = pset->OnGetPointNum();
	if (num <= 5)
	{
		printf("[ERROR] OnSetPlnCart_A: there are fewer than 5 planned points, cannot operate \n");
		return false;
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_A(num);
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetTrajInit_A: OnSetSendWaitResponse timeout\n");
		return false;
	}
	SLEEP(SLEEP_TIME);
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState != 1)
		{
			printf("[ERROR] OnSetTrajInit_A: The controller has not entered planning mode\n");
			return false;
		}
	}
	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii, jj, kk;
	double SendData[350];
	double *retp;
	long spos;
	long ipos = 0;
	for (ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for (jj = 0; jj < 50; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(ii, 50, SendData);

		unsigned char buf[1500] = {0};
		long buf_len = 1500;

		if (CRobot::OnSetSendWaitResponse2(TIME_OUT, buf, buf_len) < 0)
		{
			printf("[ERROR] OnSetTrajSet_A: OnSetSendWaitResponse timeout\n");
			return false;
		}
	}
	if (relic_num != 0)
	{
		spos = 0;
		for (jj = 0; jj < relic_num; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(send_g_num, relic_num, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_A: OnSetSendWaitResponse timeout\n");
			return false;
		}
	}
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState != 2)
		{
			printf("[ERROR] OnSetTrajSet_A: The controller did not receive the sent trajectory\n");
			return false;
		}
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_A();
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetTrajRun_A: OnSetSendWaitResponse timeout\n");
		return false;
	}
	return true;
}

bool CRobot::OnSetPlnJoint_A(double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}

	DCSS t;
	double vr = vel_ratio;
	double ar = acc_ratio;
	if (vr < 0.01)
		vr = 0.01;
	if (vr > 1.0)
		vr = 1.0;
	if (ar < 0.01)
		ar = 0.01;
	if (ar > 1.0)
		ar = 1.0;
	long i = 0;
	double sta[8] = {0};
	double sto[8] = {0};
	for (i = 0; i < 7; i++)
	{
		sta[i] = start_joints[i];
		sto[i] = stop_joints[i];
	}
	long num = pln_A.OnPln(sta, sto, vr, ar);
	if (num <= 0)
	{
		printf("[ERROR] OnSetPlnJoint_A: planning failed, please check start joints and end joints\n");
		return false;
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_A(num);
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetPlnJoint_A: OnSetSendWaitResponse timeout");
		return false;
	}
	SLEEP(1000);
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState != 1)
		{
			printf("[ERROR] OnSetTrajInit_A: The controller has not entered planning mode\n");
			return false;
		}
	}
	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii, jj, kk;
	double SendData[350];
	double retp[8];
	long spos;
	for (ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for (jj = 0; jj < 50; jj++)
		{
			pln_A.OnCut(retp);
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(ii, 50, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_A: OnSetSendWaitResponse timeout");
			return false;
		}
	}
	if (relic_num != 0)
	{
		spos = 0;
		for (jj = 0; jj < relic_num; jj++)
		{
			pln_A.OnCut(retp);
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(send_g_num, relic_num, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_A: OnSetSendWaitResponse timeout");
			return false;
		}
	}
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState != 2)
		{
			printf("[ERROR] OnSetTrajSet_A: The controller did not receive the sent trajectory\n");
			return false;
		}
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_A();
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetTrajRun_A: OnSetSendWaitResponse timeout");
		return false;
	}
	return true;
}

bool CRobot::OnSetTrajInit_A(int pointNum)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	if (pointNum < 0)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK]: Initial A arm Planning Trajectory failed, number of point=%d,\n", pointNum);
		}
		return false;
	}
	int data[10] = {0};
	data[0] = pointNum;
	long ret = OnWriteInt32(112, 1, data);
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Initial A arm Planning Trajectory , number of point=%d,\n", pointNum);
	}
	return (ret == 0);
}

bool CRobot::OnSetTrajRun_A()
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int data[10] = {0};
	long ret = OnWriteInt32(114, 0, data);
	return (ret == 0);
}

bool CRobot::OnStopPlnJoint_A()
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	CRobot::OnClearSet();
	CRobot::OnStopPlnJoint_interA();
	if (CRobot::OnSetSendWaitResponse(50) < 0)
	{
		printf("[ERROR] OnStopPlnJoint_interA: OnSetSendWaitResponse timeout\n");
		return false;
	}
	else
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK]: A arm stop run Planning Trajectory \n");
		}
		return true;
	}
}

bool CRobot::OnStopPlnJoint_interA()
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int data[10] = {0};
	long ret = OnWriteInt32(115, 0, data);
	return (ret == 0);
}

bool CRobot::OnSetTrajSet_A(long serial, long pointNum, double *data)
{
	if (pointNum <= 0 || pointNum > 50)
	{
		return false;
	}

	long ret = OnWriteIntFloat(113, serial, pointNum * 7, data);
	return (ret == 0);
}

bool CRobot::OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int pv1[10] = {0};
	pv1[0] = fcType;
	double pv[20] = {0};
	pv[0] = fxDir[0];
	pv[1] = fxDir[1];
	pv[2] = fxDir[2];
	pv[3] = fxDir[3];
	pv[4] = fxDir[4];
	pv[5] = fxDir[5];
	pv[6] = fcCtrlPara[0];
	pv[7] = fcCtrlPara[1];
	pv[8] = fcCtrlPara[2];
	pv[9] = fcCtrlPara[3];
	pv[10] = fcCtrlPara[4];
	pv[11] = fcCtrlPara[5];
	pv[12] = fcCtrlPara[6];
	pv[13] = fcAdjLmt;
	long ret = OnWriteInt32Float(107, 1, pv1, 14, pv);
	return (ret == 0);
}

bool CRobot::OnSetDragSpace_A(int zsType)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int idata[10] = {0};
	double fdata[10] = {0};
	idata[0] = zsType;
	long ret = OnWriteInt32Float(106, 1, idata, 6, fdata);
	return {ret == 0};
}

bool CRobot::OnSetCartKD_A(double K[7], double D[7], int type)
{
	double pv[20] = {0};
	pv[0] = K[0];
	pv[1] = K[1];
	pv[2] = K[2];
	pv[3] = K[3];
	pv[4] = K[4];
	pv[5] = K[5];
	pv[6] = K[6];
	pv[7] = D[0];
	pv[8] = D[1];
	pv[9] = D[2];
	pv[10] = D[3];
	pv[11] = D[4];
	pv[12] = D[5];
	pv[13] = D[6];

	int idata[10] = {0};

	idata[0] = type;
	long ret = OnWriteFloatInt32(105, 14, pv, 1, idata);
	return {ret == 0};
}

bool CRobot::OnSetEefRot_A(int fcType, double CartCtrlPara[7])
{

	int pv1[20] = {0};
	pv1[0] = fcType;
	double pv[20] = {0};
	pv[0] = 0;
	pv[1] = 0;
	pv[2] = 0;
	pv[3] = 0;
	pv[4] = 0;
	pv[5] = 0;
	pv[6] = CartCtrlPara[0];
	pv[7] = CartCtrlPara[1];
	pv[8] = CartCtrlPara[2];
	pv[9] = CartCtrlPara[3];
	pv[10] = CartCtrlPara[4];
	pv[11] = CartCtrlPara[5];
	pv[12] = CartCtrlPara[6];
	pv[13] = 0;
	long ret = OnWriteInt32Float(107, 1, pv1, 14, pv);
	return {ret == 0};
}

bool CRobot::OnSetJointKD_A(double K[7], double D[7])
{
	double pv[20] = {0};
	pv[0] = K[0];
	pv[1] = K[1];
	pv[2] = K[2];
	pv[3] = K[3];
	pv[4] = K[4];
	pv[5] = K[5];
	pv[6] = K[6];
	pv[7] = D[0];
	pv[8] = D[1];
	pv[9] = D[2];
	pv[10] = D[3];
	pv[11] = D[4];
	pv[12] = D[5];
	pv[13] = D[6];

	long ret = OnWriteFloat(104, 14, pv);

	return ret == 0;
}
bool CRobot::OnSetVelEstStep_A(long step)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	double pv[20] = {0};
	pv[0] = step;
	if (pv[0] < 0)
	{
		pv[0] = 0;
	}
	else if (pv[0] > 1000)
	{
		pv[0] = 1000;
	}

	long ret = OnWriteFloat(121, 1, pv);
	return (ret == 0);
}
bool CRobot::OnSetTool_A(double kinePara[6], double dynPara[10])
{
	double pv[20] = {0};
	pv[0] = kinePara[0];
	pv[1] = kinePara[1];
	pv[2] = kinePara[2];
	pv[3] = kinePara[3];
	pv[4] = kinePara[4];
	pv[5] = kinePara[5];
	pv[6] = dynPara[0];
	pv[7] = dynPara[1];
	pv[8] = dynPara[2];
	pv[9] = dynPara[3];
	pv[10] = dynPara[4];
	pv[11] = dynPara[5];
	pv[12] = dynPara[6];
	pv[13] = dynPara[7];
	pv[14] = dynPara[8];
	pv[15] = dynPara[9];

	long ret = OnWriteFloat(102, 16, pv);

	return ret == 0;
}

bool CRobot::OnSetTargetState_A(int state)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int pv[20] = {0};
	pv[0] = state;
	long ret = OnWriteInt32(101, 1, pv);

	return ret == 0;
}

bool CRobot::OnSetImpType_A(int type)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int pv[20] = {0};
	pv[0] = type;
	long ret = OnWriteInt32(111, 1, pv);

	return ret == 0;
}

/////////////////B//////////////////
bool CRobot::OnSetJointLmt_B(int velRatio, int AccRatio)
{
	int data[10] = {0};
	int v = velRatio;
	int a = AccRatio;
	if (v < 1)
		v = 1;
	if (v > 100)
		v = 100;
	if (a < 1)
		a = 1;
	if (a > 100)
		a = 100;
	data[0] = v;
	data[1] = a;
	long ret = OnWriteInt16(203, 2, data);
	if (ret != 0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetForceCmd_B(double force)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	double data[10] = {0};
	data[0] = force;
	long ret = OnWriteFloat(209, 1, data);
	return (ret == 0);
}

bool CRobot::OnSetJointCmdPos_B(double joint[7])
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		return false;
	}
	double data[10] = {0};
	data[0] = joint[0];
	data[1] = joint[1];
	data[2] = joint[2];
	data[3] = joint[3];
	data[4] = joint[4];
	data[5] = joint[5];
	data[6] = joint[6];
	m_InsRobot->m_Arm1PosCmdSendSerial++;
	if (m_InsRobot->m_Arm1PosCmdSendSerial > 9999)
	{
		m_InsRobot->m_Arm1PosCmdSendSerial = 7;
	}
	// long ret = OnWriteFloat(208, 7, data);
	long ret = OnWriteIntFloat(208, m_InsRobot->m_Arm1PosCmdSendSerial, 7, data);
	return (ret == 0);
}

bool CRobot::OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int pv1[10] = {0};
	pv1[0] = fcType;
	double pv[20] = {0};
	pv[0] = fxDir[0];
	pv[1] = fxDir[1];
	pv[2] = fxDir[2];
	pv[3] = fxDir[3];
	pv[4] = fxDir[4];
	pv[5] = fxDir[5];
	pv[6] = fcCtrlPara[0];
	pv[7] = fcCtrlPara[1];
	pv[8] = fcCtrlPara[2];
	pv[9] = fcCtrlPara[3];
	pv[10] = fcCtrlPara[4];
	pv[11] = fcCtrlPara[5];
	pv[12] = fcCtrlPara[6];
	pv[13] = fcAdjLmt;
	long ret = OnWriteInt32Float(207, 1, pv1, 14, pv);
	return (ret == 0);
}

bool CRobot::OnSetDragSpace_B(int zsType)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int idata[10] = {0};
	double fdata[10] = {0};
	idata[0] = zsType;
	long ret = OnWriteInt32Float(206, 1, idata, 6, fdata);
	return {ret == 0};
}

bool CRobot::OnSetCartKD_B(double K[7], double D[7], int type)
{
	double pv[20] = {0};
	pv[0] = K[0];
	pv[1] = K[1];
	pv[2] = K[2];
	pv[3] = K[3];
	pv[4] = K[4];
	pv[5] = K[5];
	pv[6] = K[6];
	pv[7] = D[0];
	pv[8] = D[1];
	pv[9] = D[2];
	pv[10] = D[3];
	pv[11] = D[4];
	pv[12] = D[5];
	pv[13] = D[6];

	int idata[10] = {0};

	idata[0] = type;
	long ret = OnWriteFloatInt32(205, 14, pv, 1, idata);
	return {ret == 0};
}

bool CRobot::OnSetEefRot_B(int fcType, double CartCtrlPara[7])
{
	int pv1[20] = {0};
	pv1[0] = fcType;
	double pv[20] = {0};
	pv[0] = 0;
	pv[1] = 0;
	pv[2] = 0;
	pv[3] = 0;
	pv[4] = 0;
	pv[5] = 0;
	pv[6] = CartCtrlPara[0];
	pv[7] = CartCtrlPara[1];
	pv[8] = CartCtrlPara[2];
	pv[9] = CartCtrlPara[3];
	pv[10] = CartCtrlPara[4];
	pv[11] = CartCtrlPara[5];
	pv[12] = CartCtrlPara[6];
	pv[13] = 0;
	long ret = OnWriteInt32Float(207, 1, pv1, 14, pv);
	return {ret == 0};
}

bool CRobot::OnSetJointKD_B(double K[7], double D[7])
{
	double pv[20] = {0};
	pv[0] = K[0];
	pv[1] = K[1];
	pv[2] = K[2];
	pv[3] = K[3];
	pv[4] = K[4];
	pv[5] = K[5];
	pv[6] = K[6];
	pv[7] = D[0];
	pv[8] = D[1];
	pv[9] = D[2];
	pv[10] = D[3];
	pv[11] = D[4];
	pv[12] = D[5];
	pv[13] = D[6];

	long ret = OnWriteFloat(204, 14, pv);

	return ret == 0;
}

bool CRobot::OnSetVelEstStep_B(long step)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	double pv[20] = {0};
	pv[0] = step;
	if (pv[0] < 0)
	{
		pv[0] = 0;
	}
	else if (pv[0] > 1000)
	{
		pv[0] = 1000;
	}

	long ret = OnWriteFloat(221, 1, pv);
	return (ret == 0);
}
bool CRobot::OnSetTool_B(double kinePara[6], double dynPara[10])
{
	double pv[20] = {0};
	pv[0] = kinePara[0];
	pv[1] = kinePara[1];
	pv[2] = kinePara[2];
	pv[3] = kinePara[3];
	pv[4] = kinePara[4];
	pv[5] = kinePara[5];
	pv[6] = dynPara[0];
	pv[7] = dynPara[1];
	pv[8] = dynPara[2];
	pv[9] = dynPara[3];
	pv[10] = dynPara[4];
	pv[11] = dynPara[5];
	pv[12] = dynPara[6];
	pv[13] = dynPara[7];
	pv[14] = dynPara[8];
	pv[15] = dynPara[9];

	long ret = OnWriteFloat(202, 16, pv);

	return ret == 0;
}

bool CRobot::OnSetTargetState_B(int state)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int pv[20] = {0};
	pv[0] = state;
	long ret = OnWriteInt32(201, 1, pv);

	return ret == 0;
}

bool CRobot::OnSetImpType_B(int type)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int pv[20] = {0};
	pv[0] = type;
	long ret = OnWriteInt32(211, 1, pv);

	return ret == 0;
}

bool CRobot::OnSetPlnCart_B(CPointSet *pset)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	DCSS t;
	long num = pset->OnGetPointNum();
	if (num <= 5)
	{
		printf("[ERROR] OnSetPlnCart_B: there are fewer than 5 planned points, cannot operate \n");
		return false;
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_B(num);
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetTrajInit_B: OnSetSendWaitResponse timeout\n");
		return false;
	}
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState != 1)
		{
			printf("[ERROR] OnSetTrajInit_B: The controller has not entered planning mode\n");
			return false;
		}
	}
	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii, jj, kk;
	double SendData[350];
	double *retp;
	long spos;
	long ipos = 0;
	for (ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for (jj = 0; jj < 50; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(ii, 50, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_B: OnSetSendWaitResponse timeout\n");
			return false;
		}
	}
	if (relic_num != 0)
	{
		spos = 0;
		for (jj = 0; jj < relic_num; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(send_g_num, relic_num, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_B: OnSetSendWaitResponse timeout\n");
			return false;
		}
	}
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState != 2)
		{
			printf("[ERROR] OnSetTrajSet_B: The controller did not receive the sent trajectory\n");
			return false;
		}
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_B();
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetTrajRun_B: OnSetSendWaitResponse timeout\n");
		return false;
	}
	return true;
}

bool CRobot::OnSetPlnJoint_B(double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	DCSS t;
	double vr = vel_ratio;
	double ar = acc_ratio;
	if (vr < 0.01)
		vr = 0.01;
	if (vr > 1.0)
		vr = 1.0;
	if (ar < 0.01)
		ar = 0.01;
	if (ar > 1.0)
		ar = 1.0;
	long i = 0;
	double sta[8] = {0};
	double sto[8] = {0};
	for (i = 0; i < 7; i++)
	{
		sta[i] = start_joints[i];
		sto[i] = stop_joints[i];
	}

	long num = pln_B.OnPln(sta, sto, vr, ar);
	if (num <= 0)
	{
		printf("[ERROR] OnSetPlnJoint_B: planning failed, please check start joints and end joints\n");
		return false;
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_B(num);
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetPlnJoint_B: OnSetSendWaitResponse timeout\n");
		return false;
	}
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState != 1)
		{
			printf("[ERROR] OnSetTrajInit_B: The controller has not entered planning mode\n");
			return false;
		}
	}
	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii, jj, kk;

	double SendData[350];
	double retp[8];
	long spos;
	for (ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for (jj = 0; jj < 50; jj++)
		{
			pln_B.OnCut(retp);
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(ii, 50, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_B: OnSetSendWaitResponse timeout\n");
			return false;
		}
	}
	if (relic_num != 0)
	{
		spos = 0;
		for (jj = 0; jj < relic_num; jj++)
		{
			pln_B.OnCut(retp);
			for (kk = 0; kk < 7; kk++)
			{
				SendData[spos] = retp[kk];
				spos++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(send_g_num, relic_num, SendData);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] OnSetTrajSet_B: OnSetSendWaitResponse timeout\n");
			return false;
		}
	}
	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState != 2)
		{
			printf("[ERROR] OnSetTrajSet_B: The controller did not receive the sent trajectory\n");
			return false;
		}
	}
	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_B();
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnSetTrajRun_B: OnSetSendWaitResponse timeout\n");
		return false;
	}
	return true;
}

bool CRobot::OnSetTrajInit_B(int pointNum)
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	if (pointNum < 0)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK]: Initial B arm Planning Trajectory failed, number of point=%d,\n", pointNum);
		}
		return false;
	}

	int data[10] = {0};
	data[0] = pointNum;
	long ret = OnWriteInt32(212, 1, data);
	if (m_InsRobot->m_LocalLogTag == true)
	{
		printf("[Marvin SDK]: Initial B arm Planning Trajectory , number of point=%d,\n", pointNum);
	}
	return (ret == 0);
}

bool CRobot::OnSetTrajRun_B()
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int data[10] = {0};
	long ret = OnWriteInt32(214, 0, data);
	return (ret == 0);
}

bool CRobot::OnStopPlnJoint_B()
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	CRobot::OnClearSet();
	CRobot::OnStopPlnJoint_interB();
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] OnStopPlnJoint_interB: OnSetSendWaitResponse timeout\n");
		return false;
	}
	else
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK]: B arm stop run Planning Trajectory \n");
		}
		return true;
	}
}

bool CRobot::OnStopPlnJoint_interB()
{
	if (m_InsRobot->m_VersionMatchTag == FX_FALSE)
	{
		if (m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK %s]: Warning: Version mismatch between control system %d and SDK %d\n", __FUNCTION__, m_InsRobot->m_ctrlSysVersion, SDK_VERSION);
		}
		return false;
	}
	int data[10] = {0};
	long ret = OnWriteInt32(215, 0, data);
	return (ret == 0);
}

bool CRobot::OnSetTrajSet_B(long serial, long pointNum, double *data)
{
	if (pointNum <= 0 || pointNum > 50)
	{
		return false;
	}

	long ret = OnWriteIntFloat(213, serial, pointNum * 7, data);
	return (ret == 0);
}

long CRobot::OnSetSendWaitResponse(long time_out)
{
	if (m_InsRobot->m_SendTag == 100)
	{
		return -1;
	}
	if (m_InsRobot->m_send_response_local_tag < 7)
	{
		m_InsRobot->m_send_response_local_tag = 7;
	}
	if (m_InsRobot->m_send_response_local_tag > 100)
	{
		m_InsRobot->m_send_response_local_tag = 7;
	}
	m_InsRobot->m_send_response_local_tag++;

	unsigned char pv[10] = {0};
	pv[0] = m_InsRobot->m_send_response_local_tag;
	long ret = OnWriteRaw(251, 1, pv);
	if (ret != 0)
	{
		return -1;
	}
	long tmp_time_out = time_out;
	if (tmp_time_out < 20)
	{
		tmp_time_out = 20;
	}
	if (tmp_time_out > 1000)
	{
		tmp_time_out = 1000;
	}
	m_InsRobot->m_respones_time_tag = 0;
	m_InsRobot->m_send_response_timeout_cnt = tmp_time_out;

	if (OnSetSend() == false)
	{
		return -1;
	}

	while (m_InsRobot->m_send_response_timeout_cnt > 0)
	{
		SLEEP(1);
	}
	if (m_InsRobot->m_respones_time_tag == 1)
	{
		m_InsRobot->m_respones_time_tag = 0;
		if (m_InsRobot->m_LocalLogTag == true)
			printf("[Marvin SDK]: OnSetSendWaitResponse\n");
		return m_InsRobot->m_respones_time_cnt;
	}
	if (m_InsRobot->m_send_response_recv_tag == m_InsRobot->m_send_response_local_tag)
	{
		return 1;
	}
	return -1;
}

long CRobot::OnSetSendWaitResponse2(long time_out, unsigned char ret_debug[1500], long &ret_debug_len)
{
	if (m_InsRobot->m_SendTag == 100)
	{
		return -1;
	}
	if (m_InsRobot->m_send_response_local_tag < 7)
	{
		m_InsRobot->m_send_response_local_tag = 7;
	}
	if (m_InsRobot->m_send_response_local_tag > 100)
	{
		m_InsRobot->m_send_response_local_tag = 7;
	}
	m_InsRobot->m_send_response_local_tag++;

	unsigned char pv[10] = {0};
	pv[0] = m_InsRobot->m_send_response_local_tag;

	long ret = OnWriteRaw(251, 1, pv);
	if (ret != 0)
	{
		return -1;
	}

	ret_debug_len = m_InsRobot->m_Slen;
	long copy_len = ret_debug_len;
	if (copy_len > 1500)
	{
		copy_len = 1500;
	}
	memcpy(ret_debug, m_InsRobot->m_SendBuf, copy_len);

	long tmp_time_out = time_out;
	if (tmp_time_out < 20)
	{
		tmp_time_out = 20;
	}
	if (tmp_time_out > 1000)
	{
		tmp_time_out = 1000;
	}
	m_InsRobot->m_respones_time_tag = 0;
	m_InsRobot->m_send_response_timeout_cnt = tmp_time_out;

	if (OnSetSend() == false)
	{
		return -1;
	}

	while (m_InsRobot->m_send_response_timeout_cnt > 0)
	{
		SLEEP(1);
	}
	if (m_InsRobot->m_respones_time_tag == 1)
	{
		m_InsRobot->m_respones_time_tag = 0;
		if (m_InsRobot->m_LocalLogTag == true)
			printf("[Marvin SDK]: OnSetSendWaitResponse\n");
		return m_InsRobot->m_respones_time_cnt;
	}
	return -1;
}

bool CRobot::OnSetSend()
{
	if (m_InsRobot->m_SendTag == 100 || m_InsRobot->m_SendTag == 50)
	{
		return false;
	}
	m_InsRobot->m_SendTag = 50;

	unsigned char crc_frm_head = 0;

	crc_frm_head += m_InsRobot->m_SendBuf[0];
	crc_frm_head += m_InsRobot->m_SendBuf[1];
	crc_frm_head += m_InsRobot->m_SendBuf[2];
	crc_frm_head += m_InsRobot->m_SendBuf[3];
	crc_frm_head += m_InsRobot->m_SendBuf[4];
	crc_frm_head += m_InsRobot->m_SendBuf[5];

	if (crc_frm_head != 0)
	{
		m_InsRobot->m_SendTag = 0;
		m_InsRobot->m_SendBuf[0] = 'F';
		m_InsRobot->m_SendBuf[1] = 'Y';
		m_InsRobot->m_SendBuf[2] = 0; // INS number
		m_InsRobot->m_SendBuf[3] = 0; // LSB of data len
		m_InsRobot->m_SendBuf[4] = 0; // MSB of data len
		m_InsRobot->m_SendBuf[5] = 0; // CRC of Head
		m_InsRobot->m_SendBuf[6] = 0; // CRC of data

		{
			unsigned char addv = 0;
			addv += m_InsRobot->m_SendBuf[0];
			addv += m_InsRobot->m_SendBuf[1];
			addv += m_InsRobot->m_SendBuf[2];
			addv += m_InsRobot->m_SendBuf[3];
			addv += m_InsRobot->m_SendBuf[4];
			m_InsRobot->m_SendBuf[5] = 256 - addv;
		}
		m_InsRobot->m_Slen = 7;
		m_InsRobot->m_SendTag = 0;

		printf("Onsend false\n");
		return false;
	}

	long date_len = m_InsRobot->m_SendBuf[3] + 256 * m_InsRobot->m_SendBuf[4];
	unsigned char crc_frm_data = 0;
	for (int i = 0; i < date_len; i++)
	{
		crc_frm_data += m_InsRobot->m_SendBuf[i + 7];
	}
	m_InsRobot->m_SendBuf[6] = crc_frm_data;

	m_InsRobot->m_SendTag = 100;

	return true;
}

bool CRobot::OnUpdateSystem(char *local_path)
{
	if (!m_InsRobot->SendFile(local_path, (char *)"/home/FUSION/Tmp/ctrl_package.tar"))
	{
		return false;
	}
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "UPDATES");
	OnSetIntPara(name, 0);
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: SDK update!\n");
	return true;
}

bool CRobot::OnDownloadLog(char *local_path)
{
	if (!m_InsRobot->RecvFile(local_path, (char *)"/home/FUSION/log/LOG.txt"))
	{
		return false;
	}
	if (m_InsRobot->m_LocalLogTag == true)
		printf("[Marvin SDK]: Send log to host:%s,\n", local_path);
	return true;
}

FX_BOOL CRobot::SendFile(char *local_file, char *remote_file)
{
	if (m_LinkTag == false)
	{
		return FX_FALSE;
	}
	CTCPFileClient cln;
	if (cln.OnLinkTo(m_ip1, m_ip2, m_ip3, m_ip4, 10240) == false)
	{
		return FX_FALSE;
	}
	bool ret = cln.OnSendFile(local_file, remote_file);
	cln.OnQuit();
	if (ret == true)
	{
		return FX_TRUE;
	}
	return FX_FALSE;
}

FX_BOOL CRobot::RecvFile(char *local_file, char *remote_file)
{
	if (m_LinkTag == false)
	{
		return FX_FALSE;
	}
	CTCPFileClient cln;
	if (cln.OnLinkTo(m_ip1, m_ip2, m_ip3, m_ip4, 10240) == false)
	{
		return FX_FALSE;
	}
	bool ret = cln.OnGetFile(local_file, remote_file);
	cln.OnQuit();
	if (ret == true)
	{
		return FX_TRUE;
	}
	return FX_FALSE;
}
