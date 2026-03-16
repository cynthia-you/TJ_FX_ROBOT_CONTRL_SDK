#include "Robot.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <math.h> 

static CRobot* m_InsRobot = NULL;

bool CRobot::OnClearChDataA()
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB1.ReadBuf((unsigned char*)&t, si);
	while (num > 0)
	{
		num = m_InsRobot->m_ACB1.ReadBuf((unsigned char*)&t, si);
	}
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Clear 485 cache of A arm\n");

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
	long num = m_InsRobot->m_ACB2.ReadBuf((unsigned char*)&t, si);
	while (num > 0)
	{
		num = m_InsRobot->m_ACB2.ReadBuf((unsigned char*)&t, si);
	}
    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Clear 485 cache of B arm\n");
	return true;
}

long CRobot::OnGetChDataA(unsigned char data_ptr[256], long* ret_ch)
{
	if (*ret_ch != 1 && *ret_ch != 2 && *ret_ch != 3)
    {
		printf("OnGetChDataA ERROR: Invalid ret_ch number %d (valid range: 1~3)\n", ret_ch);
		return false;
    }

	if (m_InsRobot == NULL)
	{
		return 0;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num =  m_InsRobot->m_ACB1.ReadBuf((unsigned char *) & t, si);
	if (num == 0)
	{
		return num;
	}

    memset(data_ptr,0,256);
	*ret_ch = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Get 485 of A arm: \nchannel =%d\n",ret_ch);
		printf("data:\n");
        for (int i = 0; i < 256; ++i) {
            printf("%02x ", data_ptr[i]);

            if ((i + 1) % 16 == 0) {
                printf("  ");
                for (int j = i-15; j <= i; j++) {
                    printf("%c", (data_ptr[j] >= 32 && data_ptr[j] <= 126) ? data_ptr[j] : '.');
                }
                printf("\n");
            }
        }
	    printf("\ndata size=%d \n",t.m_Size);

	}
	return t.m_Size;

}

bool CRobot::OnSetChDataA(unsigned char* data_ptr, long size_int, long set_ch)
{
	if (size_int <= 0 || size_int >256)
	{
		printf("OnSetChDataA ERROR: Channel must in the range of  0~255\n");
		return false;
	}
    if (set_ch != 1 && set_ch != 2 && set_ch != 3)
    {
        printf("OnSetChDataA ERROR: Invalid set_ch number %d (valid range: 1~3)\n", set_ch);
		return false;
    }
	if (m_InsRobot == NULL)
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
	sendto(m_InsRobot->_tosock_, (char*)m_InsRobot->m_SendBuf1, sizeof(DDSS)+2, 0, (struct sockaddr*)&m_InsRobot->_to, sizeof(m_InsRobot->_to));

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set 485 of A arm: \nchannel =%d\n",set_ch);
		printf("data:\n");
        for (int i = 0; i < 256; ++i) {
            printf("%02x ", data_ptr[i]);

            if ((i + 1) % 16 == 0) {
                printf("  ");
                for (int j = i-15; j <= i; j++) {
                    printf("%c", (data_ptr[j] >= 32 && data_ptr[j] <= 126) ? data_ptr[j] : '.');
                }
                printf("\n");
            }
        }
	    printf("\ndata size=%d \n",size_int);
	}

	return true;
}

long CRobot::OnGetChDataB(unsigned char data_ptr[256], long* ret_ch)
{
	if (*ret_ch != 1 && *ret_ch != 2 && *ret_ch != 3)
    {
		printf("OnGetChDataB ERROR: Invalid ret_ch number %d (valid range: 1~3)\n", ret_ch);
		return false;
    }
	if (m_InsRobot == NULL)
	{
		return 0;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB2.ReadBuf((unsigned char*)&t, si);
	if (num == 0)
	{
		return num;
	}


	memset(data_ptr, 0, 256);
	*ret_ch = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Get 485 of B arm: \nchannel =%d\n",ret_ch);
		printf("data:\n");
        for (int i = 0; i < 256; ++i) {
            printf("%02x ", data_ptr[i]);

            if ((i + 1) % 16 == 0) {
                printf("  ");
                for (int j = i-15; j <= i; j++) {
                    printf("%c", (data_ptr[j] >= 32 && data_ptr[j] <= 126) ? data_ptr[j] : '.');
                }
                printf("\n");
            }
        }
	    printf("\ndata size=%d \n",t.m_Size);

	}
	return t.m_Size;
}

bool CRobot::OnSetChDataB(unsigned char* data_ptr, long size_int, long set_ch)
{
	if (size_int <= 0 || size_int >256)
	{
		printf("OnSetChDataB ERROR: Channel must in the range of 0~255\n");
		return false;
	}
    if (set_ch != 1 && set_ch != 2 && set_ch != 3)
    {
        printf("OnSetChDataB ERROR: Invalid set_ch number %d (valid range: 1~3)\n", set_ch);
		return false;
    }

	if (m_InsRobot == NULL)
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
	sendto(m_InsRobot->_tosock_, (char*)m_InsRobot->m_SendBuf2, sizeof(DDSS) + 2, 0, (struct sockaddr*)&m_InsRobot->_to, sizeof(m_InsRobot->_to));

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set 485 of B arm: channel =%d\n",set_ch);
		printf("data:\n");
        for (int i = 0; i < 256; ++i) {
            printf("%02x ", data_ptr[i]);

            if ((i + 1) % 16 == 0) {
                printf("  ");
                for (int j = i-15; j <= i; j++) {
                    printf("%c", (data_ptr[j] >= 32 && data_ptr[j] <= 126) ? data_ptr[j] : '.');
                }
                printf("\n");
            }
        }
	    printf("\ndata size=%d \n",size_int);
	}

	return true;
}

long CRobot::OnGetSDKVersion()
{
    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: SDK version %d\n",SDK_VERSION);
	return SDK_VERSION;
}

bool CRobot::OnSendPVT_A(char* local_file, long serial)
{
	if (serial < 0 || serial >= 100)
	{
		return false;
	}
	char remote[256];
	memset(remote,0,256);
	sprintf(remote, "\/home\/FUSION\/Config\/pvt\/user0\/%d.txt", serial);
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Send A arm pvt of serial=%d to local=%s\n ", serial, local_file);
	return OnSendFile(local_file, remote);
}

bool CRobot::OnSendPVT_B(char* local_file, long serial)
{
	if (serial < 0 || serial >= 100)
	{
		return false;
	}
	char remote[256];
	memset(remote, 0, 256);
	sprintf(remote, "\/home\/FUSION\/Config\/pvt\/user1\/%d.txt", serial);
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Send B arm pvt of serial=%d to local=%s\n ", serial, local_file);
	return OnSendFile(local_file, remote);
}

bool  CRobot:: OnSendFile( char* local_file, char* remote_file)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}

	if (m_InsRobot->SendFile(local_file, remote_file) == FX_TRUE)
	{
	    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: send file local file:%s, remote file: %s\n ", local_file, remote_file);
		return true;
	}
	return  false;
}

bool CRobot::OnRecvFile(char* local_file, char* remote_file)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}

	if (m_InsRobot->RecvFile(local_file, remote_file) == FX_TRUE)
	{
		return true;
	}
	return  false;
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

void  CallBackFunc(union sigval v)
{
	m_InsRobot->DoRecv();
	m_InsRobot->DoSend();
	m_InsRobot->DoCnt();
}
#endif

CRobot::CRobot()
{
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
	memset(&m_DCSS, 0, sizeof(DCSS));
	m_LastGatherTag = FX_FALSE;
	m_GatherTag = FX_FALSE;
	m_RunState = 0;
	m_LinkTag = FX_FALSE;
	_local_sock = INVALID_SOCKET;
	_tosock_ = INVALID_SOCKET;


	pDDSS1 = (DDSS*)&m_SendBuf1[2];
	memset(pDDSS1, 0, sizeof(DDSS));
	m_SendBuf1[0] = 'C';
	m_SendBuf1[1] = 'H';
	pDDSS1->m_Serial = 1;
	pDDSS1->m_CH = 1;


	pDDSS2 = (DDSS*)&m_SendBuf2[2];
	memset(pDDSS2, 0, sizeof(DDSS));
	m_SendBuf2[0] = 'C';
	m_SendBuf2[1] = 'H';
	pDDSS2->m_Serial = 1;
	pDDSS2->m_CH = 2;

#ifdef CMPL_WIN
    char* shm_name = "AAA";  // Windows
#else
    char* shm_name = "/var/tmp/AAA_shm";  // 在/var/tmp创建文件（重启后保留）
#endif

    ShmOnInit(&m_ShMem);

// Windows 下检查共享内存是否存在
#ifdef CMPL_WIN
    printf("Checking Windows shared memory: %s\n", shm_name);
    HANDLE hMapFile = OpenFileMappingA(
        FILE_MAP_ALL_ACCESS,
        FALSE,
        shm_name
    );
    if (hMapFile != NULL)
    {
        CloseHandle(hMapFile);
    }
    else
    {
        DWORD err = GetLastError();
        if (err == ERROR_FILE_NOT_FOUND)
        {
            printf("Shared memory %s does not exist (error: %lu), will be created\n", shm_name, err);
        }
        else
        {
            printf("Error checking shared memory %s, error code: %lu\n", shm_name, err);
        }
    }

#else
    // Linux下使用传统文件方式创建共享内存
    printf("Checking Linux file-based shared memory: %s\n", shm_name);
    FILE* fp = fopen(shm_name, "rb");
    if (fp != NULL)
    {
        fclose(fp);
        printf("Shared memory file %s already exists\n", shm_name);
        struct stat st;
        if (stat(shm_name, &st) == 0)
        {
            printf("  Size: %ld bytes\n", st.st_size);
        }
    }
    else
    {
        printf("Shared memory file %s does not exist, will be created\n", shm_name);
        fp = fopen(shm_name, "wb");
        if (fp != NULL)
        {
            fseek(fp, 102399, SEEK_SET);
            fputc(0, fp);
            fclose(fp);
            chmod(shm_name, 0666);
            printf("  Created shared memory file with size 102400 bytes\n");
            struct stat st;
            if (stat(shm_name, &st) == 0)
            {
                printf("  File permissions: %o\n", st.st_mode & 0777);
            }
        }
        else
        {
            printf("  Failed to create shared memory file, errno: %d (%s)\n", errno, strerror(errno));
        }
    }
    printf("  /var/tmp/ directory permissions:\n");
    system("ls -ld /var/tmp/");
#endif

    int master_result = m_ShMem.OnMapMster(&m_ShMem, shm_name, 102400);
    m_psm = m_ShMem.OnGetMem(&m_ShMem);

    if(m_psm == NULL)
    {
        printf("Map Master Err - m_psm is NULL\n");

#ifdef CMPL_WIN
        DWORD err = GetLastError();
#else
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "ls -la %s 2>/dev/null || echo '  File not found'", shm_name);
        system(cmd);
        int fd = open(shm_name, O_RDWR);
        if (fd != -1)
        {
            void* test_map = mmap(NULL, 102400, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (test_map != MAP_FAILED)
            {
                munmap(test_map, 102400);
            }
            else
            {
                printf("    Direct mmap failed, errno: %d (%s)\n", errno, strerror(errno));
            }
            close(fd);
        }
        else
        {
            printf("    Failed to open file, errno: %d (%s)\n", errno, strerror(errno));
        }
#endif

        int slave_result = m_ShMem.OnMapSlave(&m_ShMem, shm_name);
        m_psm = m_ShMem.OnGetMem(&m_ShMem);
        if(m_psm == NULL)
        {
            printf("Map Slave Err - m_psm is NULL\n");

#ifdef CMPL_WIN
            DWORD err = GetLastError();
            printf("  Windows Error Code: %lu\n", err);
            printf("  Checking if Global\\ path requires admin privileges\n");
#else
            printf("  errno: %d (%s)\n", errno, strerror(errno));
            printf("  Checking file existence:\n");
            char cmd[256];
            snprintf(cmd, sizeof(cmd), "ls -la %s 2>/dev/null || echo '  File not found'", shm_name);
            system(cmd);
#endif
        }
        else
        {
            printf("Map Slave success - m_psm = %p\n", m_psm);
        }
    }
    else
    {
        printf("Map Master success - m_psm = %p\n", m_psm);
    }
    if(m_psm != NULL)
    {
        m_ACB_ShMem.OnSetBuf(m_psm, 102400);
        m_ACB_ShMem.Empty();
    }
    else
    {

    }
}


bool CRobot::OnRelease()
{
    if (m_InsRobot == NULL)
    {
        return true;
    }
#ifdef CMPL_WIN
    timeKillEvent(m_InsRobot->m_TimeEventID);
    Sleep(10);
#endif
#ifdef CMPL_LIN
    if (m_InsRobot->m_LinkTag == FX_TRUE) {
        timer_delete(m_InsRobot->robot_timer);
    }
    usleep(10000);
#endif

    // m_InsRobot->m_ShMem.OnDest(&m_InsRobot->m_ShMem);// 先注释，防止销毁
	m_InsRobot->m_ShMem.OnDest(&m_InsRobot->m_ShMem);

    delete m_InsRobot;
    m_InsRobot = NULL;

    printf("[Marvin SDK]: Robot released\n");
    return true;
}

CRobot::~CRobot()
{
	m_InsRobot->m_ShMem.OnDest(&m_InsRobot->m_ShMem);
}

CRobot* CRobot::GetIns()
{
	if (m_InsRobot == NULL)
	{
		m_InsRobot = new CRobot();
	}
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

bool CRobot::OnGetBuf(DCSS* ret)
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	memcpy(ret,&m_InsRobot->m_DCSS,sizeof(m_DCSS));

	return true;
}

bool CRobot::OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	assert(ip1 >= 0 && ip1 <= 255);
    assert(ip2 >= 0 && ip2 <= 255);
    assert(ip3 >= 0 && ip3 <= 255);
    assert(ip4 >= 0 && ip4 <= 255);
    if (ip1 == 0 && ip2 == 0 && ip3 == 0 && ip4 == 0)
    {
        printf("OnLinkTo ERROR: Invalid IP address: 0.0.0.0");
        return false;
    }
    if (ip1 == 255 && ip2 == 255 && ip3 == 255 && ip4 == 255)
    {
        printf("OnLinkTo ERROR: Invalid IP address: broadcast address");
        return false;
    }
	if (ip1 == 127)
    {
        printf("OnLinkTo ERROR: Loopback address not allowed");
        return false;
    }
	
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

    if (bind(m_InsRobot->_local_sock, (struct sockaddr*)&m_InsRobot->_local, sizeof(m_InsRobot->_local)) != 0)
    {
        if(m_InsRobot->m_LocalLogTag == true) printf("port bind failure, possibly occupied by another program\n");
        #ifdef CMPL_WIN
            closesocket(m_InsRobot->_local_sock);
        #endif
        #ifdef CMPL_LIN
            close(m_InsRobot->_local_sock);
        #endif
        m_InsRobot->_local_sock = 0;
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
				int Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr*)&m_InsRobot->_local, &_localLen);
#endif
#ifdef CMPL_LIN
				int Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr*)&m_InsRobot->_local, (socklen_t*)&_localLen);
#endif
		while(Len > 0)
		{
#ifdef CMPL_WIN
					 Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr*)&m_InsRobot->_local, &_localLen);
#endif
#ifdef CMPL_LIN
					 Len = recvfrom(m_InsRobot->_local_sock, m_InsRobot->recvbuf, 2000, 0, (struct sockaddr*)&m_InsRobot->_local, (socklen_t*)&_localLen);
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
		memset   (&evp, 0, sizeof(evp));
		evp.sigev_value.sival_ptr = &m_InsRobot->robot_timer;
		evp.sigev_notify = SIGEV_THREAD;
		evp.sigev_notify_function = CallBackFunc;
		evp.sigev_value.sival_int = 0;
		ret = timer_create(CLOCK_REALTIME, &evp, &m_InsRobot->robot_timer);
		if( ret){
			return false;
		}

		ts.it_interval.tv_sec = 0;
		ts.it_interval.tv_nsec = 1000000;
		ts.it_value.tv_sec = 0;
		ts.it_value.tv_nsec = 1000000;
		ret = timer_settime(m_InsRobot->robot_timer, TIMER_ABSTIME, &ts, NULL);
		if( ret )
		{
			return false;
		}
	}
#endif

 //   m_InsRobot->ReadPendingData();
	m_InsRobot->m_RunState = 0;

	m_InsRobot->m_ip1 = ip1;
	m_InsRobot->m_ip2 = ip2;
	m_InsRobot->m_ip3 = ip3;
	m_InsRobot->m_ip4 = ip4;

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Robot connected  IP=%d.%d.%d.%d\n", ip1, ip2, ip3, ip4);
	}
	return true;

}

//bool CRobot::OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
//{
//    GetIns();
//    if (m_InsRobot->m_LinkTag == FX_TRUE)
//    {
//        return false;
//    }
//#ifdef CMPL_WIN
//    WSADATA wsadata;
//    int ret;
//    ret = WSAStartup(0x101, &wsadata);
//    if (ret != 0)
//    {
//        return false;
//    }
//#endif
//    memset(&m_InsRobot->_local, 0, sizeof(m_InsRobot->_local));
//    m_InsRobot->_localLen = sizeof(sockaddr_in);
//    m_InsRobot->_local.sin_family = AF_INET;
//    m_InsRobot->_local.sin_port = htons(4730);
//    m_InsRobot->_local.sin_addr.s_addr = INADDR_ANY;
//    m_InsRobot->_local_sock = socket(AF_INET, SOCK_DGRAM, 0);
//
//    // 检查socket创建是否成功
//    if (m_InsRobot->_local_sock == INVALID_SOCKET)
//    {
//        if(m_InsRobot->m_LocalLogTag == true) printf("socket creation failure\n");
//        return false;
//    }
//
//    unsigned long on = 1;
//#ifdef CMPL_WIN
//    if (0 != ioctlsocket(m_InsRobot->_local_sock, FIONBIO, &on))
//    {
//        closesocket(m_InsRobot->_local_sock);
//        return false;
//    }
//#endif
//#ifdef CMPL_LIN
//    if (0 != ioctl(m_InsRobot->_local_sock, FIONBIO, &on))
//    {
//        close(m_InsRobot->_local_sock);
//        return false;
//    }
//#endif
//    // 绑定Socket并检查结果
//    if (bind(m_InsRobot->_local_sock, (struct sockaddr*)&m_InsRobot->_local, sizeof(m_InsRobot->_local)) != 0)
//    {
//        if(m_InsRobot->m_LocalLogTag == true) printf("port bind failure, possibly occupied by another program\n");
//        // 关闭socket连接
//        #ifdef CMPL_WIN
//            closesocket(m_InsRobot->_local_sock);
//        #endif
//        #ifdef CMPL_LIN
//            close(m_InsRobot->_local_sock);
//        #endif
//        m_InsRobot->_local_sock = 0;
//        return false;
//    }
//    memset(&m_InsRobot->_to, 0, sizeof(_to));
//    char ip_str[100];
//    sprintf(ip_str, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
//    m_InsRobot->_toLen = sizeof(sockaddr_in);
//    m_InsRobot->_to.sin_family = AF_INET;
//    m_InsRobot->_to.sin_port = htons(4729);
//    m_InsRobot->_to.sin_addr.s_addr = inet_addr(ip_str);
//    m_InsRobot->_tosock_ = socket(AF_INET, SOCK_DGRAM, 0);
//
//    // 检查_tosock_创建是否成功
//    if (m_InsRobot->_tosock_ == INVALID_SOCKET)
//    {
//        if(m_InsRobot->m_LocalLogTag == true) printf("target socket creation failure\n");
//        #ifdef CMPL_WIN
//            closesocket(m_InsRobot->_local_sock);
//        #endif
//        #ifdef CMPL_LIN
//            close(m_InsRobot->_local_sock);
//        #endif
//        return false;
//    }
//
//    m_InsRobot->m_LinkTag = FX_TRUE;
//
//#ifdef CMPL_WIN
//    m_InsRobot->m_TimeEventID = timeSetEvent(1, 1, CallBackFunc2, (DWORD)NULL, TIME_PERIODIC);
//#endif
//#ifdef CMPL_LIN
//    {
//        struct sigevent evp;
//        struct itimerspec ts;
//        int ret;
//        memset   (&evp, 0, sizeof(evp));
//        evp.sigev_value.sival_ptr = &m_InsRobot->robot_timer;
//        evp.sigev_notify = SIGEV_THREAD;
//        evp.sigev_notify_function = CallBackFunc;
//        evp.sigev_value.sival_int = 0;
//        ret = timer_create(CLOCK_REALTIME, &evp, &m_InsRobot->robot_timer);
//        if( ret){
//            return false;
//        }
//
//        ts.it_interval.tv_sec = 0;
//        ts.it_interval.tv_nsec = 1000000;
//        ts.it_value.tv_sec = 0;
//        ts.it_value.tv_nsec = 1000000;
//        ret = timer_settime(m_InsRobot->robot_timer, TIMER_ABSTIME, &ts, NULL);
//        if( ret )
//        {
//            return false;
//        }
//    }
//#endif
//
//    // 通过对象实例调用ReadPendingData
//    m_InsRobot->ReadPendingData();  // 修改这里：通过m_InsRobot指针调用
//
//    m_InsRobot->m_RunState = 0;
//
//    m_InsRobot->m_ip1 = ip1;
//    m_InsRobot->m_ip2 = ip2;
//    m_InsRobot->m_ip3 = ip3;
//    m_InsRobot->m_ip4 = ip4;
//
//    if(m_InsRobot->m_LocalLogTag == true)
//    {
//        printf("[Marvin SDK]: Robot connected  IP=%d.%d.%d.%d\n", ip1, ip2, ip3, ip4);
//    }
//    return true;
//}

void CRobot::ReadPendingData()
{
    if (!m_InsRobot) {
        return;
    }

    char buffer[2048];
    sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int dataCount = 0;

    if(m_InsRobot->m_LocalLogTag == true)
    {
        printf("[Marvin SDK]: Checking for pending data...\n");
    }

    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        memset(&fromAddr, 0, sizeof(fromAddr));
        fromLen = sizeof(fromAddr);

        int len = recvfrom(m_InsRobot->_local_sock, buffer, sizeof(buffer) - 1, 0,
                          (struct sockaddr*)&fromAddr, &fromLen);

        if (len <= 0)
        {
            break;
        }

        dataCount++;
        buffer[len] = '\0'; 

        if(m_InsRobot->m_LocalLogTag == true)
        {

            char fromIp[32];
            #ifdef CMPL_WIN
                sprintf(fromIp, "%s", inet_ntoa(fromAddr.sin_addr));
            #else
                inet_ntop(AF_INET, &(fromAddr.sin_addr), fromIp, sizeof(fromIp));
            #endif

            printf("[Marvin SDK]: Read pending data %d bytes from %s:%d\n",
                   len, fromIp, ntohs(fromAddr.sin_port));
        }
    }

    if(m_InsRobot->m_LocalLogTag == true && dataCount > 0)
    {
        printf("[Marvin SDK]: Read %d pending packets\n", dataCount);
    }
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

	long add_size = 1 + sizeof(FX_CHAR) * 32 + sizeof(FX_INT32) + sizeof(FX_FLOAT) + sizeof(FX_INT16);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return -1;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 150;
	m_InsRobot->m_Slen++;

	FX_CHAR* pName = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += 30 * sizeof(FX_CHAR);
	FX_CHAR* pType = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);
	FX_CHAR* pIns = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);


	FX_INT32* pValueInt = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_FLOAT* pValueFloat = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);

	FX_INT16* pSerial = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT16);

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
	long serial = m_InsRobot->m_ParaSerial;
	*pType = 1;
	*pIns = 101;
	*pValueInt = setValue;
	*pValueFloat = 0;
	*pSerial = serial;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

    if(m_InsRobot->m_LocalLogTag == true)
    {
        printf("[Marvin SDK]: Set int parameter: %s, value=%ld\n",paraName,setValue);
    }

    CRobot::OnSetSend();

	for (i = 0; i < 50; i++)
	{
#ifdef CMPL_WIN
		Sleep(2);
#endif
#ifdef CMPL_LIN
		usleep(2000);
#endif
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

	long add_size = 1 + sizeof(FX_CHAR) * 32 + sizeof(FX_INT32) + sizeof(FX_FLOAT) + sizeof(FX_INT16);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return -1;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 150;
	m_InsRobot->m_Slen++;

	FX_CHAR* pName = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += 30 * sizeof(FX_CHAR);
	FX_CHAR* pType = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);
	FX_CHAR* pIns = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);


	FX_INT32* pValueInt = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_FLOAT* pValueFloat = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);

	FX_INT16* pSerial = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT16);

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
	long serial = m_InsRobot->m_ParaSerial;
	*pType = 2;
	*pIns = 102;
	*pValueInt = 0;
	*pValueFloat = setValue;
	*pSerial = serial;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
    if(m_InsRobot->m_LocalLogTag == true)
    {
        printf("[Marvin SDK]: Set float parameter: %s, value=%lf\n",paraName,setValue);
    }
	CRobot::OnSetSend();

	for (i = 0; i < 50; i++)
	{
#ifdef CMPL_WIN
		Sleep(2);
#endif
#ifdef CMPL_LIN
		usleep(2000);
#endif


		long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
		if (ret_s % 100 == serial)
		{
			long ret_v = ret_s / 100;
			return ret_v;
		}
	}
	return -2;
}

long CRobot::OnGetIntPara(char paraName[30], long* retValue)
{
	if (paraName[29] != 0)
	{
		return -1;
	}

	if (OnClearSet() == false)
	{
		return -1;
	}

	long add_size = 1 + sizeof(FX_CHAR) * 32 + sizeof(FX_INT32) + sizeof(FX_FLOAT) + sizeof(FX_INT16);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return -1;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 150;
	m_InsRobot->m_Slen++;

	FX_CHAR* pName = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += 30 * sizeof(FX_CHAR);
	FX_CHAR* pType = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);
	FX_CHAR* pIns = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);


	FX_INT32* pValueInt = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_FLOAT* pValueFloat = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);

	FX_INT16* pSerial = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT16);

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
	long serial = m_InsRobot->m_ParaSerial;
	*pType = 1;
	*pIns = 103;
	*pValueInt = 0;
	*pValueFloat = 0;
	*pSerial = serial;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	CRobot::OnSetSend();

	for (i = 0; i < 50; i++)
	{
#ifdef CMPL_WIN
		Sleep(2);
#endif
#ifdef CMPL_LIN
		usleep(2000);
#endif


    long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
    if (ret_s % 100 == serial)
    {
        long ret_v = ret_s / 100;
        if (ret_v == 0)
        {
            *retValue = m_InsRobot->m_DCSS.m_ParaValueI;
        }
        if(m_InsRobot->m_LocalLogTag == true)
        {

            if (retValue != nullptr) {
            printf("[Marvin SDK]: Get int parameter: %s, value=%ld\n",paraName, *retValue);
        } else {
            printf("[Marvin SDK] retValue is null pointer.\n");
        }

        }
        return ret_v;
    }
	}
	return -2;
}

long CRobot::OnGetFloatPara(char paraName[30], double* retValue)
{
	if (paraName[29] != 0)
	{
		return -1;
	}

	if (OnClearSet() == false)
	{
		return -1;
	}

	long add_size = 1 + sizeof(FX_CHAR) * 32 + sizeof(FX_INT32) + sizeof(FX_FLOAT) + sizeof(FX_INT16);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return -1;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 150;
	m_InsRobot->m_Slen++;

	FX_CHAR* pName = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += 30 * sizeof(FX_CHAR);
	FX_CHAR* pType = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);
	FX_CHAR* pIns = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);


	FX_INT32* pValueInt = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_FLOAT* pValueFloat = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);

	FX_INT16* pSerial = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT16);

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
	long serial = m_InsRobot->m_ParaSerial;
	*pType = 2;
	*pIns = 104;
	*pValueInt = 0;
	*pValueFloat = 0;
	*pSerial = serial;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	CRobot::OnSetSend();

	for (i = 0; i < 50; i++)
	{
#ifdef CMPL_WIN
		Sleep(2);
#endif
#ifdef CMPL_LIN
		usleep(2000);
#endif


    long ret_s = m_InsRobot->m_DCSS.m_ParaRetSerial;
    if (ret_s % 100 == serial)
    {
        long ret_v = ret_s / 100;
        if (ret_v == 0)
        {
            *retValue = m_InsRobot->m_DCSS.m_ParaValueF;
        }

            if(m_InsRobot->m_LocalLogTag == true)
        {

            if (retValue != nullptr) {
            printf("[Marvin SDK]: Get float parameter: %s, value=%lf\n",paraName, *retValue);
        } else {
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

	long add_size = 1 + sizeof(FX_CHAR) * 32 + sizeof(FX_INT32) + sizeof(FX_FLOAT) +  sizeof(FX_INT16);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return -1;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 150;
	m_InsRobot->m_Slen++;
	
	FX_CHAR* pName = (FX_CHAR *) &m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += 30 * sizeof(FX_CHAR);
	FX_CHAR* pType = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen]; 
	m_InsRobot->m_Slen += sizeof(FX_CHAR);
	FX_CHAR* pIns = (FX_CHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_CHAR);


	FX_INT32* pValueInt = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_FLOAT* pValueFloat = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);

	FX_INT16* pSerial = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	m_InsRobot->m_Slen += sizeof(FX_INT16);

	long i;
	for ( i = 0; i < 30; i++)
	{
		pName[i] = 0;
	}
	m_InsRobot->m_ParaSerial++;
	if (m_InsRobot->m_ParaSerial >= 99)
	{
		m_InsRobot->m_ParaSerial = 1;
	}
	long serial = m_InsRobot->m_ParaSerial;
	*pType = 0;
	*pIns = 105;
	*pValueInt = 0;
	*pValueFloat = 0;
	*pSerial = serial;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Save parameters\n");
	}
	CRobot::OnSetSend();

	for ( i = 0; i < 50; i++)
	{
#ifdef CMPL_WIN
		Sleep(2);
#endif
#ifdef CMPL_LIN
		usleep(2000);
#endif


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
		int Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr*)&_local, &_localLen);
#endif
#ifdef CMPL_LIN
		int Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr*)&_local, (socklen_t*)&_localLen);
#endif

	while(Len > 0)
	{
		if (Len == sizeof(DCSS) + 2)
		{
			if (recvbuf[0] == 'F' && recvbuf[1] == 'X')
			{
				m_ACB_ShMem.WriteBuf((unsigned char *)recvbuf, Len);
				DCSS* p = (DCSS*)&recvbuf[2];
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
							v[i+2] = *m_GatherItem[i];
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
		else if(Len == sizeof(DDSS) + 2)
		{
			if (recvbuf[0] == 'C' && recvbuf[1] == 'H')
			{
				DDSS* p = (DDSS*)&recvbuf[2];
				if (p->m_CH == 1)
				{
					m_ACB1.WriteBuf((unsigned char *)p, sizeof(DDSS));
				}

				if (p->m_CH == 2)
				{
					m_ACB2.WriteBuf((unsigned char*)p, sizeof(DDSS));
				}

			}
		}

		
#ifdef CMPL_WIN
		 Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr*)&_local, &_localLen);
#endif
#ifdef CMPL_LIN
		 Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr*)&_local, (socklen_t*)&_localLen);
#endif


	}

}

void CRobot::DoSend()
{
	if (m_SendTag == 100)
	{
		int tt = sendto(_tosock_, (char*)m_SendBuf, m_Slen, 0, (struct sockaddr*)&_to, sizeof(_to));
		m_SendTag = 0;
		m_Slen = 0;

	}
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
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Stop collect data\n");
	}
	return true;
}

bool CRobot::OnSaveGatherData(char* path)
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

	bool ret =  m_InsRobot->m_GatherSet.OnSave(path);
	m_InsRobot->m_GatherTag = 0;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Save collected data to %s\n",path);
	}
	return ret;

}

bool CRobot::OnSaveGatherDataCSV(char* path)
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
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Collected csv saved path=%s\n",path);
	}
	return ret;

}

bool CRobot::OnStartGather(long targetNum, long targetID[35], long recordNum)
{
	if (targetNum < 0)
    {
        printf("OnStartGather ERROR: Invalid targetNum %ld (valid range: 0~35)\n", targetNum);
        return false;
    }
	else if (targetNum >35)
    {
        printf("OnStartGather WARINING: targetNum %ld exceeds maximum, set to 35)\n", targetNum);
        targetNum=35;
    }
    if (recordNum < 1000)
    {
        printf("OnStartGather WARNING: recordNum %ld is below minimum, set to 1000\n", recordNum);
        recordNum = 1000;
    }
    else if (recordNum > 1000000)
    {
        printf("OnStartGather WARNING : recordNum %ld exceeds maximum, set to 1000000\n", recordNum);
        recordNum = 1000000;
    }

	GetIns();
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Collect data settings\n");
		printf("targetNum=%d\n",targetNum);
		printf("targetID= [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\n",
		targetID[0],targetID[1],targetID[2],targetID[3],targetID[4],targetID[5],targetID[6],
		targetID[7],targetID[8],targetID[9],targetID[10],targetID[11],targetID[12],targetID[13],
		targetID[14],targetID[15],targetID[16],targetID[17],targetID[18],targetID[19],targetID[20],
		targetID[21],targetID[22],targetID[23],targetID[24],targetID[25],targetID[26],targetID[27],
		targetID[28],targetID[29],targetID[30],targetID[31],targetID[32],targetID[33],targetID[34]);
		printf("recordNum=%d\n",recordNum);
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
	for ( i = 0; i < targetNum; i++)
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
				m_InsRobot->m_GatherItem[i] = & m_InsRobot->m_DCSS.m_Out[0].m_FB_Joint_Pos[pos];
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
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Data collected, targetNum=%ld, recordNum=%ld\n",targetNum,recordNum);
	    printf("targetID= [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\n",
		targetID[0],targetID[1],targetID[2],targetID[3],targetID[4],targetID[5],targetID[6],
		targetID[7],targetID[8],targetID[9],targetID[10],targetID[11],targetID[12],targetID[13],
		targetID[14],targetID[15],targetID[16],targetID[17],targetID[18],targetID[19],targetID[20],
		targetID[21],targetID[22],targetID[23],targetID[24],targetID[25],targetID[26],targetID[27],
		targetID[28],targetID[29],targetID[30],targetID[31],targetID[32],targetID[33],targetID[34]);
	}
	m_InsRobot->m_GatherTag = true;

	return true;


}

bool CRobot::OnClearSet()
{
	if (m_InsRobot->m_SendTag == 100)
	{
		return false;
	}
	m_InsRobot->m_SendTag = 0;
	m_InsRobot->m_SendBuf[0] = 'F';
	m_InsRobot->m_SendBuf[1] = 'X';
	m_InsRobot->m_SendBuf[2] = 0;
	m_InsRobot->m_Slen = 3;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Clear set\n");
	}

	return true;
}

bool CRobot::OnSetJointLmt_A(int velRatio, int AccRatio)
{
	if (velRatio < 1)
    {
        printf("OnSetJointLmt_A WARNING: velRatio %d is below minimum, set to 1\n", velRatio);
        velRatio = 1;
    }
    else if (velRatio > 100)
    {
        printf("OnSetJointLmt_A WARNING: velRatio %d exceeds maximum, set to 100\n", velRatio);
        velRatio = 100;
    }
    if (AccRatio < 1)
    {
        printf("OnSetJointLmt_A WARNING: AccRatio %d is below minimum, set to 1\n", AccRatio);
        AccRatio = 1;
    }
    else if (AccRatio > 100)
    {
        printf("OnSetJointLmt_A WARNING: AccRatio %d exceeds maximum, set to 100\n", AccRatio);
        AccRatio = 100;
    }
	long add_size = 1 + sizeof(FX_INT16) * 2;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 103;
	m_InsRobot->m_Slen++;
	FX_INT16* pv = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	if (velRatio < 1)
	{
		velRatio = 1;
	}
	if (velRatio > 100)
	{
		velRatio = 100;
	}

	if (AccRatio < 1)
	{
		AccRatio = 1;
	}
	if (AccRatio > 100)
	{
		AccRatio = 100;
	}

	pv[0] = velRatio;
	pv[1] = AccRatio;
	

	m_InsRobot->m_Slen += sizeof(FX_INT16) * 2;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set A arm velRatio=%d, AccRatio=%d \n",velRatio,AccRatio);
	}

	return true;

}

bool CRobot::OnSetPVT_A(int id)
{
	if (id < 0 || id >= 100)
	{
		printf("OnSetPVT_A ERROR: Invalid id  %d (valid range: 0~99)n", id);
		return false;
	}
	long add_size = 1 + sizeof(FX_UCHAR);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 110;
	m_InsRobot->m_Slen++;

	FX_UCHAR* pv = (FX_UCHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	pv[0] = id;
	m_InsRobot->m_Slen += sizeof(FX_UCHAR);
	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set A arm PVT id=%d\n",id);
	}

	return true;
}

bool CRobot::OnSetPVT_B(int id)
{
	if (id < 0 || id >= 100)
	{
		printf("OnSetPVT_B ERROR: Invalid id  %d (valid range: 0~99)n", id);
		return false;
	}
	long add_size = 1 + sizeof(FX_UCHAR);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 210;
	m_InsRobot->m_Slen++;

	FX_UCHAR* pv = (FX_UCHAR*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	pv[0] = id;
	m_InsRobot->m_Slen += sizeof(FX_UCHAR);
	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set B arm PVT id=%d\n",id);
	}

	return true;
}

bool CRobot::OnSetForceCmd_A(double force)
{
    if (isnan(force) || isinf(force))
	{
		printf("OnSetForceCmd_A ERROR: force %d is invalid (NaN or Inf)\n");
		return false;
	}
	long add_size = 1 + sizeof(FX_FLOAT);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 109;
	m_InsRobot->m_Slen++;

	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	pv[0] = force;
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);
	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set A arm force cmd force=%lf\n",force);
	}

	return true;
}

bool CRobot::OnSetJointCmdPos_A(double joint[7])
{
	if (joint == nullptr)
    {
        printf("OnSetJointCmdPos_A ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(joint[i]) || isinf(joint[i]))
        {
            printf("OnSetJointCmdPos_A ERROR: joint[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	long add_size = 1 + sizeof(FX_FLOAT) * 7;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 108;
	m_InsRobot->m_Slen++;

	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	
	pv[0] = joint[0];
	pv[1] = joint[1];
	pv[2] = joint[2];
	pv[3] = joint[3];
	pv[4] = joint[4];
	pv[5] = joint[5];
	pv[6] = joint[6];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 7;
	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Set A arm joint cmd pose, joints=[%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]);
	}

	return true;
}

bool CRobot::OnInitPlnLmt(char * path)
{
	double vel[8]={0.0};
    double acc[8]={0.0};
    double npos[8]={0.0};
    double ppos[8]={0.0};

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
	FILE* fp = fopen(path, "rb");
	if (fp == NULL)
	{
		return false;
	}
    for ( i = 0; i < 2; i++)
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
		for ( j = 0; j < 7; j++)
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
		if (fscanf(fp, "%lf,%lf,%lf,%lf,%c",&DH[i][7][0], &DH[i][7][1], &DH[i][7][2], &DH[i][7][3],
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
				 &BD[i][j][0], &BD[i][j][1], &BD[i][j][2],&c) != 4)
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
		m_InsRobot->pln_A.OnSetLmt(7,npos,ppos,vel,acc);
	}

	for (j = 0; j < 7; j++)
    {
        npos[j] = NPVA[1][j][0];  
        ppos[j] = NPVA[1][j][1];  
        vel[j] = NPVA[1][j][2];   
        acc[j] = NPVA[1][j][3];   

    }
	{
		m_InsRobot->pln_B.OnSetLmt(7,npos,ppos,vel,acc);
	}
	return true;
}

bool CRobot::OnSetPlnCart_A(CPointSet* pset)
{
	DCSS t;

	long num = pset->OnGetPointNum();
	if(num <=5)
	{
		return false;
	}

	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_A(num);
	if( CRobot::OnSetSendWaitResponse(100)<0)
	{
		return false;
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState !=1)
		{
			return false;
		}
	}

	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii,jj,kk;

	double SendData[350];
	double * retp;
	long spos;
	long ipos = 0;
	for ( ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for ( jj = 0; jj< 50; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(ii,50,SendData);
		if( CRobot::OnSetSendWaitResponse(100)<0)
		{
			return false;
		}

	}
	

	if(relic_num != 0)
	{
		spos = 0;
		for ( jj = 0; jj< relic_num; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(send_g_num,relic_num,SendData);
		if( CRobot::OnSetSendWaitResponse(200)<0)
		{
			return false;
		}
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState !=2)
		{
			return false;
		}
	}


	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_A();
	if( CRobot::OnSetSendWaitResponse(50)<0)
	{
		return false;
	}

	return true;
}



bool CRobot::OnSetPlnCart_B(CPointSet* pset)
{
	DCSS t;

	long num = pset->OnGetPointNum();
	if(num <=5)
	{
		return false;
	}

	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_B(num);
	if( CRobot::OnSetSendWaitResponse(100)<0)
	{
		return false;
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState !=1)
		{
			return false;
		}
	}

	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii,jj,kk;

	double SendData[350];
	double * retp;
	long spos;
	long ipos = 0;
	for ( ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for ( jj = 0; jj< 50; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(ii,50,SendData);
		if( CRobot::OnSetSendWaitResponse(100)<0)
		{
			return false;
		}

	}
	

	if(relic_num != 0)
	{
		spos = 0;
		for ( jj = 0; jj< relic_num; jj++)
		{
			retp = pset->OnGetPoint(ipos);
			ipos++;
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(send_g_num,relic_num,SendData);
		if( CRobot::OnSetSendWaitResponse(200)<0)
		{
			return false;
		}
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState !=2)
		{
			return false;
		}
	}


	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_B();
	if( CRobot::OnSetSendWaitResponse(50)<0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetPlnJoint_A(double start_joints[7], double stop_joints[7], double vel_ratio,double acc_ratio)
{
	if (start_joints == nullptr || stop_joints==nullptr)
    {
        printf("OnSetPlnJoint_A ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(start_joints[i]) || isinf(start_joints[i]))
        {
            printf("OnSetPlnJoint_A ERROR: start_joints[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(stop_joints[i]) || isinf(stop_joints[i]))
        {
            printf("OnSetPlnJoint_A ERROR: stop_joints[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	if (vel_ratio < 0)
    {
        printf("OnSetPlnJoint_A ERROR: Invalid vel_ratio %d (valid range: 0~1)\n", vel_ratio);
        return false;
    }
	if (vel_ratio > 1)
    {
        printf("OnSetPlnJoint_A WARNING: Invalid vel_ratio %d (valid range: 0~1), set vel_ratio to 1\n", vel_ratio);
        vel_ratio=1.0;
    }
	if (acc_ratio < 0)
    {
        printf("OnSetPlnJoint_A ERROR: Invalid acc_ratio %d (valid range: 0~1)\n", acc_ratio);
        return false;
    }
	if (acc_ratio > 1)
    {
        printf("OnSetPlnJoint_A WARNING: Invalid acc_ratio %d (valid range: 0~1), set acc_ratio to 1\n", acc_ratio);
        acc_ratio=1.0;
    }


	DCSS t;

	double vr = vel_ratio;
	double ar = acc_ratio;

	if(vr < 0.01 ) vr = 0.01;
	if(vr > 1.0  ) vr = 1.0;
	if(ar < 0.01 ) ar = 0.01;
	if(ar > 1.0  ) ar = 1.0;
    
    long i = 0;
	double sta[8] = {0};
	double sto[8] = {0};
	for ( i = 0; i < 7; i++)
	{
		sta[i] = start_joints[i];
		sto[i] = stop_joints[i];
	}
	
    long num = m_InsRobot->pln_A.OnPln(sta, sto, vr, ar);
	if(num <=0)
	{
		return false;
	}

	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_A(num);
	if( CRobot::OnSetSendWaitResponse(100)<0)
	{
		return false;
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState !=1)
		{
			return false;
		}
	}

	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii,jj,kk;

	double SendData[350];
	double retp[8];
	long spos;
	for ( ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for ( jj = 0; jj< 50; jj++)
		{
			m_InsRobot->pln_A.OnCut(retp);
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(ii,50,SendData);
		if( CRobot::OnSetSendWaitResponse(100)<0)
		{
			return false;
		}

	}
	

	if(relic_num != 0)
	{
		spos = 0;
		for ( jj = 0; jj< relic_num; jj++)
		{
			m_InsRobot->pln_A.OnCut(retp);
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_A(send_g_num,relic_num,SendData);
		if( CRobot::OnSetSendWaitResponse(200)<0)
		{
			return false;
		}
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[0].m_TrajState !=2)
		{
			return false;
		}
	}


	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_A();
	if( CRobot::OnSetSendWaitResponse(50)<0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetTrajInit_A(int pointNum)
{
	if (pointNum < 0)
	{
		return false;
	}

	long add_size = 1 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 112;
	m_InsRobot->m_Slen++;
	FX_INT32* pv = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = pointNum;

	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Initial A arm Planning Trajectory , number of point=%d,\n",pointNum);
	}
	return true;
}

bool CRobot::OnSetTrajRun_A()
{
	long add_size = 1 + sizeof(FX_FLOAT) * 1;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 114;
	m_InsRobot->m_Slen++;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: A arm run Planning Trajectory \n");
	}
	return true;
}


bool CRobot::OnStopPlnJoint_A()
{
	CRobot::OnClearSet();
	CRobot::OnStopPlnJoint_interA();
	if( CRobot::OnSetSendWaitResponse(50)<0)
	{
		return false;
	}else
	{
		if(m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK]: A arm stop run Planning Trajectory \n");
		}
		return true;
	}
}


bool CRobot::OnStopPlnJoint_interA()
{
	long add_size = 1 + sizeof(FX_FLOAT) * 1;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 115;
	m_InsRobot->m_Slen++;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	return true;
}

bool CRobot::OnSetTrajSet_A(long serial,long pointNum, double* data)
{
	if ( m_InsRobot->m_Slen !=3)
	{
		return false;
	}
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 113;
	m_InsRobot->m_Slen++;

	FX_INT32 * pserial = (FX_INT32 *)& m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	* pserial = serial;
	m_InsRobot->m_Slen+= sizeof(FX_INT32);

	FX_FLOAT * pdat = (FX_FLOAT * ) & m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	long i;
    long j;
    long spos = 0;
    // printf("\n=== Trajectory Points (total %ld points) ===\n", pointNum);
    // printf("     Point    J1      J2      J3      J4      J5      J6      J7\n");
    // printf("--------------------------------------------------------\n");
    
    for(i = 0; i < pointNum; i++)
    {
        // printf("  %6ld: ", i);  // 打印点序号，右对齐
        for(j = 0; j < 7; j++)
        {
            pdat[spos] = data[spos];
            // printf("%7.3f ", data[spos]);  // 每个数据宽度7，保留3位小数
            spos++;
        }
        // printf("\n");
    }
    // printf("========================================================\n");

	m_InsRobot->m_Slen+= sizeof(FX_FLOAT) * 7 * pointNum;


	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;


	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: A arm run Planning Trajectory \n");
	}
	return true;
    
}

bool CRobot::OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	if (fxDir == nullptr || fcCtrlPara == nullptr)
    {
        printf("OnSetForceCtrPara_A ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 6; ++i)
    {
        if (isnan(fxDir[i]) || isinf(fxDir[i]))
        {
            printf("OnSetForceCtrPara_A ERROR: fxDir[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(fcCtrlPara[i]) || isinf(fcCtrlPara[i]))
        {
            printf("OnSetForceCtrPara_A ERROR: fcCtrlPara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	if (fcType!=0)
        {
            printf("OnSetForceCtrPara_A WARNING: fcType set to 0\n",fcType);
            fcType=0;
        }
	if (isnan(fcAdjLmt) || isinf(fcAdjLmt))
	{
		printf("OnSetForceCtrPara_A ERROR: fcAdjLmt %d is invalid (NaN or Inf)\n");
		return false;
	}
	long add_size = 1 + sizeof(FX_FLOAT) * 14 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 107;
	m_InsRobot->m_Slen++;

	FX_INT32* pv1 = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv1[0] = fcType;

	m_InsRobot->m_Slen += sizeof(FX_INT32);
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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

	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm force control parameters\n fcType=%d,\n",fcType);
	if(m_InsRobot->m_LocalLogTag == true) printf("fcDirection=[%.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf],\n",fxDir[0],fxDir[1],fxDir[2],fxDir[3],fxDir[4],fxDir[5]);
	if(m_InsRobot->m_LocalLogTag == true) printf("fcCtrlPara=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\n",fcCtrlPara[0],fcCtrlPara[1],fcCtrlPara[2],fcCtrlPara[3],fcCtrlPara[4],fcCtrlPara[5],fcCtrlPara[6]);
	if(m_InsRobot->m_LocalLogTag == true) printf("fcAdjLmt=%lf\n",fcAdjLmt);
	return true;
}

bool CRobot::OnSetDragSpace_A(int zsType)
{
	if (zsType <0 || zsType >5)
    {
        printf("OnSetDragSpace_A ERROR: Invalid zsType  %d (valid range: 0~5), exit drag mode\n", zsType);
		zsType=0;
    } 

	long add_size = 1 + sizeof(FX_FLOAT) * 6 + sizeof(FX_INT32) ;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 106;
	m_InsRobot->m_Slen++;

	FX_INT32* pv1 = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv1[0] = zsType;

	m_InsRobot->m_Slen += sizeof(FX_INT32);
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = 0;
	pv[1] = 0;
	pv[2] = 0;
	pv[3] = 0;
	pv[4] = 0;
	pv[5] = 0;



	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 6;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm drag space type=%d\n",zsType);
	return true;
}

bool CRobot::OnSetCartKD_A(double K[7], double D[7],int type)
{
	if (K == nullptr || D == nullptr)
    {
        printf("OnSetCartKD_A ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(K[i]) || isinf(K[i]))
        {
            printf("OnSetCartKD_A ERROR: K[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (K[i] < 0.0)
        {
            printf("OnSetCartKD_A WARNING: K[%d] is negative (%f), set to 0\n", i, K[i]);
            K[i] = 0.0;
        }
    }
    for (int i = 0; i < 7; ++i)
    {
        if (isnan(D[i]) || isinf(D[i]))
        {
            printf("OnSetCartKD_A ERROR: D[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (D[i] < 0.0)
        {
            printf("OnSetCartKD_A WARNING: D[%d] is negative (%f), set to 0\n", i, D[i]);
            D[i] = 0.0;
        }
        else if (D[i] > 1.0)
        {
            printf("OnSetCartKD_A WARNING: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
            D[i] = 1.0;
        }
    }
	if (type !=2)
	{
		type=2;
	}

	long add_size = 1 + sizeof(FX_FLOAT) * 14 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 105;
	m_InsRobot->m_Slen++;
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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


	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_INT32* pi = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pi[0] = type;
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm Cart k=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\nD=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\ntype=%d\n",
	K[0],K[1],K[2],K[3],K[4],K[5],K[6],
	D[0],D[1],D[2],D[3],D[4],D[5],D[6],
	type);

	return true;
}

bool CRobot::OnSetEefRot_A(int fcType, double CartCtrlPara[7])
{
	if (CartCtrlPara == nullptr)
    {
        printf("OnSetEefRot_A ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(CartCtrlPara[i]) || isinf(CartCtrlPara[i]))
        {
            printf("OnSetEefRot_A ERROR: CartCtrlPara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	if (fcType != 1 && fcType != 2)
    {
        printf("OnSetEefRot_A ERROR: Invalid fcType number %d (valid range: 1~2)\n", fcType);
		return false;
    }

	long add_size = 1 + sizeof(FX_FLOAT) * 14 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 107;
	m_InsRobot->m_Slen++;

	FX_INT32* pv1 = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv1[0] = fcType;

	m_InsRobot->m_Slen += sizeof(FX_INT32);
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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

	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm flange Cart rotation parameters.\n fcType=%d,\n",fcType);
    if(m_InsRobot->m_LocalLogTag == true) printf("CartCtrlPara=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\n",CartCtrlPara[0],CartCtrlPara[1],CartCtrlPara[2],CartCtrlPara[3],CartCtrlPara[4],CartCtrlPara[5],CartCtrlPara[6]);
	return true;
}

bool CRobot::OnSetJointKD_A(double K[7], double D[7])
{
	
	if (K == nullptr || D == nullptr)
    {
        printf("OnSetJointKD_A ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(K[i]) || isinf(K[i]))
        {
            printf("OnSetJointKD_A ERROR: K[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (K[i] < 0.0)
        {
            printf("OnSetJointKD_A WARNING: K[%d] is negative (%f), set to 0\n", i, K[i]);
            K[i] = 0.0;
        }
    }
    for (int i = 0; i < 7; ++i)
    {
        if (isnan(D[i]) || isinf(D[i]))
        {
            printf("OnSetJointKD_A ERROR: D[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (D[i] < 0.0)
        {
            printf("OnSetJointKD_A WARNING: D[%d] is negative (%f), set to 0\n", i, D[i]);
            D[i] = 0.0;
        }
        else if (D[i] > 1.0)
        {
            printf("OnSetJointKD_A WARNING: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
            D[i] = 1.0;
        }
    }
	long add_size = 1 + sizeof(FX_FLOAT) * 14;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 104;
	m_InsRobot->m_Slen++;
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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


	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm joint k=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\nD=[%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",
	K[0],K[1],K[2],K[3],K[4],K[5],K[6],
	D[0],D[1],D[2],D[3],D[4],D[5],D[6]);

	return true;
}

bool CRobot::OnSetTool_A(double kinePara[6], double dynPara[10])
{
	if (kinePara == nullptr || dynPara == nullptr)
    {
        printf("OnSetTool_A ERROR: Null pointer input\n");
        return false;
    }
    for (int i = 0; i < 6; ++i)
    {
        if (isnan(kinePara[i]) || isinf(kinePara[i]))
        {
            printf("OnSetTool_A ERROR: kinePara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
    }
    for (int i = 0; i < 10; ++i)
    {
         if (isnan(dynPara[i]) || isinf(dynPara[i]))
        {
            printf("OnSetTool_A ERROR: dynPara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
    }
	
	long add_size = 1 + sizeof(FX_FLOAT) * 16;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 102;
	m_InsRobot->m_Slen++;
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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


	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 16;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm tool kineParams=[%lf,%lf,%lf,%lf,%lf,%lf]\n dynamicParams=[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",
	kinePara[0],kinePara[1],kinePara[2],kinePara[3],kinePara[4],kinePara[5],
	dynPara[0],dynPara[1],dynPara[2],dynPara[3],dynPara[4],dynPara[5],dynPara[6],dynPara[7],dynPara[8],dynPara[9]);

	return true;
}

bool CRobot::OnSetTargetState_A(int state)
{
	if(state<0 || state>4)
	{
		printf("OnSetTargetState_A ERROR: Invalid state %d (valid range: 0~4), set state to 0(disable)\n ",state);
		state=0;
	}
	long add_size = 1 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 101;
	m_InsRobot->m_Slen++;
	FX_INT32* pv = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = state;

	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm state=%d\n",state);
	return true;
}

bool CRobot::OnSetImpType_A(int type)
{
	if (type != 1 && type != 2 && type != 3)
    {
        printf("OnSetImpType_A ERROR: Invalid type  %d (valid range: 1~3)\n", type);
		return false;
    }
	long add_size = 1 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 111;
	m_InsRobot->m_Slen++;
	FX_INT32* pv = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = type;

	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set A arm impedance type=%d\n",type);
	return true;
}

///////////////////////////////////

bool CRobot::OnSetJointLmt_B(int velRatio, int AccRatio)
{
	if (velRatio < 1)
    {
        printf("OnSetJointLmt_B WARNING: velRatio %d is below minimum, set to 1\n", velRatio);
        velRatio = 1;
    }
    else if (velRatio > 100)
    {
        printf("OnSetJointLmt_B WARNING: velRatio %d exceeds maximum, set to 100\n", velRatio);
        velRatio = 100;
    }
    if (AccRatio < 1)
    {
        printf("OnSetJointLmt_B WARNING: AccRatio %d is below minimum, set to 1\n", AccRatio);
        AccRatio = 1;
    }
    else if (AccRatio > 100)
    {
        printf("OnSetJointLmt_B WARNING: AccRatio %d exceeds maximum, set to 100\n", AccRatio);
        AccRatio = 100;
    }
	long add_size = 1 + sizeof(FX_INT16) * 2;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 203;
	m_InsRobot->m_Slen++;
	FX_INT16* pv = (FX_INT16*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	if (velRatio < 1)
	{
		velRatio = 1;
	}
	if (velRatio > 100)
	{
		velRatio = 100;
	}

	if (AccRatio < 1)
	{
		AccRatio = 1;
	}
	if (AccRatio > 100)
	{
		AccRatio = 100;
	}

	pv[0] = velRatio;
	pv[1] = AccRatio;


	m_InsRobot->m_Slen += sizeof(FX_INT16) * 2;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm velRatio=%d, AccRatio=%d \n",velRatio,AccRatio);
	return true;

}

bool CRobot::OnSetForceCmd_B(double force)
{
	if (isnan(force) || isinf(force))
	{
		printf("OnSetForceCmd_B ERROR: force %d is invalid (NaN or Inf)\n");
		return false;
	}
	long add_size = 1 + sizeof(FX_FLOAT);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 209;
	m_InsRobot->m_Slen++;

	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	pv[0] = force;
	m_InsRobot->m_Slen += sizeof(FX_FLOAT);
	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm force cmd force=%lf\n",force);
	return true;
}

bool CRobot::OnSetJointCmdPos_B(double joint[7])
{
	if (joint == nullptr)
    {
        printf("OnSetJointCmdPos_B ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(joint[i]) || isinf(joint[i]))
        {
            printf("OnSetJointCmdPos_B ERROR: joint[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	long add_size = 1 + sizeof(FX_FLOAT) * 7;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 208;
	m_InsRobot->m_Slen++;

	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	pv[0] = joint[0];
	pv[1] = joint[1];
	pv[2] = joint[2];
	pv[3] = joint[3];
	pv[4] = joint[4];
	pv[5] = joint[5];
	pv[6] = joint[6];
	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 7;
	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm joint cmd pose, joints=[%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",
	joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]);

	return true;
}

bool CRobot::OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	if (fxDir == nullptr || fcCtrlPara == nullptr)
    {
        printf("OnSetForceCtrPara_B ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 6; ++i)
    {
        if (isnan(fxDir[i]) || isinf(fxDir[i]))
        {
            printf("OnSetForceCtrPara_B ERROR: fxDir[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(fcCtrlPara[i]) || isinf(fcCtrlPara[i]))
        {
            printf("OnSetForceCtrPara_B ERROR: fcCtrlPara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	if (fcType!=0)
	{
		printf("OnSetForceCtrPara_B WARNING: fcType set to 0\n",fcType);
        fcType=0;
    }
	if (isnan(fcAdjLmt) || isinf(fcAdjLmt))
        {
            printf("OnSetForceCtrPara_B ERROR: fcAdjLmt %d is invalid (NaN or Inf)\n");
            return false;
        }
		
	long add_size = 1 + sizeof(FX_FLOAT) * 14 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 207;
	m_InsRobot->m_Slen++;

	FX_INT32* pv1 = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv1[0] = fcType;

	m_InsRobot->m_Slen += sizeof(FX_INT32);
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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

	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm force control parameters\n fcType=%d,\n",fcType);
	if(m_InsRobot->m_LocalLogTag == true) printf("fcDirection=[%.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf],\n",fxDir[0],fxDir[1],fxDir[2],fxDir[3],fxDir[4],fxDir[5]);
	if(m_InsRobot->m_LocalLogTag == true) printf("fcCtrlPara=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\n",fcCtrlPara[0],fcCtrlPara[1],fcCtrlPara[2],fcCtrlPara[3],fcCtrlPara[4],fcCtrlPara[5],fcCtrlPara[6]);
	if(m_InsRobot->m_LocalLogTag == true) printf("fcAdjLmt=%lf\n",fcAdjLmt);

	return true;
}

bool CRobot::OnSetDragSpace_B(int zsType)
{
	if (zsType <0 || zsType >5)
    {
        printf("OnSetDragSpace_B ERROR: Invalid zsType  %d (valid range: 0~5), exit drag mode\n", zsType);
		zsType=0;
    } 
	long add_size = 1 + sizeof(FX_FLOAT) * 6 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 206;
	m_InsRobot->m_Slen++;

	FX_INT32* pv1 = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv1[0] = zsType;

	m_InsRobot->m_Slen += sizeof(FX_INT32);
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = 0;
	pv[1] = 0;
	pv[2] = 0;
	pv[3] = 0;
	pv[4] = 0;
	pv[5] = 0;



	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 6;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm drag space type=%d\n",zsType);
	return true;
}

bool CRobot::OnSetCartKD_B(double K[7], double D[7],int type)
{
	if (K == nullptr || D == nullptr)
    {
        printf("OnSetCartKD_B ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(K[i]) || isinf(K[i]))
        {
            printf("OnSetCartKD_B ERROR: K[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (K[i] < 0.0)
        {
            printf("OnSetCartKD_B WARNING: K[%d] is negative (%f), set to 0\n", i, K[i]);
            K[i] = 0.0;
        }
    }
    for (int i = 0; i < 7; ++i)
    {
        if (isnan(D[i]) || isinf(D[i]))
        {
            printf("OnSetCartKD_B ERROR: D[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (D[i] < 0.0)
        {
            printf("OnSetCartKD_B WARNING: D[%d] is negative (%f), set to 0\n", i, D[i]);
            D[i] = 0.0;
        }
        else if (D[i] > 1.0)
        {
            printf("OnSetCartKD_B WARNING: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
            D[i] = 1.0;
        }
    }
	if (type !=2)
	{
		type=2;
	}
	long add_size = 1 + sizeof(FX_FLOAT) * 14 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 205;
	m_InsRobot->m_Slen++;
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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


	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_INT32* pi = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pi[0] = type;
	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm Cart k=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\nD=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\ntype=%d\n",
	K[0],K[1],K[2],K[3],K[4],K[5],K[6],
	D[0],D[1],D[2],D[3],D[4],D[5],D[6],
	type);

	return true;
}

bool CRobot::OnSetEefRot_B(int fcType, double CartCtrlPara[7])
{
	if (CartCtrlPara == nullptr)
    {
        printf("OnSetEefRot_B ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(CartCtrlPara[i]) || isinf(CartCtrlPara[i]))
        {
            printf("OnSetEefRot_B ERROR: CartCtrlPara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	if (fcType != 1 && fcType != 2)
    {
        printf("OnSetEefRot_B ERROR: Invalid fcType number %d (valid range: 1~2)\n", fcType);
		return false;
    }

	long add_size = 1 + sizeof(FX_FLOAT) * 14 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 207;
	m_InsRobot->m_Slen++;

	FX_INT32* pv1 = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv1[0] = fcType;

	m_InsRobot->m_Slen += sizeof(FX_INT32);
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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

	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm flange Cart rotation parameters.\n fcType=%d,\n",fcType);
    if(m_InsRobot->m_LocalLogTag == true) printf("CartCtrlPara=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\n",CartCtrlPara[0],CartCtrlPara[1],CartCtrlPara[2],CartCtrlPara[3],CartCtrlPara[4],CartCtrlPara[5],CartCtrlPara[6]);
	return true;

}

bool CRobot::OnSetJointKD_B(double K[7], double D[7])
{
	if (K == nullptr || D == nullptr)
    {
        printf("OnSetJointKD_B ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(K[i]) || isinf(K[i]))
        {
            printf("OnSetJointKD_B ERROR: K[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (K[i] < 0.0)
        {
            printf("OnSetJointKD_B WARNING: K[%d] is negative (%f), set to 0\n", i, K[i]);
            K[i] = 0.0;
        }
    }
    for (int i = 0; i < 7; ++i)
    {
        if (isnan(D[i]) || isinf(D[i]))
        {
            printf("OnSetJointKD_B ERROR: D[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
		if (D[i] < 0.0)
        {
            printf("OnSetJointKD_B WARNING: D[%d] is negative (%f), set to 0\n", i, D[i]);
            D[i] = 0.0;
        }
        else if (D[i] > 1.0)
        {
            printf("OnSetJointKD_B WARNING: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
            D[i] = 1.0;
        }
    }
	long add_size = 1 + sizeof(FX_FLOAT) * 14;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 204;
	m_InsRobot->m_Slen++;
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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


	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 14;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm joint k=[%lf,%lf,%lf,%lf,%lf,%lf,%lf],\nD=[%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",
	K[0],K[1],K[2],K[3],K[4],K[5],K[6],
	D[0],D[1],D[2],D[3],D[4],D[5],D[6]);

	return true;
}

bool CRobot::OnSetTool_B(double kinePara[6], double dynPara[10])
{
	if (kinePara == nullptr || dynPara == nullptr)
    {
        printf("OnSetTool_B ERROR: Null pointer input\n");
        return false;
    }
    for (int i = 0; i < 6; ++i)
    {
        if (isnan(kinePara[i]) || isinf(kinePara[i]))
        {
            printf("OnSetTool_B ERROR: kinePara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
    }
    for (int i = 0; i < 10; ++i)
    {
        if (isnan(dynPara[i]) || isinf(dynPara[i]))
        {
            printf("OnSetTool_B ERROR: dynPara[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
    }
	
	long add_size = 1 + sizeof(FX_FLOAT) * 16;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 202;
	m_InsRobot->m_Slen++;
	FX_FLOAT* pv = (FX_FLOAT*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
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


	m_InsRobot->m_Slen += sizeof(FX_FLOAT) * 16;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm tool kineParams=[%lf,%lf,%lf,%lf,%lf,%lf]\n dynamicParams=[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",
	kinePara[0],kinePara[1],kinePara[2],kinePara[3],kinePara[4],kinePara[5],
	dynPara[0],dynPara[1],dynPara[2],dynPara[3],dynPara[4],dynPara[5],dynPara[6],dynPara[7],dynPara[8],dynPara[9]);

	return true;
}

bool CRobot::OnSetTargetState_B(int state)
{
	if(state<0 || state>4)
	{
		printf("OnSetTargetState_B ERROR: Invalid state %d (valid range: 0~4), set state to 0(disable)\n ",state);
		state=0;
	}
	long add_size = 1 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 201;
	m_InsRobot->m_Slen++;
	FX_INT32* pv = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = state;

	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm state=%d\n",state);

	return true;
}

bool CRobot::OnSetImpType_B(int type)
{
	if (type != 1 && type != 2 && type != 3)
    {
        printf("OnSetImpType_B ERROR: Invalid type  %d (valid range: 1~3)\n", type);
		return false;
    }
	long add_size = 1 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 211;
	m_InsRobot->m_Slen++;
	FX_INT32* pv = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = type;

	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
    if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Set B arm impedance type=%d\n",type);

	return true;
}

bool CRobot::OnSetPlnJoint_B(double start_joints[7], double stop_joints[7], double vel_ratio,double acc_ratio)
{
	if (start_joints == nullptr || stop_joints==nullptr)
    {
        printf("OnSetPlnJoint_B ERROR: Null pointer input\n");
        return false;
    }
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(start_joints[i]) || isinf(start_joints[i]))
        {
            printf("OnSetPlnJoint_B ERROR: start_joints[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	for (int i = 0; i < 7; ++i)
    {
        if (isnan(stop_joints[i]) || isinf(stop_joints[i]))
        {
            printf("OnSetPlnJoint_B ERROR: stop_joints[%d] is invalid (NaN or Inf)\n", i);
            return false;
        }
	}
	if (vel_ratio < 0)
    {
        printf("OnSetPlnJoint_B ERROR: Invalid vel_ratio %d (valid range: 0~1)\n", vel_ratio);
        return false;
    }
	if (vel_ratio > 1)
    {
        printf("OnSetPlnJoint_B WARNING: Invalid vel_ratio %d (valid range: 0~1), set vel_ratio to 1\n", vel_ratio);
        vel_ratio=1.0;
    }
	if (acc_ratio < 0)
    {
        printf("OnSetPlnJoint_B ERROR: Invalid acc_ratio %d (valid range: 0~1)\n", acc_ratio);
        return false;
    }
	if (acc_ratio > 1)
    {
        printf("OnSetPlnJoint_B WARNING: Invalid acc_ratio %d (valid range: 0~1), set acc_ratio to 1\n", acc_ratio);
        acc_ratio=1.0;
    }
	DCSS t;

	double vr = vel_ratio;
	double ar = acc_ratio;

	if(vr < 0.01 ) vr = 0.01;
	if(vr > 1.0  ) vr = 1.0;
	if(ar < 0.01 ) ar = 0.01;
	if(ar > 1.0  ) ar = 1.0;
    
    long i = 0;
	double sta[8] = {0};
	double sto[8] = {0};
	for ( i = 0; i < 7; i++)
	{
		sta[i] = start_joints[i];
		sto[i] = stop_joints[i];
	}
	
    long num = m_InsRobot->pln_B.OnPln(sta, sto, vr, ar);
	if(num <=0)
	{
		return false;
	}

	CRobot::OnClearSet();
	CRobot::OnSetTrajInit_B(num);
	if( CRobot::OnSetSendWaitResponse(100)<0)
	{
		return false;
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState !=1)
		{
			return false;
		}
	}

	long send_g_num = num / 50;
	long relic_num = num % 50;
	long ii,jj,kk;

	double SendData[350];
	double retp[8];
	long spos;
	for ( ii = 0; ii < send_g_num; ii++)
	{
		spos = 0;
		for ( jj = 0; jj< 50; jj++)
		{
			m_InsRobot->pln_B.OnCut(retp);
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(ii,50,SendData);
		if( CRobot::OnSetSendWaitResponse(100)<0)
		{
			return false;
		}

	}
	

	if(relic_num != 0)
	{
		spos = 0;
		for ( jj = 0; jj< relic_num; jj++)
		{
			m_InsRobot->pln_B.OnCut(retp);
			for(kk = 0; kk < 7; kk ++)
			{
				SendData[spos] = retp[kk];
				spos ++;
			}
		}
		CRobot::OnClearSet();
		CRobot::OnSetTrajSet_B(send_g_num,relic_num,SendData);
		if( CRobot::OnSetSendWaitResponse(200)<0)
		{
			return false;
		}
	}

	if (CRobot::OnGetBuf(&t) == true)
	{
		if (t.m_Out[1].m_TrajState !=2)
		{
			return false;
		}
	}


	CRobot::OnClearSet();
	CRobot::OnSetTrajRun_B();
	if( CRobot::OnSetSendWaitResponse(50)<0)
	{
		return false;
	}

	return true;
}

bool CRobot::OnSetTrajInit_B(int pointNum)
{
	if (pointNum < 0)
	{
		return false;
	}

	long add_size = 1 + sizeof(FX_INT32);

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 212;
	m_InsRobot->m_Slen++;
	FX_INT32* pv = (FX_INT32*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = pointNum;

	m_InsRobot->m_Slen += sizeof(FX_INT32);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: Initial B arm Planning Trajectory , number of point=%d,\n",pointNum);
	}
	return true;
}

bool CRobot::OnSetTrajRun_B()
{
	long add_size = 1 + sizeof(FX_FLOAT) * 1;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 214;
	m_InsRobot->m_Slen++;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;
	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: B arm run Planning Trajectory \n");
	}
	return true;
}


bool CRobot::OnStopPlnJoint_B()
{
	CRobot::OnClearSet();
	CRobot::OnStopPlnJoint_interB();
	if( CRobot::OnSetSendWaitResponse(50)<0)
	{
		return false;
	} else
	{
		if(m_InsRobot->m_LocalLogTag == true)
		{
			printf("[Marvin SDK]: B arm stop run Planning Trajectory \n");
		}
		return true;
	}
}

bool CRobot::OnStopPlnJoint_interB()
{
	long add_size = 1 + sizeof(FX_FLOAT) * 1;

	if (add_size + m_InsRobot->m_Slen >= 1400)
	{
		return false;
	}
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 215;
	m_InsRobot->m_Slen++;

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;

	return true;
}

bool CRobot::OnSetTrajSet_B(long serial,long pointNum, double* data)
{
	if ( m_InsRobot->m_Slen !=3)
	{
		return false;
	}
	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 213;
	m_InsRobot->m_Slen++;

	FX_INT32 * pserial = (FX_INT32 *)& m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	* pserial = serial;
	m_InsRobot->m_Slen+= sizeof(FX_INT32);

	FX_FLOAT * pdat = (FX_FLOAT * ) & m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];

	long i;
    long j;
    long spos = 0;
    // printf("\n=== Trajectory Points (total %ld points) ===\n", pointNum);
    // printf("     Point    J1      J2      J3      J4      J5      J6      J7\n");
    // printf("--------------------------------------------------------\n");
    
    for(i = 0; i < pointNum; i++)
    {
        // printf("  %6ld: ", i);  // 打印点序号，右对齐
        for(j = 0; j < 7; j++)
        {
            pdat[spos] = data[spos];
            // printf("%7.3f ", data[spos]);  // 每个数据宽度7，保留3位小数
            spos++;
        }
        // printf("\n");
    }
    // printf("========================================================\n");

	m_InsRobot->m_Slen+= sizeof(FX_FLOAT) * 7 * pointNum;


	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;


	if(m_InsRobot->m_LocalLogTag == true)
	{
	    printf("[Marvin SDK]: B arm run Planning Trajectory \n");
	}
	return true;
    
}

///////////////////////////////////

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

	long add_size = 1 + sizeof(FX_CHAR);

	if (add_size + m_InsRobot->m_Slen >= 1450)
	{
		return -1;
	}

	m_InsRobot->m_SendBuf[m_InsRobot->m_Slen] = 251;
	m_InsRobot->m_Slen++;
	char* pv = (char*)&m_InsRobot->m_SendBuf[m_InsRobot->m_Slen];
	pv[0] = m_InsRobot->m_send_response_local_tag;

	m_InsRobot->m_Slen += sizeof(char);

	FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_SendBuf[2];
	(*pnum)++;



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
	m_InsRobot->m_SendTag = 100;

	while (m_InsRobot->m_send_response_timeout_cnt > 0)
	{
		
		#ifdef CMPL_WIN
		    Sleep(1);
		#endif
		#ifdef CMPL_LIN
		    usleep(1000);
		#endif
	}
	if (m_InsRobot->m_respones_time_tag == 1)
	{
		m_InsRobot->m_respones_time_tag = 0;
		if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: OnSetSendWaitResponse\n");
		return m_InsRobot->m_respones_time_cnt;
	}

	return 0;
}

bool CRobot::OnSetSend()
{
	if (m_InsRobot->m_SendTag == 100)
	{
		return false;
	}
	m_InsRobot->m_SendTag = 100;
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: OnSetSend\n");
	return true;
}

bool CRobot::OnUpdateSystem(char * local_path)
{
	if (!m_InsRobot->SendFile(local_path, (char *)"/home/FUSION/Tmp/ctrl_package.tar"))
	{
		return false;
	}
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "UPDATES");
	OnSetIntPara(name, 0);
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: SDK update!\n");
	return true;
}

bool CRobot::OnDownloadLog(char* local_path)
{
	
	if (!m_InsRobot->RecvFile(local_path, (char*)"/home/FUSION/log/LOG.txt"))
	{
		return false;
	}
	if(m_InsRobot->m_LocalLogTag == true) printf("[Marvin SDK]: Send log to host:%s,\n",local_path);
	return true;
}

FX_BOOL CRobot::SendFile(char* local_file, char* remote_file)
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

FX_BOOL CRobot::RecvFile(char* local_file, char* remote_file)
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
