#include "FXULT.h"

CFXULT::CFXULT()
{
}

CFXULT::~CFXULT()
{
}

void CFXULT::UniMilliSleep(long millisecond)
{

#ifdef CMPL_WIN
	Sleep(millisecond);
#endif
#ifdef CMPL_LIN
	timeval tm;
	FX_INT32 sec;
	FX_INT32 usec;
	sec = millisecond / 1000;
	usec = (millisecond % 1000) * 1000;
	tm.tv_sec = sec;

	if (usec == 0)
	{
		tm.tv_usec = 2;
	}
	else
	{
		tm.tv_usec = usec;
	}
	select(0, NULL, NULL, NULL, &tm);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////
/////////////////
///////////////////////////////////////////////////////////////////////////////////////

CMarvNetAgent::CMarvNetAgent()
{
	m_nc._local_sock = INVALID_SOCKET;
	m_nc._tosock_ = INVALID_SOCKET;
	m_nc._toLen = 0;
	m_nc._localLen = 0;
	m_nc._from_valid = 0;
	memset(&m_nc._local, 0, sizeof(m_nc._local));
	memset(&m_nc._to, 0, sizeof(m_nc._to));

	m_buf.m_Slen = 0;
	m_buf.m_Rlen = 0;
	m_buf.m_STag = 0;

	m_LinkTag = FX_FALSE;
}

CMarvNetAgent::~CMarvNetAgent()
{
}

FX_BOOL CMarvNetAgent::OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4, FX_INT32L port)
{
	if (m_LinkTag == FX_TRUE)
	{
		return FX_FALSE;
	}

#ifdef CMPL_WIN
	WSADATA wsadata;
	FX_INT32 ret = WSAStartup(0x101, &wsadata);
	if (ret != 0)
	{
		return FX_FALSE;
	}
#endif

	memset(&m_nc._local, 0, sizeof(m_nc._local));
	m_nc._localLen = sizeof(sockaddr_in);
	m_nc._local.sin_family = AF_INET;
	m_nc._local.sin_port = htons(port);
	m_nc._local.sin_addr.s_addr = INADDR_ANY;
	m_nc._local_sock = socket(AF_INET, SOCK_DGRAM, 0);

	FX_UINT32L on = 1;

#ifdef CMPL_WIN
	if (0 != ioctlsocket(m_nc._local_sock, FIONBIO, &on))
	{
		closesocket(m_nc._local_sock);
		m_nc._local_sock = 0;
		return FX_FALSE;
	}
#endif
#ifdef CMPL_LIN
	if (0 != ioctl(m_nc._local_sock, FIONBIO, &on))
	{
		close(m_nc._local_sock);
		m_nc._local_sock = 0;
		return FX_FALSE;
	}
#endif

	if (bind(m_nc._local_sock, (struct sockaddr *)&m_nc._local, sizeof(m_nc._local)) != 0)
	{
#ifdef CMPL_WIN
		closesocket(m_nc._local_sock);
		m_nc._local_sock = 0;
#endif

#ifdef CMPL_LIN
		close(m_nc._local_sock);
		m_nc._local_sock = 0;
#endif
		return FX_FALSE;
	};
	memset(&m_nc._to, 0, sizeof(m_nc._to));

	FX_CHAR ip_str[100] = {0};
	snprintf(ip_str, 99, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
	m_nc._toLen = sizeof(sockaddr_in);
	m_nc._to.sin_family = AF_INET;
	m_nc._to.sin_port = htons(port);
	m_nc._to.sin_addr.s_addr = inet_addr(ip_str);
	m_nc._tosock_ = socket(AF_INET, SOCK_DGRAM, 0);

	m_LinkTag = FX_TRUE;
	return FX_TRUE;
}

FX_BOOL CMarvNetAgent::OnRecv()
{
	if (m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	m_buf.m_Rlen = 0;
	m_nc._localLen = sizeof(m_nc._local);

#ifdef CMPL_WIN
	m_buf.m_Rlen = recvfrom(m_nc._local_sock, m_buf.m_Recvbuf, 2000, 0, (struct sockaddr *)&m_nc._local, &m_nc._localLen);
#endif
#ifdef CMPL_LIN
	m_buf.m_Rlen = recvfrom(m_nc._local_sock, m_buf.m_Recvbuf, 2000, 0, (struct sockaddr *)&m_nc._local, (socklen_t *)&m_nc._localLen);
#endif
	return FX_TRUE;
}

FX_BOOL CMarvNetAgent::OnSendRaw()
{
	if (m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_buf.m_STag != 100)
	{
		return FX_FALSE;
	}
	if (m_buf.m_Slen <= 0)
	{
		m_buf.m_Slen = 0;
		m_buf.m_STag = 0;
		return FX_FALSE;
	}

	FX_INT32L send_len = 0;

	if (m_buf.m_Slen >= 0)
	{
		send_len = sendto(m_nc._tosock_, (char *)m_buf.m_SendBuf, m_buf.m_Slen, 0, (struct sockaddr *)&m_nc._to, sizeof(m_nc._to));
	}

	m_buf.m_STag = 0;
	if (send_len != m_buf.m_Slen)
	{
		m_buf.m_Slen = 0;
		return FX_FALSE;
	}
	m_buf.m_Slen = 0;
	return FX_TRUE;
}

FX_BOOL CMarvNetAgent::OnSend()
{
	if (m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (m_buf.m_STag != 100)
	{
		return FX_FALSE;
	}
	if (m_buf.m_Slen <= 0)
	{
		m_buf.m_Slen = 0;
		m_buf.m_STag = 0;
		return FX_FALSE;
	}
	FX_INT32L send_len = 0;
	if (m_buf.m_Slen >= 0)
	{
		m_buf.m_SendBuf[2] = (m_buf.m_Slen - 6) % 256;
		m_buf.m_SendBuf[3] = (m_buf.m_Slen - 6) / 256;
		FX_UCHAR crc = 0;
		for (FX_INT32 i = 5; i < m_buf.m_Slen; i++)
		{
			crc += m_buf.m_SendBuf[i];
		}
		m_buf.m_SendBuf[4] = crc;
		send_len = sendto(m_nc._tosock_, (char *)m_buf.m_SendBuf, m_buf.m_Slen, 0, (struct sockaddr *)&m_nc._to, sizeof(m_nc._to));
	}

	m_buf.m_STag = 0;
	if (send_len != m_buf.m_Slen)
	{
		m_buf.m_Slen = 0;
		return FX_FALSE;
	}
	m_buf.m_Slen = 0;
	return FX_TRUE;
}

FX_BOOL CMarvNetAgent::OnSendLink()
{
	if (m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_CHAR buf[2];
	buf[0] = 'F';
	buf[1] = 'X';
	FX_INT32L send_len = 0;

	send_len = sendto(m_nc._tosock_, (FX_CHAR *)buf, 2, 0, (struct sockaddr *)&m_nc._to, sizeof(m_nc._to));
	if (send_len != 2)
	{
		return FX_FALSE;
	}
	return FX_TRUE;
}

//////////////////////////////////////////////////////////////////////////////////////
/////////////////
///////////////////////////////////////////////////////////////////////////////////////

CACB::CACB()
{
	init_tag_ = FX_FALSE;
	base_ = NULL;
	item_num_ = 0;
	size_ = 10240;
	base_ = (FX_UCHAR *)malloc(size_);
	init_tag_ = FX_TRUE;
	write_pos_ = 1;
	read_pos_ = 0;
	write_lock_ = 0;
	read_lock_ = 0;
	buf_serial_ = 0;
	item_num_ = 0;
}

CACB::~CACB()
{
	if (init_tag_ == FX_TRUE)
	{
		free(base_);
	}
}
FX_INT32 CACB::OnGetStoreNum()
{
	return item_num_;
}

FX_BOOL CACB::WriteBuf(FX_UCHAR *data_ptr, FX_INT32 size_int)
{
	if (size_int < 1 || init_tag_ == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (write_lock_ != 0)
	{
		return FX_FALSE;
	}
	write_lock_ = 1;

	FX_INT32 emptysize;
	FX_INT32 wpos = write_pos_;
	FX_INT32 rpos = read_pos_;

	FX_UINT32 tmpserial = buf_serial_;
	tmpserial++;
	if (tmpserial >= 100000000)
	{
		tmpserial = 0;
	}

	if (wpos < rpos)
	{
		emptysize = rpos - wpos - 1;
		if (emptysize < size_int + 6)
		{
			write_lock_ = 0;
			return FX_FALSE;
		}
		base_[wpos] = size_int / 256;
		base_[wpos + 1] = size_int % 256;

		base_[wpos + 2] = (FX_UCHAR)(tmpserial / 0x1000000);
		base_[wpos + 3] = (FX_UCHAR)((tmpserial % 0x1000000) / 0x10000);
		base_[wpos + 4] = (FX_UCHAR)((tmpserial % 0x10000) / 0x100);
		base_[wpos + 5] = (FX_UCHAR)((tmpserial % 0x100));

		memcpy(&base_[wpos + 6], data_ptr, size_int);
		wpos += 6;
		wpos += size_int;
		write_pos_ = wpos;

		buf_serial_ = tmpserial;
		write_lock_ = 0;
		item_num_++;
		return FX_TRUE;
	}
	else
	{
		FX_INT32 epos = size_ - wpos;
		emptysize = epos + rpos - 1;

		if (emptysize < size_int + 6)
		{
			write_lock_ = 0;
			return FX_FALSE;
		}

		base_[wpos] = size_int / 256;
		wpos++;
		wpos %= size_;
		base_[wpos] = size_int % 256;
		wpos++;
		wpos %= size_;

		base_[wpos] = (FX_UCHAR)(tmpserial / 0x1000000);
		wpos++;
		wpos %= size_;
		base_[wpos] = (FX_UCHAR)((tmpserial % 0x1000000) / 0x10000);
		wpos++;
		wpos %= size_;
		base_[wpos] = (FX_UCHAR)((tmpserial % 0x10000) / 0x100);
		wpos++;
		wpos %= size_;
		base_[wpos] = (FX_UCHAR)((tmpserial % 0x100));
		wpos++;
		wpos %= size_;

		epos -= 6;
		if (epos <= size_int)
		{
			if (epos > 0)
			{
				memcpy(&base_[wpos], data_ptr, epos);
				if (size_int - epos > 0)
				{
					memcpy(&base_[0], &data_ptr[epos], size_int - epos);
				}
			}
			else
			{
				memcpy(&base_[wpos], data_ptr, size_int);
			}
		}
		else
		{
			memcpy(&base_[wpos], data_ptr, size_int);
		}
		wpos += size_int;
		wpos %= size_;
		write_pos_ = wpos;

		buf_serial_ = tmpserial;
		write_lock_ = 0;
		item_num_++;
		return FX_TRUE;
	}
}

FX_INT32 CACB::ReadBuf(FX_UCHAR *data_ptr, FX_INT32 size_int)
{
	if (init_tag_ == FX_FALSE)
	{
		return -1;
	}
	if (read_lock_ != 0)
	{
		return -1;
	}
	read_lock_ = 1;

	FX_INT32 wpos = write_pos_;
	FX_INT32 rpos = read_pos_;
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		read_lock_ = 0;
		return 0;
	}

	FX_INT32 sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (size_int < sizetmp)
	{
		read_lock_ = 0;
		return -2;
	}

	rpos++;
	rpos %= size_;

	rpos += 4;
	rpos %= size_;

	FX_INT32 explen = size_ - rpos;
	if (explen <= sizetmp)
	{
		memcpy(data_ptr, &base_[rpos], explen);
		if (sizetmp - explen > 0)
		{
			memcpy(&data_ptr[explen], base_, sizetmp - explen);
		}
	}
	else
	{
		memcpy(data_ptr, &base_[rpos], sizetmp);
	}
	rpos += (sizetmp - 1);
	rpos %= size_;
	read_pos_ = rpos;
	read_lock_ = 0;

	item_num_--;
	return sizetmp;
}

FX_BOOL CACB::Empty()
{
	if (init_tag_ == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (read_lock_ != 0 || write_lock_ != 0)
	{
		return FX_FALSE;
	}

	read_lock_ = 1;
	write_lock_ = 1;

	read_pos_ = 0;
	write_pos_ = 1;

	write_lock_ = 0;
	read_lock_ = 0;

	item_num_ = 0;

	return FX_TRUE;
}

//////////////////////////////////////////////////////////////////////////////////////
/////////////////
///////////////////////////////////////////////////////////////////////////////////////
