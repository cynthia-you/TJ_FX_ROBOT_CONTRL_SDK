#include "RobotCtrl.h"

static RobotCtrl *m_InsRobot = NULL;

#ifdef CMPL_WIN
FX_VOID CALLBACK CallBackFunc2(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
	RobotCtrl::GetIns();
	if (m_InsRobot->IsLinked() == FX_FALSE)
	{
		return;
	}
	m_InsRobot->DoSend();
	m_InsRobot->DoRecv();
	m_InsRobot->DoCnt();
}
#endif
#ifdef CMPL_LIN
FX_VOID CallBackFunc(union sigval v)
{
	RobotCtrl::GetIns();
	if (m_InsRobot->IsLinked() == FX_FALSE)
	{
		return;
	}
	m_InsRobot->DoSend();
	m_InsRobot->DoRecv();
	m_InsRobot->DoCnt();
}
#endif

RobotCtrl::RobotCtrl()
{
	m_LinkTag = FX_FALSE;
	m_RtSendLock = FX_FALSE;
	m_RobotRTUpdateTag = 0;
	m_RobotSGUpdateTag = 0;
	m_RobotRTRecvTag = 0;
	m_RobotSGRecvTag = 0;
	m_ParaSerial = 0;

	memset(m_SendBuf1, 0, 600);
	memset(m_SendBuf2, 0, 600);

	pDDSS1 = (DDSS *)m_SendBuf1;
	pDDSS2 = (DDSS *)m_SendBuf2;

	m_send_response_recv_tag = 0;
	m_send_response_timeout_cnt = 0;
	m_last_response_timeout_cnt = 0;
	m_respones_time_tag = 0;
	m_respones_time_cnt = 0;
}

RobotCtrl::~RobotCtrl()
{
}

// /////////////////////
FX_BOOL RobotCtrl::Link(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (ins->m_LinkTag == FX_TRUE)
	{
		return FX_FALSE;
	}
	if (ins->m_RT_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3721) == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (ins->m_SG_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3722) == FX_FALSE)
	{
		return FX_FALSE;
	}

	if (ins->m_Flange_NA0.OnLinkTo(ip1, ip2, ip3, ip4, 3723) == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (ins->m_Flange_NA1.OnLinkTo(ip1, ip2, ip3, ip4, 3724) == FX_FALSE)
	{
		return FX_FALSE;
	}

	ins->m_LinkTag = FX_TRUE;

#ifdef CMPL_WIN
	ins->m_TimeEventID = timeSetEvent(1, 1, CallBackFunc2, (DWORD)NULL, TIME_PERIODIC);
#endif
#ifdef CMPL_LIN
	{
		struct sigevent evp;
		struct itimerspec ts;
		FX_INT32 ret;
		memset(&evp, 0, sizeof(evp));
		evp.sigev_value.sival_ptr = &ins->robot_timer;
		evp.sigev_notify = SIGEV_THREAD;
		evp.sigev_notify_function = CallBackFunc;
		evp.sigev_value.sival_int = 0;
		ret = timer_create(CLOCK_REALTIME, &evp, &ins->robot_timer);
		if (ret)
		{
			return FX_FALSE;
		}

		ts.it_interval.tv_sec = 0;
		ts.it_interval.tv_nsec = 1000000;
		ts.it_value.tv_sec = 0;
		ts.it_value.tv_nsec = 1000000;
		ret = timer_settime(ins->robot_timer, TIMER_ABSTIME, &ts, NULL);
		if (ret)
		{
			return FX_FALSE;
		}
	}
#endif

	return FX_TRUE;
}

FX_INT32 RobotCtrl::TestLink()
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return -1;
	}

	ins->m_RobotRTRecvTag = 0;
	ins->m_RobotSGRecvTag = 0;
	ins->m_RT_NA.OnSendLink();
	ins->m_SG_NA.OnSendLink();

	FX_INT32 cnt = 0;

	CFXULT::UniMilliSleep(2);
	FX_INT32 ret = 2;
	while (cnt < 1000)
	{
		cnt++;
		if (ins->m_RobotRTRecvTag != 0 && ins->m_RobotSGRecvTag != 0)
		{
			return ret;
		}
		CFXULT::UniMilliSleep(1);
		ret++;
	}
	return -1;
}

FX_BOOL RobotCtrl::IsLinked()
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	return ins->m_LinkTag;
}

FX_VOID RobotCtrl::OnLocalLogOn()
{
	m_InsRobot->m_LocalLogTag = FX_TRUE;
}

FX_VOID RobotCtrl::OnLocalLogOff()
{
	m_InsRobot->m_LocalLogTag = FX_FALSE;
}

////////////////////////
FX_BOOL RobotCtrl::ClearSend(FX_UINT32 timeout)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	while (ins->m_RT_NA.m_buf.m_STag == 100)
	{
		if (timeout == 0)
		{
			return FX_FALSE;
		}
		timeout--;
		CFXULT::UniMilliSleep(1);
	}

	ins->m_RT_NA.m_buf.m_STag = 0;
	ins->m_RT_NA.m_buf.m_SendBuf[0] = 'F';
	ins->m_RT_NA.m_buf.m_SendBuf[1] = 'a';
	ins->m_RT_NA.m_buf.m_SendBuf[2] = 0; // LSB of ins data len + 'X'
	ins->m_RT_NA.m_buf.m_SendBuf[3] = 0; // MSB of ins data len + 'X'
	ins->m_RT_NA.m_buf.m_SendBuf[4] = 0; // crc for InsNum + ins data + 'X'
	ins->m_RT_NA.m_buf.m_SendBuf[5] = 0; // InsNum
	ins->m_RT_NA.m_buf.m_Slen = 6;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::SetSend()
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_RT_NA.m_buf.m_STag == 100)
	{
		return FX_FALSE;
	}
	ins->m_RT_NA.m_buf.m_STag = 100;
	return FX_TRUE;
}

FX_INT32 RobotCtrl::WaitSend(FX_UINT32 timeout)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_RT_NA.m_buf.m_STag == 100)
	{
		return -1;
	}
	if (ins->m_send_response_local_tag < 7)
	{
		ins->m_send_response_local_tag = 7;
	}
	if (ins->m_send_response_local_tag > 100)
	{
		ins->m_send_response_local_tag = 7;
	}
	ins->m_send_response_local_tag++;
	FX_UCHAR buf[2] = {0};
	buf[0] = ins->m_send_response_local_tag;
	RobotCtrl::SetRawData(251, 1, buf);

	long tmp_time_out = timeout;
	if (tmp_time_out < 20)
	{
		tmp_time_out = 20;
	}
	if (tmp_time_out > 1000)
	{
		tmp_time_out = 1000;
	}

	ins->m_respones_time_tag = 0;
	ins->m_send_response_timeout_cnt = tmp_time_out;

	RobotCtrl::SetSend();
	while (ins->m_send_response_timeout_cnt > 0)
	{
		CFXULT::UniMilliSleep(1);
	}
	if (ins->m_respones_time_tag == 1)
	{
		ins->m_respones_time_tag = 0;
		return ins->m_respones_time_cnt;
	}
	return 0;
}

////////////////////////
FX_INT32 RobotCtrl::System_GetVersion()
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_SYSTEM_GET_VERSION;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();

	if (!ins->WaitOpReturn(serial, 500))
	{
		return -1;
	}
	return ins->m_RobotSG.m_OP_SET.m_OpValueI;
}

FX_BOOL RobotCtrl::System_Reboot()
{
	if (!ClearSend(100))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_SYSTEM_REBOOT;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return FX_TRUE;
}

////////////////////////
FX_BOOL RobotCtrl::Para_GetInt(FX_CHAR name[30], FX_INT32 *ret_value)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_PARAM_GET_INT32;
	memcpy(buf.m_OpValueS, name, 30);

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();

	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*ret_value = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Para_GetFloat(FX_CHAR name[30], FX_FLOAT *ret_value)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_PARAM_GET_FLOAT;
	memcpy(buf.m_OpValueS, name, 30);

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();

	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*ret_value = ins->m_RobotSG.m_OP_SET.m_OpValueF;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Para_SetInt(FX_CHAR name[30], FX_INT32 target_value)
{
	if (ClearSend(500) == FX_FALSE)
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_PARAM_SET_INT32;
	buf.m_OpValueI = target_value;
	memcpy(buf.m_OpValueS, name, 30);

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Para_SetFloat(FX_CHAR name[30], FX_FLOAT target_value)
{
	if (ClearSend(500) == FX_FALSE)
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_PARAM_SET_FLOAT;
	buf.m_OpValueF = target_value;
	memcpy(buf.m_OpValueS, name, 30);

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Para_Save()
{
	if (ClearSend(500) == FX_FALSE)
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_PARAM_SAVE;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

///////////////////////
FX_BOOL RobotCtrl::Arm0_Terminal_ClearData()
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	DDSS t;
	FX_INT32 si = sizeof(DDSS);
	FX_INT32 num = ins->m_ACB1.ReadBuf((FX_UCHAR *)&t, si);
	while (num > 0)
	{
		num = ins->m_ACB1.ReadBuf((FX_UCHAR *)&t, si);
	}
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_Terminal_ClearData()
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	DDSS t;
	FX_INT32 si = sizeof(DDSS);
	FX_INT32 num = ins->m_ACB2.ReadBuf((FX_UCHAR *)&t, si);
	while (num > 0)
	{
		num = ins->m_ACB2.ReadBuf((FX_UCHAR *)&t, si);
	}
	return FX_TRUE;
}

FX_INT32 RobotCtrl::Arm0_Terminal_GetData(FX_INT32 *channel_type_ptr, FX_UCHAR data_ptr[64])
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	DDSS t;
	FX_INT32 si = sizeof(DDSS);
	FX_INT32 num = ins->m_ACB1.ReadBuf((FX_UCHAR *)&t, si);
	if (num == 0)
	{
		return num;
	}
	memset(data_ptr, 0, 64);
	*channel_type_ptr = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);
	return t.m_Size;
}

FX_INT32 RobotCtrl::Arm1_Terminal_GetData(FX_INT32 *channel_type_ptr, FX_UCHAR data_ptr[64])
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	DDSS t;
	FX_INT32 si = sizeof(DDSS);
	FX_INT32 num = ins->m_ACB2.ReadBuf((FX_UCHAR *)&t, si);
	if (num == 0)
	{
		return num;
	}
	memset(data_ptr, 0, 64);
	*channel_type_ptr = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);
	return t.m_Size;
}

FX_BOOL RobotCtrl::Arm0_Terminal_SetData(FX_INT32 channel_type, FX_UCHAR *data_ptr, FX_INT32 data_len)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (data_len <= 0 || data_len > 256)
	{
		return FX_FALSE;
	}

	FX_INT32 serial = m_InsRobot->pDDSS1->m_Serial + 1;
	if (serial > 1000000)
	{
		serial = 1;
	}
	ins->pDDSS1->m_Serial = serial;
	ins->pDDSS1->m_Size = data_len;
	ins->pDDSS1->m_SUB_CH = channel_type;
	memcpy(ins->pDDSS1->m_Data, data_ptr, data_len);
	memcpy(ins->m_Flange_NA0.m_buf.m_SendBuf, (FX_UCHAR *)ins->pDDSS1, sizeof(DDSS));
	ins->m_Flange_NA0.m_buf.m_Slen = sizeof(DDSS);
	ins->m_Flange_NA0.m_buf.m_STag = 100;
	return ins->m_Flange_NA0.OnSendRaw();
}

FX_BOOL RobotCtrl::Arm1_Terminal_SetData(FX_INT32 channel_type, FX_UCHAR *data_ptr, FX_INT32 data_len)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (data_len <= 0 || data_len > 256)
	{
		return FX_FALSE;
	}

	long serial = ins->pDDSS2->m_Serial + 1;
	if (serial > 1000000)
	{
		serial = 1;
	}
	ins->pDDSS2->m_Serial = serial;
	ins->pDDSS2->m_Size = data_len;
	ins->pDDSS2->m_SUB_CH = channel_type;
	memcpy(ins->pDDSS2->m_Data, data_ptr, data_len);
	memcpy(ins->m_Flange_NA1.m_buf.m_SendBuf, (unsigned char *)m_InsRobot->pDDSS2, sizeof(DDSS));
	ins->m_Flange_NA1.m_buf.m_Slen = sizeof(DDSS);
	ins->m_Flange_NA1.m_buf.m_STag = 100;
	return ins->m_Flange_NA1.OnSendRaw();
}

///////////////////////

FX_BOOL RobotCtrl::Arm0_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_SERVO_GET_ERROR_CODE;
	buf.m_OpValueI = axis_id;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();

	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = m_InsRobot->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_SERVO_GET_ERROR_CODE;
	buf.m_OpValueI = axis_id;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = m_InsRobot->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Head_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
	if (axis_id < 0 || axis_id >= 3)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_SERVO_GET_ERROR_CODE;
	buf.m_OpValueI = axis_id;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = m_InsRobot->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Body_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
	if (axis_id < 0 || axis_id >= 6)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_SERVO_GET_ERROR_CODE;
	buf.m_OpValueI = axis_id;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = m_InsRobot->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Lift_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
	if (axis_id < 0 || axis_id >= 2)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_LIFT_SERVO_GET_ERROR_CODE;
	buf.m_OpValueI = axis_id;

	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = m_InsRobot->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

///////////////////////

FX_BOOL RobotCtrl::Arm0_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_BRAKE_LOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_BRAKE_UNLOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_ENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 5000); // servo soft reset will consume more time
}

FX_BOOL RobotCtrl::Arm0_Config_ClearEncError(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_ENC_CLEAR_ERROR;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_EXTENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_DISABLE_SOFTLIMIT;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_SENSOR0_SET_OFFSET + axis_id;
	buf.m_OpValueI = offset;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_BRAKE_LOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_BRAKE_UNLOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_ENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 5000); // servo soft reset will consume more time
}

FX_BOOL RobotCtrl::Arm1_Config_ClearEncError(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_ENC_CLEAR_ERROR;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_EXTENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_DISABLE_SOFTLIMIT;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_SENSOR0_SET_OFFSET + axis_id;
	buf.m_OpValueI = offset;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_BRAKE_LOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_BRAKE_UNLOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_ENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 5000);
}

FX_BOOL RobotCtrl::Head_Config_ClearEncError(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_ENC_CLEAR_ERROR;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_EXTENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_DISABLE_SOFTLIMIT;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_BRAKE_LOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_BRAKE_UNLOCK;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_ENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 5000);
}

FX_BOOL RobotCtrl::Body_Config_ClearEncError(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_ENC_CLEAR_ERROR;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_EXTENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_DISABLE_SOFTLIMIT;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_SENSOR0_SET_OFFSET + axis_id;
	buf.m_OpValueI = offset;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Lift_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_LIFT_ENC_RESET_OFFSET;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Lift_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_LIFT_DISABLE_SOFTLIMIT;
	buf.m_OpValueI = axis_mask;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

///////////////////////
FX_BOOL RobotCtrl::Arm0_State_Reset()
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_RESET;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_State_Reset()
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_RESET;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_State_Reset()
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_RESET;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_State_Reset()
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_RESET;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Lift_State_Reset()
{
	if (!ClearSend(500))
	{
		return FX_FALSE;
	}
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_LIFT_RESET;
	SetRawData(UDP_OPERATION, 44, (FX_UCHAR *)&buf);
	SetSend();
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Runtime_EmergencyStop()
{
	return RobotCtrl::SetRawData(UDP_ARM0_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetState(FX_INT32 state)
{
	return RobotCtrl::SetState(UDP_ARM0_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_RT_CmdJointPos, 7, joint_pos);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointTorCmd(FX_DOUBLE joint_tor[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_RT_CmdJointTor, 7, joint_tor);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetForceCtrl(FX_DOUBLE force_ctrl[5])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_RT_ForceDir, 5, force_ctrl);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetTorqueCtrl(FX_DOUBLE torque_ctrl[5])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_RT_TorqueDir, 5, torque_ctrl);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointK(FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_JointK, 7, k);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointD(FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_JointD, 7, d);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetCartK(FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_CartK, 7, k);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetCartD(FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_CartD, 7, d);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetToolK(FX_DOUBLE k[6])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_ToolK, 6, k);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetToolD(FX_DOUBLE d[10])
{
	return RobotCtrl::SetFLoat(UDP_ARM0_SG_ToolD, 10, d);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetImpType(FX_INT32 imp_type)
{
	if (imp_type < 0 || imp_type > 3)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(UDP_ARM0_SG_ImpType, 1, &imp_type);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetDragType(FX_INT16 drag_type)
{
	if (drag_type < 0 || drag_type > 5)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetShortInt(UDP_ARM0_RT_DragType, 1, &drag_type);
}

FX_BOOL RobotCtrl::Arm0_Runtime_InitTraj(FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(UDP_ARM0_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
	if (point_num < 1 || point_num > 50)
	{
		return FX_FALSE;
	}
	FX_UCHAR tmp[1450];
	FX_INT32 *pserial = (FX_INT32 *)tmp;
	FX_FLOAT *pdata = (FX_FLOAT *)&tmp[sizeof(FX_INT32)];
	*pserial = serial;
	FX_INT32 spos = 0;
	for (FX_INT32 i = 0; i < point_num; i++)
	{
		for (FX_INT32 j = 0; j < 7; j++)
		{
			pdata[spos] = point_data[spos];
			spos++;
		}
	}
	return RobotCtrl::SetRawData(UDP_ARM0_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 7, tmp);
	;
}

FX_BOOL RobotCtrl::Arm0_Runtime_RunTraj()
{
	return SetIns(UDP_ARM0_SP_RunTraj);
}

FX_BOOL RobotCtrl::Arm0_Runtime_StopTraj()
{
	return SetIns(UDP_ARM0_SP_StopTraj);
}

FX_BOOL RobotCtrl::Arm1_Runtime_EmergencyStop()
{
	return RobotCtrl::SetRawData(UDP_ARM1_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetState(FX_INT32 state)
{
	return RobotCtrl::SetState(UDP_ARM1_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_RT_CmdJointPos, 7, joint_pos);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointTorCmd(FX_DOUBLE joint_tor[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_RT_CmdJointTor, 7, joint_tor);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetForceCtrl(FX_DOUBLE force_ctrl[5])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_RT_ForceDir, 5, force_ctrl);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetTorqueCtrl(FX_DOUBLE torque_ctrl[5])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_RT_TorqueDir, 5, torque_ctrl);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointK(FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_JointK, 7, k);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointD(FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_JointD, 7, d);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetCartK(FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_CartK, 7, k);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetCartD(FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_CartD, 7, d);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetToolK(FX_DOUBLE k[6])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_ToolK, 6, k);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetToolD(FX_DOUBLE d[10])
{
	return RobotCtrl::SetFLoat(UDP_ARM1_SG_ToolD, 10, d);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetImpType(FX_INT32 imp_type)
{
	if (imp_type < 0 || imp_type > 3)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(UDP_ARM1_SG_ImpType, 1, &imp_type);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetDragType(FX_INT16 drag_type)
{
	if (drag_type < 0 || drag_type > 5)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetShortInt(UDP_ARM1_RT_DragType, 1, &drag_type);
}

FX_BOOL RobotCtrl::Arm1_Runtime_InitTraj(FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(UDP_ARM1_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
	if (point_num < 1 || point_num > 50)
	{
		return FX_FALSE;
	}
	FX_UCHAR tmp[1450];
	FX_INT32 *pserial = (FX_INT32 *)tmp;
	FX_FLOAT *pdata = (FX_FLOAT *)&tmp[sizeof(FX_INT32)];
	*pserial = serial;
	FX_INT32 spos = 0;
	for (FX_INT32 i = 0; i < point_num; i++)
	{
		for (FX_INT32 j = 0; j < 7; j++)
		{
			pdata[spos] = point_data[spos];
			spos++;
		}
	}
	return RobotCtrl::SetRawData(UDP_ARM1_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 7, tmp);
	;
}

FX_BOOL RobotCtrl::Arm1_Runtime_RunTraj()
{
	return SetIns(UDP_ARM1_SP_RunTraj);
}

FX_BOOL RobotCtrl::Arm1_Runtime_StopTraj()
{
	return SetIns(UDP_ARM1_SP_StopTraj);
}

FX_BOOL RobotCtrl::Head_Runtime_EmergencyStop()
{
	return RobotCtrl::SetRawData(UDP_HEAD_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Head_Runtime_SetState(FX_INT32 state)
{
	return RobotCtrl::SetState(UDP_HEAD_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Head_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[3])
{
	return RobotCtrl::SetFLoat(UDP_HEAD_RT_CmdPos, 3, joint_pos);
}

FX_BOOL RobotCtrl::Head_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_HEAD_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Head_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_HEAD_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Body_Runtime_EmergencyStop()
{
	return RobotCtrl::SetRawData(UDP_BODY_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Body_Runtime_SetState(FX_INT32 state)
{
	return RobotCtrl::SetState(UDP_BODY_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Body_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[6])
{
	return RobotCtrl::SetFLoat(UDP_BODY_RT_CmdPos, 6, joint_pos);
}

FX_BOOL RobotCtrl::Body_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_BODY_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Body_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_BODY_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Body_Runtime_SetPDP(FX_DOUBLE p[6])
{
	return RobotCtrl::SetFLoat(UDP_BODY_SG_PDK, 6, p);
}

FX_BOOL RobotCtrl::Body_Runtime_SetPDD(FX_DOUBLE d[6])
{
	return RobotCtrl::SetFLoat(UDP_BODY_SG_PDD, 6, d);
}

FX_BOOL RobotCtrl::Body_Runtime_InitTraj(FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(UDP_BODY_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Body_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
	if (point_num < 1 || point_num > 50)
	{
		return FX_FALSE;
	}
	FX_UCHAR tmp[1450];
	FX_INT32 *pserial = (FX_INT32 *)tmp;
	FX_FLOAT *pdata = (FX_FLOAT *)&tmp[sizeof(FX_INT32)];
	*pserial = serial;
	FX_INT32 spos = 0;
	for (FX_INT32 i = 0; i < point_num; i++)
	{
		for (FX_INT32 j = 0; j < 6; j++)
		{
			pdata[spos] = point_data[spos];
			spos++;
		}
	}
	return RobotCtrl::SetRawData(UDP_BODY_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 6, tmp);
}

FX_BOOL RobotCtrl::Body_Runtime_RunTraj()
{
	return SetIns(UDP_BODY_SP_RunTraj);
}

FX_BOOL RobotCtrl::Body_Runtime_StopTraj()
{
	return SetIns(UDP_BODY_SP_StopTraj);
}

FX_BOOL RobotCtrl::Lift_Runtime_EmergencyStop()
{
	return RobotCtrl::SetRawData(UDP_LIFT_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetState(FX_INT32 state)
{
	return RobotCtrl::SetState(UDP_LIFT_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[2])
{
	return RobotCtrl::SetFLoat(UDP_LIFT_RT_CmdPos, 2, joint_pos);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_LIFT_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(UDP_LIFT_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Lift_Runtime_InitTraj(FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(UDP_LIFT_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
	if (point_num < 1 || point_num > 50)
	{
		return FX_FALSE;
	}
	FX_UCHAR tmp[1450];
	FX_INT32 *pserial = (FX_INT32 *)tmp;
	FX_FLOAT *pdata = (FX_FLOAT *)&tmp[sizeof(FX_INT32)];
	*pserial = serial;
	FX_INT32 spos = 0;
	for (FX_INT32 i = 0; i < point_num; i++)
	{
		for (FX_INT32 j = 0; j < 2; j++)
		{
			pdata[spos] = point_data[spos];
			spos++;
		}
	}
	return RobotCtrl::SetRawData(UDP_LIFT_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 2, tmp);
}

FX_BOOL RobotCtrl::Lift_Runtime_RunTraj()
{
	return SetIns(UDP_LIFT_SP_RunTraj);
}

FX_BOOL RobotCtrl::Lift_Runtime_StopTraj()
{
	return SetIns(UDP_LIFT_SP_StopTraj);
}
///////////////////////
RobotCtrl *RobotCtrl::GetIns()
{
	if (m_InsRobot == NULL)
	{
		m_InsRobot = new RobotCtrl();
	}
	return m_InsRobot;
}

FX_VOID RobotCtrl::DoCnt()
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
FX_VOID RobotCtrl::DoRecv()
{
	static FX_INT32 robot_rt_size = sizeof(ROBOT_RT);
	static FX_INT32 robot_sg_size = sizeof(ROBOT_SG);

	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return;
	}

	m_InsRobot->m_RT_NA.OnRecv();
	while (m_InsRobot->m_RT_NA.m_buf.m_Rlen > 0)
	{
		if (m_InsRobot->m_RT_NA.m_buf.m_Rlen == robot_rt_size + 2)
		{
			m_InsRobot->m_RobotRTRecvTag = 1;
			memcpy(&m_InsRobot->m_RobotRT, &m_InsRobot->m_RT_NA.m_buf.m_Recvbuf[2], robot_rt_size);
			m_send_response_recv_tag = m_InsRobot->m_RobotRT.wait_serial;
			m_InsRobot->m_RT_NA.m_buf.m_Rlen = 0;
			m_InsRobot->m_RobotRTUpdateTag++;
			if (m_InsRobot->m_RobotRTUpdateTag > 100)
			{
				m_InsRobot->m_RobotRTUpdateTag = 100;
			}
		}
		m_InsRobot->m_RT_NA.OnRecv();
	}
	m_InsRobot->m_SG_NA.OnRecv();
	if (m_InsRobot->m_SG_NA.m_buf.m_Rlen > 0)
	{

		if (m_InsRobot->m_SG_NA.m_buf.m_Rlen == robot_sg_size + 2)
		{
			m_InsRobot->m_RobotSGRecvTag = 1;
			memcpy(&m_InsRobot->m_RobotSG, &m_InsRobot->m_SG_NA.m_buf.m_Recvbuf[2], robot_sg_size);
			m_InsRobot->m_SG_NA.m_buf.m_Rlen = 0;
			m_InsRobot->m_RobotSGUpdateTag++;
			if (m_InsRobot->m_RobotSGUpdateTag > 100)
			{
				m_InsRobot->m_RobotSGUpdateTag = 100;
			}
		}
	}
	m_InsRobot->m_Flange_NA0.OnRecv();
	while (m_InsRobot->m_Flange_NA0.m_buf.m_Rlen > 0)
	{
		if (m_InsRobot->m_Flange_NA0.m_buf.m_Rlen == sizeof(DDSS))
		{
			m_InsRobot->m_ACB1.WriteBuf((unsigned char *)m_InsRobot->m_Flange_NA0.m_buf.m_Recvbuf, sizeof(DDSS));
		}
		m_InsRobot->m_Flange_NA0.m_buf.m_Rlen = 0;
		m_InsRobot->m_Flange_NA0.OnRecv();
	}

	m_InsRobot->m_Flange_NA1.OnRecv();
	while (m_InsRobot->m_Flange_NA1.m_buf.m_Rlen > 0)
	{
		if (m_InsRobot->m_Flange_NA1.m_buf.m_Rlen == sizeof(DDSS))
		{
			m_InsRobot->m_ACB2.WriteBuf((unsigned char *)m_InsRobot->m_Flange_NA1.m_buf.m_Recvbuf, sizeof(DDSS));
		}
		m_InsRobot->m_Flange_NA1.m_buf.m_Rlen = 0;
		m_InsRobot->m_Flange_NA1.OnRecv();
	}
}

FX_VOID RobotCtrl::DoSend()
{
	GetIns();

	if (m_InsRobot->m_RtSendLock)
	{
		return;
	}
	m_InsRobot->m_RtSendLock = FX_TRUE;
	m_InsRobot->m_RT_NA.OnSend();
	m_InsRobot->m_RtSendLock = FX_FALSE;
}

FX_BOOL RobotCtrl::WaitOpReturn(FX_INT32 serial, FX_INT32 timeout)
{
	GetIns();
	for (FX_INT32 i = 0; i < timeout; i++)
	{
		CFXULT::UniMilliSleep(1);
		FX_INT32 ret_s = m_InsRobot->m_RobotSG.m_OP_SET.m_OpRetSerial;
		if (ret_s % 100 == serial)
		{
			FX_INT32 ret_v = ret_s / 100;
			if (ret_v == 0)
			{
				return FX_TRUE;
			}
			else
			{
				return FX_FALSE;
			}
		}
	}
	return FX_FALSE;
}

FX_BOOL RobotCtrl::SetIns(FX_INT32 ins)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}

	FX_INT32 add_size = 4;
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}

	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 0;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 0;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;

	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
	FX_INT32 crc_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	FX_UCHAR crc = 0;
	for (FX_INT32 j = crc_start_pos; j < crc_pos; j++)
	{
		crc += m_InsRobot->m_RT_NA.m_buf.m_SendBuf[j];
	}
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[crc_pos] = 256 - crc;

	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pnum = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[5];
	(*pnum)++;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::SetState(FX_INT32 ins, FX_INT32 cmd_state)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}

	FX_INT32 add_size = 4 + sizeof(FX_INT16);
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}

	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;

	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = sizeof(FX_INT16);
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 0;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;

	FX_INT16 *pv = (FX_INT16 *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
	*pv = cmd_state;

	m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(FX_INT16);

	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
	FX_INT32 crc_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	FX_UCHAR crc = 0;
	for (FX_INT32 j = crc_start_pos; j < crc_pos; j++)
	{
		crc += m_InsRobot->m_RT_NA.m_buf.m_SendBuf[j];
	}
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[crc_pos] = 256 - crc;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pnum = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[5];
	(*pnum)++;

	return FX_TRUE;
}

FX_BOOL RobotCtrl::SetInt(FX_INT32 ins, FX_INT32 num, FX_INT32 *pdata)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}

	FX_INT32 add_size = 4 + sizeof(FX_INT32) * num;
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}

	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_INT32) * num) % 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_INT32) * num) / 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;

	FX_INT32 *pv = (FX_INT32 *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
	for (FX_INT32 i = 0; i < num; i++)
	{
		pv[i] = pdata[i];
	}

	m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(FX_INT32) * num;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
	FX_INT32 crc_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	FX_UCHAR crc = 0;
	for (FX_INT32 j = crc_start_pos; j < crc_pos; j++)
	{
		crc += m_InsRobot->m_RT_NA.m_buf.m_SendBuf[j];
	}
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[crc_pos] = 256 - crc;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pnum = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[5];
	(*pnum)++;

	return FX_TRUE;
}

FX_BOOL RobotCtrl::SetShortInt(FX_INT32 ins, FX_INT32 num, FX_INT16 *pdata)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}

	FX_INT32 add_size = 4 + sizeof(FX_INT16) * num;
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}

	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_INT16) * num) % 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_INT16) * num) / 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;

	FX_INT16 *pv = (FX_INT16 *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
	for (FX_INT32 i = 0; i < num; i++)
	{
		pv[i] = pdata[i];
	}

	m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(FX_INT16) * num;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
	FX_INT32 crc_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	FX_UCHAR crc = 0;
	for (FX_INT32 j = crc_start_pos; j < crc_pos; j++)
	{
		crc += m_InsRobot->m_RT_NA.m_buf.m_SendBuf[j];
	}
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[crc_pos] = 256 - crc;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pnum = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[5];
	(*pnum)++;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::SetFLoat(FX_INT32 ins, FX_INT32 num, FX_DOUBLE *pdata)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}

	FX_INT32 add_size = 4 + sizeof(FX_FLOAT) * num;
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}
	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_FLOAT) * num) % 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_FLOAT) * num) / 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;

	FX_FLOAT *pv = (FX_FLOAT *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
	for (FX_INT32 i = 0; i < num; i++)
	{
		pv[i] = pdata[i];
	}

	m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(FX_FLOAT) * num;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
	FX_INT32 crc_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	FX_UCHAR crc = 0;
	for (FX_INT32 j = crc_start_pos; j < crc_pos; j++)
	{
		crc += m_InsRobot->m_RT_NA.m_buf.m_SendBuf[j];
	}
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[crc_pos] = 256 - crc;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pnum = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[5];
	(*pnum)++;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::SetRawData(FX_INT32 ins, FX_INT32 num, FX_UCHAR *pdata)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}

	FX_INT32 add_size = 4 + num;
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}

	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = num % 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = num / 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pv = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
	for (FX_INT32 i = 0; i < num; i++)
	{
		pv[i] = pdata[i];
	}

	m_InsRobot->m_RT_NA.m_buf.m_Slen += num;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
	FX_INT32 crc_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	FX_UCHAR crc = 0;
	for (FX_INT32 j = crc_start_pos; j < crc_pos; j++)
	{
		crc += m_InsRobot->m_RT_NA.m_buf.m_SendBuf[j];
	}
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[crc_pos] = 256 - crc;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	FX_UCHAR *pnum = (FX_UCHAR *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[5];
	(*pnum)++;
	return FX_TRUE;
}
