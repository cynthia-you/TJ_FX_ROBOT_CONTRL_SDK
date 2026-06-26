#include "RobotCtrl.h"

static RobotCtrl *m_InsRobot = NULL;

#ifdef CMPL_WIN
FX_VOID CALLBACK CallBackFunc2(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->IsLinked() == FX_FALSE)
	{
		return;
	}
	ins->DoBeat();
	ins->DoSend();
	ins->DoRecv();
	ins->DoCnt();
}
#endif
#ifdef CMPL_LIN
FX_VOID CallBackFunc(union sigval v)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->IsLinked() == FX_FALSE)
	{
		return;
	}
	ins->DoBeat();
	ins->DoSend();
	ins->DoRecv();
	ins->DoCnt();
}
#endif

RobotCtrl::RobotCtrl()
{
	m_LinkTag = FX_FALSE;
	m_RtSendLock = FX_FALSE;
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
FX_INT32 RobotCtrl::RequestControl(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (ins->m_Comm_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3720) == FX_FALSE)
	{
		ins->m_Comm_NA.OnUnlink();
		return -1;
	}
	FX_INT32 ret = ins->m_Comm_NA.OnRequestCtrl();
	if (ret < 0)
	{
		ins->m_Comm_NA.OnUnlink();
		return ret;
	}
	return 0;
}

FX_BOOL RobotCtrl::Link(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (ins->m_LinkTag == FX_TRUE)
	{
		return FX_FALSE;
	}

	if (ins->m_RT_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3721) == FX_FALSE)
	{
		ins->m_RT_NA.OnUnlink();
		return FX_FALSE;
	}
	if (ins->m_SG_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3722) == FX_FALSE)
	{
		ins->m_RT_NA.OnUnlink();
		ins->m_SG_NA.OnUnlink();
		return FX_FALSE;
	}

	if (ins->m_Flange_NA0.OnLinkTo(ip1, ip2, ip3, ip4, 3723) == FX_FALSE)
	{
		ins->m_RT_NA.OnUnlink();
		ins->m_SG_NA.OnUnlink();
		ins->m_Flange_NA0.OnUnlink();
		return FX_FALSE;
	}
	if (ins->m_Flange_NA1.OnLinkTo(ip1, ip2, ip3, ip4, 3724) == FX_FALSE)
	{
		ins->m_RT_NA.OnUnlink();
		ins->m_SG_NA.OnUnlink();
		ins->m_Flange_NA0.OnUnlink();
		ins->m_Flange_NA1.OnUnlink();
		return FX_FALSE;
	}

	ins->m_LinkTag = FX_TRUE;
	ins->m_LinkLifeCount = 100;
	ins->m_LinkState = FX_TRUE;

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
			ins->m_RT_NA.OnUnlink();
			ins->m_SG_NA.OnUnlink();
			ins->m_Flange_NA0.OnUnlink();
			ins->m_Flange_NA1.OnUnlink();
			ins->m_LinkState = FX_FALSE;
			return FX_FALSE;
		}

		ts.it_interval.tv_sec = 0;
		ts.it_interval.tv_nsec = 1000000;
		ts.it_value.tv_sec = 0;
		ts.it_value.tv_nsec = 1000000;
		ret = timer_settime(ins->robot_timer, TIMER_ABSTIME, &ts, NULL);
		if (ret)
		{
			ins->m_RT_NA.OnUnlink();
			ins->m_SG_NA.OnUnlink();
			ins->m_Flange_NA0.OnUnlink();
			ins->m_Flange_NA1.OnUnlink();
			ins->m_LinkState = FX_FALSE;
			return FX_FALSE;
		}
	}
#endif
	return FX_TRUE;
}

FX_VOID RobotCtrl::Unlink()
{
	RobotCtrl *ins = RobotCtrl::GetIns();

#ifdef CMPL_WIN
	timeKillEvent(ins->m_TimeEventID);
#endif

#ifdef CMPL_LIN
	timer_delete(ins->robot_timer);
#endif
	ins->m_Comm_NA.OnUnlink();
	ins->m_RT_NA.OnUnlink();
	ins->m_SG_NA.OnUnlink();
	ins->m_Flange_NA0.OnUnlink();
	ins->m_Flange_NA1.OnUnlink();
	ins->m_LinkTag = FX_FALSE;
	ins->m_LinkLifeCount = 0;
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

	CUtility::UniMilliSleep(2);
	FX_INT32 ret = 2;
	while (cnt < 1000)
	{
		cnt++;
		if (ins->m_RobotRTRecvTag != 0 && ins->m_RobotSGRecvTag != 0)
		{
			return ret;
		}
		CUtility::UniMilliSleep(1);
		ret++;
	}
	return -1;
}

FX_BOOL RobotCtrl::IsLinked()
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	return ins->m_LinkTag;
}

FX_BOOL RobotCtrl::GetLinkState()
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	return ins->m_LinkState;
}

FX_VOID RobotCtrl::OnLocalLogOn()
{
    RobotCtrl *ins = RobotCtrl::GetIns();
	ins->m_LocalLogTag = FX_TRUE;
}

FX_VOID RobotCtrl::OnLocalLogOff()
{
    RobotCtrl *ins = RobotCtrl::GetIns();
	ins->m_LocalLogTag = FX_FALSE;
}

////////////////////////
FX_INT32 RobotCtrl::System_GetControllerVersion()
{
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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return -2;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return -1;
	}
	return ins->m_RobotSG.m_OP_SET.m_OpValueI;
}

FX_INT32 RobotCtrl::System_GetSdkVersion()
{
	return MAKE_VERSION(FX_SDK_MAJOR_VERSION, FX_SDK_MINOR_VERSION, FX_SDK_PATCH_VERSION);
}

FX_BOOL RobotCtrl::System_CheckVersion()
{
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
	buf.m_OpIns = OPINS_SYSTEM_CHECK_VERSION;
	buf.m_OpValueI = MAKE_VERSION(FX_SDK_MAJOR_VERSION, FX_SDK_MINOR_VERSION, FX_SDK_PATCH_VERSION);

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	if (ins->m_RobotSG.m_OP_SET.m_OpValueI == 1)
	{
		return FX_TRUE;
	}
	else
	{
		return FX_FALSE;
	}
}

FX_BOOL RobotCtrl::System_Reboot()
{
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

	return SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf);
}

FX_BOOL RobotCtrl::System_Update()
{
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
	buf.m_OpIns = OPINS_SYSTEM_UPDATE;

	return SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf);
}

FX_BOOL RobotCtrl::System_SetPDCmdCycleTime(FX_INT32 cycle_time)
{
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
	buf.m_OpIns = OPINS_SYSTEM_SET_PD_CMD_CYCLE_TIME;
	buf.m_OpValueI = cycle_time;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	return FX_TRUE;
}

FX_BOOL RobotCtrl::System_GetPDCmdCycleTime(FX_INT32 *cycle_time)
{
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
	buf.m_OpIns = OPINS_SYSTEM_GET_PD_CMD_CYCLE_TIME;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*cycle_time = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

////////////////////////
FX_BOOL RobotCtrl::Para_GetInt(FX_CHAR name[30], FX_INT32 *ret_value)
{
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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*ret_value = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Para_GetFloat(FX_CHAR name[30], FX_FLOAT *ret_value)
{
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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*ret_value = ins->m_RobotSG.m_OP_SET.m_OpValueF;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Para_GetString(FX_CHAR name[30], FX_CHAR ret_value[30])
{
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
	buf.m_OpIns = OPINS_PARAM_GET_STRING;
	memcpy(buf.m_OpValueS, name, 30);

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	strncpy(ret_value, ins->m_RobotSG.m_OP_SET.m_OpValueS, 29);
	ret_value[29] = 0;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Para_SetInt(FX_CHAR name[30], FX_INT32 target_value)
{
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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Para_SetFloat(FX_CHAR name[30], FX_FLOAT target_value)
{
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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Para_Save()
{
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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
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

	FX_INT32 serial = ins->pDDSS1->m_Serial + 1;
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

FX_BOOL RobotCtrl::Arm0_State_GetServoErrorCode(FX_INT32 axis_id, FX_UINT32 *error_code)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || error_code == NULL)
	{
		return FX_FALSE;
	}

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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_State_GetServoErrorCode(FX_INT32 axis_id, FX_UINT32 *error_code)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || error_code == NULL)
	{
		return FX_FALSE;
	}

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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Head_State_GetServoErrorCode(FX_INT32 axis_id, FX_UINT32 *error_code)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 3 || error_code == NULL)
	{
		return FX_FALSE;
	}

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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Body_State_GetServoErrorCode(FX_INT32 axis_id, FX_UINT32 *error_code)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 6 || error_code == NULL)
	{
		return FX_FALSE;
	}

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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Lift_State_GetServoErrorCode(FX_INT32 axis_id, FX_UINT32 *error_code)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 2 || error_code == NULL)
	{
		return FX_FALSE;
	}

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

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*error_code = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm0_State_GetServoVersion(FX_INT32 axis_id, FX_CHAR version[30])
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_SERVO_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	strncpy(version, ins->m_RobotSG.m_OP_SET.m_OpValueS, 29);
	version[29] = 0;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_State_GetServoVersion(FX_INT32 axis_id, FX_CHAR version[30])
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_SERVO_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	strncpy(version, ins->m_RobotSG.m_OP_SET.m_OpValueS, 29);
	version[29] = 0;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Head_State_GetServoVersion(FX_INT32 axis_id, FX_CHAR version[30])
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 3 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_HEAD_SERVO_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	strncpy(version, ins->m_RobotSG.m_OP_SET.m_OpValueS, 29);
	version[29] = 0;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Body_State_GetServoVersion(FX_INT32 axis_id, FX_CHAR version[30])
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 6 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_SERVO_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	strncpy(version, ins->m_RobotSG.m_OP_SET.m_OpValueS, 29);
	version[29] = 0;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm0_State_GetSensorVersion(FX_INT32 axis_id, FX_INT32 *version)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM0_SENSOR_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*version = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_State_GetSensorVersion(FX_INT32 axis_id, FX_INT32 *version)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_ARM1_SENSOR_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*version = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Body_State_GetSensorVersion(FX_INT32 axis_id, FX_INT32 *version)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 6 || version == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = serial;
	buf.m_OpIns = OPINS_BODY_SENSOR_GET_VERSION;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(serial, 500))
	{
		return FX_FALSE;
	}
	*version = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm0_State_GetSensorSerial(FX_INT32 axis_id, FX_INT32 *serial)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || serial == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_ARM0_SENSOR_GET_SERIAL;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*serial = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_State_GetSensorSerial(FX_INT32 axis_id, FX_INT32 *serial)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7 || serial == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_ARM1_SENSOR_GET_SERIAL;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*serial = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Body_State_GetSensorSerial(FX_INT32 axis_id, FX_INT32 *serial)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 6 || serial == NULL)
	{
		return FX_FALSE;
	}

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_BODY_SENSOR_GET_SERIAL;
	buf.m_OpValueI = axis_id;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*serial = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm0_State_GetPhyscialState(FX_INT32 *physical_state)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_ARM0_GET_PHYSIC_STATE;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*physical_state = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm1_State_GetPhyscialState(FX_INT32 *physical_state)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_ARM1_GET_PHYSIC_STATE;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*physical_state = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Head_State_GetPhyscialState(FX_INT32 *physical_state)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_HEAD_GET_PHYSIC_STATE;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*physical_state = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Body_State_GetPhyscialState(FX_INT32 *physical_state)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_BODY_GET_PHYSIC_STATE;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*physical_state = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Lift_State_GetPhyscialState(FX_INT32 *physical_state)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
		ins->m_ParaSerial = 1;
	}
	FX_INT32 cmd_serial = ins->m_ParaSerial;
	OP_SET buf;
	memset(&buf, 0, sizeof(OP_SET));
	buf.m_OpCmdSerial = cmd_serial;
	buf.m_OpIns = OPINS_LIFT_GET_PHYSIC_STATE;

	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	if (!ins->WaitOpReturn(cmd_serial, 500))
	{
		return FX_FALSE;
	}
	*physical_state = ins->m_RobotSG.m_OP_SET.m_OpValueI;
	return FX_TRUE;
}

FX_BOOL RobotCtrl::Arm0_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_ResetEncSingleTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_ARM0_ENC_RESET_SINGLE_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 5000); // servo soft reset will consume more time
}

FX_BOOL RobotCtrl::Arm0_Config_ClearEncError(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_ResetEncMultiTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_ARM0_ENC_RESET_MULTI_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}

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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_ResetEncSingleTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_ARM1_ENC_RESET_SINGLE_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 5000); // servo soft reset will consume more time
}

FX_BOOL RobotCtrl::Arm1_Config_ClearEncError(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_ResetEncMultiTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_ARM1_ENC_RESET_MULTI_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}

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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_ResetEncSingleTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_HEAD_ENC_RESET_SINGLE_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 5000);
}

FX_BOOL RobotCtrl::Head_Config_ClearEncError(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_ResetEncMultiTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_HEAD_ENC_RESET_MULTI_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_ResetEncSingleTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_BODY_ENC_RESET_SINGLE_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 5000);
}

FX_BOOL RobotCtrl::Body_Config_ClearEncError(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_ResetEncMultiTurn(FX_UINT8 axis_mask)
{
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
	buf.m_OpIns = OPINS_BODY_ENC_RESET_MULTI_TURN;
	buf.m_OpValueI = axis_mask;
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (axis_id < 0 || axis_id >= 7)
	{
		return FX_FALSE;
	}

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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Lift_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Lift_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

///////////////////////
FX_BOOL RobotCtrl::Arm0_State_Reset()
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm1_State_Reset()
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Head_State_Reset()
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Body_State_Reset()
{
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Lift_State_Reset()
{   
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
	if(!SetRawData(FX_DEFAULT_THREAD_ID, UDP_OPERATION, 44, (FX_UCHAR *)&buf))
    {
        return FX_FALSE;
    }   
	return ins->WaitOpReturn(serial, 500);
}

FX_BOOL RobotCtrl::Arm0_Runtime_EmergencyStop(FX_UINT32 thread_id)
{
	return RobotCtrl::SetRawData(thread_id, UDP_ARM0_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetState(FX_UINT32 thread_id, FX_INT32 state)
{
	return RobotCtrl::SetState(thread_id, UDP_ARM0_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetTag(FX_UINT32 thread_id, FX_INT32 tag)
{
	return RobotCtrl::SetInt(thread_id, UDP_ARM0_RT_CmdTag, 1, &tag);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointPosCmd(FX_UINT32 thread_id, FX_DOUBLE joint_pos[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_RT_CmdJointPos, 7, joint_pos);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointTorCmd(FX_UINT32 thread_id, FX_DOUBLE joint_tor[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_RT_CmdJointTor, 7, joint_tor);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetForceCtrl(FX_UINT32 thread_id, FX_DOUBLE force_ctrl[5])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_RT_ForceDir, 5, force_ctrl);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetTorqueCtrl(FX_UINT32 thread_id, FX_DOUBLE torque_ctrl[5])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_RT_TorqueDir, 5, torque_ctrl);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetVelRatio(FX_UINT32 thread_id, FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetAccRatio(FX_UINT32 thread_id, FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointK(FX_UINT32 thread_id, FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_JointK, 7, k);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetJointD(FX_UINT32 thread_id, FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_JointD, 7, d);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetCartK(FX_UINT32 thread_id, FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_CartK, 7, k);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetCartD(FX_UINT32 thread_id, FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_CartD, 7, d);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetToolK(FX_UINT32 thread_id, FX_DOUBLE k[6])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_ToolK, 6, k);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetToolD(FX_UINT32 thread_id, FX_DOUBLE d[10])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM0_SG_ToolD, 10, d);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetImpType(FX_UINT32 thread_id, FX_INT32 imp_type)
{
	if (imp_type < 0 || imp_type > 4)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(thread_id, UDP_ARM0_SG_ImpType, 1, &imp_type);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetDragType(FX_UINT32 thread_id, FX_INT16 drag_type)
{
	if (drag_type < 0 || drag_type > 5)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetShortInt(thread_id, UDP_ARM0_RT_DragType, 1, &drag_type);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetCmdPDSerial(FX_UINT32 thread_id, FX_INT16 serial)
{
	return RobotCtrl::SetShortInt(thread_id, UDP_ARM0_RT_CmdPDSerial, 1, &serial);
}

FX_BOOL RobotCtrl::Arm0_Runtime_InitTraj(FX_UINT32 thread_id, FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(thread_id, UDP_ARM0_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Arm0_Runtime_SetTraj(FX_UINT32 thread_id, FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
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
	return RobotCtrl::SetRawData(thread_id, UDP_ARM0_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 7, tmp);
	;
}

FX_BOOL RobotCtrl::Arm0_Runtime_RunTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_ARM0_SP_RunTraj);
}

FX_BOOL RobotCtrl::Arm0_Runtime_StopTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_ARM0_SP_StopTraj);
}

FX_BOOL RobotCtrl::Arm1_Runtime_EmergencyStop(FX_UINT32 thread_id)
{
	return RobotCtrl::SetRawData(thread_id, UDP_ARM1_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetState(FX_UINT32 thread_id, FX_INT32 state)
{
	return RobotCtrl::SetState(thread_id, UDP_ARM1_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetTag(FX_UINT32 thread_id, FX_INT32 tag)
{
	return RobotCtrl::SetInt(thread_id, UDP_ARM1_RT_CmdTag, 1, &tag);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointPosCmd(FX_UINT32 thread_id, FX_DOUBLE joint_pos[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_RT_CmdJointPos, 7, joint_pos);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointTorCmd(FX_UINT32 thread_id, FX_DOUBLE joint_tor[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_RT_CmdJointTor, 7, joint_tor);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetForceCtrl(FX_UINT32 thread_id, FX_DOUBLE force_ctrl[5])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_RT_ForceDir, 5, force_ctrl);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetTorqueCtrl(FX_UINT32 thread_id, FX_DOUBLE torque_ctrl[5])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_RT_TorqueDir, 5, torque_ctrl);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetVelRatio(FX_UINT32 thread_id, FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetAccRatio(FX_UINT32 thread_id, FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointK(FX_UINT32 thread_id, FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_JointK, 7, k);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetJointD(FX_UINT32 thread_id, FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_JointD, 7, d);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetCartK(FX_UINT32 thread_id, FX_DOUBLE k[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_CartK, 7, k);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetCartD(FX_UINT32 thread_id, FX_DOUBLE d[7])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_CartD, 7, d);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetToolK(FX_UINT32 thread_id, FX_DOUBLE k[6])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_ToolK, 6, k);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetToolD(FX_UINT32 thread_id, FX_DOUBLE d[10])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_ARM1_SG_ToolD, 10, d);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetImpType(FX_UINT32 thread_id, FX_INT32 imp_type)
{
	if (imp_type < 0 || imp_type > 4)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(thread_id, UDP_ARM1_SG_ImpType, 1, &imp_type);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetDragType(FX_UINT32 thread_id, FX_INT16 drag_type)
{
	if (drag_type < 0 || drag_type > 5)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetShortInt(thread_id, UDP_ARM1_RT_DragType, 1, &drag_type);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetCmdPDSerial(FX_UINT32 thread_id, FX_INT16 serial)
{
	return RobotCtrl::SetShortInt(thread_id, UDP_ARM1_RT_CmdPDSerial, 1, &serial);
}

FX_BOOL RobotCtrl::Arm1_Runtime_InitTraj(FX_UINT32 thread_id, FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(thread_id, UDP_ARM1_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Arm1_Runtime_SetTraj(FX_UINT32 thread_id, FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
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
	return RobotCtrl::SetRawData(thread_id, UDP_ARM1_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 7, tmp);
	;
}

FX_BOOL RobotCtrl::Arm1_Runtime_RunTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_ARM1_SP_RunTraj);
}

FX_BOOL RobotCtrl::Arm1_Runtime_StopTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_ARM1_SP_StopTraj);
}

FX_BOOL RobotCtrl::Head_Runtime_EmergencyStop(FX_UINT32 thread_id)
{
	return RobotCtrl::SetRawData(thread_id, UDP_HEAD_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Head_Runtime_SetState(FX_UINT32 thread_id, FX_INT32 state)
{
	return RobotCtrl::SetState(thread_id, UDP_HEAD_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Head_Runtime_SetTag(FX_UINT32 thread_id, FX_INT32 tag)
{
	return RobotCtrl::SetInt(thread_id, UDP_HEAD_RT_CmdTag, 1, &tag);
}

FX_BOOL RobotCtrl::Head_Runtime_SetJointPosCmd(FX_UINT32 thread_id, FX_DOUBLE joint_pos[3])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_HEAD_RT_CmdPos, 3, joint_pos);
}

FX_BOOL RobotCtrl::Head_Runtime_SetVelRatio(FX_UINT32 thread_id, FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_HEAD_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Head_Runtime_SetAccRatio(FX_UINT32 thread_id, FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_HEAD_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Body_Runtime_EmergencyStop(FX_UINT32 thread_id)
{
	return RobotCtrl::SetRawData(thread_id, UDP_BODY_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Body_Runtime_SetState(FX_UINT32 thread_id, FX_INT32 state)
{
	return RobotCtrl::SetState(thread_id, UDP_BODY_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Body_Runtime_SetTag(FX_UINT32 thread_id, FX_INT32 tag)
{
	return RobotCtrl::SetInt(thread_id, UDP_BODY_RT_CmdTag, 1, &tag);
}

FX_BOOL RobotCtrl::Body_Runtime_SetJointPosCmd(FX_UINT32 thread_id, FX_DOUBLE joint_pos[6])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_BODY_RT_CmdPos, 6, joint_pos);
}

FX_BOOL RobotCtrl::Body_Runtime_SetVelRatio(FX_UINT32 thread_id, FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_BODY_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Body_Runtime_SetAccRatio(FX_UINT32 thread_id, FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_BODY_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Body_Runtime_SetPDP(FX_UINT32 thread_id, FX_DOUBLE p[6])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_BODY_SG_PDK, 6, p);
}

FX_BOOL RobotCtrl::Body_Runtime_SetPDD(FX_UINT32 thread_id, FX_DOUBLE d[6])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_BODY_SG_PDD, 6, d);
}

FX_BOOL RobotCtrl::Body_Runtime_SetCmdPDSerial(FX_UINT32 thread_id, FX_INT16 serial)
{
	return RobotCtrl::SetShortInt(thread_id, UDP_BODY_RT_CmdPDSerial, 1, &serial);
}

FX_BOOL RobotCtrl::Body_Runtime_InitTraj(FX_UINT32 thread_id, FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(thread_id, UDP_BODY_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Body_Runtime_SetTraj(FX_UINT32 thread_id, FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
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
	return RobotCtrl::SetRawData(thread_id, UDP_BODY_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 6, tmp);
}

FX_BOOL RobotCtrl::Body_Runtime_RunTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_BODY_SP_RunTraj);
}

FX_BOOL RobotCtrl::Body_Runtime_StopTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_BODY_SP_StopTraj);
}

FX_BOOL RobotCtrl::Lift_Runtime_EmergencyStop(FX_UINT32 thread_id)
{
	return RobotCtrl::SetRawData(thread_id, UDP_LIFT_SP_Emcy, 0, NULL);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetState(FX_UINT32 thread_id, FX_INT32 state)
{
	return RobotCtrl::SetState(thread_id, UDP_LIFT_RT_CmdState, state);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetTag(FX_UINT32 thread_id, FX_INT32 tag)
{
	return RobotCtrl::SetInt(thread_id, UDP_LIFT_RT_CmdTag, 1, &tag);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetJointPosCmd(FX_UINT32 thread_id, FX_DOUBLE joint_pos[2])
{
	return RobotCtrl::SetFLoat(thread_id, UDP_LIFT_RT_CmdPos, 2, joint_pos);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetVelRatio(FX_UINT32 thread_id, FX_DOUBLE vel_ratio)
{
	if (vel_ratio < 1)
	{
		vel_ratio = 1;
	}
	else if (vel_ratio > 100.0)
	{
		vel_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_LIFT_SG_VelRatio, 1, &vel_ratio);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetAccRatio(FX_UINT32 thread_id, FX_DOUBLE acc_ratio)
{
	if (acc_ratio < 1)
	{
		acc_ratio = 1;
	}
	else if (acc_ratio > 100.0)
	{
		acc_ratio = 100.0;
	}
	return RobotCtrl::SetFLoat(thread_id, UDP_LIFT_SG_AccRatio, 1, &acc_ratio);
}

FX_BOOL RobotCtrl::Lift_Runtime_InitTraj(FX_UINT32 thread_id, FX_INT32 point_num)
{
	if (point_num < 5 || point_num > 5000)
	{
		return FX_FALSE;
	}
	return RobotCtrl::SetInt(thread_id, UDP_LIFT_SP_InitTraj, 1, &point_num);
}

FX_BOOL RobotCtrl::Lift_Runtime_SetTraj(FX_UINT32 thread_id, FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
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
	return RobotCtrl::SetRawData(thread_id, UDP_LIFT_SP_SetTraj, sizeof(FX_INT32) + sizeof(FX_FLOAT) * point_num * 2, tmp);
}

FX_BOOL RobotCtrl::Lift_Runtime_RunTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_LIFT_SP_RunTraj);
}

FX_BOOL RobotCtrl::Lift_Runtime_StopTraj(FX_UINT32 thread_id)
{
	return SetIns(thread_id, UDP_LIFT_SP_StopTraj);
}

FX_BOOL RobotCtrl::Hand0_Runtime_SetCmdAction(FX_UINT32 thread_id, FX_INT8 action_type)
{
	return SetByte(thread_id, UDP_HAND0_RT_CmdAction, 1, &action_type);
}

FX_BOOL RobotCtrl::Hand0_Runtime_SetCmdPos(FX_UINT32 thread_id, FX_INT8 pos[24])
{
	return SetByte(thread_id, UDP_HAND0_RT_CmdPos, 24, pos);
}

FX_BOOL RobotCtrl::Hand0_Runtime_SetCmdP(FX_UINT32 thread_id, FX_INT8 p[24])
{
	return SetByte(thread_id, UDP_HAND0_SG_CmdP, 24, p);
}

FX_BOOL RobotCtrl::Hand0_Runtime_SetCmdD(FX_UINT32 thread_id, FX_INT8 d[24])
{
	return SetByte(thread_id, UDP_HAND0_SG_CmdD, 24, d);
}

FX_BOOL RobotCtrl::Hand0_Runtime_SetCmdMaxTor(FX_UINT32 thread_id, FX_INT8 max_tor[24])
{
	return SetByte(thread_id, UDP_HAND0_SG_CmdMaxTor, 24, max_tor);
}

FX_BOOL RobotCtrl::Hand1_Runtime_SetCmdAction(FX_UINT32 thread_id, FX_INT8 action_type)
{
	return SetByte(thread_id, UDP_HAND1_RT_CmdAction, 1, &action_type);
}

FX_BOOL RobotCtrl::Hand1_Runtime_SetCmdPos(FX_UINT32 thread_id, FX_INT8 pos[24])
{
	return SetByte(thread_id, UDP_HAND1_RT_CmdPos, 24, pos);
}

FX_BOOL RobotCtrl::Hand1_Runtime_SetCmdP(FX_UINT32 thread_id, FX_INT8 p[24])
{
	return SetByte(thread_id, UDP_HAND1_SG_CmdP, 24, p);
}

FX_BOOL RobotCtrl::Hand1_Runtime_SetCmdD(FX_UINT32 thread_id, FX_INT8 d[24])
{
	return SetByte(thread_id, UDP_HAND1_SG_CmdD, 24, d);
}

FX_BOOL RobotCtrl::Hand1_Runtime_SetCmdMaxTor(FX_UINT32 thread_id, FX_INT8 max_tor[24])
{
	return SetByte(thread_id, UDP_HAND1_SG_CmdMaxTor, 24, max_tor);
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

	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return;
	}
	if (ins->m_LinkLifeCount > 0)
	{
		ins->m_LinkLifeCount--;
	}
	if (ins->m_LinkLifeCount == 0)
	{
		ins->m_LinkState = FX_FALSE;
	}
	else
	{
		ins->m_LinkState = FX_TRUE;
	}
	ins->m_RT_NA.OnRecv();
	while (ins->m_RT_NA.m_buf.m_Rlen > 0)
	{
		if (ins->m_RT_NA.m_buf.m_Rlen == robot_rt_size + 2)
		{
			ins->m_LinkLifeCount = 100;
			ins->m_RobotRTRecvTag = 1;
			memcpy(&ins->m_RobotRT, &ins->m_RT_NA.m_buf.m_Recvbuf[2], robot_rt_size);
			m_send_response_recv_tag = ins->m_RobotRT.wait_serial;
			ins->m_RT_NA.m_buf.m_Rlen = 0;
		}
		ins->m_RT_NA.OnRecv();
	}
	ins->m_SG_NA.OnRecv();
	if (ins->m_SG_NA.m_buf.m_Rlen > 0)
	{
		if (ins->m_SG_NA.m_buf.m_Rlen == robot_sg_size + 2)
		{
			ins->m_RobotSGRecvTag = 1;
			memcpy(&ins->m_RobotSG, &ins->m_SG_NA.m_buf.m_Recvbuf[2], robot_sg_size);
			ins->m_SG_NA.m_buf.m_Rlen = 0;
		}
	}
	ins->m_Flange_NA0.OnRecv();
	while (ins->m_Flange_NA0.m_buf.m_Rlen > 0)
	{
		if (ins->m_Flange_NA0.m_buf.m_Rlen == sizeof(DDSS))
		{
			ins->m_ACB1.WriteBuf((unsigned char *)ins->m_Flange_NA0.m_buf.m_Recvbuf, sizeof(DDSS));
		}
		ins->m_Flange_NA0.m_buf.m_Rlen = 0;
		ins->m_Flange_NA0.OnRecv();
	}

	ins->m_Flange_NA1.OnRecv();
	while (ins->m_Flange_NA1.m_buf.m_Rlen > 0)
	{
		if (ins->m_Flange_NA1.m_buf.m_Rlen == sizeof(DDSS))
		{
			ins->m_ACB2.WriteBuf((unsigned char *)ins->m_Flange_NA1.m_buf.m_Recvbuf, sizeof(DDSS));
		}
		ins->m_Flange_NA1.m_buf.m_Rlen = 0;
		ins->m_Flange_NA1.OnRecv();
	}
}

FX_VOID RobotCtrl::DoBeat()
{
	static FX_INT32 count = 0;

	RobotCtrl *ins = RobotCtrl::GetIns();
	if (count % 200 == 0)
	{
		ins->m_Comm_NA.OnSendBeat();
	}
	count++;
	if (count >= 200)
	{
		count = 0;
	}
}

FX_VOID RobotCtrl::DoSend()
{
	RobotCtrl *ins = RobotCtrl::GetIns();

	if (ins->m_RtSendLock)
	{
		return;
	}
	ins->m_RtSendLock = FX_TRUE;
    
    FX_UCHAR read_buf[2048] = {0};
    FX_INT32 read_buf_len = 0;
    FX_INT32 expect_send_len = 0;

	ins->m_RT_NA.m_buf.m_SendBuf[0] = 'F';
	ins->m_RT_NA.m_buf.m_SendBuf[1] = 'a';
	ins->m_RT_NA.m_buf.m_SendBuf[2] = 0; // LSB of ins data len
	ins->m_RT_NA.m_buf.m_SendBuf[3] = 0; // MSB of ins data len
	ins->m_RT_NA.m_buf.m_SendBuf[4] = 0; // crc for InsNum + ins data
	ins->m_RT_NA.m_buf.m_SendBuf[5] = 0; // InsNum
	ins->m_RT_NA.m_buf.m_Slen = 6;
    
    for(FX_INT32 i = 0; i < 8;)
    {
        // check if next ins can be read out
        read_buf_len = ins->m_RuntimeACB[i].PeekBuf(read_buf, 2048);
        if(read_buf_len <= 0)
        {
            i++;
            continue;
        }
        expect_send_len = ins->m_RT_NA.m_buf.m_Slen + read_buf_len;
        if(expect_send_len > 1450)
        {
            break;
        }
        // read out ins, add it to send buf
        read_buf_len = ins->m_RuntimeACB[i].ReadBuf(read_buf, 2048);
        if(read_buf_len <= 0)
        {
            i++;
            continue;
        }
        memcpy(&ins->m_RT_NA.m_buf.m_SendBuf[ins->m_RT_NA.m_buf.m_Slen], read_buf, read_buf_len);
        ins->m_RT_NA.m_buf.m_Slen += read_buf_len;
        (*((FX_UINT8*)&ins->m_RT_NA.m_buf.m_SendBuf[5]))++;
    }

    if(ins->m_RT_NA.m_buf.m_Slen > 6)
    {
        ins->m_RT_NA.m_buf.m_SendBuf[2] = (ins->m_RT_NA.m_buf.m_Slen - 6) % 256;
        ins->m_RT_NA.m_buf.m_SendBuf[3] = (ins->m_RT_NA.m_buf.m_Slen - 6) / 256;
        FX_UCHAR crc = 0;
        for (FX_INT32 i = 5; i < ins->m_RT_NA.m_buf.m_Slen; i++)
        {
            crc += ins->m_RT_NA.m_buf.m_SendBuf[i];
        }
        ins->m_RT_NA.m_buf.m_SendBuf[4] = crc;        
    	ins->m_RT_NA.OnSend();
    }
	ins->m_RtSendLock = FX_FALSE;
}

FX_BOOL RobotCtrl::WaitOpReturn(FX_INT32 serial, FX_INT32 timeout)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	for (FX_INT32 i = 0; i < timeout; i++)
	{
		CUtility::UniMilliSleep(1);
		FX_INT32 ret_s = ins->m_RobotSG.m_OP_SET.m_OpRetSerial;
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

FX_BOOL RobotCtrl::SetIns(FX_UINT32 thread_id, FX_INT32 cmd)
{    
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[4] = {0};
    FX_UCHAR crc = 0;
    buf[0] = cmd;
    for(FX_INT32 i = 0; i < 3; i++)
    {
        crc += buf[i];
    }
    buf[3] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, 4);
    
#if 0
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
#endif    
}

FX_BOOL RobotCtrl::SetState(FX_UINT32 thread_id, FX_INT32 cmd, FX_INT32 cmd_state)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[6] = {0};
    FX_UCHAR crc = 0;
    buf[0] = cmd;
    buf[1] = sizeof(FX_INT16);
    *((FX_INT16*)&buf[3]) = cmd_state;
    for(FX_INT32 i = 0; i < 5; i++)
    {
        crc += buf[i];
    }
    buf[5] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, 6);

#if 0
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
#endif    
}

FX_BOOL RobotCtrl::SetByte(FX_UINT32 thread_id, FX_INT32 cmd, FX_INT32 num, FX_INT8 *pdata) // INT8
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_INT32 data_len = num * sizeof(FX_INT8);
    FX_INT32 buf_len = data_len + 4;
    if(buf_len > 1450)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[1450] = {0};
    FX_UCHAR crc = 0;
    FX_INT32 crc_len = data_len + 3;
    buf[0] = cmd;
    buf[1] = data_len % 256;
    buf[2] = data_len / 256;
    memcpy(&buf[3], pdata, num * sizeof(FX_INT8));
    for(FX_INT32 i = 0; i < crc_len; i++)
    {
        crc += buf[i];
    }
    buf[crc_len] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, buf_len);


#if 0
	FX_INT32 add_size = 4 + sizeof(FX_INT8) * num;
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1450)
	{
		return FX_FALSE;
	}

	FX_INT32 crc_start_pos = m_InsRobot->m_RT_NA.m_buf.m_Slen;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = ins;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_INT8) * num) % 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;
	m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (sizeof(FX_INT8) * num) / 256;
	m_InsRobot->m_RT_NA.m_buf.m_Slen++;

	FX_INT8 *pv = (FX_INT8 *)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
	for (FX_INT32 i = 0; i < num; i++)
	{
		pv[i] = pdata[i];
	}

	m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(FX_INT8) * num;
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
#endif    
}

FX_BOOL RobotCtrl::SetInt(FX_UINT32 thread_id, FX_INT32 cmd, FX_INT32 num, FX_INT32 *pdata)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_INT32 data_len = num * sizeof(FX_INT32);
    FX_INT32 buf_len = data_len + 4;    
    if(buf_len > 1450)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[1450] = {0};
    FX_UCHAR crc = 0;
    FX_INT32 crc_len = buf_len - 1;
    buf[0] = cmd;
    buf[1] = data_len % 256;
    buf[2] = data_len / 256;
    memcpy(&buf[3], pdata, num * sizeof(FX_INT32));
    for(FX_INT32 i = 0; i < crc_len; i++)
    {
        crc += buf[i];
    }
    buf[crc_len] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, buf_len);

#if 0
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
#endif    
}

FX_BOOL RobotCtrl::SetShortInt(FX_UINT32 thread_id, FX_INT32 cmd, FX_INT32 num, FX_INT16 *pdata)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_INT32 data_len = num * sizeof(FX_INT16);
    FX_INT32 buf_len = data_len + 4;
    if(buf_len > 1450)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[1450] = {0};
    FX_UCHAR crc = 0;
    FX_INT32 crc_len = buf_len - 1;
    buf[0] = cmd;
    buf[1] = data_len % 256;
    buf[2] = data_len / 256;
    memcpy(&buf[3], pdata, num * sizeof(FX_INT16));
    for(FX_INT32 i = 0; i < crc_len; i++)
    {
        crc += buf[i];
    }
    buf[crc_len] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, buf_len);

#if 0
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
#endif    
}

FX_BOOL RobotCtrl::SetFLoat(FX_UINT32 thread_id, FX_INT32 cmd, FX_INT32 num, FX_DOUBLE *pdata)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_INT32 data_len = num * sizeof(FX_FLOAT);
    FX_INT32 buf_len = data_len + 4;
    if(buf_len > 1450)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[1450] = {0};
    FX_UCHAR crc = 0;
    FX_INT32 crc_len = buf_len - 1;
    buf[0] = cmd;
    buf[1] = data_len % 256;
    buf[2] = data_len / 256;
    for(FX_INT32 i = 0; i < num; i++)
    {
        *((FX_FLOAT*)&buf[3 + i * sizeof(FX_FLOAT)]) = pdata[i];
    }
    for(FX_INT32 i = 0; i < crc_len; i++)
    {
        crc += buf[i];
    }
    buf[crc_len] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, buf_len);

#if 0
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
#endif    
}

FX_BOOL RobotCtrl::SetRawData(FX_UINT32 thread_id, FX_INT32 cmd, FX_INT32 num, FX_UCHAR *pdata)
{
	RobotCtrl *ins = RobotCtrl::GetIns();
	if (ins->m_LinkTag == FX_FALSE)
	{
		return FX_FALSE;
	}
    if(thread_id >= 8)
    {
        return FX_FALSE;
    }

    FX_INT32 data_len = num * sizeof(FX_UCHAR);
    FX_INT32 buf_len = data_len + 4;
    if(buf_len > 1450)
    {
        return FX_FALSE;
    }

    FX_UCHAR buf[1450] = {0};
    FX_UCHAR crc = 0;
    FX_INT32 crc_len = buf_len - 1;
    buf[0] = cmd;
    buf[1] = data_len % 256;
    buf[2] = data_len / 256;
    memcpy(&buf[3], pdata, num * sizeof(FX_UCHAR));
    for(FX_INT32 i = 0; i < crc_len; i++)
    {
        crc += buf[i];
    }
    buf[crc_len] = 256 - crc;

    return ins->m_RuntimeACB[thread_id].WriteBuf(buf, buf_len);

#if 0
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
#endif    
}

