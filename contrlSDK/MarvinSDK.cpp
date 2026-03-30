#include "MarvinSDK.h"

static bool local_log_tag = true;

bool OnUpdateSystem(char *local_path)
{
	return CRobot::OnUpdateSystem(local_path);
}

bool OnDownloadLog(char *local_path)
{
	return CRobot::OnDownloadLog(local_path);
}

void OnEMG_A()
{
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara((char *)"EMCY0", 0);
		SLEEP(2);
	}
	CRobot::OnClearSet();
	CRobot::OnSetTargetState_A(ARM_STATE_IDLE);
	CRobot::OnSetSend();
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: A arm soft stop! \n");
	}
}

void OnEMG_B()
{
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara((char *)"EMCY1", 0);
		SLEEP(2);
	}
	CRobot::OnClearSet();
	CRobot::OnSetTargetState_B(ARM_STATE_IDLE);
	CRobot::OnSetSend();
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: B arm soft stop! \n");
	}
}

void OnEMG_AB()
{
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara((char *)"EMCY0", 0);
		CRobot::OnSetIntPara((char *)"EMCY1", 0);
		SLEEP(2);
	}
	CRobot::OnClearSet();
	CRobot::OnSetTargetState_A(ARM_STATE_IDLE);
	CRobot::OnSetTargetState_B(ARM_STATE_IDLE);
	CRobot::OnSetSend();
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: A and B arm soft stop! \n");
	}
}

void OnServoReset_A(int axis)
{
	if (axis < 0 || axis > 6)
	{
		printf("[ERROR] OnServoReset_A: Invalid axis number %d (valid range: 0~6)\n", axis);
	}
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara((char *)"RESETS0", axis);
		SLEEP(2);
	}
}

void OnServoReset_B(int axis)
{
	if (axis < 0 || axis > 6)
	{
		printf("[ERROR] OnServoReset_B: Invalid axis number %d (valid range: 0~6)\n", axis);
	}
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara((char *)"RESETS1", axis);
		SLEEP(2);
	}
}

void OnGetServoErr_A(long ErrCode[7])
{
	char name[30];
	memset(name, 0, 30);
	for (long i = 0; i < 7; i++)
	{
		sprintf(name, "SERVO0ERR%d", i);
		CRobot::OnGetIntPara(name, &ErrCode[i]);
	}
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: A arm Servo error code=[%d,%d,%d,%d,%d,%d,%d],\n", ErrCode[0], ErrCode[1], ErrCode[2], ErrCode[3], ErrCode[4], ErrCode[5], ErrCode[6]);
	}
}

void OnGetServoErr_B(long ErrCode[7])
{
	char name[30];
	memset(name, 0, 30);

	for (long i = 0; i < 7; i++)
	{
		sprintf(name, "SERVO1ERR%d", i);
		CRobot::OnGetIntPara(name, &ErrCode[i]);
	}
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: B arm Servo error code=[%d,%d,%d,%d,%d,%d,%d],\n", ErrCode[0], ErrCode[1], ErrCode[2], ErrCode[3], ErrCode[4], ErrCode[5], ErrCode[6]);
	}
}

void OnClearErr_A()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "RESET0");
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara(name, 0);
		SLEEP(2);
	}
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: A arm clear error\n");
	}
}

void OnClearErr_B()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "RESET1");
	for (long i = 0; i < 10; i++)
	{
		CRobot::OnSetIntPara(name, 0);
		SLEEP(2);
	}
	if (local_log_tag == true)
		printf("[Marvin SDK]: B arm clear error\n");
}

void OnLogOn()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "LOGON");
	CRobot::OnSetIntPara(name, 0);
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: OnLogOn\n");
	}
}

void OnLogOff()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "LOGOF");
	CRobot::OnSetIntPara(name, 0);
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: OnLogOff\n");
	}
}

void OnLocalLogOn()
{
	local_log_tag = true;
	CRobot::OnLocalLogOn();
}

void OnLocalLogOff()
{
	local_log_tag = false;
	CRobot::OnLocalLogOff();
}

bool OnSendPVT_A(char *local_file, long serial)
{
	if (serial < 0 || serial >= 100)
	{
		printf("[ERROR] OnSendPVT_A: Invalid serial  %d (valid range: 0~99)n", serial);
		return false;
	}
	return CRobot::OnSendPVT_A(local_file, serial);
}

bool OnSendPVT_B(char *local_file, long serial)
{
	if (serial < 0 || serial >= 100)
	{
		printf("[ERROR] OnSendPVT_B: Invalid serial  %d (valid range: 0~99)n", serial);
		return false;
	}
	return CRobot::OnSendPVT_B(local_file, serial);
}

long OnGetSDKVersion()
{
	return CRobot::OnGetSDKVersion();
}

bool OnSendFile(char *local_file, char *remote_file)
{
	return CRobot::OnSendFile(local_file, remote_file);
}

bool OnRecvFile(char *local_file, char *remote_file)
{
	return CRobot::OnRecvFile(local_file, remote_file);
}

long OnSetIntPara(char paraName[30], long setValue)
{
	return CRobot::OnSetIntPara(paraName, setValue);
}

long OnSetFloatPara(char paraName[30], double setValue)
{
	return CRobot::OnSetFloatPara(paraName, setValue);
}

long OnGetIntPara(char paraName[30], long *retValue)
{
	return CRobot::OnGetIntPara(paraName, retValue);
}

long OnGetFloatPara(char paraName[30], double *retValue)
{
	return CRobot::OnGetFloatPara(paraName, retValue);
}

long OnSavePara()
{
	return CRobot::OnSavePara();
}

bool OnGetBuf(DCSS *ret)
{
	return CRobot::OnGetBuf(ret);
}

bool OnStartGather(long targetNum, long targetID[35], long recordNum)
{
	if (targetNum < 0)
	{
		printf("[ERROR] OnStartGather: Invalid targetNum %ld (valid range: 0~35)\n", targetNum);
		return false;
	}
	else if (targetNum > 35)
	{
		printf("[WARNING] OnStartGather: targetNum %ld exceeds maximum, set to 35)\n", targetNum);
		targetNum = 35;
	}
	if (recordNum < 1000)
	{
		printf("[WARNING] OnStartGather: recordNum %ld is below minimum, set to 1000\n", recordNum);
		recordNum = 1000;
	}
	else if (recordNum > 1000000)
	{
		printf("[WARNING] OnStartGather: recordNum %ld exceeds maximum, set to 1000000\n", recordNum);
		recordNum = 1000000;
	}
	return CRobot::OnStartGather(targetNum, targetID, recordNum);
}

bool OnStopGather()
{
	return CRobot::OnStopGather();
}

bool OnSaveGatherData(char *path)
{
	return CRobot::OnSaveGatherData(path);
}

bool OnSaveGatherDataCSV(char *path)
{
	return CRobot::OnSaveGatherDataCSV(path);
}

bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	assert(ip1 >= 0 && ip1 <= 255);
	assert(ip2 >= 0 && ip2 <= 255);
	assert(ip3 >= 0 && ip3 <= 255);
	assert(ip4 >= 0 && ip4 <= 255);
	if (ip1 == 0 && ip2 == 0 && ip3 == 0 && ip4 == 0)
	{
		printf("[ERROR] OnLinkTo: Invalid IP address: 0.0.0.0");
		return false;
	}
	if (ip1 == 255 && ip2 == 255 && ip3 == 255 && ip4 == 255)
	{
		printf("[ERROR] OnLinkTo: Invalid IP address: broadcast address");
		return false;
	}
	if (ip1 == 127)
	{
		printf("[ERROR] OnLinkTo: Loopback address not allowed");
		return false;
	}
	return CRobot::OnLinkTo(ip1, ip2, ip3, ip4);
}

bool OnRelease()
{
	return CRobot::OnRelease();
}

bool OnClearSet()
{
	return CRobot::OnClearSet();
}

bool OnSetTargetState_A(int state)
{
	if (state < 0 || state > 4)
	{
		printf("[ERROR] OnSetTargetState_A: Invalid state %d (valid range: 0~4), set state to 0(disable)\n ", state);
		state = 0;
	}
	return CRobot::OnSetTargetState_A(state);
}

bool OnSetTool_A(double kinePara[6], double dynPara[10])
{
	if (kinePara == nullptr || dynPara == nullptr)
	{
		printf("[ERROR] OnSetTool_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 6; ++i)
	{
		if (isnan(kinePara[i]) || isinf(kinePara[i]))
		{
			printf("[ERROR] OnSetTool_A: kinePara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 10; ++i)
	{
		if (isnan(dynPara[i]) || isinf(dynPara[i]))
		{
			printf("[ERROR] OnSetTool_A: dynPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	return CRobot::OnSetTool_A(kinePara, dynPara);
}

bool OnSetJointLmt_A(int velRatio, int AccRatio)
{
	if (velRatio < 1)
	{
		printf("[WARNING] OnSetJointLmt_A: velRatio %d is below minimum, set to 1\n", velRatio);
		velRatio = 1;
	}
	else if (velRatio > 100)
	{
		printf("[WARNING] OnSetJointLmt_A: velRatio %d exceeds maximum, set to 100\n", velRatio);
		velRatio = 100;
	}
	if (AccRatio < 1)
	{
		printf("[WARNING]OnSetJointLmt_A: AccRatio %d is below minimum, set to 1\n", AccRatio);
		AccRatio = 1;
	}
	else if (AccRatio > 100)
	{
		printf("[WARNING] OnSetJointLmt_A: AccRatio %d exceeds maximum, set to 100\n", AccRatio);
		AccRatio = 100;
	}
	return CRobot::OnSetJointLmt_A(velRatio, AccRatio);
}

bool OnSetJointKD_A(double K[7], double D[7])
{
	if (K == nullptr || D == nullptr)
	{
		printf("[ERROR] OnSetJointKD_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(K[i]) || isinf(K[i]))
		{
			printf("[ERROR] OnSetJointKD_A: K[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (K[i] < 0.0)
		{
			printf("[WARNING] OnSetJointKD_A: K[%d] is negative (%f), set to 0\n", i, K[i]);
			K[i] = 0.0;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(D[i]) || isinf(D[i]))
		{
			printf("[ERROR] OnSetJointKD_A: D[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (D[i] < 0.0)
		{
			printf("[WARNING] OnSetJointKD_A: D[%d] is negative (%f), set to 0\n", i, D[i]);
			D[i] = 0.0;
		}
		else if (D[i] > 1.0)
		{
			printf("[WARNING] OnSetJointKD_A: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
			D[i] = 1.0;
		}
	}
	return CRobot::OnSetJointKD_A(K, D);
}

bool OnSetCartKD_A(double K[7], double D[7], int type)
{
	if (K == nullptr || D == nullptr)
	{
		printf("[ERROR] OnSetCartKD_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(K[i]) || isinf(K[i]))
		{
			printf("[ERROR] OnSetCartKD_A: K[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (K[i] < 0.0)
		{
			printf("[WARNING] OnSetCartKD_A: K[%d] is negative (%f), set to 0\n", i, K[i]);
			K[i] = 0.0;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(D[i]) || isinf(D[i]))
		{
			printf("[ERROR] OnSetCartKD_A: D[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (D[i] < 0.0)
		{
			printf("[WARNING] OnSetCartKD_A: D[%d] is negative (%f), set to 0\n", i, D[i]);
			D[i] = 0.0;
		}
		else if (D[i] > 1.0)
		{
			printf("[WARNING] OnSetCartKD_A: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
			D[i] = 1.0;
		}
	}
	if (type != 2)
	{
		type = 2;
	}
	return CRobot::OnSetCartKD_A(K, D, type);
}

bool OnSetEefRot_A(int fcType, double CartCtrlPara[7])
{
	if (CartCtrlPara == nullptr)
	{
		printf("[ERROR] OnSetEefRot_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(CartCtrlPara[i]) || isinf(CartCtrlPara[i]))
		{
			printf("[ERROR] OnSetEefRot_A: CartCtrlPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (fcType != 1 && fcType != 2)
	{
		printf("[ERROR] OnSetEefRot_A: Invalid fcType number %d (valid range: 1~2)\n", fcType);
		return false;
	}
	return CRobot::OnSetEefRot_A(fcType, CartCtrlPara);
}

bool OnSetDragSpace_A(int dgType)
{
	if (dgType < 0 || dgType > 5)
	{
		printf("[ERROR] OnSetDragSpace_A: Invalid dgType  %d (valid range: 0~5), exit drag mode\n", dgType);
		dgType = 0;
	}
	return CRobot::OnSetDragSpace_A(dgType);
}

bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	if (fxDir == nullptr || fcCtrlPara == nullptr)
	{
		printf("[ERROR] OnSetForceCtrPara_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 6; ++i)
	{
		if (isnan(fxDir[i]) || isinf(fxDir[i]))
		{
			printf("[ERROR] OnSetForceCtrPara_A: fxDir[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(fcCtrlPara[i]) || isinf(fcCtrlPara[i]))
		{
			printf("[ERROR] OnSetForceCtrPara_A: fcCtrlPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (fcType != 0)
	{
		printf("[WARNING] OnSetForceCtrPara_A: fcType set to 0\n", fcType);
		fcType = 0;
	}
	if (isnan(fcAdjLmt) || isinf(fcAdjLmt))
	{
		printf("[ERROR] OnSetForceCtrPara_A: fcAdjLmt %d is invalid (NaN or Inf)\n");
		return false;
	}
	return CRobot::OnSetForceCtrPara_A(fcType, fxDir, fcCtrlPara, fcAdjLmt);
}

bool OnSetJointCmdPos_A(double joint[7])
{
	if (joint == nullptr)
	{
		printf("[ERROR] OnSetJointCmdPos_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(joint[i]) || isinf(joint[i]))
		{
			printf("[ERROR] OnSetJointCmdPos_A: joint[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	return CRobot::OnSetJointCmdPos_A(joint);
}

bool OnSetForceCmd_A(double force)
{
	if (isnan(force) || isinf(force))
	{
		printf("[ERROR] OnSetForceCmd_A: force %d is invalid (NaN or Inf)\n");
		return false;
	}
	return CRobot::OnSetForceCmd_A(force);
}

bool OnSetPVT_A(int id)
{
	if (id < 0 || id >= 100)
	{
		printf("[ERROR] OnSetPVT_A: Invalid id  %d (valid range: 0~99)n", id);
		return false;
	}
	return CRobot::OnSetPVT_A(id);
}

bool OnSetImpType_A(int type)
{
	if (type != 1 && type != 2 && type != 3)
	{
		printf("[ERROR] OnSetImpType_A: Invalid type  %d (valid range: 1~3)\n", type);
		return false;
	}
	return CRobot::OnSetImpType_A(type);
}

bool OnSetTargetState_B(int state)
{
	if (state < 0 || state > 4)
	{
		printf("[ERROR] OnSetTargetState_B: Invalid state %d (valid range: 0~4), set state to 0(disable)\n ", state);
		state = 0;
	}
	return CRobot::OnSetTargetState_B(state);
}

bool OnSetTool_B(double kinePara[6], double dynPara[10])
{
	if (kinePara == nullptr || dynPara == nullptr)
	{
		printf("[ERROR] OnSetTool_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 6; ++i)
	{
		if (isnan(kinePara[i]) || isinf(kinePara[i]))
		{
			printf("[ERROR] OnSetTool_B: kinePara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 10; ++i)
	{
		if (isnan(dynPara[i]) || isinf(dynPara[i]))
		{
			printf("[ERROR] OnSetTool_B: dynPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	return CRobot::OnSetTool_B(kinePara, dynPara);
}

bool OnSetJointLmt_B(int velRatio, int AccRatio)
{
	if (velRatio < 1)
	{
		printf("[WARNING] OnSetJointLmt_B: velRatio %d is below minimum, set to 1\n", velRatio);
		velRatio = 1;
	}
	else if (velRatio > 100)
	{
		printf("[WARNING] OnSetJointLmt_B: velRatio %d exceeds maximum, set to 100\n", velRatio);
		velRatio = 100;
	}
	if (AccRatio < 1)
	{
		printf("[WARNING] OnSetJointLmt_B: AccRatio %d is below minimum, set to 1\n", AccRatio);
		AccRatio = 1;
	}
	else if (AccRatio > 100)
	{
		printf("[WARNING] OnSetJointLmt_B: AccRatio %d exceeds maximum, set to 100\n", AccRatio);
		AccRatio = 100;
	}
	return CRobot::OnSetJointLmt_B(velRatio, AccRatio);
}

bool OnSetJointKD_B(double K[7], double D[7])
{
	if (K == nullptr || D == nullptr)
	{
		printf("[ERROR] OnSetJointKD_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(K[i]) || isinf(K[i]))
		{
			printf("[ERROR] OnSetJointKD_B: K[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (K[i] < 0.0)
		{
			printf("[WARNING] OnSetJointKD_B: K[%d] is negative (%f), set to 0\n", i, K[i]);
			K[i] = 0.0;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(D[i]) || isinf(D[i]))
		{
			printf("[ERROR] OnSetJointKD_B: D[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (D[i] < 0.0)
		{
			printf("[WARNING] OnSetJointKD_B: D[%d] is negative (%f), set to 0\n", i, D[i]);
			D[i] = 0.0;
		}
		else if (D[i] > 1.0)
		{
			printf("[WARNING] OnSetJointKD_B: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
			D[i] = 1.0;
		}
	}
	return CRobot::OnSetJointKD_B(K, D);
}

bool OnSetCartKD_B(double K[6], double D[6], int type)
{
	if (K == nullptr || D == nullptr)
	{
		printf("[ERROR] OnSetCartKD_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(K[i]) || isinf(K[i]))
		{
			printf("[ERROR] OnSetCartKD_B: K[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (K[i] < 0.0)
		{
			printf("[WARNING] OnSetCartKD_B: K[%d] is negative (%f), set to 0\n", i, K[i]);
			K[i] = 0.0;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(D[i]) || isinf(D[i]))
		{
			printf("[ERROR] OnSetCartKD_B: D[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
		if (D[i] < 0.0)
		{
			printf("[WARNING] OnSetCartKD_B: D[%d] is negative (%f), set to 0\n", i, D[i]);
			D[i] = 0.0;
		}
		else if (D[i] > 1.0)
		{
			printf("[WARNING] OnSetCartKD_B: D[%d] exceeds 1 (%f), set to 1\n", i, D[i]);
			D[i] = 1.0;
		}
	}
	if (type != 2)
	{
		type = 2;
	}
	return CRobot::OnSetCartKD_B(K, D, type);
}

bool OnSetEefRot_B(int fcType, double CartCtrlPara[7])
{
	if (CartCtrlPara == nullptr)
	{
		printf("[ERROR] OnSetEefRot_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(CartCtrlPara[i]) || isinf(CartCtrlPara[i]))
		{
			printf("[ERROR] OnSetEefRot_B: CartCtrlPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (fcType != 1 && fcType != 2)
	{
		printf("[ERROR] OnSetEefRot_B: Invalid fcType number %d (valid range: 1~2)\n", fcType);
		return false;
	}
	return CRobot::OnSetEefRot_B(fcType, CartCtrlPara);
}

bool OnSetDragSpace_B(int dgType)
{
	if (dgType < 0 || dgType > 5)
	{
		printf("[ERROR] OnSetDragSpace_B: Invalid dgType  %d (valid range: 0~5), exit drag mode\n", dgType);
		dgType = 0;
	}
	return CRobot::OnSetDragSpace_B(dgType);
}

bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	if (fxDir == nullptr || fcCtrlPara == nullptr)
	{
		printf("[ERROR] OnSetForceCtrPara_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 6; ++i)
	{
		if (isnan(fxDir[i]) || isinf(fxDir[i]))
		{
			printf("[ERROR] OnSetForceCtrPara_B: fxDir[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(fcCtrlPara[i]) || isinf(fcCtrlPara[i]))
		{
			printf("[ERROR] OnSetForceCtrPara_B: fcCtrlPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (fcType != 0)
	{
		printf("[WARNING] OnSetForceCtrPara_B: fcType set to 0\n", fcType);
		fcType = 0;
	}
	if (isnan(fcAdjLmt) || isinf(fcAdjLmt))
	{
		printf("[ERROR] OnSetForceCtrPara_B: fcAdjLmt %d is invalid (NaN or Inf)\n");
		return false;
	}
	return CRobot::OnSetForceCtrPara_B(fcType, fxDir, fcCtrlPara, fcAdjLmt);
}

bool OnSetJointCmdPos_B(double joint[7])
{
	if (joint == nullptr)
	{
		printf("[ERROR] OnSetJointCmdPos_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(joint[i]) || isinf(joint[i]))
		{
			printf("[ERROR] OnSetJointCmdPos_B: joint[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	return CRobot::OnSetJointCmdPos_B(joint);
}

bool OnSetForceCmd_B(double force)
{
	if (isnan(force) || isinf(force))
	{
		printf("[ERROR] OnSetForceCmd_B: force %d is invalid (NaN or Inf)\n");
		return false;
	}
	return CRobot::OnSetForceCmd_B(force);
}

bool OnSetImpType_B(int type)
{
	if (type != 1 && type != 2 && type != 3)
	{
		printf("[ERROR] OnSetImpType_B: Invalid type  %d (valid range: 1~3)\n", type);
		return false;
	}
	return CRobot::OnSetImpType_B(type);
}

bool OnSetPVT_B(int id)
{
	if (id < 0 || id >= 100)
	{
		printf("[ERROR] OnSetPVT_B: Invalid id  %d (valid range: 0~99)n", id);
		return false;
	}
	return CRobot::OnSetPVT_B(id);
}

bool OnInitPlnLmt(char *path)
{
	return CRobot::OnInitPlnLmt(path);
}

bool OnSetPlnJoint_A(double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio)
{
	if (start_joints == nullptr || stop_joints == nullptr)
	{
		printf("[ERROR] OnSetPlnJoint_A: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(start_joints[i]) || isinf(start_joints[i]))
		{
			printf("[ERROR] OnSetPlnJoint_A: start_joints[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(stop_joints[i]) || isinf(stop_joints[i]))
		{
			printf("[ERROR] OnSetPlnJoint_A: stop_joints[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (vel_ratio < 0)
	{
		printf("[ERROR] OnSetPlnJoint_A: Invalid vel_ratio %d (valid range: 0~1)\n", vel_ratio);
		return false;
	}
	if (vel_ratio > 1)
	{
		printf("[WARNING] OnSetPlnJoint_A: Invalid vel_ratio %d (valid range: 0~1), set vel_ratio to 1\n", vel_ratio);
		vel_ratio = 1.0;
	}
	if (acc_ratio < 0)
	{
		printf("[ERROR] OnSetPlnJoint_A: Invalid acc_ratio %d (valid range: 0~1)\n", acc_ratio);
		return false;
	}
	if (acc_ratio > 1)
	{
		printf("[WARNING] OnSetPlnJoint_A: Invalid acc_ratio %d (valid range: 0~1), set acc_ratio to 1\n", acc_ratio);
		acc_ratio = 1.0;
	}
	return CRobot::OnSetPlnJoint_A(start_joints, stop_joints, vel_ratio, acc_ratio);
}

bool OnSetPlnJoint_B(double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio)
{
	if (start_joints == nullptr || stop_joints == nullptr)
	{
		printf("[ERROR] OnSetPlnJoint_B: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(start_joints[i]) || isinf(start_joints[i]))
		{
			printf("[ERROR] OnSetPlnJoint_B: start_joints[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(stop_joints[i]) || isinf(stop_joints[i]))
		{
			printf("[ERROR] OnSetPlnJoint_B: stop_joints[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (vel_ratio < 0)
	{
		printf("[ERROR] OnSetPlnJoint_B: Invalid vel_ratio %d (valid range: 0~1)\n", vel_ratio);
		return false;
	}
	if (vel_ratio > 1)
	{
		printf("[WARNING] OnSetPlnJoint_B: Invalid vel_ratio %d (valid range: 0~1), set vel_ratio to 1\n", vel_ratio);
		vel_ratio = 1.0;
	}
	if (acc_ratio < 0)
	{
		printf("[ERROR] OnSetPlnJoint_B: Invalid acc_ratio %d (valid range: 0~1)\n", acc_ratio);
		return false;
	}
	if (acc_ratio > 1)
	{
		printf("[WARNING] OnSetPlnJoint_B: Invalid acc_ratio %d (valid range: 0~1), set acc_ratio to 1\n", acc_ratio);
		acc_ratio = 1.0;
	}
	return CRobot::OnSetPlnJoint_B(start_joints, stop_joints, vel_ratio, acc_ratio);
}

bool OnStopPlnJoint_B()
{
	return CRobot::OnStopPlnJoint_B();
}

bool OnStopPlnJoint_A()
{
	return CRobot::OnStopPlnJoint_A();
}

bool OnSetPlnCart_A(void *pset)
{
	return CRobot::OnSetPlnCart_A(static_cast<CPointSet *>(pset));
}

bool OnSetPlnCart_B(void *pset)
{
	return CRobot::OnSetPlnCart_B(static_cast<CPointSet *>(pset));
}

bool OnSetSend()
{
	return CRobot::OnSetSend();
}

long OnSetSendWaitResponse(long time_out)
{
	if (time_out < 0 || time_out > 200)
	{
		printf("[WARNING] OnSetSendWaitResponse: Invalid time_out  %ld (valid range: 1-200), set time_out as 50ms\n", time_out);
		time_out = 50;
	}
	return CRobot::OnSetSendWaitResponse(time_out);
}

long OnGetChDataA(unsigned char data_ptr[256], long *ret_ch)
{
	if (*ret_ch != 1 && *ret_ch != 2 && *ret_ch != 3)
	{
		printf("[ERROR] OnGetChDataA: Invalid ret_ch number %d (valid range: 1~3)\n", ret_ch);
		return false;
	}
	return CRobot::OnGetChDataA(data_ptr, ret_ch);
}

bool OnSetChDataA(unsigned char data_ptr[256], long size_int, long set_ch)
{
	if (size_int <= 0 || size_int > 256)
	{
		printf("[ERROR] OnSetChDataA: Channel must in the range of  0~255\n");
		return false;
	}
	if (set_ch != 1 && set_ch != 2 && set_ch != 3)
	{
		printf("[ERROR] OnSetChDataA: Invalid set_ch number %d (valid range: 1~3)\n", set_ch);
		return false;
	}
	return CRobot::OnSetChDataA(data_ptr, size_int, set_ch);
}

long OnGetChDataB(unsigned char data_ptr[256], long *ret_ch)
{
	if (*ret_ch != 1 && *ret_ch != 2 && *ret_ch != 3)
	{
		printf("[ERROR] OnGetChDataB: Invalid ret_ch number %d (valid range: 1~3)\n", ret_ch);
		return false;
	}
	return CRobot::OnGetChDataB(data_ptr, ret_ch);
}

bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch)
{
	if (size_int <= 0 || size_int > 256)
	{
		printf("[ERROR] OnSetChDataB: Channel must in the range of  0~255\n");
		return false;
	}
	if (set_ch != 1 && set_ch != 2 && set_ch != 3)
	{
		printf("[ERROR] OnSetChDataB: Invalid set_ch number %d (valid range: 1~3)\n", set_ch);
		return false;
	}
	return CRobot::OnSetChDataB(data_ptr, size_int, set_ch);
}

bool OnClearChDataA()
{
	return CRobot::OnClearChDataA();
}

bool OnClearChDataB()
{
	return CRobot::OnClearChDataB();
}

void *FX_CPointSet_Create()
{
	return new CPointSet();
}

void FX_CPointSet_Destroy(void *pset)
{
	if (pset)
	{
		delete static_cast<CPointSet *>(pset);
	}
}

//////////////////////////////////简明式接口Concise SDK API////////////////////////////////////////////////////////
bool CheckArmError()
{
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int arm_error_a = dcss.m_State[0].m_ERRCode;
	int arm_error_b = dcss.m_State[1].m_ERRCode;
	int arm_state_a = dcss.m_State[0].m_CurState;
	int arm_state_b = dcss.m_State[1].m_CurState;
	if (arm_error_a == 0 && arm_error_b == 0 && arm_state_a != 100 && arm_state_b != 100)
	{
		return true;
	}
	if (arm_error_a != 0 || arm_state_a == 100)
	{
		OnClearSet();
		OnClearErr_A();
		OnSetSend();
		SLEEP(SLEEP_TIME);
	}
	if (arm_error_b != 0 || arm_state_b == 100)
	{
		OnClearSet();
		OnClearErr_B();
		OnSetSend();
		SLEEP(SLEEP_TIME);
	}
	CRobot::OnGetBuf(&dcss);
	arm_error_a = dcss.m_State[0].m_ERRCode;
	arm_error_b = dcss.m_State[1].m_ERRCode;
	arm_state_a = dcss.m_State[0].m_CurState;
	arm_state_b = dcss.m_State[1].m_CurState;
	if (arm_error_a != 0 || arm_state_a == 100)
	{
		printf("[ERROR] CheckArmError: Arm A error still present, arm_error_code = %d, arm_state = %d\n", arm_error_a, arm_state_a);
		return false;
	}
	if (arm_error_a != 0 || arm_state_a == 100 || arm_error_b != 0 || arm_state_b == 100)
	{
		printf("[ERROR] CheckArmError: Arm B error still present, arm_error_code = %d, arm_state = %d\n", arm_error_b, arm_state_b);
		return false;
	}
}

bool CheckServoError()
{
	long errCode_a[7] = {0};
	long errCode_b[7] = {0};
	char name_a[30];
	char name_b[30];
	for (long i = 0; i < 7; i++)
	{
		sprintf(name_a, "SERVO0ERR%d", i);
		sprintf(name_b, "SERVO1ERR%d", i);
		CRobot::OnGetIntPara(name_a, &errCode_a[i]);
		CRobot::OnGetIntPara(name_b, &errCode_b[i]);
	}
	bool a_ok = true;
	bool b_ok = true;
	for (int i = 0; i < 7; ++i)
	{
		if (errCode_a[i] != 0)
			a_ok = false;
		if (errCode_b[i] != 0)
			b_ok = false;
	}
	if (a_ok && b_ok)
	{
		return true;
	}
	if (!a_ok)
	{
		OnClearSet();
		OnClearErr_A();
		OnSetSend();
		SLEEP(SLEEP_TIME);
	}
	else if (!b_ok)
	{
		OnClearSet();
		OnClearErr_B();
		OnSetSend();
		SLEEP(SLEEP_TIME);
	}
	for (long i = 0; i < 7; i++)
	{
		sprintf(name_a, "SERVO0ERR%d", i);
		sprintf(name_b, "SERVO1ERR%d", i);
		CRobot::OnGetIntPara(name_a, &errCode_a[i]);
		CRobot::OnGetIntPara(name_b, &errCode_b[i]);
	}
	for (int i = 0; i < 7; ++i)
	{
		if (errCode_a[i] != 0)
			a_ok = false;
		if (errCode_b[i] != 0)
			b_ok = false;
	}
	if (a_ok && b_ok)
	{
		return true;
	}
	if (!a_ok)
	{
		printf("[ERROR] CheckServoError: Arm A servo error still present, Servo error code=[%d,%d,%d,%d,%d,%d,%d],\n", errCode_a[0], errCode_a[1], errCode_a[2], errCode_a[3], errCode_a[4], errCode_a[5], errCode_a[6]);
		return false;
	}
	else if (!b_ok)
	{
		printf("[ERROR] CheckServoError: Arm B servo error still present, Servo error code=[%d,%d,%d,%d,%d,%d,%d],\n", errCode_b[0], errCode_b[1], errCode_b[2], errCode_b[3], errCode_b[4], errCode_b[5], errCode_b[6]);
		return false;
	}
}

bool ValidateArm(char arm)
{
	if (arm != 'A' && arm != 'B')
	{
		return false;
	}
	return true;
}

bool Connect(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4, int log_swtich)
{
	assert(ip1 >= 0 && ip1 <= 255);
	assert(ip2 >= 0 && ip2 <= 255);
	assert(ip3 >= 0 && ip3 <= 255);
	assert(ip4 >= 0 && ip4 <= 255);
	if (ip1 == 0 && ip2 == 0 && ip3 == 0 && ip4 == 0)
	{
		printf("[ERROR] Connect: Invalid IP address: 0.0.0.0");
		return false;
	}
	if (ip1 == 255 && ip2 == 255 && ip3 == 255 && ip4 == 255)
	{
		printf("[ERROR] Connect: Invalid IP address: broadcast address");
		return false;
	}
	if (ip1 == 127)
	{
		printf("[ERROR] Connect: Loopback address not allowed");
		return false;
	}
	if (!CRobot::OnLinkTo(ip1, ip2, ip3, ip4))
	{
		printf("[ERROR] Connect: failed to connect to the robot, port is occupied\n");
		return false;
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	int motion_tag = 0;
	int frame_update = 0;
	for (int i = 0; i < 5; i++)
	{
		OnGetBuf(&dcss);
		std::cout << "connect frames :" << dcss.m_Out[0].m_OutFrameSerial << std::endl;
		if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
			frame_update != dcss.m_Out[0].m_OutFrameSerial)
		{
			motion_tag++;
			break;
			frame_update = dcss.m_Out[0].m_OutFrameSerial;
		}
		SLEEP(1);
	}
	if (motion_tag > 0)
	{
		std::cout << "[SUCEESE]:robot connected\n"
				  << std::endl;
		if (log_swtich == 0)
		{
			LogSwitch(0);
		}
		else
		{
			LogSwitch(1);
		}
		SLEEP(SLEEP_TIME);
		return true;
	}
	else
	{
		std::cerr << "[ERROR]:robot connection failed\n"
				  << std::endl;
		return false;
	}
}

void LogSwitch(int signal)
{
	if (signal == 0)
	{
		OnLogOff();
		OnLocalLogOff();
	}
	else if (signal == 1)
	{
		OnLogOn();
		OnLocalLogOn();
	}
	else
	{
		printf("[ERROR] LogSwitch: invalid switch signal: %d. Must be 0 or 1.\n", signal);
	}
}

void EStop(const FX_CHAR *arm)
{
	if (arm == NULL)
	{
		printf("[ERROR] EStop: arm is NULL.\n");
		return;
	}
	size_t len = strlen(arm);
	if (len == 0)
	{
		printf("[ERROR] EStop: arm is empty.\n");
		return;
	}
	if (len > 2)
	{
		printf("[ERROR] EStop: arm \"%s\" too long (max 2 chars).\n", arm);
		return;
	}
	if (strcmp(arm, "A") != 0 &&
		strcmp(arm, "B") != 0 &&
		strcmp(arm, "AB") != 0)
	{
		printf("[ERROR] EStop: invalid arm \"%s\". Must be exactly \"A\", \"B\", or \"AB\".\n", arm);
		return;
	}
	if (strcmp(arm, "A") == 0)
	{
		OnEMG_A();
	}
	else if (strcmp(arm, "B") == 0)
	{
		OnEMG_B();
	}
	else if (strcmp(arm, "AB") == 0)
	{
		OnEMG_AB();
	}
}

void ServoReset(FX_CHAR arm, int axis)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] ServoReset: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return;
	}
	if (axis < 0 || axis > 6)
	{
		printf("[ERROR] ServoReset : Invalid axis number %d (valid range: 0~6)\n", axis);
		return;
	}
	const char *cmd = (arm == 'A') ? "RESETS0" : "RESETS1";
	for (int i = 0; i < 3; ++i)
	{
		CRobot::OnSetIntPara(const_cast<char *>(cmd), axis);
		SLEEP(2);
	}
}

bool SendPVT(FX_CHAR arm, char *local_file, long serial)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SendPVT: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (serial < 0 || serial >= 100)
	{
		printf("[ERROR] SendPVT: Invalid serial  %d (valid range: 0~99)n", serial);
		return false;
	}
	if (arm == 'A')
	{
		return CRobot::OnSendPVT_A(local_file, serial);
	}
	else
	{
		return CRobot::OnSendPVT_B(local_file, serial);
	}
}

bool RunPVT(FX_CHAR arm, int id)
{
	// 先检查是否静止，非静止不让切PVT模式
	// 清错,再切PVT模式
	if (!ValidateArm(arm))
	{
		printf("[ERROR] RunPVT: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (id < 0 || id >= 100)
	{
		printf("[ERROR] RunPVT: Invalid id  %d (valid range: 0~99)n", id);
		return false;
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			if (CRobot::OnGetBuf(&dcss))
				;

			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] RunPVT: arm '%c' is not stationary, switch to position failed.\n", arm);
			return false;
		}
		CRobot::OnClearSet();
		CRobot::OnSetPVT_A(id);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] RunPVT: arm '%c' switch to position failed., timeout.\n", arm);
			return false;
		}
		SLEEP(SLEEP_TIME);
		return true;
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			if (CRobot::OnGetBuf(&dcss))
				;

			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] RunPVT: arm '%c' is not stationary, switch to position failed.\n", arm);
			return false;
		}
		CRobot::OnClearSet();
		CRobot::OnSetPVT_B(id);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] RunPVT: arm '%c' switch to position failed., timeout.\n", arm);
			return false;
		}
		SLEEP(SLEEP_TIME);
		return true;
	}
}

void ClearErr()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "RESET0");
	char name1[30];
	memset(name1, 0, 30);
	sprintf(name1, "RESET1");
	for (long i = 0; i < 3; i++)
	{
		CRobot::OnSetIntPara(name, 0);
		CRobot::OnSetIntPara(name1, 0);
		SLEEP(2);
	}
	if (local_log_tag == true)
	{
		printf("[Marvin SDK]: A&B arm clear error\n");
	}
}

bool SetTool(FX_CHAR arm, double kinePara[6], double dynPara[10])
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetTool: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (kinePara == nullptr || dynPara == nullptr)
	{
		printf("[ERROR] SetTool: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 6; ++i)
	{
		if (isnan(kinePara[i]) || isinf(kinePara[i]))
		{
			printf("[ERROR] SetTool: kinePara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 10; ++i)
	{
		if (isnan(dynPara[i]) || isinf(dynPara[i]))
		{
			printf("[ERROR] SetTool: dynPara[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}

	DCSS dcss;
	if (arm == 'A')
	{
		CRobot::OnClearSet();
		CRobot::OnSetTool_A(kinePara, dynPara);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] SetTool: arm '%c' set tool failed, timeout.\n", arm);
			return false;
		}
		SLEEP(TIME_OUT);
		const float epsilon = 1e-6f;
		const int max_attempts = 3;
		bool verified = false;
		for (int attempt = 0; attempt < max_attempts; ++attempt)
		{
			CRobot::OnGetBuf(&dcss);
			const float *current_kine = dcss.m_In[0].m_ToolKine;
			const float *current_dyn = dcss.m_In[0].m_ToolDyn;
			bool kine_ok = true;
			for (int i = 0; i < 6; ++i)
			{
				if (fabsf(current_kine[i] - static_cast<float>(kinePara[i])) > epsilon)
				{
					kine_ok = false;
					break;
				}
			}
			bool dyn_ok = true;
			for (int i = 0; i < 10; ++i)
			{
				if (fabsf(current_dyn[i] - static_cast<float>(dynPara[i])) > epsilon)
				{
					dyn_ok = false;
					break;
				}
			}
			if (kine_ok && dyn_ok)
			{
				verified = true;
				break;
			}
			SLEEP(SLEEP_TIME);
		}
		return verified;
	}
	if (arm == 'B')
	{
		CRobot::OnClearSet();
		CRobot::OnSetTool_B(kinePara, dynPara);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] SetTool: arm '%c' set tool failed, timeout.\n", arm);
			return false;
		}
		SLEEP(SLEEP_TIME);
		const float epsilon = 1e-6f;
		const int max_attempts = 3;
		bool verified = false;
		for (int attempt = 0; attempt < max_attempts; ++attempt)
		{
			CRobot::OnGetBuf(&dcss);
			const float *current_kine = dcss.m_In[1].m_ToolKine;
			const float *current_dyn = dcss.m_In[1].m_ToolDyn;
			bool kine_ok = true;
			for (int i = 0; i < 6; ++i)
			{
				if (fabsf(current_kine[i] - static_cast<float>(kinePara[i])) > epsilon)
				{
					kine_ok = false;
					break;
				}
			}
			bool dyn_ok = true;
			for (int i = 0; i < 10; ++i)
			{
				if (fabsf(current_dyn[i] - static_cast<float>(dynPara[i])) > epsilon)
				{
					dyn_ok = false;
					break;
				}
			}
			if (kine_ok && dyn_ok)
			{
				verified = true;
				break;
			}
			SLEEP(SLEEP_TIME);
		}
		if (!verified)
		{
			printf("[ERROR] SetTool: arm '%c' set tool failed, pleas retry or check data.\n", arm);
		}
		return verified;
	}
}

bool SetJointMode(FX_CHAR arm, int velRatio, int AccRatio)
{
	// 先检查是否静止，非静止不让切位置模式
	// 非位置模式，设速度，再切位置模式
	// 已经是位置模式，设速度
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetJointMode: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (velRatio < 1)
	{
		printf("[WARNING] SetJointMode: arm '%c' velRatio %d is below minimum, set to 1\n", arm, velRatio);
		velRatio = 1;
	}
	else if (velRatio > 100)
	{
		printf("[WARNING] SetJointMode: arm '%c' velRatio %d exceeds maximum, set to 100\n", arm, velRatio);
		velRatio = 100;
	}
	if (AccRatio < 1)
	{
		printf("[WARNING] SetJointMode: arm '%c' AccRatio %d is below minimum, set to 1\n", arm, AccRatio);
		AccRatio = 1;
	}
	else if (AccRatio > 100)
	{
		printf("[WARNING] SetJointMode: arm '%c' AccRatio %d exceeds maximum, set to 100\n", arm, AccRatio);
		AccRatio = 100;
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetJointMode: arm '%c' is not stationary, switch to position failed.\n", arm);
			return false;
		}
		if (dcss.m_State[0].m_CurState == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(velRatio, AccRatio);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointMode: arm '%c' already in position state, but set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_State[0].m_CurState == 1)
			{
				return true;
			}
			else
			{
				printf("[ERROR] SetJointMode: arm '%c' switch to position state failed.\n", arm);
				return false;
			}
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(velRatio, AccRatio);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointMode: arm '%c' set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_POSITION);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointMode: arm '%c' switch to position failed., timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetJointMode: arm '%c' is not stationary, switch to position failed.\n", arm);
			return false;
		}
		if (dcss.m_State[1].m_CurState == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(velRatio, AccRatio);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointMode: arm '%c' already in position state, but set vel and acc failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(velRatio, AccRatio);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointMode: arm '%c' set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_POSITION);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointMode: arm '%c' switch to position failed., timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
}

bool SetImpJointMode(FX_CHAR arm, int velRatio, int AccRatio, double K[7], double D[7])
{
	// 先检查是否静止，非静止不让切扭矩模式
	// 再检查是否是是扭矩模式，是否是关节阻抗
	// 非关节阻抗，设速度和KD，再切关节阻抗
	// 已经是关节阻抗，设速度和KD
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetImpJointMode: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (velRatio < 1)
	{
		printf("[WARNING] SetImpJointMode: arm '%c' velRatio %d is below minimum, set to 1\n", arm, velRatio);
		velRatio = 1;
	}
	else if (velRatio > 100)
	{
		printf("[WARNING] SetImpJointMode: arm '%c' velRatio %d exceeds maximum, set to 100\n", arm, velRatio);
		velRatio = 100;
	}
	if (AccRatio < 1)
	{
		printf("[WARNING] SetImpJointMode: arm '%c' AccRatio %d is below minimum, set to 1\n", arm, AccRatio);
		AccRatio = 1;
	}
	else if (AccRatio > 100)
	{
		printf("[WARNING] SetImpJointMode: arm '%c' AccRatio %d exceeds maximum, set to 100\n", arm, AccRatio);
		AccRatio = 100;
	}
	if (K == nullptr || D == nullptr)
	{
		printf("[ERROR] SetImpJointMode: arm '%c' Null pointer input\n", arm);
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(K[i]) || isinf(K[i]))
		{
			printf("[ERROR] SetImpJointMode: arm '%c' K[%d] is invalid (NaN or Inf)\n", arm, i);
			return false;
		}
		if (K[i] < 0.0)
		{
			printf("[WARNING] SetImpJointMode: arm '%c' K[%d] is negative (%f), set to 0\n", arm, i, K[i]);
			K[i] = 0.0;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(D[i]) || isinf(D[i]))
		{
			printf("[ERROR] SetImpJointMode: arm '%c' D[%d] is invalid (NaN or Inf)\n", arm, i);
			return false;
		}
		if (D[i] < 0.0)
		{
			printf("[WARNING] SetImpJointMode: arm '%c' D[%d] is negative (%f), set to 0\n", arm, i, D[i]);
			D[i] = 0.0;
		}
		else if (D[i] > 1.0)
		{
			printf("[WARNING] SetImpJointMode: arm '%c' D[%d] exceeds 1 (%f), set to 1\n", arm, i, D[i]);
			D[i] = 1.0;
		}
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetImpJointMode: arm '%c' is not stationary, switch to joint impedance mode failed.\n", arm);
			return false;
		}
		if (dcss.m_State[0].m_CurState == 3 && dcss.m_In[0].m_ImpType == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(velRatio, AccRatio);
			CRobot::OnSetJointKD_A(K, D);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpJointMode: arm '%c' already in joint impedance state, but set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(velRatio, AccRatio);
			CRobot::OnSetJointKD_A(K, D);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpJointMode: arm '%c' set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_TORQ);
			CRobot::OnSetImpType_A(ARM_IMP_JOINT);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpJointMode: arm '%c' switch to joint impedance state failed., timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetImpJointMode: arm '%c' is not stationary, switch to joint impedance mode failed.\n", arm);
			return false;
		}
		if (dcss.m_State[1].m_CurState == 3 && dcss.m_In[1].m_ImpType == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(velRatio, AccRatio);
			CRobot::OnSetJointKD_B(K, D);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpJointMode: arm '%c' already in joint impedance state, but set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(velRatio, AccRatio);
			CRobot::OnSetJointKD_B(K, D);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpJointMode: arm '%c' set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_TORQ);
			CRobot::OnSetImpType_B(ARM_IMP_JOINT);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpJointMode: arm '%c' switch to joint impedance state failed., timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
}

bool SetImpCartMode(FX_CHAR arm, int velRatio, int AccRatio, double K[7], double D[7], int RotType, double CartCtrlPara[7])
{
	// 先检查是否静止，非静止不让切扭矩模式
	// 再检查是否是是扭矩模式，是否是笛卡尔阻抗
	// 非笛卡尔阻抗，设速度和KD，再切笛卡尔阻抗
	// 已经是笛卡尔阻抗，设速度和KD
	// 如果定义末端旋转，再设旋转参数
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetImpCartMode: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (velRatio < 1)
	{
		printf("[WARNING] SetImpCartMode: arm '%c' velRatio %d is below minimum, set to 1\n", arm, velRatio);
		velRatio = 1;
	}
	else if (velRatio > 100)
	{
		printf("[WARNING] SetImpCartMode: arm '%c' velRatio %d exceeds maximum, set to 100\n", arm, velRatio);
		velRatio = 100;
	}
	if (AccRatio < 1)
	{
		printf("[WARNING] SetImpCartMode: arm '%c' AccRatio %d is below minimum, set to 1\n", arm, AccRatio);
		AccRatio = 1;
	}
	else if (AccRatio > 100)
	{
		printf("[WARNING] SetImpCartMode: arm '%c' AccRatio %d exceeds maximum, set to 100\n", arm, AccRatio);
		AccRatio = 100;
	}
	if (K == nullptr || D == nullptr)
	{
		printf("[ERROR] SetImpCartMode: arm '%c' Null pointer input\n", arm);
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(K[i]) || isinf(K[i]))
		{
			printf("[ERROR] SetImpCartMode: arm '%c' K[%d] is invalid (NaN or Inf)\n", arm, i);
			return false;
		}
		if (K[i] < 0.0)
		{
			printf("[WARNING] SetImpCartMode: arm '%c' K[%d] is negative (%f), set to 0\n", arm, i, K[i]);
			K[i] = 0.0;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(D[i]) || isinf(D[i]))
		{
			printf("[ERROR] SetImpCartMode: arm '%c' D[%d] is invalid (NaN or Inf)\n", arm, i);
			return false;
		}
		if (D[i] < 0.0)
		{
			printf("[WARNING] SetImpCartMode: arm '%c' D[%d] is negative (%f), set to 0\n", arm, i, D[i]);
			D[i] = 0.0;
		}
		else if (D[i] > 1.0)
		{
			printf("[WARNING] SetImpCartMode: arm '%c' D[%d] exceeds 1 (%f), set to 1\n", arm, i, D[i]);
			D[i] = 1.0;
		}
	}
	if (RotType != 0 && RotType != 1 && RotType != 2)
	{
		printf("[ERROR] SetImpCartMode: arm '%c' Invalid RotType number %d (valid range: 0~2)\n", arm, RotType);
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(CartCtrlPara[i]) || isinf(CartCtrlPara[i]))
		{
			printf("[ERROR] SetImpCartMode: arm '%c' CartCtrlPara[%d] is invalid (NaN or Inf)\n", arm, i);
			return false;
		}
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetImpCartMode: arm '%c' is not stationary, switch to cartesian impedance mode failed.\n", arm);
			return false;
		}
		if (dcss.m_State[0].m_CurState == 3 && dcss.m_In[0].m_ImpType == 2)
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(velRatio, AccRatio);
			CRobot::OnSetCartKD_A(K, D, 2);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpCartMode: arm '%c' already in cartesian impedance state, but set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			if (RotType != 0)
			{
				SLEEP(SLEEP_TIME);
				CRobot::OnClearSet();
				CRobot::OnSetEefRot_A(RotType, CartCtrlPara);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] SetImpCartMode: arm '%c' set RotType Aand CartCtrlParad failed, timeout.\n", arm);
					return false;
				}
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(velRatio, AccRatio);
			CRobot::OnSetCartKD_A(K, D, 2);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpCartMode: arm '%c' set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_TORQ);
			CRobot::OnSetImpType_A(ARM_IMP_CART);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpCartMode: arm '%c' switch to cartesian impedance state failed., timeout.\n", arm);
				return false;
			}
			if (RotType != 0)
			{
				SLEEP(SLEEP_TIME);
				CRobot::OnClearSet();
				CRobot::OnSetEefRot_A(RotType, CartCtrlPara);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] SetImpCartMode: arm '%c' set RotType Aand CartCtrlParad failed, timeout.\n", arm);
					return false;
				}
			}
			return true;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetImpCartMode: arm '%c' is not stationary, switch to cartesian impedance mode failed.\n", arm);
			return false;
		}
		if (dcss.m_State[1].m_CurState == 3 && dcss.m_In[1].m_ImpType == 2)
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(velRatio, AccRatio);
			CRobot::OnSetCartKD_B(K, D, 2);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpCartMode: arm '%c' already in cartesian impedance state, but set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			if (RotType != 0)
			{
				SLEEP(SLEEP_TIME);
				CRobot::OnClearSet();
				CRobot::OnSetEefRot_B(RotType, CartCtrlPara);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] SetImpCartMode: arm '%c' set RotType Aand CartCtrlParad failed, timeout.\n", arm);
					return false;
				}
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(velRatio, AccRatio);
			CRobot::OnSetCartKD_B(K, D, 2);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpCartMode: arm '%c' set vel & acc & k & d failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_TORQ);
			CRobot::OnSetImpType_B(ARM_IMP_CART);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpCartMode: arm '%c' switch to cartesian impedance state failed., timeout.\n", arm);
				return false;
			}
			if (RotType != 0)
			{
				SLEEP(SLEEP_TIME);
				CRobot::OnClearSet();
				CRobot::OnSetEefRot_B(RotType, CartCtrlPara);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] SetImpCartMode: arm '%c' set RotType Aand CartCtrlParad failed, timeout.\n", arm);
					return false;
				}
			}
			return true;
		}
	}
}

bool SetImpForceMode(FX_CHAR arm, double fxDir[6], double fcAdjLmt)
{
	// 先检查是否静止，非静止不让切扭矩模式
	// 再检查是否是是扭矩模式，是否是力阻抗
	// 非力阻抗，设速度和KD，再切力阻抗
	// 已经是力阻抗，设力控参数
	double fcCtrlPara[7] = {0.0};
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetImpForceMode: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	for (int i = 0; i < 6; ++i)
	{
		if (isnan(fxDir[i]) || isinf(fxDir[i]))
		{
			printf("[ERROR] SetImpForceMode: arm '%c' fxDir[%d] is invalid (NaN or Inf)\n", arm, i);
			return false;
		}
	}
	if (isnan(fcAdjLmt) || isinf(fcAdjLmt))
	{
		printf("[ERROR] SetImpForceMode: arm '%c' fcAdjLmt %d is invalid (NaN or Inf)\n", arm);
		return false;
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetImpForceMode: arm '%c' is not stationary, switch to force impedance mode failed.\n", arm);
			return false;
		}
		if (dcss.m_State[0].m_CurState == 3 && dcss.m_In[0].m_ImpType == 3)
		{
			CRobot::OnClearSet();
			CRobot::OnSetForceCtrPara_A(0, fxDir, fcCtrlPara, fcAdjLmt);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpForceMode: arm '%c' already in force impedance state, but set fcCtrlPara and fcAdjLmt failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetForceCtrPara_A(0, fxDir, fcCtrlPara, fcAdjLmt);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpForceMode: arm '%c' already in force impedance state, but set fcCtrlPara and fcAdjLmt failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_TORQ);
			CRobot::OnSetImpType_A(ARM_IMP_FORCE);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpForceMode: arm '%c' switch to force impedance state failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetImpForceMode: arm '%c' is not stationary, switch to force impedance mode failed.\n", arm);
			return false;
		}
		if (dcss.m_State[1].m_CurState == 3 && dcss.m_In[1].m_ImpType == 3)
		{
			CRobot::OnClearSet();
			CRobot::OnSetForceCtrPara_B(0, fxDir, fcCtrlPara, fcAdjLmt);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpForceMode: arm '%c' already in force impedance state, but set fcCtrlPara and fcAdjLmt failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetForceCtrPara_B(0, fxDir, fcCtrlPara, fcAdjLmt);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpForceMode: arm '%c' already in force impedance state, but set fcCtrlPara and fcAdjLmt failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_TORQ);
			CRobot::OnSetImpType_B(ARM_IMP_FORCE);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetImpForceMode: arm '%c' switch to force impedance state failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
}

bool SetJointDrag(FX_CHAR arm)
{
	// 先判断是否静止
	// 静止下，判断是否位于关节阻抗，不是则切为关节阻抗
	// 关节阻抗模式下设置关节拖动
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetJointDrag: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (!CheckArmError())
	{
		return false;
	}
	if (!CheckServoError())
	{
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetJointDrag: arm '%c' is not stationary, switch to joint drag failed.\n", arm);
			return false;
		}
		if (dcss.m_State[0].m_CurState == 3 && dcss.m_In[0].m_ImpType == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_A(1);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_TORQ);
			CRobot::OnSetImpType_A(ARM_IMP_JOINT);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' switch to joint impedance state failed., timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_A(1);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetJointDrag: arm '%c' is not stationary, switch to joint drag failed.\n", arm);
			return false;
		}
		if (dcss.m_State[1].m_CurState == 3 && dcss.m_In[1].m_ImpType == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_B(1);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_TORQ);
			CRobot::OnSetImpType_B(ARM_IMP_JOINT);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' switch to joint impedance state failed., timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_A(1);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
}

bool SetCartDrag(FX_CHAR arm, FX_CHAR type)
{
	// 先判断是否静止
	// 静止下，判断是否位于关节阻抗，不是则切为关节阻抗
	// 关节阻抗模式下设置关节拖动
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetJointDrag: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (type != 'X' && type != 'Y' && type != 'Z' && type != 'R')
	{
		printf("[ERROR] SetCartDrag: arm '%c' invalid type '%c'. Must be 'X', 'Y', 'Z', or 'R'.\n", arm, type);
		return false;
	}
	int drag_type = 1;
	if (type == 'X')
	{
		drag_type = 2;
	}
	else if (type == 'Y')
	{
		drag_type = 3;
	}
	else if (type == 'Z')
	{
		drag_type = 4;
	}
	else if (type == 'R')
	{
		drag_type = 5;
	}
	if (!CheckArmError())
	{
		printf("[ERROR] SetCartDrag: arm '%c' error cannot be cleared, please check\n", arm);
		return false;
	}
	if (!CheckServoError())
	{
		printf("[ERROR] SetCartDrag: arm '%c' servo error cannot be cleared, please check\n", arm);
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetJointDrag: arm '%c' is not stationary, switch to joint drag failed.\n", arm);
			return false;
		}
		if (dcss.m_State[0].m_CurState == 3 && dcss.m_In[0].m_ImpType == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_A(drag_type);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_TORQ);
			CRobot::OnSetImpType_A(ARM_IMP_CART);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' switch to joint impedance state failed., timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_A(drag_type);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] SetJointDrag: arm '%c' is not stationary, switch to joint drag failed.\n", arm);
			return false;
		}
		if (dcss.m_State[1].m_CurState == 3 && dcss.m_In[1].m_ImpType == 1)
		{
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_B(drag_type);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
		else
		{
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_TORQ);
			CRobot::OnSetImpType_B(ARM_IMP_CART);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' switch to joint impedance state failed., timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetDragSpace_B(drag_type);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] SetJointDrag: arm '%c' already in joint impedance state, but set joint drag failed, timeout.\n", arm);
				return false;
			}
			return true;
		}
	}
}

bool ExitDrag(FX_CHAR arm)
{
	// 先判断是否静止
	// 静止下，判断是否位于关节阻抗，不是则切为关节阻抗
	// 关节阻抗模式下设置关节拖动
	if (!ValidateArm(arm))
	{
		printf("[ERROR] ExitDrag: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	DCSS dcss;
	CRobot::OnGetBuf(&dcss);
	int state_tag = 0;
	if (arm == 'A')
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[0].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] ExitDrag: arm '%c' is not stationary, cannot exit drag.\n", arm);
			return false;
		}
		CRobot::OnClearSet();
		CRobot::OnSetDragSpace_A(0);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] ExitDrag: arm '%c' exit drag failed, timeout.\n", arm);
			return false;
		}
		return true;
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			CRobot::OnGetBuf(&dcss);
			if (dcss.m_Out[1].m_LowSpdFlag == 1)
			{
				state_tag++;
				break;
			}
			SLEEP(1);
		}
		if (state_tag = 0)
		{
			printf("[ERROR] ExitDrag: arm '%c' is not stationary, cannot exit drag.\n", arm);
			return false;
		}
		CRobot::OnClearSet();
		CRobot::OnSetDragSpace_B(0);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] ExitDrag: arm '%c' exit drag failed, timeout.\n", arm);
			return false;
		}
		return true;
	}
}

bool SetJointPostionCmd(FX_CHAR arm, double joint[7])
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetJointPostionCmd: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (joint == nullptr)
	{
		printf("[ERROR] SetJointPostionCmd: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(joint[i]) || isinf(joint[i]))
		{
			printf("[ERROR] SetJointPostionCmd: joint[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (arm == 'A')
	{
		CRobot::OnClearSet();
		CRobot::OnSetJointCmdPos_A(joint);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] SetJointPostionCmd: arm '%c' set joint position failed, timeout.\n", arm);
			return false;
		}
		return true;
	}
	else
	{
		CRobot::OnClearSet();
		CRobot::OnSetJointCmdPos_B(joint);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] SetJointPostionCmd: arm '%c' set joint position failed, timeout.\n", arm);
			return false;
		}
		return true;
	}
}

bool SetForceCmd(FX_CHAR arm, double force)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetForceCmd: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (isnan(force) || isinf(force))
	{
		printf("[ERROR] SetForceCmd: force %d is invalid (NaN or Inf)\n");
		return false;
	}
	if (arm == 'A')
	{
		CRobot::OnClearSet();
		CRobot::OnSetForceCmd_A(force);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] SetForceCmd: arm '%c' set joint position failed, timeout.\n", arm);
			return false;
		}
		return true;
	}
	else
	{
		CRobot::OnClearSet();
		CRobot::OnSetForceCmd_B(force);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] SetForceCmd: arm '%c' set joint position failed, timeout.\n", arm);
			return false;
		}
		return true;
	}
}

bool PlnInit(char *path)
{
	return CRobot::OnInitPlnLmt(path);
}

bool RunPlnJoint(FX_CHAR arm, double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] RunPlnJoint: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (start_joints == nullptr || stop_joints == nullptr)
	{
		printf("[ERROR] RunPlnJoint: Null pointer input\n");
		return false;
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(start_joints[i]) || isinf(start_joints[i]))
		{
			printf("[ERROR] RunPlnJoint: start_joints[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	for (int i = 0; i < 7; ++i)
	{
		if (isnan(stop_joints[i]) || isinf(stop_joints[i]))
		{
			printf("[ERROR] RunPlnJoint: stop_joints[%d] is invalid (NaN or Inf)\n", i);
			return false;
		}
	}
	if (vel_ratio < 0)
	{
		printf("[ERROR] RunPlnJoint: Invalid vel_ratio %d (valid range: 0~1)\n", vel_ratio);
		return false;
	}
	if (vel_ratio > 1)
	{
		printf("[WARNING] RunPlnJoint: Invalid vel_ratio %d (valid range: 0~1), set vel_ratio to 1\n", vel_ratio);
		vel_ratio = 1.0;
	}
	if (acc_ratio < 0)
	{
		printf("[ERROR] RunPlnJoint: Invalid acc_ratio %d (valid range: 0~1)\n", acc_ratio);
		return false;
	}
	if (acc_ratio > 1)
	{
		printf("[WARNING] RunPlnJoint: Invalid acc_ratio %d (valid range: 0~1), set acc_ratio to 1\n", acc_ratio);
		acc_ratio = 1.0;
	}
	int get_vel = 0;
	int get_acc = 0;
	DCSS dcss;
	OnGetBuf(&dcss);
	if (arm == 'A')
	{
		if (dcss.m_State[0].m_CurState == 1)
		{
			get_vel = dcss.m_In[0].m_Joint_Vel_Ratio;
			get_acc = dcss.m_In[0].m_Joint_Acc_Ratio;
			if (get_vel < vel_ratio * 100 && get_acc < acc_ratio * 100)
			{
				CRobot::OnClearSet();
				CRobot::OnSetJointLmt_A(100, 100);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] RunPlnJoint: arm '%c' already in position state, but set vel adn acc failed, timeout.\n", arm);
					return false;
				}
			}
		}
		if (dcss.m_State[0].m_CurState != 1)
		{
			if (!CheckArmError())
			{
				return false;
			}
			if (!CheckServoError())
			{
				return false;
			}
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(100, 100);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnJoint: arm '%c' set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_POSITION);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnJoint: arm '%c' switch to position failed., timeout.\n", arm);
				return false;
			}
		}
		SLEEP(SLEEP_TIME);
		do
		{
			OnGetBuf(&dcss);
			SLEEP(1);
		} while (dcss.m_Out[0].m_TrajState != 0);
		if (!CRobot::OnSetPlnJoint_A(start_joints, stop_joints, vel_ratio, acc_ratio))
		{
			printf("[ERROR] RunPlnJoint: arm '%c' planing joint space failed, please check start_joints and stop_joints\n", arm);
			return false;
		}
		return true;
	}
	else
	{
		if (dcss.m_State[1].m_CurState == 1)
		{
			get_vel = dcss.m_In[1].m_Joint_Vel_Ratio;
			get_acc = dcss.m_In[1].m_Joint_Acc_Ratio;
			if (get_vel < vel_ratio * 100 && get_acc < acc_ratio * 100)
			{
				CRobot::OnClearSet();
				CRobot::OnSetJointLmt_B(100, 100);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] RunPlnJoint: arm '%c' already in position state, but set vel adn acc failed, timeout.\n", arm);
					return false;
				}
			}
		}
		if (dcss.m_State[1].m_CurState != 1)
		{
			if (!CheckArmError())
			{
				return false;
			}
			if (!CheckServoError())
			{
				return false;
			}
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(100, 100);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnJoint: arm '%c' set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_POSITION);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnJoint: arm '%c' switch to position failed., timeout.\n", arm);
				return false;
			}
		}
		SLEEP(SLEEP_TIME);
		do
		{
			OnGetBuf(&dcss);
			SLEEP(1);
		} while (dcss.m_Out[1].m_TrajState != 0);
		if (!CRobot::OnSetPlnJoint_B(start_joints, stop_joints, vel_ratio, acc_ratio))
		{
			printf("[ERROR] RunPlnJoint: arm '%c' planing joint space failed, please check start_joints and stop_joints\n", arm);
			return false;
		}
		return true;
	}
}

bool RunPlnCart(FX_CHAR arm, void *pset)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] RunPlnCart: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	int get_vel = 0;
	int get_acc = 0;
	DCSS dcss;
	OnGetBuf(&dcss);
	if (arm == 'A')
	{
		if (dcss.m_State[0].m_CurState == 1)
		{
			get_vel = dcss.m_In[0].m_Joint_Vel_Ratio;
			get_acc = dcss.m_In[0].m_Joint_Acc_Ratio;
			if (get_vel < 100 && get_acc < 100)
			{
				CRobot::OnClearSet();
				CRobot::OnSetJointLmt_A(100, 100);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] RunPlnCart: arm '%c' already in position state, but set vel adn acc failed, timeout.\n", arm);
					return false;
				}
			}
		}
		if (dcss.m_State[0].m_CurState != 1)
		{
			if (!CheckArmError())
			{
				return false;
			}
			if (!CheckServoError())
			{
				return false;
			}
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_A(100, 100);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnCart: arm '%c' set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_A(ARM_STATE_POSITION);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnCart: arm '%c' switch to position failed., timeout.\n", arm);
				return false;
			}
		}
		SLEEP(SLEEP_TIME);
		do
		{
			OnGetBuf(&dcss);
			SLEEP(1);
		} while (dcss.m_Out[0].m_TrajState != 0);
		if (!CRobot::OnSetPlnCart_A(static_cast<CPointSet *>(pset)))
		{
			printf("[ERROR] RunPlnCart: arm '%c' planning cartesian space failed, please check start_pose and stop_pose\n", arm);
			return false;
		}
		return true;
	}
	else
	{
		if (dcss.m_State[1].m_CurState == 1)
		{
			get_vel = dcss.m_In[1].m_Joint_Vel_Ratio;
			get_acc = dcss.m_In[1].m_Joint_Acc_Ratio;
			if (get_vel < 100 && get_acc < 100)
			{
				CRobot::OnClearSet();
				CRobot::OnSetJointLmt_B(100, 100);
				if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
				{
					printf("[ERROR] RunPlnCart: arm '%c' already in position state, but set vel adn acc failed, timeout.\n", arm);
					return false;
				}
			}
		}
		if (dcss.m_State[1].m_CurState != 1)
		{
			if (!CheckArmError())
			{
				return false;
			}
			if (!CheckServoError())
			{
				return false;
			}
			CRobot::OnClearSet();
			CRobot::OnSetJointLmt_B(100, 100);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnCart: arm '%c' set vel adn acc failed, timeout.\n", arm);
				return false;
			}
			SLEEP(SLEEP_TIME);
			CRobot::OnClearSet();
			CRobot::OnSetTargetState_B(ARM_STATE_POSITION);
			if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
			{
				printf("[ERROR] RunPlnCart: arm '%c' switch to position failed., timeout.\n", arm);
				return false;
			}
		}
		SLEEP(SLEEP_TIME);
		do
		{
			OnGetBuf(&dcss);
			SLEEP(1);
		} while (dcss.m_Out[1].m_TrajState != 0);
		if (!CRobot::OnSetPlnCart_B(static_cast<CPointSet *>(pset)))
		{
			printf("[ERROR] RunPlnCart: arm '%c' planning cartesian space failed, please check start_pose and stop_pose\n", arm);
			return false;
		}
		return true;
	}
}

bool StopPln(FX_CHAR arm)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] StopPln: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (arm == 'A')
	{
		if (!CRobot::OnStopPlnJoint_A())
		{
			printf("[ERROR] StopPln: arm '%c' send cmd timeout.\n", arm);
			return false;
		}
		return true;
	}
	else
	{
		if (!CRobot::OnStopPlnJoint_B())
		{
			printf("[ERROR] StopPln: arm '%c' send cmd timeout.\n", arm);
			return false;
		}
		return true;
	}
}

bool ClearChData(FX_CHAR arm)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] ClearChData: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (arm == 'A')
	{
		return CRobot::OnClearChDataA();
	}
	else
	{
		return CRobot::OnClearChDataB();
	}
}

long GetChData(FX_CHAR arm, unsigned char data_ptr[256], long *ret_ch)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] GetChData: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (*ret_ch != 1 && *ret_ch != 2 && *ret_ch != 3)
	{
		printf("[ERROR] GetChData: arm '%c' Invalid ret_ch number %d (valid range: 1~3)\n", arm, ret_ch);
		return false;
	}
	if (arm == 'A')
	{
		return CRobot::OnGetChDataA(data_ptr, ret_ch);
	}
	else
	{
		return CRobot::OnGetChDataB(data_ptr, ret_ch);
	}
}

long SetChData(FX_CHAR arm, unsigned char data_ptr[256], long size_int, long set_ch)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] SetChData: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	if (size_int <= 0 || size_int > 256)
	{
		printf("[ERROR] SetChData: Channel must in the range of  0~255\n");
		return false;
	}
	if (set_ch != 1 && set_ch != 2 && set_ch != 3)
	{
		printf("[ERROR] SetChData: Invalid set_ch number %d (valid range: 1~3)\n", set_ch);
		return false;
	}
	if (arm == 'A')
	{
		return CRobot::OnSetChDataA(data_ptr, size_int, set_ch);
	}
	else
	{
		return CRobot::OnSetChDataB(data_ptr, size_int, set_ch);
	}
}

bool StartCollectData(long targetNum, long targetID[35], long recordNum)
{
	if (targetNum < 0)
	{
		printf("[ERROR] StartCollectData: Invalid targetNum %ld (valid range: 0~35)\n", targetNum);
		return false;
	}
	else if (targetNum > 35)
	{
		printf("[WARNING] StartCollectData: targetNum %ld exceeds maximum, set to 35)\n", targetNum);
		targetNum = 35;
	}
	if (recordNum < 1000)
	{
		printf("[WARNING] StartCollectData: recordNum %ld is below minimum, set to 1000\n", recordNum);
		recordNum = 1000;
	}
	else if (recordNum > 1000000)
	{
		printf("[WARNING] StartCollectData: recordNum %ld exceeds maximum, set to 1000000\n", recordNum);
		recordNum = 1000000;
	}
	CRobot::OnClearSet();
	CRobot::OnStartGather(targetNum, targetID, recordNum);
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] StartCollectData: start collect data failed, timeout.\n");
		return false;
	}
	return true;
}

bool StopCollectData()
{
	CRobot::OnClearSet();
	CRobot::OnStopGather();
	if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
	{
		printf("[ERROR] StopCollectData: stop collect data failed, timeout.\n");
		return false;
	}
	return true;
}

bool Disable(FX_CHAR arm)
{
	if (!ValidateArm(arm))
	{
		printf("[ERROR] Disable: invalid arm '%c'. Must be 'A' or 'B'.\n", arm);
		return false;
	}
	// if (!CheckArmError())
	// {
	//     return false;
	// }
	// if ( !CheckServoError())
	// {
	//     return false;
	// }
	if (arm == 'A')
	{
		CRobot::OnClearSet();
		CRobot::OnSetTargetState_A(0);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] Disable: arm '%c' disable failed, timeout.\n", arm);
			return false;
		}
		SLEEP(SLEEP_TIME);
		return true;
	}
	else
	{
		CRobot::OnClearSet();
		CRobot::OnSetTargetState_B(0);
		if (CRobot::OnSetSendWaitResponse(TIME_OUT) < 0)
		{
			printf("[ERROR] Disable: arm '%c' disable failed, timeout.\n", arm);
			return false;
		}
		SLEEP(SLEEP_TIME);
		return true;
	}
}
