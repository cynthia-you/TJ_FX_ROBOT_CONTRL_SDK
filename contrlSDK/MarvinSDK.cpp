
#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"

static bool local_log_tag = true;



bool OnUpdateSystem(char* local_path)
{
	return CRobot::OnUpdateSystem(local_path);
}
bool OnDownloadLog(char* local_path)
{
	return CRobot::OnDownloadLog(local_path);
}

void OnEMG_A()
{
	CRobot::OnSetIntPara((char *)"EMCY0", 0);
#ifdef CMPL_WIN
	Sleep(10);
#endif 
#ifdef CMPL_LIN
	usleep(10000);
#endif
	CRobot::OnClearSet();
	CRobot::OnSetTargetState_A(ARM_STATE_IDLE);
	CRobot::OnSetSend();
	if(local_log_tag == true)
	{
	    printf("[Marvin SDK]: A arm soft stop! \n");
	}
}
void OnEMG_B()
{
	CRobot::OnSetIntPara((char*)"EMCY1", 0);
#ifdef CMPL_WIN
	Sleep(10);
#endif 
#ifdef CMPL_LIN
	usleep(10000);
#endif
	CRobot::OnClearSet();
	CRobot::OnSetTargetState_B(ARM_STATE_IDLE);
	CRobot::OnSetSend();
	if(local_log_tag == true)
	{
	    printf("[Marvin SDK]: B arm soft stop! \n");
	}
}
void OnEMG_AB()
{
	CRobot::OnSetIntPara((char*)"EMCY0", 0);
	CRobot::OnSetIntPara((char*)"EMCY1", 0);
#ifdef CMPL_WIN
	Sleep(10);
#endif 
#ifdef CMPL_LIN
	usleep(10000);
#endif
	CRobot::OnClearSet();
	CRobot::OnSetTargetState_A(ARM_STATE_IDLE);
	CRobot::OnSetTargetState_B(ARM_STATE_IDLE);

	CRobot::OnSetSend();
	if(local_log_tag == true)
	{
	    printf("[Marvin SDK]: A and B arm soft stop! \n");
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
	if(local_log_tag == true)
	{
	    printf("[Marvin SDK]: A arm Servo error code=[%d,%d,%d,%d,%d,%d,%d],\n",ErrCode[0],ErrCode[1],ErrCode[2] ,ErrCode[3] ,ErrCode[4] ,ErrCode[5] ,ErrCode[6]);
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
	if(local_log_tag == true)
	{
	    printf("[Marvin SDK]: B arm Servo error code=[%d,%d,%d,%d,%d,%d,%d],\n",ErrCode[0],ErrCode[1],ErrCode[2] ,ErrCode[3] ,ErrCode[4] ,ErrCode[5] ,ErrCode[6]);
	}
}



void OnClearErr_A()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "RESET0");
	CRobot::OnSetIntPara(name, 0);
	if(local_log_tag == true)
	{
	    printf("[Marvin SDK]: A arm clear error\n");
	}
}

void OnLogOn()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "LOGON");
	CRobot::OnSetIntPara(name, 0);
	if(local_log_tag == true)
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
	if(local_log_tag ==true)
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

void OnClearErr_B()
{
	char name[30];
	memset(name, 0, 30);
	sprintf(name, "RESET1");
	CRobot::OnSetIntPara(name, 0);
	if(local_log_tag == true) printf("[Marvin SDK]: B arm clear error\n");
}


bool OnSendPVT_A(char* local_file, long serial)
{
	// printf("9\n");
	return CRobot::OnSendPVT_A(local_file, serial);
}
bool OnSendPVT_B(char* local_file, long serial)
{
	return CRobot::OnSendPVT_B(local_file, serial);
}
long OnGetSDKVersion()
{
	return CRobot::OnGetSDKVersion();
}
bool OnSendFile(char* local_file, char* remote_file)
{
	return CRobot::OnSendFile(local_file, remote_file);
}

bool OnRecvFile(char* local_file, char* remote_file)
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
long OnGetIntPara(char paraName[30],long * retValue)
{
	return CRobot::OnGetIntPara(paraName, retValue);
}
long OnGetFloatPara(char paraName[30],double * retValue)
{
	return CRobot::OnGetFloatPara(paraName, retValue);
}
long OnSavePara()
{
	return CRobot::OnSavePara();
}
bool OnGetBuf(DCSS * ret)
{
	return CRobot::OnGetBuf(ret);
}

bool OnStartGather(long targetNum, long targetID[35], long recordNum)
{
	return CRobot::OnStartGather(targetNum, targetID, recordNum);
}
bool OnStopGather()
{
	return CRobot::OnStopGather();
}
bool OnSaveGatherData(char * path)
{
	return CRobot::OnSaveGatherData(path);
}
bool OnSaveGatherDataCSV(char* path)
{
	return CRobot::OnSaveGatherDataCSV(path);
}

bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
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
	return CRobot::OnSetTargetState_A(state);
}
bool OnSetTool_A(double kinePara[6], double dynPara[10])
{
	return CRobot::OnSetTool_A(kinePara, dynPara);
}
bool OnSetJointLmt_A(int velRatio, int AccRatio)
{
	return CRobot::OnSetJointLmt_A(velRatio, AccRatio);
}
bool OnSetJointKD_A(double K[7], double D[7])
{
	return CRobot::OnSetJointKD_A(K, D);
}
bool OnSetCartKD_A(double K[7], double D[7], int type)
{
	return CRobot::OnSetCartKD_A(K, D, type);
}
bool OnSetDragSpace_A(int dgType)
{
	return CRobot::OnSetDragSpace_A(dgType);
}
bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	return CRobot::OnSetForceCtrPara_A(fcType, fxDir, fcCtrlPara, fcAdjLmt);
}
bool OnSetJointCmdPos_A(double joint[7])
{
	return CRobot::OnSetJointCmdPos_A(joint);
}
bool OnSetForceCmd_A(double force)
{
	return CRobot::OnSetForceCmd_A(force);
}
bool OnSetPVT_A(int id)
{
	return CRobot::OnSetPVT_A(id);
}
bool OnSetImpType_A(int type)
{
	return CRobot::OnSetImpType_A(type);
}
bool OnSetTargetState_B(int state)
{
	return CRobot::OnSetTargetState_B(state);
}
bool OnSetTool_B(double kinePara[6], double dynPara[10])
{
	return CRobot::OnSetTool_B(kinePara, dynPara);
}
bool OnSetJointLmt_B(int velRatio, int AccRatio)
{
	return CRobot::OnSetJointLmt_B(velRatio, AccRatio);
}
bool OnSetJointKD_B(double K[7], double D[7])
{
	return CRobot::OnSetJointKD_B(K, D);
}
bool OnSetCartKD_B(double K[6], double D[6],int type)
{
	return CRobot::OnSetCartKD_B(K, D, type);
}

bool OnSetDragSpace_B(int dgType)
{
	return CRobot::OnSetDragSpace_B(dgType);
}
bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
{
	return CRobot::OnSetForceCtrPara_B(fcType, fxDir, fcCtrlPara, fcAdjLmt);
}
bool OnSetJointCmdPos_B(double joint[7])
{
	return CRobot::OnSetJointCmdPos_B(joint);
}
bool OnSetForceCmd_B(double force)
{
	return CRobot::OnSetForceCmd_B(force);
}
bool OnSetImpType_B(int type)
{
	return CRobot::OnSetImpType_B(type);
}
bool OnSetPVT_B(int id)
{
	return CRobot::OnSetPVT_B(id);
}

bool OnSetSend()
{
	return CRobot::OnSetSend();
}


long OnGetChDataA(unsigned char data_ptr[256], long* ret_ch)
{
	return CRobot::OnGetChDataA(data_ptr,ret_ch);
}
bool OnSetChDataA(unsigned char data_ptr[256], long size_int,long set_ch)
{
	return CRobot::OnSetChDataA(data_ptr,size_int,set_ch);
}


//bool AscIIToHex(unsigned char ascbuf[256], long len, unsigned char hexbuf[256], long& retlen)
//{
//	unsigned char V = 0;
//	retlen = 0;
//	long cnt = 0;
//	for (long i = 0; i < len; i++)
//	{
//		unsigned char c = ascbuf[i];
//		unsigned char valid = 0;
//		if (c >= '0' && c <= '9')
//		{
//			V *= 16;
//			V += (c - '0');
//			cnt++;
//			valid = 1;
//		}
//
//		if (c >= 'a' && c <= 'f')
//		{
//			V *= 16;
//			V += (10 + c - 'a');
//			cnt++;
//			valid = 1;
//		}
//
//		if (c >= 'A' && c <= 'F')
//		{
//			V *= 16;
//			V += (10 + c - 'A');
//			cnt++;
//			valid = 1;
//		}
//
//		if (c == ' ')
//		{
//			valid = 1;
//		}
//
//		if (valid == 0)
//		{
//			return false;
//		}
//		if (cnt >= 3)
//		{
//			return false;
//		}
//
//		if (c == ' ' || i == len-1)
//		{
//			if (cnt != 0)
//			{
//				hexbuf[retlen] = V;
//				retlen++;
//				if (retlen >= 63)
//				{
//					return false;
//				}
//				cnt = 0;
//				V = 0;
//			}
//		}
//	}
//	if (retlen <= 0)
//	{
//		return false;
//	}
//	return true;
//}

//bool OnSetChDataA_hex(unsigned char data_ptr[256], long size_int,long set_ch)
//{
//
//    unsigned char buf2[256];
//    memset(buf2, 0, 256);
//    long hlen;
//    if (AscIIToHex(data_ptr, size_int, buf2, hlen) == false)
//    {
//        AfxMessageBox("FORMAT Err");
//        return;
//    }
//	return CRobot::OnSetChDataA_hex(buf2, hlen, 1);
//}

long OnGetChDataB(unsigned char data_ptr[256], long* ret_ch)
{
	return CRobot::OnGetChDataB(data_ptr,ret_ch);
}
bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch)
{
	return CRobot::OnSetChDataB(data_ptr,size_int,set_ch);
}

//bool OnSetChDataB_hex(unsigned char data_ptr[256], long size_int, long set_ch)
//{
//	unsigned char buf2[256];
//    memset(buf2, 0, 256);
//    long hlen;
//    if (AscIIToHex(data_ptr, size_int, buf2, hlen) == false)
//    {
//        AfxMessageBox("FORMAT Err");
//        return;
//    }
//	return CRobot::OnSetChDataB_hex(buf2, hlen, 1);
//}

bool OnClearChDataA()
{
	return CRobot::OnClearChDataA();
}
bool OnClearChDataB()
{
	return CRobot::OnClearChDataB();
}