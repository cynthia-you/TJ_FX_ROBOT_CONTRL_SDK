#ifndef FX_ERRORTYPE_H_
#define FX_ERRORTYPE_H_
typedef enum
{
    ERR_LoadIni = 1,
    ERR_IniConfig = 2,
    ERR_MasterConfig = 3,
    ERR_SlaveConfig = 4,
    ERR_ActiveMaster = 5,
    ERR_RtTask = 6,
    ERR_KTask = 7,
    ERR_Internal = 100,
    ERR_Emcy = 101,
    ERR_Servo = 102,
    ERR_PvtStreamBroken = 103,
    ERR_RequestPositionMode = 104,
    ERR_ResponsePositionMode = 105,
    ERR_RequestTorqueMode = 106,
    ERR_ResponseTorqueMode = 107,
    ERR_RequestEnableServo = 108,
    ERR_ResponseEnableServo = 109,
    ERR_ResponseDisableServo = 110,
    ERR_ServoStateAbnormal = 111,
    ERR_SlavePdoAbnormal = 112,
    ERR_SlaveStateAbnormal = 113,
    ERR_BusLinkDown = 114,
} ErrorCode;

#endif