#ifndef FX_MARVINSDK_H_ 
#define FX_MARVINSDK_H_

#define FX_VOID  	void
#define FX_BOOL  	unsigned char
#define FX_TRUE  	1
#define FX_FALSE 	0

#define FX_CHAR   	char
#define FX_UCHAR  	unsigned char
#define FX_INT8   	char
#define FX_INT16 	short 
#define FX_INT32 	int
#define FX_INT32L 	long
#define FX_INT64 	long long

#define FX_UINT8  	unsigned char
#define FX_UINT16 	unsigned short
#define FX_UINT32 	unsigned int
#define FX_UINT32L 	unsigned long
#define FX_UINT64 	unsigned long long

#define FX_FLOAT 	float
#define FX_DOUBLE 	double

typedef enum
{
	ARM_STATE_IDLE = 0,             //////// 下伺服
	ARM_STATE_POSITION = 1,			//////// 位置跟随
	ARM_STATE_PVT = 2,				//////// PVT
	ARM_STATE_TORQ = 3,				//////// 扭矩

	ARM_STATE_ERROR = 100,
	ARM_STATE_TRANS_TO_POSITION = 101,
	ARM_STATE_TRANS_TO_PVT = 102,
	ARM_STATE_TRANS_TO_TORQ = 103,
}ArmState;

typedef struct
{
	FX_INT32   m_CurState;	///* 当前状态 */ ArmState
	FX_INT32   m_CmdState;	///* 指令状态 */ DCSSCmdType 0
	FX_INT32   m_ERRCode;	///* 错误码   */
}StateCtr;

typedef struct
{
	FX_INT32 	m_OutFrameSerial;   	///* 输出帧序号   0 -  1000000 取模*/
	FX_FLOAT    m_FB_Joint_Pos[7];		///* 反馈关节位置 */							0-6
	FX_FLOAT    m_FB_Joint_Vel[7];		///* 反馈关节速度 */							10-16
	FX_FLOAT    m_FB_Joint_PosE[7];		///* 反馈关节位置(外编) */						20-26
	FX_FLOAT    m_FB_Joint_Cmd[7];		///* 位置关节指令 */							30-36
	FX_FLOAT    m_FB_Joint_CToq[7];		///* 反馈关节电流 */							40-46
	FX_FLOAT    m_FB_Joint_SToq[7];		///* 反馈关节扭矩 */							50-56
	FX_FLOAT    m_FB_Joint_Them[7];		///* 反馈关节温度 */
	FX_FLOAT    m_EST_Joint_Firc[7];	///* 关节摩檫力估计值 */						60-66
	FX_FLOAT    m_EST_Joint_Firc_Dot[7];	///* 关节力扰动估计值微分 */				70-76
	FX_FLOAT    m_EST_Joint_Force[7];	///* 关节力扰动估计值 */						80-86
	FX_FLOAT    m_EST_Cart_FN[6];		///* 末端扰动估计值 */							90-95
	FX_CHAR     m_TipDI;
	FX_CHAR     m_LowSpdFlag;			//机器人停止运动标志， 可用于判断是否运动到位。
	FX_CHAR     m_pad[2];
}RT_OUT;

typedef struct
{
	FX_INT32 m_RtInSwitch;  	 	///* 实时输入开关 用户实时数据 进行开关设置 0 -  close rt_in ;1- open rt_in*/
	FX_INT32 m_ImpType;
	FX_INT32 m_InFrameSerial;    	///* 输入帧序号   0 -  1000000 取模*/
	FX_INT16 m_FrameMissCnt;    	///* 丢帧计数*/
	FX_INT16 m_MaxFrameMissCnt;		///* 开 启 后 最 大 丢 帧 计 数 */

	FX_INT32 m_SysCyc;    			///* 0 -  1000000 */
	FX_INT16 m_SysCycMissCnt;		///* 实 时 性  Miss 计 数*/
	FX_INT16 m_MaxSysCycMissCnt;	///* 开 启 后 最 大 实 时 性Miss 计 数 */

	FX_FLOAT m_ToolKine[6];			///* 工 具 运 动 学 参 数 */ 1
	FX_FLOAT m_ToolDyn[10];			///* 工 具 动 力 学 参 数 */ 1

	FX_FLOAT m_Joint_CMD_Pos[7];	///* 关 节 位 置 指 令 */         7     
	FX_INT16 m_Joint_Vel_Ratio;		///* 关 节 速 度 限 制 百分比*/        2
	FX_INT16 m_Joint_Acc_Ratio;		///* 关 节 加 速 度 限 制  百分比*/    2

	FX_FLOAT m_Joint_K[7]; 			///* 关节阻抗刚度K指令*///3
	FX_FLOAT m_Joint_D[7]; 			///* 关节阻抗阻尼D指令*///3

	FX_INT32 m_DragSpType; 			///* 零空间类型*///5
	FX_FLOAT m_DragSpPara[6]; 		///* 零空间参数类型*///5
	
	FX_INT32 m_Cart_KD_Type;		///* 坐标阻抗类型*/
	FX_FLOAT m_Cart_K[6]; 			///* 坐标阻抗刚度K指令*///4
	FX_FLOAT m_Cart_D[6]; 			///* 坐标阻抗阻尼D指令*///4
	FX_FLOAT m_Cart_KN;             /// 4
	FX_FLOAT m_Cart_DN;             /// 4	

	FX_INT32  m_Force_FB_Type;		///* 力控反馈源类型*/
	FX_INT32  m_Force_Type;			///* 力控类型*///6
	FX_FLOAT  m_Force_Dir[6];		///* 力控方向6维空间方向*///6
	FX_FLOAT  m_Force_PIDUL[7];		///* 力控pid*///6
	FX_FLOAT  m_Force_AdjLmt;		///* 允许调节最大范围*///6

	FX_FLOAT  m_Force_Cmd;			///* 力控指令*///8

	FX_UCHAR m_SET_Tags[16];
	FX_UCHAR m_Update_Tags[16];

	FX_UCHAR m_PvtID;
	FX_UCHAR m_PvtID_Update;
	FX_UCHAR m_Pvt_RunID;    // 0: no pvt file; 1~249: user ; 250: TOOL_CALIB.txt; 251: LEFT_ARM_CALIB.txt; 252: RIGHT_ARM_CALIB.txt 
	FX_UCHAR m_Pvt_RunState; // 0: idle; 1: loading ; 2: running; 3: error

}RT_IN;

typedef struct
{
	StateCtr m_State[2];
	RT_IN    m_In[2];
	RT_OUT	 m_Out[2];

	FX_CHAR m_ParaName[30]; // section_name.param_name
	FX_UCHAR m_ParaType; // 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
	FX_UCHAR m_ParaIns;  // DCSSCfgOperationType
	FX_INT32 m_ParaValueI; // FX_INT32 value
	FX_FLOAT m_ParaValueF; // FX_FLOAT value
	FX_INT16 m_ParaCmdSerial; // from PC
	FX_INT16 m_ParaRetSerial; // working: 0; finish: cmd serial; error cmd_serial + 100
}DCSS;



typedef struct
{
	FX_INT32 m_CH;
	FX_INT32 m_SUB_CH;
	FX_INT32 m_Serial;
	FX_INT32 m_Size;
	FX_UCHAR m_Data[256];
}DDSS;


typedef enum
{
    DCSS_CMD_ARM0_TRANS_STATE = 101,
    DCSS_CMD_ARM0_SET_TOOL = 102,
    DCSS_CMD_ARM0_SET_JOINT_VA = 103,
    DCSS_CMD_ARM0_SET_JOINT_KD = 104,
    DCSS_CMD_ARM0_SET_CART_KD = 105,
    DCSS_CMD_ARM0_SET_ZERO_SP = 106,
    DCSS_CMD_ARM0_SET_FORCE_PIDUL = 107,
    DCSS_CMD_ARM0_SET_JOINT_CMD = 108,
    DCSS_CMD_ARM0_SET_FORCE_CMD = 109,
    DCSS_CMD_ARM0_SET_PVT_CMD = 110,
    DCSS_CMD_ARM0_SET_IMP_TYPE = 111,

    DCSS_CMD_CFG_OPERATION = 150,

    DCSS_CMD_ARM1_TRANS_STATE = 201,
    DCSS_CMD_ARM1_SET_TOOL = 202,
    DCSS_CMD_ARM1_SET_JOINT_VA = 203,
    DCSS_CMD_ARM1_SET_JOINT_KD = 204,
    DCSS_CMD_ARM1_SET_CART_KD = 205,
    DCSS_CMD_ARM1_SET_ZERO_SP = 206,
    DCSS_CMD_ARM1_SET_FORCE_PIDUL = 207,
    DCSS_CMD_ARM1_SET_JOINT_CMD = 208,
    DCSS_CMD_ARM1_SET_FORCE_CMD = 209,
    DCSS_CMD_ARM1_SET_PVT_CMD = 210,
    DCSS_CMD_ARM1_SET_IMP_TYPE = 211,

}DCSSCmdType;

typedef enum
{
	DCSS_CFG_OP_SET_INT32 = 101,
	DCSS_CFG_OP_SET_DOUBLE = 102,
	DCSS_CFG_OP_GET_INT32 = 103,
	DCSS_CFG_OP_GET_DOUBLE = 104,
	DCSS_CFG_OP_SAVE = 105,
}DCSSCfgOperationType;


#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////
	 bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);
	 bool OnRelease();
	//////////////////////////////////////////////////////////////////////
	
	 long OnGetSDKVersion();
	 bool OnUpdateSystem(char* local_path);
	 bool OnDownloadLog(char* local_path);
	 bool OnGetBuf(DCSS * ret);
	
	 void OnEMG_A();
	 void OnEMG_B();
	 void OnEMG_AB();
	////////////////////////////////////////////////////////////////////////////////////////////////
	 void OnGetServoErr_A(long ErrCode[7]);
	 void OnGetServoErr_B(long ErrCode[7]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	 void OnClearErr_A();
	 void OnClearErr_B();
	 void OnLogOn();
	 void OnLogOff();
	 void OnLocalLogOn();
	 void OnLocalLogOff();
	////////////////////////////////////////////////////////////////////////////////////////////////
	 bool OnSendPVT_A(char* local_file, long serial);
	 bool OnSendPVT_B(char* local_file, long serial);
	 long OnSetIntPara(char paraName[30],long setValue);
	 long OnSetFloatPara(char paraName[30], double setValue);
	 long OnGetIntPara(char paraName[30],long * retValue);
	 long OnGetFloatPara(char paraName[30],double * retValue);
	 long OnSavePara();

	 bool OnStartGather(long targetNum, long targetID[35], long recordNum);
	 bool OnStopGather();
	 bool OnSaveGatherData(char * path);
	 bool OnSaveGatherDataCSV(char* path);

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	 bool OnClearSet();
	
	 bool OnSetTargetState_A(int state);
	 bool OnSetTool_A(double kinePara[6], double dynPara[10]);
	 bool OnSetJointLmt_A(int velRatio, int AccRatio);
	 bool OnSetJointKD_A(double K[7], double D[7]);
	 bool OnSetCartKD_A(double K[7], double D[7], int type);
	 bool OnSetDragSpace_A(int dgType);
	 bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	 bool OnSetJointCmdPos_A(double joint[7]);
	 bool OnSetForceCmd_A(double force);
	 bool OnSetPVT_A(int id);
	 bool OnSetImpType_A(int type);
	 bool OnSetTargetState_B(int state);
	 bool OnSetTool_B(double kinePara[6], double dynPara[10]);
	 bool OnSetJointLmt_B(int velRatio, int AccRatio);
	 bool OnSetJointKD_B(double K[7], double D[7]);
	 bool OnSetCartKD_B(double K[6], double D[6],int type);
	 bool OnSetDragSpace_B(int dgType);
	 bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	 bool OnSetJointCmdPos_B(double joint[7]);
	 bool OnSetForceCmd_B(double force);
	 bool OnSetImpType_B(int type);
	 bool OnSetPVT_B(int id);

	 bool OnSetSend();

	 bool OnClearChDataA();
	 bool OnClearChDataB();

	 long OnGetChDataA(unsigned char data_ptr[256], long* ret_ch);
	 bool OnSetChDataA(unsigned char data_ptr[256], long size_int,long set_ch);

	 long OnGetChDataB(unsigned char data_ptr[256], long* ret_ch);
	 bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch);



#ifdef __cplusplus
}
#endif

#endif


