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
	ARM_STATE_RELEASE = 4,			//////// 释放

	ARM_STATE_ERROR = 100, //////// 报错了，清错
	ARM_STATE_TRANS_TO_POSITION = 101, //////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_PVT = 102,//////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_TORQ = 103,//////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_RELEASE = 104,//////// 正常，切换过程,但是如果一直是这个值就是切换失败.
	ARM_STATE_TRANS_TO_IDLE = 109, //////// 正常，切换过程,但是如果一直是这个值就是切换失败.
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
	FX_INT32 m_ImpType;				///* 阻抗类型:1关节阻抗 2 迪卡尔阻抗 3力控*/
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

	FX_INT32 m_DragSpType; 			///* 拖动类型*///5
	FX_FLOAT m_DragSpPara[6]; 		///* 拖动参数类型*///5
	
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

    //////// API之间SLEEP至少1毫秒. 个别API如清错建议至少200毫秒,保存文件建议至少1秒 ////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
     //连接机器人
	 bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);
	 //释放机器人:只要有连接一定要释放,以便别的程序或者用户控制机器人
	 bool OnRelease();
	//////////////////////////////////////////////////////////////////////
	//获取SDK大版本号
	 long OnGetSDKVersion();
	 //升级控制器系统,本地升级包路径
	 bool OnUpdateSystem(char* local_path);
	 //下载控制器日志到本地
	 bool OnDownloadLog(char* local_path);
	 //本地文件上传到控制器远程目录， 绝对路径
	 bool OnSendFile(char* local_file, char* remote_file);
	 //控制器文件从远程传到本地目录， 绝对路径
	 bool OnRecvFile(char* local_file, char* remote_file);
	////////////////////////////////////////////////////////////////////////////////////////////////
	//订阅数据接口,所有数据是结构体.
	 bool OnGetBuf(DCSS * ret);
	////////////////////////////////////////////////////////////////////////////////////////////////
	//软急停
	 void OnEMG_A();
	 void OnEMG_B();
	 void OnEMG_AB();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//获取伺服错误
	 void OnGetServoErr_A(long ErrCode[7]);
	 void OnGetServoErr_B(long ErrCode[7]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	//SDK日志开关
	 void OnLogOn();
	 void OnLogOff();
	 void OnLocalLogOn();
	 void OnLocalLogOff();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//上传本地PVT轨迹文件存为指定ID
	 bool OnSendPVT_A(char* local_file, long serial);
	 bool OnSendPVT_B(char* local_file, long serial);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //获取 设置 保存机器人配置参数
	 long OnSetIntPara(char paraName[30],long setValue);
	 long OnSetFloatPara(char paraName[30], double setValue);
	 long OnGetIntPara(char paraName[30],long * retValue);
	 long OnGetFloatPara(char paraName[30],double * retValue);
	 long OnSavePara();
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //自动修正传感器偏移,测试中
	 long OnAutoRectifySensor();
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //保存数据,该接口后要睡久一点,留够保存数据文件的时间,以防保存出错
	 bool OnSaveGatherData(char * path);
	 bool OnSaveGatherDataCSV(char* path);

	 ////////////////////////////////////////////////////////////////////////////////////////////////
	 //清除缓存指令
	 bool OnClearSet();

	 // 注意 以下的API都要在 OnClearSet() 和 OnSetSend()之间使用 //

	 //清伺服错误,在使用OnLinkTo接口后,立即清错以防总线通讯异常导致
	 void OnClearErr_A();
	 void OnClearErr_B();

     //设置保存数据参数并开始保存
	 bool OnStartGather(long targetNum, long targetID[35], long recordNum);
	 //停止数据采集,用于提前中止
	 bool OnStopGather();

	 //设置指定手臂的工具参数:运动学和动力学参数,运动学参数使正解到TCP, 动力学使扭矩模式可以正常使用
	 bool OnSetTool_A(double kinePara[6], double dynPara[10]);
	 bool OnSetTool_B(double kinePara[6], double dynPara[10]);

	 //切换到控制模式之前先设参数//
	 //1 设置指定手臂的速度和加速度,注意PVT和拖动不受该速度限制
	 bool OnSetJointLmt_A(int velRatio, int AccRatio);
	 bool OnSetJointLmt_B(int velRatio, int AccRatio);
	 //2 设置指定手臂的关节阻抗参数, 在扭矩模式关节阻抗模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(1) 下参数才有意义(以左臂为例)
	 bool OnSetJointKD_A(double K[7], double D[7]);
	 bool OnSetJointKD_B(double K[7], double D[7]);
	 //3 设置指定手臂的迪卡尔阻抗参数, 在扭矩模式迪卡尔阻抗模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(2) 下参数才有意义(以左臂为例)
	 bool OnSetCartKD_A(double K[7], double D[7], int type);
	 bool OnSetCartKD_B(double K[6], double D[6],int type);

	//3.2 设置末端力控类型和笛卡尔方向的旋转
	//设置左臂力控类型fcType=1。 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
	bool OnSetEefRot_A(int fcType, double CartCtrlPara[7]);
	//设置右臂力控类型fcType=1。 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
	bool OnSetEefRot_B(int fcType, double CartCtrlPara[7]);
	 //4 如果使用力控模式,在扭矩模式力控模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(3) 以下两个指令连用
	 //4.1 设置指定手臂的力控参数
	 bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	 bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	 //4.2 设置指定手臂的力值
	 bool OnSetForceCmd_A(double force);
	 bool OnSetForceCmd_B(double force);

	 //设置指定手臂的目标状态:0下使能 1位置 2PVT 3扭矩 4协作释放
	 bool OnSetTargetState_A(int state);
	 bool OnSetTargetState_B(int state);

	 //设置指定手臂的扭矩类型:1关节 2迪卡尔 3力
	 bool OnSetImpType_A(int type);
	 bool OnSetImpType_B(int type);

	 //设置指定手臂的拖动类型,0退出拖动；1关节拖动(进拖动前必须先进关节阻抗模式)；2-5迪卡尔拖动(进每一种迪卡尔拖动前必须先进迪卡尔阻抗模式)
	 bool OnSetDragSpace_A(int dgType);
	 bool OnSetDragSpace_B(int dgType);

	 //设置指定手臂的PVT号并立即运行该轨迹,需在PVT模式下,即OnSetTargetState_A(2)才会生效(以左臂为例)
	 bool OnSetPVT_A(int id);
	 bool OnSetPVT_B(int id);

	 //设置指定手臂的目标关节位置:位置模式扭矩模式下的关节指令
	 bool OnSetJointCmdPos_A(double joint[7]);
	 bool OnSetJointCmdPos_B(double joint[7]);

     // 注意 以上的API都要在 OnClearSet() 和 OnSetSend()之间使用 //
     //发送指令给机器人
	 bool OnSetSend();
	 ////////////////////////////////////////////////////////////////////////////////////////////////

     // 末端工具通讯用接口//
     //1 清缓存数据
	 bool OnClearChDataA();
	 bool OnClearChDataB();
     //2 获取指定手臂指定通道的数据: ret_ch==1:CANFD  ret_ch==2 COM1  ret_ch==3 COM2
	 long OnGetChDataA(unsigned char data_ptr[256], long* ret_ch);
	 bool OnSetChDataA(unsigned char data_ptr[256], long size_int,long set_ch);
     //3 给指定手臂指定通道发送数据
	 long OnGetChDataB(unsigned char data_ptr[256], long* ret_ch);
	 bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch);


#ifdef __cplusplus
}
#endif

#endif


