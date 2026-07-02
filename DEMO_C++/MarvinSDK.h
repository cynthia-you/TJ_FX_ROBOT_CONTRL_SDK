#ifndef FX_MARVINSDK_H_
#define FX_MARVINSDK_H_

#include "PointSet.h"
#include "FxRtCSDef.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//////// API之间SLEEP至少1毫秒. 个别API如清错建议至少200毫秒,保存文件建议至少1秒 ////////
	// 检查调用方字节大小是否与SDK定义(FxType.h)一致、#pragma pack(4)对齐是否生效，并检测大小端
	// 返回值: 0=全部通过; 负数=不一致(错误码按位或); 1=小端(little-endian); 2=大端(big-endian)
	// pByteOrder: 若非NULL，写入字节序结果 0=小端 1=大端
	int CheckSDKTypeCompat(int *pByteOrder);
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 连接机器人
	bool OnLinkTo(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4);
	// 释放机器人:只要有连接一定要释放,以便别的程序或者用户控制机器人
	bool OnRelease();
	//////////////////////////////////////////////////////////////////////
	// 获取SDK大版本号
	long OnGetSDKVersion();
	// 升级控制器系统,本地升级包路径
	bool OnUpdateSystem(char *local_path);
	// 下载控制器日志到本地
	bool OnDownloadLog(char *local_path);
	// 本地文件上传到控制器远程目录， 绝对路径
	bool OnSendFile(char *local_file, char *remote_file);
	// 控制器文件从远程传到本地目录， 绝对路径
	bool OnRecvFile(char *local_file, char *remote_file);
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 订阅数据接口,所有数据是结构体.
	bool OnGetBuf(DCSS *ret);
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 软急停
	void OnEMG_A();
	void OnEMG_B();
	void OnEMG_AB();
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 获取伺服错误
	void OnGetServoErr_A(long ErrCode[7]);
	void OnGetServoErr_B(long ErrCode[7]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	// SDK日志开关
	void OnLogOn();
	void OnLogOff();
	void OnLocalLogOn();
	void OnLocalLogOff();
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 上传本地PVT轨迹文件存为指定ID
	bool OnSendPVT_A(char *local_file, long serial);
	bool OnSendPVT_B(char *local_file, long serial);
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 获取 设置 保存机器人配置参数
	long OnSetIntPara(char paraName[30], long setValue);
	long OnSetFloatPara(char paraName[30], double setValue);
	long OnGetIntPara(char paraName[30], long *retValue);
	long OnGetFloatPara(char paraName[30], double *retValue);
	long OnSavePara();
	////////////////////////////////////////////////////////////////////////////////////////////////
	// 保存数据,该接口后要睡久一点,留够保存数据文件的时间,以防保存出错
	bool OnSaveGatherData(char *path);
	bool OnSaveGatherDataCSV(char *path);

	////////////////////////////////////////////////////////////////////////////////////////////////
	// 清除缓存指令
	bool OnClearSet();

	// 注意 以下的API都要在 OnClearSet() 和 OnSetSend()之间使用 //

	// 清伺服错误,在使用OnLinkTo接口后,立即清错以防总线通讯异常导致
	void OnClearErr_A();
	void OnClearErr_B();

	// 设置保存数据参数并开始保存
	bool OnStartGather(long targetNum, long targetID[35], long recordNum);
	// 停止数据采集,用于提前中止
	bool OnStopGather();

	// 设置指定手臂的工具参数:运动学和动力学参数,运动学参数使正解到TCP, 动力学使扭矩模式可以正常使用
	bool OnSetTool_A(double kinePara[6], double dynPara[10]);
	bool OnSetTool_B(double kinePara[6], double dynPara[10]);

	// 切换到控制模式之前先设参数//
	// 1 设置指定手臂的速度和加速度,注意PVT和拖动不受该速度限制
	bool OnSetJointLmt_A(int velRatio, int AccRatio);
	bool OnSetJointLmt_B(int velRatio, int AccRatio);
	// 2 设置指定手臂的关节阻抗参数, 在扭矩模式关节阻抗模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(1) 下参数才有意义(以左臂为例)
	bool OnSetJointKD_A(double K[7], double D[7]);
	bool OnSetJointKD_B(double K[7], double D[7]);
	// 3 设置指定手臂的迪卡尔阻抗参数, 在扭矩模式迪卡尔阻抗模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(2) 下参数才有意义(以左臂为例)
	// 3.1 设置笛卡尔阻抗的刚度和阻尼参数
	// 设置左臂笛卡尔阻抗的刚度和阻尼参数，以及阻抗类型（ type=2）
	bool OnSetCartKD_A(double K[7], double D[7], int type = 2);
	// 设置右臂笛卡尔阻抗的刚度和阻尼参数，以及阻抗类型（ type=2）
	bool OnSetCartKD_B(double K[7], double D[7], int type = 2);
	// 3.2 设置指定手臂的末端笛卡尔方向的旋转
	// 设置fcType=1，为自定义笛卡尔旋转方向，CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0
	// 设置fcType=2，为系统自动计算末端笛卡尔旋转， CartCtrlPara全填0
	// 设置fcType=3，与末端力控接口OnSetForceCtrPara_A/OnSetForceCtrPara_B一起使用， CartCtrlPara全填0
	// 设置左臂末端笛卡尔旋转
	bool OnSetEefRot_A(int fcType, double CartCtrlPara[7]);
	// 设置右臂末端笛卡尔旋转
	bool OnSetEefRot_B(int fcType, double CartCtrlPara[7]);
	// 4 如果使用力控模式,在扭矩模式力控模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(3) 以下两个指令连用
	// 4.1 设置指定手臂的力控参数
	// 设置fcType=0，基于基座的力控。
	// 设置fcType=3，末端阻抗下力控，先使用OnSetEefRot_A/OnSetEefRot_B
	// 设置fcType=4，进立场控制，设置后再调用FTArmControl接口
	// 设置左臂力控参数
	bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	// 设置右臂力控参数
	bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	// 4.2 设置指定手臂的力值
	// 设置左臂力控目标
	bool OnSetForceCmd_A(double force);
	// 设置右臂力控目标
	bool OnSetForceCmd_B(double force);

	// 设置指定手臂的目标状态:0下使能 1位置 2PVT 3扭矩 4协作释放
	bool OnSetTargetState_A(int state);
	bool OnSetTargetState_B(int state);

	// 设置指定手臂的扭矩类型:1关节 2迪卡尔 3力
	bool OnSetImpType_A(int type);
	bool OnSetImpType_B(int type);

	// 设置指定手臂的拖动类型,0退出拖动；1关节拖动(进拖动前必须先进关节阻抗模式)；2-5迪卡尔拖动(进每一种迪卡尔拖动前必须先进迪卡尔阻抗模式)
	bool OnSetDragSpace_A(int dgType);
	bool OnSetDragSpace_B(int dgType);

	// 设置指定手臂的PVT号并立即运行该轨迹,需在PVT模式下,即OnSetTargetState_A(2)才会生效(以左臂为例)
	bool OnSetPVT_A(int id);
	bool OnSetPVT_B(int id);

	// 设置指定手臂的目标关节位置:位置模式扭矩模式下的关节指令
	bool OnSetJointCmdPos_A(double joint[7]);
	bool OnSetJointCmdPos_B(double joint[7]);

	// 注意 以上的API都要在 OnClearSet() 和 OnSetSend()之间使用 //
	// 发送指令给机器人
	bool OnSetSend();
	// 发送指令给机器人 设置等待时间，获取指令响应的延时（毫秒）
	long OnSetSendWaitResponse(long time_out);
	////////////////////////////////////////////////////////////////////////////////////////////////

	// pln
	// 关节空间PLN方式发送指令
	bool OnInitPlnLmt(char *path);
	bool OnSetPlnJoint_A(double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio);
	bool OnSetPlnJoint_B(double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio);

	// 笛卡尔空间PLN方式发送指令
	void *FX_CPointSet_Create();
	void FX_CPointSet_Destroy(void *pset);
	bool OnSetPlnCart_A(void *pset);
	bool OnSetPlnCart_B(void *pset);

	// 中断规划运行，笛卡尔空间和关节空间都适用
	bool OnStopPlnJoint_A();
	bool OnStopPlnJoint_B();

	//// collaboration
	// 关节空间两个手臂同时规划运行，注意同时开始，不一定同时结束。
	bool CoRunPlnJoint(double start_joints_A[7], double stop_joints_A[7], double start_joints_B[7], double stop_joints_B[7], double vel_ratio, double acc_ratio);
	// 笛卡尔空间两个手臂从当前点规划方式运行到目标点，规划点位pset由KinematicsSDK计算接口计算得出。
	bool CoRunPlnCart(void *pset0, void *pset1);
	// 同时中断两个手臂的规划运行，笛卡尔空间和关节空间都适用
	bool CoStopPln();

	// 末端工具通讯用接口//
	// 1 清缓存数据
	bool OnClearChDataA();
	bool OnClearChDataB();
	// 2 获取指定手臂指定通道的数据: ret_ch==1:CANFD  ret_ch==2 COM1  ret_ch==3 COM2
	long OnGetChDataA(unsigned char data_ptr[256], long *ret_ch);
	bool OnSetChDataA(unsigned char data_ptr[256], long size_int, long set_ch);
	// 3 给指定手臂指定通道发送数据
	long OnGetChDataB(unsigned char data_ptr[256], long *ret_ch);
	bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch);

	// 设置PD控制速度前馈 轨迹发送周期
	// arm:"A" "B" ； step： 轨迹发送周期（单位： ms ），小于1 则不添加速度前馈
	bool FX_OnSetVelEstStep(char arm, long step);

	// 设置指定手臂的扭矩控制指令
	// 机械臂末端以给定的力和扭矩运动到给定的位置距离和姿态距离。可实时触发调整力的方向和大小。
	// arm:"A" "B" ；
	// FTCmds ：见下FxRtCSDef.h下FTCmd结构体
	bool FTArmControl(char arm, FTCmd FTCmds);

	// 设置获取用户自定义数据接口
	bool OnSetUserSpcfData_A(long data_category);
	bool OnSetUserSpcfData_B(long data_category);
	bool OnSetUserSpcfData(long data_category);

	// 获取机器人名称
	bool OnGetRobotName(char *robotName);

	/////////////////////////////////////简明式接口Concise SDK API//////////////////////////////////////////////
	// 简明式接口，摒弃了老接口需要在OnClearSet() 和 OnSetSend()之间使用，且左右臂要的单独调取用，且需要查询伺服是否有错，清错后使用的逻辑。
	// 简明式接口自行在内部做错误状态检查。
	// 老接口和简明式接口并存兼容

	// 使用简明式接口请注意：老接口中，以下接口未作变化，请正常使用：
	//  //获取 设置 保存机器人配置参数
	//  long OnSetIntPara(char paraName[30],long setValue);
	//  long OnSetFloatPara(char paraName[30], double setValue);
	//  long OnGetIntPara(char paraName[30],long * retValue);
	//  long OnGetFloatPara(char paraName[30],double * retValue);
	//  long OnSavePara();
	//  ////////////////////////////////////////////////////////////////////////////////////////////////
	//  //自动修正传感器偏移,测试中
	//  long OnAutoRectifySensor();
	//  ////////////////////////////////////////////////////////////////////////////////////////////////
	//  //保存数据,该接口后要睡久一点,留够保存数据文件的时间,以防保存出错
	//  bool OnSaveGatherData(char * path);
	//  bool OnSaveGatherDataCSV(char* path);
	//   //释放机器人:只要有连接一定要释放,以便别的程序或者用户控制机器人
	//  bool OnRelease();
	//  //////////////////////////////////////////////////////////////////////
	//  //获取SDK大版本号
	//  long OnGetSDKVersion();
	//  //升级控制器系统,本地升级包路径
	//  bool OnUpdateSystem(char* local_path);
	//  //下载控制器日志到本地
	//  bool OnDownloadLog(char* local_path);
	//  //本地文件上传到控制器远程目录， 绝对路径
	//  bool OnSendFile(char* local_file, char* remote_file);
	//  //控制器文件从远程传到本地目录， 绝对路径
	//  bool OnRecvFile(char* local_file, char* remote_file);
	//  ////////////////////////////////////////////////////////////////////////////////////////////////
	//  //订阅数据接口,所有数据是结构体.
	//  bool OnGetBuf(DCSS * ret);
	// // 设置PD控制速度前馈 轨迹发送周期
	// // arm:"A" "B" ； step： 轨迹发送周期（单位： ms ），小于1 则不添加速度前馈
	// FX_DLL_EXPORT bool FX_OnSetVelEstStep(char arm, long step);
	// // 设置指定手臂的扭矩控制指令
	// // 机械臂末端以给定的力和扭矩运动到给定的位置距离和姿态距离。可实时触发调整力的方向和大小。
	// // arm:"A" "B" ；
	// // FTCmds ：见下FxRtCSDef.h下FTCmd结构体
	// FX_DLL_EXPORT bool FTArmControl(char arm, FTCmd FTCmds);
	// //设置获取用户自定义数据接口
	// FX_DLL_EXPORT bool OnSetUserSpcfData_A(long data_category);
	// FX_DLL_EXPORT bool OnSetUserSpcfData_B(long data_category);
	// FX_DLL_EXPORT bool OnSetUserSpcfData(long data_category);
	// // collaboration
	// // 关节空间两个手臂同时规划运行，注意同时开始，不一定同时结束。
	// FX_DLL_EXPORT bool CoRunPlnJoint(double start_joints_A[7], double stop_joints_A[7], double start_joints_B[7], double stop_joints_B[7], double vel_ratio, double acc_ratio);
	// // 笛卡尔空间两个手臂从当前点规划方式运行到目标点，规划点位pset由KinematicsSDK计算接口计算得出。
	// FX_DLL_EXPORT bool CoRunPlnCart(void *pset0, void *pset1);
	// // 同时中断两个手臂的规划运行，笛卡尔空间和关节空间都适用
	// FX_DLL_EXPORT bool CoStopPln();

	// 连接机器人,log_switch（日志默认为关）： 0 关; 1 开。
	bool Connect(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, int log_switch = 0);

	// 机器人日志开关, signal: 0 关; 1 开
	void LogSwitch(int signal);

	// 指定手臂软急停, arm: "A" "B" "AB" 三种字符是许可值
	void EStop(const char *arm);

	// 指定关节伺服软复位, arm: "A" "B"  两种字符是许可值; axis:0~6
	void ServoReset(char arm, int axis);

	// 检查手臂错误并清错
	bool CheckArmError();
	// 检查伺服错误并清错
	bool CheckServoError();

	// 清除两手臂的错误
	void ClearErr();

	// 设置指定手臂的工具参数:运动学和动力学参数,运动学参数使正解到TCP, 动力学使扭矩模式可以正常使用
	// arm:"A" "B"  两种字符是许可值; kinePara:工具相对于末端法兰的位置的偏移（毫米）和姿态的旋转（角度，XYZ顺序）；dynPara：工具动力学参数，用提供的上位机软件可识别
	bool SetTool(char arm, double kinePara[6], double dynPara[10]);

	// 设置指定手臂的速度和加速度和位置模式,注意PVT和拖动不受该速度限制。arm:"A" "B"  两种字符是许可值; velRatio:0~100; AccRatio:0~100
	bool SetJointMode(char arm, int velRatio, int AccRatio);

	// 设置指定手臂的速度和加速度和关节阻抗模式。arm:"A" "B"  两种字符是许可值; velRatio:0~100; AccRatio:0~100; K:非负值； D：0~1
	bool SetImpJointMode(char arm, int velRatio, int AccRatio, double K[7], double D[7]);

	// 设置指定手臂的速度和加速度和笛卡尔阻抗模式。arm:"A" "B"  两种字符是许可值; velRatio:0~100; AccRatio:0~100; K:非负值； D：0~1
	// 如果不定义末端笛卡尔的旋转：RotType=0；double CartCtrlPara[7]={0}
	// 设置末端笛卡尔方向的旋转：
	// RotType=1，为自定义末端旋转方向； 笛卡尔方向：CartCtrlPara前三个参数置为末端基于基座X Y Z顺序的旋转，后四个为保留参数，填0；
	// RotType=2，为系统自动计算末端笛卡尔旋转； double CartCtrlPara[7]={0}
	bool SetImpCartMode(char arm, int velRatio, int AccRatio, double K[7], double D[7], int RotType, double CartCtrlPara[7]);

	// 设置指定手臂的关节空间位置指令（位置模式扭矩模式下的关节指令）。  arm:"A" "B"  两种字符是许可值； joint：七个关节的目标角度(单位：度）
	bool SetJointPostionCmd(char arm, double joint[7]);

	// 设置指定手臂的力控参数和力阻抗模式。arm:"A" "B"  两种字符是许可值; fxDir：任意定义方向； fcAdjLmt：力的调节范围，单位毫米
	bool SetImpForceMode(char arm, double fxDir[6], double fcAdjLmt);
	// 设置指定手臂的力值：arm:"A" "B"  两种字符是许可值; force: 力，单位：牛
	bool SetForceCmd(char arm, double force);

	// 以规划方式运动到目标点（位置模式下，规划执行频率50HZ）
	// 关节空间规划初始化，只需初始化一次
	bool PlnInit(char *path);
	// 关节空间下从当前点规划方式运行到目标点
	bool RunPlnJoint(char arm, double start_joints[7], double stop_joints[7], double vel_ratio, double acc_ratio);
	// 笛卡尔空间下从当前点规划方式运行到目标点，规划点位pset由KinematicsSDK计算接口FX_Robot_PLN_MOVLA计算得出。
	bool RunPlnCart(char arm, void *pset);
	// 中断规划运行，笛卡尔空间和关节空间都适用
	bool StopPln(char arm);

	// 上传本地PVT轨迹文件存为指定ID, arm:"A" "B"  两种字符是许可值; local_file:相对或绝对路径； serial:0~99
	bool SendPVT(char arm, char *local_file, long serial);
	// 设置指定手臂的PVT号并立即运行该轨迹, arm:"A" "B"  两种字符是许可值； id：0~99(SendPVT上传的pvt文件的serial).特别注意运行PVT，需要将机器人位置调到PVT规划轨迹的起点
	bool RunPVT(char arm, int id);

	// 拖动，每种拖动使用完毕需要退出拖动再切换为别的拖动模式，否则拖动效果是叠加混乱的哦。
	// 设置指定手臂为关节拖动。 arm:"A" "B"  两种字符是许可值
	bool SetJointDrag(char arm);
	// 设置指定手臂为笛卡尔拖动。 arm:"A" "B"  两种字符是许可值; type: "X" "Y" "Z" "R" 四种字符是许可值（X/Y/Z/旋转， 四个方向选一）;
	bool SetCartDrag(char arm, char type);
	// 设置指定手臂退出拖动。arm:"A" "B"  两种字符是许可值
	bool ExitDrag(char arm);

	// 手臂末端安装工具的通讯
	// 清缓存数据。arm:"A" "B"  两种字符是许可值
	bool ClearChData(char arm);
	// 获取指定手臂指定通道的数据. arm:"A" "B" 两种字符是许可值; ret_ch==1: CAN/CANFD  ret_ch==2: COM1  ret_ch==3: COM2
	long GetChData(char arm, unsigned char data_ptr[256], long *ret_ch);
	// 给指定手臂指定通道发送数据. arm:"A" "B" 两种字符是许可值; ret_ch==1: CAN/CANFD  ret_ch==2: COM1  ret_ch==3: COM2
	long SetChData(char arm, unsigned char data_ptr[256], long size_int, long set_ch);

	// 采集数据 停止采集
	// 设置保存参数并开始采集数据
	bool StartCollectData(long targetNum, long targetID[35], long recordNum);
	// 停止数据采集
	bool StopCollectData();
	// 保存数据使用老接口：bool OnSaveGatherData(char * path);

	// 下使能/复位
	// 设置指定手臂下使能/复位。arm:"A" "B"  两种字符是许可值
	bool Disable(char arm);

#ifdef __cplusplus
}
#endif

#endif
