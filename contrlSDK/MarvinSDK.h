#ifndef FX_SDKIF_H_ 
#define FX_SDKIF_H_
#include "Robot.h"

#ifdef CMPL_WIN
#define FX_DLL_EXPORT __declspec(dllexport) 
#endif

#ifdef CMPL_LIN
#define FX_DLL_EXPORT  
#endif
#ifdef __cplusplus
extern "C" {
#endif
	//////////////////////////////////////////////////////////////////////
	//连接机器人
	FX_DLL_EXPORT bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);
	//释放机器人
	FX_DLL_EXPORT bool OnRelease();
	//////////////////////////////////////////////////////////////////////
	
	FX_DLL_EXPORT long OnGetSDKVersion();
	FX_DLL_EXPORT bool OnUpdateSystem(char* local_path);
	FX_DLL_EXPORT bool OnDownloadLog(char* local_path);
	//本地文件上传到控制器远程目录， 绝对路径
	FX_DLL_EXPORT bool OnSendFile(char* local_file, char* remote_file);
	//控制器文件从远程传到本地目录， 绝对路径
	FX_DLL_EXPORT bool OnRecvFile(char* local_file, char* remote_file);
	//订阅数据结构体
	FX_DLL_EXPORT bool OnGetBuf(DCSS * ret);



	// 左臂软急停
	FX_DLL_EXPORT void OnEMG_A();
	// 右臂软急停
	FX_DLL_EXPORT void OnEMG_B();
	// 左右臂同时软急停
	FX_DLL_EXPORT void OnEMG_AB();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//获取左臂伺服错误码
	FX_DLL_EXPORT void OnGetServoErr_A(long ErrCode[7]);
	//获取右臂伺服错误码
	FX_DLL_EXPORT void OnGetServoErr_B(long ErrCode[7]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	//清除左臂错误
	FX_DLL_EXPORT void OnClearErr_A();
	//清除右臂错误
	FX_DLL_EXPORT void OnClearErr_B();
	//全局日志开
	FX_DLL_EXPORT void OnLogOn();
	//全局日志关
	FX_DLL_EXPORT void OnLogOff();
	//本地日志开
	FX_DLL_EXPORT void OnLocalLogOn();
	//本地日志关
	FX_DLL_EXPORT void OnLocalLogOff();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//上传PVT文件 
	FX_DLL_EXPORT bool OnSendPVT_A(char* local_file, long serial);
	FX_DLL_EXPORT bool OnSendPVT_B(char* local_file, long serial);

	//设置整形和浮点参数信息
	FX_DLL_EXPORT long OnSetIntPara(char paraName[30],long setValue);
	FX_DLL_EXPORT long OnSetFloatPara(char paraName[30], double setValue);
	//读取整形和浮点参数信息
	FX_DLL_EXPORT long OnGetIntPara(char paraName[30],long * retValue);
	FX_DLL_EXPORT long OnGetFloatPara(char paraName[30],double * retValue);
	//保存参数
	FX_DLL_EXPORT long OnSavePara();
	//设置保存参数开始采集数据
	FX_DLL_EXPORT bool OnStartGather(long targetNum, long targetID[35], long recordNum);
	//停止数据采集
	FX_DLL_EXPORT bool OnStopGather();
	//保存采集数据到指定文件，任意保存类型
	FX_DLL_EXPORT bool OnSaveGatherData(char * path);
	//保存采集数据到指定文件，保存类型为CSV
	FX_DLL_EXPORT bool OnSaveGatherDataCSV(char* path);

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	//清除缓存指令
	FX_DLL_EXPORT bool OnClearSet();

	//设置左臂模式
	FX_DLL_EXPORT bool OnSetTargetState_A(int state);
	//设置左臂工具的运动学和动力学参数
	FX_DLL_EXPORT bool OnSetTool_A(double kinePara[6], double dynPara[10]);
	//设置左臂运动的速度百分比和加速度百分比
	FX_DLL_EXPORT bool OnSetJointLmt_A(int velRatio, int AccRatio);
	//设置左臂工具关节阻抗的刚度和阻尼参数
	FX_DLL_EXPORT bool OnSetJointKD_A(double K[7], double D[7]);
	//设置左臂工具笛卡尔阻抗的刚度和阻尼参数，以及阻抗类型（ type=2）
	FX_DLL_EXPORT bool OnSetCartKD_A(double K[7], double D[7], int type);
	//设置左臂工具拖动类型
	FX_DLL_EXPORT bool OnSetDragSpace_A(int dgType);
	//设置左臂力控参数
	FX_DLL_EXPORT bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	//设置左臂目标关节角度
	FX_DLL_EXPORT bool OnSetJointCmdPos_A(double joint[7]);
	//设置左臂力控目标
	FX_DLL_EXPORT bool OnSetForceCmd_A(double force);
	//选择在左臂运行的PVT号并立即n运行轨迹
	FX_DLL_EXPORT bool OnSetPVT_A(int id);
	//设置左臂阻抗类型
	FX_DLL_EXPORT bool OnSetImpType_A(int type);

	//设置右臂模式
	FX_DLL_EXPORT bool OnSetTargetState_B(int state);
	//设置右臂工具的运动学和动力学参数
	FX_DLL_EXPORT bool OnSetTool_B(double kinePara[6], double dynPara[10]);
	//设置右臂运动的速度百分比和加速度百分比
	FX_DLL_EXPORT bool OnSetJointLmt_B(int velRatio, int AccRatio);
	//设置右臂工具关节阻抗的刚度和阻尼参数
	FX_DLL_EXPORT bool OnSetJointKD_B(double K[7], double D[7]);
	//设置右臂工具笛卡尔阻抗的刚度和阻尼参数，以及阻抗类型（ type=2）
	FX_DLL_EXPORT bool OnSetCartKD_B(double K[6], double D[6],int type);
	//设置右臂工具拖动类型
	FX_DLL_EXPORT bool OnSetDragSpace_B(int dgType);
	//设置右臂力控参数
	FX_DLL_EXPORT bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	//设置右臂目标关节角度
	FX_DLL_EXPORT bool OnSetJointCmdPos_B(double joint[7]);
	//设置右臂力控目标
	FX_DLL_EXPORT bool OnSetForceCmd_B(double force);
	//设置右臂阻抗类型
	FX_DLL_EXPORT bool OnSetImpType_B(int type);
	//选择在右臂运行的PVT号并立即n运行轨迹
	FX_DLL_EXPORT bool OnSetPVT_B(int id);

	//发送指令给机器人
	FX_DLL_EXPORT bool OnSetSend();

	//清除左臂末端模块的缓存数据
	FX_DLL_EXPORT bool OnClearChDataA();
	//清除右臂末端模块的缓存数据
	FX_DLL_EXPORT bool OnClearChDataB();
	//获取左臂末端通信模组回复的数据
	FX_DLL_EXPORT long OnGetChDataA(unsigned char data_ptr[256], long* ret_ch);
	//发送协议指令给左臂末端模组
	FX_DLL_EXPORT bool OnSetChDataA(unsigned char data_ptr[256], long size_int,long set_ch);
	//获取右臂末端通信模组回复的数据
	FX_DLL_EXPORT long OnGetChDataB(unsigned char data_ptr[256], long* ret_ch);
	//发送协议指令给右臂末端模组
	FX_DLL_EXPORT bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch);


#ifdef __cplusplus
}
#endif

#endif


