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


	//////// API之间SLEEP至少1毫秒. 个别API如清错建议至少200毫秒,保存文件建议至少1秒 ////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //连接机器人
	FX_DLL_EXPORT bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);
	//释放机器人:只要有连接一定要释放,以便别的程序或者用户控制机器人
	FX_DLL_EXPORT bool OnRelease();
	//////////////////////////////////////////////////////////////////////
	//获取SDK大版本号
	FX_DLL_EXPORT long OnGetSDKVersion();
	//升级控制器系统,本地升级包路径
	FX_DLL_EXPORT bool OnUpdateSystem(char* local_path);
	//下载控制器日志到本地
	FX_DLL_EXPORT bool OnDownloadLog(char* local_path);
	//本地文件上传到控制器远程目录， 绝对路径
	FX_DLL_EXPORT bool OnSendFile(char* local_file, char* remote_file);
	//控制器文件从远程传到本地目录， 绝对路径
	FX_DLL_EXPORT bool OnRecvFile(char* local_file, char* remote_file);
	////////////////////////////////////////////////////////////////////////////////////////////////
	//订阅数据接口,所有数据是结构体.
	FX_DLL_EXPORT bool OnGetBuf(DCSS * ret);
    ////////////////////////////////////////////////////////////////////////////////////////////////
	//软急停
	// 左臂软急停
	FX_DLL_EXPORT void OnEMG_A();
	// 右臂软急停
	FX_DLL_EXPORT void OnEMG_B();
	// 左右臂同时软急停
	FX_DLL_EXPORT void OnEMG_AB();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//获取伺服错误
	//获取左臂伺服错误码
	FX_DLL_EXPORT void OnGetServoErr_A(long ErrCode[7]);
	//获取右臂伺服错误码
	FX_DLL_EXPORT void OnGetServoErr_B(long ErrCode[7]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	//全局日志开
	FX_DLL_EXPORT void OnLogOn();
	//全局日志关
	FX_DLL_EXPORT void OnLogOff();
	//本地日志开
	FX_DLL_EXPORT void OnLocalLogOn();
	//本地日志关
	FX_DLL_EXPORT void OnLocalLogOff();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//上传本地PVT轨迹文件存为指定ID
	FX_DLL_EXPORT bool OnSendPVT_A(char* local_file, long serial);
	FX_DLL_EXPORT bool OnSendPVT_B(char* local_file, long serial);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //获取 设置 保存机器人配置参数
	//1 设置整形和浮点参数信息
	FX_DLL_EXPORT long OnSetIntPara(char paraName[30],long setValue);
	FX_DLL_EXPORT long OnSetFloatPara(char paraName[30], double setValue);
	//2 读取整形和浮点参数信息
	FX_DLL_EXPORT long OnGetIntPara(char paraName[30],long * retValue);
	FX_DLL_EXPORT long OnGetFloatPara(char paraName[30],double * retValue);
	//3 保存参数
	FX_DLL_EXPORT long OnSavePara();
	////////////////////////////////////////////////////////////////////////////////////////////////
	//自动修正传感器偏置,测试中
	FX_DLL_EXPORT long OnAutoRectifySensor();
	////////////////////////////////////////////////////////////////////////////////////////////////
    //保存数据,该接口后要睡久一点,留够保存数据文件的时间,以防保存出错
	//保存采集数据到指定文件，任意保存类型
	FX_DLL_EXPORT bool OnSaveGatherData(char * path);
	//保存采集数据到指定文件，保存类型为CSV
	FX_DLL_EXPORT bool OnSaveGatherDataCSV(char* path);

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	//清除缓存指令
	FX_DLL_EXPORT bool OnClearSet();
	// 注意 以下的API都要在 OnClearSet() 和 OnSetSend()之间使用 //

	//清伺服错误,在使用OnLinkTo接口后,立即清错以防总线通讯异常导致
	//清除左臂错误
	FX_DLL_EXPORT void OnClearErr_A();
	//清除右臂错误
	FX_DLL_EXPORT void OnClearErr_B();

	//设置保存参数开始采集数据
	FX_DLL_EXPORT bool OnStartGather(long targetNum, long targetID[35], long recordNum);
	//停止数据采集
	FX_DLL_EXPORT bool OnStopGather();

    //设置指定手臂的工具参数:运动学和动力学参数,运动学参数使正解到TCP, 动力学使扭矩模式可以正常使用
    //设置左臂工具的运动学和动力学参数
	FX_DLL_EXPORT bool OnSetTool_A(double kinePara[6], double dynPara[10]);
    //设置右臂工具的运动学和动力学参数
	FX_DLL_EXPORT bool OnSetTool_B(double kinePara[6], double dynPara[10]);

	//切换到控制模式之前先设参数//
	//1 设置指定手臂的速度和加速度,注意PVT和拖动不受该速度限制
	//设置左臂运动的速度百分比和加速度百分比
	FX_DLL_EXPORT bool OnSetJointLmt_A(int velRatio, int AccRatio);
	//设置右臂运动的速度百分比和加速度百分比
	FX_DLL_EXPORT bool OnSetJointLmt_B(int velRatio, int AccRatio);
	//2 设置指定手臂的关节阻抗参数, 在扭矩模式关节阻抗模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(1) 下参数才有意义(以左臂为例)
	//设置左臂工具关节阻抗的刚度和阻尼参数
	FX_DLL_EXPORT bool OnSetJointKD_A(double K[7], double D[7]);
	//设置右臂工具关节阻抗的刚度和阻尼参数
	FX_DLL_EXPORT bool OnSetJointKD_B(double K[7], double D[7]);
	//3 设置指定手臂的迪卡尔阻抗参数, 在扭矩模式迪卡尔阻抗模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(2) 下参数才有意义(以左臂为例)
	//设置左臂工具笛卡尔阻抗的刚度和阻尼参数，以及阻抗类型（ type=2）
	FX_DLL_EXPORT bool OnSetCartKD_A(double K[7], double D[7], int type);
	//设置右臂工具笛卡尔阻抗的刚度和阻尼参数，以及阻抗类型（ type=2）
	FX_DLL_EXPORT bool OnSetCartKD_B(double K[6], double D[6],int type);
	//4 如果使用力控模式,在扭矩模式力控模式下,即 OnSetTargetState_A(3) && OnSetImpType_A(3) 以下两个指令连用
	//4.1 设置指定手臂的力控参数
	//设置左臂力控参数
	FX_DLL_EXPORT bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	//设置右臂力控参数
	FX_DLL_EXPORT bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	//4.2 设置指定手臂的力值
	//设置左臂力控目标
	FX_DLL_EXPORT bool OnSetForceCmd_A(double force);
	//设置右臂力控目标
	FX_DLL_EXPORT bool OnSetForceCmd_B(double force);

	//设置指定手臂的目标状态:0下使能 1位置 2PVT 3扭矩 4协作释放
	//设置左臂模式
	FX_DLL_EXPORT bool OnSetTargetState_A(int state);
    //设置右臂模式
	FX_DLL_EXPORT bool OnSetTargetState_B(int state);

	//设置指定手臂的扭矩类型:1关节 2迪卡尔 3力
	//设置左臂阻抗类型
	FX_DLL_EXPORT bool OnSetImpType_A(int type);
	//设置右臂阻抗类型
	FX_DLL_EXPORT bool OnSetImpType_B(int type);

	//设置指定手臂的拖动类型,0退出拖动；1关节拖动(进拖动前必须先进关节阻抗模式)；2-5迪卡尔拖动(进每一种迪卡尔拖动前必须先进迪卡尔阻抗模式)
	//设置左臂工具拖动类型
	FX_DLL_EXPORT bool OnSetDragSpace_A(int dgType);
	//设置右臂工具拖动类型
	FX_DLL_EXPORT bool OnSetDragSpace_B(int dgType);

	//设置指定手臂的目标关节位置:位置模式扭矩模式下的关节指令
	//设置左臂目标关节角度
	FX_DLL_EXPORT bool OnSetJointCmdPos_A(double joint[7]);
	//设置右臂目标关节角度
	FX_DLL_EXPORT bool OnSetJointCmdPos_B(double joint[7]);

	//设置指定手臂的PVT号并立即运行该轨迹,需在PVT模式下,即OnSetTargetState_A(2)才会生效(以左臂为例)
	//选择在左臂运行的PVT号并立即n运行轨迹
	FX_DLL_EXPORT bool OnSetPVT_A(int id);
	//选择在右臂运行的PVT号并立即n运行轨迹
	FX_DLL_EXPORT bool OnSetPVT_B(int id);

    // 注意 以上的API都要在 OnClearSet() 和 OnSetSend()之间使用 //
	//发送指令给机器人
	FX_DLL_EXPORT bool OnSetSend();
	////////////////////////////////////////////////////////////////////////////////////////////////

    // 末端工具通讯用接口//
	//清除左臂末端模块的缓存数据
	FX_DLL_EXPORT bool OnClearChDataA();
	//清除右臂末端模块的缓存数据
	FX_DLL_EXPORT bool OnClearChDataB();
	//获取左臂末端通信模组回复的数据 ret_ch==1:CANFD  ret_ch==2 COM1  ret_ch==3 COM2
	FX_DLL_EXPORT long OnGetChDataA(unsigned char data_ptr[256], long* ret_ch);
	//发送协议指令给左臂末端模组 ret_ch==1:CANFD  ret_ch==2 COM1  ret_ch==3 COM2
	FX_DLL_EXPORT bool OnSetChDataA(unsigned char data_ptr[256], long size_int,long set_ch);
	//获取右臂末端通信模组回复的数据
	FX_DLL_EXPORT long OnGetChDataB(unsigned char data_ptr[256], long* ret_ch);
	//发送协议指令给右臂末端模组
	FX_DLL_EXPORT bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch);


#ifdef __cplusplus
}
#endif

#endif


