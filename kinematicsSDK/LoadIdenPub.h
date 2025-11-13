#ifndef _LoadIdenPub_H_
#include "FxType.h"

typedef struct {
	FX_DOUBLE m;
	FX_DOUBLE r[3];
	FX_DOUBLE I[6];
}LoadDynamicPara;

typedef enum {
	LOAD_IDEN_NoErr = 0, // No error
	LOAD_IDEN_CalErr = 1, // Calculation error, 计算错误，需重新采集数据计算
	LOAD_IDEN_OpenSmpDateFieErr = 2, //  Open sample file error 打开采集数据文件错误，须检查采样文件
	LOAD_IDEN_OpenCfgFileErr = 3, // Open config file error 配置文件被修改
	LOAD_IDEN_DataSmpErr = 4 // Data sample error 采集时间不够，缺少有效数据
}LoadIdenErrCode;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

LoadIdenErrCode OnCalLoadDyn(LoadDynamicPara *DynPara, FX_INT32 RobotType, const FX_CHAR *UserPath);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !LoadIden_H_
