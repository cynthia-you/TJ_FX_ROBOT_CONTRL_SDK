#ifndef _LoadIdenPub_H_
#include "FXType.h"

typedef struct {
	FX_DOUBLE m;
	FX_DOUBLE r[3];
	FX_DOUBLE I[6];
}LoadDynamicPara;

typedef enum {
	LOAD_IDEN_NoErr = 0, // No error
	LOAD_IDEN_CalErr = 1, // Calculation error; reacquire data and recalculate
	LOAD_IDEN_OpenSmpDateFieErr = 2, // Open sample file error; check the sampled data file
	LOAD_IDEN_OpenCfgFileErr = 3, // Open config file error; the configuration file was modified
	LOAD_IDEN_DataSmpErr = 4, // Data sample error; sampling time was insufficient and valid data is missing
	LOAD_INDEN_TypeErr = 5 // Robot type error; the robot type is not supported
}LoadIdenErrCode;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

LoadIdenErrCode OnCalLoadDyn(LoadDynamicPara *DynPara, FX_INT32 RobotType, const FX_CHAR *UserPath);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !LoadIden_H_
