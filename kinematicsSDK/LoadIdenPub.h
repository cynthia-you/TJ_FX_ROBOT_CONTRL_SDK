#ifndef _LoadIdenPub_H_
#include "FxType.h"

typedef struct {
	FX_DOUBLE m;
	FX_DOUBLE r[3];
	FX_DOUBLE I[6];
}LoadDynamicPara;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

FX_BOOL OnCalLoadDyn(LoadDynamicPara *DynPara, FX_BOOL IsCCS, const FX_CHAR *UserPath);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !LoadIden_H_

