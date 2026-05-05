#ifndef FX_FXMATH_H_
#define FX_FXMATH_H_

#include "../Common/FxMathType.h"

#ifdef __cplusplus
extern "C" {
#endif

FX_DOUBLE FX_Value_Sig(FX_DOUBLE x);
FX_DOUBLE FX_Fabs(FX_DOUBLE x);
FX_BOOL IsZero(FX_DOUBLE v);
FX_BOOL IsZeroL(FX_DOUBLE v);
///////////////////////////////////////////////////////////////////////////////
FX_DOUBLE FX_SIN_ARC(FX_DOUBLE ArcAngle);
FX_DOUBLE FX_COS_ARC(FX_DOUBLE ArcAngle);
FX_VOID   FX_SIN_COS_ARC(FX_DOUBLE ArcAngle, FX_DOUBLE* retSin, FX_DOUBLE* retCos);
FX_DOUBLE FX_SIN_DEG(FX_DOUBLE DegAngle);
FX_DOUBLE FX_COS_DEG(FX_DOUBLE DegAngle);
FX_VOID   FX_SIN_COS_DEG(FX_DOUBLE DegAngle, FX_DOUBLE* retSin, FX_DOUBLE* retCos);
FX_DOUBLE FX_ATan2(FX_DOUBLE dy, FX_DOUBLE dx);
FX_DOUBLE FX_ACOS(FX_DOUBLE x);
///////////////////////////////////////////////////////////////////////////////
FX_DOUBLE FX_Sqrt(FX_DOUBLE x);
FX_DOUBLE FX_3Root(FX_DOUBLE ix);
///////////////////////////////////////////////////////////////////////////////
FX_DOUBLE FX_MinDif_Circle(FX_DOUBLE refv, FX_DOUBLE* v);
FX_DOUBLE FX_Floor(FX_DOUBLE x);
FX_DOUBLE FX_Max(FX_DOUBLE a, FX_DOUBLE b);
FX_DOUBLE FX_Min(FX_DOUBLE a, FX_DOUBLE b);

	
#ifdef __cplusplus
}
#endif


#endif
