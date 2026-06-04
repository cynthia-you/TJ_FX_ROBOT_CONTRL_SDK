#ifndef FX_FXMATH_H_
#define FX_FXMATH_H_
#include "FXKineCommon.h"

///////////////////////////////////////////////////////////////////////////////
#define FXARM_D2R (0.01745329251994329576923690768489)
#define FXARM_R2D (57.295779513082320876798154814105)
#define FXARM_PI (3.1415926535897932384626433832795)
#define FXARM_HLFPI (1.5707963267948966192313216916398)
#define FXARMD_2PI (6.283185307179586476925286766559)
#define FXARM_MICRO (1e-6)
#define FXARM_TINYV (1e-9)
#define FXARM_EPS (1e-15)
#define FXARM_EPS_L (1e-13)
///////////////////////////////////////////////////////////////////////////////
typedef FX_DOUBLE Matrix3[3][3];
typedef FX_DOUBLE Matrix4[4][4];
typedef FX_DOUBLE Matrix6[6][6];
typedef FX_DOUBLE Matrix7[7][7];
typedef FX_DOUBLE Matrix8[8][8];
typedef FX_DOUBLE Matrix67[6][7];
typedef FX_DOUBLE Matrix76[7][6];
typedef FX_DOUBLE PosGes[4][4];
typedef FX_DOUBLE Quaternion[4];
typedef FX_DOUBLE Vect3[3];
typedef FX_DOUBLE Vect4[4];
typedef FX_DOUBLE Vect6[6];
typedef FX_DOUBLE Vect7[7];
typedef FX_DOUBLE Vect8[8];

#ifdef __cplusplus
extern "C"
{
#endif

    FX_DOUBLE FX_Value_Sig(FX_DOUBLE x);
    FX_DOUBLE FX_Fabs(FX_DOUBLE x);
    FX_BOOL IsZero(FX_DOUBLE v);
    FX_BOOL IsZeroL(FX_DOUBLE v);
    ///////////////////////////////////////////////////////////////////////////////
    FX_DOUBLE FX_SIN_ARC(FX_DOUBLE ArcAngle);
    FX_DOUBLE FX_COS_ARC(FX_DOUBLE ArcAngle);
    FX_VOID FX_SIN_COS_ARC(FX_DOUBLE ArcAngle, FX_DOUBLE *retSin, FX_DOUBLE *retCos);
    FX_DOUBLE FX_SIN_DEG(FX_DOUBLE DegAngle);
    FX_DOUBLE FX_COS_DEG(FX_DOUBLE DegAngle);
    FX_VOID FX_SIN_COS_DEG(FX_DOUBLE DegAngle, FX_DOUBLE *retSin, FX_DOUBLE *retCos);
    FX_DOUBLE FX_ATan2(FX_DOUBLE dy, FX_DOUBLE dx);
    FX_DOUBLE FX_ACOS(FX_DOUBLE x);
    ///////////////////////////////////////////////////////////////////////////////
    FX_DOUBLE FX_Sqrt(FX_DOUBLE x);
    FX_DOUBLE FX_3Root(FX_DOUBLE ix);
    ///////////////////////////////////////////////////////////////////////////////
    FX_DOUBLE FX_MinDif_Circle(FX_DOUBLE refv, FX_DOUBLE *v);
    FX_DOUBLE FX_Floor(FX_DOUBLE x);

#ifdef __cplusplus
}
#endif

#endif
