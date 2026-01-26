#ifndef FX_FXMATH_H_
#define FX_FXMATH_H_

// #include "FxType.h"
#define _FX_CPL_UBUNTU_XENOMAI_J1900_
//#define _FX_CPL_WIN_32_CASE_1_
//#define _FX_CPL_WIN_64_CASE_1_


#ifdef  _FX_CPL_UBUNTU_XENOMAI_J1900_
#define FX_VOID     void

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

#endif
//#define USE_SYS_MATH
///////////////////////////////////////////////////////////////////////////////
#define FXARM_D2R						(0.01745329251994329576923690768489)
#define FXARM_R2D						(57.295779513082320876798154814105)
#define FXARM_PI						(3.1415926535897932384626433832795)
#define FXARM_HLFPI						(1.5707963267948966192313216916398)
#define FXARMD_2PI						(6.283185307179586476925286766559)
#define FXARM_MICRO      				(1e-6)
#define FXARM_TINYV      				(1e-9)
#define FXARM_EPS						(1e-15)
#define FXARM_EPS_L						(1e-13)
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

	
#ifdef __cplusplus
}
#endif


#endif
