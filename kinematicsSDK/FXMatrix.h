#ifndef FX_FXMATRIX_H_
#define FX_FXMATRIX_H_
#include "FXMath.h"
#define SVD_MAX_IT_NUM 6000
#ifdef __cplusplus
extern "C" {
#endif

FX_VOID         FX_Vect3Copy(Vect3 src, Vect3 dst);
FX_VOID         FX_Vect4Copy(Vect4 src, Vect4 dst);
FX_VOID         FX_Vect6Copy(Vect6 src, Vect6 dst);
FX_VOID         FX_Vect7Copy(Vect7 src, Vect7 dst);
FX_VOID         FX_Vect8Copy(Vect8 src, Vect8 dst);


FX_VOID			FX_Vect3Sub(Vect3 a, Vect3 b, Vect3 result);
FX_VOID			FX_Vect4Sub(Vect4 a, Vect4 b, Vect4 result);
FX_VOID			FX_Vect6Sub(Vect6 a, Vect6 b, Vect6 result);
FX_VOID			FX_Vect7Sub(Vect7 a, Vect7 b, Vect7 result);
FX_VOID			FX_Vect8Sub(Vect8 a, Vect8 b, Vect8 result);


FX_VOID			FX_Vect3Add(Vect3 a, Vect3 b, Vect3 result);
FX_VOID			FX_Vect4Add(Vect4 a, Vect4 b, Vect4 result);
FX_VOID			FX_Vect6Add(Vect6 a, Vect6 b, Vect6 result);
FX_VOID			FX_Vect7Add(Vect7 a, Vect7 b, Vect7 result);
FX_VOID			FX_Vect8Add(Vect8 a, Vect8 b, Vect8 result);

FX_VOID			FX_Vect3AddToA(Vect3 a, Vect3 b);
FX_VOID			FX_Vect4AddToA(Vect4 a, Vect4 b);
FX_VOID			FX_Vect6AddToA(Vect6 a, Vect6 b);
FX_VOID			FX_Vect7AddToA(Vect7 a, Vect7 b);
FX_VOID			FX_Vect8AddToA(Vect8 a, Vect8 b);

FX_VOID			FX_IdentM33(Matrix3 m);
FX_VOID			FX_IdentM44(Matrix4 m);
FX_VOID			FX_IdentM66(Matrix6 m);
FX_VOID			FX_IdentM77(Matrix7 m);
FX_VOID			FX_IdentM88(Matrix8 m);

FX_VOID			FX_M33Copy(Matrix3 src, Matrix3 dst);
FX_VOID			FX_M44Copy(Matrix4 src, Matrix4 dst);
FX_VOID			FX_M66Copy(Matrix6 src, Matrix6 dst);
FX_VOID			FX_M77Copy(Matrix7 src, Matrix7 dst);
FX_VOID			FX_M88Copy(Matrix8 src, Matrix8 dst);
FX_VOID			FX_M67Copy(Matrix67 src, Matrix67 dst);
FX_VOID			FX_M76Copy(Matrix76 src, Matrix76 dst);

FX_VOID			FX_M33Trans(Matrix3 Org, Matrix3 Result);
FX_VOID			FX_M44Trans(Matrix4 Org, Matrix4 Result);
FX_VOID			FX_M66Trans(Matrix6 Org, Matrix6 Result);
FX_VOID			FX_M77Trans(Matrix7 Org, Matrix7 Result);
FX_VOID			FX_M88Trans(Matrix8 Org, Matrix8 Result);



FX_VOID			FX_MMV3(Matrix3 L, Vect3 R, Vect3 Result);
FX_VOID			FX_MMV4(Matrix4 L, Vect4 R, Vect4 Result);
FX_VOID			FX_MMV6(Matrix6 L, Vect6 R, Vect6 Result);
FX_VOID			FX_MMV7(Matrix7 L, Vect7 R, Vect7 Result);
FX_VOID			FX_MMV8(Matrix8 L, Vect8 R, Vect8 Result);
FX_VOID			FX_MVM666(Vect6 L, Matrix6 R, Vect6 Result);
FX_VOID			FX_MVM777(Vect7 L, Matrix7 R, Vect7 Result);


FX_VOID			FX_MVM677(Matrix67 L, Vect7 R, Vect6 Result);
FX_VOID			FX_MVM766(Matrix76 L, Vect6 R, Vect7 Result);


FX_VOID			FX_MAddM33(Matrix3 L, Matrix3 R, Matrix3 Result);
FX_VOID			FX_MAddM44(Matrix4 L, Matrix4 R, Matrix4 Result);
FX_VOID			FX_MAddM66(Matrix6 L, Matrix6 R, Matrix6 Result);
FX_VOID			FX_MAddM77(Matrix7 L, Matrix7 R, Matrix7 Result);
FX_VOID			FX_MAddM88(Matrix8 L, Matrix8 R, Matrix8 Result);

FX_VOID			FX_MMM33(Matrix3 L, Matrix3 R, Matrix3 Result);
FX_VOID			FX_MMM44(Matrix4 L, Matrix4 R, Matrix4 Result);
FX_VOID			FX_MMM66(Matrix6 L, Matrix6 R, Matrix6 Result);
FX_VOID			FX_MMM77(Matrix7 L, Matrix7 R, Matrix7 Result);
FX_VOID			FX_MMM88(Matrix8 L, Matrix8 R, Matrix8 Result);
FX_VOID			FX_MMM6776(Matrix67 L, Matrix76 R, Matrix6 Result);
FX_VOID			FX_MMM6777(Matrix67 L, Matrix7 R, Matrix67 Result);
FX_VOID			FX_MMM7667(Matrix76 L, Matrix67 R, Matrix7 Result);


FX_BOOL			MatrixInv33(Matrix3 orgm, Matrix3 invm);
FX_BOOL			MatrixInv44(Matrix4 orgm, Matrix4 invm);
FX_BOOL			MatrixInv66(Matrix6 orgm, Matrix6 invm);
FX_BOOL			MatrixInv77(Matrix7 orgm, Matrix7 invm);
FX_BOOL			MatrixInv88(Matrix8 orgm, Matrix8 invm);

FX_BOOL			MatrixInvDP33(Matrix3 m, FX_DOUBLE r, Matrix3 invm);
FX_BOOL			MatrixInvDP44(Matrix4 m, FX_DOUBLE r, Matrix4 invm);
FX_BOOL			MatrixInvDP66(Matrix6 m, FX_DOUBLE r, Matrix6 invm);
FX_BOOL			MatrixInvDP77(Matrix7 m, FX_DOUBLE r, Matrix7 invm);
FX_BOOL			MatrixInvDP88(Matrix8 m, FX_DOUBLE r, Matrix8 invm);

FX_BOOL			FX_SVDM_33(Matrix3 mo, Matrix3 u, Matrix3 s, Matrix3 v);
FX_BOOL			FX_SVDM_44(Matrix4 mo, Matrix4 u, Matrix4 s, Matrix4 v);
FX_BOOL			FX_SVDM_66(Matrix6 mo, Matrix6 u, Matrix6 s, Matrix6 v);
FX_BOOL			FX_SVDM_77(Matrix7 mo, Matrix7 u, Matrix7 s, Matrix7 v);
FX_BOOL			FX_SVDM_88(Matrix8 mo, Matrix8 u, Matrix8 s, Matrix8 v);


FX_VOID			FX_UTM_33(Matrix3 m);
FX_VOID			FX_UTM_44(Matrix4 m);
FX_VOID			FX_UTM_66(Matrix6 m);
FX_VOID			FX_UTM_77(Matrix7 m);
FX_VOID			FX_UTM_88(Matrix8 m);

FX_DOUBLE       FX_DetM_33(Matrix3 m);
FX_DOUBLE       FX_DetM_44(Matrix4 m);
FX_DOUBLE       FX_DetM_66(Matrix6 m);
FX_DOUBLE       FX_DetM_77(Matrix7 m);
FX_DOUBLE       FX_DetM_88(Matrix8 m);

FX_DOUBLE		FX_VectDot3(Vect3 a, Vect3 b);
FX_DOUBLE		FX_VectDot4(Vect4 a, Vect4 b);
FX_DOUBLE		FX_VectDot6(Vect6 a, Vect6 b);
FX_DOUBLE		FX_VectDot7(Vect7 a, Vect7 b);
FX_DOUBLE		FX_VectDot8(Vect8 a, Vect8 b);



FX_VOID			FX_SPMatInv77(Matrix7 orgm, Matrix7 invm);


FX_BOOL         FX_MatrixNormZX(Vect3 z,Vect3 x,Matrix3 ret_m);
FX_BOOL			FX_VectNorm(Vect3 a);
FX_VOID			FX_VectCross(Vect3 a, Vect3 b, Vect3 result);
FX_BOOL			FX_Matrix2ZYZ(Matrix3 m, Vect3  ret);
FX_BOOL			FX_Matrix2ZYX(Matrix3 m, Vect3  ret);
FX_BOOL			FX_Matrix2ZYZ_DGR(Matrix3 m,FX_DOUBLE dgr, Vect3  ret);

FX_VOID			FX_MatRotAxis(Vect3 rot_dir, FX_DOUBLE rot_angle, Matrix3 m, Matrix3 m_roted);
FX_VOID			FX_MMV3(Matrix3 L, Vect3 R, Vect3 Result);
FX_VOID			FX_MMM33(Matrix3 L, Matrix3 R, Matrix3 Result);
FX_VOID			FX_M33Copy(Matrix3 src, Matrix3 dst);
FX_VOID			FX_M33Trans( Matrix3 Org, Matrix3 Result);
FX_VOID			Tmat(FX_DOUBLE DH[4], Matrix4 T);


FX_VOID			FX_PGPointMap(Matrix4 a_to_b, Vect3 in_a, Vect3 ret_in_b);
FX_VOID			FX_PGVectMap(Matrix4 a_to_b, Vect3 in_a, Vect3 ret_in_b);
FX_VOID			FX_PGVectMapInv(Matrix4 a_to_b, Vect3 ret_in_a, Vect3 in_b);
FX_VOID			FX_PGMatrixInv(Matrix4 pg, Matrix4 pginv);
FX_VOID			FX_MMM44(Matrix4 L, Matrix4 R, Matrix4 Result);
FX_VOID			FX_PGMult(Matrix4 L, Matrix4 R, Matrix4 Result);

FX_VOID			FX_PGGetGes(PosGes src, Matrix3 ges);
FX_VOID			FX_PGGetPos(PosGes src, Vect3 pos);
FX_VOID			FX_PGGetAxisX(PosGes src, Vect3 axis_x);
FX_VOID			FX_PGGetAxisY(PosGes src, Vect3 axis_y);
FX_VOID			FX_PGGetAxisZ(PosGes src, Vect3 axis_z);
FX_VOID			FX_PGTranXYZABC2PG(FX_DOUBLE xyzabc[6], FX_DOUBLE pg[4][4]);





FX_BOOL         FX_RightPsoInv67(FX_DOUBLE m[6][7], FX_DOUBLE inv[7][6]);

FX_VOID			FX_QuatMult(FX_DOUBLE q1[4], FX_DOUBLE q2[4], FX_DOUBLE q[4]);
FX_BOOL			FX_QuaternionNorm(FX_DOUBLE q[4]);
FX_DOUBLE		FX_QuaternionSqrtNorm(FX_DOUBLE q[4]);
FX_VOID			FX_QuaternionConj(FX_DOUBLE q[4], FX_DOUBLE retq[4]);
FX_BOOL			FX_QuaternionInverse(FX_DOUBLE q[4], FX_DOUBLE retqInv[4]);
FX_VOID			FX_Matrix2Quaternion(Matrix3 m, Vect4 q);
FX_VOID         FX_ABC2Q(Vect3 abc,Vect4 retq);

FX_VOID PGErr(Matrix4 Td, Matrix4 Te, Vect6 err);
void eig(Matrix6 a, Matrix6 v, double eps);
FX_BOOL generalized_eig(double eps, double A[6][6], double B[6][6], double V[6][6], double D[6]);

#ifdef __cplusplus
}
#endif

#endif
