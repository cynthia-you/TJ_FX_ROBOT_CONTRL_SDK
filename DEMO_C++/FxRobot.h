#ifndef FX_FXROBOT_H_
#define FX_FXROBOT_H_

#include "PointSet.h"
#include "FXKineCommon.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define _USER_IF_TAG_

#ifdef _USER_IF_TAG_

	FX_BOOL LOADMvCfg(FX_CHAR *path, FX_INT32L TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],
					  FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6]);

#endif // _USER_IF_TAG_

	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_VOID FX_LOG_SWITCH(FX_INT32L log_tag_input);
	FX_BOOL FX_Robot_Init_Type(FX_INT32L RobotSerial, FX_INT32L RobotType);
	FX_BOOL FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4]);
	FX_BOOL FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_BOOL FX_Robot_Tool_Set(FX_INT32L RobotSerial, Matrix4 tool);
	FX_BOOL FX_Robot_Tool_Rmv(FX_INT32L RobotSerial);
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_BOOL FX_Robot_Kine_FK(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos);
	FX_BOOL FX_Robot_Kine_FK_NSP(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos, Matrix3 nspg);
	FX_BOOL FX_Robot_Kine_Jacb(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_Jacobi *jcb);
	FX_BOOL FX_Robot_Kine_IK(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para);
	FX_BOOL FX_Robot_Kine_IK_NSP(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para);
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////Motion Planning
	FX_BOOL FX_Robot_CalEndXYZABC(Vect6 Start_XYZABC, Vect3 Pos_offset, FX_INT32L RotType, Vect3 Angle_Param, Vect6 End_XYZABC);
	FX_BOOL FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32L Freq, FX_CHAR *OutPutPath);
	FX_BOOL FX_Robot_PLN_MOVL_KeepJ(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32L Freq, FX_CHAR *OutPutPath);

	/////Multi-Point Motion Planning
	FX_BOOL FX_Robot_PLN_Set_MOVL_Start(FX_INT32L RobotSerial, Vect7 Ref_Joints, Vect6 Start_XYZABC, Vect6 End_XYZABC, FX_DOUBLE Allow_Range, FX_INT32L ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE Acc, FX_INT32L Freq);
	FX_BOOL FX_Robot_PLN_Set_MOVL_Next_Point(FX_INT32L RobotSerial, Vect6 Next_XYZABC, FX_DOUBLE Allow_Range, FX_INT32L ZSP_Type, Vect6 ZSP_Para, FX_DOUBLE Vel, FX_DOUBLE Acc);
	FX_BOOL FX_Robot_PLN_Get_MOVL_Path(FX_INT32L RobotSerial, CPointSet *ret_Pset);

	/////Joint Torque to EE Torque Mapping
	FX_BOOL FX_Robot_JntTau2EETau(FX_INT32L RobotSerial, Vect7 q, Vect7 Joint_Torque, Vect6 EE_Torque);

	////Parameters Identification
	FX_INT32 FX_Robot_Iden_LoadDyn(FX_INT32 Type, FX_CHAR *path, FX_DOUBLE *mass, Vect3 mr, Vect6 I);
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_VOID FX_XYZABC2Matrix4DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4]);
	FX_BOOL FX_Matrix42XYZABCDEG(FX_DOUBLE m[4][4], FX_DOUBLE xyzabc[6]);

	// CPointSet的C风格包装接口
	void *FX_CPointSet_Create();
	void FX_CPointSet_Destroy(void *pset);
	FX_BOOL FX_CPointSet_OnInit(void *pset, FX_INT32L ptype);
	FX_INT32L FX_CPointSet_OnGetPointNum(void *pset);
	FX_DOUBLE *FX_CPointSet_OnGetPoint(void *pset, FX_INT32L pos);
	FX_BOOL FX_CPointSet_OnSetPoint(void *pset, FX_DOUBLE point_value[]);

	// 使用C风格接口的MOVLA函数
	FX_BOOL FX_Robot_PLN_MOVLA_C(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC,
								 Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32L Freq, void *ret_pset);
	FX_BOOL FX_Robot_PLN_MOVL_KeepJA_C(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints,
									   FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32L Freq, void *ret_pset);
	FX_BOOL FX_Robot_PLN_Get_MOVL_Path_C(FX_INT32L RobotSerial, void *ret_Pset);

#ifdef __cplusplus
}
// C++ only functions
FX_BOOL FX_Robot_PLN_MOVLA(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC,
						   Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT32L Freq, CPointSet *ret_pset);
FX_BOOL FX_Robot_PLN_MOVL_KeepJA(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints,
								 FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32L Freq, CPointSet *ret_pset);
#endif

#endif
