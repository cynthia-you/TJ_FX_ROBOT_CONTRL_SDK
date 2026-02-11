#ifndef FX_FXROBOT_H_
#define FX_FXROBOT_H_
#include "PointSet.h"
#ifdef __cplusplus
extern "C" {
#endif


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

//math
#define FXARM_D2R						(0.01745329251994329576923690768489)
#define FXARM_R2D						(57.295779513082320876798154814105)

#define  MAX_RUN_ROBOT_NUM 10

typedef FX_DOUBLE Matrix3[3][3];
typedef FX_DOUBLE Matrix4[4][4];
typedef FX_DOUBLE Matrix6[6][6];
typedef FX_DOUBLE Matrix7[7][7];
typedef FX_DOUBLE Matrix8[8][8];
typedef FX_DOUBLE PosGes[4][4];
typedef FX_DOUBLE Quaternion[4];
typedef FX_DOUBLE Vect3[3];
typedef FX_DOUBLE Vect6[6];
typedef FX_DOUBLE Vect7[7];
typedef FX_DOUBLE Vect8[8];

enum FX_ROBOT_TYPES
{
	FX_ROBOT_TYPE_DL = 1006,
	FX_ROBOT_TYPE_PILOT_SRS = 1007,
	FX_ROBOT_TYPE_PILOT_CCS = 1017,
};

enum FX_PILOT_NSP_TYPES
{
	FX_PILOT_NSP_TYPES_NEAR_REF = 0,
	FX_PILOT_NSP_TYPES_NEAR_DIR = 1,
};

enum FX_ROTATION_TYPE
{
	FX_ROT_NULL = 11,

	FX_ROT_EULER_XYZ = 101,
	FX_ROT_EULER_XZY = 102,
	FX_ROT_EULER_YXZ = 103,
	FX_ROT_EULER_YZX = 104,
	FX_ROT_EULER_ZXY = 105,
	FX_ROT_EULER_ZYX = 106,

	FX_ROT_EULER_XYX = 107,
	FX_ROT_EULER_XZX = 108,
	FX_ROT_EULER_YXY = 109,
	FX_ROT_EULER_YZY = 110,
	FX_ROT_EULER_ZXZ = 111,
	FX_ROT_EULER_ZYZ = 112,

	FX_ROT_FIXED_XYZ = 201,
	FX_ROT_FIXED_XZY = 202,
	FX_ROT_FIXED_YXZ = 203,
	FX_ROT_FIXED_YZX = 204,
	FX_ROT_FIXED_ZXY = 205,
	FX_ROT_FIXED_ZYX = 206,

	FX_ROT_FIXED_XYX = 207,
	FX_ROT_FIXED_XZX = 208,
	FX_ROT_FIXED_YXY = 209,
	FX_ROT_FIXED_YZY = 210,
	FX_ROT_FIXED_ZXZ = 211,
	FX_ROT_FIXED_ZYZ = 212,
};

typedef struct
{
	/////////////////////////////// Kinematic
	//////// Kinematic Config
	Matrix4   m_AxisRotBase[7];
	Matrix4   m_Flange;
	Matrix4   m_InvFlange;
	Matrix4   m_FlangeTip;
	Matrix4   m_Tool;
	Matrix4   m_InvTool;
	//////// Kinematic Calculate
	Matrix4   m_TCP;
	Matrix4   m_JointPG[7];
	Matrix4   m_AxisRotTip[7];
	///////////////////////////////
}FX_KineBase;


typedef struct
{
	FX_DOUBLE m_JLmtPos_P[7];
	FX_DOUBLE m_JLmtPos_N[7];
	FX_DOUBLE m_JLmtVel[7];
	FX_DOUBLE m_JLmtAcc[7];
}FX_RobotLmt;

typedef struct
{
	/////////////////////////////// Dynamic
	//////// Dynamic Config
	FX_DOUBLE	m_JntMass[8];
	Vect3		m_JntMCP[8];
	Matrix3		m_JntInertia[8];
	FX_DOUBLE   m_ToolMass;
	Vect3		m_ToolMCP;
	Matrix3		m_ToolInertia;

	FX_BOOL     m_NToolTag;
	FX_DOUBLE   m_NToolMass;
	Vect3		m_NToolMCP;
	Matrix3		m_NToolInertia;

	FX_BOOL     m_BaseFloatationTag;
	Matrix3     m_BaseFloatationGes;
	Vect3       m_BaseFloatationOmg_Dir;
	FX_DOUBLE   m_BaseFloatationOmg_Val;
	Vect3       m_BaseFloatationOmgD_Dir;
	FX_DOUBLE   m_BaseFloatationOmgD_Val;
	Vect3       m_BaseFloatationAcc;
	Vect3		m_CalGravity;
	Vect3		m_SettingGravity;

	FX_BOOL     m_BaseFloatationOmg_Deg_Tag;
	FX_BOOL     m_BaseFloatationOmgD_Deg_Tag;

	//////// Dynamic Calculate

	Vect3		m_JntRotAxisDir_inBase[8];
	Matrix3     m_JntInertia_inBase[8];
	Matrix3     m_ToolInertia_inBase;
	Matrix3     m_NToolInertia_inBase;
	Vect3       m_JntMCP_inBase[8];
	Vect3       m_ToolMCP_inBase;
	Vect3       m_NToolMCP_inBase;

	Vect3		m_OMG[8];
	Vect3		m_OMGd[8];
	Vect3		m_Acc[8];
	Vect3		m_AccMC[8];
	Vect3       m_ToolAccMC;
	Vect3       m_NToolAccMC;

	Vect3       m_NToolDynF_inBase;
	Vect3       m_NToolDynN_inBase;

	Vect3       m_ToolDynF_inBase;
	Vect3       m_ToolDynN_inBase;
	Vect3       m_JntF_inBase[8];
	Vect3       m_JntN_inBase[8];

	Vect3       m_Link_F_inBase[8];
	Vect3       m_Link_N_inBase[8];

}FX_DynBase;

typedef struct
{
	FX_INT32L		m_RobotType;
	FX_INT32L		m_RobotDOF;
	FX_DOUBLE       m_RobotDH[8][4];
	FX_KineBase		m_KineBase;
	FX_RobotLmt		m_Lmt;
	FX_DynBase		m_DynaBase;
	FX_VOID*		m_KineSPC;
}FX_Robot;


typedef struct
{
	FX_BOOL	  m_IsCorss;
	FX_INT32  j4type;
	FX_DOUBLE j4v;
	FX_DOUBLE wristges[3][3];

    FX_DOUBLE rot_m[3][3];
	FX_DOUBLE rot_axis[3];
	FX_DOUBLE j123Base[3][3];
	FX_DOUBLE j567Base[3][3];
}NSPBase;

typedef struct
{
	FX_BOOL		m_IsCross;
	FX_DOUBLE	L1;
	FX_DOUBLE	L2;
	FX_DOUBLE	Ang1;
	FX_DOUBLE	Ang2;
	FX_DOUBLE	Angt;
	FX_DOUBLE	cart_len;
	FX_DOUBLE   m_J4_Bound;

	FX_DOUBLE lmtj67_pp[3];
	FX_DOUBLE lmtj67_np[3];
	FX_DOUBLE lmtj67_nn[3];
	FX_DOUBLE lmtj67_pn[3];

	NSPBase   m_nsp;

}FX_KineSPC_Pilot;


typedef struct
{
	FX_BOOL		m_PAD;

}FX_KineSPC_DL;

typedef struct
{
	FX_INT32L m_AxisNum;
	FX_DOUBLE m_Jcb[6][7];
}FX_Jacobi;

typedef struct
{
	Matrix4					m_Input_IK_TargetTCP;
	Vect7					m_Input_IK_RefJoint;
	FX_INT32L				m_Input_IK_ZSPType;
	FX_DOUBLE				m_Input_IK_ZSPPara[6];
	FX_DOUBLE				m_Input_ZSP_Angle;
	FX_DOUBLE               m_DGR1;
	FX_DOUBLE               m_DGR2;
	FX_DOUBLE               m_DGR3;
	/////////////////////////////////////
	Vect7	m_Output_RetJoint;
	Matrix8 m_OutPut_AllJoint;
	FX_INT32L m_OutPut_Result_Num;
	FX_BOOL m_Output_IsOutRange;
	FX_BOOL m_Output_IsDeg[7];
	FX_BOOL m_Output_JntExdTags[7];
	FX_DOUBLE m_Output_JntExdABS;
	FX_BOOL m_Output_IsJntExd;
	Vect7	m_Output_RunLmtP;
	Vect7	m_Output_RunLmtN;
}FX_InvKineSolvePara;

typedef struct
{
	Vect7 m_Input_Joint;
	Vect7 m_Input_JointVel;
	Vect7 m_Input_JointAcc;

	FX_BOOL m_Input_IsBaseMove;
	Vect3 m_Input_BaseOmg;
	Vect3 m_Input_BaseOmgD;
	Vect3 m_Input_BaseAcc;

	Vect3 m_Input_ExForce;
	Vect3 m_Input_ExTorque;

}FX_InvDynaSolvePara;

#define  _USER_IF_TAG_

#ifdef _USER_IF_TAG_

FX_BOOL  LOADMvCfg(FX_CHAR* path, FX_INT32L TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],
	FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6]);

#endif // _USER_IF_TAG_


////////////////////////////////////////////////////////////////////////////////////////////////
FX_VOID  FX_LOG_SWITCH(FX_INT32L log_tag_input);
FX_BOOL  FX_Robot_Init_Type(FX_INT32L RobotSerial, FX_INT32L RobotType);
FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4]);
FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3]);
////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_Robot_Tool_Set(FX_INT32L RobotSerial, Matrix4 tool);
FX_BOOL  FX_Robot_Tool_Rmv(FX_INT32L RobotSerial);
////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_Robot_Kine_FK(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos);
FX_BOOL  FX_Robot_Kine_FK_NSP(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos, Matrix3 nspg);
FX_BOOL  FX_Robot_Kine_Jacb(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_Jacobi* jcb);
FX_BOOL  FX_Robot_Kine_IK(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para);
FX_BOOL  FX_Robot_Kine_IK_NSP(FX_INT32L RobotSerial, FX_InvKineSolvePara* solve_para);
////////////////////////////////////////////////////////////////////////////////////////////////
/////Motion Planning
FX_BOOL  FX_Robot_CalEndXYZABC(Vect6 Start_XYZABC, Vect3 Pos_offset, FX_INT32L RotType, Vect3 Angle_Param, Vect6 End_XYZABC);
FX_BOOL  FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_CHAR* OutPutPath);
FX_BOOL  FX_Robot_PLN_MOVL_KeepJ(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE acc,FX_CHAR* OutPutPath);
////Parameters Identification
FX_INT32  FX_Robot_Iden_LoadDyn(FX_INT32 Type,FX_CHAR* path,FX_DOUBLE* mass, Vect3 mr, Vect6 I);
////////////////////////////////////////////////////////////////////////////////////////////////
FX_VOID FX_XYZABC2Matrix4DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4]);
FX_BOOL FX_Matrix42XYZABCDEG(FX_DOUBLE m[4][4],FX_DOUBLE xyzabc[6]);


// CPointSet的C风格包装接口
void* FX_CPointSet_Create();
void FX_CPointSet_Destroy(void* pset);
FX_BOOL FX_CPointSet_OnInit(void* pset, FX_INT32L ptype);
FX_INT32L FX_CPointSet_OnGetPointNum(void* pset);
FX_DOUBLE* FX_CPointSet_OnGetPoint(void* pset, FX_INT32L pos);
FX_BOOL FX_CPointSet_OnSetPoint(void* pset, FX_DOUBLE point_value[]);

// 使用C风格接口的MOVLA函数
FX_BOOL FX_Robot_PLN_MOVLA_C(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC,
                            Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, void* ret_pset);
FX_BOOL  FX_Robot_PLN_MOVL_KeepJA_C(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints,
                            FX_DOUBLE vel, FX_DOUBLE acc,void* ret_pset);


#ifdef __cplusplus
}
// C++ only functions
FX_BOOL FX_Robot_PLN_MOVLA(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC,
                          Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, CPointSet* ret_pset);
FX_BOOL  FX_Robot_PLN_MOVL_KeepJA(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints,
                            FX_DOUBLE vel, FX_DOUBLE acc,CPointSet* ret_pset);
#endif

#endif
