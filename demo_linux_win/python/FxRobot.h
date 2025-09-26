#ifndef FX_FXROBOT_H_
#define FX_FXROBOT_H_


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
	FX_BOOL m_Output_IsOutRange;
	FX_BOOL m_Output_IsDeg[7];
	FX_BOOL m_Output_JntExdTags[7];
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
FX_BOOL  FX_Robot_Init_Type(FX_INT32L RobotSerial, FX_INT32L RobotType);
FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4]);
FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3]);
////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_Robot_Tool_Set(FX_INT32L RobotSerial, Matrix4 tool);
FX_BOOL  FX_Robot_Tool_Rmv(FX_INT32L RobotSerial);
////////////////////////////////////////////////////////////////////////////////////////////////
FX_BOOL  FX_Robot_Kine_FK(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos);
FX_BOOL  FX_Robot_Kine_Jacb(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_Jacobi* jcb);
FX_BOOL  FX_Robot_Kine_IK(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para);
FX_BOOL  FX_Robot_Kine_IK_NSP(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para);

/////Motion Planning
FX_BOOL  FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_CHAR* OutPutPath);
////Parameters Identification
FX_BOOL FX_Robot_Iden_LoadDyn(FX_INT32L Type,FX_CHAR* path,FX_DOUBLE mass, Vect3 mr, Vect6 I);
FX_VOID FX_XYZABC2Matrix4DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4]);
FX_BOOL FX_Matrix42XYZABCDEG(FX_DOUBLE m[4][4],FX_DOUBLE xyzabc[6]);
#ifdef __cplusplus
}
#endif

#endif
