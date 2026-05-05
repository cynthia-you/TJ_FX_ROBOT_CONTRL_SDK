#include "MAX_Body_Kinematics.h"
#include "FXMatrix.h"
#include "Log.h"

static FX_BOOL KineMAX_Log_Switch = FX_FALSE;

CFxKineMAX::CFxKineMAX()
{
	m_dArm2Pitch = 433.5;
	m_dShoulder = 230.5;

	FX_IdentM44(m_ArmL_Set);
	FX_IdentM44(m_ArmR_Set);

	m_ArmL_Set[1][1] = 0;  m_ArmL_Set[1][2] = 1; m_ArmL_Set[1][3] = m_dShoulder;
	m_ArmL_Set[2][1] = -1; m_ArmL_Set[2][2] = 0;


	m_ArmR_Set[1][1] = 0;  m_ArmR_Set[1][2] = -1; m_ArmR_Set[1][3] = -m_dShoulder;
	m_ArmR_Set[2][1] = 1;  m_ArmR_Set[2][2] = 0;


	FX_IdentM44(m_ArmL_SetB);
	FX_IdentM44(m_ArmR_SetB);

	m_ArmL_SetB[1][1] = 0;  m_ArmL_SetB[1][2] = 1; m_ArmL_SetB[1][3] = 56 ;
	m_ArmL_SetB[2][1] = -1; m_ArmL_SetB[2][2] = 0;


	m_ArmR_SetB[1][1] = 0;  m_ArmR_SetB[1][2] = -1; m_ArmR_SetB[1][3] = -56;
	m_ArmR_SetB[2][1] = 1;  m_ArmR_SetB[2][2] = 0;

}

CFxKineMAX::~CFxKineMAX()
{

}

FX_VOID CFxKineMAX::L0_OnLogSwitch(FX_INT32 log_tag_input)
{
	if (log_tag_input == 1)
	{
		CLog::SetLogOn();
	}
	else
	{
		CLog::SetLogOff();
	}

	KineMAX_Log_Switch = (log_tag_input == 1) ? FX_TRUE : FX_FALSE;
}

FX_VOID CFxKineMAX::L0_OnSetCondition(Vect3 std_Body, Vect3 k_Body, FX_DOUBLE std_L_len, FX_DOUBLE k_L, FX_DOUBLE std_R_len, FX_DOUBLE k_R)
{
	m_std_body[0] = std_Body[0]; m_std_body[1] = -std_Body[1]; m_std_body[2] = std_Body[2];
	m_k_body[0] = k_Body[0]; m_k_body[1] = k_Body[1]; m_k_body[2] = k_Body[2];
	m_std_L_len = std_L_len;
	m_k_L = k_L;
	m_std_R_len = std_R_len;
	m_k_R = k_R;
}

FX_VOID CFxKineMAX::OnKineBLR(Vect3 jv, Matrix4 pgB, Matrix4 pgL, Matrix4 pgR, Matrix3 jcbL, Matrix3 jcbR)
{
	Matrix4 m;
	FX_RotEuler_XYZ(0, jv[1], jv[2], m);
	FX_M44Copy(m, pgB);
	FX_DOUBLE dx;
	FX_DOUBLE dz;
	
	dx = m[0][2];
	dz = m[2][2];

	pgB[2][3] +=  jv[0];


	m[0][3] += dx * m_dArm2Pitch;
	m[2][3] += dz * m_dArm2Pitch + jv[0];

	FX_MMM44(m, m_ArmL_Set, pgL);
	FX_MMM44(m, m_ArmR_Set, pgR);
	{
		Vect3 Vm;
		Vect3 Vr;
		Vect3 V;

		jcbL[0][0] = 0;
		jcbL[1][0] = 0;
		jcbL[2][0] = 1;

		Vm[0] = pgL[0][3];
		Vm[1] = pgL[1][3];
		Vm[2] = pgL[2][3] - jv[0];

		Vr[0] = 0;
		Vr[1] = 1;
		Vr[2] = 0;


		FX_VectCross(Vr, Vm, V);

		jcbL[0][1] = V[0] * 0.001;
		jcbL[1][1] = V[1] * 0.001;
		jcbL[2][1] = V[2] * 0.001;

		Vr[0] = m[0][2];
		Vr[1] = m[1][2];
		Vr[2] = m[2][2];


		FX_VectCross(Vr, Vm, V);

		jcbL[0][2] = V[0] * 0.001;
		jcbL[1][2] = V[1] * 0.001;
		jcbL[2][2] = V[2] * 0.001;
	}


	{
		Vect3 Vm;
		Vect3 Vr;
		Vect3 V;

		jcbR[0][0] = 0;
		jcbR[1][0] = 0;
		jcbR[2][0] = 1;

		Vm[0] = pgR[0][3];
		Vm[1] = pgR[1][3];
		Vm[2] = pgR[2][3] - jv[0];

		Vr[0] = 0;
		Vr[1] = 1;
		Vr[2] = 0;


		FX_VectCross(Vr, Vm, V);

		jcbR[0][1] = V[0] * 0.001;
		jcbR[1][1] = V[1] * 0.001;
		jcbR[2][1] = V[2] * 0.001;

		Vr[0] = m[0][2];
		Vr[1] = m[1][2];
		Vr[2] = m[2][2];


		FX_VectCross(Vr, Vm, V);

		jcbR[0][2] = V[0] * 0.001;
		jcbR[1][2] = V[1] * 0.001;
		jcbR[2][2] = V[2] * 0.001;
	}

}

FX_VOID CFxKineMAX::OnKineLR(Vect3 jv, Matrix4 pgL, Matrix4 pgR, Matrix3 jcbL, Matrix3 jcbR)
{
	Matrix4 m;
	FX_RotEuler_XYZ(0, jv[1], jv[2], m);
	FX_DOUBLE dx;
	FX_DOUBLE dz;

	dx = m[0][2];
	dz = m[2][2];

	m[0][3] += dx * m_dArm2Pitch;
	m[2][3] += dz * m_dArm2Pitch + jv[0];

	FX_MMM44(m, m_ArmL_Set, pgL);
	FX_MMM44(m, m_ArmR_Set, pgR);
	{
		Vect3 Vm;
		Vect3 Vr;
		Vect3 V;

		jcbL[0][0] = 0;
		jcbL[1][0] = 0;
		jcbL[2][0] = 1;

		Vm[0] = pgL[0][3];
		Vm[1] = pgL[1][3];
		Vm[2] = pgL[2][3] - jv[0];

		Vr[0] = 0;
		Vr[1] = 1;
		Vr[2] = 0;


		FX_VectCross(Vr, Vm, V);

		jcbL[0][1] = V[0] * 0.001;
		jcbL[1][1] = V[1] * 0.001;
		jcbL[2][1] = V[2] * 0.001;

		Vr[0] = m[0][2];
		Vr[1] = m[1][2];
		Vr[2] = m[2][2];


		FX_VectCross(Vr, Vm, V);

		jcbL[0][2] = V[0] * 0.001;
		jcbL[1][2] = V[1] * 0.001;
		jcbL[2][2] = V[2] * 0.001;
	}


	{
		Vect3 Vm;
		Vect3 Vr;
		Vect3 V;

		jcbR[0][0] = 0;
		jcbR[1][0] = 0;
		jcbR[2][0] = 1;

		Vm[0] = pgR[0][3];
		Vm[1] = pgR[1][3];
		Vm[2] = pgR[2][3] - jv[0];

		Vr[0] = 0;
		Vr[1] = 1;
		Vr[2] = 0;


		FX_VectCross(Vr, Vm, V);

		jcbR[0][1] = V[0] * 0.001;
		jcbR[1][1] = V[1] * 0.001;
		jcbR[2][1] = V[2] * 0.001;

		Vr[0] = m[0][2];
		Vr[1] = m[1][2];
		Vr[2] = m[2][2];


		FX_VectCross(Vr, Vm, V);

		jcbR[0][2] = V[0] * 0.001;
		jcbR[1][2] = V[1] * 0.001;
		jcbR[2][2] = V[2] * 0.001;
	}
	


}

FX_VOID CFxKineMAX::L0_OnKineLR(Vect3 jv, Matrix4 pgL, Matrix4 pgR)
{
	Matrix4 m;
	FX_RotEuler_XYZ(0, -jv[1], jv[2], m);
	FX_DOUBLE dx;
	FX_DOUBLE dz;

	dx = m[0][2];
	dz = m[2][2];

	m[0][3] += dx * m_dArm2Pitch;
	m[2][3] += dz * m_dArm2Pitch + jv[0];

	FX_MMM44(m, m_ArmL_SetB, pgL);
	FX_MMM44(m, m_ArmR_SetB, pgR);
}

FX_DOUBLE CFxKineMAX::CalLen(Matrix4 ps, Vect3 t, Vect3 ret_vct)
{
	FX_DOUBLE a, b, c;
	a = t[0] - ps[0][3];
	b = t[1] - ps[1][3];
	c = t[2] - ps[2][3];

	ret_vct[0] = a;
	ret_vct[1] = b;
	ret_vct[2] = c;
	return  FX_VectNorm_RetLen(ret_vct);
}

FX_DOUBLE  CFxKineMAX::CalForce(FX_DOUBLE k, FX_DOUBLE stdlen, Matrix4 ps, Vect3 pos, Vect3 ret_vct)
{
	FX_DOUBLE td = (CalLen(ps, pos, ret_vct) - stdlen) * k;
	ret_vct[0] *= td;
	ret_vct[1] *= td;
	ret_vct[2] *= td;
	return td;
}

FX_VOID CFxKineMAX::CalFT(FX_DOUBLE k, FX_DOUBLE stdlen, Matrix4 pb, Matrix4 ps, Vect3 pos, Vect3 ret_tf)
{
	Vect3 ret_vct;
	CalForce(k, stdlen, ps, pos, ret_vct);
	Vect3 vct;
	vct[0] = (ps[0][3] - pb[0][3]) * 0.001;
	vct[1] = (ps[1][3] - pb[1][3]) * 0.001;
	vct[2] = (ps[2][3] - pb[2][3]) * 0.001;

	Vect3 tq;
	FX_VectCross(vct, ret_vct, tq);

	ret_tf[0] = ret_vct[2];
	ret_tf[1] = tq[1];


	Vect3 zdir;

	zdir[0] = pb[0][2];
	zdir[1] = pb[1][2];
	zdir[2] = pb[2][2];

	ret_tf[2] = FX_VectDot3(tq, zdir);
}

FX_VOID CFxKineMAX::L0_OnCalBody_withref(Vect3 refjv, Vect3 tpos1, Vect3 tpos2, Vect3 ret_pos)
{
	FX_DOUBLE rfact = 0.1;
	FX_DOUBLE len1 = m_std_L_len;
	FX_DOUBLE len2 = m_std_R_len;

	FX_DOUBLE k1 = m_k_L;
	FX_DOUBLE k2 = m_k_R;

	FX_DOUBLE Jzpos2[3];
	Jzpos2[0] = refjv[0];
	Jzpos2[1] = -refjv[1];
	Jzpos2[2] = refjv[2];
	Matrix4 pg1, pg2, bm;
	Matrix3 jcb1, jcb2;
	CFxKineMAX bk;
	bk.OnKineBLR(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

	Vect3 FN1;
	CalFT(k1, len1, bm, pg1, tpos1, FN1);


	Vect3 FN2;
	CalFT(k2, len2, bm, pg2, tpos2, FN2);

	Vect3 FNB;

	FNB[0] = (m_std_body[0] - Jzpos2[0]) * m_k_body[0];
	FNB[1] = (m_std_body[1] - Jzpos2[1]) * m_k_body[1];
	FNB[2] = (m_std_body[2] - Jzpos2[2]) * m_k_body[2];

	FX_DOUBLE Dx = (FN1[0] + FN2[0] + FNB[0]) / m_k_body[0];
	FX_DOUBLE Dy = (FN1[1] + FN2[1] + FNB[1]) / m_k_body[1];
	FX_DOUBLE Dz = (FN1[2] + FN2[2] + FNB[2]) / m_k_body[2];

	Jzpos2[0] += Dx * rfact;
	Jzpos2[1] += Dy * rfact;
	Jzpos2[2] += Dz * rfact;

	while (FX_Fabs(Dx) > 0.0001 || FX_Fabs(Dy) > 0.0001 || FX_Fabs(Dz) > 0.0001)
	{
		bk.OnKineBLR(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

		Vect3 FN1;
		CalFT(k1, len1, bm, pg1, tpos1, FN1);


		Vect3 FN2;
		CalFT(k2, len2, bm, pg2, tpos2, FN2);


		Vect3 FNB;

		FNB[0] = (m_std_body[0] - Jzpos2[0]) * m_k_body[0];
		FNB[1] = (m_std_body[1] - Jzpos2[1]) * m_k_body[1];
		FNB[2] = (m_std_body[2] - Jzpos2[2]) * m_k_body[2];

		Dx = (FN1[0] + FN2[0] + FNB[0]) / m_k_body[0];
		Dy = (FN1[1] + FN2[1] + FNB[1]) / m_k_body[1];
		Dz = (FN1[2] + FN2[2] + FNB[2]) / m_k_body[2];
		Jzpos2[0] += Dx * rfact;
		Jzpos2[1] += Dy * rfact;
		Jzpos2[2] += Dz * rfact;


	
	}


	ret_pos[0] = Jzpos2[0];
	ret_pos[1] = -Jzpos2[1];
	ret_pos[2] = Jzpos2[2];
}

FX_VOID CFxKineMAX::L0_OnCalBody(Vect3 tpos1, Vect3 tpos2, Vect3 ret_pos)
{

	FX_DOUBLE rfact = 0.1;
	FX_DOUBLE len1 = m_std_L_len;
	FX_DOUBLE len2 = m_std_R_len;

	FX_DOUBLE k1 = m_k_L;
	FX_DOUBLE k2 = m_k_R;

	FX_DOUBLE Jzpos2[3];
	Jzpos2[0] = m_std_body[0];
	Jzpos2[1] = m_std_body[1];
	Jzpos2[2] = m_std_body[2];
	Matrix4 pg1, pg2, bm;
	Matrix3 jcb1, jcb2;
	CFxKineMAX bk;
	bk.OnKineBLR(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

	Vect3 FN1;
	CalFT(k1, len1, bm, pg1, tpos1, FN1);


	Vect3 FN2;
	CalFT(k2, len2, bm, pg2, tpos2, FN2);

	Vect3 FNT;
	FNT[0] = (FN1[0] + FN2[0]);
	FNT[1] = (FN1[1] + FN2[1]);
	FNT[2] = (FN1[2] + FN2[2]);

	FX_DOUBLE Dx = (FN1[0] + FN2[0] + FNT[0]) / m_k_body[0];
	FX_DOUBLE Dy = (FN1[1] + FN2[1] + FNT[1]) / m_k_body[1];
	FX_DOUBLE Dz = (FN1[2] + FN2[2] + FNT[2]) / m_k_body[2];
	Jzpos2[0] += Dx*rfact;
	Jzpos2[1] += Dy*rfact;
	Jzpos2[2] += Dz*rfact;

	while (FX_Fabs(Dx) > 0.0001 || FX_Fabs(Dy) > 0.0001 || FX_Fabs(Dz) > 0.0001)
	{
		bk.OnKineBLR(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

		Vect3 FN1;
		CalFT(k1, len1, bm, pg1, tpos1, FN1);


		Vect3 FN2;
		CalFT(k2, len2, bm, pg2, tpos2, FN2);

		
		Vect3 FNB;

		FNB[0] = (m_std_body[0] - Jzpos2[0]) * m_k_body[0];
		FNB[1] = (m_std_body[1] - Jzpos2[1]) * m_k_body[1];
		FNB[2] = (m_std_body[2] - Jzpos2[2]) * m_k_body[2];

		Dx = (FN1[0] + FN2[0] + FNB[0]) / m_k_body[0];
		Dy = (FN1[1] + FN2[1] + FNB[1]) / m_k_body[1];
		Dz = (FN1[2] + FN2[2] + FNB[2]) / m_k_body[2];
		Jzpos2[0] += Dx*rfact;
		Jzpos2[1] += Dy*rfact;
		Jzpos2[2] += Dz*rfact;

	}

	ret_pos[0] = Jzpos2[0];
	ret_pos[1] = -Jzpos2[1];
	ret_pos[2] = Jzpos2[2];
}
