#include "ICBase.h"
#include "stdio.h"
#include "stdlib.h"
#include "../FxCfg/CfgBase.h"
#include "math.h"
#include "FXMatrix.h"


CICBase::CICBase()
{
	m_Cords = NULL;
	// m_ICs = NULL;
	m_CCs = NULL;
	m_CordsNum = 0;
	m_ICsNum = 0;
	m_CCsNum = 0;

	for (long i = 0; i < 100; i++)
	{
		m_Cord_Input_Arrange[i] = NULL;
		m_Cord_Link_Arrange_Pre[i] = NULL;
		m_Cord_Link_Arrange_Nex[i] = NULL;
	}
	m_Cord_input_Num = 0;
	m_Cord_Link_Num = 0;

	m_pAgvCord = NULL;
}

CICBase::~CICBase()
{
	FX_OnEmtpy();
}

long CICBase::FX_OnDefInputArrange(char *cord_name, long input_serial)
{
	if (input_serial < 0 || input_serial >= 100)
	{
		return -1;
	}
	if (input_serial != m_Cord_input_Num)
	{
		return -2;
	}
	if (m_Cord_Input_Arrange[input_serial] != NULL)
	{
		return -3;
	}

	if (m_Cord_input_Num >= 99)
	{
		return -4;
	}

	pCordCell t = FX_SearchCord(cord_name);
	if (t == NULL)
	{
		return -5;
	}

	m_Cord_Input_Arrange[input_serial] = t;
	m_Cord_input_Num++;

	return m_Cord_input_Num;
}

long CICBase::FX_OnDefCalLink(char *pre_name, char *nex_name, long link_serial)
{
	if (link_serial < 0 || link_serial >= 100)
	{
		return -1;
	}

	if (link_serial != m_Cord_Link_Num)
	{
		return -2;
	}
	if (m_Cord_Link_Arrange_Pre[link_serial] != NULL)
	{
		return -3;
	}
	if (m_Cord_Link_Arrange_Nex[link_serial] != NULL)
	{
		return -3;
	}

	if (m_Cord_Link_Num >= 99)
	{
		return -4;
	}

	pCordCell t = FX_SearchCord(pre_name);
	if (t == NULL)
	{
		return -5;
	}

	pCordCell t2 = FX_SearchCord(nex_name);
	if (t2 == NULL)
	{
		return -5;
	}

	m_Cord_Link_Arrange_Pre[link_serial] = t;
	m_Cord_Link_Arrange_Nex[link_serial] = t2;
	m_Cord_Link_Num++;

	return m_Cord_Link_Num;
}

bool CICBase::FX_OnUpdateLink()
{
	for (long i = 0; i < m_Cord_Link_Num; i++)
	{
		FX_PGMult(m_Cord_Link_Arrange_Pre[i]->m_RunW, m_Cord_Link_Arrange_Nex[i]->m_RunL, m_Cord_Link_Arrange_Nex[i]->m_RunW);
	}
	return true;
}

pConvexCell CICBase::FX_SearchConvex(char *name)
{
	char tmp[32];
	memset(tmp, 0, 32);
	long namelen = strlen(name);
	if (namelen < 1)
	{
		return NULL;
	}
	long i;
	for (i = 0; i < 31 && i < namelen; i++)
	{
		tmp[i] = name[i];
	}
	for (i = 0; i < m_CCsNum; i++)
	{
		if (strcmp(m_CCs[i].ConvexName, tmp) == 0)
		{
			return &m_CCs[i];
		}
	}
	return NULL;
}

pCordCell CICBase::FX_SearchCord(char *name)
{
	char tmp[32];
	memset(tmp, 0, 32);
	long namelen = strlen(name);
	if (namelen < 1)
	{
		return NULL;
	}
	long i;
	for (i = 0; i < 31 && i < namelen; i++)
	{
		tmp[i] = name[i];
	}
	for (i = 0; i < m_CordsNum; i++)
	{
		if (strcmp(m_Cords[i].m_CordName, tmp) == 0)
		{
			return &m_Cords[i];
		}
	}
	return NULL;
}

bool CICBase::FX_OnSetAGV(double pg[4][4])
{
	if (m_pAgvCord == NULL)
	{
		m_pAgvCord = FX_SearchCord((char *)"AGV");
		if (m_pAgvCord == NULL)
		{
			return false;
		}

		long j;
		for (j = 0; j < 3; j++)
		{
			m_pAgvCord->m_Set[3][j] = 0;
			m_pAgvCord->m_RunW[3][j] = 0;
			m_pAgvCord->m_RunL[3][j] = 0;
		}

		m_pAgvCord->m_Set[3][3] = 1;
		m_pAgvCord->m_Set[3][3] = 1;
		m_pAgvCord->m_RunW[3][3] = 1;
	}
	long i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			m_pAgvCord->m_Set[i][j] = pg[i][j];
			m_pAgvCord->m_RunW[i][j] = pg[i][j];
			m_pAgvCord->m_RunL[i][j] = pg[i][j];
		}
	}
	return true;
}

bool CICBase::FX_OnUpdateConvex()
{
	long i;
	long j;
	for (i = 0; i < m_CCsNum; i++)
	{
		for (j = 0; j < m_CCs[i].PointNum; j++)
		{
			FX_PGPointMap(m_CCs[i].pCord->m_RunW, m_CCs[i].PointsDef[j], m_CCs[i].PointsInW[j]);
		}
	}

	return true;
}

void CICBase::FX_OnInitV(long anum, double a[20][3], long bnum, double b[20][3], double v[3])
{
	double t1[3] = {0};
	double t2[3] = {0};

	long i;
	for (i = 0; i < anum; i++)
	{
		t1[0] += a[i][0];
		t1[1] += a[i][1];
		t1[2] += a[i][2];
	}
	for (i = 0; i < bnum; i++)
	{
		t2[0] += b[i][0];
		t2[1] += b[i][1];
		t2[2] += b[i][2];
	}

	double danum = anum;
	t1[0] /= danum;
	t1[1] /= danum;
	t1[2] /= danum;

	double dbnum = bnum;
	t2[0] /= dbnum;
	t2[1] /= dbnum;
	t2[2] /= dbnum;

	v[0] = t1[0] - t2[0];
	v[1] = t1[1] - t2[1];
	v[2] = t1[2] - t2[2];
	FX_VectNorm(v);
}

void CICBase::FX_OnProjectMinMax(long num, double a[20][3], double v[3], double minmax[2])
{
	double d = a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2];
	minmax[0] = d;
	minmax[1] = d;
	for (long i = 1; i < num; i++)
	{
		d = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2];
		if (d < minmax[0])
		{
			minmax[0] = d;
		}
		if (d > minmax[1])
		{
			minmax[1] = d;
		}
	}
}
double CICBase::FX_OnCalSpan(long anum, double a[20][3], long bnum, double b[20][3], double v[3])
{
	double mnmx1[2];
	double mnmx2[2];
	FX_OnProjectMinMax(anum, a, v, mnmx1);
	FX_OnProjectMinMax(bnum, b, v, mnmx2);

	double a1 = mnmx1[0];
	double b1 = mnmx1[1];
	double a2 = mnmx2[0];
	double b2 = mnmx2[1];
	if (a1 >= b2)
	{
		return a1 - b2;
	}
	if (a2 >= b1)
	{
		return a2 - b1;
	}

	if (b1 > b2)
	{
		double c = b2;
		b2 = b1;
		b1 = c;
		c = a2;
		a2 = a1;
		a1 = c;
	}

	if (a2 > a1)
	{
		return a2 - b1;
	}
	else
	{
		double t1 = a2 - b1;
		double t2 = a1 - b2;
		if (t1 > t2)
		{
			return t1;
		}
		return t2;
	}
}

void CICBase::FX_OnGetCrossXYDir(double z[3], double d, double ret_x[3], double ret_y[3])
{
	double tx = FX_Fabs(z[0]);
	double ty = FX_Fabs(z[1]);
	double tz = FX_Fabs(z[2]);

	ret_x[0] = 0;
	ret_x[1] = 0;
	ret_x[2] = 0;
	if (tx <= ty && tx <= tz)
	{
		ret_x[0] = 1;
	}
	else if (ty <= tx && ty <= tz)
	{
		ret_x[1] = 1;
	}
	else
	{
		ret_x[2] = 1;
	}

	FX_VectCross(z, ret_x, ret_y);
	FX_VectNorm(ret_y);
	FX_VectCross(ret_y, z, ret_x);
	FX_VectNorm(ret_x);
	ret_y[0] *= d;
	ret_y[1] *= d;
	ret_y[2] *= d;

	ret_x[0] *= d;
	ret_x[1] *= d;
	ret_x[2] *= d;

	ret_x[0] += z[0];
	ret_x[1] += z[1];
	ret_x[2] += z[2];

	ret_y[0] += z[0];
	ret_y[1] += z[1];
	ret_y[2] += z[2];

	FX_VectNorm(ret_x);
	FX_VectNorm(ret_y);
}

double CICBase::FX_OnAdjOneStep(long anum, double a[20][3], long bnum, double b[20][3], double v[3])
{
	double d1 = FX_OnCalSpan(anum, a, bnum, b, v);
	double va[3];
	double vb[3];
	double vc[3];
	double vd[3];
	FX_OnGetCrossXYDir(v, 0.01, va, vb);

	vc[0] = 2 * v[0] - va[0];
	vc[1] = 2 * v[1] - va[1];
	vc[2] = 2 * v[2] - va[2];

	vd[0] = 2 * v[0] - vb[0];
	vd[1] = 2 * v[1] - vb[1];
	vd[2] = 2 * v[2] - vb[2];

	FX_VectNorm(vc);
	FX_VectNorm(vd);

	double da = FX_OnCalSpan(anum, a, bnum, b, va);
	double db = FX_OnCalSpan(anum, a, bnum, b, vb);
	double dc = FX_OnCalSpan(anum, a, bnum, b, vc);
	double dd = FX_OnCalSpan(anum, a, bnum, b, vd);

	if (da > d1)
	{
		d1 = da;
		v[0] = va[0];
		v[1] = va[1];
		v[2] = va[2];
	}

	if (db > d1)
	{
		d1 = db;
		v[0] = vb[0];
		v[1] = vb[1];
		v[2] = vb[2];
	}

	if (dc > d1)
	{
		d1 = dc;
		v[0] = vc[0];
		v[1] = vc[1];
		v[2] = vc[2];
	}

	if (dd > d1)
	{
		d1 = dd;
		v[0] = vd[0];
		v[1] = vd[1];
		v[2] = vd[2];
	}
	return d1;
}

double CICBase::FX_OnCalMaxSpan(long anum, double a[20][3], long bnum, double b[20][3], double v[3])
{

	double spc = FX_OnAdjOneStep(anum, a, bnum, b, v);
	double tmp = FX_OnAdjOneStep(anum, a, bnum, b, v);
	while (tmp > spc)
	{
		spc = tmp;
		tmp = FX_OnAdjOneStep(anum, a, bnum, b, v);
	}
	return spc;
}

long CICBase::FX_OnGetInterfCellNum()
{
	return m_ICsNum;
}

bool CICBase::FX_OnGetInterfSpan(long serial, double &ret_span)
{
	if (serial < 0 || serial >= m_ICsNum)
	{
		return false;
	}
	if (m_ICs[serial].SetTag == false)
	{
		return false;
	}

	ret_span = m_ICs[serial].MinSpan;

	return true;
}
bool CICBase::FX_OnGetInterfInfo(long serial, double name_a[32], double name_b[32], double &ret_span)
{
	if (serial < 0 || serial >= m_ICsNum)
	{
		return false;
	}
	if (m_ICs[serial].SetTag == false)
	{
		return false;
	}

	ret_span = m_ICs[serial].MinSpan;

	memcpy(name_a, m_ICs[serial].pCC_A->ConvexName, 32);
	memcpy(name_b, m_ICs[serial].pCC_B->ConvexName, 32);

	return true;
}

void CICBase::FX_OnGetCoM(double mcp_average[3])
{
	double mcp[3] = {0.0};
	for (int i = 0; i < m_CordsNum; i++)
	{
		mcp[0] += m_Cords[i].m_McpW[0];
		mcp[1] += m_Cords[i].m_McpW[1];
		mcp[2] += m_Cords[i].m_McpW[2];
	}
	mcp_average[0] = mcp[0] / m_CordsNum;
	mcp_average[1] = mcp[1] / m_CordsNum;
	mcp_average[2] = mcp[2] / m_CordsNum;
}

void CICBase::FX_OnGetCordsNum(long &cords_num)
{
	cords_num = m_CordsNum;
}

void CICBase::FX_OnGetM(double *m)
{
	for (int i = 0; i < m_CordsNum; i++)
	{
		m[i] = m_Cords[i].m_M;
	}
}

void CICBase::FX_OnGetMinSpan(double *MinSpan)
{
	for (long i = 0; i < m_ICsNum; i++)
	{
		MinSpan[i] = m_ICs[i].MinSpan;
	}
}

bool CICBase::FX_OnIsInterf(double *inteThsh)
{
	for (long i = 0; i < m_ICsNum; i++)
	{
		if(m_ICs[i].MinSpan < fabs(inteThsh[i]))
		{
			return true;
		}
	}
	return false;
}



bool CICBase::FX_OnCalInterf()
{
	long i;
	for (i = 0; i < m_ICsNum; i++)
	{
		if (m_ICs[i].SetTag == false)
		{
			FX_OnInitV(m_ICs[i].pCC_A->PointNum, m_ICs[i].pCC_A->PointsInW,
					   m_ICs[i].pCC_B->PointNum, m_ICs[i].pCC_B->PointsInW, m_ICs[i].MapDir);
			m_ICs[i].SetTag = true;
			m_ICs[i].SubCnt = 1;
		}
		m_ICs[i].SubCnt--;
		if (m_ICs[i].SubCnt <= 0)
		{
			m_ICs[i].MinSpan = FX_OnCalMaxSpan(m_ICs[i].pCC_A->PointNum, m_ICs[i].pCC_A->PointsInW,
											   m_ICs[i].pCC_B->PointNum, m_ICs[i].pCC_B->PointsInW, m_ICs[i].MapDir);
			if (m_ICs[i].MinSpan > 200)
			{
				m_ICs[i].SubCnt = m_ICs[i].MinSpan / 10;
			}
			else
			{
				m_ICs[i].SubCnt = 1;
			}
		}
		// printf("Interf[%ld] span:%lf\n", i, m_ICs[i].MinSpan);
	}
	return true;
}

void FPCORD(FILE *fp, double m[4][4])
{
	if (fp == NULL)
	{
		return;
	}
	double d1 = 30;
	double d2 = 10;
	double d3 = 50;
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3], m[1][3], m[2][3]);
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3] + m[0][0] * d1, m[1][3] + m[1][0] * d1, m[2][3] + m[2][0] * d1);
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3], m[1][3], m[2][3]);
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3] + m[0][1] * d2, m[1][3] + m[1][1] * d2, m[2][3] + m[2][1] * d2);
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3], m[1][3], m[2][3]);
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3] + m[0][2] * d3, m[1][3] + m[1][2] * d3, m[2][3] + m[2][2] * d3);
	fprintf(fp, "X%lf Y%lf Z%lf\n", m[0][3], m[1][3], m[2][3]);
	printf("---------------------\n");
	for (long i = 0; i < 3; i++)
	{
		printf("%lf	%lf	%lf		|	%lf\n", m[i][0], m[i][1], m[i][2], m[i][3]);
	}
	printf("---------------------\n");
}

void CICBase::FX_DebugPrintCord(char *path)
{
	FILE *fp = fopen(path, "wb");
	if (fp != NULL)
	{
		for (long i = 0; i < m_CordsNum; i++)
		{
			printf("%s\n", m_Cords[i].m_CordName);

			for (long j = 0; j < 3; j++)
			{
				printf("%lf	%lf	%lf		|	%lf\n", m_Cords[i].m_Set[j][0], m_Cords[i].m_Set[j][1], m_Cords[i].m_Set[j][2], m_Cords[i].m_Set[j][3]);
			}
			FPCORD(fp, m_Cords[i].m_RunW);
		}
		fclose(fp);
	}
}

void FPConvex(FILE *fp, pConvexCell cell)
{
	if (fp == NULL)
	{
		return;
	}

	long num = cell->PointNum;
	long i, j;
	for (i = 0; i < num; i++)
	{
		for (j = i + 1; j < num; j++)
		{
			fprintf(fp, "X%lf Y%lf Z%lf\n", cell->PointsInW[i][0], cell->PointsInW[i][1], cell->PointsInW[i][2]);
			fprintf(fp, "X%lf Y%lf Z%lf\n", cell->PointsInW[j][0], cell->PointsInW[j][1], cell->PointsInW[j][2]);
		}
	}
}

void CICBase::FX_DebugPrintConvex(char *path)
{
	FILE *fp = fopen(path, "wb");
	if (fp != NULL)
	{
		for (long i = 0; i < m_CCsNum; i++)
		{
			FPConvex(fp, &m_CCs[i]);
		}
		fclose(fp);
	}
}

bool CICBase::FX_OnUpdateCord(long cord_num, double *input)
{
	double xyz[3] = {0};
	if (cord_num != m_Cord_input_Num)
	{
		return false;
	}
	for (long i = 0; i < cord_num; i++)
	{
		if (m_Cord_Input_Arrange[i]->m_Type == 0)
		{
			FX_MatrixRotZ(m_Cord_Input_Arrange[i]->m_Set, input[i], m_Cord_Input_Arrange[i]->m_RunL);
		}
		else if (m_Cord_Input_Arrange[i]->m_Type == 10)
		{
			xyz[2] = input[i];
			FX_MatrixMoveXYZ(m_Cord_Input_Arrange[i]->m_Set, xyz, m_Cord_Input_Arrange[i]->m_RunL);
		}
		else
		{
			xyz[0] = 0;
			xyz[1] = 0;
			xyz[2] = 0;
			FX_MatrixMoveXYZ(m_Cord_Input_Arrange[i]->m_Set, xyz, m_Cord_Input_Arrange[i]->m_RunL);
		}
		FX_PGPointMap(m_Cord_Input_Arrange[i]->m_RunL, m_Cord_Input_Arrange[i]->m_Mcp, m_Cord_Input_Arrange[i]->m_McpW);
	}
	return FX_OnUpdateLink();
}

long CICBase::FX_OnLoadICDef(char *ICDef)
{
	CCfgFILE ic_cfg;
	if (ic_cfg.FX_OnGetOpen(ICDef) == false)
	{
		ic_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long ic_num;
	if (ic_cfg.FX_OnGetLong(ic_num) == false)
	{
		ic_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}

	if (FX_OnInitICsBuf(ic_num) == false)
	{

		ic_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long i;

	for (i = 0; i < ic_num; i++)
	{
		char name_a[32];
		char name_b[32];
		if (ic_cfg.FX_OnGetStr(name_a) == false)
		{
			ic_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
		if (ic_cfg.FX_OnGetStr(name_b) == false)
		{
			ic_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		m_ICs[i].pCC_A = FX_SearchConvex(name_a);
		m_ICs[i].pCC_B = FX_SearchConvex(name_b);

		if (m_ICs[i].pCC_A == NULL || m_ICs[i].pCC_B == NULL)
		{

			ic_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		m_ICs[i].SetTag = false;
	}

	ic_cfg.FX_OnClose();
	return true;
}

long CICBase::FX_OnLoadConvex(char *ConvexDef)
{
	CCfgFILE convex_cfg;
	if (convex_cfg.FX_OnGetOpen(ConvexDef) == false)
	{
		convex_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long convex_num;
	if (convex_cfg.FX_OnGetLong(convex_num) == false)
	{
		convex_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}

	if (FX_OnInitCCsBuf(convex_num) == false)
	{
		convex_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long i;
	long j;

	for (i = 0; i < convex_num; i++)
	{
		if (convex_cfg.FX_OnGetStr(m_CCs[i].ConvexName) == false)
		{
			convex_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		char cord_name[32];
		if (convex_cfg.FX_OnGetStr(cord_name) == false)
		{
			convex_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		m_CCs[i].pCord = FX_SearchCord(cord_name);
		if (m_CCs[i].pCord == NULL)
		{
			convex_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		if (convex_cfg.FX_OnGetLong(m_CCs[i].PointNum) == false)
		{
			convex_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
		if (m_CCs[i].PointNum < 4 || m_CCs[i].PointNum > 20)
		{
			convex_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
		for (j = 0; j < m_CCs[i].PointNum; j++)
		{

			if (convex_cfg.FX_OnGetVect3(m_CCs[i].PointsDef[j]) == false)
			{
				convex_cfg.FX_OnClose();
				FX_OnEmtpy();
				return false;
			}
		}
	}
	convex_cfg.FX_OnClose();
	return true;
}

bool CICBase::FX_OnLoadCalInput(char *inputDef)
{
	CCfgFILE calinput_cfg;
	if (calinput_cfg.FX_OnGetOpen(inputDef) == false)
	{
		calinput_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long input_num;
	if (calinput_cfg.FX_OnGetLong(input_num) == false)
	{
		calinput_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long i;

	for (i = 0; i < input_num; i++)
	{
		char input_name[32];
		if (calinput_cfg.FX_OnGetStr(input_name) == false)
		{
			calinput_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		if (FX_OnDefInputArrange(input_name, i) <= 0)
		{

			calinput_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
	}

	calinput_cfg.FX_OnClose();
	return true;
}

bool CICBase::FX_OnLoadCalLink(char *CalLinkDef)
{
	CCfgFILE link_cfg;
	if (link_cfg.FX_OnGetOpen(CalLinkDef) == false)
	{
		link_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long link_num;

	if (link_cfg.FX_OnGetLong(link_num) == false)
	{
		link_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long i;
	for (i = 0; i < link_num; i++)
	{
		char name_pre[32];
		char name_nex[32];
		if (link_cfg.FX_OnGetStr(name_pre) == false)
		{
			link_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
		if (link_cfg.FX_OnGetStr(name_nex) == false)
		{
			link_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}

		if (FX_OnDefCalLink(name_pre, name_nex, i) <= 0)
		{
			link_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
	}

	link_cfg.FX_OnClose();
	return true;
}

bool CICBase::FX_OnLoadCord(char *CordDef)
{
	CCfgFILE cord_cfg;
	if (cord_cfg.FX_OnGetOpen(CordDef) == false)
	{
		cord_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long cord_num;
	if (cord_cfg.FX_OnGetLong(cord_num) == false)
	{
		cord_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}

	if (FX_OnInitCordBuf(cord_num) == false)
	{

		cord_cfg.FX_OnClose();
		FX_OnEmtpy();
		return false;
	}
	long i;

	for (i = 0; i < cord_num; i++)
	{
		// char cord_name[32];
		// char father_name[32];
		double mtmp[3][3];
		if (cord_cfg.FX_OnGetStr(m_Cords[i].m_CordName) == false)
		{
			cord_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
		if (cord_cfg.FX_OnGetStr(m_Cords[i].m_FatherName) == false)
		{
			cord_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
		if (cord_cfg.FX_OnGetMat3(mtmp) == false)
		{
			cord_cfg.FX_OnClose();
			FX_OnEmtpy();
			return false;
		}
					
		
		double vx[3];
		double vy[3];
		double vz[3];
		double pp[3];
		vx[0] = mtmp[0][0];
		vx[1] = mtmp[0][1];
		vx[2] = mtmp[0][2];
		vz[0] = mtmp[1][0];
		vz[1] = mtmp[1][1];
		vz[2] = mtmp[1][2];
		pp[0] = mtmp[2][0];
		pp[1] = mtmp[2][1];
		pp[2] = mtmp[2][2];

		FX_VectNorm(vx);
		FX_VectNorm(vz);
		FX_VectCross(vz, vx, vy);
		FX_VectNorm(vy);
		FX_VectCross(vy, vz, vx);
		FX_VectNorm(vx);

		m_Cords[i].m_Set[0][0] = vx[0];
		m_Cords[i].m_Set[1][0] = vx[1];
		m_Cords[i].m_Set[2][0] = vx[2];
		m_Cords[i].m_Set[0][1] = vy[0];
		m_Cords[i].m_Set[1][1] = vy[1];
		m_Cords[i].m_Set[2][1] = vy[2];
		m_Cords[i].m_Set[0][2] = vz[0];
		m_Cords[i].m_Set[1][2] = vz[1];
		m_Cords[i].m_Set[2][2] = vz[2];
		m_Cords[i].m_Set[0][3] = pp[0];
		m_Cords[i].m_Set[1][3] = pp[1];
		m_Cords[i].m_Set[2][3] = pp[2];
		m_Cords[i].m_Set[3][0] = 0;
		m_Cords[i].m_Set[3][1] = 0;
		m_Cords[i].m_Set[3][2] = 0;
		m_Cords[i].m_Set[3][3] = 1;

		m_Cords[i].m_RunL[0][0] = vx[0];
		m_Cords[i].m_RunL[1][0] = vx[1];
		m_Cords[i].m_RunL[2][0] = vx[2];
		m_Cords[i].m_RunL[0][1] = vy[0];
		m_Cords[i].m_RunL[1][1] = vy[1];
		m_Cords[i].m_RunL[2][1] = vy[2];
		m_Cords[i].m_RunL[0][2] = vz[0];
		m_Cords[i].m_RunL[1][2] = vz[1];
		m_Cords[i].m_RunL[2][2] = vz[2];
		m_Cords[i].m_RunL[0][3] = pp[0];
		m_Cords[i].m_RunL[1][3] = pp[1];
		m_Cords[i].m_RunL[2][3] = pp[2];
		m_Cords[i].m_RunL[3][0] = 0;
		m_Cords[i].m_RunL[3][1] = 0;
		m_Cords[i].m_RunL[3][2] = 0;
		m_Cords[i].m_RunL[3][3] = 1;

		m_Cords[i].m_RunW[0][0] = vx[0];
		m_Cords[i].m_RunW[1][0] = vx[1];
		m_Cords[i].m_RunW[2][0] = vx[2];
		m_Cords[i].m_RunW[0][1] = vy[0];
		m_Cords[i].m_RunW[1][1] = vy[1];
		m_Cords[i].m_RunW[2][1] = vy[2];
		m_Cords[i].m_RunW[0][2] = vz[0];
		m_Cords[i].m_RunW[1][2] = vz[1];
		m_Cords[i].m_RunW[2][2] = vz[2];
		m_Cords[i].m_RunW[0][3] = pp[0];
		m_Cords[i].m_RunW[1][3] = pp[1];
		m_Cords[i].m_RunW[2][3] = pp[2];
		m_Cords[i].m_RunW[3][0] = 0;
		m_Cords[i].m_RunW[3][1] = 0;
		m_Cords[i].m_RunW[3][2] = 0;
		m_Cords[i].m_RunW[3][3] = 1;
	}
	cord_cfg.FX_OnClose();
	return true;
}


bool CICBase::FX_OnLoadCfg(char *CordDef, char *CalLinkDef, char *InputDef, char *ConvexDef, char *ICDef)
{
	FX_OnEmtpy();
	if (FX_OnLoadCord(CordDef) == false)
	{
		FX_OnEmtpy();
		return false;
	}
	if (FX_OnLoadCalLink(CalLinkDef) == false)
	{
		FX_OnEmtpy();
		return false;
	}

	if (FX_OnLoadCalInput(InputDef) == false)
	{
		FX_OnEmtpy();
		return false;
	}

	if (FX_OnLoadConvex(ConvexDef) == false)
	{
		FX_OnEmtpy();
		return false;
	}
	if (FX_OnLoadICDef(ICDef) == false)
	{

		FX_OnEmtpy();
		return true;
	}
	return true;
}

bool CICBase::FX_OnInitCordBuf(long CordNum)
{
	if (CordNum < 0)
	{
		return false;
	}
	m_Cords = (pCordCell)malloc(sizeof(CordCell) * CordNum);
	if (m_Cords == NULL)
	{
		FX_OnEmtpy();
		return false;
	}
	m_CordsNum = CordNum;
	return true;
}
bool CICBase::FX_OnInitCCsBuf(long CCNum)
{
	if (CCNum < 2)
	{
		return false;
	}
	m_CCs = (pConvexCell)malloc(sizeof(ConvexCell) * CCNum);
	if (m_CCs == NULL)
	{
		return false;
	}
	m_CCsNum = CCNum;

	return true;
}

// bool CICBase::OnInitICsBuf(long ICNum)
// {
// 	if ( ICNum < 1)
// 	{
// 		return false;
// 	}
// 	m_ICs = (pICCell)malloc(sizeof(ICCell) * ICNum);
// 	if (m_ICs == NULL)
// 	{
// 		OnEmtpy();
// 		return false;
// 	}
// 	m_ICsNum = ICNum;
// 	return true;
// }

bool CICBase::FX_OnInitICsBuf(long ICNum)
{
	if (ICNum < 1)
	{
		return false;
	}

	m_ICs.clear();
	m_ICs.resize(100); // 100
	m_ICsNum = ICNum;

	for (long i = 0; i < 100; i++)
	{
		m_ICs[i].SetTag = false;
		m_ICs[i].SubCnt = 0;
		m_ICs[i].MinSpan = 0;
		m_ICs[i].pCC_A = NULL;
		m_ICs[i].pCC_B = NULL;
	}
	return true;
}

void CICBase::FX_OnEmtpy()
{
	if (m_Cords != NULL)
	{
		free(m_Cords);
		m_Cords = NULL;
	}

	m_ICs.clear();
	if (m_CCs != NULL)
	{
		free(m_CCs);
		m_CCs = NULL;
	}
	m_CCsNum = 0;
	m_ICsNum = 0;
	m_CCsNum = 0;

	for (long i = 0; i < 100; i++)
	{
		m_Cord_Input_Arrange[i] = NULL;
		m_Cord_Link_Arrange_Pre[i] = NULL;
		m_Cord_Link_Arrange_Nex[i] = NULL;
	}
	m_Cord_input_Num = 0;
	m_Cord_Link_Num = 0;

	m_pAgvCord = NULL;
}