#ifndef _CICBASE_H_
#define _CICBASE_H_
#include <vector>

#define MAXSAPNNUM 50

typedef struct CordCell
{
	char   m_CordName[32];
	char   m_FatherName[32];
	double m_Set[4][4];
	double m_RunL[4][4];
	double m_RunW[4][4];
	double m_Mcp[3];
	double m_McpW[3];
	double m_M;
	long   m_Type;
}*pCordCell;


typedef struct ConvexCell
{
	pCordCell   pCord;
	char   ConvexName[32];
	long   PointNum;
	double PointsDef[20][3];
	double PointsInW[20][3];
}*pConvexCell;

typedef struct ICCell
{
	pConvexCell pCC_A;
	pConvexCell pCC_B;
	double MapDir[3];
	double MinSpan;
	long   SubCnt;
	bool   SetTag;	
}*pICCell;


class CICBase
{
public:
	CICBase();
	virtual ~CICBase();
	bool FX_OnLoadCfg(char * CordDef,char * CalLinkDef,char * InputDef,char * ConvexDef,char * ICDef);
	bool FX_OnSetAGV(double pg[4][4]);
	void FX_DebugPrintCord(char * path);
	void FX_DebugPrintConvex(char * path);
	bool FX_OnUpdateCord(long cord_num, double * input);
	bool FX_OnUpdateConvex();
	bool FX_OnCalInterf();
	long FX_OnGetInterfCellNum();
	bool FX_OnGetInterfSpan(long serial,double & ret_span);
	bool FX_OnGetInterfInfo(long serial,double name_a[32],double name_b[32], double& ret_span);
	void FX_OnGetCoM(double mcp_average[3]);
	void FX_OnGetCordsNum(long &cords_num);
	void FX_OnGetM(double *m);
	void FX_OnGetMinSpan(double *MinSpan);
	bool FX_OnIsInterf(double * inteThsh);
protected:

	/////////////////////
	double FX_OnCalMaxSpan(long anum, double a[20][3], long bnum, double b[20][3], double v[3]);
	double FX_OnAdjOneStep(long anum, double a[20][3], long bnum, double b[20][3], double v[3]);
	void FX_OnGetCrossXYDir(double z[3], double d, double ret_x[3], double ret_y[3]);
	double FX_OnCalSpan(long anum, double a[20][3], long bnum, double b[20][3], double v[3]);
	void FX_OnProjectMinMax(long num, double a[20][3], double v[3], double minmax[2]);
	////////////////////
	long FX_OnLoadICDef(char* ICDef);
	long FX_OnLoadConvex(char* ConvexDef);
	long FX_OnDefInputArrange(char* cord_name, long input_serial);
	long FX_OnDefCalLink(char* pre_name, char* nex_name, long link_serial);
	bool FX_OnLoadCalInput(char* inputDef);
	bool FX_OnLoadCalLink(char* CalLinkDef);
	bool FX_OnLoadCord(char * CordDef);
	bool FX_OnUpdateLink();
	pCordCell FX_SearchCord(char* name);
	pConvexCell FX_SearchConvex(char* name);

	void FX_OnInitV(long anum, double a[20][3], long bnum, double b[20][3], double v[3]);

	bool FX_OnInitCordBuf(long CordNum);
	bool FX_OnInitCCsBuf(long CCNum);
	bool FX_OnInitICsBuf(long ICNum);
	void FX_OnEmtpy();
	pCordCell	m_Cords;
	//pICCell		m_ICs;
	std::vector<ICCell> m_ICs;
	pConvexCell m_CCs;
	long        m_CordsNum;
	long        m_ICsNum;
	long        m_CCsNum;

	pCordCell   m_Cord_Input_Arrange[100];
	long        m_Cord_input_Num;


	pCordCell   m_Cord_Link_Arrange_Pre[100];
	pCordCell   m_Cord_Link_Arrange_Nex[100];
	long        m_Cord_Link_Num;


	pCordCell   m_pAgvCord;


};
#endif
