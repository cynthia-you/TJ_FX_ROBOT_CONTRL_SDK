#ifndef _CFG_BASE_H_
#define _CFG_BASE_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


class CCfgFILE
{
public:

	CCfgFILE();
	virtual ~CCfgFILE();

	bool FX_OnSetOpen(char *pathname);
	bool FX_OnGetOpen(char * pathname);
	void FX_OnClose();
	
	bool FX_OnSetLong(long value);
	bool FX_OnSetDouble(double value);
	bool FX_OnSetStr(char *value);
	bool FX_OnSetVect3(double value[3]);
	bool FX_OnSetVect6(double value[6]);
	bool FX_OnSetMat3(double value[3][3]);
	bool FX_OnSetCR();

	bool FX_OnGetLong(long &value);
	bool FX_OnGetDouble(double &value);
	bool FX_OnGetStr(char value[32]);
	bool FX_OnGetVect3(double value[3]);
	bool FX_OnGetVect6(double value[6]);
	bool FX_OnGetMat3(double value[3][3]);

protected:
	bool FX_OnGetValue(char* buf, double* retv, long& retn);
	long FX_GetValues();
	bool FX_OnGetTmpBuf();
	double m_valueTmp[36];
	char m_tmpbuf[512];

	FILE* m_fp;
	long  m_fp_state;
};

#endif
