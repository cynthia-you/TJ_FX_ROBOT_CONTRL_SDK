#ifndef _POINTSET_H_ 
#define _POINTSET_H_
#include "FXDG.h"
#include "stdio.h"
#include "stdlib.h"

enum PoinType
{
	PotT_BEGIN = 0,
	PotT_1d = 1,
	PotT_2d = 2,
	PotT_3d = 3,
	PotT_4d = 4,
	PotT_5d = 5,
	PotT_6d = 6,
	PotT_7d = 7,
	PotT_8d = 8,
	PotT_9d = 9,
	PotT_10d = 10,
	PotT_11d = 11,
	PotT_12d = 12,
	PotT_13d = 13,
	PotT_14d = 14,
	PotT_15d = 15,
	PotT_16d = 16,
	PotT_17d = 17,
	PotT_18d = 18,
	PotT_19d = 19,
	PotT_20d = 20,
	PotT_21d = 21,
	PotT_22d = 22,
	PotT_23d = 23,
	PotT_24d = 24,
	PotT_25d = 25,
	PotT_26d = 26,
	PotT_27d = 27,
	PotT_28d = 28,
	PotT_29d = 29,
	PotT_30d = 30,
	PotT_31d = 31,
	PotT_32d = 32,
	PotT_33d = 33,
	PotT_34d = 34,
	PotT_35d = 35,
	PotT_36d = 36,
	PotT_37d = 37,
	PotT_38d = 38,
	PotT_39d = 39,
	PotT_40d = 40,
	PotT_FINISH = 41,
};

class CPointSet
{
public:

	CPointSet();
	virtual ~CPointSet();
	bool OnInit(PoinType ptype);
	bool OnInit(PoinType ptype, long start_cap);
	bool OnEmpty();
	PoinType OnGetType();
	long OnGetPointNum();
	double* OnGetPoint(long pos);
	bool OnSetPoint(double point_value[]);
	bool OnSave(char* path);
	bool OnSaveHL(char* path);
	bool OnSaveCSV(char* path);
	bool OnLoadXFile(char* path, long load_col_num, long exp_col_num);
	bool OnLoadExp(char* path, long exp_col_num);
	bool OnLoad(char* path);
	bool OnLoadFast(char* path);
	bool OnSave(FILE* fp);
	bool OnLoad(FILE* fp);
	bool OnSetTag(char* tag);
	bool OnSetSubTag(char* subtag);
	char* OnGetTag();
	bool OnCheckTag(char* c);
	bool OnCheckSubTag(char* c);
	bool OnSetNum(long num);
	bool OnMovCloToTail(long col_serial);
	bool OnMult(long col_serial, double mult_value);
	bool OnAdd(long col_serial, double add_value);
	bool OnAdd(long src_col_serial1, long src_col_serial2, long tgt_col_serial);
	bool OnCalDif(long src_col_serial, long tgt_col_serial, long skip_num);
	bool OnSubAtValue(long serial, double target_v, double eps, CPointSet* ret);
	bool OnCpyC2C(long target_c, long src_c);
	bool OnCpyC2C(long target_c, long src_c, CPointSet* src);
	bool OnCpyFrm(CPointSet* src);
	bool OnSubNoEmpty(long serial, double target_v, double eps, CPointSet* ret);
	bool OnSub(long serial, bool in_or_de, CPointSet* ret);
	bool OnSubZone(long serial, double min_v, double max_v, CPointSet* ret);
	bool OnMakeZeroData(PoinType ptype, long num);
	bool OnDifferenceO12(long serial, double ts, CPointSet* pva);

	bool OnSub(long head_size, long head_repeatnum, long tail_size, long tail_repeatnum);

	long OnFindAvgZero(long serial, long ww, double eps, bool is_inc);
	bool OnFltClnCPD3(long serial, long time);
	bool OnFltCln(long serial, long time);
	bool OnFltCln3(long serial, long time);
	bool OnSubSmp(long skp_num);

	bool OnFindByTag(long serial, double tag, long startpos, long& ret_start, long& ret_end);
	bool OnGetSubAVG(long serial,  long start, long end,double & ret_avg);

	bool OnFindByTagGetAvg(long serial, double tag, long startpos, long& ret_start, long& ret_end, long a_serial, double & ret_avg);
protected:
	bool OnGetLine(FILE* fp, char* buf);
	bool OnGetValue(char* buf, double* retv, long& retn);
	PoinType m_PointType;
	CFXDG    m_Points;
	char     m_Tag[256];
	char     m_SubTag[256];

	PoinType m_L_Type[41];






};

#endif 
