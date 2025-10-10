#include "PointSet.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

CPointSet::CPointSet()
{
	m_PointType = PotT_BEGIN;
	memset(m_Tag, 0, 256);
	memset(m_SubTag, 0, 256);

	m_L_Type[0] = PotT_BEGIN;
	m_L_Type[1] = PotT_1d;
	m_L_Type[2] = PotT_2d;
	m_L_Type[3] = PotT_3d;
	m_L_Type[4] = PotT_4d;
	m_L_Type[5] = PotT_5d;
	m_L_Type[6] = PotT_6d;
	m_L_Type[7] = PotT_7d;
	m_L_Type[8] = PotT_8d;
	m_L_Type[9] = PotT_9d;
	m_L_Type[10] = PotT_10d;


	m_L_Type[11] = PotT_11d;
	m_L_Type[12] = PotT_12d;
	m_L_Type[13] = PotT_13d;
	m_L_Type[14] = PotT_14d;
	m_L_Type[15] = PotT_15d;
	m_L_Type[16] = PotT_16d;
	m_L_Type[17] = PotT_17d;
	m_L_Type[18] = PotT_18d;
	m_L_Type[19] = PotT_19d;
	m_L_Type[20] = PotT_20d;


	m_L_Type[21] = PotT_21d;
	m_L_Type[22] = PotT_22d;
	m_L_Type[23] = PotT_23d;
	m_L_Type[24] = PotT_24d;
	m_L_Type[25] = PotT_25d;
	m_L_Type[26] = PotT_26d;
	m_L_Type[27] = PotT_27d;
	m_L_Type[28] = PotT_28d;
	m_L_Type[29] = PotT_29d;
	m_L_Type[30] = PotT_30d;
	m_L_Type[31] = PotT_31d;
	m_L_Type[32] = PotT_32d;
	m_L_Type[33] = PotT_33d;
	m_L_Type[34] = PotT_34d;
	m_L_Type[35] = PotT_35d;
	m_L_Type[36] = PotT_36d;
	m_L_Type[37] = PotT_37d;
	m_L_Type[38] = PotT_38d;
	m_L_Type[39] = PotT_39d;
	m_L_Type[40] = PotT_40d;

}

CPointSet::~CPointSet()
{

}

bool CPointSet::OnSetNum(long num)
{
	if (num<0 || num >OnGetPointNum())
	{
		return false;
	}

	return m_Points.OnSetNum(num);

}
bool CPointSet::OnSubNoEmpty(long serial, double target_v, double eps, CPointSet* ret)
{
	if (this->m_PointType == PotT_BEGIN)
	{
		return false;
	}
	if (serial < 0 || serial >= this->m_PointType)
	{
		return false;
	}
	if (this->m_PointType != ret->m_PointType)
	{
		return false;
	}
	long num = this->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		if (fabs(p[serial] - target_v) < fabs(eps))
		{
			ret->OnSetPoint(p);
		}
	}

	num = ret->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}
	return true;
}
bool CPointSet::OnSubZone(long serial, double min_v, double max_v, CPointSet* ret)
{
	if (this->m_PointType == PotT_BEGIN)
	{
		return false;
	}
	if (serial < 0 || serial >= this->m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	ret->OnInit(this->m_PointType);
	ret->OnEmpty();
	long i;

	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		if (p[serial] < max_v && p[serial] > min_v)
		{
			ret->OnSetPoint(p);
		}
	}


	num = ret->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}
	return true;
}
bool CPointSet::OnGetSubAVG(long serial, long start, long end, double& ret_avg)
{
	double dnum = 0;
	double tv = 0;

	long i;
	long tp = 0;
	for (i = start; i <= end; i++)
	{
		double* v = this->OnGetPoint(i);
		if (v != NULL)
		{
			tv += v[serial];
			dnum += 1;
		}

	}

	if (dnum < 0.5)
	{
		return false;
	}
	tv /= dnum;
	return true;
}

bool CPointSet::OnFindByTagGetAvg(long serial, double tag, long startpos, long& ret_start, long& ret_end,long a_serial, double& ret_avg)
{
	long num = OnGetPointNum();
	if (startpos < 0)
	{
		startpos = 0;
	}
	double tn = 0;
	double tv = 0;
	long i;
	long tp = 0;
	for (i = startpos; i < num; i++)
	{
		double* v = this->OnGetPoint(i);
		bool sm = false;
		if (fabs(v[serial] - tag) < 0.00001)
		{
			sm = true;
		}
		if (tp == 0)
		{
			if (sm == true)
			{
				tn += 1;
				tv += v[a_serial];
				ret_start = i;
				ret_end = i;
				tp = 1;
				ret_avg = tv / tn;
			}
		}
		else
		{
			if (sm == false)
			{
				
				return true;
			}
			else
			{
				tn += 1;
				tv += v[a_serial];
				ret_end = i;
				ret_avg = tv / tn;
			}
		}

	}

	return tp == 1;
}
bool CPointSet::OnFindByTag(long serial, double tag, long startpos, long& ret_start, long& ret_end)
{
	long num = OnGetPointNum();
	if (startpos < 0)
	{
		startpos = 0;
	}
	long i;
	long tp = 0;
	for ( i = startpos; i < num; i++)
	{
		double* v = this->OnGetPoint(i);
		bool sm = false;
		if (fabs(v[serial] - tag) < 0.00001)
		{
			sm = true;
		}
		if (tp == 0)
		{
			if (sm == true)
			{
				ret_start = i;
				ret_end = i;
				tp = 1;
			}
		}
		else
		{
			if (sm == false)
			{
				return true;
			}
			else
			{
				ret_end = i;
			}
		}
		
	}

	return tp == 1;
}

bool CPointSet::OnSub(long head_size, long head_repeatnum, long tail_size, long tail_repeatnum)
{
	long num = this->OnGetPointNum();
	if (num < head_size + tail_size + 2)
	{
		return false;
	}
	CPointSet pset;
	pset.OnInit(this->OnGetType());

	long i;
	for ( i = head_size; i < num - tail_size; i++)
	{
		double* v = OnGetPoint(i);
		if (i == head_size)
		{
			for (long j = 0; j < head_repeatnum-1; j++)
			{
				pset.OnSetPoint(v);
			}
		}
		if (i == num - tail_size - 1)
		{
			for (long j = 0; j < tail_repeatnum - 1; j++)
			{
				pset.OnSetPoint(v);
			}
		}
		pset.OnSetPoint(v);
	}

	this->OnEmpty();
	num = pset.OnGetPointNum();
	for ( i = 0; i < num; i++)
	{
		double* v = pset.OnGetPoint(i);
		OnSetPoint(v);
	}
	return true;
}

bool CPointSet::OnSubSmp(long skp_num)
{
	if (skp_num < 1)
	{
		skp_num = 1;
	}
	if (skp_num > 100)
	{
		skp_num = 100;
	}
	CPointSet tmp;
	tmp.OnInit(this->OnGetType());
	long num = this->OnGetPointNum();
	long i;
	skp_num += 1;
	for (i = 0; i < num; i += skp_num)
	{
		tmp.OnSetPoint(this->OnGetPoint(i));
	}

	num = tmp.OnGetPointNum();
	this->OnEmpty();
	for (i = 0; i < num; i++)
	{
		this->OnSetPoint(tmp.OnGetPoint(i));
	}
	return true;

}
bool CPointSet::OnSub(long serial, bool in_or_de, CPointSet* ret)
{
	if (this->m_PointType == PotT_BEGIN)
	{
		return false;
	}
	if (serial < 0 || serial >= this->m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	ret->OnInit(this->m_PointType);
	ret->OnEmpty();
	long i;
	if (in_or_de == true)
	{
		for (i = 0; i < num - 1; i++)
		{
			double* p = this->OnGetPoint(i);
			double* n = this->OnGetPoint(i + 1);
			if (p[serial] < n[serial])
			{
				ret->OnSetPoint(p);
			}
		}
	}
	else
	{
		for (i = 0; i < num + 1; i++)
		{
			double* p = this->OnGetPoint(i);
			double* n = this->OnGetPoint(i + 1);
			if (p[serial] > n[serial])
			{
				ret->OnSetPoint(p);
			}
		}
	}


	num = ret->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}
	return true;
}
bool CPointSet::OnSubAtValue(long serial, double target_v, double eps, CPointSet* ret)
{
	if (this->m_PointType == PotT_BEGIN)
	{
		return false;
	}
	if (serial < 0 || serial >= this->m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	ret->OnInit(this->m_PointType);
	ret->OnEmpty();
	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		if (fabs(p[serial] - target_v) < fabs(eps))
		{
			ret->OnSetPoint(p);
		}
	}

	num = ret->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}
	return true;
}
bool CPointSet::OnInit(PoinType ptype, long start_cap)
{
	if (ptype <= PotT_BEGIN || ptype >= PotT_FINISH)
	{
		return false;
	}

	if (m_PointType == PotT_BEGIN)
	{
		if (m_Points.OnInit(ptype * sizeof(double), start_cap) == true)
		{
			m_PointType = ptype;
			memset(m_Tag, 0, 256);
			return true;
		}
	}
	else
	{
		if (m_PointType == ptype)
		{
			if (m_Points.OnEmpty() == true)
			{
				memset(m_Tag, 0, 256);
				return true;
			}
		}
		else
		{
			if (m_Points.OnInit(ptype * sizeof(double), start_cap) == true)
			{
				m_PointType = ptype;
				memset(m_Tag, 0, 256);
				return true;
			}

		}
	}

	memset(m_Tag, 0, 256);
	return false;

}

bool CPointSet::OnInit(PoinType ptype)
{
	if (ptype <= PotT_BEGIN || ptype >= PotT_FINISH)
	{
		return false;
	}

	if (m_PointType == PotT_BEGIN)
	{
		if (m_Points.OnInit(ptype * sizeof(double)) == true)
		{
			m_PointType = ptype;
			memset(m_Tag, 0, 256);
			return true;
		}
	}
	else
	{
		if (m_PointType == ptype)
		{
			if (m_Points.OnEmpty() == true)
			{
				memset(m_Tag, 0, 256);
				return true;
			}
		}
		else
		{

		}
	}

	memset(m_Tag, 0, 256);
	return false;
}

PoinType CPointSet::OnGetType()
{
	return m_PointType;
}

bool CPointSet::OnEmpty()
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	return m_Points.OnEmpty();
}


bool CPointSet::OnSetSubTag(char* subtag)
{
	if (subtag == NULL)
	{
		return false;
	}
	long tlen = strlen(subtag);
	if (tlen <= 0 || tlen >= 256)
	{
		return false;
	}

	bool ok_tag = true;
	long i;
	for (i = 0; i < tlen && ok_tag == true; i++)
	{
		ok_tag = false;
		if (subtag[i] >= 'a' && subtag[i] <= 'z')
		{
			ok_tag = true;
		}

		if (subtag[i] >= 'A' && subtag[i] <= 'Z')
		{
			ok_tag = true;
		}

		if (i != 0)
		{
			if (subtag[i] >= '0' && subtag[i] <= '9')
			{
				ok_tag = true;
			}
		}

		if (subtag[i] == '_' || subtag[i] == '#')
		{
			ok_tag = true;

		}
	}

	if (ok_tag == false)
	{
		return false;
	}

	memset(m_SubTag, 0, 256);

	for (i = 0; i < tlen; i++)
	{
		m_SubTag[i] = subtag[i];
	}
	return true;

}

bool CPointSet::OnSetTag(char* tag)
{
	if (tag == NULL)
	{
		return false;
	}
	long tlen = strlen(tag);
	if (tlen <= 0 || tlen >= 256)
	{
		return false;
	}

	bool ok_tag = true;
	long i;
	for (i = 0; i < tlen && ok_tag == true; i++)
	{
		ok_tag = false;
		if (tag[i] >= 'a' && tag[i] <= 'z')
		{
			ok_tag = true;
		}

		if (tag[i] >= 'A' && tag[i] <= 'Z')
		{
			ok_tag = true;
		}

		if (i != 0)
		{
			if (tag[i] >= '0' && tag[i] <= '9')
			{
				ok_tag = true;
			}
		}

		if (tag[i] == '_' || tag[i] == '#')
		{
			ok_tag = true;

		}
	}

	if (ok_tag == false)
	{
		return false;
	}

	memset(m_Tag, 0, 256);

	for (i = 0; i < tlen; i++)
	{
		m_Tag[i] = tag[i];
	}
	return true;
}

char* CPointSet::OnGetTag()
{
	if (strlen(m_Tag) == 0)
	{
		return NULL;
	}
	return m_Tag;
}

bool CPointSet::OnCheckSubTag(char* c)
{
	long len = strlen(m_SubTag);
	if (len == 0)
	{
		return false;
	}
	if (c == NULL)
	{
		return false;
	}
	long len2 = strlen(c);
	if (len != len2)
	{
		return false;
	}

	for (long i = 0; i < len; i++)
	{
		if (m_SubTag[i] != c[i])
		{
			return false;
		}
	}
	return true;
}

bool CPointSet::OnCheckTag(char* c)
{
	long len = strlen(m_Tag);
	if (len == 0)
	{
		return false;
	}
	if (c == NULL)
	{
		return false;
	}
	long len2 = strlen(c);
	if (len != len2)
	{
		return false;
	}

	for (long i = 0; i < len; i++)
	{
		if (m_Tag[i] != c[i])
		{
			return false;
		}
	}
	return true;
}

long CPointSet::OnGetPointNum()
{
	return m_Points.OnGetNum();
}

double* CPointSet::OnGetPoint(long pos)
{
	return (double*)m_Points.OnGet(pos);
}

bool CPointSet::OnSetPoint(double point_value[])
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	return m_Points.OnAdd(point_value);
}

bool CPointSet::OnSave(FILE* fp)
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	long num = OnGetPointNum();


	char r = 0x0a;
	if (strlen(m_Tag) == 0)
	{
		fprintf(fp, "PoinType=%d@%d%c", m_PointType, num, r);
	}
	else
	{
		if (strlen(m_SubTag) == 0)
		{
			fprintf(fp, "PoinType=%d@%dT%d%s%c", m_PointType, num, strlen(m_Tag), m_Tag, r);
		}
		else
		{
			fprintf(fp, "PoinType=%d@%dT%d%sST%d%s%c", m_PointType, num, strlen(m_Tag), m_Tag, strlen(m_SubTag), m_SubTag, r);
		}
	}

	double* tmp;
	for (long i = 0; i < num; i++)
	{
		tmp = OnGetPoint(i);
		for (long j = 0; j < m_PointType; j++)
		{
			if (j < 3)
			{
				fprintf(fp, "%c %lf$", 'X' + j, tmp[j]);
			}
			if (j >= 3 && j < 6)
			{
				fprintf(fp, "%c %lf$", 'A' + j - 3, tmp[j]);
			}
			if (j >= 6 && j < 9)
			{
				fprintf(fp, "%c %lf$", 'U' + j - 6, tmp[j]);
			}
			if (j >= 9 && j < 12)
			{
				fprintf(fp, "%c %lf$", 'O' + j - 9, tmp[j]);
			}
			if (j >= 12 && j < 15)
			{
				fprintf(fp, "%c %lf$", 'I' + j - 12, tmp[j]);
			}

			if (j >= 15 && j < 18)
			{
				fprintf(fp, "%c %lf$", 'L' + j - 15, tmp[j]);
			}

			if (j >= 18 && j < 21)
			{
				fprintf(fp, "%c %lf$", 'R' + j - 18, tmp[j]);
			}

			if (j >= 21)
			{
				fprintf(fp, "O %lf$", tmp[j]);
			}
		}
		fprintf(fp, "%c", r);
	}
	return true;
}

bool CPointSet::OnSave(char* path)
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	long num = OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	if (path == NULL)
	{
		return false;
	}
	FILE* fp = fopen(path, "wb");
	if (fp == NULL)
	{
		return false;
	}
	fflush(fp);

	char r = 0x0a;
	if (strlen(m_Tag) == 0)
	{
		fprintf(fp, "PoinType=%d@%d%c", m_PointType, num, r);
	}
	else
	{
		if (strlen(m_SubTag) == 0)
		{
			fprintf(fp, "PoinType=%d@%dT%d%s%c", m_PointType, num, strlen(m_Tag), m_Tag, r);
		}
		else
		{
			fprintf(fp, "PoinType=%d@%dT%d%sST%d%s%c", m_PointType, num, strlen(m_Tag), m_Tag, strlen(m_SubTag), m_SubTag, r);
		}
	}

	double* tmp;
	for (long i = 0; i < num; i++)
	{
		tmp = OnGetPoint(i);
		for (long j = 0; j < m_PointType; j++)
		{
			if (j < 3)
			{
				fprintf(fp, "%c %lf$", 'X' + j, tmp[j]);
			}
			if (j >= 3 && j < 6)
			{
				fprintf(fp, "%c %lf$", 'A' + j - 3, tmp[j]);
			}
			if (j >= 6 && j < 9)
			{
				fprintf(fp, "%c %lf$", 'U' + j - 6, tmp[j]);
			}
			if (j >= 9 && j < 12)
			{
				fprintf(fp, "%c %lf$", 'O' + j - 9, tmp[j]);
			}
			if (j >= 12 && j < 15)
			{
				fprintf(fp, "%c %lf$", 'I' + j - 12, tmp[j]);
			}

			if (j >= 15 && j < 18)
			{
				fprintf(fp, "%c %lf$", 'L' + j - 15, tmp[j]);
			}

			if (j >= 18 && j < 21)
			{
				fprintf(fp, "%c %lf$", 'R' + j - 18, tmp[j]);
			}

			if (j >= 21)
			{
				fprintf(fp, "O %lf$", tmp[j]);
			}
		}
		fprintf(fp, "%c", r);
	}
	fclose(fp);
	return true;
}


bool CPointSet::OnSaveHL(char* path)
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	long num = OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	if (path == NULL)
	{
		return false;
	}
	FILE* fp = fopen(path, "wb");
	if (fp == NULL)
	{
		return false;
	}
	fflush(fp);

	char r = 0x0a;
	if (strlen(m_Tag) == 0)
	{
		fprintf(fp, "PoinType=%d@%d%c", m_PointType, num, r);
	}
	else
	{
		if (strlen(m_SubTag) == 0)
		{
			fprintf(fp, "PoinType=%d@%dT%d%s%c", m_PointType, num, strlen(m_Tag), m_Tag, r);
		}
		else
		{
			fprintf(fp, "PoinType=%d@%dT%d%sST%d%s%c", m_PointType, num, strlen(m_Tag), m_Tag, strlen(m_SubTag), m_SubTag, r);
		}
	}

	double* tmp;
	for (long i = 0; i < num; i++)
	{
		tmp = OnGetPoint(i);
		for (long j = 0; j < m_PointType; j++)
		{
			if (j < 3)
			{
				fprintf(fp, "%c %.15lf$", 'X' + j, tmp[j]);
			}
			if (j >= 3 && j < 6)
			{
				fprintf(fp, "%c %.15lf$", 'A' + j - 3, tmp[j]);
			}
			if (j >= 6 && j < 9)
			{
				fprintf(fp, "%c %.15lf$", 'U' + j - 6, tmp[j]);
			}
			if (j >= 9 && j < 12)
			{
				fprintf(fp, "%c %.15lf$", 'O' + j - 9, tmp[j]);
			}
			if (j >= 12 && j < 15)
			{
				fprintf(fp, "%c %.15lf$", 'I' + j - 12, tmp[j]);
			}

			if (j >= 15 && j < 18)
			{
				fprintf(fp, "%c %.15lf$", 'L' + j - 15, tmp[j]);
			}

			if (j >= 18 && j < 21)
			{
				fprintf(fp, "%c %.15lf$", 'R' + j - 18, tmp[j]);
			}

			if (j >= 21)
			{
				fprintf(fp, "O %.15lf$", tmp[j]);
			}
		}
		fprintf(fp, "%c", r);
	}
	fclose(fp);
	return true;
}


bool CPointSet::OnSaveCSV(char* path)
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	long num = OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	if (path == NULL)
	{
		return false;
	}
	FILE* fp = fopen(path, "wb");
	if (fp == NULL)
	{
		return false;
	}
	fflush(fp);

	char r = 0x0a;


	double* tmp;
	for (long i = 0; i < num; i++)
	{
		tmp = OnGetPoint(i);
		for (long j = 0; j < m_PointType; j++)
		{
			fprintf(fp, "%lf,", tmp[j]);
		}
		fprintf(fp, "%c", r);
	}
	fclose(fp);
	return true;
}

bool CPointSet::OnGetLine(FILE* fp, char* buf)
{
	if (fp == NULL || buf == NULL)
	{
		return false;
	}

	buf[0] = '\0';
	char c = EOF;
	fread(&c, 1, 1, fp);
	long len = 0;
	while (c != EOF)
	{
		if (c != 0x0d && c != 0x0a)
		{
			buf[len] = c;
			len++;
			buf[len] = '\0';
			if (len > 1000)
			{
				return false;
			}
		}
		else
		{
			if (len != 0)
			{
				return true;
			}
		}
		c = EOF;
		fread(&c, 1, 1, fp);
		if (c == EOF)
		{
			if (len != 0)
			{
				return true;
			}
		}
	}

	return false;
}

bool CPointSet::OnGetValue(char* buf, double* retv, long& retn)
{
	long pos = 0;
	double tmp = 0;
	bool value_tag = false;
	long slen = strlen(buf);
	double dpos = 1;

	double min_v = 1;
	for (long i = 0; i < slen; i++)
	{
		char c = buf[i];

		if (c == '.' ||
			c == '-' ||
			c == '0' ||
			c == '1' ||
			c == '2' ||
			c == '3' ||
			c == '4' ||
			c == '5' ||
			c == '6' ||
			c == '7' ||
			c == '8' ||
			c == '9')
		{

			if (c == '.' || c == '-')
			{
				if (c == '-')
				{
					if (min_v < 0)
					{
						return false;
					}
					min_v = -1.0;
				}
				if (c == '.')
				{
					if (dpos < 0.7)
					{
						return false;
					}
					dpos = 0.1;
				}
			}
			else
			{
				double v = c - '0';
				if (dpos > 0.7)
				{
					tmp *= 10;
					tmp += v;
				}
				else
				{
					v *= dpos;
					tmp += v;
					dpos *= 0.1;
				}
				value_tag = true;
			}

		}
		else
		{
			if (value_tag == true)
			{
				retv[pos] = tmp * min_v;
				pos++;
				value_tag = false;
				dpos = 1.0;
				min_v = 1.0;
				tmp = 0;
			}

			if (c == 0x0d || c == 0x0a)
			{

				retn = pos;
				return true;
			}
		}
	}

	if (value_tag == true)
	{
		retv[pos] = tmp * min_v;
		pos++;
		value_tag = false;
		dpos = 1.0;
		min_v = 1.0;
	}

	retn = pos;
	return true;

}

bool CPointSet::OnMult(long col_serial, double mult_value)
{
	if (col_serial < 0 || col_serial >= m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		p[col_serial] *= mult_value;
	}
	return true;
}

bool CPointSet::OnAdd(long src_col_serial1, long src_col_serial2, long tgt_col_serial)
{
	if (src_col_serial1 < 0 || src_col_serial1 >= m_PointType)
	{
		return false;
	}
	if (src_col_serial2 < 0 || src_col_serial2 >= m_PointType)
	{
		return false;
	}
	if (tgt_col_serial < 0 || tgt_col_serial >= m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		p[tgt_col_serial] = p[src_col_serial1] + p[src_col_serial2];
	}
	return true;
}

long CPointSet::OnFindAvgZero(long serial, long ww, double eps ,bool is_inc )
{
	if (eps < 0.001)
	{
		eps = 0.001;
	}
	long w = ww;
	if (w < 3 )
	{
		w = 3;
	}

	if (w > 100)
	{
		w = 100;
	}

	if (serial < 0 || serial >= m_PointType)
	{
		return -1;
	}
	long num = this->OnGetPointNum();
	if (num < w + 100)
	{
		return -1;
	}
	if (is_inc)
	{
		long i;
		double totalv = 0;
		double sp = w;
		sp *= eps;
		for (i = 0; i < w; i++)
		{
			double* v = this->OnGetPoint(i);
			totalv += v[serial];
		}
		if (totalv > sp || totalv < -sp)
		{
			return -2;
		}

		for (i = 1; i + w < num; i++)
		{
			double* v = this->OnGetPoint(i-1);
			totalv -= v[serial];
			v = this->OnGetPoint(i + w);
			totalv += v[serial];
			if (totalv > sp || totalv < -sp )
			{
				return i;
			}

		}

		return i-1;
	}
	else
	{
		long i;
		double totalv = 0;
		double sp = w;
		sp *= eps;
		for (i = 0; i < w; i++)
		{
			double* v = this->OnGetPoint(num -1 - i);
			totalv += v[serial];
		}
		if (totalv > sp || totalv < -sp)
		{
			return -2;
		}

		for (i = num - 2 - w; i >= 0; i--)
		{
			double* v = this->OnGetPoint(i);
			totalv += v[serial];
			v = this->OnGetPoint(i + w + 1);
			totalv -= v[serial];
			if (totalv > sp || totalv < -sp)
			{
				return i+w;
			}
		}
		return i + 1;
	}
}

bool CPointSet::OnDifferenceO12(long serial, double ts, CPointSet* pva)
{
	if (serial < 0 || serial >= m_PointType)
	{
		return false;
	}

	pva->OnInit(PotT_3d);
	pva->OnEmpty();
	long num = this->OnGetPointNum(); 
	long i;
	double p[3];
	p[0] = 0;
	p[1] = 0;
	p[2] = 0;
	for ( i = 0; i < num; i++)
	{
		double * pp = this->OnGetPoint(i);
		p[0] = pp[serial];
		pva->OnSetPoint(p);
	}

	for ( i = 1; i < num-1; i++)
	{
		double* pre = pva->OnGetPoint(i - 1);
		double* cur = pva->OnGetPoint(i );
		double* nex = pva->OnGetPoint(i + 1);
		cur[1] = 2 *  (nex[0] - pre[0]) / ts;
		if (i == 1)
		{
			pre[1] = cur[1];
		}
		if (i == num - 2)
		{
			nex[1] = cur[1];
		}
	}


	for (i = 1; i < num - 1; i++)
	{
		double* pre = pva->OnGetPoint(i - 1);
		double* cur = pva->OnGetPoint(i);
		double* nex = pva->OnGetPoint(i + 1);
		cur[2] = 2 * (nex[1] - pre[1]) / ts;
		if (i == 1)
		{
			pre[2] = cur[2];
		}
		if (i == num - 2)
		{
			nex[2] = cur[2];
		}
	}

	return true;

}

bool CPointSet::OnCalDif(long src_col_serial, long tgt_col_serial, long skip_num)
{
	if (src_col_serial < 0 || src_col_serial >= m_PointType)
	{
		return false;
	}
	long num = this->OnGetPointNum();
	if (skip_num < 1 || skip_num > 0.3 * num)
	{
		return false;
	}
	if (tgt_col_serial < 0 || tgt_col_serial >= m_PointType)
	{
		return false;
	}


	long i;
	for (i = skip_num; i < num - skip_num; i++)
	{
		double* pre = this->OnGetPoint(i - skip_num);
		double* cur = this->OnGetPoint(i);
		double* nex = this->OnGetPoint(i + skip_num);
		cur[tgt_col_serial] = nex[src_col_serial] - pre[src_col_serial];
	}

	for (i = 0; i < skip_num; i++)
	{
		double* pre = this->OnGetPoint(i);
		double* cur = this->OnGetPoint(skip_num);
		pre[tgt_col_serial] = cur[tgt_col_serial];
	}
	for (i = num - 1; i > num - skip_num - 1; i--)
	{
		double* cur = this->OnGetPoint(num - skip_num - 1);
		double* nxt = this->OnGetPoint(i);
		nxt[tgt_col_serial] = cur[tgt_col_serial];
	}

	return true;
}

bool CPointSet::OnMakeZeroData(PoinType ptype, long num)
{
	OnInit(ptype);
	double v[100];
	long i;
	for (i = 0; i < 100; i++)
	{
		v[i] = 0;
	}
	for (i = 0; i < num; i++)
	{
		OnSetPoint(v);
	}
	return true;
}

bool CPointSet::OnCpyFrm(CPointSet* src)
{
	if (this->OnInit(src->OnGetType()) == false)
	{
		return false;
	}

	if (this->OnEmpty() == false)
	{
		return false;
	}

	long num = src->OnGetPointNum();
	for (long i = 0; i < num; i++)
	{
		this->OnSetPoint(src->OnGetPoint(i));
	}

	num = this->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}
	return true;
}
bool CPointSet::OnCpyC2C(long target_c, long src_c, CPointSet* src)
{
	if (target_c < 0 || target_c >= m_PointType)
	{
		return false;
	}

	if (src_c < 0 || src_c >= src->OnGetType())
	{
		return false;
	}

	long nn = src->OnGetPointNum();
	long mm = OnGetPointNum();
	if (nn != mm)
	{
		return false;
	}
	long i;
	for (i = 0; i < nn; i++)
	{
		double* s = src->OnGetPoint(i);
		double* t = OnGetPoint(i);
		t[target_c] = s[src_c];

	}
	return true;
}


void PSlinearSmooth3(double in[], double out[], int size)
{
	int i;
	if (size < 3)
	{
		for (i = 0; i <= size - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (5.0 * in[0] + 2.0 * in[1] - in[2]) / 6.0;

		for (i = 1; i <= size - 2; i++)
		{
			out[i] = (in[i - 1] + in[i] + in[i + 1]) / 3.0;
		}

		out[size - 1] = (5.0 * in[size - 1] + 2.0 * in[size - 2] - in[size - 3]) / 6.0;
	}
}


void PSlinearSmooth7(double in[], double out[], int size)
{
	int i;
	if (size < 7)
	{
		for (i = 0; i <= size - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (13.0 * in[0] + 10.0 * in[1] + 7.0 * in[2] + 4.0 * in[3]
			+ in[4] - 2.0 * in[5] - 5.0 * in[6]) / 28.0;

		out[1] = (5.0 * in[0] + 4.0 * in[1] + 3 * in[2] + 2 * in[3]
			+ in[4] - in[5]) / 14.0;

		out[2] = (7.0 * in[0] + 6.0 * in[1] + 5.0 * in[2] + 4.0 * in[3]
			+ 3.0 * in[4] + 2.0 * in[5] + in[6]) / 28.0;

		for (i = 3; i <= size - 4; i++)
		{
			out[i] = (in[i - 3] + in[i - 2] + in[i - 1] + in[i] + in[i + 1]
				+ in[i + 2] + in[i + 3]) / 7.0;
		}

		out[size - 3] = (7.0 * in[size - 1] + 6.0 * in[size - 2] + 5.0 * in[size - 3]
			+ 4.0 * in[size - 4] + 3.0 * in[size - 5] + 2.0 * in[size - 6]
			+ in[size - 7]) / 28.0;

		out[size - 2] = (5.0 * in[size - 1] + 4.0 * in[size - 2] + 3.0 * in[size - 3]
			+ 2.0 * in[size - 4] + in[size - 5] - in[size - 6]) / 14.0;

		out[size - 1] = (13.0 * in[size - 1] + 10.0 * in[size - 2] + 7.0 * in[size - 3]
			+ 4.0 * in[size - 4] + in[size - 5] - 2.0 * in[size - 6] - 5.0 * in[size - 7]) / 28.0;
	}
}

bool CPointSet::OnFltClnCPD3(long serial, long time)
{
	if (OnFltCln3(serial, time) == false)
	{
		return false;
	}
	CPointSet tmp;
	tmp.OnCpyFrm(this);
	tmp.OnFltCln3(serial, time);
	long num = this->OnGetPointNum();
	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		double* t = tmp.OnGetPoint(i);
		p[serial] += (p[serial] - t[serial]);
	}
	return true;
}
bool CPointSet::OnFltCln3(long serial, long time)
{
	long offset = serial;
	long i;
	long num = OnGetPointNum();
	if (num <= 2)
	{
		return false;
	}

	double x_s;
	double x_e;

	double* tmp1 = NULL;
	double* tmp2 = NULL;

	tmp1 = (double*)malloc(sizeof(double) * num);
	tmp2 = (double*)malloc(sizeof(double) * num);
	if (tmp1 == NULL ||
		tmp2 == NULL)
	{
		free(tmp1);
		free(tmp2);
		return false;
	}
	double* v;

	for (i = 0; i < num; i++)
	{
		v = OnGetPoint(i);
		tmp1[i] = v[offset];
	}


	x_s = tmp1[0];
	x_e = tmp1[num - 1];

	for (i = 0; i < time; i++)
	{
		PSlinearSmooth3(tmp1, tmp2, num);
		tmp2[0] = x_s;
		tmp2[num - 1] = x_e;
		PSlinearSmooth3(tmp2, tmp1, num);
		tmp1[0] = x_s;
		tmp1[num - 1] = x_e;
	}

	for (i = 0; i < num; i++)
	{
		v = OnGetPoint(i);
		v[offset] = tmp1[i];
	}

	free(tmp1);
	free(tmp2);
	return true;
}


bool CPointSet::OnFltCln(long serial, long time)
{
	long offset = serial;
	long i;
	long num = OnGetPointNum();
	if (num <= 2)
	{
		return false;
	}

	double x_s;
	double x_e;

	double* tmp1 = NULL;
	double* tmp2 = NULL;

	tmp1 = (double*)malloc(sizeof(double) * num);
	tmp2 = (double*)malloc(sizeof(double) * num);
	if (tmp1 == NULL ||
		tmp2 == NULL)
	{
		free(tmp1);
		free(tmp2);
		return false;
	}
	double* v;

	for (i = 0; i < num; i++)
	{
		v = OnGetPoint(i);
		tmp1[i] = v[offset];
	}


	x_s = tmp1[0];
	x_e = tmp1[num - 1];

	for (i = 0; i < time; i++)
	{
		PSlinearSmooth7(tmp1, tmp2, num);
		tmp2[0] = x_s;
		tmp2[num - 1] = x_e;
		PSlinearSmooth7(tmp2, tmp1, num);
		tmp1[0] = x_s;
		tmp1[num - 1] = x_e;
	}

	for (i = 0; i < num; i++)
	{
		v = OnGetPoint(i);
		v[offset] = tmp1[i];
	}

	free(tmp1);
	free(tmp2);
	return true;
}
bool CPointSet::OnCpyC2C(long target_c, long src_c)
{
	if (target_c < 0 || target_c >= m_PointType)
	{
		return false;
	}
	if (src_c < 0 || src_c >= m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		p[target_c] = p[src_c];
	}
	return true;
}
bool  CPointSet::OnAdd(long col_serial, double add_value)
{
	if (col_serial < 0 || col_serial >= m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	long i;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		p[col_serial] += add_value;
	}
	return true;
}

bool CPointSet::OnMovCloToTail(long col_serial)
{
	if (col_serial < 0 || col_serial >= m_PointType)
	{
		return false;
	}

	long num = this->OnGetPointNum();
	long cnum = m_PointType;
	long i, j;
	double tmp;
	for (i = 0; i < num; i++)
	{
		double* p = this->OnGetPoint(i);
		tmp = p[col_serial];
		for (j = col_serial; j < cnum - 1; j++)
		{
			p[j] = p[j + 1];
		}
		p[cnum - 1] = tmp;
	}
	return true;
}
bool CPointSet::OnLoadXFile(char* path, long load_col_num, long exp_col_num)
{
	if (exp_col_num < 1 || exp_col_num >40)
	{
		return false;
	}
	if (load_col_num < 1 || load_col_num >40)
	{
		return false;
	}
	if (load_col_num > exp_col_num)
	{
		return false;
	}

	this->OnEmpty();
	this->OnInit(m_L_Type[exp_col_num]);


	FILE* fp = fopen(path, "rb");
	if (fp == NULL)
	{
		printf("load err\n");
		return false;
	}

	char buf[1024];
	double xy[50];
	long rnum;
	bool ctag = false;
	while (OnGetLine(fp, buf))
	{
		if (OnGetValue(buf, xy, rnum) == false)
		{
			fclose(fp);
			return false;
		}
		else
		{
			if (rnum >= load_col_num)
			{
				for (long i = load_col_num; i < 50; i++)
				{
					xy[i] = 0;
				}
				this->OnSetPoint(xy);
			}
		}
	}
	fclose(fp);

	long num = this->OnGetPointNum();
	if (num <= 0)
	{
		return false;
	}

	return true;
}
bool CPointSet::OnLoad(FILE* fp)
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}


	char tag_buf[256];
	memset(tag_buf, 0, 256);
	char stag_buf[256];
	memset(stag_buf, 0, 256);

	char r = 0x0a;
	long lt;
	long num;
	if (fscanf(fp, "PoinType=%d@%d%c", &lt, &num, &r) != 3)
	{
		fclose(fp);
		return false;
	}

	if (r == 'T')
	{
		long tlen;
		if (fscanf(fp, "%d", &tlen) != 1)
		{
			fclose(fp);
			return false;
		}
		if (tlen <= 0 || tlen > 256)
		{
			fclose(fp);
			return false;
		}


		for (long i = 0; i < tlen; i++)
		{
			if (fscanf(fp, "%c", &tag_buf[i]) != 1)
			{
				fclose(fp);
				return false;
			}
		}
		if (fscanf(fp, "%c", &r) != 1)
		{
			fclose(fp);
			return false;
		}
		if (r != 0x0a)
		{
			if (r != 'S')
			{
				fclose(fp);
				return false;
			}
			if (fscanf(fp, "T%d", &tlen) != 1)
			{
				fclose(fp);
				return false;
			}
			if (tlen <= 0 || tlen > 256)
			{
				fclose(fp);
				return false;
			}


			for (long i = 0; i < tlen; i++)
			{
				if (fscanf(fp, "%c", &stag_buf[i]) != 1)
				{
					fclose(fp);
					return false;
				}
			}
			if (fscanf(fp, "%c", &r) != 1)
			{
				fclose(fp);
				return false;
			}
			if (r != 0x0a)
			{
				fclose(fp);
				return false;
			}
		}
	}


	if (lt != m_PointType)
	{
		return false;
	}

	if (num < 0)
	{
		return false;
	}

	if (strlen(tag_buf) != 0)
	{
		if (OnSetTag(tag_buf) == false)
		{
			fclose(fp);
			return false;
		}
	}

	if (strlen(stag_buf) != 0)
	{
		if (OnSetSubTag(stag_buf) == false)
		{
			fclose(fp);
			return false;
		}
	}


	if (m_Points.OnInit(m_PointType * sizeof(double)) == false)
	{
		fclose(fp);
		return false;
	}
	double tmp[100];
	char c;
	for (long i = 0; i < num; i++)
	{
		for (long j = 0; j < m_PointType; j++)
		{
			if (fscanf(fp, "%c %lf$", &c, &tmp[j]) != 2)
			{
				return false;
			};
		}
		fscanf(fp, "%c", &r);
		if (r != 0x0a)
		{
			return false;
		}
		OnSetPoint(tmp);
	}
	return true;
}

bool CPointSet::OnLoadExp(char* path, long exp_col_num)
{
	if (exp_col_num < 1 || exp_col_num >40)
	{
		return false;
	}
	this->OnEmpty();
	this->OnInit(m_L_Type[exp_col_num]);

	CPointSet tmp;
	if (tmp.OnLoadFast(path) == false)
	{
		return false;
	}

	double v[100];
	long i;
	for (i = 0; i < 100; i++)
	{
		v[i] = 0;
	}

	long num = tmp.OnGetPointNum();
	long tt = tmp.OnGetType();
	if (tt > exp_col_num)
	{
		tt = exp_col_num;
	}
	long j;
	for (i = 0; i < num; i++)
	{
		double* p = tmp.OnGetPoint(i);
		for (j = 0; j < tt; j++)
		{
			v[j] = p[j];
		}
		this->OnSetPoint(v);

	}


	return true;
}


bool CPointSet::OnLoadFast(char* path)
{
	if (path == NULL)
	{
		return false;
	}
	FILE* fp = fopen(path, "rb");
	if (fp == NULL)
	{
		return false;
	}

	char tag_buf[256];
	memset(tag_buf, 0, 256);
	memset(m_Tag, 0, 256);
	char stag_buf[256];
	memset(stag_buf, 0, 256);
	memset(m_SubTag, 0, 256);


	char r = 0x0a;
	long lt;
	long num;
	if (fscanf(fp, "PoinType=%d@%d%c", &lt, &num, &r) != 3)
	{
		fclose(fp);
		return false;
	}


	if (r == 'T')
	{
		long tlen;
		if (fscanf(fp, "%d", &tlen) != 1)
		{
			fclose(fp);
			return false;
		}
		if (tlen <= 0 || tlen > 256)
		{
			fclose(fp);
			return false;
		}


		for (long i = 0; i < tlen; i++)
		{
			if (fscanf(fp, "%c", &tag_buf[i]) != 1)
			{
				fclose(fp);
				return false;
			}
		}
		if (fscanf(fp, "%c", &r) != 1)
		{
			fclose(fp);
			return false;
		}
		if (r != 0x0a)
		{
			if (r != 'S')
			{
				fclose(fp);
				return false;
			}
			if (fscanf(fp, "T%d", &tlen) != 1)
			{
				fclose(fp);
				return false;
			}
			if (tlen <= 0 || tlen > 256)
			{
				fclose(fp);
				return false;
			}


			for (long i = 0; i < tlen; i++)
			{
				if (fscanf(fp, "%c", &stag_buf[i]) != 1)
				{
					fclose(fp);
					return false;
				}
			}
			if (fscanf(fp, "%c", &r) != 1)
			{
				fclose(fp);
				return false;
			}
			if (r != 0x0a)
			{
				fclose(fp);
				return false;
			}
		}
	}

	if (OnInit(PoinType(lt)) == false)
	{
		fclose(fp);
		return false;
	}

	if (strlen(tag_buf) != 0)
	{
		if (OnSetTag(tag_buf) == false)
		{
			fclose(fp);
			return false;
		}
	}

	if (strlen(stag_buf) != 0)
	{
		if (OnSetSubTag(stag_buf) == false)
		{
			fclose(fp);
			return false;
		}
	}

	if (num <= 0)
	{
		fclose(fp);
		return false;
	}

	if (m_Points.OnInit(m_PointType * sizeof(double)) == false)
	{
		fclose(fp);
		return false;
	}
	double tmp[100];
	char c;
	for (long i = 0; i < num; i++)
	{
		for (long j = 0; j < m_PointType; j++)
		{
			if (fscanf(fp, "%c %lf$", &c, &tmp[j]) != 2)
			{
				return false;
			}
		}
		fscanf(fp, "%c", &r);
		if (r != 0x0a)
		{
			fclose(fp);
			return false;
		}
		OnSetPoint(tmp);
	}
	fclose(fp);
	return true;
}
bool CPointSet::OnLoad(char* path)
{
	if (m_PointType == PotT_BEGIN)
	{
		return false;
	}
	if (path == NULL)
	{
		return false;
	}
	FILE* fp = fopen(path, "rb");
	if (fp == NULL)
	{
		return false;
	}

	char tag_buf[256];
	memset(tag_buf, 0, 256);
	char stag_buf[256];
	memset(stag_buf, 0, 256);

	char r = 0x0a;
	long lt;
	long num;
	if (fscanf(fp, "PoinType=%d@%d%c", &lt, &num, &r) != 3)
	{
		fclose(fp);
		return false;
	}

	if (r == 'T')
	{
		long tlen;
		if (fscanf(fp, "%d", &tlen) != 1)
		{
			fclose(fp);
			return false;
		}
		if (tlen <= 0 || tlen > 256)
		{
			fclose(fp);
			return false;
		}


		for (long i = 0; i < tlen; i++)
		{
			if (fscanf(fp, "%c", &tag_buf[i]) != 1)
			{
				fclose(fp);
				return false;
			}
		}
		if (fscanf(fp, "%c", &r) != 1)
		{
			fclose(fp);
			return false;
		}
		if (r != 0x0a)
		{
			if (r != 'S')
			{
				fclose(fp);
				return false;
			}
			if (fscanf(fp, "T%d", &tlen) != 1)
			{
				fclose(fp);
				return false;
			}
			if (tlen <= 0 || tlen > 256)
			{
				fclose(fp);
				return false;
			}


			for (long i = 0; i < tlen; i++)
			{
				if (fscanf(fp, "%c", &stag_buf[i]) != 1)
				{
					fclose(fp);
					return false;
				}
			}
			if (fscanf(fp, "%c", &r) != 1)
			{
				fclose(fp);
				return false;
			}
			if (r != 0x0a)
			{
				fclose(fp);
				return false;
			}
		}
	}

	if (lt != m_PointType)
	{
		fclose(fp);
		return false;
	}
	if (num <= 0)
	{
		fclose(fp);
		return false;
	}

	if (strlen(tag_buf) != 0)
	{
		if (OnSetTag(tag_buf) == false)
		{
			fclose(fp);
			return false;
		}
	}
	if (strlen(stag_buf) != 0)
	{
		if (OnSetSubTag(stag_buf) == false)
		{
			fclose(fp);
			return false;
		}
	}
	if (m_Points.OnInit(m_PointType * sizeof(double)) == false)
	{
		fclose(fp);
		return false;
	}

	double tmp[100];
	char c;
	for (long i = 0; i < num; i++)
	{
		for (long j = 0; j < m_PointType; j++)
		{
			if (fscanf(fp, "%c %lf$", &c, &tmp[j]) != 2)
			{
				return false;
			}
		}
		fscanf(fp, "%c", &r);
		if (r != 0x0a)
		{
			fclose(fp);
			return false;
		}
		OnSetPoint(tmp);
	}
	fclose(fp);
	return true;
}

