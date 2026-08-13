#include "CfgBase.h"

CCfgFILE::CCfgFILE()
{
	m_fp_state = 0;
	m_fp = NULL;
}

CCfgFILE::~CCfgFILE()
{
	FX_OnClose();
}

bool CCfgFILE::FX_OnSetOpen(char* pathname)
{
	if (m_fp_state != 0)
	{
		return false;
	}
	
	m_fp = fopen(pathname, "wb");
	if (m_fp == NULL)
	{
		return false;
	}
	fflush(m_fp);
	m_fp_state = 1;
	return true;
}
bool  CCfgFILE::FX_OnGetOpen(char* pathname)
{
	if (m_fp_state != 0)
	{
		return false;
	}

	m_fp = fopen(pathname, "rb");
	if (m_fp == NULL)
	{
		return false;
	}
	m_fp_state = -1;
	return true;
}
void CCfgFILE::FX_OnClose()
{
	if (m_fp_state == 0)
	{
		return ;
	}
	if (m_fp_state == 1)
	{
		fflush(m_fp);
	}
	fclose(m_fp);
	m_fp = NULL;
	m_fp_state = 0;

}


bool CCfgFILE::FX_OnSetLong(long value)
{
	if (m_fp_state != 1)
	{
		return false;
	}
	fprintf(m_fp,"<i,%ld>",value);
	return true;
}
bool CCfgFILE::FX_OnSetDouble(double value)
{
	if (m_fp_state != 1)
	{
		return false;
	}
	fprintf(m_fp, "<f,%lf>", value);
	return true;
}
bool CCfgFILE::FX_OnSetStr(char *value)
{
	if (m_fp_state != 1)
	{
		return false;
	}
	long slen = strlen(value);
	if (slen < 1 || slen >= 32)
	{
		return false;
	}
	fprintf(m_fp, "<s,%s>",  value);
	return true;
}


bool CCfgFILE::FX_OnSetVect3(double value[3])
{

	if (m_fp_state != 1)
	{
		return false;
	}
	fprintf(m_fp, "<v3,%lf,%lf,%lf,>", value[0], value[1], value[2]);
	return true;
}

bool CCfgFILE::FX_OnSetVect6(double value[6])
{

	if (m_fp_state != 1)
	{
		return false;
	}
	fprintf(m_fp, "<v6,%lf,%lf,%lf,%lf,%lf,%lf>", value[0], value[1], value[2], value[3], value[4], value[5]);
	return true;
}

bool CCfgFILE::FX_OnSetMat3(double value[3][3])
{
	if (m_fp_state != 1)
	{
		return false;
	}
	fprintf(m_fp, "<m3,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf>"
		, value[0][0], value[0][1], value[0][2]
		, value[1][0], value[1][1], value[1][2]
		, value[2][0], value[2][1], value[2][2]);
	return true;
}

bool CCfgFILE::FX_OnSetCR()
{
	if (m_fp_state != 1)
	{
		return false;
	}
	fprintf(m_fp, "\n");
	return true;
}

bool CCfgFILE::FX_OnGetLong(long& value)
{
	if (m_fp_state != -1)
	{
		return false;
	}
	char c = '\0';
	fread(&c,1,1,m_fp);
	if (c == EOF)
	{
		return false;
	}
	while (c != '<')
	{
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			return false;
		}
	}
	char c1, c2;
	fread(&c2, 1, 1, m_fp);
	fread(&c1, 1, 1, m_fp);
	if (c1 != ',' || ! (c2 == 'I' || c2 == 'i'))
	{
		fseek(m_fp, -3, SEEK_CUR);
		return false;
	}
	else
	{
		if (FX_GetValues() != 1)
		{
			return false;
		}
		value = m_valueTmp[0];
		return true;
	}
}

bool CCfgFILE::FX_OnGetDouble(double& value)
{
	if (m_fp_state != -1)
	{
		return false;
	}
	char c = '\0';
	fread(&c, 1, 1, m_fp);
	if (c == EOF)
	{
		return false;
	}
	while (c != '<')
	{
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			return false;
		}
	}
	char c1, c2;
	fread(&c2, 1, 1, m_fp);
	fread(&c1, 1, 1, m_fp);
	if (c1 != ',' || !(c2 == 'f' || c2 == 'F'))
	{
		fseek(m_fp, -3, SEEK_CUR);
		return false;
	}
	else
	{
		if (FX_GetValues() != 1)
		{
			return false;
		}
		value = m_valueTmp[0];
		return true;
	}
}

bool CCfgFILE::FX_OnGetStr(char value[32])
{
	memset(value, 0, 32);
	if (m_fp_state != -1)
	{
		return false;
	}
	char c = '\0';
	fread(&c, 1, 1, m_fp);
	if (c == EOF)
	{
		return false;
	}
	while (c != '<')
	{
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			return false;
		}
	}
	char c1, c2;
	fread(&c2, 1, 1, m_fp);
	fread(&c1, 1, 1, m_fp);
	if (c1 != ',' || !(c2 == 'S' || c2 == 's'))
	{
		fseek(m_fp, -3, SEEK_CUR);
		return false;
	}
	else
	{

		char ct;
		fread(&ct, 1, 1, m_fp);
		long rc = 0;
		while (ct != '>')
		{
			if (rc < 32)
			{

				value[rc] = ct;
			}
			rc++;
			fread(&ct, 1, 1, m_fp);
			if (ct == EOF)
			{
				return false;
			}
		}
		return true;
	}
}


bool CCfgFILE::FX_OnGetVect3(double value[3])
{
	if (m_fp_state != -1)
	{
		return false;
	}
	char c = '\0';
	fread(&c, 1, 1, m_fp);
	if (c == EOF)
	{
		return false;
	}
	while (c != '<')
	{
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			return false;
		}
	}
	char c1, c2, c3;
	fread(&c2, 1, 1, m_fp);
	fread(&c3, 1, 1, m_fp);
	fread(&c1, 1, 1, m_fp);
	if (c1 != ','|| !( c2 == 'V' || c2 == 'v') || c3 != '3')
	{
		fseek(m_fp, -4, SEEK_CUR);
		return false;
	}
	else
	{
		if (FX_GetValues() != 3)
		{
			return false;
		}
		value[0] = m_valueTmp[0];
		value[1] = m_valueTmp[1];
		value[2] = m_valueTmp[2];
		return true;
	}
}


bool CCfgFILE::FX_OnGetVect6(double value[6])
{
	if (m_fp_state != -1)
	{
		return false;
	}
	char c = '\0';
	fread(&c, 1, 1, m_fp);
	if (c == EOF)
	{
		return false;
	}
	while (c != '<')
	{
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			return false;
		}
	}
	char c1, c2, c3;
	fread(&c2, 1, 1, m_fp);
	fread(&c3, 1, 1, m_fp);
	fread(&c1, 1, 1, m_fp);
	if (c1 != ',' || !(c2 == 'V' || c2 == 'v') || c3 != '6')
	{
		fseek(m_fp, -4, SEEK_CUR);
		return false;
	}
	else
	{

		if (FX_GetValues() != 6)
		{
			return false;
		}
		value[0] = m_valueTmp[0];
		value[1] = m_valueTmp[1];
		value[2] = m_valueTmp[2];
		value[3] = m_valueTmp[3];
		value[4] = m_valueTmp[4];
		value[5] = m_valueTmp[5];
		return true;
	}
}

bool CCfgFILE::FX_OnGetMat3(double value[3][3])
{
	if (m_fp_state != -1)
	{
		return false;
	}
	char c = '\0';
	fread(&c, 1, 1, m_fp);
	if (c == EOF)
	{
		return false;
	}
	while (c != '<')
	{
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			return false;
		}
	}
	char c1, c2,c3;
	fread(&c2, 1, 1, m_fp);
	fread(&c3, 1, 1, m_fp);
	fread(&c1, 1, 1, m_fp);
	if (c1 != ',' || !(c2 == 'm' || c2 == 'M') || c3 != '3')
	{
		fseek(m_fp, -4, SEEK_CUR);
		return false;
	}
	else
	{


		if (FX_GetValues() != 9)
		{
			return false;
		}
		value[0][0] = m_valueTmp[0];
		value[0][1] = m_valueTmp[1];
		value[0][2] = m_valueTmp[2];
		value[1][0] = m_valueTmp[3];
		value[1][1] = m_valueTmp[4];
		value[1][2] = m_valueTmp[5];
		value[2][0] = m_valueTmp[6];
		value[2][1] = m_valueTmp[7];
		value[2][2] = m_valueTmp[8];
		return true;
	}



}

long CCfgFILE::FX_GetValues()
{
	if (FX_OnGetTmpBuf() == false)
	{
		return 0;
	}

	long num = 32;
	if (FX_OnGetValue(m_tmpbuf, m_valueTmp, num) == false)
	{
		return 0;
	}
	if (num == 0)
	{
		return 0;
	}
	return num;

}


bool CCfgFILE::FX_OnGetValue(char* buf, double* retv, long& retn)
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

bool CCfgFILE::FX_OnGetTmpBuf()
{
	memset(m_tmpbuf, 0, 512);
	long wpos = 0;
	char c = '\0';
	fread(&c, 1, 1, m_fp);
	if (c == EOF)
	{
		return false;
	}


	while (c != '>')
	{
		if (c != 0x0d && c != 0x0a)
		{
			m_tmpbuf[wpos] = c;
			wpos++;
		}
		
		if (wpos > 510)
		{
			return false;
		}
		c = '\0';
		fread(&c, 1, 1, m_fp);
		if (c == EOF)
		{
			//return true;
			return false;
		}
	}
	return true;
}