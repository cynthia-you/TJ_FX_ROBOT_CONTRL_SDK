
#include "Parser.h"
#include "string.h"


CParser::CParser()
{
	m_raw_data = (char *)malloc(MAX_PARSER_BUFSIZE * 10);
	m_context  = (char *)malloc(MAX_PARSER_BUFSIZE);
	m_maked_buf = (char *)malloc(MAX_PARSER_BUFSIZE);

	m_maked_buf[0] = 'I';
	m_maked_buf[1] = 'N';
	m_maked_buf[2] = 'S';
	m_maked_buf[3] = 'D';
	m_maked_buf[4] = 'A';
	m_maked_buf[5] = '@';

	uc_len_1 = (unsigned char *) &m_maked_buf[6];
	uc_len_2 = (unsigned char *) &m_maked_buf[7];
	uc_len_3 = (unsigned char *) &m_maked_buf[8];
	m_maked_buf[9] = '#';

	m_raw_data_len = 0;
	m_context_len = 0;
	m_raw_data_start_pos = 0;

}
CParser::~CParser()
{
	free(m_raw_data);
	free(m_context);
	free(m_maked_buf);
	
}

char * CParser::OnPack(char * data,long size,long & ret_size)
{
	if(size <=0 || data == NULL)
	{
		return NULL;
	}
	if(size + 12 >= MAX_PARSER_BUFSIZE)
	{
		return NULL;
	}

	*uc_len_1 = size / 10000;
	*uc_len_2 = (size % 10000) / 100;
	*uc_len_3 = (size % 100) ;
	memcpy(&m_maked_buf[10],data,size);
	m_maked_buf [10 + size] =   'E';
	m_maked_buf [10 + size+1] = 'D';
	ret_size =  10 + size + 2;
	return m_maked_buf;
}


bool CParser::OnAddRawData(char * data,long size)
{
	if(size <= 0)
	{
		return false;
	}

	long relic = MAX_PARSER_BUFSIZE * 10 - m_raw_data_len;
	if(relic < size )
	{
		return false;
	}
	long w_pos = m_raw_data_start_pos + m_raw_data_len;
	w_pos %= (MAX_PARSER_BUFSIZE * 10);
	
	long lt = w_pos + size;

	if(lt <= (MAX_PARSER_BUFSIZE * 10))
	{
		memcpy(&m_raw_data[w_pos],data,size);
		m_raw_data_len += size;
		return true;
	}

	lt %= (MAX_PARSER_BUFSIZE * 10);

	memcpy(&m_raw_data[w_pos],data,size - lt);
	memcpy(&m_raw_data[0],&data[size - lt],lt);
	m_raw_data_len += size;
	return true;
}

bool CParser::OnUnPack()
{
	if(m_raw_data_len <= 0)
	{
		
		m_raw_data_len = 0;
		m_context_len = 0;
		m_raw_data_start_pos = 0;
		return false;
	}

	///////////// find_head;
	long tlen = MAX_PARSER_BUFSIZE * 10;
	bool head_tag = false;
	long m_scan_skip_num = 0;
	long context_len = 0;
	if(1)
	{
		
		long head_tag_pos1;
		long head_tag_pos2;
		long head_tag_pos3;
		long head_tag_pos4;
		long head_tag_pos5;
		long head_tag_pos6;
		long head_tag_pos7;
		long head_tag_pos8;
		long head_tag_pos9;
		long head_tag_pos10;

		head_tag_pos1 = 0 + m_raw_data_start_pos;
		head_tag_pos2 = 0 + m_raw_data_start_pos;
		head_tag_pos3 = 1 + m_raw_data_start_pos;
		head_tag_pos4 = 2 + m_raw_data_start_pos;
		head_tag_pos5 = 3 + m_raw_data_start_pos;
		head_tag_pos6 = 4 + m_raw_data_start_pos;
		head_tag_pos7 = 5 + m_raw_data_start_pos;
		head_tag_pos8 = 6 + m_raw_data_start_pos;
		head_tag_pos9 = 7 + m_raw_data_start_pos;
		head_tag_pos10 = 8 + m_raw_data_start_pos;

		head_tag_pos1 %= tlen;
		head_tag_pos2 %= tlen;
		head_tag_pos3 %= tlen;
		head_tag_pos4 %= tlen;
		head_tag_pos5 %= tlen;
		head_tag_pos6 %= tlen;
		head_tag_pos7 %= tlen;
		head_tag_pos8 %= tlen;
		head_tag_pos9 %= tlen;
		head_tag_pos10 %= tlen;

		for(long i = 0; i < m_raw_data_len - 10 && head_tag == false; i ++ )
		{
			head_tag_pos1 = head_tag_pos2;
			head_tag_pos2 = head_tag_pos3;
			head_tag_pos3 = head_tag_pos4;
			head_tag_pos4 = head_tag_pos5;
			head_tag_pos5 = head_tag_pos6;
			head_tag_pos6 = head_tag_pos7;
			head_tag_pos7 = head_tag_pos8;
			head_tag_pos8 = head_tag_pos9;
			head_tag_pos9 = head_tag_pos10;

			head_tag_pos10 = i + m_raw_data_start_pos + 9;
			head_tag_pos10 %= tlen;

			if(m_raw_data[head_tag_pos1] != 'I')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}
			if(m_raw_data[head_tag_pos2] != 'N')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}
			if(m_raw_data[head_tag_pos3] != 'S')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}
			if(m_raw_data[head_tag_pos4] != 'D')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}
			if(m_raw_data[head_tag_pos5] != 'A')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}
			if(m_raw_data[head_tag_pos6] != '@')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}

			if(m_raw_data[head_tag_pos10] != '#')
			{
				m_raw_data[head_tag_pos1] = '\0';
				m_scan_skip_num ++;
				continue;
			}
			unsigned char len1 = m_raw_data[head_tag_pos7];
			unsigned char len2 = m_raw_data[head_tag_pos8];
			unsigned char len3 = m_raw_data[head_tag_pos9];

			context_len = len1 * 10000 + len2 * 100 + len3; 
			head_tag = true;	
		}
	}

	m_raw_data_len -= m_scan_skip_num;

	m_raw_data_start_pos += m_scan_skip_num;

	m_raw_data_start_pos %=  tlen;

	if(head_tag == false)
	{
		return false;
	}

	long expect_len = context_len + 12;
	if(m_raw_data_len < expect_len)
	{
		return false;
	}

	long chk1_ppos;
	long chk2_ppos;
	chk1_ppos = m_raw_data_start_pos + 10 + context_len;
	chk2_ppos = m_raw_data_start_pos + 11 + context_len;
	chk1_ppos %= tlen;
	chk2_ppos %= tlen;
	if(m_raw_data[chk1_ppos] != 'E' || m_raw_data[chk2_ppos] != 'D')
	{
		m_raw_data_start_pos += 1;
		m_raw_data_start_pos%= tlen;
		m_raw_data_len -= 1;
		if(m_raw_data_len <= 0)
		{
			m_raw_data_start_pos = 0;
			m_raw_data_len = 0;
		}
		return OnUnPack();
	}



	long read_start_pos;
	long read_len;

	read_start_pos = m_raw_data_start_pos + 10;
	read_start_pos %= tlen;
	read_len = context_len;

	long relic = tlen - read_start_pos;
	memset(m_context,0,read_len);
	if( relic > read_len )
	{
		memcpy(m_context,&m_raw_data[read_start_pos],read_len);
		m_context_len = read_len;
	}
	else
	{
		memcpy(m_context,&m_raw_data[read_start_pos],relic);
		memcpy(&m_context[relic],&m_raw_data[0],read_len - relic);
	}
	m_raw_data_len -= (12 + read_len);
	m_raw_data_start_pos += (12 + read_len);
	m_raw_data_start_pos %= tlen;
	if(m_raw_data_len <= 0)
	{
		m_raw_data_len = 0;
		m_raw_data_start_pos = 0;
	}

	return true;

}

char * CParser::OnGetContent(long & size)
{
	size = m_context_len;
	return m_context;
}