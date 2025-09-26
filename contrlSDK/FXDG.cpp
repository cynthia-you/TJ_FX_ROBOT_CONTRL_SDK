
#include "FXDG.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

CFXDG::CFXDG()
{
	m_pBase = NULL;
	m_BaseNum = 0;
	m_BaseSize = 0;
	m_ItemNum = 0;
	m_ItemSize = 0;
	m_InitTag = false;
}

CFXDG::~CFXDG()
{
	if (m_pBase != NULL)
	{
		for (long i = 0; i < m_BaseSize; i++)
		{
			if (m_pBase[i] != NULL)
			{
				free(m_pBase[i]);
				m_pBase[i] = NULL;
			}
		}
		free(m_pBase);
	}
}

bool CFXDG::OnEmpty()
{
	if (m_InitTag == false)
	{
		return false;
	}
	m_ItemNum = 0;
	return true;
}

bool CFXDG::OnSetNum(long num)
{
	if (num<0 || num >m_ItemNum)
	{
		return false;
	}
	m_ItemNum = num;
	return true;

}

bool CFXDG::OnInit(long item_size, long count_size)
{

	if (OnInit(item_size) == false)
	{
		return false;
	}
	if (count_size < 100)
	{
		return true;
	}
	long target_count = count_size;
	if (target_count > 1000000)
	{
		target_count = 1000000;
	}
	long cap = m_BaseNum * 100;
	while (cap < target_count)
	{
		if (EXPadBlock() == false)
		{
			return false;
		}
		cap = m_BaseNum * 100;
	}
	return true;
}

bool CFXDG::OnInit(long item_size)
{
	if (item_size <= 0)
	{
		return false;
	}

	if (m_InitTag == true)
	{
		if (item_size != m_ItemSize)
		{
			return false;
		}
		else
		{
			OnEmpty();
			return true;
		}
	}


	m_pBase = (void**)malloc(sizeof(void*) * 100);
	if (m_pBase == NULL)
	{
		return false;
	}
	for (long i = 0; i < 100; i++)
	{
		m_pBase[i] = NULL;
	}
	m_BaseSize = 100;
	m_BaseNum = 0;
	m_ItemSize = item_size;
	m_ItemNum = 0;
	m_InitTag = true;
	return true;
}


bool CFXDG::EXPadBase()
{
	if (m_InitTag == false)
	{
		return false;
	}
	long tmp = m_BaseSize * 2;
	long i;
	void** new_base = (void**)malloc(sizeof(void*) * tmp);
	if (new_base == NULL)
	{
		return false;
	}
	for (i = 0; i < tmp; i++)
	{
		new_base[i] = NULL;
	}
	for (i = 0; i < m_BaseSize; i++)
	{
		new_base[i] = m_pBase[i];
	}
	free(m_pBase);
	m_pBase = new_base;
	m_BaseSize = tmp;
	return true;
}


bool CFXDG::EXPadBlock()
{
	if (m_InitTag == false)
	{
		return false;
	}
	if (m_BaseNum + 1 >= m_BaseSize)
	{
		if (EXPadBase() == false)
		{
			return false;
		}
	}
	m_pBase[m_BaseNum] = malloc(100 * m_ItemSize);
	if (m_pBase[m_BaseNum] == NULL)
	{
		return false;
	}
	m_BaseNum++;
	return true;
}

long CFXDG::OnGetNum()
{
	if (m_InitTag == false)
	{
		return 0;
	}
	return m_ItemNum;
}

void* CFXDG::OnGet(long serial)
{
	if (m_InitTag == false || serial < 0 || serial >= m_ItemNum)
	{
		return NULL;
	}
	long b_serial = serial / 100;
	long i_serial = serial % 100;
	if (b_serial >= m_BaseNum)
	{
		return NULL;
	}
	unsigned char* p1 = (unsigned char*)m_pBase[b_serial];
	p1 = p1 + m_ItemSize * i_serial;
	return (void*)p1;
}

bool CFXDG::OnAdd(void* data)
{
	if (m_InitTag == false || data == NULL)
	{
		return false;
	}
	long cap = m_BaseNum * 100;
	long tmp = m_ItemNum;
	if (tmp + 2 >= cap)
	{
		if (EXPadBlock() == false)
		{
			return false;
		}
	}

	long b_serial = tmp / 100;
	long i_serial = tmp % 100;
	if (b_serial >= m_BaseNum)
	{
		return false;
	}
	unsigned char* p1 = (unsigned char*)m_pBase[b_serial];
	p1 = p1 + m_ItemSize * i_serial;
	memcpy((void*)p1, data, m_ItemSize);
	m_ItemNum++;
	return true;
}




