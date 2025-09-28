#include "ACB.h"
#include "stdio.h"
#include "stdlib.h"
#include <cstring>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CACB::CACB()
{
	init_tag_ = false;
	base_ = NULL;
	item_num = 0;

	size_ = 10240;
	base_ = (unsigned char*)malloc(size_);

	init_tag_ = true;
	write_pos_ = 1;
	read_pos_ = 0;

	write_lock_ = 0;
	read_lock_ = 0;
	buf_serial_ = 0;

	item_num = 0;



}

CACB::~CACB()
{
	if (init_tag_ == true)
	{
		free(base_);
	}
}
long CACB::OnGetStoreNum()
{
	return item_num;
}

bool CACB::WriteBuf(unsigned char* data_ptr, long size_int)
{
	if (size_int < 1 || init_tag_ == false)
	{
		return false;
	}
	if (write_lock_ != 0)
	{
		return false;
	}
	write_lock_ = 1;

	long emptysize;
	long wpos = write_pos_;
	long rpos = read_pos_;

	unsigned long tmpserial = buf_serial_;
	tmpserial++;
	if (tmpserial >= 100000000)
	{
		tmpserial = 0;
	}


	if (wpos < rpos)
	{
		emptysize = rpos - wpos - 1;
		if (emptysize < size_int + 6)
		{
			write_lock_ = 0;
			return false;
		}
		base_[wpos] = size_int / 256;
		base_[wpos + 1] = size_int % 256;

		base_[wpos + 2] = (unsigned char)(tmpserial / 0x1000000);
		base_[wpos + 3] = (unsigned char)((tmpserial % 0x1000000) / 0x10000);
		base_[wpos + 4] = (unsigned char)((tmpserial % 0x10000) / 0x100);
		base_[wpos + 5] = (unsigned char)((tmpserial % 0x100));


		memcpy(&base_[wpos + 6], data_ptr, size_int);
		wpos += 6;
		wpos += size_int;
		write_pos_ = wpos;

		buf_serial_ = tmpserial;
		write_lock_ = 0;
		item_num++;
		return true;
	}
	else
	{
		long epos = size_ - wpos;
		emptysize = epos + rpos - 1;

		if (emptysize < size_int + 6)
		{
			write_lock_ = 0;
			return false;
		}

		base_[wpos] = size_int / 256;
		wpos++;
		wpos %= size_;
		base_[wpos] = size_int % 256;
		wpos++;
		wpos %= size_;

		base_[wpos] = (unsigned char)(tmpserial / 0x1000000);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)((tmpserial % 0x1000000) / 0x10000);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)((tmpserial % 0x10000) / 0x100);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)((tmpserial % 0x100));
		wpos++;
		wpos %= size_;


		epos -= 6;

		if (epos <= size_int)
		{
			if (epos > 0)
			{
				memcpy(&base_[wpos], data_ptr, epos);
				if (size_int - epos > 0)
				{
					memcpy(&base_[0], &data_ptr[epos], size_int - epos);
				}
			}
			else
			{
				memcpy(&base_[wpos], data_ptr, size_int);
			}
		}
		else
		{
			memcpy(&base_[wpos], data_ptr, size_int);
		}
		wpos += size_int;
		wpos %= size_;
		write_pos_ = wpos;

		buf_serial_ = tmpserial;
		write_lock_ = 0;
		item_num++;
		return true;
	}
}

long CACB::ReadBuf(unsigned char* data_ptr, long size_int)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	if (read_lock_ != 0)
	{
		return -1;
	}
	read_lock_ = 1;

	long wpos = write_pos_;
	long rpos = read_pos_;
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		read_lock_ = 0;
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (size_int < sizetmp)
	{
		read_lock_ = 0;
		return -2;
	}

	rpos++;
	rpos %= size_;


	rpos += 4;
	rpos %= size_;

	long explen = size_ - rpos;
	if (explen <= sizetmp)
	{
		memcpy(data_ptr, &base_[rpos], explen);
		if (sizetmp - explen > 0)
		{
			memcpy(&data_ptr[explen], base_, sizetmp - explen);
		}
	}
	else
	{
		memcpy(data_ptr, &base_[rpos], sizetmp);
	}
	rpos += (sizetmp - 1);
	rpos %= size_;
	read_pos_ = rpos;
	read_lock_ = 0;

	item_num--;
	return sizetmp;
}


long CACB::ReadBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	if (read_lock_ != 0)
	{
		return -1;
	}
	read_lock_ = 1;

	long wpos = write_pos_;
	long rpos = read_pos_;
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		read_lock_ = 0;
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (size_int < sizetmp)
	{
		read_lock_ = 0;
		return -2;
	}

	rpos++;
	rpos %= size_;


	unsigned long v1;
	unsigned long v2;
	unsigned long v3;
	unsigned long v4;

	v1 = base_[rpos];
	rpos++;
	rpos %= size_;
	v2 = base_[rpos];
	rpos++;
	rpos %= size_;
	v3 = base_[rpos];
	rpos++;
	rpos %= size_;
	v4 = base_[rpos];
	rpos++;
	rpos %= size_;

	serial = v1 * 0x1000000 + v2 * 0x10000 + v3 * 0x100 + v4;


	long explen = size_ - rpos;
	if (explen <= sizetmp)
	{
		memcpy(data_ptr, &base_[rpos], explen);
		if (sizetmp - explen > 0)
		{
			memcpy(&data_ptr[explen], base_, sizetmp - explen);
		}
	}
	else
	{
		memcpy(data_ptr, &base_[rpos], sizetmp);
	}
	rpos += (sizetmp - 1);
	rpos %= size_;
	read_pos_ = rpos;
	read_lock_ = 0;
	item_num--;
	return sizetmp;
}

long CACB::PeekBuf(unsigned char* data_ptr, long size_int)
{
	if (init_tag_ == false)
	{
		return -1;
	}

	if (size_int == 0 || data_ptr == NULL)
	{
		long wpos = write_pos_;
		long rpos = read_pos_;
		rpos++;
		rpos %= size_;
		if (rpos == wpos)
		{
			read_lock_ = 0;
			return 0;
		}
		return 1;
	}
	if (read_lock_ != 0)
	{
		return -1;
	}
	read_lock_ = 1;

	long wpos = write_pos_;
	long rpos = read_pos_;
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		read_lock_ = 0;
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (size_int < sizetmp)
	{
		read_lock_ = 0;
		return -2;
	}

	rpos++;
	rpos %= size_;

	rpos += 4;
	rpos %= size_;

	long explen = size_ - rpos;
	if (explen <= sizetmp)
	{
		memcpy(data_ptr, &base_[rpos], explen);
		if (sizetmp - explen > 0)
		{
			memcpy(&data_ptr[explen], base_, sizetmp - explen);
		}
	}
	else
	{
		memcpy(data_ptr, &base_[rpos], sizetmp);
	}

	read_lock_ = 0;
	return sizetmp;
}

long CACB::PeekBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	if (read_lock_ != 0)
	{
		return -1;
	}
	read_lock_ = 1;

	long wpos = write_pos_;
	long rpos = read_pos_;
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		read_lock_ = 0;
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (size_int < sizetmp)
	{
		read_lock_ = 0;
		return -2;
	}

	rpos++;
	rpos %= size_;


	unsigned long v1;
	unsigned long v2;
	unsigned long v3;
	unsigned long v4;

	v1 = base_[rpos];
	rpos++;
	rpos %= size_;
	v2 = base_[rpos];
	rpos++;
	rpos %= size_;
	v3 = base_[rpos];
	rpos++;
	rpos %= size_;
	v4 = base_[rpos];
	rpos++;
	rpos %= size_;

	serial = v1 * 0x1000000 + v2 * 0x10000 + v3 * 0x100 + v4;


	long explen = size_ - rpos;
	if (explen <= sizetmp)
	{
		memcpy(data_ptr, &base_[rpos], explen);
		if (sizetmp - explen > 0)
		{
			memcpy(&data_ptr[explen], base_, sizetmp - explen);
		}
	}
	else
	{
		memcpy(data_ptr, &base_[rpos], sizetmp);
	}
	rpos += (sizetmp - 1);
	rpos %= size_;
	read_pos_ = rpos;
	read_lock_ = 0;
	return sizetmp;

}




bool CACB::Empty()
{
	if (init_tag_ == false)
	{
		return false;
	}
	if (read_lock_ != 0 || write_lock_ != 0)
	{
		return false;
	}

	read_lock_ = 1;
	write_lock_ = 1;

	read_pos_ = 0;
	write_pos_ = 1;

	write_lock_ = 0;
	read_lock_ = 0;


	item_num = 0;

	return true;
}

