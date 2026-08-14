#include "ACB.h"
#include <cstring>
#include <string>
#include <thread>

// On Linux/ARM, shared memory accesses need explicit memory ordering.
// x86's TSO model makes plain loads/stores safe on Windows.
#ifdef __linux__
#define CGACB_STORE(p, v) __atomic_store_n(p, v, __ATOMIC_SEQ_CST)
#define CGACB_LOAD(p)     __atomic_load_n(p, __ATOMIC_SEQ_CST)
#else
#define CGACB_STORE(p, v) (*(p) = (v))
#define CGACB_LOAD(p)     (*(p))
#endif

#define ACB_MAX_ITEM_RATIO 2


/////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CGACB::CGACB()
{
	init_tag_ = false;
	base_ = NULL;
	item_num = 0;

	size_ = 0;
	base_ = NULL;

	init_tag_ = true;

	buf_serial_ = 0;

	item_num = 0;
	max_item_size_ = 0;
}
CGACB::~CGACB()
{
	if (init_tag_ == true)
	{
	}
}
long CGACB::OnGetStoreNum()
{
	return item_num;
}
void CGACB::OnSetBuf(unsigned char * buf,long size)
{
	size_ = size-sizeof(long)*2;
	base_ = &buf[sizeof(long)*2];
	write_pos_ = (long * )&buf[0];
	read_pos_ = (long * )&buf[sizeof(long)];

	CGACB_STORE(write_pos_, 1);
	CGACB_STORE(read_pos_, 0);

	max_item_size_ = size_ / ACB_MAX_ITEM_RATIO;
}
bool CGACB::WriteBuf(unsigned char* data_ptr, long size_int)
{
	if (size_int < 1 || init_tag_ == false)
	{
		return false;
	}
	if (max_item_size_ > 0 && size_int > max_item_size_)
	{
		return false;
	}

	long emptysize;
	long wpos = CGACB_LOAD(write_pos_);
	long rpos = CGACB_LOAD(read_pos_);

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
		CGACB_STORE(write_pos_, wpos);

		buf_serial_ = tmpserial;

		item_num++;
		return true;
	}
	else
	{
		long epos = size_ - wpos;
		emptysize = epos + rpos - 1;

		if (emptysize < size_int + 6)
		{
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
		CGACB_STORE(write_pos_, wpos);

		buf_serial_ = tmpserial;
		item_num++;
		return true;
	}
}

long CGACB::ReadBuf(unsigned char* data_ptr, long size_int)
{
	if (init_tag_ == false)
	{
		return -1;
	}

	long wpos = CGACB_LOAD(write_pos_);
	long rpos = CGACB_LOAD(read_pos_);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		CGACB_STORE(read_pos_, wpos);
		item_num = 0;
		return -3;  
	}
	if (size_int < sizetmp)
	{
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
	CGACB_STORE(read_pos_, rpos);

	item_num--;
	return sizetmp;
}

long CGACB::ReadBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
	if (init_tag_ == false)
	{
		return -1;
	}

	long wpos = CGACB_LOAD(write_pos_);
	long rpos = CGACB_LOAD(read_pos_);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		CGACB_STORE(read_pos_, wpos);
		item_num = 0;
		return -3;
	}
	if (size_int < sizetmp)
	{
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
	CGACB_STORE(read_pos_, rpos);
	item_num--;
	return sizetmp;
}

long CGACB::PeekBuf(unsigned char* data_ptr, long size_int)
{
	if (init_tag_ == false)
	{
		return -1;
	}

	if (size_int == 0 || data_ptr == NULL)
	{
		long wpos = CGACB_LOAD(write_pos_);
		long rpos = CGACB_LOAD(read_pos_);
		rpos++;
		rpos %= size_;
		if (rpos == wpos)
		{
			return 0;
		}
		return 1;
	}

	long wpos = CGACB_LOAD(write_pos_);
	long rpos = CGACB_LOAD(read_pos_);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		return -3;
	}
	if (size_int < sizetmp)
	{
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

	return sizetmp;
}

long CGACB::PeekBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	long wpos = CGACB_LOAD(write_pos_);
	long rpos = CGACB_LOAD(read_pos_);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		return -3;
	}
	if (size_int < sizetmp)
	{
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
	CGACB_STORE(read_pos_, rpos);
	return sizetmp;
}

bool CGACB::Empty()
{
	if (init_tag_ == false)
	{
		return false;
	}

	CGACB_STORE(read_pos_, 0);
	CGACB_STORE(write_pos_, 1);


	item_num = 0;

	return true;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CACB::CACB()
{
	init_tag_ = false;
	base_ = NULL;
	size_ = 10240;
	base_ = (unsigned char*)malloc(size_);
	state_.write_lock.store(0, std::memory_order_relaxed);
	state_.read_lock.store(0, std::memory_order_relaxed);
	state_.write_pos.store(1, std::memory_order_relaxed);
	state_.read_pos.store(0, std::memory_order_relaxed);
	state_.item_num.store(0, std::memory_order_relaxed);
	state_.buf_serial.store(0, std::memory_order_relaxed);
	max_item_size_ = size_ / ACB_MAX_ITEM_RATIO;

	init_tag_ = true;
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
	return state_.item_num.load(std::memory_order_acquire);
}

bool CACB::AcquireLock(std::atomic<unsigned char> &lock)
{
	unsigned char expected = 0;
	while (!lock.compare_exchange_weak(expected, 1,
		std::memory_order_acquire, std::memory_order_relaxed))
	{
		expected = 0;
		std::this_thread::yield();
	}
	return true;
}
void CACB::ReleaseLock(std::atomic<unsigned char> &lock)
{
	lock.store(0, std::memory_order_release);
}

long CACB::ResyncBadHead(long wpos, long rpos)
{
	(void)rpos;  // 直接对齐写指针, rpos 仅用于日志/调试
	state_.read_pos.store(wpos, std::memory_order_release);
	state_.item_num.store(0, std::memory_order_release);
	return -3;
}

bool CACB::WriteBuf(unsigned char* data_ptr, long size_int)
{
	if (size_int < 1 || init_tag_ == false)
	{
		return false;
	}
	if (max_item_size_ > 0 && size_int > max_item_size_)
	{
		return false;
	}

	AcquireLock(state_.write_lock);

	long emptysize;
	long wpos = state_.write_pos.load(std::memory_order_acquire);
	long rpos = state_.read_pos.load(std::memory_order_acquire);

	unsigned long tmpserial = state_.buf_serial.load(std::memory_order_relaxed);
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
			ReleaseLock(state_.write_lock);
			return false;
		}
		base_[wpos] = (unsigned char)(size_int / 256);
		base_[wpos + 1] = (unsigned char)(size_int % 256);

		base_[wpos + 2] = (unsigned char)(tmpserial / 0x1000000);
		base_[wpos + 3] = (unsigned char)((tmpserial % 0x1000000) / 0x10000);
		base_[wpos + 4] = (unsigned char)((tmpserial % 0x10000) / 0x100);
		base_[wpos + 5] = (unsigned char)((tmpserial % 0x100));

		memcpy(&base_[wpos + 6], data_ptr, size_int);
		wpos += 6;
		wpos += size_int;
		state_.write_pos.store(wpos, std::memory_order_release);

		state_.buf_serial.store(tmpserial, std::memory_order_relaxed);
		state_.item_num.fetch_add(1, std::memory_order_release);
		ReleaseLock(state_.write_lock);
		return true;
	}
	else
	{
		long epos = size_ - wpos;
		emptysize = epos + rpos - 1;

		if (emptysize < size_int + 6)
		{
			ReleaseLock(state_.write_lock);
			return false;
		}

		base_[wpos] = (unsigned char)(size_int / 256);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)(size_int % 256);
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
		state_.write_pos.store(wpos, std::memory_order_release);

		state_.buf_serial.store(tmpserial, std::memory_order_relaxed);
		state_.item_num.fetch_add(1, std::memory_order_release);
		ReleaseLock(state_.write_lock);
		return true;
	}
}

long CACB::ReadBuf(unsigned char* data_ptr, long size_int)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	AcquireLock(state_.read_lock);

	long wpos = state_.write_pos.load(std::memory_order_acquire);
	long rpos = state_.read_pos.load(std::memory_order_acquire);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		ReleaseLock(state_.read_lock);
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];

	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		long ret = ResyncBadHead(wpos, rpos);
		ReleaseLock(state_.read_lock);
		return ret; 
	}
	if (size_int < sizetmp)
	{
		ReleaseLock(state_.read_lock);
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
	state_.read_pos.store(rpos, std::memory_order_release);
	ReleaseLock(state_.read_lock);

	state_.item_num.fetch_sub(1, std::memory_order_release);
	return sizetmp;
}

long CACB::ReadBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	AcquireLock(state_.read_lock);

	long wpos = state_.write_pos.load(std::memory_order_acquire);
	long rpos = state_.read_pos.load(std::memory_order_acquire);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		ReleaseLock(state_.read_lock);
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		long ret = ResyncBadHead(wpos, rpos);
		ReleaseLock(state_.read_lock);
		return ret;
	}
	if (size_int < sizetmp)
	{
		ReleaseLock(state_.read_lock);
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
	state_.read_pos.store(rpos, std::memory_order_release);
	ReleaseLock(state_.read_lock);
	state_.item_num.fetch_sub(1, std::memory_order_release);
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
		long wpos = state_.write_pos.load(std::memory_order_acquire);
		long rpos = state_.read_pos.load(std::memory_order_acquire);
		rpos++;
		rpos %= size_;
		if (rpos == wpos)
		{
			return 0;
		}
		return 1;
	}
	AcquireLock(state_.read_lock);

	long wpos = state_.write_pos.load(std::memory_order_acquire);
	long rpos = state_.read_pos.load(std::memory_order_acquire);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		ReleaseLock(state_.read_lock);
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		ReleaseLock(state_.read_lock);
		return -3;
	}
	if (size_int < sizetmp)
	{
		ReleaseLock(state_.read_lock);
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

	ReleaseLock(state_.read_lock);
	return sizetmp;
}

long CACB::PeekBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	AcquireLock(state_.read_lock);

	long wpos = state_.write_pos.load(std::memory_order_acquire);
	long rpos = state_.read_pos.load(std::memory_order_acquire);
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		ReleaseLock(state_.read_lock);
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (sizetmp <= 0 || (max_item_size_ > 0 && sizetmp > max_item_size_))
	{
		ReleaseLock(state_.read_lock);
		return -3;
	}
	if (size_int < sizetmp)
	{
		ReleaseLock(state_.read_lock);
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
	state_.read_pos.store(rpos, std::memory_order_release);
	ReleaseLock(state_.read_lock);
	return sizetmp;
}

bool CACB::Empty()
{
	if (init_tag_ == false)
	{
		return false;
	}

	AcquireLock(state_.read_lock);
	AcquireLock(state_.write_lock);

	state_.read_pos.store(0, std::memory_order_release);
	state_.write_pos.store(1, std::memory_order_release);
	state_.item_num.store(0, std::memory_order_release);

	ReleaseLock(state_.write_lock);
	ReleaseLock(state_.read_lock);

	return true;
}
