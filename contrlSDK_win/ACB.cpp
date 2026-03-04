#include "ACB.h"
#include <cstring>
#include <string>


CGACB::CGACB()
{
    init_tag_ = false;
    base_ = nullptr;
    size_ = 0;
    ctrl_ = nullptr;
#ifdef _WIN32
    hMutex_ = nullptr;
#else
    sem_ = SEM_FAILED;
#endif
}

CGACB::~CGACB()
{
    if (init_tag_) {
#ifdef _WIN32
        if (hMutex_) CloseHandle(hMutex_);
#else
        if (sem_ != SEM_FAILED) sem_close(sem_);
#endif
    }
}

void CGACB::OnSetBuf(unsigned char* buf, long size, const char* shm_name)
{
    ctrl_ = reinterpret_cast<SharedControl*>(buf);
    base_ = buf + sizeof(SharedControl);
    size_ = size - static_cast<long>(sizeof(SharedControl));

#ifdef _WIN32
    std::string mutexName = std::string("Global\\") + shm_name + "_Mutex";
    hMutex_ = CreateMutexA(nullptr, FALSE, mutexName.c_str());
    if (hMutex_ == nullptr) {
        return;
    }
#else
    std::string semName = std::string("/") + shm_name + "_sem";
    sem_ = sem_open(semName.c_str(), O_CREAT, 0666, 1);
    if (sem_ == SEM_FAILED) {
        perror("sem_open");
        return;
    }
#endif

    Lock();
    if (ctrl_->write_pos == 0 && ctrl_->read_pos == 0) {
        ctrl_->write_pos = 1;
        ctrl_->read_pos = 0;
        ctrl_->buf_serial = 0;
        ctrl_->item_num = 0;
    }
    Unlock();

    init_tag_ = true;
}

inline void CGACB::Lock()
{
#ifdef _WIN32
    WaitForSingleObject(hMutex_, INFINITE);
#else
    sem_wait(sem_);
#endif
}

inline void CGACB::Unlock()
{
#ifdef _WIN32
    ReleaseMutex(hMutex_);
#else
    sem_post(sem_);
#endif
}

long CGACB::OnGetStoreNum()
{
    if (!init_tag_) return 0;
    Lock();
    long num = ctrl_->item_num;
    Unlock();
    return num;
}

bool CGACB::WriteBuf(unsigned char* data_ptr, long size_int)
{
    if (size_int < 1 || !init_tag_) return false;
    Lock();

    long wpos = ctrl_->write_pos;
    long rpos = ctrl_->read_pos;

    unsigned long tmpserial = ctrl_->buf_serial + 1;
    if (tmpserial >= 100000000) tmpserial = 0;

    bool success = false;
    if (wpos < rpos) {
        long emptysize = rpos - wpos - 1;
        if (emptysize >= size_int + 6) {
            base_[wpos] = static_cast<unsigned char>(size_int / 256);
            base_[wpos + 1] = static_cast<unsigned char>(size_int % 256);

            base_[wpos + 2] = static_cast<unsigned char>(tmpserial >> 24);
            base_[wpos + 3] = static_cast<unsigned char>((tmpserial >> 16) & 0xFF);
            base_[wpos + 4] = static_cast<unsigned char>((tmpserial >> 8) & 0xFF);
            base_[wpos + 5] = static_cast<unsigned char>(tmpserial & 0xFF);

            memcpy(&base_[wpos + 6], data_ptr, size_int);
            wpos += 6 + size_int;
            ctrl_->write_pos = wpos;
            ctrl_->buf_serial = tmpserial;
            ctrl_->item_num++;
            success = true;
        }
    } else {
        long epos = size_ - wpos;
        long emptysize = epos + rpos - 1;
        if (emptysize >= size_int + 6) {
            long tmp_wpos = wpos;
            base_[tmp_wpos] = static_cast<unsigned char>(size_int / 256);
            tmp_wpos = (tmp_wpos + 1) % size_;
            base_[tmp_wpos] = static_cast<unsigned char>(size_int % 256);
            tmp_wpos = (tmp_wpos + 1) % size_;

            base_[tmp_wpos] = static_cast<unsigned char>(tmpserial >> 24);
            tmp_wpos = (tmp_wpos + 1) % size_;
            base_[tmp_wpos] = static_cast<unsigned char>((tmpserial >> 16) & 0xFF);
            tmp_wpos = (tmp_wpos + 1) % size_;
            base_[tmp_wpos] = static_cast<unsigned char>((tmpserial >> 8) & 0xFF);
            tmp_wpos = (tmp_wpos + 1) % size_;
            base_[tmp_wpos] = static_cast<unsigned char>(tmpserial & 0xFF);
            tmp_wpos = (tmp_wpos + 1) % size_;

            long first_len = size_ - tmp_wpos;
            if (first_len <= size_int) {
                if (first_len > 0) {
                    memcpy(&base_[tmp_wpos], data_ptr, first_len);
                }
                memcpy(base_, &data_ptr[first_len], size_int - first_len);
            } else {
                memcpy(&base_[tmp_wpos], data_ptr, size_int);
            }

            tmp_wpos = (tmp_wpos + size_int) % size_;
            ctrl_->write_pos = tmp_wpos;
            ctrl_->buf_serial = tmpserial;
            ctrl_->item_num++;
            success = true;
        }
    }

    Unlock();
    return success;
}

long CGACB::ReadBuf(unsigned char* data_ptr, long size_int)
{
    if (!init_tag_) return -1;
    Lock();

    long wpos = ctrl_->write_pos;
    long rpos = ctrl_->read_pos;
    rpos = (rpos + 1) % size_;
    if (rpos == wpos) {
        Unlock();
        return 0;
    }

    long sizetmp;
    sizetmp = base_[rpos] * 256;
    rpos = (rpos + 1) % size_;
    sizetmp += base_[rpos];
    if (size_int < sizetmp) {
        Unlock();
        return -2;
    }

    rpos = (rpos + 1) % size_; 
    rpos = (rpos + 4) % size_; 

    long explen = size_ - rpos;
    if (explen <= sizetmp) {
        memcpy(data_ptr, &base_[rpos], explen);
        if (sizetmp - explen > 0) {
            memcpy(&data_ptr[explen], base_, sizetmp - explen);
        }
    } else {
        memcpy(data_ptr, &base_[rpos], sizetmp);
    }

    rpos = (rpos + sizetmp - 1) % size_; 
    ctrl_->read_pos = rpos;
    ctrl_->item_num--;

    Unlock();
    return sizetmp;
}

long CGACB::ReadBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
    if (!init_tag_) return -1;
    Lock();

    long wpos = ctrl_->write_pos;
    long rpos = ctrl_->read_pos;
    rpos = (rpos + 1) % size_;
    if (rpos == wpos) {
        Unlock();
        return 0;
    }

    long sizetmp;
    sizetmp = base_[rpos] * 256;
    rpos = (rpos + 1) % size_;
    sizetmp += base_[rpos];
    if (size_int < sizetmp) {
        Unlock();
        return -2;
    }

    rpos = (rpos + 1) % size_; 

    unsigned long v1, v2, v3, v4;
    v1 = base_[rpos]; rpos = (rpos + 1) % size_;
    v2 = base_[rpos]; rpos = (rpos + 1) % size_;
    v3 = base_[rpos]; rpos = (rpos + 1) % size_;
    v4 = base_[rpos]; rpos = (rpos + 1) % size_;

    serial = (v1 << 24) | (v2 << 16) | (v3 << 8) | v4;

    long explen = size_ - rpos;
    if (explen <= sizetmp) {
        memcpy(data_ptr, &base_[rpos], explen);
        if (sizetmp - explen > 0) {
            memcpy(&data_ptr[explen], base_, sizetmp - explen);
        }
    } else {
        memcpy(data_ptr, &base_[rpos], sizetmp);
    }

    rpos = (rpos + sizetmp - 1) % size_;
    ctrl_->read_pos = rpos;
    ctrl_->item_num--;

    Unlock();
    return sizetmp;
}

long CGACB::PeekBuf(unsigned char* data_ptr, long size_int)
{
    if (!init_tag_) return -1;

    if (size_int == 0 || data_ptr == NULL) {
        Lock();
        long wpos = ctrl_->write_pos;
        long rpos = ctrl_->read_pos;
        rpos = (rpos + 1) % size_;
        int hasData = (rpos != wpos) ? 1 : 0;
        Unlock();
        return hasData;
    }

    Lock();

    long wpos = ctrl_->write_pos;
    long rpos = ctrl_->read_pos;
    rpos = (rpos + 1) % size_;
    if (rpos == wpos) {
        Unlock();
        return 0;
    }

    long sizetmp;
    sizetmp = base_[rpos] * 256;
    rpos = (rpos + 1) % size_;
    sizetmp += base_[rpos];
    if (size_int < sizetmp) {
        Unlock();
        return -2;
    }

    rpos = (rpos + 1) % size_; 
    rpos = (rpos + 4) % size_; 

    long explen = size_ - rpos;
    if (explen <= sizetmp) {
        memcpy(data_ptr, &base_[rpos], explen);
        if (sizetmp - explen > 0) {
            memcpy(&data_ptr[explen], base_, sizetmp - explen);
        }
    } else {
        memcpy(data_ptr, &base_[rpos], sizetmp);
    }

    Unlock();
    return sizetmp;
}

long CGACB::PeekBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial)
{
    if (!init_tag_) return -1;
    Lock();

    long wpos = ctrl_->write_pos;
    long rpos = ctrl_->read_pos;
    rpos = (rpos + 1) % size_;
    if (rpos == wpos) {
        Unlock();
        return 0;
    }

    long sizetmp;
    sizetmp = base_[rpos] * 256;
    rpos = (rpos + 1) % size_;
    sizetmp += base_[rpos];
    if (size_int < sizetmp) {
        Unlock();
        return -2;
    }

    rpos = (rpos + 1) % size_;

    unsigned long v1, v2, v3, v4;
    v1 = base_[rpos]; rpos = (rpos + 1) % size_;
    v2 = base_[rpos]; rpos = (rpos + 1) % size_;
    v3 = base_[rpos]; rpos = (rpos + 1) % size_;
    v4 = base_[rpos]; rpos = (rpos + 1) % size_;

    serial = (v1 << 24) | (v2 << 16) | (v3 << 8) | v4;

    long explen = size_ - rpos;
    if (explen <= sizetmp) {
        memcpy(data_ptr, &base_[rpos], explen);
        if (sizetmp - explen > 0) {
            memcpy(&data_ptr[explen], base_, sizetmp - explen);
        }
    } else {
        memcpy(data_ptr, &base_[rpos], sizetmp);
    }

    Unlock();
    return sizetmp;
}

bool CGACB::Empty()
{
    if (!init_tag_) return false;
    Lock();
    ctrl_->read_pos = 0;
    ctrl_->write_pos = 1;
    ctrl_->item_num = 0;
    ctrl_->buf_serial = 0;// 注意：buf_serial_ 保持不变（原版未复位）
    Unlock();
    return true;
}



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
