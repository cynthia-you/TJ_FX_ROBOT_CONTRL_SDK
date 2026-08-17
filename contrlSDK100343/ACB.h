#ifndef FX_ACB_H_
#define FX_ACB_H_
#include <atomic>
#include "CmplOpt.h"

struct ACB_AtomicState
{
    std::atomic<unsigned char> write_lock;   
    std::atomic<unsigned char> read_lock;    
    std::atomic<long>          write_pos;    
    std::atomic<long>          read_pos;     
    std::atomic<long>          item_num;     
    std::atomic<unsigned long> buf_serial;   
};

class CACB
{
public:
    CACB();
    virtual ~CACB();
    long OnGetStoreNum();
    bool WriteBuf(unsigned char *data_ptr, long size_int);
    long ReadBuf(unsigned char *data_ptr, long size_int);
    long ReadBufWithSer(unsigned char *data_ptr, long size_int, unsigned long &serial);
    long PeekBuf(unsigned char *data_ptr, long size_int);
    long PeekBufWithSer(unsigned char *data_ptr, long size_int, unsigned long &serial);
    bool Empty();

protected:
    bool init_tag_;
    ACB_AtomicState state_;          
    unsigned char *base_;
    long size_;
    long max_item_size_;            

    bool AcquireLock(std::atomic<unsigned char> &lock);
    void ReleaseLock(std::atomic<unsigned char> &lock);
    long ResyncBadHead(long wpos, long rpos);
};

// Supports cross-process sharing
class CGACB
{
public:
    CGACB();
    virtual ~CGACB();
    long OnGetStoreNum();
    bool WriteBuf(unsigned char *data_ptr, long size_int);
    long ReadBuf(unsigned char *data_ptr, long size_int);
    long ReadBufWithSer(unsigned char *data_ptr, long size_int, unsigned long &serial);
    long PeekBuf(unsigned char *data_ptr, long size_int);
    long PeekBufWithSer(unsigned char *data_ptr, long size_int, unsigned long &serial);
    bool Empty();
    void OnSetBuf(unsigned char *buf, long size);

protected:
    bool init_tag_;
    long *write_pos_;
    long *read_pos_;
    unsigned long buf_serial_;
    long item_num;
    long max_item_size_;          

    unsigned char *base_;
    long size_;
};

#endif
