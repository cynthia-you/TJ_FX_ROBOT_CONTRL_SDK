#ifndef FX_ACB_H_
#define FX_ACB_H_

#include <cstdint>
#ifdef _WIN32
#include <windows.h>
#else
#include <semaphore.h>
#include <fcntl.h>
#endif

struct SharedControl {
    volatile int32_t  write_pos;   
    volatile int32_t  read_pos;    
    volatile uint32_t buf_serial;  
    volatile int32_t  item_num;    
};

class CACB
{
public:
    CACB();
    virtual ~CACB();
    long OnGetStoreNum();
    bool WriteBuf(unsigned char* data_ptr, long size_int);
    long ReadBuf(unsigned char* data_ptr, long size_int);
    long ReadBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial);
    long PeekBuf(unsigned char* data_ptr, long size_int);
    long PeekBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial);
    bool Empty();
protected:
    bool   init_tag_;
    long   write_pos_;
    long   read_pos_;
    unsigned char  write_lock_;
    unsigned char  read_lock_;
    unsigned long  buf_serial_;
    unsigned char* base_;
    long           size_;
    long           item_num;
};

//Supports cross-process sharing
class CGACB
{
public:
    CGACB();
    virtual ~CGACB();

    long OnGetStoreNum();
    bool WriteBuf(unsigned char* data_ptr, long size_int);
    long ReadBuf(unsigned char* data_ptr, long size_int);
    long ReadBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial);
    long PeekBuf(unsigned char* data_ptr, long size_int);
    long PeekBufWithSer(unsigned char* data_ptr, long size_int, unsigned long& serial);
    bool Empty();
    void OnSetBuf(unsigned char* buf, long size, const char* shm_name);
    

    SharedControl* GetCtrlPtr() const { return ctrl_; }

protected:
    bool   init_tag_;
    unsigned char* base_;  
    long   size_;          
    SharedControl* ctrl_;  

#ifdef _WIN32
    HANDLE hMutex_;        
#else
    sem_t* sem_;           
#endif

private:
    inline void Lock();
    inline void Unlock();
};

#endif