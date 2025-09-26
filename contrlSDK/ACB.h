#ifndef FX_ACB_H_ 
#define FX_ACB_H_

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
	long			size_;
	long            item_num;
};


#endif


