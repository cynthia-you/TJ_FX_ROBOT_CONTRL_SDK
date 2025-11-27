#ifndef _FXDG_H_ 
#define _FXDG_H_

class CFXDG
{
public:
	CFXDG();
	virtual ~CFXDG();
	bool OnInit(long item_size);
	bool OnInit(long item_size, long count_size);
	bool OnEmpty();
	bool OnAdd(void* data);
	long OnGetNum();
	bool OnSetNum(long num);
	void* OnGet(long serial);
protected:
	bool EXPadBase();
	bool EXPadBlock();
	void** m_pBase;
	long m_BaseNum;
	long m_BaseSize;
	long m_ItemSize;
	long m_ItemNum;
	bool m_InitTag;
};

#endif
