#ifndef _FX_LOG_H_
#define _FX_LOG_H_

#define FX_LOG_INFO(format) if(CLog::DoLog()) printf(format);


class CLog
{
public:
	static void SetLogOn();
	static void SetLogOff();
	static int DoLog();
	~CLog();
protected:
	CLog();
};

#endif

