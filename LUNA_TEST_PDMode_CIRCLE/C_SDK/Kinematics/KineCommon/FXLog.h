#ifndef _FX_LOG_H_
#define _FX_LOG_H_

#include "../../Common/FXType.h"

#define FX_LOG_DEBG_FLAG (1u << 0)
#define FX_LOG_INFO_FLAG (1u << 1)
#define FX_LOG_WARN_FLAG (1u << 2)
#define FX_LOG_ERROR_FLAG (1u << 3)
#define FX_LOG_ALL_FLAG (FX_LOG_DEBG_FLAG | FX_LOG_INFO_FLAG | FX_LOG_WARN_FLAG | FX_LOG_ERROR_FLAG)

#define FX_LOG_DEBG(...) do { CLog::Debug(__VA_ARGS__); } while (0)
#define FX_LOG_INFO(...) do { CLog::Info(__VA_ARGS__); } while (0)
#define FX_LOG_WARN(...) do { CLog::Warn(__VA_ARGS__); } while (0)
#define FX_LOG_ERRO(...) do { CLog::Error(__VA_ARGS__); } while (0)

class CLog
{
public:
	static void SetLogOn();
	static void SetLogOff();
	static void SetLogLevel(FX_UINT32 log_level);
	static FX_UINT32 GetLogLevel();
	static int DoLog();
	static int DoLog(FX_UINT32 log_flag);
	static void Debug(const FX_CHAR *fmt, ...);
	static void Info(const FX_CHAR *fmt, ...);
	static void Warn(const FX_CHAR *fmt, ...);
	static void Error(const FX_CHAR *fmt, ...);
	~CLog();
protected:
	CLog();
};

#endif

