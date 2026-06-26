#include "FXLog.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static FX_UINT32 fx_log_level = 0;

static FX_VOID FX_LogTrimLineEnd(FX_CHAR *msg)
{
    size_t len = strlen(msg);
    while (len > 0 && (msg[len - 1] == '\n' || msg[len - 1] == '\r'))
    {
        msg[len - 1] = '\0';
        len--;
    }
}

static FX_VOID FX_LogVPrint(FX_UINT32 log_flag, const FX_CHAR *level_tag, const FX_CHAR *fmt, va_list args)
{
    if (!CLog::DoLog(log_flag) || fmt == NULL)
    {
        return;
    }

    FX_CHAR fmt_str[512] = {0};
    vsnprintf(fmt_str, sizeof(fmt_str) - 1, fmt, args);
    FX_LogTrimLineEnd(fmt_str);
    printf("[%s][Kine] %s\n", level_tag, fmt_str);
}

CLog::CLog()
{
}

CLog::~CLog()
{
}

int CLog::DoLog()
{
    return fx_log_level != 0;
}

int CLog::DoLog(FX_UINT32 log_flag)
{
    return (fx_log_level & log_flag) != 0;
}

void CLog::SetLogOn()
{
    fx_log_level = FX_LOG_ALL_FLAG;
}

void CLog::SetLogOff()
{
    fx_log_level = 0;
}

void CLog::SetLogLevel(FX_UINT32 log_level)
{
    fx_log_level = log_level;
}

FX_UINT32 CLog::GetLogLevel()
{
    return fx_log_level;
}

void CLog::Debug(const FX_CHAR *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    FX_LogVPrint(FX_LOG_DEBG_FLAG, "DEBG", fmt, args);
    va_end(args);
}

void CLog::Info(const FX_CHAR *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    FX_LogVPrint(FX_LOG_INFO_FLAG, "INFO", fmt, args);
    va_end(args);
}

void CLog::Warn(const FX_CHAR *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    FX_LogVPrint(FX_LOG_WARN_FLAG, "WARN", fmt, args);
    va_end(args);
}

void CLog::Error(const FX_CHAR *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    FX_CHAR fmt_str[512] = {0};
    vsnprintf(fmt_str, sizeof(fmt_str) - 1, fmt, args);
    FX_LogTrimLineEnd(fmt_str);
    printf("[%s][Kinematics] %s\n", "ERRO", fmt_str);

    va_end(args);
}
