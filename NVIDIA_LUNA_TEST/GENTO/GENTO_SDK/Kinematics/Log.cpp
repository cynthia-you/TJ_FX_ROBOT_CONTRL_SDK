#include "Log.h"

static int fx_log_is_open = 0;

CLog::CLog()
{

}

CLog::~CLog()
{

}

int CLog::DoLog()
{
	return fx_log_is_open;
}
void CLog::SetLogOn()
{
	fx_log_is_open = 1;
}
void CLog::SetLogOff()
{

	fx_log_is_open = 0;
}
