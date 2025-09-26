
#include "TCPFileClient.h"


bool SendFile (unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4,char * local_file,char * remote_file)
{
	CTCPFileClient cln;
	if(cln.OnLinkTo(ip1,ip2,ip3,ip4,10240) == false)
	{
		return false;
	}
	bool ret = cln.OnSendFile(local_file,remote_file);
	cln.OnQuit();
	return ret;
}


bool RecvFile (unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4,char * local_file,char * remote_file)
{
	CTCPFileClient cln;
	if(cln.OnLinkTo(ip1,ip2,ip3,ip4,10240) == false)
	{
		return false;
	}
	bool ret =  cln.OnGetFile(local_file,remote_file);
	cln.OnQuit();
	return ret;
}

