
#include "FileOP.h"
#include "string.h"



CFileOp::CFileOp()
{
	m_state = File_OP_OK;
	m_ErrorTag = false;
	m_fp = NULL;
}

CFileOp::~CFileOp()
{
	Empty();
}

void CFileOp::Empty()
{
	if(m_fp!= NULL)
	{
		fclose(m_fp);
		m_fp = NULL;
	}
//	m_ErrorTag = false;
	m_state = File_OP_OK;
}

void CFileOp::SetErr()
{
	m_ErrorTag = true;
}
void CFileOp::OnReSetErrorTag()
{
	m_ErrorTag = false;
}

bool CFileOp::OnCheckErrorTag()
{
	return m_ErrorTag;
}

bool CFileOp::OnCheckStateOK()
{
    if(m_state == File_OP_OK)
    {
        return true;
    }
    return false;
}

FileIns * CFileOp::OnRecvFile(char * lpath,char * rpath)
{
	if(lpath == NULL || rpath == NULL )
	{
		return NULL;
	}

	if( m_state != File_OP_OK )
	{
		return NULL;
	}

	if(strlen(lpath) <=0 || strlen(lpath) >= 512 )
	{
		return NULL;
	}
	
	if(strlen(rpath) <=0 || strlen(rpath) >= 512 )
	{
		return NULL;
	}

	memcpy(m_FilePath,lpath,strlen(lpath));
	m_FilePath[strlen(lpath)] = '\0';
	m_fp = fopen(m_FilePath,"wb");

	if(m_fp == NULL)
	{
		return NULL;
	}
	
	pFileIns ins = (pFileIns)malloc(sizeof(FileIns));
	ins->m_InsType = INS_TYPE_Get_Request;
	ins->m_CellSize = strlen(rpath);
	ins->m_TotalBlockNum = m_SendTotalBlockNum;
	memcpy(ins->m_CellContent,rpath,ins->m_CellSize);
	m_state = File_OP_Recv_CLN;
	return ins;
}


FileIns * CFileOp::OnSendFile(char * lpath,char * rpath)
{
	if(lpath == NULL || rpath == NULL )
	{
		return NULL;
	}
	if( m_state != File_OP_OK )
	{
		return NULL;
	}
	if(strlen(rpath) <=0 || strlen(rpath) >= 512 )
	{
		return NULL;
	}

	m_fp = fopen(lpath,"rb");
	if(m_fp == NULL)
	{
		return NULL;
	}
	fseek(m_fp,0,SEEK_END);
	m_SendTotalLen = ftell(m_fp); 
	if(m_SendTotalLen == 0)
	{
		fclose(m_fp);
		m_fp = NULL;
		return NULL;
	}
	m_SendTotalBlockNum = m_SendTotalLen / MAX_FILE_CELL_SIZE;
	if(m_SendTotalLen % MAX_FILE_CELL_SIZE != 0)
	{
		m_SendTotalBlockNum += 1;
	}

	pFileIns ins = (pFileIns)malloc(sizeof(FileIns));
	ins->m_InsType = INS_TYPE_Send_Request;
	ins->m_CellSize = strlen(rpath);
	ins->m_TotalBlockNum = m_SendTotalBlockNum;
	memcpy(ins->m_CellContent,rpath,ins->m_CellSize);
	m_state = File_OP_Send_CLN;
	return ins;

}

bool CFileOp::OnIns(FileIns * ins)
{
	if(ins == NULL)
	{
		return false;
	}

	switch (m_state)
	{
	case File_OP_OK:
		return OnStateOK(ins);
		break;
	case File_OP_Send_SVR:
		return OnStateSendSvr(ins);
		break;
	case File_OP_Send_CLN:
		return OnStateSendCln(ins);
		break;
	case File_OP_Recv_SVR:
		return OnStateRecvSvr(ins);
		break;
	case File_OP_Recv_CLN:
		return OnStateRecvCln(ins);
		break;
	default:
		return false;
	}
	
	return false;
}


bool CFileOp::OnStateOK(FileIns * ins)
{
	if(ins->m_InsType != INS_TYPE_Send_Request && ins->m_InsType != INS_TYPE_Get_Request )
	{
		ins->m_InsType = INS_TYPE_Error;
		return false;
	}
	if(ins->m_InsType == INS_TYPE_Send_Request)
	{
		//收到发送请求
		if(ins->m_CellSize >= 512 || ins->m_CellSize <= 0 )
		{
			ins->m_InsType = INS_TYPE_Send_Request_Report;
			ins->m_ErrorCode = 1;
			return true;
		}
		memcpy(m_FilePath,ins->m_CellContent,ins->m_CellSize);
		m_FilePath[ins->m_CellSize] = '\0';
		m_fp = fopen(m_FilePath,"wb");
		if(m_fp == NULL)
		{
			ins->m_InsType = INS_TYPE_Send_Request_Report;
			ins->m_ErrorCode = 2;
			return true;
		}
		ins->m_InsType = INS_TYPE_Send_Request_Report;
		ins->m_ErrorCode = 0;
		ins->m_CurrentBlockSerial = 0;
		m_ExpectSerial = 0;
		m_state = File_OP_Send_SVR;
		return true;
	}

	if(ins->m_InsType == INS_TYPE_Get_Request)
	{
		//收到发送请求
		if(ins->m_CellSize >= 512 || ins->m_CellSize <= 0 )
		{
			ins->m_InsType = INS_TYPE_Get_Request_Report;
			ins->m_ErrorCode = 1;
			return true;
		}
		char path[1024];
		memcpy(path,ins->m_CellContent,ins->m_CellSize);
		path[ins->m_CellSize] = '\0';
		m_fp = fopen(path,"rb");

		if(m_fp == NULL)
		{
			ins->m_InsType = INS_TYPE_Get_Request_Report;
			ins->m_ErrorCode = 2;
			return true;
		}

		fseek(m_fp,0,SEEK_END);
	

		///////////////////////////////////////

		m_SendTotalLen = ftell(m_fp); 
		if(m_SendTotalLen == 0)
		{
			fclose(m_fp);
			m_fp = NULL;
			ins->m_InsType = INS_TYPE_Get_Request_Report;
			ins->m_ErrorCode = 3;
			return true;
		}
		m_SendTotalBlockNum = m_SendTotalLen / MAX_FILE_CELL_SIZE;
		if(m_SendTotalLen % MAX_FILE_CELL_SIZE != 0)
		{
			m_SendTotalBlockNum += 1;
		}

		ins->m_TotalBlockNum = m_SendTotalBlockNum;
		ins->m_CurrentBlockSerial = 0;
		ins->m_CellSize = 0;
		ins->m_ErrorCode = 0;
		ins->m_InsType = INS_TYPE_Get_Request_Report;
		m_state = File_OP_Recv_SVR;
		return true;
	}

	return false;
}

bool CFileOp::OnStateSendSvr(FileIns * ins)
{
	if(ins->m_InsType != INS_TYPE_Send_File_Cell)
	{
		return false;
	}

	long wltnum = ins->m_CellSize;
	while(wltnum > 0)
	{
		long wn = fwrite(& ins->m_CellContent[ins->m_CellSize - wltnum],1,wltnum,m_fp);
		wltnum -= wn; 
	}

	ins->m_InsType = INS_TYPE_Send_File_Cell_Report;
	ins->m_CurrentBlockSerial += 1;
	if(ins->m_CurrentBlockSerial >= ins->m_TotalBlockNum)
	{
		fclose(m_fp);
		m_fp = NULL;
		m_state = File_OP_OK;
	}
	return true;
}

bool CFileOp::OnStateSendCln(FileIns * ins)
{
	if(ins->m_InsType != INS_TYPE_Send_Request_Report && ins->m_InsType != INS_TYPE_Send_File_Cell_Report)
	{
		return false;
	}
	if(ins->m_InsType == INS_TYPE_Send_Request_Report || ins->m_InsType == INS_TYPE_Send_File_Cell_Report)
	{
		if(ins->m_ErrorCode != 0)
		{
			fclose(m_fp);
			m_fp = NULL;
			m_state = File_OP_OK;
			m_ErrorTag = true;
			return false;
		}
		long a_serial = ins->m_CurrentBlockSerial;
		if(a_serial >= m_SendTotalBlockNum)
		{
			fclose(m_fp);
			m_fp = NULL;
			m_state = File_OP_OK;
			m_ErrorTag = false;
			return false;
		}
		if(a_serial < m_SendTotalBlockNum - 1)
		{
			fseek(m_fp,a_serial * MAX_FILE_CELL_SIZE,SEEK_SET);
			ins->m_CellSize = MAX_FILE_CELL_SIZE;
			long rltnum = ins->m_CellSize;
			while(rltnum > 0)
			{
				long rn = fread(& ins->m_CellContent[ins->m_CellSize - rltnum],1,rltnum,m_fp);
				rltnum -= rn; 
			}
		}
		else
		{
			long relic = m_SendTotalLen%MAX_FILE_CELL_SIZE;
			fseek(m_fp,a_serial * MAX_FILE_CELL_SIZE,SEEK_SET);
			ins->m_CellSize = relic;
			
			long rltnum = ins->m_CellSize;
			while(rltnum > 0)
			{
				long rn = fread(& ins->m_CellContent[ins->m_CellSize - rltnum],1,rltnum,m_fp);
				rltnum -= rn; 
			}
		}
		ins->m_InsType = INS_TYPE_Send_File_Cell;
		ins->m_ErrorCode = 0;
		return true;
	}
	return false;
}

bool CFileOp::OnStateRecvSvr(FileIns * ins)
{
	if(ins->m_InsType != INS_TYPE_Get_File_Cell)
	{
		return false;
	}

	if(ins->m_ErrorCode != 0)
	{
		fclose(m_fp);
		m_fp = NULL;
		m_state = File_OP_OK;
		m_ErrorTag = true;
		return false;
	}
	long a_serial = ins->m_CurrentBlockSerial;
	if(a_serial >= m_SendTotalBlockNum)
	{
		fclose(m_fp);
		m_fp = NULL;
		m_state = File_OP_OK;
		m_ErrorTag = false;
		return false;
	}
	if(a_serial < m_SendTotalBlockNum - 1)
	{
		fseek(m_fp,a_serial * MAX_FILE_CELL_SIZE,SEEK_SET);
		ins->m_CellSize = MAX_FILE_CELL_SIZE;
		long rltnum = ins->m_CellSize;
		while(rltnum > 0)
		{
			long rn = fread(& ins->m_CellContent[ins->m_CellSize - rltnum],1,rltnum,m_fp);
			rltnum -= rn; 
		}
	}
	else
	{
		long relic = m_SendTotalLen%MAX_FILE_CELL_SIZE;
		fseek(m_fp,a_serial * MAX_FILE_CELL_SIZE,SEEK_SET);
		ins->m_CellSize = relic;
		
		long rltnum = ins->m_CellSize;
		while(rltnum > 0)
		{
			long rn = fread(& ins->m_CellContent[ins->m_CellSize - rltnum],1,rltnum,m_fp);
			rltnum -= rn; 
		}
	}
	ins->m_InsType = INS_TYPE_Get_File_Cell_Report;
	ins->m_ErrorCode = 0;
	return true;

}


bool CFileOp::OnStateRecvCln(FileIns * ins)
{
	if(ins->m_InsType != INS_TYPE_Get_Request_Report && ins->m_InsType != INS_TYPE_Get_File_Cell_Report)
	{
		return false;
	}

	if(ins->m_ErrorCode != 0)
	{
		m_ErrorTag = true;
		fclose(m_fp);
		m_fp = NULL;
		m_state = File_OP_OK;
		return false;
	}
	
	if(ins->m_InsType == INS_TYPE_Get_File_Cell_Report)
	{
		long wltnum = ins->m_CellSize;
		
		while(wltnum > 0)
		{
			long wn = fwrite(& ins->m_CellContent[ins->m_CellSize - wltnum],1,wltnum,m_fp);
			wltnum -= wn; 
		}
		
		ins->m_InsType = INS_TYPE_Get_File_Cell;
		ins->m_CurrentBlockSerial += 1;
		if(ins->m_CurrentBlockSerial >= ins->m_TotalBlockNum)
		{
			fclose(m_fp);	
			m_fp = NULL;
			m_state = File_OP_OK;
		}
		return true;
	}

	if(ins->m_InsType == INS_TYPE_Get_Request_Report)
	{
	
		
		ins->m_InsType = INS_TYPE_Get_File_Cell;
		ins->m_CurrentBlockSerial = 0;
		if(ins->m_CurrentBlockSerial >= ins->m_TotalBlockNum)
		{
			fclose(m_fp);
			m_fp = NULL;
			m_state = File_OP_OK;
		}
		return true;
	}
	
	return false;

}
