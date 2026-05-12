#ifndef FX_FILEOP_H_ 
#define FX_FILEOP_H_
#pragma pack(4)
/**
 * @file FileOP.h
 * @brief The file includes the definitions for file operation.
 * @author lync
 */

#include "stdio.h"
#include "stdlib.h"
#include "netdef.h"

/**
 * @brief Defines the status of file operation.
 */
enum File_OP_State
{ 
	File_OP_OK             = 1,
	File_OP_Send_SVR       = 2,
	File_OP_Send_CLN       = 3,
	File_OP_Recv_SVR       = 4,
	File_OP_Recv_CLN       = 5,
};
/**
 * @brief Defines the types of file transfer frame.
 */
enum InsType
{ 
	INS_TYPE_Send_Request            = 1,
	INS_TYPE_Send_Request_Report     = 2,
	INS_TYPE_Send_File_Cell          = 3,
	INS_TYPE_Send_File_Cell_Report   = 4,	
	INS_TYPE_Get_Request             = 5,
	INS_TYPE_Get_Request_Report      = 6,
	INS_TYPE_Get_File_Cell           = 7,
	INS_TYPE_Get_File_Cell_Report    = 8,
	INS_TYPE_Error                   = 9,
};
/**
 * @brief Defines the struct pointer of a TCP frame for file transfer.
 */
typedef struct FileIns
{
	InsType m_InsType;
	int    m_TotalBlockNum;
	int    m_CurrentBlockSerial;
	int    m_CellSize;
	int    m_ErrorCode;
	unsigned char m_CellContent[MAX_FILE_CELL_SIZE + 1];
	unsigned char CRC;
}*pFileIns;
/**
 * @brief Defines the class of to handle file transfer.
 */
class CFileOp  
{
public:
    /**
     * @brief Constructor of the class.
     */    
	CFileOp();
    /**
     * @brief Destructor of the class.
     */     
	virtual ~CFileOp();
    /**
     * @brief Handle a file transfer frame according to the file operation state.
     * @param [in] ins Pointer of a file transfer frame.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
	bool OnIns(FileIns* ins);
    /**
     * @brief Create a file transfer frame to send.
     * @param [in] lpath Absolute file path of local side.
     * @param [in] rpath Absolute file path of remote side.
     * @return Point to the file transfer frame.
     */    
	FileIns * OnSendFile(char* lpath,char* rpath);
    /**
     * @brief Create a file transfer frame to receive.
     * @param [in] lpath Absolute file path of local side.
     * @param [in] rpath Absolute file path of remote side.
     * @return Point to the file transfer frame.
     */    
	FileIns * OnRecvFile(char* lpath,char* rpath);
    /**
     * @brief Check if the file operation state is in File_OP_OK.
     * @retval true State is in File_OP_OK.
     * @retval false State is not in File_OP_OK.
     */        
	bool OnCheckStateOK();
    /**
     * @brief Set error flag to false.
     */    
	void OnReSetErrorTag();
    /**
     * @brief Check error flag.
     * @retval true The error flag is true.
     * @retval false The error flag is false.
     */     
	bool OnCheckErrorTag();
    /**
     * @brief Abort current file operation and set file operation state to File_OP_OK.
     */    
	void Empty();
    /**
     * @brief Set error flag to true.
     */    
	void SetErr();
    
	File_OP_State m_state;  /**< File operation state.*/
protected:
    /**
     * @brief Handle a file transfer frame in File_OP_OK state.
     * @param [in] ins Point to a file transfer frame.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
	bool OnStateOK(FileIns* ins);
    /**
     * @brief Handle a file transfer frame in File_OP_Send_SVR state.
     * @param [in] ins Point to a file transfer frame.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnStateSendSvr(FileIns* ins);
    /**
     * @brief Handle a file transfer frame in File_OP_Send_CLN state.
     * @param [in] ins Point to a file transfer frame.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnStateSendCln(FileIns* ins);
    /**
     * @brief Handle a file transfer frame in File_OP_Recv_SVR state.
     * @param [in] ins Point to a file transfer frame.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnStateRecvSvr(FileIns* ins);
    /**
     * @brief Handle a file transfer frame in File_OP_Recv_CLN state.
     * @param [in] ins Point to a file transfer frame.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnStateRecvCln(FileIns* ins);

	FILE*  m_fp;                    /**< File descriptor.*/
	long   m_ExpectSerial;          /**< Not used.*/
	long   m_SendSerial;            /**< Not used.*/
	long   m_SendTotalLen;          /**< Byte size of the file to send.*/
	long   m_SendTotalBlockNum;     /**< Number of blocks to send.*/
	bool   m_ErrorTag;              /**< Error flag.*/
	char   m_FilePath[513];         /**< Not used.*/
	
};

#endif
