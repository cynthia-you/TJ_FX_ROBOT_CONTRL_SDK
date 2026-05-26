#ifndef FX_TCPFILECLIENT_H_ 
#define FX_TCPFILECLIENT_H_

/**
 * @file TCPFileClient.h
 * @brief The file defines the class of file transfer agent on client side.
 * @author lync
 */


#include "TCPAgent.h"
#include "FileOP.h"

/**
 * @brief Defines the class of file transfer agent on client side.
 */
class CTCPFileClient : public CTCPAgent  
{
public:
    /**
     * @brief Constructor of the class.
     */    
	CTCPFileClient();
    /**
     * @brief Destructor of the class.
     */    
	virtual ~CTCPFileClient();
    /**
     * @brief Copy a local file to remote.
     * @param [in] lpath Absolute file path of local side.
     * @param [in] rpath Absolute file path of remote side.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */      
	bool OnSendFile(char * lpath,char * rpath);
    /**
     * @brief Copy a remote file to local.
     * @param [in] lpath Absolute file path of local side.
     * @param [in] rpath Absolute file path of remote side.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnGetFile(char * lpath,char * rpath);
protected:
    /**
     * @brief Close file operation and set error flag.
     */ 
	virtual void OnDisconnect();
    /**
     * @brief Send data by socket.
     * @param [in] data Pointer of data.
     * @param [in] size Data byte size.
     */    
	virtual void OnRecvData(char * data,long size);
    
	CFileOp      m_fop;     /**< Object to handler file.*/
	long         m_inslen;  /**< Byte size of struct FileIns.*/

};

#endif
