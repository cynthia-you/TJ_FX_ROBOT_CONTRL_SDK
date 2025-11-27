#ifndef FX_TCPAGENT_H_ 
#define FX_TCPAGENT_H_

/**
 * @file TCPAgent.h
 * @brief The file defines the class of agent on client side.
 * @author lync
 */

#include "CmplOpt.h"
#include "Parser.h"
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include "string.h"
#include "netdef.h"

using namespace std;

/**
 * @brief Wait until timeout.
 * @param [in] usecond Time duration to wait, unit in ms.
 */
void UninetSleep(long usecond);

/**
 * @brief Defines the class of agent on client side.
 */
class CTCPAgent  
{
public:
    /**
     * @brief Constructor of the class.
     */     
	CTCPAgent();
    /**
     * @brief Destructor of the class.
     */    
	virtual ~CTCPAgent();
    /**
     * @brief Sponsor a connection to file server.
     * @param [in] ip1 The first section of ip address.
     * @param [in] ip2 The second section of ip address.
     * @param [in] ip3 The third section of ip address.
     * @param [in] ip4 The fourth section of ip address.
     * @param [in] port IP port.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnLinkTo(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, unsigned long port);
    /**
     * @brief Send data to file server.
     * @param [in] buf Pointer of data.
     * @param [in] size Data byte size.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
    bool OnSend(char* buf, long size);
    /**
     * @brief Check if the connection is built up.
     * @retval true The connection is ready.
     * @retval false The connection is not ready.
     */    
	bool OnCheckLink();
    /**
     * @brief Break the connection and close the socket.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnQuit();
protected:
    /**
     * @brief Thread function to receive data from file server.
     */    
	static void NetLoop(void* p); 
    /**
     * @brief Close socket.
     */    
	virtual void OnDisconnect();
    /**
     * @brief Print data to stdout.
     * @param [in] data Pointer of data.
     * @param [in] size Data byte size.
     */    
	virtual void OnRecvData(char * data,long size);
    /**
     * @brief Initialize running enviroment(only valid in windows OS).
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
    bool OnInitNet();
    
	bool m_bLinkTag;        /**< Flag to denote if the connection is built up.*/
	SOCKET m_iSocket;       /**< Socket file descriptor.*/
	bool m_bNetInitTag;     /**< Flag to denote if the running enviroment is initialized.*/
	LOOPHANDLE m_LoopThreadHandle; /**< Thread handler.*/
	char * SrvUtText;       /**< Point to the socket receiving buffer.*/
	bool m_quit_tag;        /**< Flag to denote if it is on quit.*/
	CParser * m_parser;     /**< Point to the object for handling TCP file protocal.*/
};

#endif
