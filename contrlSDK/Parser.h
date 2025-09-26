#ifndef FX_PARSER_H_ 
#define FX_PARSER_H_

/**
 * @file Parser.h
 * @brief The file defines the base class to handle raw TCP file transfer frames.
 * @author lync
 */

#include "stdio.h" 
#include "stdlib.h"

#define MAX_PARSER_BUFSIZE 81000    /**< Maximum byte size of a TCP frame for file transfer.*/
/**
 * @brief Defines the class of the application layer protocal for file transfer.
 */
class CParser  
{
public:
    /**
     * @brief Constructor of the class.
     */    
	CParser();
    /**
     * @brief Destructor of the class.
     */    
	virtual ~CParser();
    /**
     * @brief Splice raw TCP frames.
     * @param [in] data Pointer of raw data.
     * @param [in] size Byte length of raw data.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnAddRawData(char* data, long size);
    /**
     * @brief Unpack a frame from the raw TCP data stream.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
	bool OnUnPack();
    /**
     * @brief Pack a frame with application header and crc tail.
     * @param [in] data Pointer of raw data.
     * @param [in] size Byte Length of raw data.
     * @param [out] ret_size Byte length of packed data.
     * @return Pointer of packed data.
     */    
	char * OnPack(char* data, long size, long& ret_size);
    /**
     * @brief Get the unpacked frame's context.
     * @param [out] size Byte length of the context.
     * @return Pointer of the context.
     */     
	char * OnGetContent(long& size);
protected:
	char * m_raw_data;              /**< Pointer of raw TCP data stream buffer.*/
	long   m_raw_data_len;          /**< Valid byte size of raw TCP data stream.*/
	long   m_raw_data_start_pos;    /**< Start offset of raw TCP data stream in the buffer.*/
	char * m_context;               /**< Pointer of frame's context buffer.*/
	long   m_context_len;           /**< Byte size of frame's context buffer.*/
	
	char * m_maked_buf;             /**< Pointer of packed data buffer.*/
	unsigned char * uc_len_1;       /**< Pointer of hundreds place of the size of a frame's context.*/
	unsigned char * uc_len_2;       /**< Pointer of decade place of the size of a frame's context.*/
	unsigned char * uc_len_3;       /**< Pointer of unit place of the size of a frame's context.*/
};


#endif
