#ifndef FX_FILIE_PORTAL_H_
#define FX_FILIE_PORTAL_H_

/**
 * @file FilePortal.h
 * @brief The file includes the interface of file transfer.
 * @author lync
 */


/**
 * @brief Transfer a local file to a remote host.
 * @param [in] ip1 The first section of ip address.
 * @param [in] ip2 The second section of ip address.
 * @param [in] ip3 The third section of ip address.
 * @param [in] ip4 The fourth section of ip address.
 * @param [in] local_file Absolute file path of the local file.
 * @param [in] remote_file Absolute file path of the remote file.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool SendFile (unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4,char * local_file,char * remote_file);
/**
 * @brief Transfer a remote to a local host.
 * @param [in] ip1 The first section of ip address.
 * @param [in] ip2 The second section of ip address.
 * @param [in] ip3 The third section of ip address.
 * @param [in] ip4 The fourth section of ip address.
 * @param [in] local_file Absolute file path of the local file.
 * @param [in] remote_file Absolute file path of the remote file.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool RecvFile (unsigned char ip1,unsigned char ip2,unsigned char ip3,unsigned char ip4,char * local_file,char * remote_file);


#endif

