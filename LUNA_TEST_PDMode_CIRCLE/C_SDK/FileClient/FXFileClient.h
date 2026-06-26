#ifndef FX_FILECLIENT_H_
#define FX_FILECLIENT_H_

#if defined(_WIN32) || defined(_WIN64)
#define FX_FileClient_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define FX_FileClient_SDK_API
#endif

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief Send a file to a remote file server.
 *
 * This function transmits a specified local file to a remote server
 * identified by an IPv4 address, and stores it on the server under
 * the given remote file path.
 *
 * @param[in] ip1 First octet of the server IPv4 address.
 * @param[in] ip2 Second octet of the server IPv4 address.
 * @param[in] ip3 Third octet of the server IPv4 address.
 * @param[in] ip4 Fourth octet of the server IPv4 address.
 * @param[in] local_file  Path to the source file on the local system.
 * @param[in] remote_file Destination path where the file will be stored on the remote server.
 *
 * @return A status code indicating the result of the operation.
 *         Typically, 1 indicates success while 0 values indicate errors.
 */
FX_FileClient_SDK_API int FX_FileClient_SendFile(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, char* local_file, char* remote_file);
/**
 * @brief Receive a file from a remote file server.
 *
 * This function retrieves a specified file from a remote server
 * identified by an IPv4 address and saves it to the local system
 * under the given local file path.
 *
 * @param[in] ip1 First octet of the server IPv4 address.
 * @param[in] ip2 Second octet of the server IPv4 address.
 * @param[in] ip3 Third octet of the server IPv4 address.
 * @param[in] ip4 Fourth octet of the server IPv4 address.
 * @param[in] local_file  Path where the received file will be stored on the local system.
 * @param[in] remote_file Path of the source file on the remote server.
 *
 * @return A status code indicating the result of the operation.
 *         Typically, 1 indicates success while 0 values indicate errors.
 */
FX_FileClient_SDK_API int FX_FileClient_RecvFile(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, char* local_file, char* remote_file);

#ifdef __cplusplus
}
#endif

#endif

