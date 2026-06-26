#ifndef FX_FXCMPLOPT_H_
#define FX_FXCMPLOPT_H_

#ifdef CMPL_WIN
#include <windows.h>
#include <winsock.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define LOOPHANDLE HANDLE
#define socklen_t int
#include <mmsystem.h>
#include <atomic>
#endif

#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "winmm.lib")
#endif

#ifdef CMPL_LIN
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <pthread.h>
#include <errno.h>
#include <dirent.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <atomic>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define INVALID_HANDLE_VALUE -1
#define LOOPHANDLE int
#endif

#endif
