#ifndef FX_CMPLOPT_H_
#define FX_CMPLOPT_H_

// 移除默认的 CMPL_LIN 定义，让用户在编译时决定
// #define CMPL_LIN
// #define CMPL_WIN

// 或者更好的方式：让编译器命令行定义，例如 -DCMPL_WIN

#ifdef CMPL_WIN
    #include <winsock2.h>
    #include <windows.h>
    #include <stdio.h>
    #include <stdlib.h>
    #define LOOPHANDLE HANDLE
    #define socklen_t int
#endif

#ifdef CMPL_LIN
    #include <stdio.h>
    #include <stdlib.h>
    #include <stdint.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/mman.h>
    #include <sys/types.h>
    #include <sys/ipc.h>
    #include <sys/shm.h>
    #include <string.h>
    #include <sys/stat.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <sys/time.h>
    #include <sys/times.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <pthread.h>
    #include <errno.h>
    #include <sys/wait.h>
    #include <net/if.h>
    #include <netinet/tcp.h>
    #include <dirent.h>
    #include <dlfcn.h>
    #include <sys/ioctl.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define INVALID_HANDLE_VALUE -1
    #define LOOPHANDLE int
#endif

#endif