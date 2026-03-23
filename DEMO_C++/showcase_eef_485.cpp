#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#ifdef _WIN32
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif
// '''#################################################################
// 该DEMO 为末端模组485通信控制案列
// 使用逻辑
//    初始化订阅数据的结构体
//    查验连接是否成功,失败程序直接退出
//    发送数据前，先清缓存
//    发送HEX数据到com1串口
//    每0.2秒接收com1串口的HEX数据
//    任务完成,释放内存使别的程序或者用户可以连接机器人

// 建议的做法是： 连接机器人清缓存后，先起一个线程读回末端工具发出的数据，再起一个线程，在这个线程中发送数据。
// '''#################################################################

// 将十六进制数据转换为字符串
void hex_to_str(const unsigned char* data, int size, char* output, int output_size) {
    int pos = 0;
    for (int i = 0; i < size && pos < output_size - 3; i++) {
        // 每个字节转换为两个十六进制字符
        sprintf(output + pos, "%02X ", data[i]);
        pos += 3;
    }
    if (pos > 0) {
        output[pos - 1] = '\0'; // 替换最后一个空格为结束符
    } else {
        output[0] = '\0';
    }
}

// 将十六进制字符串转换为字节数组
int hex_string_to_bytes(const char* hex_str, unsigned char* bytes, int max_bytes) {
    int count = 0;
    char byte_str[3] = {0};
    const char* pos = hex_str;

    while (*pos && count < max_bytes) {
        // 跳过空格
        while (*pos == ' ') pos++;
        if (!*pos) break;

        // 提取两个字符作为一个字节
        byte_str[0] = *pos++;
        if (!*pos) break; // 确保有第二个字符
        byte_str[1] = *pos++;

        // 转换为字节
        bytes[count++] = (unsigned char)strtol(byte_str, NULL, 16);
    }

    return count;
}

int main()
{
   // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed to connect to the robot, port is occupied" << std::endl;
        return -1;
    }

    SLEEP(200);
    //检查伺服和手臂是否有错，有错误清错
    //订阅最新数据获取机械臂的错误和状态，有错误清错
    OnGetBuf(&dcss);
    int arm_error_a=dcss.m_State[0].m_ERRCode;
    int arm_error_b=dcss.m_State[1].m_ERRCode;
    int arm_state_a=dcss.m_State[0].m_CurState;
    int arm_state_b=dcss.m_State[1].m_CurState;
   if (arm_error_a!=0 || arm_state_a==100)
    {
        std::cout << "arm A: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    }
    if (arm_error_b!=0 || arm_state_b==100)
    {
        std::cout << "arm B: exits error, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    } 

    //获取伺服错误，有错误清错
    long ErrCode_A[7]={};
    long ErrCode_B[7]={};
    OnGetServoErr_A(ErrCode_A);
    OnGetServoErr_B(ErrCode_B);
    bool allZero_a = true;
    bool allZero_b = true;
    for (int i = 0; i < 7; ++i) 
    {
        if (ErrCode_A[i] != 0) {
            allZero_a = false;
            break;
        }
    }
    for (int i = 0; i < 7; ++i) 
    {
        if (ErrCode_B[i] != 0) {
            allZero_b = false;
            break;
        }
    }
    if (allZero_a)
    {
        std::cout << "arm A: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_A();
        OnSetSend();
        SLEEP(20);
    } 
    if (allZero_b)
    {
        std::cout << "arm B: srvo error exists, clear error\n" << std::endl;
        SLEEP(20);
        OnClearSet();
        OnClearErr_B();
        OnSetSend();
        SLEEP(20);
    }
    
    //通过确认freame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）
    int motion_tag = 0;
    int frame_update = 0;

    for (int i = 0; i < 5; i++) {
        OnGetBuf(&dcss);
        std::cout << "connect frames:" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

        if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
            frame_update != dcss.m_Out[0].m_OutFrameSerial) {
            motion_tag++;
            frame_update = dcss.m_Out[0].m_OutFrameSerial;
        }
        SLEEP(1);
    }
    if (motion_tag > 0) {
        std::cout << "success:robot connected\n" << std::endl;
    } else {
        std::cerr << "failed:robot connection failed\n"<< std::endl;
        OnRelease();
        return -1;
    }

     //开启日志
    OnLogOn();
    OnLocalLogOn();


    //发送数据前，先清缓存
    OnClearChDataA();
    usleep(500000); 


    //发送HEX数据到com1串口，通道set_ch设为2; COM2,通道set_ch设为3
    const char* hex_str = "01 06 00 00 00 01 48 0A";
    unsigned char data_ptr[256] = {0};
    long size_int;
    long set_ch = 2;

    // 转换十六进制字符串为字节数组
    size_int = hex_string_to_bytes(hex_str, data_ptr, 256);
    printf("转换后的字节数据: ");
    for (int i = 0; i < size_int; i++) {
        printf("%02X ", data_ptr[i]);
    }
    printf("\n数据长度: %ld\n", size_int);

    // 发送HEX数据到com1串口
    bool result = OnSetChDataA(data_ptr, size_int, set_ch);
    printf("函数执行结果: %s\n", result ? "成功" : "失败");

    // 接收com1串口的HEX数据
    long set_ch1 = 2;
    unsigned char data_buf[256]; 
    int data_size = 0;
    char hex_str1[512]; 
    while (true) {
        int tag = OnGetChDataA(data_buf, &set_ch1);
        sleep(0.2);
        if (tag >= 1) {
            hex_to_str(data_buf, data_size, hex_str1, sizeof(hex_str1));
            printf("接收信号: %d, 接收的HEX数据: %s", tag, hex_str1);
        }
    }

    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
 }
