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

// '''#################################################################
// 该DEMO 为末端模组485通信控制案列
// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 设置位置模式和速度保障连接：听上始能声音
//     5 发送数据前，先清缓存
//     6 发送HEX数据到com1串口
//     7 每0.2秒接收com1串口的HEX数据
//     8 任务完成,释放内存使别的程序或者用户可以连接机器人
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
    DCSS t;

    // 查验连接是否成功
    bool init = OnLinkTo(192,168,1,190);
    if (!init) {
        std::cerr << "failed:端口占用，连接失败!" << std::endl;
        return -1;
    } else {

        //防总线通信异常,先清错
        usleep(100000);
        OnClearSet();
        OnClearErr_A();
        OnClearErr_B();
        OnSetSend();
        usleep(100000);

        int motion_tag = 0;
        int frame_update = 0;

        for (int i = 0; i < 5; i++) {
            OnGetBuf(&t);
            std::cout << "connect frames :" << t.m_Out[0].m_OutFrameSerial << std::endl;

            if (t.m_Out[0].m_OutFrameSerial != 0 &&
                frame_update != t.m_Out[0].m_OutFrameSerial) {
                motion_tag++;
                frame_update = t.m_Out[0].m_OutFrameSerial;
            }
            usleep(100000);
        }

        if (motion_tag > 0) {
            std::cout << "success:机器人连接成功!" << std::endl;
        } else {
            std::cerr << "failed:机器人连接失败!" << std::endl;
            return -1;
        }
    }

    //为了防止伺服有错，先清错
    OnClearSet();
    OnClearErr_A();
    OnSetSend();
    usleep(100000);

    //设置位置模式和速度保障连接：听上使能声音'
    OnClearSet();
    OnSetTargetState_A(1) ;
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    sleep(1);

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
