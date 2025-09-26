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
    if(OnLinkTo(192,168,1,190) == false)
    {
    printf("link err \n");
    return -1;
    }

    //'''设置位置模式和速度保障连接：听上私服声音'''
    OnClearSet();
    OnSetTargetState_A(1) ;
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    sleep(1);

    //发送数据前，先清缓存
    OnClearChDataA();
    sleep(1);


    //发送HEX数据到com1串口，通道set_ch设为2; COM2,通道set_ch设为3
    //CAN  CANFD相同接口， 通道set_ch设为1
      // 十六进制字符串
    const char* hex_str = "01 06 00 00 00 01 48 0A";
    // 准备数据缓冲区
    unsigned char data_ptr[256] = {0};
    long size_int;
    long set_ch = 2;

    // 转换十六进制字符串为字节数组
    size_int = hex_string_to_bytes(hex_str, data_ptr, 256);

    // 输出转换结果以供验证
    printf("bytes data: ");
    for (int i = 0; i < size_int; i++) {
        printf("%02X ", data_ptr[i]);
    }
    printf("\nlength of bytes data: %ld\n", size_int);

    // 调用函数
    bool result = OnSetChDataA(data_ptr, size_int, set_ch);

    // 输出结果
    printf("OnSetChDataA result: %s\n", result ? "success" : "failed");



    long set_ch1 = 2;
    unsigned char data_buf[256]; // 数据缓冲区
    int data_size = 0;
    char hex_str1[512]; // 十六进制字符串缓冲区

    while (true) {
        // 调用获取数据的函数
        int tag = OnGetChDataA(data_buf, &set_ch1);
        // 等待 200 毫秒
        sleep(0.2);
        // 如果有数据，则记录日志
        if (tag >= 1) {
            // 将十六进制数据转换为字符串
            hex_to_str(data_buf, data_size, hex_str1, sizeof(hex_str1));

            // 格式化日志消息
            printf("receive from eef: %d, HEX: %s", tag, hex_str1);
        }
    }

    return 0;

 }
