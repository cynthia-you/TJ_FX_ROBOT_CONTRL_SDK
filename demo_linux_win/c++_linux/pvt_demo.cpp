#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
// '''#################################################################
// 该DEMO 为跑PVT轨迹并保存数据的案列

// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 设置PVT模式
//     5 订阅查看设置是否成功
//     6 设置PVT 轨迹本机路径 和PVT号
//     7 机器人运动前开始设置保存数据并开始采集数据
//     8 设置运行的PVT号并立即执行PVT轨迹
//     9 保存采集数据
//     10 任务完成,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################

int main()
{
  // 初始化订阅数据的结构体
    DCSS dcss;

    // 查验连接是否成功
    int init = OnLinkTo(192,168,1,190);
    if (init == -1) {
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
            OnGetBuf(&dcss);
            std::cout << "connect frames :" << dcss.m_Out[0].m_OutFrameSerial << std::endl;

            if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
                frame_update != dcss.m_Out[0].m_OutFrameSerial) {
                motion_tag++;
                frame_update = dcss.m_Out[0].m_OutFrameSerial;
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


    //设置PVT模式
    OnClearSet();
    OnSetTargetState_A(2) ; //3:torque mode; 1:position mode；2：pvt
    OnSetSend();
    usleep(100000);


    //选择PVT轨迹文件和设置PVT号
    char* path = "DEMO_SRS_Left.fmv"; //改成你的绝对路径
    long serial=27;
    bool re=false;
    re=OnSendPVT_A(path,serial);
    printf("send pvt return =%d\n",re);
    usleep(100000);

    //设置保存数据的n参数并开始采集数据
                    // targetNum采集列数 （1-35列）
                    // targetID[35] 对应采集数据ID序号  
                    //           左臂序号：
                    //               0-6  	左臂关节位置 
                    //               10-16 	左臂关节速度
                    //               20-26   左臂外编位置
                    //               30-36   左臂关节指令位置
                    //               40-46	左臂关节电流（千分比）
                    //               50-56   左臂关节传感器扭矩NM
                    //               60-66	左臂摩擦力估计值
                    //               70-76	左臂摩檫力速度估计值
                    //               80-85   左臂关节外力估计值
                    //               90-95	左臂末端点外力估计值
                    //           右臂对应 + 100
                    // recordNum  采集行数 ，小于1000会采集1000行，设置大于一百万行会采集一百万行

    long targetNum=7;
    long targetID[35]={0,1,2,3,4,5,6,
        0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,
        0,0,0,0,0,0,0};
    long recordNum=1000;
    OnStartGather(targetNum, targetID, recordNum);
    usleep(100000);


    //执行指定的PVT号
    int id=27;
    OnClearSet();
    OnSetPVT_A(id);
    OnSetSend();
    sleep(10);//模拟执行时长


    //保存数据为TXT
    char* save_path="aaa.txt"; //改成你的绝对路径
    OnSaveGatherData(save_path);
    usleep(100000);

    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}
