#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
//'''#################################################################
//该DEMO 为在迪卡尔阻抗模式下,进去迪卡尔Y方向拖动,拖动并保存数据的控制案列
//
//使用逻辑
//    1 初始化订阅数据的结构体
//    2 初始化机器人接口
//    3 查验连接是否成功,失败程序直接退出
//    4 开启日志以便检查
//    5 为了防止伺服有错，先清错
//    6 进拖动前先切换扭矩模式和切到关节阻抗模式
//    7 设置拖动类型
//    8 订阅查看是否进入扭矩模式-->是否为迪卡尔阻抗模式-->拖动类型是否为迪卡尔Y拖动-->检测是否按下拖动按钮
//    9  第8步条件满足,设置保存关节轨迹并开始保存
//    10 拖动完成松开按钮即可,程序自动检测是否松开按钮,松开停止采集数据并保存数据到指定文件
//    11 拖动任务完成，退出拖动下使能
//    12 释放内存使别的程序或者用户可以连接机器人
//'''#################################################################

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

    //进关节拖动前先设置机器人运动控制模式为关节阻抗
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetSend();
    usleep(100000);
    OnClearSet();
    OnSetImpType_A(2) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    usleep(100000);



    //设置拖动类型
    int dgType = 3;
    //   gType
    // # 0 退出拖动模式
    // # 1 关节空间拖动
    // # 2 笛卡尔空间x方向拖动
    // # 3 笛卡尔空间y方向拖动
    // # 4 笛卡尔空间z方向拖动
    // # 5 笛卡尔空间旋转方向拖动
    OnClearSet();
    OnSetDragSpace_A(dgType);
    OnSetSend();
    usleep(100000);


    int stage1=1;
    int stage2=0;
    //是否进入扭矩模式-->是否为迪卡尔阻抗模式-->拖动类型是否为迪卡尔Y拖动-->检测是否按下拖动按钮, 满足条件: 设置保存数据参数并开始保存数据
    while (stage1==1)
    {
    OnGetBuf(&t);
    if (t.m_State[0].m_CurState==3)
    {
        if(t.m_In[0].m_ImpType==2)
        {
            if (t.m_In[0].m_DragSpType==2)
            {
                if(t.m_Out[0].m_TipDI==1)
                {
                    long targetNum=7;
                    long targetID[35]={0,1,2,3,4,5,6,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0};
                    long recordNum=1000;
                    OnStartGather(targetNum, targetID, recordNum);
                    usleep(100000);
                    stage2=1;
                    stage1=0;
                    break;
                }
            }
        }
    }
    usleep(100000);
    }

    //检测是否松开拖动按钮,松开停止数据采集
    while (stage2==1)
    {
        OnGetBuf(&t);
        if(t.m_Out[0].m_TipDI==0)
        {
          OnClearSet();
          OnStopGather();
          OnSetSend();
          usleep(500000);
          stage2=0;
          break;
        }
    }


    //保存采集数据
    char save_path[] = "drag_cart_y.txt";
    OnSaveGatherData(save_path);

    sleep(5);

    //拖动任务完成，退出拖动下使能
    OnClearSet();
    OnSetImpType_A(0) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    usleep(100000);
    OnClearSet();
    OnSetTargetState_A(0) ; //3:torque mode; 1:position mode
    OnSetSend();
    usleep(100000);

    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}
