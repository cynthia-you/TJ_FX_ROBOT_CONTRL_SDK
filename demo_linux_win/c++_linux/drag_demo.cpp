#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>
// '''#################################################################
// 该DEMO 为拖动控制案列

// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 设置拖动类型
//     5 订阅查看设置是否成功
//     6 拖动,拖动结束,退出拖动
//     7 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################


int main()
{
  // 初始化订阅数据的结构体
  DCSS t;

  // 查验连接是否成功
  int init = OnLinkTo(192,168,1,190);
  if (init == -1) {
      std::cerr << "failed:端口占用，连接失败!" << std::endl;
      return -1;
  } else {
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


  //设置拖动类型
  int dgType = 1;
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

  //刷新订阅数据查看设置是否成功
  OnGetBuf(&t);
  printf("cmd of drag spcae type:%d\n",t.m_In[0].m_DragSpType);

  usleep(100000);//模拟拖动时长

  //拖动结束,退出拖动
  OnClearSet();
  OnSetDragSpace_A(0);
  OnSetSend();
  usleep(100000);

 //任务完成,释放内存使别的程序或者用户可以连接机器人
  OnRelease();
  return 1;
}
