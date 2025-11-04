#include "MarvinSDK.h" 
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>

// '''#################################################################
// 该DEMO 为获取和设置参数案列

// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 获取整形参数&浮点形参数
//     5 设置整形参数&浮点形参数 
//     6 保存参数
//     7 任务完成,释放内存使别的程序或者用户可以连接机器人
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


  //获取整型参数
  char paraName[30]="R.A0.L0.BASIC.TorqueMax";
  long retValue=0;
  OnGetIntPara(paraName,&retValue);
  printf("int param: %ld\n", retValue);

  //获取浮点型参数
  char paraName1[30]="R.A1.L0.BASIC.SensorK";
  double retValue1=0.0;
  OnGetFloatPara(paraName1,&retValue1);
  printf("float param: %lf\n", retValue1);
  usleep(100000);

  //设置整型参数
  OnSetIntPara(paraName,retValue);
  usleep(100000);

  //设置浮点型参数
  OnSetFloatPara(paraName1, retValue1);
  usleep(100000);

  //保存参数
  OnSavePara();
  usleep(100000);


  //任务完成,释放内存使别的程序或者用户可以连接机器人
  OnRelease();
  return 1;
}
