#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"

int main()
{
  if(OnLinkTo(192,168,1,190) == false)
  {
    printf("link err \n");
    return -1;
  }


  OnClearSet();
  OnSetTargetState_A(2) ; //3:torque mode; 1:position mode；2：pvt
  OnSetSend();
  usleep(100000);

  char* path = "DEMO_SRS_Left.fmv"; //改成你的绝对路径
  long serial=27;
  bool re=false;
  re=OnSendPVT_A(path,serial);
  printf("re =%d\n",re);
  usleep(100000);

  //save data
  long targetNum=7;
  long targetID[35]={0,1,2,3,4,5,6,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0};
  long recordNum=1000;
  OnStartGather(targetNum, targetID, recordNum);
  usleep(100000);

  int id=27;
  OnClearSet();
  OnSetPVT_A(id);
  OnSetSend();
  sleep(10);//模拟执行时长




  //save data as txt
  usleep(100000);
  char* save_path="aaa.txt"; //改成你的绝对路径
  OnSaveGatherData(save_path);

  usleep(100000);
  OnRelease();

  return 1;
}
