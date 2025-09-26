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

  int dgType = 1;
  //   gType
  // # 0 退出拖动模式
  // # 1 关节空间拖动
  // # 2 笛卡尔空间x方向拖动
  // # 3 笛卡尔空间y方向拖动
  // # 4 笛卡尔空间z方向拖动
  // # 5 笛卡尔空间旋转方向拖动
  double force = 10 ;
  
  OnClearSet();
  OnSetDragSpace_A(dgType);
//  OnSetForceCmd_A(force);
  OnSetSend();
  usleep(100000);


  DCSS t;
  OnGetBuf(&t);
  printf("cmd of drag spcae type:%d\n",t.m_In[0].m_DragSpType);
  usleep(100000);

  OnRelease();
  usleep(100000);


  return 1;
}
