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
  OnSetJointLmt_A(10, 10) ;
  OnSetSend();
  usleep(100000);


  OnClearSet();
  OnSetTargetState_A(1) ; //3:torque mode; 1:position mode
  OnSetSend();
  usleep(100000);


  char paraName[30]="R.A1.L0.BASIC.TorqueMax";
  long retValue=0;
  OnGetIntPara(paraName,&retValue);
  printf("int param: %ld\n", retValue);

  char paraName1[30]="R.A1.L0.BASIC.SensorK";
  double retValue1=0.0;
  OnGetFloatPara(paraName1,&retValue1);
  printf("float param: %lf\n", retValue1);
  usleep(100000);

  OnClearSet();
  OnSetTargetState_A(0) ; //3:torque mode; 1:position mode
  OnSetSend();
  usleep(100000);

  OnRelease();
  usleep(100000);


  return 1;
}
