#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include "unistd.h"


int main()
{
  if(OnLinkTo(192,168,1,190) == false)
  {
    printf("link err \n");
    return -1;
  }

  usleep(100000);

  int fcType=0; // current only support 0
  double fxDirection[6] = {0, 0, 1, 0, 0, 0}; //前三个方向可调：X，Y，Z ；一次仅可控制一个方向
  double fcCtrlpara[7]={0, 0, 0, 0, 0, 0, 0}; //initial as 0
  double fcAdjLmt=5.0; // 5 厘米
  double force=10;

  OnClearSet();
  //这两条指令搭配使用才有力控的效果,设置是在Y轴方向有个2斤的力一直拽着手臂提起5厘米， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
  OnSetForceCtrPara_A(fcType, fxDirection,fcCtrlpara,fcAdjLmt);
  OnSetForceCmd_A(force);
  OnSetSend();
  usleep(100000);

  OnClearSet();
  OnSetJointLmt_A(10, 10) ;
  OnSetSend();
  usleep(100000);


  OnClearSet();
  OnSetTargetState_A(3) ; //3:torque mode; 1:position mode; 
  OnSetImpType_A(3) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
  OnSetSend();
  usleep(100000);

  DCSS t;
  OnGetBuf(&t);
  printf("current state of A arm:%d\n",t.m_State[0].m_CurState);
  printf("cmd state of A arm:%d\n",t.m_State[0].m_CmdState);
  printf("error code of A arms:%d\n",t.m_State[0].m_ERRCode);

  printf("CMD of imdepancd:%d\n",t.m_In[0].m_ImpType);
  printf("CMD of vel and acc:%d %d\n",t.m_In[0].m_Joint_Vel_Ratio,t.m_In[0].m_Joint_Acc_Ratio);

    printf("CMD of force type=%d\n",t.m_In[0].m_Force_Type);
  
  printf("CMD of force fxDirection=[%lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Force_Dir[0],
                                                          t.m_In[0].m_Force_Dir[1],
                                                          t.m_In[0].m_Force_Dir[2],
                                                          t.m_In[0].m_Force_Dir[3],
                                                          t.m_In[0].m_Force_Dir[4],
                                                          t.m_In[0].m_Force_Dir[5]);

  printf("CMD of force fcCtrlpara=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Force_PIDUL[0],
                                                          t.m_In[0].m_Force_PIDUL[1],
                                                          t.m_In[0].m_Force_PIDUL[2],
                                                          t.m_In[0].m_Force_PIDUL[3],
                                                          t.m_In[0].m_Force_PIDUL[4],
                                                          t.m_In[0].m_Force_PIDUL[5],
                                                          t.m_In[0].m_Force_PIDUL[6]);

  printf("CMD of force fcAdjLmt=%lf\n",t.m_In[0].m_Force_AdjLmt);


  // joints pose 
  double joints[7] = {10,20,30,40,50,60,70};
  OnClearSet();
  OnSetJointCmdPos_A(joints);
  OnSetSend();
  usleep(100000);


  OnGetBuf(&t);
  printf("CMD joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",t.m_In[0].m_Joint_CMD_Pos[0],
                                                              t.m_In[0].m_Joint_CMD_Pos[1],
                                                              t.m_In[0].m_Joint_CMD_Pos[2],
                                                              t.m_In[0].m_Joint_CMD_Pos[3],
                                                              t.m_In[0].m_Joint_CMD_Pos[4],
                                                              t.m_In[0].m_Joint_CMD_Pos[5],
                                                              t.m_In[0].m_Joint_CMD_Pos[6]);
                                                              
  printf("current joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",t.m_Out[0].m_FB_Joint_Pos[0],
                                                                    t.m_Out[0].m_FB_Joint_Pos[1],
                                                                    t.m_Out[0].m_FB_Joint_Pos[2],
                                                                    t.m_Out[0].m_FB_Joint_Pos[3],
                                                                    t.m_Out[0].m_FB_Joint_Pos[4],
                                                                    t.m_Out[0].m_FB_Joint_Pos[5],
                                                                    t.m_Out[0].m_FB_Joint_Pos[6]);
  usleep(100000);


  OnRelease();
  usleep(100000);

  return 1;
}
