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



  DCSS t;
  OnGetBuf(&t);
  printf("current state of A arm:%d\n",t.m_State[0].m_CurState);
  printf("cmd state of A arm:%d\n",t.m_State[0].m_CmdState);
  printf("error code of A arms:%d\n",t.m_State[0].m_ERRCode);
  
  printf("cmd of vel and acc:%d %d\n",t.m_In[0].m_Joint_Vel_Ratio,t.m_In[0].m_Joint_Acc_Ratio);
    

  // joints pose 
  double joints[7] = {0,0,0,0,0,0,10};
  OnClearSet();
  OnSetJointCmdPos_A(joints);
  OnSetSend();
  sleep(10);


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
