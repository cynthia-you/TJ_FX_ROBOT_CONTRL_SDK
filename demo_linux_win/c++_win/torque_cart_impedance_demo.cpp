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

  sleep(1);
  double K[7] = {2000,2000,2000,10,10,10,0}; //预设为参数最大上限，供参考。
  double D[7] = {0.1,0.1,0.1,0.3,0.3,1};//预设为参数最大上限，供参考。
  int type = 2; //type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控

  OnClearSet();
  OnSetCartKD_A(K, D,type) ;
  OnSetSend();
  sleep(1);


  OnClearSet();
  OnSetJointLmt_A(10, 10) ;
  OnSetSend();
  sleep(1);


  OnClearSet();
  OnSetTargetState_A(3) ; //3:torque mode; 1:position mode; 
  OnSetImpType_A(2) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
  OnSetSend();
  sleep(1);

  DCSS t;
  OnGetBuf(&t);
  printf("current state of A arm:%d\n",t.m_State[0].m_CurState);
  printf("cmd state of A arm:%d\n",t.m_State[0].m_CmdState);
  printf("error code of A arms:%d\n",t.m_State[0].m_ERRCode);

  printf("CMD of imdepancd:%d\n",t.m_In[0].m_ImpType);
  printf("CMD of vel and acc:%d %d\n",t.m_In[0].m_Joint_Vel_Ratio,t.m_In[0].m_Joint_Acc_Ratio);
  
  printf("CMD of cart D=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Cart_K[0],
                                                          t.m_In[0].m_Cart_K[1],
                                                          t.m_In[0].m_Cart_K[2],
                                                          t.m_In[0].m_Cart_K[3],
                                                          t.m_In[0].m_Cart_K[4],
                                                          t.m_In[0].m_Cart_K[5],
                                                          t.m_In[0].m_Cart_K[6]);

  printf("CMD of cart D=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Cart_D[0],
                                                          t.m_In[0].m_Cart_D[1],
                                                          t.m_In[0].m_Cart_D[2],
                                                          t.m_In[0].m_Cart_D[3],
                                                          t.m_In[0].m_Cart_D[4],
                                                          t.m_In[0].m_Cart_D[5],
                                                          t.m_In[0].m_Cart_D[6]);

  printf("CMD of cart type=%d\n",t.m_In[0].m_Cart_KD_Type);


  // joints pose 
  double joints[7] = {10,20,30,40,50,60,70};
  OnClearSet();
  OnSetJointCmdPos_A(joints);
  OnSetSend();
  sleep(5);


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
  sleep(1);


  OnRelease();
  sleep(1);

  return 1;
}
