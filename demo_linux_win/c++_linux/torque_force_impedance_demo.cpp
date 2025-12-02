#include "MarvinSDK.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <iostream>
#include <cstdlib>

// '''#################################################################
// 该DEMO 为力控案列

// 使用逻辑
//     1 初始化订阅数据的结构体
//     2 查验连接是否成功,失败程序直接退出
//     3 为了防止伺服有错，先清错
//     4 设置机器人运动控制模式为力控
//     5 设置力控参数
//     6 刷新订阅数据查看设置是否成功
//     7 订阅数据查看是否设置
//     8 下发力控指令
//     9 订阅数据查看是否设置
//     10 任务完成,释放内存使别的程序或者用户可以连接机器人
// '''#################################################################

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

    //设置机器人运动控制模式为力控
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode;
    OnSetImpType_A(3) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    usleep(100000);

    //设置力控参数：
    //设置在Y轴方向有5厘米的调节范围
    int fcType=0; // current only support 0
    double fxDirection[6] = {0, 1, 0, 0, 0, 0}; //前三个方向可调：X，Y，Z ；一次仅可控制一个方向
    double fcCtrlpara[7]={0, 0, 0, 0, 0, 0, 0}; //initial as 0
    double fcAdjLmt=5.0; // 5 厘米
    OnClearSet();
    OnSetForceCtrPara_A(fcType, fxDirection,fcCtrlpara,fcAdjLmt);
    OnSetSend();
    usleep(100000);

    //刷新订阅数据查看设置是否成功
    OnGetBuf(&t);
    printf("current state of A arm:%d\n",t.m_State[0].m_CurState);
    printf("cmd state of A arm:%d\n",t.m_State[0].m_CmdState);
    printf("error code of A arms:%d\n",t.m_State[0].m_ERRCode);
    printf("CMD of impedance:%d\n",t.m_In[0].m_ImpType);
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


    //力控指令
    //在Y轴方向有个10N的力一直拽着手臂提起5厘米， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
    OnClearSet();
    double force=10;
    OnSetForceCmd_A(force);
    OnSetSend();
    usleep(100000);

    //刷新订阅数据查看设置是否成功
    OnGetBuf(&t);
    printf("CMD of force fcAdjLmt=%lf\n",t.m_In[0].m_Force_AdjLmt);

    usleep(100000);


    //任务完成,释放内存使别的程序或者用户可以连接机器人
    OnRelease();
    return 1;
}
