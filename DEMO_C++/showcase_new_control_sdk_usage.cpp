#include "MarvinSDK.h"
#include "FxRobot.h" // only used by run_pln_cart_space() case
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif

bool checkJointsReached(double target_joints[7], double current_joints[7], double tolerance = 0.05)
{
    for (int i = 0; i < 7; i++)
    {
        double error = std::abs(target_joints[i] - current_joints[i]);
        if (error >= tolerance)
        {
            return false;
        }
    }
    return true;
}

void hex_to_str(const unsigned char *data, int size, char *output, int output_size)
{
    int pos = 0;
    for (int i = 0; i < size && pos < output_size - 3; i++)
    {
        sprintf(output + pos, "%02X ", data[i]);
        pos += 3;
    }
    if (pos > 0)
    {
        output[pos - 1] = '\0';
    }
    else
    {
        output[0] = '\0';
    }
}

int hex_string_to_bytes(const char *hex_str, unsigned char *bytes, int max_bytes)
{
    int count = 0;
    char byte_str[3] = {0};
    const char *pos = hex_str;
    while (*pos && count < max_bytes)
    {
        while (*pos == ' ')
            pos++;
        if (!*pos)
            break;
        byte_str[0] = *pos++;
        if (!*pos)
            break;
        byte_str[1] = *pos++;
        bytes[count++] = (unsigned char)strtol(byte_str, NULL, 16);
    }
    return count;
}

bool link()
{
    int log_switch = 1; // ON
    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }
    else
    {
        OnRelease();
        return true;
    }
}

bool position()
{
    DCSS dcss;
    double fb_joints[7] = {0.0};
    // define parameters: which arm, vel and acc, target joints
    char arm = 'A';
    int run_vel = 50;
    int run_acc = 50;
    double target_joint[7] = {0, 0, 0, 0, 0, 0, 0};
    double target_joint1[7] = {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    if (!Connect(192, 168, 1, 190)) // default log_switch=0: log off
    {
        printf("---link failed---");
        return false;
    }

    // switch to position state
    if (!SetJointMode(arm, run_vel, run_acc))
    {
        printf("---set position failed---");
        return false;
    }
    SLEEP(500); // reserve time for switch to position state

    if (!SetJointPostionCmd(arm, target_joint))
    {
        printf("---set joint pose 1 failed---");
        return false;
    }

    SLEEP(50); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint, fb_joints)));

    SLEEP(100);
    if (!SetJointPostionCmd(arm, target_joint1))
    {
        printf("---set joint pose 2 failed---");
        return false;
    }

    SLEEP(50); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint1, fb_joints)));

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool joint_impedance()
{
    DCSS dcss;
    double fb_joints[7] = {0};
    // define parameters: which arm, vel and acc and k and d, target joints
    char arm = 'A';
    int log_switch = 1;
    double k[7] = {10, 10, 10, 1.6, 1, 1, 1};
    double d[7] = {0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4};
    int run_vel = 50;
    int run_acc = 50;
    double target_joint[7] = {0, 0, 0, 0, 0, 0, 0};
    double target_joint1[7] = {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }
    // switch to joint impedance state
    if (!SetImpJointMode(arm, run_vel, run_acc, k, d))
    {
        printf("---set joint impedance state failed---");
        return false;
    }
    SLEEP(500); // reserve time for switch to joint impedance state

    // send targe joints
    if (!SetJointPostionCmd(arm, target_joint))
    {
        printf("---set joint pose 1 failed---");
        return false;
    }
    SLEEP(20); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint, fb_joints)));

    // send targe joints
    if (!SetJointPostionCmd(arm, target_joint1))
    {
        printf("---set joint pose 2 failed---");
        return false;
    }
    SLEEP(20); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint1, fb_joints)));

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool cart_impedance()
{
    DCSS dcss;
    double fb_joints[7] = {0.0};
    // define parameters: which arm, vel and acc and k and d, target joints
    char arm = 'A';
    int log_switch = 1;
    double k[7] = {3000, 3000, 3000, 300, 300, 300, 1};
    double d[7] = {0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4};
    int run_vel = 50;
    int run_acc = 50;
    int RotType = 0;
    double CartCtrlPara[7] = {0};
    double target_joint[7] = {0, 0, 0, 0, 0, 0, 0};
    double target_joint1[7] = {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }
    // turn off log
    LogSwitch(0);
    // switch to cartesian impedance state
    if (!SetImpCartMode(arm, run_vel, run_acc, k, d, RotType, CartCtrlPara))
    {
        printf("---set cart impedance state failed---");
        return false;
    }
    SLEEP(500); // reserve time for switch to cartesian impedance state

    // send target joints
    if (!SetJointPostionCmd(arm, target_joint))
    {
        printf("---set joint pose 1 failed---");
        return false;
    }

    SLEEP(20); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint, fb_joints)));

    SLEEP(1000);
    if (!SetJointPostionCmd(arm, target_joint1))
    {
        printf("---set joint pose 2 failed---");
        return false;
    }
    SLEEP(20); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint1, fb_joints)));

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool force_impedance()
{
    DCSS dcss;
    double fb_joints[7] = {0.0};
    // define parameters: which arm, vel and acc and k and d, target joints, force control
    char arm = 'A';
    int log_switch = 1;
    double k[7] = {10, 10, 10, 1.6, 1, 1, 1};
    double d[7] = {0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4};
    int run_vel = 50;
    int run_acc = 50;
    double target_joint[7] = {69.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};
    double fxDir[6] = {0, 1, 0, 0, 0, 0};
    double fcAdjLmt = 50;
    double force = 50;

    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }

    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }

    // switch to joint impedance state
    if (!SetImpJointMode(arm, run_vel, run_acc, k, d))
    {
        printf("---set joint impedance state failed---");
        return false;
    }
    SLEEP(500); // reserve time for switch to joint impedance state

    // send targe joints
    if (!SetJointPostionCmd(arm, target_joint))
    {
        printf("---set joint pose 1 failed---");
        return false;
    }
    SLEEP(20); // waite robot accelarate then check low speed
    do
    {
        OnGetBuf(&dcss);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
        }
        // printf("low speed: %d,  current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
        //     dcss.m_Out[idx].m_LowSpdFlag,
        //     fb_joints[0], fb_joints[1], fb_joints[2], fb_joints[3], fb_joints[4], fb_joints[5], fb_joints[6]);
        SLEEP(1);
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(target_joint, fb_joints)));

    SLEEP(5000);
    if (!SetImpForceMode(arm, fxDir, fcAdjLmt))
    {
        printf("---set force parameters failed---");
        return false;
    }
    SLEEP(200); // reserved time takes effect
    if (!SetForceCmd(arm, force))
    {
        printf("---set force failed---");
        return false;
    }
    SLEEP(5000);

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool joint_drag()
{
    DCSS dcss;
    // define parameters: which arm, vel and acc and k and d, target joints
    char arm = 'A';
    double k[7] = {10, 10, 10, 1.6, 1, 1, 1};
    double d[7] = {0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4};
    int run_vel = 50;
    int run_acc = 50;
    int log_switch = 1;
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    int stage = 0;
    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }

    // switch to joint impedance state
    if (!SetImpJointMode(arm, run_vel, run_acc, k, d))
    {
        printf("---set joint impedance state failed---");
        return false;
    }
    SLEEP(500); // reserve time for switch to joint impedance state

    // switch to joint drag
    if (!SetJointDrag(arm))
    {
        printf("---entrance joint drag failed---");
        return false;
    }
    printf("---press and hold the end button to start dragging\n");
    while (true)
    {
        OnGetBuf(&dcss);
        if (dcss.m_Out[idx].m_TipDI == 1)
        {
            stage = 1;
            break;
        }
        SLEEP(1);
    }
    while (stage == 1)
    {
        OnGetBuf(&dcss);
        if (dcss.m_Out[idx].m_TipDI == 0)
        {
            SLEEP(5000);
            stage = 0;
        }
        SLEEP(1);
    }
    if (!ExitDrag(arm))
    {
        printf("---exit drag fialed--\n");
        return false;
    }

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool cart_z_drag_and_save_data(char *save_path)
{
    DCSS dcss;
    // define parameters: which arm, vel and acc and k and d, target joints, and other needing parameters
    char arm = 'A';
    int log_switch = 1;
    char type = 'Z';
    double k[7] = {3000, 3000, 3000, 300, 300, 300, 1};
    double d[7] = {0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4};
    int run_vel = 50;
    int run_acc = 50;
    int RotType = 0;
    double CartCtrlPara[7] = {0};
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    long targetNum = 7;
    long targetID[35] = {0, 1, 2, 3, 4, 5, 6,
                         0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0};
    long recordNum = 1000000;
    int stage = 0;
    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }

    // switch to cartesian impedance state
    if (!SetImpCartMode(arm, run_vel, run_acc, k, d, RotType = 0, CartCtrlPara))
    {
        printf("---set cart impedance state failed---");
        return false;
    }
    SLEEP(500); // reserve time for switch to cartesian impedance state

    // set cartesian drag "z"
    if (!SetCartDrag(arm, type))
    {
        printf("---entrance cart drag failed---");
        return false;
    }
    printf("---press and hold the end button to start dragging\n");
    while (true)
    {
        OnGetBuf(&dcss);
        if (dcss.m_Out[idx].m_TipDI == 1)
        {
            bool init = StartCollectData(targetNum, targetID, recordNum);
            if (!init)
            {
                printf("---collect data failed---\n");
                return false;
            }
            stage = 1;
            break;
        }
        SLEEP(1);
    }
    while (stage == 1)
    {
        OnGetBuf(&dcss);
        if (dcss.m_Out[idx].m_TipDI == 0)
        {
            bool init1 = StopCollectData();
            if (!init1)
            {
                printf("---stop collect data failed---\n");
                return false;
            }
            OnSaveGatherData(save_path);
            SLEEP(5000);
            stage = 0;
        }
        SLEEP(1);
    }
    if (!ExitDrag(arm))
    {
        printf("---exit drag fialed--\n");
        return false;
    }

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool run_pln_joint_space(char *config_path)
{
    DCSS dcss;
    double current_joints[7] = {0.0};
    // define parameters: which arm, vel and acc, target joints
    char arm = 'A';
    int log_switch = 1;
    int pln_vel = 0.5;
    int pln_acc = 0.5;
    double start_joint[7] = {69.02, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};
    double target_joint[7] = {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35};
    double fb_joints[7] = {0.0};
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    if (!Connect(192, 168, 1, 190, log_switch))
    {
        printf("---link failed---");
        return false;
    }

    // intialize joint pln
    if (!PlnInit(config_path))
    {
        printf("---PlnInit failed---");
        return false;
    }

    // get current joints
    OnGetBuf(&dcss);
    for (long joint = 0; joint < 7; joint++)
    {
        current_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
    }
    printf("current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.3lf\n",
           current_joints[0], current_joints[1], current_joints[2], current_joints[3], current_joints[4], current_joints[5], current_joints[6]);

    // if not in start joints, go to start joints
    if (!checkJointsReached(start_joint, current_joints))
    {
        // judge no plnning trajectory running
        do
        {
            OnGetBuf(&dcss);
            printf("trajectory state: %d \n", dcss.m_Out[idx].m_TrajState);
            SLEEP(1);
        } while (dcss.m_Out[idx].m_TrajState != 0);

        if (!RunPlnJoint(arm, current_joints, start_joint, pln_vel, pln_acc))
        {
            printf("---RunPlnJoint failed---");
            return false;
        }
    }

    SLEEP(1000);
    // Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.
    for (long j = 0; j < 5; j++)
    {
        printf("---iter---:%ld\n", j);
        do
        {
            OnGetBuf(&dcss);
            // printf("trajectory state: %d \n",dcss.m_Out[idx].m_TrajState);
            SLEEP(1);
        } while (dcss.m_Out[idx].m_TrajState != 0);

        if (!RunPlnJoint(arm, start_joint, target_joint, pln_vel, pln_acc))
        {
            printf("---RunPlnJoint failed---");
            return false;
        }
        SLEEP(2000);
        if (!StopPln(arm))
        {
            printf("---StopPln failed---");
            return false;
        }
        OnGetBuf(&dcss);
        printf("arm '%c' break at:[%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n", arm, dcss.m_Out[0].m_FB_Joint_Pos);
    }

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool run_pln_cart_space(char *config_path)
{
    DCSS dcss;
    double current_joints[7] = {0.0};
    // define parameters: which arm, vel and acc, target joints
    char arm = 'A';
    int log_on = 1;
    int pln_vel = 0.5;
    int pln_acc = 0.5;
    double start_joint[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    double fb_joints[7] = {0.0};
    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    if (!Connect(192, 168, 1, 190, log_on))
    {
        printf("---link failed---");
        return false;
    }

    // position state run to start joints
    if (!SetJointMode(arm, 100, 100))
    {
        printf("---set position failed---");
        return false;
    }
    SLEEP(200); // reserve time for switch to position state
    if (!SetJointPostionCmd(arm, start_joint))
    {
        printf("---set joint pose 1 failed---");
        return false;
    }
    SLEEP(3000);

    // get current joints
    OnGetBuf(&dcss);
    for (long joint = 0; joint < 7; joint++)
    {
        current_joints[joint] = dcss.m_Out[idx].m_FB_Joint_Pos[joint];
    }
    printf("current joints:%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
           current_joints[0], current_joints[1], current_joints[2], current_joints[3], current_joints[4], current_joints[5], current_joints[6]);

    // MOVLA在线直线规划步骤：
    // 1. 计算初始化
    // 2. 将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    // 3. 起点的末端位置姿态矩阵转为XYZABC
    // 4. 定义直线结束点的XYZABC，
    // 5. 运行在线规划,规划文件为50HZ执行

    // 1 计算初始化
    FX_INT32L i = 0;
    FX_INT32L j = 0;
    // 关闭打印日志a
    bool log_switch = false;
    FX_LOG_SWITCH(log_switch);
    // 导入运动学参数
    FX_INT32L TYPE[2];
    FX_DOUBLE GRV[2][3];
    FX_DOUBLE DH[2][8][4];
    FX_DOUBLE PNVA[2][7][4];
    FX_DOUBLE BD[2][4][3];
    FX_DOUBLE Mass[2][7];
    FX_DOUBLE MCP[2][7][3];
    FX_DOUBLE I[2][7][6];
    if (LOADMvCfg(config_path, TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_FALSE)
    {
        printf("Load CFG Error\n");
        return -1;
    }
    // 初始化运动学参数
    if (FX_Robot_Init_Type(idx, TYPE[0]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
        return -1;
    }
    if (FX_Robot_Init_Kine(idx, DH[0]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
        return -1;
    }
    if (FX_Robot_Init_Lmt(idx, PNVA[0], BD[0]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
        return -1;
    }
    // 2.将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    Matrix4 kine_pg;
    if (FX_Robot_Kine_FK(idx, start_joint, kine_pg) == FX_FALSE)
    {
        printf("Forward Kinematics Error\n");
        return -1;
    }

    // 3. 起点的末端位置姿态矩阵转为XYZABC
    Vect6 xyzabc = {0};
    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == FX_FALSE)
    {
        printf("matrix to xyzabc failed.");
        return -1;
    }
    Vect6 start = {0.0};
    for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
    }

    // 4. 定义直线结束点的XYZABC， showcase是规划了一个YZ平面的边长200mm的矩形
    // line-1
    Vect6 end = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }
    end[2] += 200;

    // 5. 运行在线规划,规划文件为50HZ
    CPointSet pset_movla;
    long freq = 50;
    if (FX_Robot_PLN_MOVLA(idx, start, end, current_joints, 300, 300, freq, &pset_movla) == FX_FALSE)
    {
        printf("MOVLA Error\n");
        return -1;
    }

    // 6. 下发点位
    if (!RunPlnCart(arm, &pset_movla))
    {
        printf("Failed to run cart plan\n");
        return -1;
    }
    SLEEP(200);

    // 7. 等待运动完成, 判断是否到达目标点位，并继续下一段规划
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
    } while (dcss.m_Out[0].m_TrajState != 0);

    // line-2
    CPointSet pset_movla1;
    {
        for (i = 0; i < 6; i++)
        {
            start[i] = end[i];
        }
        end[1] -= 200;

        long num1 = pset_movla.OnGetPointNum();
        double *joint1 = pset_movla.OnGetPoint(num1 - 1);
        start_joint[0] = joint1[0];
        start_joint[1] = joint1[1];
        start_joint[2] = joint1[2];
        start_joint[3] = joint1[3];
        start_joint[4] = joint1[4];
        start_joint[5] = joint1[5];
        start_joint[6] = joint1[6];

        if (FX_Robot_PLN_MOVLA(idx, start, end, start_joint, 300, 300, freq, &pset_movla1) == FX_FALSE)
        {
            printf("MOVLA Error\n");
            return -1;
        }

        if (!RunPlnCart(arm, &pset_movla1))
        {
            printf("Failed to run cart plan\n");
            return -1;
        }
    }
    SLEEP(20);
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
    } while (dcss.m_Out[idx].m_TrajState != 0);

    // line-3
    CPointSet pset_movla2;
    {
        for (i = 0; i < 6; i++)
        {
            start[i] = end[i];
        }
        end[2] -= 200;

        long num2 = pset_movla1.OnGetPointNum();
        double *joint2 = pset_movla1.OnGetPoint(num2 - 1);
        start_joint[0] = joint2[0];
        start_joint[1] = joint2[1];
        start_joint[2] = joint2[2];
        start_joint[3] = joint2[3];
        start_joint[4] = joint2[4];
        start_joint[5] = joint2[5];
        start_joint[6] = joint2[6];

        if (FX_Robot_PLN_MOVLA(idx, start, end, start_joint, 300, 300, freq, &pset_movla2) == FX_FALSE)
        {
            printf("MOVLA Error\n");
            return -1;
        }

        if (!RunPlnCart(arm, &pset_movla2))
        {
            printf("Failed to run cart plan\n");
            return -1;
        }
    }

    SLEEP(20);
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
    } while (dcss.m_Out[idx].m_TrajState != 0);

    // line-4
    CPointSet pset_movla3;
    {
        for (i = 0; i < 6; i++)
        {
            start[i] = end[i];
        }
        end[1] += 200;

        long num3 = pset_movla2.OnGetPointNum();
        double *joint3 = pset_movla2.OnGetPoint(num3 - 1);
        start_joint[0] = joint3[0];
        start_joint[1] = joint3[1];
        start_joint[2] = joint3[2];
        start_joint[3] = joint3[3];
        start_joint[4] = joint3[4];
        start_joint[5] = joint3[5];
        start_joint[6] = joint3[6];

        if (FX_Robot_PLN_MOVLA(idx, start, end, start_joint, 300, 300, freq, &pset_movla3) == FX_FALSE)
        {
            printf("MOVLA Error\n");
            return -1;
        }

        if (!RunPlnCart(arm, &pset_movla3))
        {
            printf("Failed to run cart plan\n");
            return -1;
        }
    }

    long num4 = pset_movla3.OnGetPointNum();
    double *joint4 = pset_movla3.OnGetPoint(num4 - 1);
    start_joint[0] = joint4[0];
    start_joint[1] = joint4[1];
    start_joint[2] = joint4[2];
    start_joint[3] = joint4[3];
    start_joint[4] = joint4[4];
    start_joint[5] = joint4[5];
    start_joint[6] = joint4[6];

    SLEEP(200);
    do
    {
        OnGetBuf(&dcss);
        SLEEP(1);
        for (long joint = 0; joint < 7; joint++)
        {
            fb_joints[joint] = dcss.m_Out[0].m_FB_Joint_Pos[joint];
        }
    } while (!(dcss.m_Out[idx].m_LowSpdFlag == 1 || checkJointsReached(start_joint, fb_joints)));

    // robot disable and release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    if (!Disable(arm))
    {
        printf("---disable failed---");
        return false;
    }
    OnRelease();
    return true;
}

bool set_tool()
{
    // set kinematics and dynamics infomations for two arms
    if (!Connect(192, 168, 1, 190)) // default log_switch=0: log off
    {
        printf("---link failed---");
        return false;
    }

    // The motion parameters are the offset and rotation of the tool relative to the center of the flange
    double kine_a[6] = {0.01, 0, 0, 0, 0, 0};
    double kine_b[6] = {0.02, 0, 0, 0, 0, 0};
    // The tool's dynamic parameters include mass center coordinates and inertia.
    double dyn_a[10] = {0.011, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double dyn_b[10] = {0.012, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (!SetTool('A', kine_a, dyn_a))
    {
        printf("---set A arm tool info failed---");
        return false;
    }
    if (!SetTool('B', kine_b, dyn_b))
    {
        printf("---set B arm tool info failed---");
        return false;
    }
    double current_kine_a[6] = {0};
    double current_dyn_a[10] = {0};
    double current_kine_b[6] = {0};
    double current_dyn_b[10] = {0};
    DCSS dcss;
    OnGetBuf(&dcss);
    for (int i = 0; i < 10; i++)
    {
        if (i < 6)
        {
            current_kine_a[i] = dcss.m_In[0].m_ToolKine[i];
            current_kine_b[i] = dcss.m_In[1].m_ToolKine[i];
        }
        current_dyn_a[i] = dcss.m_In[0].m_ToolDyn[i];
        current_dyn_b[i] = dcss.m_In[1].m_ToolDyn[i];
    }
    printf("arm A:\n");
    printf("kinematics: %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",
           current_kine_a[0],
           current_kine_a[1],
           current_kine_a[2],
           current_kine_a[3],
           current_kine_a[4],
           current_kine_a[5]);
    printf("dynamics: %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",
           current_dyn_a[0],
           current_dyn_a[1],
           current_dyn_a[2],
           current_dyn_a[3],
           current_dyn_a[4],
           current_dyn_a[5],
           current_dyn_a[6],
           current_dyn_a[7],
           current_dyn_a[8],
           current_dyn_a[9]);

    printf("\narm B:\n");
    printf("kinematics: %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",
           current_kine_b[0],
           current_kine_b[1],
           current_kine_b[2],
           current_kine_b[3],
           current_kine_b[4],
           current_kine_b[5]);
    printf("dynamics: %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",
           current_dyn_b[0],
           current_dyn_b[1],
           current_dyn_b[2],
           current_dyn_b[3],
           current_dyn_b[4],
           current_dyn_b[5],
           current_dyn_b[6],
           current_dyn_b[7],
           current_dyn_b[8],
           current_dyn_b[9]);

    // robot release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    OnRelease();
    return true;
}

bool eef_hands_cmd()
{
    // define parameters: which arm
    char arm = 'A';
    // send HEX data to channel com1. (set_ch=2 com1, set_ch=3 com2)
    const char *hex_str = "01 06 00 00 00 01 48 0A";
    long size_int;
    long set_ch = 2;
    unsigned char data_ptr[256] = {0};

    unsigned char data_buf[256];
    int data_size = 0;
    char hex_str1[512];
    int tag = 0;

    int idx = 0;
    if (arm == 'B')
    {
        idx = 1;
    }
    if (!Connect(192, 168, 1, 190)) // default log_switch=0: log off
    {
        printf("---link failed---");
        return false;
    }

    // clear cache before send data
    ClearChData(arm);
    SLEEP(100);

    // hex to bytes
    size_int = hex_string_to_bytes(hex_str, data_ptr, 256);
    printf("after hex_string_to_bytes:\n");
    for (int i = 0; i < size_int; i++)
    {
        printf("%02X ", data_ptr[i]);
    }
    printf("\ndata lenght: %ld\n", size_int);

    // send bytes to com
    if (!SetChData(arm, data_ptr, size_int, set_ch))
    {
        printf("---send data to com failed---");
        return false;
    }

    // recieve data from com
    while (true)
    {
        tag = OnGetChDataA(data_buf, &set_ch);
        if (tag > 0)
        {
            hex_to_str(data_buf, data_size, hex_str1, sizeof(hex_str1));
            printf("recieve data: %d, hex format: %s", tag, hex_str1);
        }
        SLEEP(1);
    }

    // robot release
    // After release, other programs or computers can connect to the robot. After release, if you want to control the robot again, you must reconnect.
    SLEEP(1000);
    OnRelease();
    return true;
}

int main()
{

    printf("----------------------------------------------------\n");
    printf("------------Concise SDK API showcases--------------\n");
    printf("Concise SDK APIe coexists compatibly with the old interface.\nPlease uncomment the showcases below one by one and compile and run them.\n");

    // // showcase1: connect to robot
    // printf("--------------------------\n");
    // printf("showcase: connect to robot:\n");
    // link();

    // //showcase2: set position state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}
    // printf("--------------------------\n");
    // printf("showcase: set position state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}\n");
    // position();

    // //showcase3: set joint impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}
    // printf("--------------------------\n");
    // printf("showcase3: set joint impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}\n");
    // joint_impedance();

    // //showcase4: set cartesian impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}
    // printf("--------------------------\n");
    // printf("showcase4: set cart impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}\n");
    // cart_impedance(); // //switch off some robot logs

    // // showcase5: set forcestate and run to {69.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}, Apply a force of 50N in the Y direction, over a range of 50 millimeters
    // printf("--------------------------\n");
    // printf("showcase5: set forcestate and run to {69.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}, Apply a force of 50N in the Y direction, over a range of 50 millimeters\n");
    // // Observation: Five seconds after reaching the position, a force of 50 N pulled the end of the arm down by 50 millimeters for 5 sec.
    // force_impedance();

    // // showcase6: set joint drag state
    // printf("--------------------------\n");
    // printf("showcase6: set joint drag state \n");
    // // USAGE:After entering drag mode, press and hold the end button to start dragging, release the button, and disable after 5 seconds.
    // joint_drag();

    // // ////showcase7: set cart drag state :direction: Z .then save drag data(max collect time:100 seconds).
    // printf("--------------------------\n");
    // printf("showcase7: set cart drag state :direction: Z \n");
    // //USAGE:After entering drag mode, press and hold the end button to start dragging, release the button, and disable after 5 seconds.
    // char save_path[] = "drag_cart_z.txt";
    // cart_z_drag_and_save_data(save_path);

    // //showcase8: In joint space: Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.
    // printf("--------------------------\n");
    // printf("showcase8: In joint space: Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.\n");
    // char config_path[] = "ccs_m6_40.MvKDCfg";
    // run_pln_joint_space(config_path);

    // //showcase9: In cartesian space: planning rectangular movement from the starting point.
    // printf("--------------------------\n");
    // printf("showcase9: In joint space: planning rectangular movement from the starting point.\n");
    // char config_path[] = "ccs_m6_40.MvKDCfg";
    // run_pln_cart_space(config_path);
    // g++ -g showcase_new_control_sdk_usage.cpp -o showcase_new_control_sdk_usage.exe -Ilib -L. -lMarvinSDK -lKine -lws2_32 -lwinmm

    // ////showcase10: set End-effector tools info :kinematics and dynamics
    // printf("--------------------------\n");
    // printf("showcase10: set End-effector tools info :kinematics and dynamics\n");
    // set_tool();

    // // ////showcase11: End-effector tool serial communication. Please note to replace the communication commands with the ones used by your own tool.
    // printf("--------------------------\n");
    // printf("showcase11: End-effector tool serial communication. Please note to replace the communication commands with the ones used by your own tool..\n");
    // eef_hands_cmd();

    printf("----------------------------------------------------\n");
    SLEEP(5000);
    return 1;
}
