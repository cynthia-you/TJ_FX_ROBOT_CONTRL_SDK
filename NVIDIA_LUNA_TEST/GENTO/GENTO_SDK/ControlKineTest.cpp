#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "L1Robot.h"

const int MAX_POINT = 5000; //!!!!最多允许5000个轨迹点

#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif

static void print_matrix(const char *name, const double mat[16])
{
    printf("%s:\n", name);
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            printf("%8.4f ", mat[i * 4 + j]);
        }
        printf("\n");
    }
}

static void print_xyzabc(const char *name, const double xyzabc[6])
{
    printf("%s: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
           name, xyzabc[0], xyzabc[1], xyzabc[2],
           xyzabc[3], xyzabc[4], xyzabc[5]);
}

static void print_vector7(const char *name, const double vec[7])
{
    printf("%s: [", name);
    for (int i = 0; i < 7; ++i)
        printf("%.4f ", vec[i]);
    printf("]\n");
}

static void print_vector3(const char *name, const double vec[3])
{
    printf("%s: [%.4f, %.4f, %.4f]\n", name, vec[0], vec[1], vec[2]);
}

static void get_translation(const double mat[16], double xyz[3])
{
    xyz[0] = mat[3];
    xyz[1] = mat[7];
    xyz[2] = mat[11];
}

static void multiply_matrix4x4(const double lhs[16], const double rhs[16], double out[16])
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k)
                sum += lhs[i * 4 + k] * rhs[k * 4 + j];
            out[i * 4 + j] = sum;
        }
    }
}

static void invert_rigid_matrix4x4(const double in[16], double out[16])
{
    memset(out, 0, sizeof(double) * 16);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
            out[i * 4 + j] = in[j * 4 + i];
    }

    out[3] = -(out[0] * in[3] + out[1] * in[7] + out[2] * in[11]);
    out[7] = -(out[4] * in[3] + out[5] * in[7] + out[6] * in[11]);
    out[11] = -(out[8] * in[3] + out[9] * in[7] + out[10] * in[11]);
    out[15] = 1.0;
}

static double distance3(const double lhs[3], const double rhs[3])
{
    const double dx = lhs[0] - rhs[0];
    const double dy = lhs[1] - rhs[1];
    const double dz = lhs[2] - rhs[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

///////////////
int test_movj_noctrl()
{
    printf("========== L1 Motion Planning Test Start ==========\n");

    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    int robot_serial = 0;
    FX_L1_Kinematics_LogSwitch(handle, robot_serial);

    // 3. 初始化单臂（左臂，robot_serial=0）
    // 3.1 获取配置参数
    int robot_type = 1017;
    double GRV[3] = {0};
    double MASS[7] = {0};
    double MCP[7][3] = {{0}};
    double I[7][6] = {{0}};
    double PNVA[8][4] =
        {
            170.0,
            -170.0,
            180,
            450,
            120.0,
            -120.0,
            180,
            450,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -145.0,
            180,
            900,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -60.0,
            180,
            900,
            90.0,
            -90.0,
            180,
            900,
        };
    double BOUND[4][3] =
        {
            0,
            -1.025,
            110,
            0,
            1.025,
            110,
            0,
            -1.025,
            -110,
            0,
            1.025,
            -110,
        };

    double DH_M3[8][4] =
        {
            0.000,
            0.000,
            177,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            272.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            256.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            87.000,
            90.000,
        };
    double DH_M6_40[8][4] =
        {
            0.000,
            0.000,
            174.5,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            287.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            314.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            95.000,
            90.000,
        };

    // 3.2 初始化单臂运动学参数
    if (!FX_L1_Kinematics_InitSingleArm(handle, 0, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Single arm initialized (left arm)\n");

    // 5. 关节空间规划 (MoveJ)
    printf("\n=== Joint Space Planning (MoveJ) ===\n");

    // 5.2 规划从初始角度到目标角度的 MoveJ 路径
    double init_joints_Arm0[7] = {10, 20, 30, 60, 60, 60, 60};
    double end_joints_Arm0[7] = {10, 20, 30, 50, 50, 10, 10};
    double MovJ_Points[MAX_POINT * 7];
    int num = 0;
    if (FX_L1_Kinematics_PlanJointMove(handle, 0, init_joints_Arm0, end_joints_Arm0, 0.1, 0.1, MovJ_Points, &num))
    {
        for (int k = 0; k < num; k++)
        {
            print_vector7("->", &MovJ_Points[k * 7]);
        }
        printf("MoveJ planning success, points = %d\n", num);
    }
    else
    {
        printf("[ERROR] MoveJ planning failed\n");
    }

    FX_L1_Kinematics_Destroy(handle);

    printf("\n========== L1 Kinematics Test Finished ==========\n");
    return 0;
}

int test_movj()
{
    printf("========== L1 Motion Planning Test Start ==========\n");
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
    SLEEP(5000);
    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    FX_L1_Kinematics_LogSwitch(handle, 0);

    // 3. 初始化单臂（左臂，robot_serial=0）
    // 3.1 获取配置参数
    int robot_type = 1017;
    double GRV[3] = {0};
    double MASS[7] = {0};
    double MCP[7][3] = {{0}};
    double I[7][6] = {{0}};
    double PNVA[8][4] =
        {
            170.0,
            -170.0,
            180,
            450,
            120.0,
            -120.0,
            180,
            450,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -145.0,
            180,
            900,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -60.0,
            180,
            900,
            90.0,
            -90.0,
            180,
            900,
        };
    double BOUND[4][3] =
        {
            0,
            -1.025,
            110,
            0,
            1.025,
            110,
            0,
            -1.025,
            -110,
            0,
            1.025,
            -110,
        };

    double DH_M3[8][4] =
        {
            0.000,
            0.000,
            177,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            272.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            256.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            87.000,
            90.000,
        };
    double DH_M6_40[8][4] =
        {
            0.000,
            0.000,
            174.5,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            287.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            314.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            95.000,
            90.000,
        };

    // 3.2 初始化单臂运动学参数
    if (!FX_L1_Kinematics_InitSingleArm(handle, 0, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Single arm initialized (left arm)\n");

    // 4. 设置位置模式
    if (FX_L1_System_Link(6, 6, 7, 190, NULL, 1) < 0)
    {
        printf("[ERROR] FX_L1_System_Link failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);

    if (FX_L1_State_ResetError(FX_OBJ_ARM0_FLAG) < 0)
    {
        printf("Reset error\n");
    }
    SLEEP(2000);

    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 1000, 10.0, 10.0) < 0)
    {
        printf("[ERROR] FX_L1_State_SwitchToPositionMode failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(250);

    // 5. 关节空间规划 (MoveJ)
    printf("\n=== Joint Space Planning (MoveJ) ===\n");
    // 5.1 设置初始角度并移动到该角度
    double init_joints_Arm0[7] = {10, 20, 30, -40, 50, 10, 10};
    if (!FX_L1_Comm_Clear(50))
    {
        printf("[ERROR] Failed to clear communication buffer\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, init_joints_Arm0);
    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send joint position command\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    SLEEP(5000); // 等待运动完成

    // SLEEP(5000);
    // FX_L1_State_SwitchToIdle(FX_OBJ_ARM0,1000);
    // FX_L1_Kinematics_Destroy(handle);
    // return 0;

    // 5.2 规划从初始角度到目标角度的 MoveJ 路径
    double end_joints_Arm0[7] = {10, 20, 30, 50, 50, 10, 10};
    double MovJ_Points[MAX_POINT * 7];
    int num = 0;
    if (FX_L1_Kinematics_PlanJointMove(handle, 0, init_joints_Arm0, end_joints_Arm0, 0.1, 0.1, MovJ_Points, &num))
    {
        printf("MoveJ planning success, points = %d\n", num);
    }
    else
    {
        printf("[ERROR] MoveJ planning failed\n");
    }

    // 5.3 发送规划结果并执行
    if (!FX_L1_Config_SetTraj(FX_OBJ_ARM0, num, MovJ_Points))
    {
        printf("[ERROR] Failed to set trajectory configuration\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    if (!FX_L1_Comm_Clear(20))
    {
        printf("[ERROR] Failed to clear communication buffer\n");

        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    unsigned int mask = FX_OBJ_ARM0_FLAG;
    unsigned int result_mask = FX_L1_Runtime_RunTraj(mask);
    printf("Trajectory execution started, mask is = %u\n", result_mask);

    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send trajectory\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    printf("here?\n");
    const ROBOT_SG *robot_state = FX_L0_GetRobotSG();

    // 5.4 Check state
    do
    {
        printf("robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState=%d\n", robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState);
        SLEEP(1);
    } while (robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 0);

    // 6. Destroy Source
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
    SLEEP(1000);
    FX_L1_Kinematics_Destroy(handle);

    printf("\n========== L1 Kinematics Test Finished ==========\n");
    return 0;
}

int test_movl()
{
    printf("========== L1 Motion Planning Test Start ==========\n");

    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    FX_L1_Kinematics_LogSwitch(handle, 0);

    // 3. 初始化单臂（左臂，robot_serial=0）
    // 3.1 获取配置参数
    int robot_type = 1017;
    double GRV[3] = {0};
    double MASS[7] = {0};
    double MCP[7][3] = {{0}};
    double I[7][6] = {{0}};
    double PNVA[8][4] =
        {
            170.0,
            -170.0,
            180,
            450,
            120.0,
            -120.0,
            180,
            450,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -145.0,
            180,
            900,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -60.0,
            180,
            900,
            90.0,
            -90.0,
            180,
            900,
        };
    double BOUND[4][3] =
        {
            0,
            -1.025,
            110,
            0,
            1.025,
            110,
            0,
            -1.025,
            -110,
            0,
            1.025,
            -110,
        };

    double DH_M3[8][4] =
        {
            0.000,
            0.000,
            177,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            272.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            256.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            87.000,
            90.000,
        };
    double DH_M6_40[8][4] =
        {
            0.000,
            0.000,
            174.5,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            287.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            314.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            95.000,
            90.000,
        };

    // 3.2 初始化单臂运动学参数
    if (!FX_L1_Kinematics_InitSingleArm(handle, 0, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Single arm initialized (left arm)\n");

    // 4. 设置位置模式
    if (FX_L1_System_Link(6, 6, 7, 190, NULL, 1) < 0)
    {
        printf("[ERROR] FX_L1_System_Link failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);

    if (FX_L1_State_ResetError(FX_OBJ_ARM0_FLAG) < 0)
    {
        printf("Reset error\n");
    }
    SLEEP(2000);

    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 1000, 10.0, 10.0) < 0)
    {
        printf("[ERROR] FX_L1_State_SwitchToPositionMode failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);
    // 4.1 移动至初始点位
    double init_joints_Arm0[7] = {-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995};
    if (!FX_L1_Comm_Clear(50))
    {
        printf("[ERROR] Failed to clear communication buffer\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, init_joints_Arm0);
    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send joint position command\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    SLEEP(10000); // 等待运动完成

    // 5. 笛卡尔直线规划 (MoveL)
    printf("\n=== Cartesian Linear Planning (MoveL) ===\n");
    // 5.1 设置MovL起始点和结束点(以当前位姿为起点，沿Z轴移动50mm为终点)
    double ref_joints[7] = {-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995};
    double cur_pose[16];

    double MovL_Points[MAX_POINT * 7];
    int num = 0;

    if (FX_L1_Kinematics_ForwardKinematics(handle, 0, ref_joints, cur_pose))
    {
        double start_xyzabc[6], end_xyzabc[6];
        FX_L1_Matrix2XYZABC(cur_pose, start_xyzabc);
        printf("%f %f %f %f %f %f\n", start_xyzabc[0], start_xyzabc[1], start_xyzabc[2], start_xyzabc[3], start_xyzabc[4], start_xyzabc[5]);
        memcpy(end_xyzabc, start_xyzabc, sizeof(start_xyzabc));
        end_xyzabc[2] += 50.0; // 沿 Z 轴移动 50 mm

        if (FX_L1_Kinematics_PlanLinearMove(handle, 0, start_xyzabc, end_xyzabc, ref_joints,
                                            10.0, 30.0, 50, MovL_Points, &num))
        {
            printf("MoveL planning success, points = %d\n", num);
        }
        else
        {
            printf("[ERROR] MoveL planning failed\n");
        }
    }
    else
    {
        printf("[ERROR] Cannot compute current pose for MoveL\n");
    }

    // 5.3 发送规划结果并执行
    if (!FX_L1_Config_SetTraj(FX_OBJ_ARM0, num, MovL_Points))
    {
        printf("[ERROR] Failed to set trajectory configuration\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    if (!FX_L1_Comm_Clear(20))
    {
        printf("[ERROR] Failed to clear communication buffer\n");

        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    unsigned int mask = FX_OBJ_ARM0_FLAG;
    unsigned int result_mask = FX_L1_Runtime_RunTraj(mask);
    printf("Trajectory execution started, mask is = %u\n", result_mask);

    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send trajectory\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(5000);

    const ROBOT_SG *robot_state = FX_L0_GetRobotSG();

    // 5.4 Check state
    do
    {
        printf("robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState=%d\n", robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState);
        SLEEP(1);
    } while (robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 0);

    // 6. Destroy Source
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
    SLEEP(1000);
    FX_L1_Kinematics_Destroy(handle);

    // 14. 清理资源
    FX_L1_Kinematics_Destroy(handle);

    printf("\n========== L1 Kinematics Test Finished ==========\n");
    return 0;
}

int test_movl_keep_J()
{
    printf("========== L1 Motion Planning Test Start ==========\n");

    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    FX_L1_Kinematics_LogSwitch(handle, 0);

    // 3. 初始化单臂（左臂，robot_serial=0）
    // 3.1 获取配置参数
    int robot_type = 1017;
    double GRV[3] = {0};
    double MASS[7] = {0};
    double MCP[7][3] = {{0}};
    double I[7][6] = {{0}};
    double PNVA[8][4] =
        {
            170.0,
            -170.0,
            180,
            450,
            120.0,
            -120.0,
            180,
            450,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -145.0,
            180,
            900,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -60.0,
            180,
            900,
            90.0,
            -90.0,
            180,
            900,
        };
    double BOUND[4][3] =
        {
            0,
            -1.025,
            110,
            0,
            1.025,
            110,
            0,
            -1.025,
            -110,
            0,
            1.025,
            -110,
        };

    double DH_M3[8][4] =
        {
            0.000,
            0.000,
            177,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            272.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            256.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            87.000,
            90.000,
        };
    double DH_M6_40[8][4] =
        {
            0.000,
            0.000,
            174.5,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            287.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            314.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            95.000,
            90.000,
        };

    // 3.2 初始化单臂运动学参数
    if (!FX_L1_Kinematics_InitSingleArm(handle, 0, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Single arm initialized (left arm)\n");

    // 4. 设置位置模式
    if (FX_L1_System_Link(6, 6, 7, 190, NULL, 1) < 0)
    {
        printf("[ERROR] FX_L1_System_Link failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);

    if (FX_L1_State_ResetError(FX_OBJ_ARM0_FLAG) < 0)
    {
        printf("Reset error\n");
    }
    SLEEP(2000);

    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 1000, 10.0, 10.0) < 0)
    {
        printf("[ERROR] FX_L1_State_SwitchToPositionMode failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);
    // 4.1 移动至初始点位
    double init_joints_Arm0[7] = {-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995};
    if (!FX_L1_Comm_Clear(50))
    {
        printf("[ERROR] Failed to clear communication buffer\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, init_joints_Arm0);
    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send joint position command\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    SLEEP(10000); // 等待运动完成

    // 5. 保持关节姿态的直线 MoveL（关节空间直线）
    printf("\n=== Cartesian Linear Planning (MoveL_Keep_J) ===\n");
    // 5.1 设置MovL起始点和结束点(以当前位姿为起点，沿Z轴移动50mm为终点)
    double end_joints_Arm0[7] = {-26.908, -80.109, 74.502, -88.083, -93.599, 17.151, -13.602};

    double MovL_KJ_Points[MAX_POINT * 7];
    int num = 0;

    if (FX_L1_Kinematics_PlanLinearKeepJoints(handle, 0, init_joints_Arm0, end_joints_Arm0, 10.0, 30.0, 50, MovL_KJ_Points, &num))
    {
        printf("MoveJ planning success, points = %d\n", num);
    }
    else
    {
        printf("[ERROR] MoveJ planning failed\n");
    }

    // 5.3 发送规划结果并执行
    if (!FX_L1_Config_SetTraj(FX_OBJ_ARM0, num, MovL_KJ_Points))
    {
        printf("[ERROR] Failed to set trajectory configuration\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    if (!FX_L1_Comm_Clear(20))
    {
        printf("[ERROR] Failed to clear communication buffer\n");

        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    unsigned int mask = FX_OBJ_ARM0_FLAG;
    unsigned int result_mask = FX_L1_Runtime_RunTraj(mask);
    printf("Trajectory execution started, mask is = %u\n", result_mask);

    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send trajectory\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(5000);

    const ROBOT_SG *robot_state = FX_L0_GetRobotSG();

    // 5.4 Check state
    do
    {
        printf("robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState=%d\n", robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState);
        SLEEP(1);
    } while (robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 0);

    // 6. Destroy Source
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
    SLEEP(1000);
    FX_L1_Kinematics_Destroy(handle);

    // 14. 清理资源
    FX_L1_Kinematics_Destroy(handle);

    printf("\n========== L1 Kinematics Test Finished ==========\n");
    return 0;
}

int test_Multi_movl()
{
    printf("========== 3 : L1 Motion Planning Test Start ==========\n");

    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    FX_L1_Kinematics_LogSwitch(handle, 0);

    // 3. 初始化单臂（左臂，robot_serial=0）
    // 3.1 获取配置参数
    int robot_type = 1017;
    double GRV[3] = {0};
    double MASS[7] = {0};
    double MCP[7][3] = {{0}};
    double I[7][6] = {{0}};
    double PNVA[8][4] =
        {
            170.0,
            -170.0,
            180,
            450,
            120.0,
            -120.0,
            180,
            450,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -145.0,
            180,
            900,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -60.0,
            180,
            900,
            90.0,
            -90.0,
            180,
            900,
        };
    double BOUND[4][3] =
        {
            0,
            -1.025,
            110,
            0,
            1.025,
            110,
            0,
            -1.025,
            -110,
            0,
            1.025,
            -110,
        };

    double DH_M3[8][4] =
        {
            0.000,
            0.000,
            177,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            272.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            256.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            87.000,
            90.000,
        };
    double DH_M6_40[8][4] =
        {
            0.000,
            0.000,
            174.5,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            287.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            314.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            95.000,
            90.000,
        };

    // 3.2 初始化单臂运动学参数
    if (!FX_L1_Kinematics_InitSingleArm(handle, 0, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Single arm initialized (left arm)\n");

    // 4. 设置位置模式
    if (FX_L1_System_Link(6, 6, 7, 190, NULL, 1) < 0)
    {
        printf("[ERROR] FX_L1_System_Link failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);

    if (FX_L1_State_ResetError(FX_OBJ_ARM0_FLAG) < 0)
    {
        printf("Reset error\n");
    }
    SLEEP(2000);

    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 1000, 10.0, 10.0) < 0)
    {
        printf("[ERROR] FX_L1_State_SwitchToPositionMode failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);
    // 4.1 移动至初始点位
    double init_joints_Arm0[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    if (!FX_L1_Comm_Clear(50))
    {
        printf("[ERROR] Failed to clear communication buffer\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, init_joints_Arm0);
    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send joint position command\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    SLEEP(10000); // 等待运动完成

    // 5. 多段连续直线 MoveL（关节空间直线）
    printf("\n=== Multi-Points Cartesian Linear Planning ===\n");
    // 5.1 设置Multi-Points MovL 第一段的起始点
    double ref_joints[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    double cur_pose[16];

    double MP_MovL_Points[MAX_POINT * 7];
    int num = 0;
    double start_xyzabc[6], end_xyzabc[6];

    if (FX_L1_Kinematics_ForwardKinematics(handle, 0, ref_joints, cur_pose))
    {
        FX_L1_Matrix2XYZABC(cur_pose, start_xyzabc);
        printf("%f %f %f %f %f %f\n", start_xyzabc[0], start_xyzabc[1], start_xyzabc[2], start_xyzabc[3], start_xyzabc[4], start_xyzabc[5]);
        memcpy(end_xyzabc, start_xyzabc, sizeof(start_xyzabc));
        end_xyzabc[2] += 200; // 沿世界坐标系Z轴正方向移动200mm
    }
    else
    {
        printf("[ERROR] Cannot compute current pose for MoveL\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    double allow_range = 5.0; // 多段允许误差5mm
    int zsp_type = 1;
    double zsp_para[6] = {0, 0, -1, 0, 0, 0};
    double vel = 500;
    double acc = 1000;
    int freq = 50; // 50Hz 规划点位
    if (FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(handle, 0, ref_joints, start_xyzabc, end_xyzabc,
                                                             allow_range, zsp_type, zsp_para, vel, acc, freq))
    {
        printf("Multi-Points MoveL planning set starte success\n");
    }
    else
    {
        printf("[ERROR] Multi-Points MoveL planning set start failed\n");
    }

    // 5.2 设置Multi-Points MovL 多段的结束点
    end_xyzabc[1] -= 200; // 第二段：沿世界坐标系Y轴负方向移动200mm
    if (FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(handle, 0, end_xyzabc, allow_range, zsp_type, zsp_para, vel, acc))
    {
        printf("Multi-Points MoveL planning set next point success\n");
    }
    else
    {
        printf("[ERROR] Multi-Points MoveL planning set start failed\n");
    }

    end_xyzabc[2] -= 200; // 第三段：沿世界坐标系Z轴负方向移动200mm
    if (FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(handle, 0, end_xyzabc, allow_range, zsp_type, zsp_para, vel, acc))
    {
        printf("Multi-Points MoveL planning set next point success\n");
    }
    else
    {
        printf("[ERROR] Multi-Points MoveL planning set start failed\n");
    }

    end_xyzabc[1] += 200; // 第三段：沿世界坐标系Y轴正方向移动200mm
    if (FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(handle, 0, end_xyzabc, allow_range, zsp_type, zsp_para, vel, acc))
    {
        printf("Multi-Points MoveL planning set next point success\n");
    }
    else
    {
        printf("[ERROR] Multi-Points MoveL planning set start failed\n");
    }

    // 5.3 获取全部规划点位数据
    FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPath(handle, MP_MovL_Points, &num);

    // 5.4 发送规划结果并执行
    if (!FX_L1_Config_SetTraj(FX_OBJ_ARM0, num, MP_MovL_Points))
    {
        printf("[ERROR] Failed to set trajectory configuration\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    if (!FX_L1_Comm_Clear(20))
    {
        printf("[ERROR] Failed to clear communication buffer\n");

        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    unsigned int mask = FX_OBJ_ARM0_FLAG;
    unsigned int result_mask = FX_L1_Runtime_RunTraj(mask);
    printf("Trajectory execution started, mask is = %u\n", result_mask);

    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send trajectory\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(5000);

    const ROBOT_SG *robot_state = FX_L0_GetRobotSG();

    // 5.5 Check state
    do
    {
        printf("robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState=%d\n", robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState);
        SLEEP(1);
    } while (robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 0);

    // 6. Destroy Source
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
    SLEEP(1000);
    FX_L1_Kinematics_Destroy(handle);

    printf("\n========== L1 Kinematics Test Finished ==========\n");
    return 0;
}

int test_Dual_Arm()
{
    printf("========== 4 : L1 Motion Planning Test Start ==========\n");

    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    FX_L1_Kinematics_LogSwitch(handle, 0);

    // 3. 初始化单臂（左臂，robot_serial=0）
    // 3.1 获取配置参数
    int robot_type[2] = {1017, 1017};
    double PNVA[2][8][4] =
        {
            {
                170.0,
                -170.0,
                180,
                450,
                120.0,
                -120.0,
                180,
                450,
                170.0,
                -170.0,
                180,
                900,
                60.0,
                -145.0,
                180,
                900,
                170.0,
                -170.0,
                180,
                900,
                60.0,
                -60.0,
                180,
                900,
                90.0,
                -90.0,
                180,
                900,
            },
            {
                170.0,
                -170.0,
                180,
                450,
                120.0,
                -120.0,
                180,
                450,
                170.0,
                -170.0,
                180,
                900,
                60.0,
                -145.0,
                180,
                900,
                170.0,
                -170.0,
                180,
                900,
                60.0,
                -60.0,
                180,
                900,
                90.0,
                -90.0,
                180,
                900,
            }};
    double BOUND[2][4][3] =
        {
            {
                0,
                -1.025,
                110,
                0,
                1.025,
                110,
                0,
                -1.025,
                -110,
                0,
                1.025,
                -110,
            },
            {
                0,
                -1.025,
                110,
                0,
                1.025,
                110,
                0,
                -1.025,
                -110,
                0,
                1.025,
                -110,
            }};

    double DH_M3[2][8][4] =
        {
            {
                0.000,
                0.000,
                177,
                0.000,
                90.000,
                0.000,
                0.000,
                0.000,
                -90.000,
                0.000,
                272.000,
                0.000,
                90.000,
                18.000,
                0.000,
                180.000,
                90.000,
                18.000,
                256.000,
                180.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                87.000,
                90.000,
            },
            {
                0.000,
                0.000,
                177,
                0.000,
                90.000,
                0.000,
                0.000,
                0.000,
                -90.000,
                0.000,
                272.000,
                0.000,
                90.000,
                18.000,
                0.000,
                180.000,
                90.000,
                18.000,
                256.000,
                180.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                87.000,
                90.000,
            }};

    double DH_M6_40[2][8][4] =
        {
            {
                0.000,
                0.000,
                174.5,
                0.000,
                90.000,
                0.000,
                0.000,
                0.000,
                -90.000,
                0.000,
                287.000,
                0.000,
                90.000,
                18.000,
                0.000,
                180.000,
                90.000,
                18.000,
                314.000,
                180.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                95.000,
                90.000,
            },
            {
                0.000,
                0.000,
                174.5,
                0.000,
                90.000,
                0.000,
                0.000,
                0.000,
                -90.000,
                0.000,
                287.000,
                0.000,
                90.000,
                18.000,
                0.000,
                180.000,
                90.000,
                18.000,
                314.000,
                180.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                0.000,
                90.000,
                90.000,
                0.000,
                95.000,
                90.000,
            }};

    // 3.2 初始化单臂运动学参数
    if (!FX_L0_Kinematics_init_dual_arm(handle, robot_type, DH_M6_40, PNVA, BOUND))
    {
        printf("[ERROR] FX_L1_Kinematics_InitDualArm failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Dual arm initialized\n");

    // 4. 设置位置模式
    if (FX_L1_System_Link(6, 6, 7, 190, NULL, 1) < 0)
    {
        printf("[ERROR] FX_L1_System_Link failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);
    // 4.1 定义全局掩码
    unsigned int mask = FX_OBJ_ARM0_FLAG | FX_OBJ_ARM1_FLAG;

    if (FX_L1_State_ResetError(mask) < 0)
    {
        printf("Reset error\n");
    }
    SLEEP(2000);

    // 4.2 双臂切换到位置模式
    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 1000, 10.0, 10.0) < 0)
    {
        printf("[ERROR] FX_L1_State_SwitchToPositionMode failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM1, 1000, 10.0, 10.0) < 0)
    {
        printf("[ERROR] FX_L1_State_SwitchToPositionMode failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(1000);
    // 4.3 双臂移动至初始点位
    double init_joints_Arm01[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    if (!FX_L1_Comm_Clear(50))
    {
        printf("[ERROR] Failed to clear communication buffer\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, init_joints_Arm01);
    FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM1, init_joints_Arm01);
    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send joint position command\n");
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }
    SLEEP(10000); // 等待运动完成

    // 5. 双臂时间同步MoveL（关节空间直线）
    printf("\n=== Dual-Arm Cartesian Linear Planning ===\n");
    // 5.1 设置Multi-Points MovL 第一段的起始点
    double cur_pose[16];

    if (FX_L1_Kinematics_ForwardKinematics(handle, 0, init_joints_Arm01, cur_pose))
    {
    }
    else
    {
        printf("[ERROR] Cannot compute current pose for MoveL\n");
    }

    // 5.2 初始点位矩阵信息转六维位姿信息
    double start_xyzabc[6];
    double end_xyzabc_arm0[6];
    double end_xyzabc_arm1[6];
    FX_L1_Matrix2XYZABC(cur_pose, start_xyzabc);
    memcpy(end_xyzabc_arm0, start_xyzabc, sizeof(start_xyzabc));
    memcpy(end_xyzabc_arm1, start_xyzabc, sizeof(start_xyzabc));

    // 5.2 配置双臂协同规划结构体参数
    DualArmFixedBodyParams DA_FB;
    int i = 0;

    // common parameters
    DA_FB.world_co_flag = FX_FALSE; // 以单臂基坐标系为准
    DA_FB.sync_type = 0;            // 默认移动距离长的为主臂
    DA_FB.freq = 50;
    DA_FB.vel = 100;
    DA_FB.acc = 300;

    // 参考角度
    for (i = 0; i < 7; i++)
    {
        DA_FB.left_ref_joints[i] = init_joints_Arm01[i];
        DA_FB.right_ref_joints[i] = init_joints_Arm01[i];
    }

    // Input parameters
    for (i = 0; i < 6; i++)
    {
        DA_FB.left_start_xyzabc[i] = start_xyzabc[i];
        DA_FB.right_start_xyzabc[i] = start_xyzabc[i];

        DA_FB.left_end_xyzabc[i] = end_xyzabc_arm0[i];
        DA_FB.right_end_xyzabc[i] = end_xyzabc_arm1[i];

        DA_FB.left_zsp_para[i] = 0.0;
        DA_FB.right_zsp_para[i] = 0.0;
    }

    DA_FB.left_end_xyzabc[2] += 200;  // 左臂末端沿Z轴正向移动200mm
    DA_FB.right_end_xyzabc[2] += 100; // 右臂末端沿Z轴正向移动100mm

    for (i = 0; i < 3; i++)
    {
        DA_FB.body_start_prr[i] = 0;
    }

    DA_FB.left_zsp_para[2] = -1;
    DA_FB.right_zsp_para[2] = -1;

    DA_FB.left_zsp_type = 0;
    DA_FB.right_zsp_type = 0;

    double DMovL0_Points[MAX_POINT * 7];
    double DMovL1_Points[MAX_POINT * 7];
    int num = 0;

    // 5.3 双臂时间同步同步规划(规划频率50Hz)
    if (FX_L1_Kinematics_PlanDualArmFixedBody(handle, &DA_FB, DMovL0_Points, DMovL1_Points, &num))
    {
        printf("Multi-Points MoveL planning set next point success\n");
    }
    else
    {
        printf("Multi-Points MoveL planning set next point failed\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000);
        FX_L1_Kinematics_Destroy(handle);
    }

    // 5.4 发送规划结果
    if (!FX_L1_Config_SetTraj(FX_OBJ_ARM0, num, DMovL0_Points))
    {
        printf("[ERROR] Failed to set trajectory configuration\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    if (!FX_L1_Config_SetTraj(FX_OBJ_ARM1, num, DMovL1_Points))
    {
        printf("[ERROR] Failed to set trajectory configuration\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    // 5.5 执行规划点位
    if (!FX_L1_Comm_Clear(20))
    {
        printf("[ERROR] Failed to clear communication buffer\n");

        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    unsigned int result_mask = FX_L1_Runtime_RunTraj(mask);
    printf("Trajectory execution started, mask is = %u\n", result_mask);

    if (!FX_L1_Comm_Send())
    {
        printf("[ERROR] Failed to send trajectory\n");
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
        FX_L1_Kinematics_Destroy(handle);
        return 0;
    }

    SLEEP(5000);

    const ROBOT_SG *robot_state = FX_L0_GetRobotSG();

    // 5.5 Check state
    do
    {
        printf("robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState=%d\n", robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState);
        SLEEP(1);
    } while (robot_state->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 0);

    // 6. Destroy Source
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000.0);
    SLEEP(1000);
    FX_L1_Kinematics_Destroy(handle);

    printf("\n========== L1 Kinematics Test Finished ==========\n");
    return 0;
}

int test_body_arm_kine()
{
    printf("========== 5 : Body + Dual-Arm IK Demo ==========\n");

    // 1. 创建运动学/规划上下文
    FX_MotionHandle handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("[ERROR] FX_L1_Kinematics_Create failed\n");
        return -1;
    }

    // 2. 日志开关（0=关闭, 1=开启）
    FX_L1_Kinematics_LogSwitch(handle, 0);

    // 3. 分别初始化左右臂运动学，单臂 FK/IK 需要对应上下文
    // 3.1 获取配置参数
    int robot_type = 1017;
    double GRV[3] = {0};
    double MASS[7] = {0};
    double MCP[7][3] = {{0}};
    double I[7][6] = {{0}};
    double PNVA[8][4] =
        {
            170.0,
            -170.0,
            180,
            450,
            120.0,
            -120.0,
            180,
            450,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -145.0,
            180,
            900,
            170.0,
            -170.0,
            180,
            900,
            60.0,
            -60.0,
            180,
            900,
            90.0,
            -90.0,
            180,
            900,
        };
    double BOUND[4][3] =
        {
            0,
            -1.025,
            110,
            0,
            1.025,
            110,
            0,
            -1.025,
            -110,
            0,
            1.025,
            -110,
        };

    double DH_M3[8][4] =
        {
            0.000,
            0.000,
            177,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            272.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            256.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            87.000,
            90.000,
        };
    double DH_M6_40[8][4] =
        {
            0.000,
            0.000,
            174.5,
            0.000,
            90.000,
            0.000,
            0.000,
            0.000,
            -90.000,
            0.000,
            287.000,
            0.000,
            90.000,
            18.000,
            0.000,
            180.000,
            90.000,
            18.000,
            314.000,
            180.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            0.000,
            90.000,
            90.000,
            0.000,
            95.000,
            90.000,
        };

    // 3.2 初始化单臂运动学参数
    if (!FX_L1_Kinematics_InitSingleArm(handle, 0, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed for arm0\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    if (!FX_L1_Kinematics_InitSingleArm(handle, 1, &robot_type, DH_M6_40, PNVA, BOUND, GRV, MASS, MCP, I))
    {
        printf("[ERROR] FX_L1_Kinematics_InitSingleArm failed for arm1\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    printf("[OK] Dual arm kinematics initialized\n");

    // 4. 构造一组已知可达的参考全身姿态，后续再基于目标末端位姿做逆解
    double ref_body_joints[3] = {80.0, 5.0, 0.0};
    double left_ref_joints[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.10};
    double right_ref_joints[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.10};

    double left_shoulder_ref[16] = {0};
    double right_shoulder_ref[16] = {0};
    if (!FX_L1_Kinematics_BodyForward(handle, ref_body_joints, left_shoulder_ref, right_shoulder_ref))
    {
        printf("[ERROR] FX_L1_Kinematics_BodyForward failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }

    double left_tcp_ref_shoulder[16] = {0};
    double right_tcp_ref_shoulder[16] = {0};
    if (!FX_L1_Kinematics_ForwardKinematics(handle, 0, left_ref_joints, left_tcp_ref_shoulder))
    {
        printf("[ERROR] Left arm FK failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    if (!FX_L1_Kinematics_ForwardKinematics(handle, 1, right_ref_joints, right_tcp_ref_shoulder))
    {
        printf("[ERROR] Right arm FK failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }

    print_matrix("solved Left Arm Joints matrix", left_tcp_ref_shoulder);

    // 5. “给定输入”：身体坐标系/躯干根坐标系下的双臂末端目标位姿
    double left_tcp_ref_body[16] = {0};
    double right_tcp_ref_body[16] = {0};
    multiply_matrix4x4(left_shoulder_ref, left_tcp_ref_shoulder, left_tcp_ref_body);
    multiply_matrix4x4(right_shoulder_ref, right_tcp_ref_shoulder, right_tcp_ref_body);

    double left_target_body[16] = {0};
    double right_target_body[16] = {0};
    memcpy(left_target_body, left_tcp_ref_body, sizeof(left_target_body));
    memcpy(right_target_body, right_tcp_ref_body, sizeof(right_target_body));

    // 只给定位置时姿态沿用参考姿态，这里对位置做小偏移，演示“身体+双臂”整体逆解
    left_target_body[3] += 20.0;
    left_target_body[11] += 15.0;
    right_target_body[3] += 20.0;
    right_target_body[11] += 5.0;

    double left_target_xyzabc_body[6] = {0};
    double right_target_xyzabc_body[6] = {0};
    FX_L1_Matrix2XYZABC(left_target_body, left_target_xyzabc_body);
    FX_L1_Matrix2XYZABC(right_target_body, right_target_xyzabc_body);
    print_xyzabc("left target pose in body frame", left_target_xyzabc_body);
    print_xyzabc("right target pose in body frame", right_target_xyzabc_body);

    // 6. 身体逆解依赖 body condition，这里用参考姿态自动构造一组稳定参数
    double left_shoulder_ref_pos[3] = {0};
    double right_shoulder_ref_pos[3] = {0};
    double left_tcp_ref_pos[3] = {0};
    double right_tcp_ref_pos[3] = {0};
    double left_target_pos[3] = {0};
    double right_target_pos[3] = {0};
    get_translation(left_shoulder_ref, left_shoulder_ref_pos);
    get_translation(right_shoulder_ref, right_shoulder_ref_pos);
    get_translation(left_tcp_ref_body, left_tcp_ref_pos);
    get_translation(right_tcp_ref_body, right_tcp_ref_pos);
    get_translation(left_target_body, left_target_pos);
    get_translation(right_target_body, right_target_pos);

    double body_stiffness[3] = {10.0, 10.0, 10.0};
    double left_std_len = distance3(left_shoulder_ref_pos, left_tcp_ref_pos);
    double right_std_len = distance3(right_shoulder_ref_pos, right_tcp_ref_pos);
    if (!FX_L1_Kinematics_SetBodyCondition(handle, ref_body_joints, body_stiffness,
                                           left_std_len, 1.0, right_std_len, 1.0))
    {
        printf("[ERROR] FX_L1_Kinematics_SetBodyCondition failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }

    // 7. 根据左右臂末端位置，先反解身体关节
    double solved_body_joints[3] = {0};
    if (!FX_L1_Kinematics_CalcBodyPositionWithRef(handle, ref_body_joints,
                                                  left_target_pos, right_target_pos, solved_body_joints))
    {
        printf("[ERROR] FX_L1_Kinematics_CalcBodyPositionWithRef failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    print_vector3("solved body joints [lift, pitch, rotation]", solved_body_joints);

    // 8. 再用身体正解得到新的左右肩基坐标系
    double left_shoulder_solved[16] = {0};
    double right_shoulder_solved[16] = {0};
    if (!FX_L1_Kinematics_BodyForward(handle, solved_body_joints,
                                      left_shoulder_solved, right_shoulder_solved))
    {
        printf("[ERROR] FX_L1_Kinematics_BodyForward failed for solved body joints\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }

    // 9. 把身体坐标系目标变换到各自肩部基坐标系，分别做左右臂 IK
    double left_shoulder_inv[16] = {0};
    double right_shoulder_inv[16] = {0};
    double left_target_shoulder[16] = {0};
    double right_target_shoulder[16] = {0};
    invert_rigid_matrix4x4(left_shoulder_solved, left_shoulder_inv);
    invert_rigid_matrix4x4(right_shoulder_solved, right_shoulder_inv);
    multiply_matrix4x4(left_shoulder_inv, left_target_body, left_target_shoulder);
    multiply_matrix4x4(right_shoulder_inv, right_target_body, right_target_shoulder);

    FX_InvKineSolverParams left_arm_ik;
    FX_InvKineSolverParams right_arm_ik;
    memset(&left_arm_ik, 0, sizeof(left_arm_ik));
    memset(&right_arm_ik, 0, sizeof(right_arm_ik));
    memcpy(left_arm_ik.target_pose, left_target_shoulder, sizeof(left_arm_ik.target_pose));
    memcpy(right_arm_ik.target_pose, right_target_shoulder, sizeof(right_arm_ik.target_pose));
    memcpy(left_arm_ik.ref_joints, left_ref_joints, sizeof(left_arm_ik.ref_joints));
    memcpy(right_arm_ik.ref_joints, right_ref_joints, sizeof(right_arm_ik.ref_joints));

    if (!FX_L1_Kinematics_InverseKinematics(handle, 0, &left_arm_ik) || !left_arm_ik.solution_valid)
    {
        printf("[ERROR] Left arm IK failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    if (!FX_L1_Kinematics_InverseKinematics(handle, 1, &right_arm_ik) || !right_arm_ik.solution_valid)
    {
        printf("[ERROR] Right arm IK failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }

    print_vector7("solved left arm joints", left_arm_ik.solution);
    print_vector7("solved right arm joints", right_arm_ik.solution);

    // 10. 正解回代校验
    double left_tcp_verify_shoulder[16] = {0};
    double right_tcp_verify_shoulder[16] = {0};
    double left_tcp_verify_body[16] = {0};
    double right_tcp_verify_body[16] = {0};
    if (!FX_L1_Kinematics_ForwardKinematics(handle, 0, left_arm_ik.solution, left_tcp_verify_shoulder))
    {
        printf("[ERROR] Left arm FK verification failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    if (!FX_L1_Kinematics_ForwardKinematics(handle, 1, right_arm_ik.solution, right_tcp_verify_shoulder))
    {
        printf("[ERROR] Right arm FK verification failed\n");
        FX_L1_Kinematics_Destroy(handle);
        return -1;
    }
    multiply_matrix4x4(left_shoulder_solved, left_tcp_verify_shoulder, left_tcp_verify_body);
    multiply_matrix4x4(right_shoulder_solved, right_tcp_verify_shoulder, right_tcp_verify_body);

    double left_verify_xyzabc_body[6] = {0};
    double right_verify_xyzabc_body[6] = {0};
    double left_verify_pos[3] = {0};
    double right_verify_pos[3] = {0};
    FX_L1_Matrix2XYZABC(left_tcp_verify_body, left_verify_xyzabc_body);
    FX_L1_Matrix2XYZABC(right_tcp_verify_body, right_verify_xyzabc_body);
    get_translation(left_tcp_verify_body, left_verify_pos);
    get_translation(right_tcp_verify_body, right_verify_pos);

    print_xyzabc("left verified pose in body frame", left_verify_xyzabc_body);
    print_xyzabc("right verified pose in body frame", right_verify_xyzabc_body);
    printf("left position error  = %.6f mm\n", distance3(left_verify_pos, left_target_pos));
    printf("right position error = %.6f mm\n", distance3(right_verify_pos, right_target_pos));

    FX_L1_Kinematics_Destroy(handle);
    printf("\n========== Body + Dual-Arm IK Demo Finished ==========\n");
    return 0;
}

int main()
{
    int test_num = 5;

    if (test_num == 0)
    {
        // test_movj();
        test_movj_noctrl();
    }
    else if (test_num == 1)
    {
        test_movl();
    }
    else if (test_num == 2)
    {
        test_movl_keep_J();
    }
    else if (test_num == 3)
    {
        test_Multi_movl();
    }
    else if (test_num == 4)
    {
        test_Dual_Arm();
    }
    else if (test_num == 5)
    {
        test_body_arm_kine();
    }
}
