#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// 示例说明：演示逆运动学求解失败的典型场景及返回结果
//
// 整体流程：
//   阶段一：运动学初始化
//     1. 定义运动学参数存储区并加载机型配置文件。
//     2. 初始化机器人类型、DH 参数及运动限制。
//   阶段二：关节奇异场景
//     3. 构造关节 4 为 0 的输入并执行逆解，观察失败结果。
//   阶段三：超出可达空间场景
//     4. 构造超出机器人可达空间的目标位姿并执行逆解。
//
// 关节超限需要检查逆运动学结构体的关节超限标志位
// 注意：配置文件必须与实际机器人型号和版本一致。
// =============================================================================

void KineFailedDemo()
{
    int i = 0;
    int j = 0;

    // [阶段一｜步骤 1] 定义并加载运动学参数
    int TYPE[2];
    double GRV[2][3];
    double DH[2][8][4];
    double PNVA[2][7][4];
    double BD[2][4][3];

    double Mass[2][7];
    double MCP[2][7][3];
    double I[2][7][6];
    // 注意：配置文件与实际机型不匹配时，程序可能正常运行但计算结果错误。
    // 参考文件：CCS 6 kg 3.1/4.0 分别使用 ccs_m6_31/40.MvKDCfg，
    // CCS 3 kg 使用 ccs_m3.MvKDCfg，SRS 使用 srs.MvKDCfg。
    // 同时需要确认 arm_type 对应左臂（0）还是右臂（1）。
    if (LOADMvCfg((char *)"ccs_m6.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == true)
    {
        printf("Robot Load CFG Success\n");
    }
    else
    {
        printf("Robot Load CFG Error\n");
    }
    printf("------------------------------\n");

    // [阶段一｜步骤 2] 初始化机器人类型、DH 参数及运动限制
    if (FX_Robot_Init_Type(0, TYPE[0]) == false)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(0, DH[0]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }
    printf("------------------------------\n");

    double jv[7] = {10, 10, 10, -6.869, 10, 10, 10};
    double kine_pg[4][4] = {0};
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
    }
    printf("------------------------------\n");

    // [阶段二｜步骤 3] 失败场景一：关节 4 发生奇异
    FX_InvKineSolvePara sp;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp.m_Input_IK_TargetTCP[i][j] = kine_pg[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp.m_Input_IK_RefJoint[i] = jv[i];
    }
    if (FX_Robot_Kine_IK(0, &sp) == false)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
    }
    printf("------------------------------\n");

    // [阶段三｜步骤 4] 失败场景二：目标位姿超出可达空间
    double xyzabc[6] = {1000, 500, 300, 0, 0, 0};
    double mat_result[4][4] = {0};
    FX_XYZABC2Matrix4DEG(xyzabc, mat_result);
    double jv1[7] = {10, 10, 10, 10, 10, 10, 10};

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp.m_Input_IK_TargetTCP[i][j] = mat_result[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp.m_Input_IK_RefJoint[i] = jv1[i];
    }
    if (FX_Robot_Kine_IK(0, &sp) == false)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
    }
}

int main()
{
    KineFailedDemo();
}
