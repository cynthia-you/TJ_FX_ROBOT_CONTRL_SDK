#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// 示例说明：演示双臂带零空间约束的逆运动学求解与臂角调整
//
// 整体流程：
//   阶段一：双臂运动学初始化
//     1. 定义双臂运动学参数存储区。
//     2. 加载与机器人型号对应的配置文件。
//     3. 初始化双臂机器人类型、DH 参数及运动限制。
//   阶段二：左臂零空间约束逆解
//     4. 计算左臂目标位置和姿态矩阵。
//     5. 设置左臂工作参考构型。
//     6. 设置左臂逆解基础参数和零空间参数。
//     7. 求解左臂零空间约束逆运动学。
//     8. 调整左臂臂角并输出结果。
//   阶段三：右臂零空间约束逆解
//     9. 计算右臂目标位置和姿态矩阵。
//    10. 设置右臂工作参考构型。
//    11. 设置右臂逆解基础参数和零空间参数。
//    12. 求解右臂零空间约束逆运动学。
//    13. 调整右臂臂角并输出结果。
//   阶段四：结果验证
//    14. 给出使用零空间约束后的双臂参考轨迹结果。
//
// 注意：左右臂的臂角正负方向不同，调整时应遵循各自坐标方向。
// =============================================================================

void ikNspTwoArmsDemo()
{
    // 辅助函数：按指定精度打印数组
    auto print_array = [](auto *arr, size_t n, const char *name = "", int precision = 2)
    {
        if (name[0] != '\0')
            printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i)
        {
            printf("%.*lf%s", precision, arr[i], i < n - 1 ? "," : "");
        }
        printf("]\n");
    };

    // 辅助函数：按指定精度打印矩阵
    auto print_matrix = [](auto *mat, size_t rows, size_t cols, const char *name = "", int precision = 2)
    {
        if (name[0] != '\0')
            printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i)
        {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j)
            {
                printf("%.*lf%s", precision, mat[i][j], j < cols - 1 ? "," : "");
            }
            printf("]%s\n", i < rows - 1 ? "," : "]");
        }
    };

    int i = 0;
    int j = 0;

    // [阶段一｜步骤 1] 定义双臂运动学参数
    int TYPE[2];
    double GRV[2][3];
    double DH[2][8][4];
    double PNVA[2][7][4];
    double BD[2][4][3];

    double Mass[2][7];
    double MCP[2][7][3];
    double I[2][7][6];

    // [阶段一｜步骤 2] 加载机型配置
    // 注意：配置文件与实际机型不匹配时，程序可能正常运行但计算结果错误。
    // // ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    // // ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
    // // srs机型为srs.MvKDCfg.
    // 同时需要确认 arm_type 对应左臂（0）还是右臂（1）。
    if (LOADMvCfg((char *)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == true)
    {
        printf("oad CFG Success\n");
    }
    else
    {
        printf("Load CFG Error\n");
    }
    printf("------------------------------\n");

    // [阶段一｜步骤 3] 初始化双臂运动学参数
    printf("A arm\n");
    if (FX_Robot_Init_Type(0, TYPE[0]) == false)
    {
        printf("Init Type Error\n");
    }
    else
    {
        printf("Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(0, DH[0]) == false)
    {
        printf("nit DH Parameters Error\n");
    }
    else
    {
        printf("Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == false)
    {
        printf("Init Limit Parameters Error\n");
    }
    else
    {
        printf("Init Limit Parameters Success\n");
    }

    printf("B arm\n");
    if (FX_Robot_Init_Type(1, TYPE[1]) == false)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(1, DH[1]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }

    printf("A arm\n");

    // -------------------------------------------------------------------------
    // [阶段二] 左臂零空间约束逆解
    // 说明：以满意构型的臂角矩阵 X 方向作为零空间引导，再调整臂角。
    // -------------------------------------------------------------------------
    // [阶段二｜步骤 4] 计算左臂目标位置和姿态矩阵
    double target_joints_A[7] = {21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68};
    double target_pose_A[4][4] = {0};
    if (FX_Robot_Kine_FK(0, target_joints_A, target_pose_A) == false)
    {
        printf("FK Error\n");
    }
    else
    {
        printf("FK Success\n");
        print_matrix(target_pose_A, 4, 4, "target_pose_A");
    }

    // [阶段二｜步骤 5] 设置左臂工作参考构型
    double jv_benchmark_A[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    double kine_pg_bm_A[4][4] = {0};
    Matrix3 nsp_bm_A;
    if (FX_Robot_Kine_FK_NSP(0, jv_benchmark_A, kine_pg_bm_A, nsp_bm_A) == false)
    {
        printf("FK_NSP Error\n");
    }
    else
    {
        printf("FK_NSP Success\n");
        print_matrix(nsp_bm_A, 3, 3, "nsp_bm_A=");
    }

    // [阶段二｜步骤 6] 设置左臂逆解基础参数和零空间参数
    FX_InvKineSolvePara sp;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp.m_Input_IK_TargetTCP[i][j] = target_pose_A[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp.m_Input_IK_RefJoint[i] = jv_benchmark_A[i];
    }

    sp.m_Input_IK_ZSPType = 1;
    sp.m_Input_IK_ZSPPara[0] = nsp_bm_A[0][0];
    sp.m_Input_IK_ZSPPara[1] = nsp_bm_A[1][0];
    sp.m_Input_IK_ZSPPara[2] = nsp_bm_A[2][0];

    // [阶段二｜步骤 7] 求解左臂零空间约束逆运动学
    if (FX_Robot_Kine_IK(0, &sp) == false)
    {
        printf("IK Error\n");
    }
    else
    {
        printf("IK Success\n");
        print_array(sp.m_Output_RetJoint, 7, "IK result under reference joints");
        printf("ik whether current pose exceeds reachable workspace (False: no; True: yes): %d\n", sp.m_Output_IsOutRange);
        printf("ik whether each joint is singular (False: no; True: yes):  %d, %d, %d, %d, %d, %d, %d\n",
               sp.m_Output_IsDeg[0],
               sp.m_Output_IsDeg[1],
               sp.m_Output_IsDeg[2],
               sp.m_Output_IsDeg[3],
               sp.m_Output_IsDeg[4],
               sp.m_Output_IsDeg[5],
               sp.m_Output_IsDeg[6]);
        printf("ik whether any joint exceeds position limit (False: no; True: yes):%d\n", sp.m_Output_IsJntExd);
        printf("ik whether each joint exceeds position limit (False: no; True: yes):  %d, %d, %d, %d, %d, %d, %d\n",
               sp.m_Output_JntExdTags[0],
               sp.m_Output_JntExdTags[1],
               sp.m_Output_JntExdTags[2],
               sp.m_Output_JntExdTags[3],
               sp.m_Output_JntExdTags[4],
               sp.m_Output_JntExdTags[5],
               sp.m_Output_JntExdTags[6]);
        print_array(sp.m_Output_RunLmtP, 7, "ik positive joint limits: ");
        print_array(sp.m_Output_RunLmtN, 7, "ik negative joint limits: ");
        printf("number of ik results:%ld\n", sp.m_OutPut_Result_Num);
    }

    // [阶段二｜步骤 8] 调整左臂臂角
    sp.m_Input_ZSP_Angle = 15; // 左臂向上调整 15°
    if (FX_Robot_Kine_IK_NSP(0, &sp) == false)
    {
        printf("IK_NSP Error\n");
    }
    else
    {
        printf("IK_NSP Success\n");
        print_array(sp.m_Output_RetJoint, 7, "IK_NSP result under reference joints");
    }

    printf("---------------------------------------\n");

    printf("B arm\n");
    // -------------------------------------------------------------------------
    // [阶段三] 右臂零空间约束逆解
    // 说明：右臂臂角方向与左臂相反，向上调整时使用负角度。
    // -------------------------------------------------------------------------
    // [阶段三｜步骤 9] 计算右臂目标位置和姿态矩阵
    double target_joints_B[7] = {-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68};
    double target_pose_B[4][4] = {0};
    if (FX_Robot_Kine_FK(0, target_joints_B, target_pose_B) == false)
    {
        printf("FK Error\n");
    }
    else
    {
        printf("FK Success\n");
        print_matrix(target_pose_B, 4, 4, "target_pose_B");
    }

    // [阶段三｜步骤 10] 设置右臂工作参考构型
    double jv_benchmark_B[7] = {-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1};
    double kine_pg_bm_B[4][4] = {0};
    Matrix3 nsp_bm_B;
    if (FX_Robot_Kine_FK_NSP(0, jv_benchmark_B, kine_pg_bm_B, nsp_bm_B) == false)
    {
        printf("FK_NSP Error\n");
    }
    else
    {
        printf("FK_NSP Success\n");
        print_matrix(nsp_bm_B, 3, 3, "nsp_bm_B");
    }

    // [阶段三｜步骤 11] 设置右臂逆解基础参数和零空间参数
    FX_InvKineSolvePara sp1;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp1.m_Input_IK_TargetTCP[i][j] = target_pose_B[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp1.m_Input_IK_RefJoint[i] = jv_benchmark_B[i];
    }

    sp1.m_Input_IK_ZSPType = 1;
    sp1.m_Input_IK_ZSPPara[0] = nsp_bm_B[0][0];
    sp1.m_Input_IK_ZSPPara[1] = nsp_bm_B[1][0];
    sp1.m_Input_IK_ZSPPara[2] = nsp_bm_B[2][0];

    // [阶段三｜步骤 12] 求解右臂零空间约束逆运动学
    if (FX_Robot_Kine_IK(0, &sp1) == false)
    {
        printf("IK Error\n");
    }
    else
    {
        printf("IK Success\n");
        print_array(sp1.m_Output_RetJoint, 7, "IK result under reference joints");
        printf("ik whether current pose exceeds reachable workspace (False: no; True: yes): %d\n", sp1.m_Output_IsOutRange);
        printf("ik whether each joint is singular (False: no; True: yes): %d, %d, %d, %d, %d, %d, %d\n",
               sp1.m_Output_IsDeg[0],
               sp1.m_Output_IsDeg[1],
               sp1.m_Output_IsDeg[2],
               sp1.m_Output_IsDeg[3],
               sp1.m_Output_IsDeg[4],
               sp1.m_Output_IsDeg[5],
               sp1.m_Output_IsDeg[6]);
        printf("ik whether any joint exceeds position limit (False: no; True: yes):%d\n", sp1.m_Output_IsJntExd);
        printf("ik whether each joint exceeds position limit (False: no; True: yes):  %d, %d, %d, %d, %d, %d, %d\n",
               sp1.m_Output_JntExdTags[0],
               sp1.m_Output_JntExdTags[1],
               sp1.m_Output_JntExdTags[2],
               sp1.m_Output_JntExdTags[3],
               sp1.m_Output_JntExdTags[4],
               sp1.m_Output_JntExdTags[5],
               sp1.m_Output_JntExdTags[6]);
        print_array(sp1.m_Output_RunLmtP, 7, "ik positive joint limits: ");
        print_array(sp1.m_Output_RunLmtN, 7, "ik negative joint limits: ");
        printf("number of ik results:%ld\n", sp1.m_OutPut_Result_Num);
    }

    // [阶段三｜步骤 13] 调整右臂臂角
    sp1.m_Input_ZSP_Angle = -15; // 右臂向上调整 15°
    if (FX_Robot_Kine_IK_NSP(0, &sp1) == false)
    {
        printf("IK_NSP Error\n");
    }
    else
    {
        printf("IK_NSP Success\n");
        print_array(sp1.m_Output_RetJoint, 7, "IK_NSP result under reference joints");
    }
}

int main()
{
    ikNspTwoArmsDemo();
    // -------------------------------------------------------------------------
    // [阶段四｜步骤 14] 零空间约束结果参考
    // 左臂：工作基准构型
    // [44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1]
    // 左臂：零位
    // [0,0,0,0,0,0,0]
    // 左臂：目标位姿的约束逆解结果
    // [26.132192947209525, -41.4299312921566, -12.382793333582738, -63.669999968333876, 15.34616386472852, 14.135899381262774, 5.014661729429383]
    // 左臂：臂角调整结果
    // [13.651106491615714, -41.22561001606551, 9.55389260549187, -63.669999968333876, 0.4169924723007425, 15.17290106829154, 12.745643235928943]

    // 右臂：工作基准构型
    // [-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1]
    // 右臂：零位
    // [0,0,0,0,0,0,0]
    // 右臂：目标位姿的约束逆解结果
    // [-26.126455465388723, -41.42961256959308, 12.382716991277716, -63.669999968336434, -15.339360903337091, 14.136820316614646, -5.018123071079321]
    // 右臂：臂角调整结果
    // [-13.645291371687428, -41.22530974382568, -9.554071963583754, -63.669999968336434, -0.41009102022740107, 15.172926189678035, -12.749249430475857]
}
