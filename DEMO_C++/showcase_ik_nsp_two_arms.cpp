#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void ikNspTwoArmsDemo()
{
    // 打印数组的lambda - 保留两位小数
    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i) {
            printf("%.*lf%s", precision, arr[i], i < n-1 ? "," : "");
        }
        printf("]\n");
    };

    // 打印矩阵的lambda - 保留两位小数
    auto print_matrix = [](auto* mat, size_t rows, size_t cols, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i) {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j) {
                printf("%.*lf%s", precision, mat[i][j], j < cols-1 ? "," : "");
            }
            printf("]%s\n", i < rows-1 ? "," : "]");
        }
    };

    FX_INT32L i = 0;
    FX_INT32L j = 0;

   ////////////////////////导入运动学参数
    FX_INT32L TYPE[2];
    FX_DOUBLE GRV[2][3];
    FX_DOUBLE DH[2][8][4];
    FX_DOUBLE PNVA[2][7][4];
    FX_DOUBLE BD[2][4][3];

    FX_DOUBLE Mass[2][7];
    FX_DOUBLE MCP[2][7][3];
    FX_DOUBLE I[2][7][6];


    ////////////////////////加载计算参数
    if (LOADMvCfg((char*)"ccs_m6.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_TRUE)
    {
        printf("oad CFG Success\n");
    }
    else
    {
        printf("Load CFG Error\n");
    }
    printf("------------------------------\n");


    
    ////////////////////////初始化运动学参数
    printf("A arm\n");
    if (FX_Robot_Init_Type(0, TYPE[0]) == FX_FALSE)
    {
        printf("Init Type Error\n");
    }
    else
    {
        printf("Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(0, DH[0]) == FX_FALSE)
    {
        printf("nit DH Parameters Error\n");
    }
    else
    {
        printf("Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == FX_FALSE)
    {
        printf("Init Limit Parameters Error\n");
    }
    else
    {
        printf("Init Limit Parameters Success\n");
    }

    printf("B arm\n");
    if (FX_Robot_Init_Type(1, TYPE[1]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(1, DH[1]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }

    printf("A arm\n");

    // '''
    // 左臂逆解到预期构型
    // 1. 确定目标点位下的位置和姿态矩阵(案例里没有加工具,用正解得到末端法兰的位姿)
    // 2. 设置工作参考构型(提前拖动到一个满意的构型,以该构型的臂角矩阵的X方向信息作为所有逆解的臂角坐标引导)
    // 3. 设置逆解基础参数(末端位置姿态和参考位置姿态)以及零空间参数(将步骤2的nsp_mat1的第一列作为逆解的了零空间方向信息)
    // 4. 逆解
    // 5. 优化,这个胳膊肘我不太满意,想要胳膊肘往上抬,左臂的臂角的Z向量是从手腕到肩的,X向量是从胳膊肘部垂直指向Z向量,根据右手法则,我想胳膊肘往上翘,顺时针角度要加,反之逆时针角度要减
    // '''

    //1. 确定目标点位的位置和姿态矩阵
    FX_DOUBLE target_joints_A[7] = {21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68};
    Matrix4 target_pose_A;
    if (FX_Robot_Kine_FK(0, target_joints_A, target_pose_A) == FX_FALSE)
    {
        printf("FK Error\n");
    }
    else
    {
        printf("FK Success\n");
        print_matrix(target_pose_A,4,4,"target_pose_A");
    }

    //2. 设置工作参考构型
    FX_DOUBLE jv_benchmark_A[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    Matrix4 kine_pg_bm_A;
    Matrix3 nsp_bm_A;
    if (FX_Robot_Kine_FK_NSP(0, jv_benchmark_A, kine_pg_bm_A,nsp_bm_A) == FX_FALSE)
    {
        printf("FK_NSP Error\n");
    }
    else
    {
        printf("FK_NSP Success\n");
        print_matrix(nsp_bm_A,3,3,"nsp_bm_A=");
    }

    //3. 设置逆解基础参数以及零空间参数
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

    sp.m_Input_IK_ZSPType=1;
    sp.m_Input_IK_ZSPPara[0]=nsp_bm_A[0][0];
    sp.m_Input_IK_ZSPPara[1]=nsp_bm_A[1][0];
    sp.m_Input_IK_ZSPPara[2]=nsp_bm_A[2][0];

    //4. 逆解
    if (FX_Robot_Kine_IK(0, &sp) == FX_FALSE)
    {
        printf("IK Error\n");
    }
    else
    {
        printf("IK Success\n");
        print_array(sp.m_Output_RetJoint,7,"IK result under reference joints");
        printf("ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: %d\n",sp.m_Output_IsOutRange);
        printf("ik 各关节是否发生奇异（False：未奇异；True：奇异）:  %d, %d, %d, %d, %d, %d, %d\n",
        sp.m_Output_IsDeg[0],
        sp.m_Output_IsDeg[1],
        sp.m_Output_IsDeg[2],
        sp.m_Output_IsDeg[3],
        sp.m_Output_IsDeg[4],
        sp.m_Output_IsDeg[5],
        sp.m_Output_IsDeg[6]);
        printf("ik 是否有关节超出位置正负限制（False：未超出；True：超出）:%d\n",sp.m_Output_IsJntExd);
        printf("ik 各关节是否超出位置正负限制（False：未超出；True：超出）:  %d, %d, %d, %d, %d, %d, %d\n",
        sp.m_Output_JntExdTags[0],
        sp.m_Output_JntExdTags[1],
        sp.m_Output_JntExdTags[2],
        sp.m_Output_JntExdTags[3],
        sp.m_Output_JntExdTags[4],
        sp.m_Output_JntExdTags[5],
        sp.m_Output_JntExdTags[6]);
        print_array(sp.m_Output_RunLmtP,7,"ik 各关节正限制: ");
        print_array(sp.m_Output_RunLmtN,7,"ik 各关节负限制: ");
        printf("number of ik results:%ld\n",sp.m_OutPut_Result_Num);
    }

    //5. 优化
    sp.m_Input_ZSP_Angle=15;//胳膊往上翘15度
    if (FX_Robot_Kine_IK_NSP(0, &sp) == FX_FALSE)
    {
        printf("IK_NSP Error\n");
    }
    else
    {
        printf("IK_NSP Success\n");
        print_array(sp.m_Output_RetJoint,7,"IK_NSP result under reference joints");
    }


    printf("---------------------------------------\n");

    printf("B arm\n");
    // '''
    // 逆解到预期构型
    // 1. 确定目标点位下的位置和姿态矩阵(案例里没有加工具,用正解得到末端法兰的位姿)
    // 2. 设置工作参考构型(提前拖动到一个满意的构型,以该构型的臂角矩阵的X方向信息作为所有逆解的臂角坐标引导)
    // 3. 设置逆解基础参数(末端位置姿态和参考位置姿态)以及零空间参数(将步骤2的nsp_mat1的第一列作为逆解的了零空间方向信息)
    // 4. 逆解
    // 5. 优化,这个胳膊肘我不太满意,想要胳膊肘往上抬,右臂的臂角的Z向量是从手腕到肩的,X向量是从胳膊肘部垂直指向Z向量,根据右手法则,我想胳膊肘往上翘,逆时针角度要减,反之顺时针角度要加
    // '''
   
        //1. 确定目标点位的位置和姿态矩阵
    FX_DOUBLE target_joints_B[7] = {-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68};
    Matrix4 target_pose_B;
    if (FX_Robot_Kine_FK(0, target_joints_B, target_pose_B) == FX_FALSE)
    {
        printf("FK Error\n");
    }
    else
    {
        printf("FK Success\n");
        print_matrix(target_pose_B,4,4,"target_pose_B");
    }

    //2. 设置工作参考构型
    FX_DOUBLE jv_benchmark_B[7] = {-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1};
    Matrix4 kine_pg_bm_B;
    Matrix3 nsp_bm_B;
    if (FX_Robot_Kine_FK_NSP(0, jv_benchmark_B, kine_pg_bm_B,nsp_bm_B) == FX_FALSE)
    {
        printf("FK_NSP Error\n");
    }
    else
    {
        printf("FK_NSP Success\n");
        print_matrix(nsp_bm_B,3,3,"nsp_bm_B");
    }

    //3. 设置逆解基础参数以及零空间参数
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

    sp1.m_Input_IK_ZSPType=1;
    sp1.m_Input_IK_ZSPPara[0]=nsp_bm_B[0][0];
    sp1.m_Input_IK_ZSPPara[1]=nsp_bm_B[1][0];
    sp1.m_Input_IK_ZSPPara[2]=nsp_bm_B[2][0];

    //4. 逆解
    if (FX_Robot_Kine_IK(0, &sp1) == FX_FALSE)
    {
        printf("IK Error\n");
    }
    else
    {
        printf("IK Success\n");
        print_array(sp1.m_Output_RetJoint,7,"IK result under reference joints");
        printf("ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: %d\n",sp1.m_Output_IsOutRange);
        printf("ik 各关节是否发生奇异（False：未奇异；True：奇异）: %d, %d, %d, %d, %d, %d, %d\n",
        sp1.m_Output_IsDeg[0],
        sp1.m_Output_IsDeg[1],
        sp1.m_Output_IsDeg[2],
        sp1.m_Output_IsDeg[3],
        sp1.m_Output_IsDeg[4],
        sp1.m_Output_IsDeg[5],
        sp1.m_Output_IsDeg[6]);
        printf("ik 是否有关节超出位置正负限制（False：未超出；True：超出）:%d\n",sp1.m_Output_IsJntExd);
        printf("ik 各关节是否超出位置正负限制（False：未超出；True：超出）:  %d, %d, %d, %d, %d, %d, %d\n",
        sp1.m_Output_JntExdTags[0],
        sp1.m_Output_JntExdTags[1],
        sp1.m_Output_JntExdTags[2],
        sp1.m_Output_JntExdTags[3],
        sp1.m_Output_JntExdTags[4],
        sp1.m_Output_JntExdTags[5],
        sp1.m_Output_JntExdTags[6]);
        print_array(sp1.m_Output_RunLmtP,7,"ik 各关节正限制: ");
        print_array(sp1.m_Output_RunLmtN,7,"ik 各关节负限制: ");
        printf("number of ik results:%ld\n",sp1.m_OutPut_Result_Num);
    }

    //5. 优化
    sp1.m_Input_ZSP_Angle=-15;//胳膊往上翘15度
    if (FX_Robot_Kine_IK_NSP(0, &sp1) == FX_FALSE)
    {
        printf("IK_NSP Error\n");
    }
    else
    {
        printf("IK_NSP Success\n");
        print_array(sp1.m_Output_RetJoint,7,"IK_NSP result under reference joints");
    }


}

int main()
{
    ikNspTwoArmsDemo();
    // '
    // 下面是一串轨迹来验证使用fk_nsp约束的点位
    // #左臂
    // #设定了一个工作的基准参考
    // [44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1]
    // #回零
    // [0,0,0,0,0,0,0]
    // #目标位置姿态逆解到约束构型
    // [26.132192947209525, -41.4299312921566, -12.382793333582738, -63.669999968333876, 15.34616386472852, 14.135899381262774, 5.014661729429383]
    // #调整臂角度
    // [13.651106491615714, -41.22561001606551, 9.55389260549187, -63.669999968333876, 0.4169924723007425, 15.17290106829154, 12.745643235928943]

    // #右臂
    // #设定了一个工作的基准参考
    // [-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1]
    // #回零
    // [0,0,0,0,0,0,0]
    // #目标位置姿态逆解到约束构型
    // [-26.126455465388723, -41.42961256959308, 12.382716991277716, -63.669999968336434, -15.339360903337091, 14.136820316614646, -5.018123071079321]
    // #调整臂角度
    // [-13.645291371687428, -41.22530974382568, -9.554071963583754, -63.669999968336434, -0.41009102022740107, 15.172926189678035, -12.749249430475857]
    // '''
}
