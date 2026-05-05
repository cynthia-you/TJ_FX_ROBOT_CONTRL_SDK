#include "L0Kinematics.h"

/* 内部上下文结构体，持有所有 C++ 对象 */
struct FX_MotionContext
{
    CFxKineIF kine_left;
    CFxKineIF kine_right;
    CFxKineMAX body_kine;
    CFxPln planner;
};

static void array_to_matrix44(double arr[16], Matrix4 mat)
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            mat[i][j] = arr[i * 4 + j];
}

static void matrix44_to_array(Matrix4 mat, double arr[16])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            arr[i * 4 + j] = mat[i][j];
}

static void copy_vect7(double src[7], Vect7 dst)
{
    for (int i = 0; i < 7; ++i)
        dst[i] = src[i];
}

static void copy_vect6(double src[6], Vect6 dst)
{
    for (int i = 0; i < 6; ++i)
        dst[i] = src[i];
}

/* ==================== 生命周期 ==================== */
FX_MotionHandle FX_L0_Kinematics_create(void)
{
    FX_MotionContext *ctx = new FX_MotionContext();
    if (!ctx)
        return nullptr;
    return ctx;
}

void FX_L0_Kinematics_destroy(FX_MotionHandle handle)
{
    if (handle)
        delete handle;
}

void FX_L0_Kinematics_log_switch(FX_MotionHandle handle, int on)
{
    if (!handle)
        return;
    handle->kine_left.L0_OnLogSwitch(on);
    handle->kine_right.L0_OnLogSwitch(on);
    handle->body_kine.L0_OnLogSwitch(on);
    handle->planner.L0_OnLogSwitch(on);
}

/* ==================== 初始化 ==================== */
int FX_L0_Kinematics_init_single_arm(FX_MotionHandle handle,
                                     int RobotSerial, int *type, double DH[8][4], double PNVA[8][4], double BOUND[4][3],
                                     double GRV[3], double MASS[7], double MCP[7][3], double I[7][6])
{
    if (!handle)
        return FX_MOTION_ERROR;
    if (RobotSerial == 0)
    {
        if (!handle->kine_left.L0_OnInitEnv(0, type, DH, PNVA, BOUND, GRV, MASS, MCP, I))
            return FX_MOTION_ERROR;
    }
    else if (RobotSerial == 1)
    {
        if (!handle->kine_right.L0_OnInitEnv(1, type, DH, PNVA, BOUND, GRV, MASS, MCP, I))
            return FX_MOTION_ERROR;
    }
    else
    {
        return FX_MOTION_ERROR;
    }
    if (!handle->planner.L0_OnInitEnv_SingleArm(RobotSerial, type, DH, PNVA, BOUND))
        return FX_MOTION_ERROR;
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_init_dual_arm(FX_MotionHandle handle, int type[2], double DH[2][8][4], double PNVA[2][8][4], double BOUND[2][4][3])
{
    if (!handle)
        return FX_MOTION_ERROR;
    if (!handle->planner.L0_OnInitEnv_DualArm(type, DH, PNVA, BOUND))
        return FX_MOTION_ERROR;
    return FX_MOTION_OK;
}

/* ==================== 单臂运动学 ==================== */
int FX_L0_Kinematics_forward_kinematics(FX_MotionHandle handle, int robot_serial,
                                        double joints[7], double pose_matrix[16])
{
    if (!handle || !joints || !pose_matrix)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    Vect7 jv;
    copy_vect7(joints, jv);
    Matrix4 mat;
    if (!kine->L0_OnSolveArmFK(jv, mat))
        return FX_MOTION_ERROR;
    matrix44_to_array(mat, pose_matrix);
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_jacobian(FX_MotionHandle handle, int robot_serial,
                              double joints[7], double jacobian[42])
{
    if (!handle || !joints || !jacobian)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    Vect7 jv;
    copy_vect7(joints, jv);
    double jcb[6][7];
    if (!kine->L0_OnSolveArmJcb(jv, jcb))
        return FX_MOTION_ERROR;
    int idx = 0;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 7; ++j)
            jacobian[idx++] = jcb[i][j];
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_inverse_kinematics(FX_MotionHandle handle, int robot_serial,
                                        FX_InvKineSolverParams *params)
{
    if (!handle || !params)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    FX_InvKineSolvePara para;
    memset(&para, 0, sizeof(para));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            para.m_Input_IK_TargetTCP[i][j] = params->target_pose[i * 4 + j];
        }
    }

    for (int i = 0; i < 7; ++i)
    {
        para.m_Input_IK_RefJoint[i] = params->ref_joints[i];
    }

    if (!kine->L0_OnSolveArmIK(&para))
        return FX_MOTION_ERROR;

    for (int i = 0; i < 7; ++i)
    {
        params->solution[i] = para.m_Output_RetJoint[i];
    }
    params->solution_valid = 1;
    return FX_MOTION_OK;
}

/* ==================== MAX 身体运动学 ==================== */
int FX_L0_Kinematics_set_body_condition(FX_MotionHandle handle,
                                        double std_body[3], double k_body[3],
                                        double std_left_len, double k_left,
                                        double std_right_len, double k_right)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect3 sb, kb;
    for (int i = 0; i < 3; ++i)
    {
        sb[i] = std_body[i];
        kb[i] = k_body[i];
    }
    handle->body_kine.L0_OnSetCondition(sb, kb, std_left_len, k_left, std_right_len, k_right);
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_body_forward(FX_MotionHandle handle, double jv[3], double left_shoulder_matrix[16], double right_shoulder_matrix[16])
{
    if (!jv || !left_shoulder_matrix || !right_shoulder_matrix)
        return FX_MOTION_ERROR;
    Vect3 jv3 = {jv[0], jv[1], jv[2]};
    Matrix4 left, right;
    handle->body_kine.L0_OnKineLR(jv3, left, right);
    matrix44_to_array(left, left_shoulder_matrix);
    matrix44_to_array(right, right_shoulder_matrix);
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_calc_body_position(FX_MotionHandle handle, double left_tcp[3], double right_tcp[3],
                                        double out_body_joints[3])
{
    if (!left_tcp || !right_tcp || !out_body_joints)
        return FX_MOTION_ERROR;
    Vect3 lt = {left_tcp[0], left_tcp[1], left_tcp[2]};
    Vect3 rt = {right_tcp[0], right_tcp[1], right_tcp[2]};
    Vect3 out;
    handle->body_kine.L0_OnCalBody(lt, rt, out);
    out_body_joints[0] = out[0];
    out_body_joints[1] = out[1];
    out_body_joints[2] = out[2];
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_calc_body_position_with_ref(FX_MotionHandle handle, double ref_body_joints[3],
                                                 double left_tcp[3], double right_tcp[3],
                                                 double out_body_joints[3])
{
    if (!ref_body_joints || !left_tcp || !right_tcp || !out_body_joints)
        return FX_MOTION_ERROR;
    Vect3 ref = {ref_body_joints[0], ref_body_joints[1], ref_body_joints[2]};
    Vect3 lt = {left_tcp[0], left_tcp[1], left_tcp[2]};
    Vect3 rt = {right_tcp[0], right_tcp[1], right_tcp[2]};
    Vect3 out;
    handle->body_kine.L0_OnCalBody_withref(ref, lt, rt, out);
    out_body_joints[0] = out[0];
    out_body_joints[1] = out[1];
    out_body_joints[2] = out[2];
    return FX_MOTION_OK;
}

/* ==================== 运动规划（单臂） ==================== */
int FX_L0_Kinematics_plan_joint_move(FX_MotionHandle handle, int robot_serial,
                                     double start_joints[7], double end_joints[7],
                                     double vel_ratio, double acc_ratio,
                                     double *point_set_handle, int *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect7 s, e;
    copy_vect7(start_joints, s);
    copy_vect7(end_joints, e);
    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.L0_OnMovJ(s, e, vel_ratio, acc_ratio, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);

    return FX_MOTION_OK;
}

int FX_L0_Kinematics_plan_linear_move(FX_MotionHandle handle, int robot_serial,
                                      double start_xyzabc[6], double end_xyzabc[6],
                                      double ref_joints[7],
                                      double vel, double acc, int freq,
                                      double *point_set_handle, int *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect6 s, e;
    copy_vect6(start_xyzabc, s);
    copy_vect6(end_xyzabc, e);
    Vect7 ref;
    copy_vect7(ref_joints, ref);
    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.L0_OnMovL(s, e, ref, vel, acc, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);

    return FX_MOTION_OK;
}

int FX_L0_Kinematics_plan_linear_keep_joints(FX_MotionHandle handle, int robot_serial,
                                             double start_joints[7], double end_joints[7],
                                             double vel, double acc, int freq,
                                             double *point_set_handle, int *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect7 s, e;
    copy_vect7(start_joints, s);
    copy_vect7(end_joints, e);
    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.L0_OnMovL_KeepJ(s, e, vel, acc, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);

    return FX_MOTION_OK;
}

int FX_L0_Kinematics_multi_points_set_movl_start(FX_MotionHandle handle, int robot_serial,
                                                 double ref_joints[7],
                                                 double start_xyzabc[6], double end_xyzabc[6],
                                                 double allow_range, int zsp_type,
                                                 double zsp_para[6],
                                                 double vel, double acc, int freq)
{
    if (!handle || !ref_joints || !start_xyzabc || !end_xyzabc || !zsp_para)
        return FX_MOTION_ERROR;

    Vect7 ref;
    Vect6 start, end, zsp;
    copy_vect7(ref_joints, ref);
    copy_vect6(start_xyzabc, start);
    copy_vect6(end_xyzabc, end);
    copy_vect6(zsp_para, zsp);

    if (!handle->planner.L0_MultiPoints_Set_MovL_Start(ref, start, end, allow_range, zsp_type, zsp, vel, acc, freq))
        return FX_MOTION_ERROR;

    return FX_MOTION_OK;
}

int FX_L0_Kinematics_multi_points_set_movl_next_points(FX_MotionHandle handle, int robot_serial,
                                                       double next_xyzabc[6],
                                                       double allow_range, int zsp_type,
                                                       double zsp_para[6],
                                                       double vel, double acc)
{
    if (!handle || !next_xyzabc || !zsp_para)
        return FX_MOTION_ERROR;

    Vect6 next, zsp;
    copy_vect6(next_xyzabc, next);
    copy_vect6(zsp_para, zsp);

    if (!handle->planner.L0_MultiPoints_Set_MovL_NextPoints(next, allow_range, zsp_type, zsp, vel, acc))
        return FX_MOTION_ERROR;

    return FX_MOTION_OK;
}

int FX_L0_Kinematics_multi_points_get_movl_path(FX_MotionHandle handle,
                                                double *point_set_handle, int *point_num)
{
    if (!handle || !point_set_handle || !point_num)
        return FX_MOTION_ERROR;

    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!pset)
        return FX_MOTION_ERROR;

    if (!handle->planner.L0_MultiPoints_Get_MovL_Path(pset))
    {
        FX_L0_CPointSet_Destroy(pset_c);
        return FX_MOTION_ERROR;
    }

    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
    {
        FX_L0_CPointSet_Destroy(pset_c);
        return FX_MOTION_ERROR;
    }

    FX_L0_CPointSet_Destroy(pset_c);
    return FX_MOTION_OK;
}

/* ==================== 双臂同步规划 ==================== */
int FX_L0_Kinematics_plan_dual_arm_fixed_body(FX_MotionHandle handle,
                                              DualArmFixedBodyParams *params,
                                              double *left_point_set, double *right_point_set, int *point_num)
{
    if (!handle || !params)
        return FX_MOTION_ERROR;

    DualArm_FixedBody da;

    da.World_Co_Flag = params->world_co_flag;

    for (int i = 0; i < 6; ++i)
    {
        da.Left_Arm_Start_XYZABC[i] = params->left_start_xyzabc[i];
        da.Left_Arm_End_XYZABC[i] = params->left_end_xyzabc[i];
        da.Left_Arm_ZSP_Para[i] = params->left_zsp_para[i];
    }
    for (int i = 0; i < 7; ++i)
    {
        da.Left_Arm_Ref_Joints[i] = params->left_ref_joints[i];
    }
    da.Left_Arm_ZSP_Type = params->left_zsp_type;

    for (int i = 0; i < 6; ++i)
    {
        da.Right_Arm_Start_XYZABC[i] = params->right_start_xyzabc[i];
        da.Right_Arm_End_XYZABC[i] = params->right_end_xyzabc[i];
        da.Right_Arm_ZSP_Para[i] = params->right_zsp_para[i];
    }
    for (int i = 0; i < 7; ++i)
    {
        da.Right_Arm_Ref_Joints[i] = params->right_ref_joints[i];
    }
    da.Right_Arm_ZSP_Type = params->right_zsp_type;

    for (int i = 0; i < 3; ++i)
    {
        da.Max_Body_Start_PRR[i] = params->body_start_prr[i];
    }

    da.Vel = params->vel;
    da.Acc = params->acc;
    da.Freq = params->freq;
    da.Sync_Type = params->sync_type;

    void *pset_left = FX_L0_CPointSet_Create();
    void *pset_right = FX_L0_CPointSet_Create();
    CPointSet *left_pset = reinterpret_cast<CPointSet *>(pset_left);
    CPointSet *right_pset = reinterpret_cast<CPointSet *>(pset_right);

    if (!handle->planner.L0_OnMovL_DualArm_FixBody(&da, left_pset, right_pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    int left_num = left_pset->OnGetPointNum();
    int right_num = right_pset->OnGetPointNum();

    if (left_num != right_num)
    {
        // 点数不一致，说明规划失败
        return FX_MOTION_ERROR;
    }
    else
    {
        *point_num = left_num;
    }

    if (!FX_L0_CPointSet_OnAppendPoint(pset_left, left_point_set) || !FX_L0_CPointSet_OnAppendPoint(pset_right, right_point_set))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_left);
    FX_L0_CPointSet_Destroy(pset_right);

    return FX_MOTION_OK;
}
/* ===================== pointset ==================*/
void *FX_L0_CPointSet_Create()
{
    return new CPointSet();
}

void FX_L0_CPointSet_Destroy(void *pset)
{
    if (pset)
    {
        delete static_cast<CPointSet *>(pset);
    }
}

int FX_L0_CPointSet_OnInit(void *pset, int ptype)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnInit(static_cast<PoinType>(ptype)) ? 1 : 0;
}

int FX_L0_CPointSet_OnGetPointNum(void *pset)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnGetPointNum();
}

double *FX_L0_CPointSet_OnGetPoint(void *pset, int pos)
{
    if (!pset)
        return nullptr;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnGetPoint(pos);
}

int FX_L0_CPointSet_OnSetPoint(void *pset, double point_value[])
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnSetPoint(point_value) ? 1 : 0;
}

int FX_L0_CPointSet_OnAppendPoint(void *pset, double *point_value)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    int num = pointSet->OnGetPointNum();

    int i = 0;
    for (i = 0; i < num; i++)
    {
        // pointset中的数据换成double序列
        double *p = pointSet->OnGetPoint(i);
        if (!p)
            return 0;

        for (int j = 0; j < 7; j++)
        {
            point_value[i * 7 + j] = p[j];
        }
    }

    return 1;
}
/* ==================== 辅助工具 ==================== */
void FX_L0_XYZABC2Matrix(double xyzabc[6], double matrix[16])
{
    CFxPln pln;
    Matrix4 m;
    double arr[6];
    for (int i = 0; i < 6; ++i)
        arr[i] = xyzabc[i];
    pln.L0_XYZABC2Matrix4_DEG(arr, m);
    matrix44_to_array(m, matrix);
}

void FX_L0_Matrix2XYZABC(double matrix[16], double xyzabc[6])
{
    CFxPln pln;
    Matrix4 m;
    array_to_matrix44(matrix, m);
    double out[6];
    pln.L0_Matrix42XYZABC_DEG(m, out);
    for (int i = 0; i < 6; ++i)
        xyzabc[i] = out[i];
}