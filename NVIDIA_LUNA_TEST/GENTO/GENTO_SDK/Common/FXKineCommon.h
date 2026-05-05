#ifndef FX_FXKINECOMMON_H_
#define FX_FXKINECOMMON_H_
#include "FxType.h"

/* 返回值约定：0 表示成功，非 0 表示失败（具体错误码可根据需要扩展） */
#define FX_MOTION_OK 0
#define FX_MOTION_ERROR -1

/* 逆运动学参数结构体（需与原有 FX_InvKineSolvePara 对应） */
typedef struct
{
    FX_DOUBLE target_pose[16]; /* 目标位姿矩阵 4x4 */
    FX_DOUBLE ref_joints[7];   /* 参考关节角 */
    FX_DOUBLE solution[7];     /* 输出解 */
    FX_INT32 solution_valid;   /* 输出是否有效 */
} FX_InvKineSolverParams;

/* 双臂固定身体规划参数结构体 */
typedef struct
{
    /* 公共规划参数 */
    FX_INT32 world_co_flag; /* 位姿是否世界坐标系 (1:世界,0:基座) */
    FX_DOUBLE vel;          /* 速度 */
    FX_DOUBLE acc;          /* 加速度 */
    FX_INT32 freq;          /* 规划频率 */
    FX_INT32 sync_type;     /* 同步类型 */

    /* 左臂参数 */
    FX_DOUBLE left_start_xyzabc[6]; /* 左臂起始位姿 (XYZABC, 度) */
    FX_DOUBLE left_end_xyzabc[6];   /* 左臂结束位姿 */
    FX_DOUBLE left_ref_joints[7];   /* 左臂参考关节角 (度) */
    FX_INT32 left_zsp_type;         /* 左臂 ZSP 类型 */
    FX_DOUBLE left_zsp_para[6];     /* 左臂 ZSP 参数 */

    /* 右臂参数 */
    FX_DOUBLE right_start_xyzabc[6]; /* 右臂起始位姿 */
    FX_DOUBLE right_end_xyzabc[6];   /* 右臂结束位姿 */
    FX_DOUBLE right_ref_joints[7];   /* 右臂参考关节角 */
    FX_INT32 right_zsp_type;         /* 右臂 ZSP 类型 */
    FX_DOUBLE right_zsp_para[6];     /* 右臂 ZSP 参数 */

    /* 身体参数 */
    FX_DOUBLE body_start_prr[3]; /* 身体起始关节 (升降,俯仰,横滚) */

} DualArmFixedBodyParams;

#endif