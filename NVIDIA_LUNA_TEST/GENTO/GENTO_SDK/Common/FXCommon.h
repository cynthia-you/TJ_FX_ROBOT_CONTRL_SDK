#ifndef FX_FXCOMMON_H_
#define FX_FXCOMMON_H_
#include "FxType.h"

extern FX_INT32 FX_ARM0_DOF;
extern FX_INT32 FX_ARM1_DOF;
extern FX_INT32 FX_HEAD_DOF;
extern FX_INT32 FX_BODY_DOF;
extern FX_INT32 FX_LIFT_DOF;

typedef enum
{
    FX_OBJ_ARM0 = 0,
    FX_OBJ_ARM1 = 1,
    FX_OBJ_HEAD = 2,
    FX_OBJ_BODY = 3,
    FX_OBJ_LIFT = 4,
    FX_OBJ_NUM = 5,
} FXObjType;

#define FX_OBJ_ARM0_FLAG (1 << 0)
#define FX_OBJ_ARM1_FLAG (1 << 1)
#define FX_OBJ_HEAD_FLAG (1 << 2)
#define FX_OBJ_BODY_FLAG (1 << 3)
#define FX_OBJ_LIFT_FLAG (1 << 4)
#define FX_OBJ_ALL_FLAG (FX_OBJ_ARM0_FLAG | FX_OBJ_ARM1_FLAG | FX_OBJ_HEAD_FLAG | FX_OBJ_BODY_FLAG | FX_OBJ_LIFT_FLAG)

typedef enum
{
    FX_PARAM_TYPE_INT,
    FX_PARAM_TYPE_FLOAT
} FXParamType;

typedef enum
{
    FX_TERMINAL_ARM0 = 0,
    FX_TERMINAL_ARM1 = 1,
} FXTerminalType;

typedef enum
{
    FX_CHN_CANFD = 1,
    FX_CHN_485A = 2,
    FX_CHN_485B = 3,
} FXChnType;

typedef enum
{
    FX_STATE_IDLE = 0,
    FX_STATE_POSITION = 1,
    FX_STATE_IMP_JOINT = 2,
    FX_STATE_IMP_CART = 3,
    FX_STATE_IMP_FORCE = 4,
    FX_STATE_DRAG_JOINT = 5,
    FX_STATE_DRAG_CART_X = 6,
    FX_STATE_DRAG_CART_Y = 7,
    FX_STATE_DRAG_CART_Z = 8,
    FX_STATE_DRAG_CART_R = 9,
    FX_STATE_RELEASE = 10,
    FX_STATE_PD = 11,
    FX_STATE_ERROR = 100,
    FX_STATE_TRANSFERRING = 101,
    FX_STATE_UNKNOWN = 200,
} FXStateType;

typedef enum
{
    FX_FORCE_DIR_X = 0,
    FX_FORCE_DIR_Y = 1,
    FX_FORCE_DIR_Z = 2,
    FX_FORCE_VALUE = 3,
    FX_FORCE_DISTANCE = 4,
    FX_FORCE_DEF_NUM = 5,
} FXForceDef;

typedef enum
{
    FX_TORQUE_DIR_A = 0,
    FX_TORQUE_DIR_B = 1,
    FX_TORQUE_DIR_C = 2,
    FX_TORQUE_VALUE = 3,
    FX_TORQUE_ANGLE = 4,
    FX_TORQUE_DEF_NUM = 5,
} FXTorqueDef;

#endif
