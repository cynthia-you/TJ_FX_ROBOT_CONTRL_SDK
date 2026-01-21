#include "FxRobot.h"
#include "PointSet.h"

// 实现C风格包装函数
extern "C" {

void* FX_CPointSet_Create() {
    return new CPointSet();
}

void FX_CPointSet_Destroy(void* pset) {
    if (pset) {
        delete static_cast<CPointSet*>(pset);
    }
}

FX_BOOL FX_CPointSet_OnInit(void* pset, FX_INT32L ptype) {
    if (!pset) return FX_FALSE;
    CPointSet* pointSet = static_cast<CPointSet*>(pset);
    return pointSet->OnInit(static_cast<PoinType>(ptype)) ? FX_TRUE : FX_FALSE;
}

FX_INT32L FX_CPointSet_OnGetPointNum(void* pset) {
    if (!pset) return 0;
    CPointSet* pointSet = static_cast<CPointSet*>(pset);
    return pointSet->OnGetPointNum();
}

FX_DOUBLE* FX_CPointSet_OnGetPoint(void* pset, FX_INT32L pos) {
    if (!pset) return nullptr;
    CPointSet* pointSet = static_cast<CPointSet*>(pset);
    return pointSet->OnGetPoint(pos);
}

FX_BOOL FX_CPointSet_OnSetPoint(void* pset, FX_DOUBLE point_value[]) {
    if (!pset) return FX_FALSE;
    CPointSet* pointSet = static_cast<CPointSet*>(pset);
    return pointSet->OnSetPoint(point_value) ? FX_TRUE : FX_FALSE;
}

// C风格包装的MOVLA函数
FX_BOOL FX_Robot_PLN_MOVLA_C(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC,
                            Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, void* ret_pset) {
    if (!ret_pset) return FX_FALSE;
    CPointSet* pointSet = static_cast<CPointSet*>(ret_pset);
    // 调用原始的C++函数
    return FX_Robot_PLN_MOVLA(RobotSerial, Start_XYZABC, End_XYZABC,
                             Ref_Joints, Vel, ACC, pointSet);
}

FX_BOOL  FX_Robot_PLN_MOVL_KeepJA_C(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints,
                            FX_DOUBLE vel, FX_DOUBLE acc, void* ret_pset){
    if (!ret_pset) return FX_FALSE;
    CPointSet* pointSet = static_cast<CPointSet*>(ret_pset);
    return FX_Robot_PLN_MOVL_KeepJA(RobotSerial, startjoints, stopjoints,
                            vel, acc, pointSet);

}

} // extern "C"
