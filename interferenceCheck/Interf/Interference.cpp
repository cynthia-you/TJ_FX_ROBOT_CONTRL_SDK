#include "Interference.h"
#include "ICBase.h"
#include "math.h"



struct FX_InterfContext
{
    CICBase* _base;
    bool _init_flag;
};

FX_InterfHandle FX_Interf_Create()
{
    FX_InterfContext* ctx = new FX_InterfContext();
  
    ctx->_base = new CICBase();
    ctx->_init_flag = false;
    return ctx;
}

void FX_Interf_Destroy(FX_InterfHandle handle)
{
    if (handle)
        delete handle;
    
}

int FX_Interf_Init(FX_InterfHandle handle, char *CordDef, char *CalLinkDef, char *InputDef, char *ConvexDef, char *ICDef)
{
    if (!handle || !CordDef || !CalLinkDef || !InputDef || !ConvexDef || !ICDef)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    bool ret = handle->_base->FX_OnLoadCfg(CordDef, CalLinkDef, InputDef, ConvexDef, ICDef);
    if (!ret)
    {
        return FUNC_RET_OPERATION_FAILED;
    }
    handle->_init_flag = true;
    return FUNC_RET_SUCCESS;
}

int FX_Interf_UpdateCord(FX_InterfHandle handle, long cord_num, double *input)
{
    if (!handle || !handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }    
    if (!handle->_base->FX_OnUpdateCord(cord_num, input))
    {
        return FUNC_RET_OPERATION_FAILED;
    }
    return FUNC_RET_SUCCESS;
}

int FX_Interf_UpdateConvex(FX_InterfHandle handle)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (!handle->_base->FX_OnUpdateConvex())
    {
        return FUNC_RET_OPERATION_FAILED;
    }
    return FUNC_RET_SUCCESS;
}

int FX_Interf_CalcInterfDistance(FX_InterfHandle handle)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (!handle->_base->FX_OnCalInterf())
    {
        return FUNC_RET_OPERATION_FAILED;
    }
    return FUNC_RET_SUCCESS;
}

int FX_Interf_GetCoM(FX_InterfHandle handle, double mcp_average[3])
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    handle->_base->FX_OnGetCoM(mcp_average);
    return FUNC_RET_SUCCESS;
}

int FX_Interf_GetCordsNum(FX_InterfHandle handle, long *cords_num)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    long num = 0;
    handle->_base->FX_OnGetCordsNum(num);
    * cords_num = num;
    return FUNC_RET_SUCCESS;
}

int FX_Interf_GetMass(FX_InterfHandle handle, double *quality)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    handle->_base->FX_OnGetM(quality);
    return FUNC_RET_SUCCESS;
}

int FX_Interf_GetMinSpan(FX_InterfHandle handle, double *MinSpan)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    handle->_base->FX_OnGetMinSpan(MinSpan);
    return FUNC_RET_SUCCESS;
}

int FX_Interf_GetInterfSpan(FX_InterfHandle handle, long serial, double *MinSpan)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    double minspan = 0;
    handle->_base->FX_OnGetInterfSpan(serial, minspan);
    *MinSpan = minspan;
    return FUNC_RET_SUCCESS;
}

int FX_Interf_OnIsInterf(FX_InterfHandle handle, double * inteThsh)
{
    if (!handle->_init_flag)
    {
        return FUNC_RET_INVALID_INPUT_ARG;
    }

    if(handle->_base->FX_OnIsInterf(inteThsh))
    {
        return FUNC_RET_ISInterference;
    }

	return FUNC_RET_NOInterference;
}


