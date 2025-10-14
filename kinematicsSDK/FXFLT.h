#ifndef _FX_FLT_H_
#define _FX_FLT_H_

#include "FXMath.h"


typedef struct
{
	FX_DOUBLE	m_datas[8][100];
	FX_DOUBLE	m_LastRet[8];
	FX_INT32L   m_axis_num;
	FX_INT32L   m_curpos;
	FX_INT32L   m_size;
	FX_INT32L   m_init_tag;
	FX_BOOL		m_frist_tag;
	FX_DOUBLE	m_ts;
} FXAVGF;


#ifdef __cplusplus
extern "C" {
#endif

	FX_BOOL FXAVGF_OnInit(FXAVGF* obj, FX_INT32L axis_num, FX_INT32L filter_size, FX_DOUBLE ts);
	FX_BOOL FXAVGF_OnEmpty(FXAVGF* obj);
	FX_BOOL FXAVGF_OnFilt(FXAVGF* obj, FX_DOUBLE data[8], FX_DOUBLE ret_data[8], FX_DOUBLE ret_vel[7]);

#ifdef __cplusplus
}
#endif

typedef struct
{
	FX_DOUBLE	m_current_value;
	FX_DOUBLE   m_new_value_ratio;
	FX_DOUBLE   m_max_step;
	FX_DOUBLE   m_min_step;
	FX_BOOL     m_is_zero;
	FX_BOOL     m_is_one;
} FX_SmoothValue;


#ifdef __cplusplus
extern "C" {
#endif

	FX_BOOL FXSMV_OnInit(FX_SmoothValue* obj, FX_DOUBLE	current_value,FX_DOUBLE   new_value_ratio, FX_DOUBLE   max_step, FX_DOUBLE   min_step);
	FX_VOID FXSMV_OnUpdateV(FX_SmoothValue* obj, FX_DOUBLE   target_value, FX_DOUBLE * current_value);
	FX_VOID FXSMV_OnUpdateVZO(FX_SmoothValue* obj, FX_DOUBLE   target_value, FX_DOUBLE* current_value, FX_BOOL* is_zero, FX_BOOL* is_one);
	FX_VOID FXSMV_OnUpdateZO(FX_SmoothValue* obj, FX_DOUBLE   target_value, FX_BOOL* is_zero, FX_BOOL* is_one);
#ifdef __cplusplus
}
#endif





#endif
