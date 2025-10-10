#ifndef _FX_DT_H_
#define _FX_DT_H_

#include "FXMath.h"


typedef struct
{
	FX_DOUBLE m_x1;
	FX_DOUBLE m_x2;
	FX_DOUBLE m_r;
	FX_DOUBLE m_h;
	FX_DOUBLE m_h0;
	FX_DOUBLE m_d;
	FX_BOOL   m_InitTag;
	FX_BOOL   m_FristTag;
	FX_DOUBLE m_max_vel;
	FX_BOOL   m_o_lmt;
	FX_DOUBLE m_v_lmt;
	FX_DOUBLE m_ts;
}FXVDT;


#ifdef __cplusplus
extern "C" {
#endif

FX_BOOL		FXVDT_OnInit(FXVDT* obj, FX_DOUBLE step_interval, FX_DOUBLE max_vel, FX_DOUBLE max_acc);
FX_BOOL		FXVDT_OnEmpty(FXVDT* obj);
FX_BOOL		FXVDT_OnDT(FXVDT* obj, FX_DOUBLE p, FX_DOUBLE* ret_p);
FX_BOOL		FXVDT_OnSetRunTimeLmt(FXVDT* obj, FX_DOUBLE vel_lmt, FX_DOUBLE acc_lmt);

#ifdef __cplusplus
}
#endif

typedef struct
{
	FXVDT 		m_tds[8];
	FX_INT32L   m_AxisNum;
	FX_BOOL   m_InitTag;
	FX_BOOL   m_FristTag;
	FX_DOUBLE m_lastVel[8];
	FX_DOUBLE m_lastAcc[8];
	FX_DOUBLE m_lastPos[8];
	FX_DOUBLE m_lastCMDPos[8];
	FX_DOUBLE m_values[8][100];
	FX_INT32L   m_vsize[8];
	FX_DOUBLE m_dvsize[8];
	FX_DOUBLE m_Intvalue[8];
	FX_INT32L   m_vcnt[8];
	FX_INT32L   m_wpos[8];
} FX_DTS;


#ifdef __cplusplus
extern "C" {
#endif

FX_BOOL FXNSyncDTS_OnInit(FX_DTS* obj, FX_INT32L axis_num, FX_DOUBLE step_interval, FX_DOUBLE max_vel[8], FX_DOUBLE max_acc[8], FX_DOUBLE max_jrk[8]);
FX_BOOL FXNSyncDTS_OnEmpty(FX_DTS* obj);
FX_BOOL FXNSyncDTS_OnSetRunTimeLmt(FX_DTS* obj, FX_DOUBLE vel_lmt[8], FX_DOUBLE acc_lmt[8]);
FX_BOOL FXNSyncDTS_OnDT(FX_DTS* obj, FX_DOUBLE target_pos[8], FX_DOUBLE ret_pos[8]);
FX_BOOL FXNSyncDTS_OnDT_U(FX_DTS* obj, FX_DOUBLE ret_pos[8]);
FX_BOOL FXNSyncDTS_OnGetCurPVA(FX_DTS* obj, FX_DOUBLE target[8], FX_DOUBLE pos[8], FX_DOUBLE vel[8], FX_DOUBLE acc[8]);
FX_BOOL FXNSyncDTS_OnGetCurPVA_U(FX_DTS* obj, FX_DOUBLE pos[8], FX_DOUBLE vel[8], FX_DOUBLE acc[8]);


#ifdef __cplusplus
}
#endif


#endif

