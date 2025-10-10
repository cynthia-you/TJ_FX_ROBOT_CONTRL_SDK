#include "FXDT.h"

FX_BOOL FXVDT_OnInit(FXVDT* obj, FX_DOUBLE step_interval, FX_DOUBLE max_vel, FX_DOUBLE max_acc)
{
	obj->m_InitTag = FX_FALSE;
	obj->m_o_lmt = FX_FALSE;
	if (step_interval < 1.0 || step_interval > 1000.0
		|| max_vel < 0.1 || max_vel > 5000.0
		|| max_acc < 0.01 || max_acc > 50000.0)
	{
		return FX_FALSE;
	}
	obj->m_ts = step_interval / 1000.0; // unit in sec
	obj->m_x1 = 0;
	obj->m_x2 = 0;
	obj->m_r = max_acc;
	obj->m_h = obj->m_ts;
	obj->m_d = obj->m_r * obj->m_ts * obj->m_ts;
	obj->m_h0 = 0.01;
	obj->m_InitTag = FX_TRUE;
	obj->m_FristTag = FX_TRUE;
	obj->m_max_vel = max_vel;
	obj->m_o_lmt = FX_FALSE;
	obj->m_v_lmt = max_vel;
	return FX_TRUE;

}

FX_BOOL  FXVDT_OnEmpty(FXVDT* obj)
{
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	obj->m_FristTag = FX_TRUE;
	return FX_TRUE;
}

FX_BOOL FXVDT_OnDT(FXVDT* obj, FX_DOUBLE p, FX_DOUBLE* ret_p)
{
	FX_DOUBLE d;
	FX_DOUBLE d0;
	FX_DOUBLE y;
	FX_DOUBLE a0;
	FX_DOUBLE alp;
	FX_DOUBLE fhan;
	FX_DOUBLE tvel;
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (obj->m_FristTag == FX_TRUE)
	{
		obj->m_x1 = p;
		obj->m_x2 = 0;
		*ret_p = obj->m_x1;
		obj->m_FristTag = FX_FALSE;
		return FX_TRUE;
	}

	d = obj->m_r * obj->m_h0;
	d0 = d * obj->m_h0;
	y = obj->m_x1 - p + obj->m_h0 * obj->m_x2;
	a0 = FX_Sqrt(d * d + 8 * obj->m_r * FX_Fabs(y));

	if (FX_Fabs(y) <= d0)
	{
		alp = obj->m_x2 + y / obj->m_h0;

	}
	else
	{
		alp = obj->m_x2 + (FX_Value_Sig(y) * 0.5 * (a0 - d));
	}

	if (FX_Fabs(alp) <= d)
	{
		fhan = -obj->m_r * alp / d;
	}
	else
	{
		fhan = -obj->m_r * FX_Value_Sig(alp);
	}

	tvel = obj->m_x2 + obj->m_h * fhan;

	if (obj->m_o_lmt == FX_FALSE)
	{
		if (FX_Fabs(tvel) > obj->m_max_vel)
		{
			if (tvel < 0)
			{
				obj->m_x2 = -obj->m_max_vel;
			}
			else
			{
				obj->m_x2 = obj->m_max_vel;
			}

			obj->m_x1 += obj->m_h * obj->m_x2;
		}
		else
		{
			obj->m_x2 += obj->m_h * fhan;
			obj->m_x1 += obj->m_h * obj->m_x2;
		}
	}
	else
	{
		if (obj->m_x2 > 0)
		{
			obj->m_x2 -= obj->m_r * obj->m_h;
			if (obj->m_x2 < obj->m_v_lmt)
			{
				obj->m_o_lmt = FX_FALSE;
				obj->m_max_vel = obj->m_v_lmt;
			}
		}
		else
		{
			obj->m_x2 += obj->m_r * obj->m_h;
			if (obj->m_x2 > -obj->m_v_lmt)
			{
				obj->m_o_lmt = FX_FALSE;
				obj->m_max_vel = obj->m_v_lmt;
			}
		}
		obj->m_x1 += obj->m_h * obj->m_x2;
	}
	*ret_p = obj->m_x1;
	return FX_TRUE;
}


FX_BOOL FXVDT_OnSetRunTimeLmt(FXVDT* obj, FX_DOUBLE vel_lmt, FX_DOUBLE acc_lmt)
{
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (FX_Fabs(vel_lmt - obj->m_v_lmt) < 0.1 && FX_Fabs(obj->m_r - acc_lmt) < 0.1)
	{
		return FX_TRUE;
	}
	{//Update acc_lmt
		obj->m_r = acc_lmt;
		obj->m_d = obj->m_r * obj->m_ts * obj->m_ts;
	}
	if (vel_lmt > obj->m_max_vel)
	{
		obj->m_max_vel = vel_lmt;
		obj->m_v_lmt = vel_lmt;
		obj->m_o_lmt = FX_FALSE;
	}
	else
	{
		if (FX_Fabs(obj->m_x2) < vel_lmt)
		{
			obj->m_max_vel = vel_lmt;
			obj->m_v_lmt = vel_lmt;
			obj->m_o_lmt = FX_FALSE;
		}
		else
		{
			obj->m_v_lmt = vel_lmt;
			obj->m_o_lmt = FX_TRUE;
		}
	}
	return FX_TRUE;
}

FX_BOOL FXNSyncDTS_OnInit(FX_DTS* obj, FX_INT32L axis_num, FX_DOUBLE step_interval, FX_DOUBLE max_vel[8], FX_DOUBLE max_acc[8], FX_DOUBLE max_jrk[8])
{
	FX_INT32L i;
	obj->m_InitTag = FX_FALSE;
	obj->m_AxisNum = 0;
	if (axis_num <= 0 || axis_num > 8)
	{
		return FX_FALSE;
	}

	for (i = 0; i < axis_num; i++)
	{
		if (max_jrk[i] < 1 || max_jrk[i] > 10000000)
		{
			return FX_FALSE;
		}
		if (FXVDT_OnInit(&obj->m_tds[i], step_interval, max_vel[i], max_acc[i]) == FX_FALSE)
		{
			return FX_FALSE;
		}

		obj->m_dvsize[i] = 1000.0 * max_acc[i] / max_jrk[i] / step_interval;
		obj->m_vsize[i] = obj->m_dvsize[i];
		if (obj->m_vsize[i] < 3)
		{
			obj->m_vsize[i] = 3;
		}
		if (obj->m_vsize[i] > 100)
		{
			obj->m_vsize[i] = 100;
		}
		obj->m_dvsize[i] = obj->m_vsize[i];
		obj->m_vcnt[i] = 0;
		obj->m_wpos[i] = 0;
	}
	obj->m_InitTag = FX_TRUE;
	obj->m_AxisNum = axis_num;
	obj->m_FristTag = FX_TRUE;
	return FX_TRUE;
}
FX_BOOL FXNSyncDTS_OnEmpty(FX_DTS* obj)
{
	FX_INT32L i;
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	for (i = 0; i < obj->m_AxisNum; i++)
	{
		if (FXVDT_OnEmpty(&obj->m_tds[i]) == FX_FALSE)
		{
			return FX_FALSE;
		}
	}
	obj->m_FristTag = FX_TRUE;
	return FX_TRUE;
}

FX_BOOL FXNSyncDTS_OnSetRunTimeLmt(FX_DTS* obj, FX_DOUBLE vel_lmt[8], FX_DOUBLE acc_lmt[8])
{
	FX_INT32L i;
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	for (i = 0; i < obj->m_AxisNum; i++)
	{
		if (FXVDT_OnSetRunTimeLmt(&obj->m_tds[i], vel_lmt[i], acc_lmt[i]) == FX_FALSE)
		{
			return FX_FALSE;
		}
	}
	return FX_TRUE;
}


FX_BOOL FXNSyncDTS_OnGetCurPVA(FX_DTS* obj, FX_DOUBLE target[8], FX_DOUBLE pos[8], FX_DOUBLE vel[8], FX_DOUBLE acc[8])
{
	FX_INT32L i;
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (obj->m_FristTag == FX_TRUE)
	{
		return FX_FALSE;
	}
	for (i = 0; i < obj->m_AxisNum; i++)
	{
		pos[i] = obj->m_lastPos[i];
		vel[i] = obj->m_lastVel[i];
		acc[i] = obj->m_lastAcc[i];
		target[i] = obj->m_lastCMDPos[i];
	}
	return FX_TRUE;
}


FX_BOOL FXNSyncDTS_OnGetCurPVA_U(FX_DTS* obj, FX_DOUBLE pos[8], FX_DOUBLE vel[8], FX_DOUBLE acc[8])
{
	FX_INT32L i;
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	for (i = 0; i < obj->m_AxisNum; i++)
	{
		pos[i] = obj->m_lastPos[i];
		vel[i] = obj->m_lastVel[i];
		acc[i] = obj->m_lastAcc[i];
	}
	return FX_TRUE;
}


FX_BOOL FXNSyncDTS_OnDT(FX_DTS* obj, FX_DOUBLE target_pos[8], FX_DOUBLE ret_pos[8])
{
	if (obj->m_InitTag == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_INT32L i, j;


	if (obj->m_FristTag == FX_TRUE)
	{
		for (i = 0; i < obj->m_AxisNum; i++)
		{
			obj->m_lastCMDPos[i] = target_pos[i];
			FXVDT_OnDT(&obj->m_tds[i], target_pos[i], &ret_pos[i]);
			obj->m_lastPos[i] = ret_pos[i];
			for (j = 0; j < obj->m_vsize[i]; j++)
			{
				obj->m_values[i][j] = ret_pos[i];
			}
			obj->m_Intvalue[i] = ret_pos[i] * obj->m_dvsize[i];
			obj->m_vcnt[i] = 0;
			obj->m_wpos[i] = 0;
			obj->m_lastVel[i] = 0;
			obj->m_lastAcc[i] = 0;
		}
		obj->m_FristTag = FX_FALSE;
		return FX_TRUE;
	}
	FX_DOUBLE tmp_ret[8];
	for (i = 0; i < obj->m_AxisNum; i++)
	{
		obj->m_lastCMDPos[i] = target_pos[i];
		FXVDT_OnDT(&obj->m_tds[i], target_pos[i], &tmp_ret[i]);
	}

	for (i = 0; i < obj->m_AxisNum; i++)
	{
		obj->m_lastPos[i] = tmp_ret[i];
	}

	FX_BOOL uptag = FX_FALSE;
	for (i = 0; i < obj->m_AxisNum; i++)
	{
		obj->m_Intvalue[i] -= obj->m_values[i][obj->m_wpos[i]];
		obj->m_Intvalue[i] += tmp_ret[i];
		obj->m_values[i][obj->m_wpos[i]] = tmp_ret[i];
		ret_pos[i] = obj->m_Intvalue[i] / obj->m_dvsize[i];

		FX_DOUBLE cur_vel = (ret_pos[i] - obj->m_lastPos[i]) / obj->m_tds[i].m_ts;
		obj->m_lastAcc[i] = (cur_vel - obj->m_lastVel[i]) / obj->m_tds[i].m_ts;
		obj->m_lastVel[i] = cur_vel;
		obj->m_lastPos[i] = ret_pos[i];

		obj->m_wpos[i]++;
		if (obj->m_wpos[i] >= obj->m_vsize[i])
		{
			obj->m_wpos[i] = 0;
		}
		obj->m_vcnt[i]++;
		if (uptag == FX_FALSE)
		{
			if (obj->m_vcnt[i] > 1000)
			{
				obj->m_vcnt[i] = 0;
				uptag = FX_TRUE;
			}
			obj->m_Intvalue[i] = 0;
			for (j = 0; j < obj->m_vsize[i]; j++)
			{
				obj->m_Intvalue[i] += obj->m_values[i][j];
			}

		}
	}


	return FX_TRUE;
}


















