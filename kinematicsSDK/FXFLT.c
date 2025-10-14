#include "FXFLT.h"


FX_BOOL FXAVGF_OnInit(FXAVGF* obj, FX_INT32L axis_num, FX_INT32L filter_size, FX_DOUBLE ts)
{
	FX_INT32L i;
	if (axis_num <= 0 || axis_num > 8 || filter_size <= 0 || filter_size > 100)
	{
		return FX_FALSE;
	}
	obj->m_axis_num = axis_num;
	obj->m_size = filter_size;
	obj->m_curpos = 0;
	obj->m_init_tag = FX_TRUE;
	obj->m_frist_tag = FX_TRUE;
	obj->m_ts = ts;

	for (i = 0; i < 8; i++)
	{
		obj->m_LastRet[i] = 0;
	}

	return FX_TRUE;
}
FX_BOOL FXAVGF_OnEmpty(FXAVGF* obj)
{
	FX_INT32L i;
	if (obj->m_init_tag == FX_FALSE)
	{
		return FX_FALSE;
	}
	for (i = 0; i < 8; i++)
	{
		obj->m_LastRet[i] = 0;
	}

	obj->m_frist_tag = FX_TRUE;
	return FX_TRUE;
}
FX_BOOL FXAVGF_OnFilt(FXAVGF* obj, FX_DOUBLE data[7], FX_DOUBLE ret_data[7], FX_DOUBLE ret_vel[7])
{
	FX_INT32L i, j;
	if (obj->m_init_tag == FX_FALSE)
	{
		return FX_FALSE;
	}
	if (obj->m_frist_tag == FX_TRUE)
	{
		for (i = 0; i < obj->m_axis_num; i++)
		{
			for (j = 0; j < obj->m_size; j++)
			{
				obj->m_datas[i][j] = data[i];
			}
			ret_data[i] = data[i];
			ret_vel[i] = 0;
			obj->m_LastRet[i] = ret_data[i];
		}
		obj->m_frist_tag = FX_FALSE;
		return FX_TRUE;
	}
	for (i = 0; i < obj->m_axis_num; i++)
	{
		FX_DOUBLE sum = 0;
		obj->m_datas[i][obj->m_curpos] = data[i];
		for (j = 0; j < obj->m_size; j++)
		{
			sum += obj->m_datas[i][j];
		}
		ret_data[i] = sum / obj->m_size;
		ret_vel[i] = (ret_data[i] - obj->m_LastRet[i]) / obj->m_ts;
		obj->m_LastRet[i] = ret_data[i];

	}
	obj->m_curpos++;
	if (obj->m_curpos >= obj->m_size)
	{
		obj->m_curpos = 0;
	}
	return FX_TRUE;

}



FX_BOOL FXSMV_OnInit(FX_SmoothValue* obj, FX_DOUBLE	current_value, FX_DOUBLE   new_value_ratio, FX_DOUBLE   max_step, FX_DOUBLE   min_step)
{
	obj->m_current_value = current_value;
	obj->m_max_step = 0.01;
	obj->m_min_step = 0.002;
	obj->m_new_value_ratio = 0.002;
	obj->m_is_one = FX_FALSE;
	obj->m_is_zero = FX_FALSE;



	if (new_value_ratio < 0.000000001 || new_value_ratio > 0.9999999999)
	{
		return FX_FALSE;
	}
	if (max_step < 0.00001 )
	{
		return FX_FALSE;
	}


	if (min_step < 0.00001)
	{
		return FX_FALSE;
	}

	if (min_step > max_step)
	{
		return FX_FALSE;
	}
	obj->m_min_step = min_step;
	obj->m_max_step = max_step;
	obj->m_new_value_ratio = new_value_ratio;
	obj->m_is_one = FX_FALSE;
	obj->m_is_zero = FX_FALSE;
	return FX_TRUE;
}

FX_VOID FXSMV_OnUpdateV(FX_SmoothValue* obj, FX_DOUBLE   target_value, FX_DOUBLE* current_value)
{
	FX_DOUBLE tmpv;
	FX_DOUBLE dff;
	FX_DOUBLE old_ratio = 1.0 - obj->m_new_value_ratio;
	tmpv = old_ratio * obj->m_current_value + obj->m_new_value_ratio * target_value;
	dff = tmpv - obj->m_current_value;
	if (dff > obj->m_max_step)
	{
		dff = obj->m_max_step;
	}
	if (dff < -obj->m_max_step)
	{
		dff = -obj->m_max_step;
	}
	if (dff > -obj->m_min_step && dff < obj->m_min_step)
	{
		tmpv = target_value - obj->m_current_value;
		if (tmpv > -obj->m_min_step && tmpv < obj->m_min_step)
		{
			obj->m_current_value = target_value;
		}
		else
		{
			if (tmpv > 0)
			{
				obj->m_current_value += obj->m_min_step;
			}
			else
			{
				obj->m_current_value -= obj->m_min_step;
			}

		}
	}
	else
	{
		obj->m_current_value += dff;
	}

	tmpv = obj->m_current_value;

	if (tmpv < 0.00000001 || tmpv > -0.00000001)
	{
		obj->m_is_zero = FX_TRUE;
	}
	else
	{
		obj->m_is_zero = FX_FALSE;
	}
	tmpv = obj->m_current_value - 1;
	if (tmpv < 0.00000001 || tmpv > -0.00000001)
	{
		obj->m_is_one = FX_TRUE;
	}
	else
	{
		obj->m_is_one = FX_FALSE;
	}
	*current_value = obj->m_current_value;
}

FX_VOID FXSMV_OnUpdateVZO(FX_SmoothValue* obj, FX_DOUBLE   target_value, FX_DOUBLE* current_value, FX_BOOL* is_zero, FX_BOOL* is_one)
{
	FX_DOUBLE tmpv;
	FX_DOUBLE dff;
	FX_DOUBLE old_ratio = 1.0 - obj->m_new_value_ratio;
	tmpv = old_ratio * obj->m_current_value + obj->m_new_value_ratio * target_value;
	dff = tmpv - obj->m_current_value;
	if (dff > obj->m_max_step)
	{
		dff = obj->m_max_step;
	}
	if (dff < -obj->m_max_step)
	{
		dff = -obj->m_max_step;
	}
	if (dff > -obj->m_min_step && dff < obj->m_min_step)
	{
		tmpv = target_value - obj->m_current_value;
		if (tmpv > -obj->m_min_step && tmpv < obj->m_min_step)
		{
			obj->m_current_value = target_value;
		}
		else
		{
			if (tmpv > 0)
			{
				obj->m_current_value += obj->m_min_step;
			}
			else
			{
				obj->m_current_value -= obj->m_min_step;
			}

		}
	}
	else
	{
		obj->m_current_value += dff;
	}

	tmpv = obj->m_current_value;

	if (tmpv < 0.00000001 || tmpv > -0.00000001)
	{
		obj->m_is_zero = FX_TRUE;
	}
	else
	{
		obj->m_is_zero = FX_FALSE;
	}
	tmpv = obj->m_current_value - 1;
	if (tmpv < 0.00000001 || tmpv > -0.00000001)
	{
		obj->m_is_one = FX_TRUE;
	}
	else
	{
		obj->m_is_one = FX_FALSE;
	}
	*current_value = obj->m_current_value;
	*is_zero = obj->m_is_zero;
	*is_one  = obj->m_is_one;
}

FX_VOID FXSMV_OnUpdateZO(FX_SmoothValue* obj, FX_DOUBLE   target_value, FX_BOOL* is_zero, FX_BOOL* is_one)
{
	FX_DOUBLE tmpv;
	FX_DOUBLE dff;
	FX_DOUBLE old_ratio = 1.0 - obj->m_new_value_ratio;
	tmpv = old_ratio * obj->m_current_value + obj->m_new_value_ratio * target_value;
	dff = tmpv - obj->m_current_value;
	if (dff > obj->m_max_step)
	{
		dff = obj->m_max_step;
	}
	if (dff < -obj->m_max_step)
	{
		dff = -obj->m_max_step;
	}
	if (dff > -obj->m_min_step && dff < obj->m_min_step)
	{
		tmpv = target_value - obj->m_current_value;
		if (tmpv > -obj->m_min_step && tmpv < obj->m_min_step)
		{
			obj->m_current_value = target_value;
		}
		else
		{
			if (tmpv > 0)
			{
				obj->m_current_value += obj->m_min_step;
			}
			else
			{
				obj->m_current_value -= obj->m_min_step;
			}

		}
	}
	else
	{
		obj->m_current_value += dff;
	}

	tmpv = obj->m_current_value;

	if (tmpv < 0.00000001 || tmpv > -0.00000001)
	{
		obj->m_is_zero = FX_TRUE;
	}
	else
	{
		obj->m_is_zero = FX_FALSE;
	}
	tmpv = obj->m_current_value - 1;
	if (tmpv < 0.00000001 || tmpv > -0.00000001)
	{
		obj->m_is_one = FX_TRUE;
	}
	else
	{
		obj->m_is_one = FX_FALSE;
	}
	*is_zero = obj->m_is_zero;
	*is_one = obj->m_is_one;
}

