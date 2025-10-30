#ifdef __cplusplus
extern "C" {
#endif

	// c句柄
	typedef void* FXSpln;

	// C API
	FXSpln AxisPln_Create();
	void AxisPln_OnMovL(FXSpln spln, long RobotSerial, double start_pos[6], double end_pos[6], double ref_joints[7], double vel, double acc, double jerk, char* path);
	void AxisPln_OnMovL_KeepJ(FXSpln spln, long RobotSerial, double start_pos[6], double end_pos[6], double vel, char* path);
	void AxisPln_OnMovJ(FXSpln spln, long RobotSerial, double start_joint[7], double end_joint[7], double vel, double acc, double jerk, char* path);
	void AxisPln_Destroy(FXSpln spln);

#ifdef __cplusplus
}
#endif

