#ifdef __cplusplus
extern "C" {
#endif

	// 不透明指针类型，对 C 来说就是一个句柄
	typedef void* FXSpln;

	// C 接口函数
	FXSpln AxisPln_Create();
	void AxisPln_OnMovL(FXSpln spln, double start_pos[6], double end_pos[6], double ref_joints[7], double vel, double acc, double jerk, char* path);
	void AxisPln_Destroy(FXSpln spln);

#ifdef __cplusplus
}
#endif

