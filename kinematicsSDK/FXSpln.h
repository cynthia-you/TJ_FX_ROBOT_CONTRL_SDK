#ifdef __cplusplus
extern "C" {
#endif

	// ��͸��ָ�����ͣ��� C ��˵����һ�����
	typedef void* FXSpln;

	// C �ӿں���
	FXSpln AxisPln_Create();
	void AxisPln_OnMovL(FXSpln spln, double start_pos[6], double end_pos[6], double ref_joints[7], double vel, double acc, double jerk, char* path);
	void AxisPln_Destroy(FXSpln spln);

#ifdef __cplusplus
}
#endif

