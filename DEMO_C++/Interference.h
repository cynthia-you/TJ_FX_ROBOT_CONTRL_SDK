#ifndef FX_INTERFERENCE_H
#define FX_INTERFERENCE_H
#include <stdint.h>


#if defined(_WIN32) || defined(_WIN64)
#define INTERF_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define INTERF_SDK_API
#endif

typedef enum
{
    FUNC_RET_SUCCESS = 0,
    FUNC_RET_INVALID_INPUT_ARG = -1,
    FUNC_RET_OPERATION_FAILED = -2,
    FUNC_RET_ISInterference = -3,
    FUNC_RET_NOInterference = -4,
} FX_InterfErrCode;

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 干涉检测句柄
     *
     * 该句柄持有 FXInterference 单例对象，
     * 包含坐标系组、凸几何体及干涉约束数据。
     */
    typedef struct FX_InterfContext *FX_InterfHandle;

    /**
     * @brief 创建干涉检测单例句柄
     *
     * 分配内存并实例化唯一的干涉检测单例对象。
     * 该句柄必须传入后续所有干涉模块 API 调用。
     * @see FX_InterfHandle
     */
    INTERF_SDK_API FX_InterfHandle FX_Interf_Create();

    /**
     * @brief 销毁干涉检测并释放资源
     *
     * 释放绑定到该句柄的所有内存、几何缓冲区及配置数据。
     * 调用此函数后句柄将失效,请勿再次使用。
     *
     * @param[in] handle 通过 FX_Interf_Create 创建的干涉句柄
     */
    INTERF_SDK_API void FX_Interf_Destroy(FX_InterfHandle handle);

    /**
     * @brief 使用多组配置定义初始化干涉模块
     *
     * 从输入的定义字符串或文件路径中解析并加载坐标系、连杆标定、输入映射、凸碰撞几何体
     * 以及干涉约束规则。
     * 在执行任何坐标更新或距离计算操作之前,必须先调用此 API。
     *
     * @param[in] handle 干涉句柄
     * @param[in] CordDef 坐标系定义字符串或文件路径
     * @param[in] CalLinkDef 连杆标定参数定义字符串或文件路径
     * @param[in] InputDef 外部输入变量映射定义字符串或文件路径
     * @param[in] ConvexDef 凸碰撞几何体定义字符串或文件路径
     * @param[in] ICDef 干涉检查约束规则定义字符串或文件路径
     *
     * @return 返回值定义见 FX_InterfErrCode
     */
    INTERF_SDK_API int FX_Interf_Init(FX_InterfHandle handle, char *CordDef, char *CalLinkDef, char *InputDef, char *ConvexDef, char *ICDef);

    /**
     * @brief 刷新指定坐标组的实时坐标数据
     *
     * 更新刚体位置与姿态向量,供后续干涉距离计算使用。
     * 需周期性调用此函数以同步机器人关节/工具的实时位姿。
     *
     * @param[in] handle 干涉句柄
     * @param[in] cord_num 目标坐标组索引标识数量
     * @param[in] input 原始坐标 double 数组,数组长度遵循已加载的配置定义
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_UpdateCord(FX_InterfHandle handle, long cord_num, double *input);

    /**
     * @brief 重建所有凸包碰撞几何模型
     *
     * 使用最新更新的坐标数据重构凸多面体碰撞体。
     * 必须在 FX_Interf_UpdateCord 之后调用,以同步碰撞网格几何体。
     *
     * @param[in] handle 干涉句柄
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_UpdateConvex(FX_InterfHandle handle);

    /**
     * @brief 计算所有刚体碰撞对的最小分离距离
     *
     * 遍历所有已定义的碰撞对,计算全局最小几何分离距离。
     * 依赖于已同步的坐标数据及已重建的凸包几何体。
     *
     * @param[in] handle 干涉句柄
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_CalcInterfDistance(FX_InterfHandle handle);

    /**
     * @brief 获取所有刚性连杆的加权平均质心 X/Y/Z
     *
     * 输出所有已加载刚体的质量加权平均质心坐标。
     * 输出数组依次存储 [X, Y, Z] 三轴空间值。
     *
     * @param[in] handle 干涉句柄
     * @param[out] mcp_average 用于输出质心 [X, Y, Z] 的 double 数组缓冲区,长度 = 3
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_GetCoM(FX_InterfHandle handle, double mcp_average[3]);

    /**
     * @brief 获取已加载坐标组的总数
     *
     * 读取在 FX_Interf_Init 配置加载期间解析到的坐标组总数量。
     *
     * @param[in] handle 干涉句柄
     * @param[out] cords_num 用于接收坐标组总数的 long 指针
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_GetCordsNum(FX_InterfHandle handle, long *cords_num);

    /**
     * @brief 获取所有已定义刚体的质量标量数组
     *
     * 输出质量值数组,数组长度与已加载坐标组总数一致。
     *
     * @param[in] handle 干涉句柄
     * @param[out] quality 用于接收刚体质量标量数组的 double 缓冲区
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_GetMass(FX_InterfHandle handle, double *quality);

    /**
     * @brief 获取所有碰撞对的全局最小几何分离距离
     *
     * 获取任意两个碰撞体之间的最小距离值,
     * 通常用于实时早期干涉预警逻辑。
     *
     * @param[in] handle 干涉句柄
     * @param[out] MinSpan 用于存储全局最小分离距离值的 double 指针
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_GetMinSpan(FX_InterfHandle handle, double *MinSpan);

    /**
     * @brief 按碰撞对序号索引查询干涉距离值
     *
     * 通过唯一序号标识读取指定碰撞对的分离距离。
     *
     * @param[in] handle 干涉句柄
     * @param[in] serial 目标碰撞对的唯一序号索引
     * @param[out] MinSpan 用于存储所查询碰撞对分离距离的 double 指针
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_GetInterfSpan(FX_InterfHandle handle, long serial, double *MinSpan);
    
    /**
     * @brief 查询碰撞对是否存在超过预定义的干涉阈值
     *
     *
     * @param[in] handle 干涉句柄
     * @param[in] inteThsh 所有碰撞对的干涉阈值
     *
     * @return 返回值定义见 FX_InterfErrCode。
     */
    INTERF_SDK_API int FX_Interf_OnIsInterf(FX_InterfHandle handle, double * inteThsh);
    
#ifdef __cplusplus
}
#endif

#endif