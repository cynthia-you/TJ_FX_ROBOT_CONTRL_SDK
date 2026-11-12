# DEMO_PYTHON — Python 示例索引

> 每个示例都可以单独运行。**先读示例文件顶部的注释**，里面写明了使用逻辑、前置条件和安全提示。
> 示例参数是研发测试值，不是生产参数。

## 运行前检查

1. Python ≥ 3.10，能导入 `SDK_PYTHON/fx_robot.py`、`SDK_PYTHON/fx_kine.py`。
2. `SDK_PYTHON/` 下的 `.dll` / `.so` 是用当前源码编译的最新版本。
3. 电脑与控制器同网段；示例默认 IP `192.168.1.190`，按现场修改。
4. `CommonConfig/` 下只有**一个**与机型匹配的 `*.MvKDCfg`。
5. MarvinPlatform / 其他 SDK 进程已关闭并释放连接。
6. 机器人周围安全，首次运行使用 10% 速度/加速度。

```bash
cd DEMO_PYTHON
python showcase_position.py
```

---

## 1. 控制基础

| 示例 | 内容 | 核心接口 |
|---|---|---|
| `showcase_check_sdk_type_compat.py` | 检查 Python 与动态库的类型/字节序兼容性 | `check_sdk_type_compat` |
| `showcase_position.py` | 双臂关节位置跟随（最基本闭环） | `connect` / `subscribe` / `set_state(1)` / `set_joint_cmd_pose` |
| `showcase_stop_run_AB.py` | 运动中停止（非急停、非下使能） | `stop_running` |
| `showcase_collaborative_release.py` | 协作释放（零力状态） | `set_state(4)` |
| `showcase_apply-brake_release-brake.py` | 强制抱闸 / 松闸 | 抱闸接口 |
| `showcase_servo_error.py` | 读取并解释伺服错误码 | `get_servo_error_code` |
| `showcase_servo_reset.py` | 指定关节伺服软复位 | `servo_reset` |
| `showcase_motor_encoder_clear.py` | 电机编码器清错 / 清零 | 编码器接口 |
| `showcase_get_set_param.py` | 读取/设置控制器参数（`robot.ini`） | `get_param` / `set_param` / `save_para_file` |
| `showcase_settime_reboot.py` | 设置系统时间 / 重启控制器 | `set_system_time` / `reboot` |
| `showcase_check_cmd_delay.py` | 测量指令下发延迟 | `send_cmd_wait_response` |

---

## 2. 阻抗、力控与拖动

| 示例 | 内容 | 核心接口 |
|---|---|---|
| `showcase_torque_joint_impedance_arm_A.py` | 关节阻抗控制 | `set_state(3)` + `set_impedance_type(1)` + `set_joint_kd_params` |
| `showcase_torque_cart_impedance_arm_A.py` | 笛卡尔阻抗控制 | `set_impedance_type(2)` + `set_joint_kd_params` |
| `showcase_cart_impedance_NSP_arm_A.py` | 笛卡尔阻抗 + 零空间约束 | 阻抗 + `ik_nsp` |
| `showcase_torque_EefCart_impedance_arm_A.py` | 工具末端笛卡尔阻抗 | `set_EefCart_control_params` |
| `showcase_torque_force_impedance_arm_A.py` | 恒力控制 | `set_force_control_params` / `set_force_cmd` |
| `showcase_eef_force_control_bsed_on_eef_cart_impedance.py` | 基于末端笛卡尔阻抗的力控 | 末端力控 |
| `showcase_force_field_control.py` | 力场控制（按方向力+位移） | `ft_arm_control` / `FTCmd` |
| `showcase_joint_drag_arm_A.py` | 关节拖动 | `set_drag_space(1)` |
| `showcase_cart_drag_arm_A.py` | 笛卡尔拖动（X/Y/Z/R 单方向） | `set_drag_space(2~5)` |
| `showcase_drag_JointImpedance_and_save_data_arm_A.py` | 关节拖动并采集数据 | 拖动 + `collect_data` |
| `showcase_drag_CartImpedance_and_save_data_arm_A.py` | 笛卡尔拖动并采集数据 | 拖动 + `collect_data` |

> 切换拖动方向前必须先退出拖动（`set_drag_space(..., 0)`）。

---

## 3. 轨迹与数据

| 示例 | 内容 | 核心接口 |
|---|---|---|
| `showcase_pvt_arm_A.py` | 上传 PVT 文件、运行指定 ID、采数据 | `send_pvt_file` / `set_pvt_id` / `collect_data` |
| `showcase_collect_data.py` | 1kHz 数据采集并保存 | `collect_data` / `stop_collect_data` / `save_collected_data_to_path` |
| `showcase_collect_data_as_csv.py` | 采集并保存 CSV | `save_collected_data_as_csv_to_path` |
| `showcase_process_collect_data_to_pvt_format.py` | 把拖动采集数据转成 PVT 文件 | PVT 文件格式 |
| `showcase_set_save_tool.py` | 设置/保存工具运动学与动力学 | `set_tool` / `get_tool_info` |
| `showcase_identy_tool_dynamic_CCS_B.py` | CCS 右臂工具动力学辨识 | `identify_tool_dyn` |
| `showcase_identy_tool_dynamic_SRS_B.py` | SRS 右臂工具动力学辨识 | `identify_tool_dyn` |
| `showcase_jointsTorque2EefTorque.py` | 关节力矩 → 末端六维力 | `jnt_tau_to_ee_tau` |
| `showcase_validate_get_dcss_data_from_user_inputCMD.py` | 用户自定义数据通道（力/重力/工具/刚度/IMU） | `set_user_specified_data` |
| `showcase_485_arm_A.py` | 末端 485 通讯 | `set_485_data` / `get_485_data` |
| `showcase_CAN_arm_A.py` | 末端 CAN/CANFD 通讯 | `set_485_data(..., com=1)` |

---

## 4. 规划（单臂）

| 示例 | 内容 | 核心接口 |
|---|---|---|
| `showcase_pln_joint_positionMode.py` | 关节空间规划下发（消抖） | `pln_init` / `setPln_joint` |
| `showcase_pln_joint_positionMode_with_break.py` | 关节规划 + 中断 | `stopRunPln_joint` |
| `showcase_pln_cart_positionMode.py` | 笛卡尔直线规划并执行 | `movLA` + `setPln_Cart` |
| `showcase_pln_cart_positionMode_with_break.py` | 笛卡尔规划 + 中断 | `stopPln_AB` / `stopRunPln_joint` |
| `showcase_pln_cart_positionmode_collabration.py` | 规划与协作释放配合 | 规划 + `set_state(4)` |
| `showcase_pln_cart_multi-segment_positionMode.py` | 多段直线规划 | `multi_movL_set_start` / `next_point` / `get_points` |
| `showcase_pln_mov_target_positionMode.py` | 直线优先、关节兜底 | `mov_target` |
| `showcase_online_pln_movl.py` | 在线直线规划 + 50Hz 执行 | `movLA` |
| `showcase_online_pln_movl_keepj.py` | 约束构型的在线直线规划 | `movL_KeepJA` |
| `showcase_online_pln_movl_with_specific_rot.py` | 固定姿态的在线直线规划 | `movLA` + `calculate_end_xyzabc` |

---

## 5. 规划（双臂）— 建议按顺序阅读

| 顺序 | 示例 | 内容 | 核心接口 |
|---|---|---|---|
| 1 | `showcase_pln_joint_to_joint_two_arms.py` | 双臂关节空间同时开始（不一定同时结束） | `setPln_joint_AB` |
| 2 | `showcase_pln_joint_to_joints_linear_two_arms.py` | 关节约束下的双臂直线 | `movL_KeepJA` + `setPln_Cart_AB` |
| 3 | `showcase_pln_cartesian_linear_two_arms.py` | 双臂笛卡尔直线 | `movLA` + `setPln_Cart_AB` |
| 4 | `showcase_pln_multi_segment_linear_two_arms.py` | 双臂多段直线 | `multi_movL_*` + `setPln_Cart_AB` |
| 5 | `showcase_pln_cartSpace_two_arms_synchronous.py` | 双臂同步直线（躯干固定） | `movL_synchronous` + `setPln_Cart_AB` |

> 双臂规划需要**两个** `Marvin_Kine` 实例，分别用 `arm_type=0` / `1` 初始化。

---

## 6. 运动学计算

| 示例 | 内容 |
|---|---|
| `showcase_kinematics_all_functions.py` | 计算 SDK 全功能串讲（推荐首先运行） |
| `showcase_kine_two_arms.py` | 双臂正解/逆解对比 |
| `showcase_ik_failed_conclusion.py` | 逆解失败原因分析 |
| `showcase_ik_nsp_two_arms.py` | 零空间逆解与参考构型 |

---

## 7. 干涉检测

| 示例 | 内容 |
|---|---|
| `showcase_drag_interference.py` | 拖动过程中做双臂碰撞检测 |

---

## 8. 简明式接口

| 示例 | 内容 |
|---|---|
| `showcases_new_control_sdk.py` | `Concise_Marvin_Robot` 全流程演示（连接→模式→力控→规划→PVT→拖动） |

---

## 9. 推荐学习顺序

```
showcase_check_sdk_type_compat.py
        ↓
showcase_position.py
        ↓
showcase_get_set_param.py / showcase_collect_data.py
        ↓
showcase_torque_joint_impedance_arm_A.py
        ↓
showcase_pln_joint_positionMode.py
        ↓
showcase_pln_cart_positionMode.py
        ↓
showcase_pln_cartesian_linear_two_arms.py
        ↓
showcase_pln_cartSpace_two_arms_synchronous.py
```

---

## 相关文档

- [SDK 总览](../README.md)
- [AI 开发指南](../AI_GUIDE.md)
- [Python 控制 SDK](../python_doc_contrl.md)
- [Python 计算 SDK](../python_doc_kine.md)
- [SDK_PYTHON 包说明](../SDK_PYTHON/readme.md)
