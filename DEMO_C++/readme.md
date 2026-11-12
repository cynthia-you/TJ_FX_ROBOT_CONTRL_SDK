# DEMO_C++ — C++ 示例索引

> 示例用于演示接口调用顺序，不是生产代码。运行前先读文件顶部的说明。
> 默认 IP `192.168.1.190`，示例里的速度/刚度/阻尼均为测试值。

## 一、准备

`DEMO_C++/` 下需要以下文件（必须与当前源码一起编译）：

```
libKine.dll / libKine.so
libMarvinSDK.dll / libMarvinSDK.so
libInterfCheck.dll / libInterfCheck.so

FXDG.h
FxRobot.h
MarvinSDK.h
PointSet.h
Interference.h
```

检查动态库是否最新：重新运行仓库根目录的自动化脚本。

```bash
# Windows
marvinSDK_windows_100343.bat

# Linux
./marvinSDK_ubuntu_100343.sh
```

## 二、编译示例

### Linux

```bash
# 只使用计算库
g++ showcase_kinematics_all_functions.cpp -o kine -L. -lKine -Wl,-rpath=.
./kine

# 控制库 + 计算库
g++ showcase_offline_movl_keepj_execution.cpp -o offline_movl_keepj \
    -L. -lKine -lMarvinSDK -Wl,-rpath=.
./offline_movl_keepj

# 再加干涉检测库
g++ showcase_drag_interference.cpp -o showcase_drag_interference \
    -L. -lKine -lMarvinSDK -lInterfCheck -Wl,-rpath=.
```

### Windows（MinGW）

```bash
g++ showcase_kinematics_all_functions.cpp -o kine.exe -L. -lKine
kine.exe

g++ showcase_offline_movl_keepj_execution.cpp -o offline_movl_keepj.exe -L. -lKine -lMarvinSDK
offline_movl_keepj.exe
```

### 不使用动态库（直接编译源码）

```bash
# Linux
g++ -Wall main.cpp ../contrlSDK100343/*.cpp -I../contrlSDK100343 -o main -lpthread -lrt -DCMPL_LIN

# Windows
g++ -Wall main.cpp ../contrlSDK100343/*.cpp -I../contrlSDK100343 -o main.exe -lws2_32 -lwinmm -DCMPL_WIN
```

---

## 三、控制示例

### 3.1 基础与安全

| 示例 | 内容 |
|---|---|
| `showcase_check_sdk_type_compat.cpp` | 检查 SDK 类型/字节序兼容性 |
| `showcase_link_check.cpp` | 连接与数据通道检查 |
| `showcase_get_set_param_demo.cpp` | 读取/设置控制器参数 |
| `showcase_cmd_delay.cpp` | 指令下发延迟测试 |
| `showcase_apply_brake_release_brake.cpp` | 强制抱闸 / 松闸 |
| `showcase_collaborative_release.cpp` | 协作释放（零力） |
| `showcase_servo_reset.cpp` | 指定关节伺服软复位 |
| `showcase_validate_stiffness_info.cpp` | 阻抗刚度/阻尼设置与回读 |

### 3.2 控制模式

| 示例 | 内容 |
|---|---|
| `showcase_position_two_arms.cpp` | 双臂位置跟随（最基本） |
| `showcase_joint_impedance_control.cpp` | 关节阻抗 |
| `showcase_cartesian_impedance_control.cpp` | 笛卡尔阻抗 |
| `showcase_eef_cart_impedance.cpp` | 末端工具笛卡尔阻抗 |
| `showcase_eef_force_control_bsed_on_eef_cart_impedance.cpp` | 基于末端阻抗的力控 |
| `showcase_Force_field_Control.cpp` | 力场控制 |
| `showcase_JointTorque2EETorque.cpp` | 关节力矩 → 末端六维力 |

### 3.3 拖动与示教

| 示例 | 内容 |
|---|---|
| `showcase_drag_joint.cpp` | 关节拖动 |
| `showcase_drag_JointImpedance_save_data.cpp` | 关节阻抗拖动并保存数据 |
| `showcase_drag_CartImpedance_save_data.cpp` | 笛卡尔阻抗拖动并保存数据 |

### 3.4 末端通讯

| 示例 | 内容 |
|---|---|
| `showcase_eef_485.cpp` | 末端 485 |
| `showcase_eef_CAN.cpp` | 末端 CAN/CANFD |

### 3.5 PVT、数据与系统

| 示例 | 内容 |
|---|---|
| `showcase_pvt.cpp` | PVT 轨迹运行 |
| `showcase_validate_get_dcss_data_from_user_inputCMD.cpp` | 用户自定义数据通道 |
| `showcase_ota_transfer.cpp` | OTA 传输 |
| `showcase_update_system.cpp` | 系统升级 |

### 3.6 简明式接口

| 示例 | 内容 |
|---|---|
| `showcase_new_control_sdk_usage.cpp` | `Concise` 风格 API 全流程 |

---

## 四、规划示例

### 4.1 单臂

| 示例 | 内容 |
|---|---|
| `showcase_pln_jointCmd_PositionMode.cpp` | 关节指令 + 位置模式 |
| `showcase_pln_jointSpace_PositionMode.cpp` | 关节空间规划（消抖） |
| `showcase_pln_jointSpace_PositionMode_with_break.cpp` | 关节规划 + 中断 |
| `showcase_pln_cartSpace_PositionMode.cpp` | 笛卡尔直线规划 |
| `showcase_pln_cartSpace_PositionMode_with_break.cpp` | 笛卡尔规划 + 中断 |
| `showcase_pln_cartSpace_MultiPoints_PositionMode.cpp` | 多段直线 |
| `showcase_pln_PositionMode_MovTarget.cpp` | 直线优先、关节兜底 |
| `showcase_online_movj_execution.cpp` | 在线关节空间规划执行 |
| `showcase_online_movla_execution.cpp` | 在线直线规划执行 |
| `showcase_online_movl_keepja_execution.cpp` | 约束构型的在线直线执行 |
| `showcase_offline_movl_execution.cpp` | 离线直线规划文件执行 |
| `showcase_offline_movl_keepj_execution.cpp` | 约束构型的离线直线执行 |
| `showcase_online_and_offline_pln_all_function.cpp` | 在线/离线规划全功能 |

### 4.2 双臂

| 示例 | 内容 |
|---|---|
| `showcase_pln_jointSpace_PositionMode_two_arms.cpp` | 双臂关节空间同时开始（不一定同时结束） |
| `showcase_pln_cartSpace_two_arms_synchronous.cpp` | 双臂同步直线（躯干固定） |

> 双臂同步规划使用 `FX_Robot_PLN_MOVL_Arms_Synchronous`，规划完成后由控制库 `CoRunPlnCart` 同时下发。

---

## 五、计算示例

| 示例 | 内容 |
|---|---|
| `showcase_kinematics_all_functions.cpp` | 计算 SDK 全功能（推荐首先运行） |
| `showcase_kine_two_arms.cpp` | 双臂正解/逆解 |
| `showcase_ik_failed_conclusion.cpp` | 逆解失败原因 |
| `showcase_ik_nsp_two_arms.cpp` | 零空间逆解 |

---

## 六、干涉检测示例

| 示例 | 内容 |
|---|---|
| `showcase_drag_interference.cpp` | 拖动时做双臂碰撞检测（需链接 `libInterfCheck`） |

---

## 七、推荐运行顺序

```
showcase_check_sdk_type_compat
        ↓
showcase_link_check
        ↓
showcase_position_two_arms
        ↓
showcase_get_set_param_demo / showcase_cmd_delay
        ↓
showcase_joint_impedance_control
        ↓
showcase_kinematics_all_functions
        ↓
showcase_pln_cartSpace_PositionMode
        ↓
showcase_pln_cartSpace_two_arms_synchronous
```

---

## 相关文档

- [SDK 总览](../README.md)
- [AI 开发指南](../AI_GUIDE.md)
- [C++ 控制 SDK](../c++_doc_contrl.md)
- [C++ 计算 SDK](../c++_doc_kine.md)
