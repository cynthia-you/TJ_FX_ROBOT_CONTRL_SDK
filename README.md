# 天机-孚晞 MARVIN 机器人 SDK

> 面向 MARVIN 人形双臂 / 单臂机器人的上位机开发包：**控制** + **运动学规划** + **干涉检测**。
> 支持 C++ / Python，Windows / Linux（x86_64 已验证）。

---

## 0. 30 秒了解

这套仓库里其实有 **三条独立的库**，可以单独用，也可以组合用：

| 库 | 作用 | Python | C++ | 典型用途 |
|---|---|---|---|---|
| **控制 SDK** | 连接机器人、切模式、下发指令、订阅状态、采集数据 | `SDK_PYTHON/fx_robot.py` | `contrlSDK100343/MarvinSDK.h` | 让机器人动起来 |
| **计算 SDK** | 正解 / 逆解 / 雅可比 / 直线与多段规划 / 双臂协同规划 | `SDK_PYTHON/fx_kine.py` | `kinematicsSDK/FxRobot.h` | 算点位、走直线 |
| **干涉检测**（可选） | 基于当前关节角做碰撞检测 | `SDK_PYTHON/fx_interference.py` | `interferenceCheck/Interf/Interference.h` | 拖动/规划前安全检查 |

最重要的两条关系：

1. **控制 SDK 负责“执行”，计算 SDK 负责“计算”。**
   计算 SDK 算出的 `CPointSet`（规划点位）交给控制 SDK 的 `setPln_Cart` / `setPln_Cart_AB` 执行。
2. **计算 SDK 不连接机器人**，只依赖机型配置文件 `*.MvKDCfg`；左右臂必须分别初始化（A=0，B=1）。

第一次接触建议按这个顺序：

```
MarvinPlatform APP 手动操作一遍
        ↓
跑通控制 demo（连接 → 位置模式 → 关节运动 → 释放）
        ↓
跑通计算 demo（加载配置 → 正解 → 逆解 → 直线规划）
        ↓
组合：计算规划 + 控制执行
```

---

## 1. 按目标找入口

| 我想…… | 首选接口 | 先看示例 | 详细文档 |
|---|---|---|---|
| 连接机器人、读状态 | `Marvin_Robot.connect` / `subscribe` | `DEMO_PYTHON/showcase_position.py` | [Python 控制](python_doc_contrl.md) |
| 关节位置控制 | `set_state(...,1)` + `set_joint_cmd_pose` | `showcase_position.py` | Python 控制 §12/§14/§17 |
| 关节/笛卡尔阻抗 | `set_state(...,3)` + `set_impedance_type` + `set_joint_kd_params` | `showcase_torque_joint_impedance_arm_A.py`、`showcase_torque_cart_impedance_arm_A.py` | Python 控制 §13/§15 |
| 恒力控制 | 力控模式 + `set_force_cmd` | `showcase_torque_force_impedance_arm_A.py` | Python 控制 §16 |
| 拖动示教 | `set_drag_space` | `showcase_joint_drag_arm_A.py`、`showcase_cart_drag_arm_A.py` | Python 控制 §18 |
| 复现示教轨迹 | PVT | `showcase_pvt_arm_A.py` | Python 控制 §10 |
| 走直线 / 多点轨迹 | `movLA` / `multi_movL_*` + `setPln_Cart` | `showcase_online_pln_movl.py`、`showcase_pln_cart_multi-segment_positionMode.py` | [Python 计算](python_doc_kine.md) |
| 双臂同步运动 | `setPln_joint_AB` / `setPln_Cart_AB` | `DEMO_PYTHON/readme.md` 第 5 节 | 控制 + 计算文档 |
| 双臂同步直线（躯干固定） | `movL_synchronous` + `setPln_Cart_AB` | `showcase_pln_cartSpace_two_arms_synchronous.py` | [Python 计算 §22](python_doc_kine.md) |
| 正解 / 逆解 / 零空间 | `fk` / `ik` / `ik_nsp` | `showcase_kinematics_all_functions.py` | [Python 计算](python_doc_kine.md) |
| 工具动力学辨识 | `identify_tool_dyn` | `showcase_identy_tool_dynamic_CCS_B.py` | Python 计算 §14 |
| 采集 1kHz 数据 | `collect_data` / 简明式 `start_collect_data` | `showcase_collect_data.py` | Python 控制 §8 |
| 末端 485 / CAN 通讯 | `set_485_data` / `get_485_data` | `showcase_485_arm_A.py`、`showcase_CAN_arm_A.py` | Python 控制 §9 |
| 碰撞检测 | `Marvin_Interference` | `showcase_drag_interference.py` | [SDK_PYTHON/readme](SDK_PYTHON/readme.md) |
| 用现成上位机调试 | MarvinPlatform | `MarvinPlatform_EN/README.md` | [MarvinPlatform 文档](MarvinPlatform_EN/README.md) |

> 完整 demo 索引见 [DEMO_PYTHON/readme.md](DEMO_PYTHON/readme.md) 和 [DEMO_C++/readme.md](DEMO_C++/readme.md)。

---

## 2. 5 分钟快速开始（Python）

### 2.1 前置条件

- Windows 10/11 或 Ubuntu 18.04–24.04（x86_64），Python ≥ 3.10。
- 电脑与控制器在同一网段，能 `ping` 通（默认示例 IP：`192.168.1.190`）。
- `CommonConfig/` 下放**唯一一个**与机型匹配的 `*.MvKDCfg`（CCS 6kg 注意 3.1/4.0 版本）。
- 关闭 MarvinPlatform / 其他占用 SDK 的程序（同一时间只能有一个连接）。
- `SDK_PYTHON/` 下的 `.dll` / `.so` 必须是当前源码编译的最新版本。

### 2.2 最小闭环：连接 → 读状态 → 关节运动 → 释放

```python
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

robot = Marvin_Robot()
dcss = DCSS()

# 1) 连接
if not robot.connect('192.168.1.190'):
    raise SystemExit('connect failed: port may be occupied')

# 2) 确认 UDP 数据通道真的在刷新（连接成功 != 能收到数据）
last = None
ok = False
for _ in range(20):
    data = robot.subscribe(dcss)
    serial = data['outputs'][0]['frame_serial']
    ok = ok or (serial != 0 and serial != last)
    last = serial
    time.sleep(0.01)
if not ok:
    robot.release_robot()
    raise SystemExit('no feedback data: check firewall / network / APP occupation')

# 3) 有错误先清错
robot.check_error_and_clear(dcss)

# 4) 设置速度/加速度并切位置模式（原接口：clear_set → 设置 → send_cmd）
robot.clear_set()
robot.set_vel_acc('A', 10, 10)      # 安全起见先 10%，验证后再提高
robot.set_vel_acc('B', 10, 10)
robot.send_cmd()
time.sleep(0.2)

robot.clear_set()
robot.set_state('A', 1)             # 1 = 位置跟随
robot.set_state('B', 1)
robot.send_cmd()
time.sleep(1.0)                     # 留出状态切换时间

# 5) 下发目标关节（单位：度）
joints_a = [0, 0, 0, 0, 0, 0, 0]
robot.clear_set()
robot.set_joint_cmd_pose('A', joints_a)
robot.send_cmd()

# 6) 等待到位：订阅反馈或 low_speed_flag
for _ in range(600):
    data = robot.subscribe(dcss)
    fb = data['outputs'][0]['fb_joint_pos']
    if all(abs(a - b) < 0.1 for a, b in zip(fb, joints_a)):
        break
    time.sleep(0.02)
else:
    print('warning: not reached target in time')

# 7) 结束务必释放，否则别的程序连不上
robot.release_robot()
```

### 2.3 C++ 最小闭环（简明式接口）

```cpp
#include "MarvinSDK.h"

#ifdef _WIN32
#include <windows.h>
#define SLEEP(ms) Sleep(ms)
#else
#include <unistd.h>
#define SLEEP(ms) usleep((ms) * 1000)
#endif

int main() {
    if (!Connect(192, 168, 1, 190, /*log_switch=*/0)) return -1;

    DCSS dcss{};
    CheckArmError();
    CheckServoError();

    SetJointMode('A', 10, 10);                 // 位置模式 + 10% 速度/加速度
    SLEEP(1000);                               // 等状态切换完成
    double q[7] = {0, 0, 0, 0, 0, 0, 0};
    SetJointPostionCmd('A', q);                // 注意函数名拼写为 Postion

    // 等待到位...
    for (int i = 0; i < 600; ++i) {
        OnGetBuf(&dcss);
        // 判断 dcss.m_Out[0].m_FB_Joint_Pos ...
        SLEEP(20);
    }

    Disable('A');
    OnRelease();                                // 必须释放
    return 0;
}
```

编译（Windows MinGW）：

```bash
g++ main.cpp -o main.exe -L./DEMO_C++ -lKine -lMarvinSDK
```

更完整的编译命令见 [DEMO_C++/readme.md](DEMO_C++/readme.md) 或直接运行 `marvinSDK_windows_100343.bat`。

---

## 3. 必读的 10 个核心概念

1. **控制库和计算库是两套库。** 控制库通过 UDP 连接控制器；计算库只做数学计算，不连接机器人的控制器。
2. **左右臂固定编号：`A` = 左臂 = 索引 `0`；`B` = 右臂 = 索引 `1`。**
   - Python 控制接口用字符 `'A'` / `'B'`；
   - `subscribe()` 返回的 `outputs[0]` / `outputs[1]` 分别对应 A / B；
   - 计算接口用 `arm_type=0` / `1`，规划接口用 `RobotSerial=0` / `1`。
3. **原控制接口是“攒批”模式：** `clear_set()` → 连续调用 `set_*` → `send_cmd()` 一次发出。
   简明式 `Concise_Marvin_Robot` 把这一步封装掉了，直接调用即可。
4. **1kHz UDP 通信。** 指令和反馈都是 1kHz；`frame_serial` 持续变化才说明数据通道正常。
5. **状态机：** `0` 下伺服 / `1` 位置 / `2` PVT / `3` 扭矩 / `4` 协作释放；`100` 表示有错误需清错，`101~103` 是切换瞬间。
6. **单位：** 关节角为**度**；笛卡尔 `XYZ` 为 **mm**，`ABC` 为**度**；力为 **N**；速度/加速度百分比为 **0~100**。
7. **机型配置必须匹配。** 配置文件导错时可能“看起来正常但结果错误”，尤其是 CCS 6kg 的 3.1 / 4.0 两个版本。
8. **三种运动方式，别混用：**
   - 直接关节指令：`set_joint_cmd_pose`（简单、实时）；
   - 在线规划后执行：`movLA` / `movL_KeepJA` / `multi_movL_*` → `setPln_Cart`（走直线、消抖）；
   - 离线 PVT：先上传轨迹文件，再按 ID 运行（复现示教轨迹）。
9. **`CPointSet` 有生命周期。** `movLA` 等返回的 `pset` 用完后必须 `destroy_point_set(pset)`；`movL` 是写文件版本，不返回点集。
10. **必须 `release_robot()` / `OnRelease()`。** 未释放会导致其他进程无法连接；APP 与 SDK 也不能同时占用。

---

## 4. 控制模式速查

| 模式 | `set_state` | 关键参数 | 关键接口 |
|---|---|---|---|
| 下伺服 / 复位 | `0` | — | `set_state(arm, 0)` |
| 位置跟随 | `1` | 速度/加速度百分比 | `set_vel_acc` → `set_joint_cmd_pose` |
| PVT 轨迹复现 | `2` | PVT 文件 + ID | `send_pvt_file` → `set_pvt_id` |
| 扭矩 / 关节阻抗 | `3` + `set_impedance_type(1)` | 7 轴 `K`（0~22 N·m/deg）、`D`（0~1，建议 0.3） | `set_joint_kd_params` |
| 扭矩 / 笛卡尔阻抗 | `3` + `set_impedance_type(2)` | 平移/旋转刚度、零空间刚度 | `set_joint_kd_params` 或 `set_cart_kd_params` |
| 扭矩 / 力控 | `3` + `set_impedance_type(3)` | 力方向、调节范围（mm）、目标力（N） | `set_force_control_params` → `set_force_cmd` |
| 协作释放（零力拖动） | `4` | — | `set_state(arm, 4)` |
| PD 前馈（遥操） | `3` + 关节阻抗 | `JointPIDCtlType=1`，速度/加速度 100 | `set_PD_vel_est_step` |

模式切换建议：**先设参数 → 再切状态 → 等 1 秒 → 再下发指令**。从 `0`（下伺服）切到其他模式尤其要留足时间。

---

## 5. 计算与规划速查

### 5.1 初始化顺序（每个手臂各一份）

```python
kk = Marvin_Kine()
cfg = kk.load_config(arm_type=0, config_path='CommonConfig/ccs_m6_40.MvKDCfg')
kk.initial_kine(cfg['TYPE'][0], cfg['DH'][0], cfg['PNVA'][0], cfg['BD'][0])
# 可选：带工具时设置 TCP
# kk.set_tool_kine(tool_4x4)
```

右臂用另一个 `Marvin_Kine()` 实例、`arm_type=1`，并用 `TYPE[1]/DH[1]/...` 初始化。

### 5.2 规划接口怎么选

| 需求 | 接口 | 执行方式 |
|---|---|---|
| 笛卡尔直线，生成文件 | `movL` | 文件交给控制器 / PVT 流程 |
| 笛卡尔直线，返回点集 | `movLA` | `setPln_Cart(arm, pset)` |
| 保持起止关节构型的直线 | `movL_KeepJA` | `setPln_Cart(arm, pset)` |
| 多段直线（可约束零空间） | `multi_movL_set_start` + `next_point` + `get_points` | `setPln_Cart(arm, pset)` |
| 关节空间点到点 | `setPln_joint`（控制库） | 控制器 50Hz 插补 |
| 直线优先、关节兜底 | `mov_target` | `setPln_Cart(arm, pset)` |
| 双臂同时开始（不保证同时结束） | `setPln_joint_AB` / `setPln_Cart_AB` | 控制库 |
| 双臂同步直线（躯干固定） | `movL_synchronous` | `setPln_Cart_AB(pset_A, pset_B)` |

### 5.3 逆解失败怎么判断

`ik()` 返回 `False` 时，依次看：

1. `m_Output_IsOutRange`：目标超出可达空间；
2. `m_Output_IsDeg`：关节奇异（尤其第 2/4 关节）；
3. `m_Output_IsJntExd` + `m_Output_JntExdTags`：哪个关节超限；
4. 参考关节 `m_Input_IK_RefJoint` 不能全 0（非零位 TCP）。

详细说明和零空间优化见 [python_doc_kine.md](python_doc_kine.md) 第（7）（8）节。

---

## 6. 仓库地图

```
TJ_FX_ROBOT_CONTRL_SDK/
├── README.md                  # 本文件：总览与导航
├── AI_GUIDE.md                # 给 AI / 代码生成工具的权威开发指南
├── AGENTS.md                  # 代码代理入口（指向 AI_GUIDE）
├── TROUBLESHOOTING.md         # 状态码、错误码、常见问题
├── c++_doc_contrl.md          # C++ 控制 SDK API
├── python_doc_contrl.md       # Python 控制 SDK API
├── c++_doc_kine.md            # C++ 计算 SDK API
├── python_doc_kine.md         # Python 计算 SDK API
├── CommonConfig/              # 机型配置文件 *.MvKDCfg
├── contrlSDK100343/           # 控制 SDK 源码（当前维护版本）
├── contrlSDK/                 # 控制 SDK 旧版本（不再维护）
├── kinematicsSDK/             # 计算 SDK 源码
├── interferenceCheck/         # 干涉检测源码
├── SDK_PYTHON/                # Python 包 + 预编译动态库
├── DEMO_C++/                  # C++ 示例
├── DEMO_PYTHON/               # Python 示例（推荐从这里开始）
├── MarvinPlatform_EN/         # 开源上位机 APP
└── robot.ini                  # 控制器参数文件（参数名查询用）
```

编译产物要放到**运行目录**：C++ demo 放 `DEMO_C++/`，Python 放 `SDK_PYTHON/`。

---

## 7. 文档怎么读

| 你是谁 / 想干什么 | 阅读顺序 |
|---|---|
| 第一次接触 | 本 README §2 → `DEMO_PYTHON/readme.md` 的控制部分 → 跑 `showcase_position.py` |
| 做控制业务 | [python_doc_contrl.md](python_doc_contrl.md) 或 [c++_doc_contrl.md](c++_doc_contrl.md) |
| 做轨迹/规划 | [python_doc_kine.md](python_doc_kine.md) 或 [c++_doc_kine.md](c++_doc_kine.md)，再对照 `DEMO_*/readme.md` 的规划示例 |
| 排查问题 | [TROUBLESHOOTING.md](TROUBLESHOOTING.md)：状态码、错误码、常见问题 |
| AI 代理 / 代码生成 | **[AI_GUIDE.md](AI_GUIDE.md)**（先读它，再按需查 API 文档） |
| 用上位机 | [MarvinPlatform_EN/README.md](MarvinPlatform_EN/README.md) |

---

## 8. 编译与库更新

```bash
# Linux：编译控制库 + 计算库 + 干涉检测，并复制到运行目录
./marvinSDK_ubuntu_100343.sh        # 老版本控制协议用 ./marvinSDK_ubuntu.sh

# Windows
marvinSDK_windows_100343.bat
```

手动编译（Linux）：

```bash
cd contrlSDK100343
g++ *.cpp -Wall -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN

cd ../kinematicsSDK
g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.so -lpthread -lrt
```

> **版本必须匹配：** 控制系统 100343 及以上只能配 `contrlSDK100343`；100343 之前的控制器用旧的 `contrlSDK`（已停止维护）。升级后请重新编译并替换 `SDK_PYTHON/`、`DEMO_C++/` 下的动态库。

更详细的平台差异、Python 专用 DLL 编译参数见 [SDK_PYTHON/readme.md](SDK_PYTHON/readme.md) 和 [DEMO_C++/readme.md](DEMO_C++/readme.md)。

---

## 9. 安全与现场注意事项

1. **首次运行把速度/加速度设为 10%**，验证通过后再逐步提高。
2. **不要在 APP 和 SDK 同时连接**；同一进程也要在结束时释放。
3. **状态切换留时间**：从下伺服到位置/扭矩模式建议等待 1 秒。
4. **扭矩模式（阻抗/力控）风险更高**，先用低刚度、小力、小范围验证；力控范围建议从 5~10 mm 开始。
5. **急停后会自动下伺服**，需要清错并重新上使能。
6. **末端夹爪/灵巧手不要直接套用 demo 指令**，必须按供应商协议确认后再发。
7. **规划文件换行符用 LF**（Windows 编辑器可能改成 CRLF，导致控制器加载 PVT 失败）。
8. **配置目录只放一个 `*.MvKDCfg`**，多个会导致解析错误。

---

## 10. 常见问题速查

| 现象 | 常见原因 | 处理 |
|---|---|---|
| 连接失败 / 端口占用 | 另一个程序（APP、旧进程）未释放 | 关闭占用进程/重启 APP，确认 `release_robot()` 被调用 |
| 连上了但订阅全是 0 | 未收到 UDP 数据 | 检查网段、防火墙、`frame_serial` 是否刷新 |
| 切模式失败/无响应 | 有错误未清、未设工具、伺服未就绪 | `check_error_and_clear` → 设工具 → 等 1 秒重试 |
| 逆解失败 | 超可达空间 / 奇异 / 关节超限 / 参考角全 0 | 看 `FX_InvKineSolvePara` 输出标志，换参考构型 |
| 轨迹不运行 | PVT 文件 CRLF / 未到起点 / ID 不匹配 | 转 LF、先运动到轨迹起点、核对 ID |
| 规划点执行后内存持续增长 | `CPointSet` 未销毁 | 执行结束后 `destroy_point_set` |
| 计算值看起来对但实际不对 | 机型配置版本不匹配 | 核对 CCS 3.1 / 4.0 / SRS 配置文件 |
| 手臂抖动 | 直接高频下发关节指令 | 改用 `setPln_joint` / `movLA` 规划方式下发 |

完整的错误码、状态码和排查步骤见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)，伺服故障对照 `JMDT驱动器报警故障和原因措施说明书`。

---

## 11. AI / 自动化开发

如果你的开发流程使用 AI 代理（Claude Code、Copilot、Cursor 等），请让它先读 **[AI_GUIDE.md](AI_GUIDE.md)**；
支持自动读取仓库说明的工具可以直接使用 **[AGENTS.md](AGENTS.md)**。AI_GUIDE 包含：

- 真实存在的 API 清单（避免模型编造接口）；
- 初始化 / 状态切换 / 规划执行的固定顺序；
- `subscribe()` 返回字典的完整字段；
- 安全约束、单位、常见反模式；
- 任务 → 接口 → 示例的映射与验证清单。

---

## 📄 许可证

本项目基于 Apache License 2.0 开源，详见 [LICENSE](LICENSE)。
