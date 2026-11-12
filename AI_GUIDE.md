# AI_GUIDE — MARVIN 机器人 SDK 开发指南（面向 AI 代理与代码生成工具）

> 本文件的目标：让 AI 在不读完所有文档的情况下，**准确**理解这套 SDK 的结构、接口、调用顺序和限制，
> 生成可直接运行的代码，而不是“看起来像那么回事”的伪代码。
>
> 人读也适用；人读版本请从 [README.md](README.md) 开始。

---

## 0. 使用规则（必须先读）

### 0.1 信息优先级

发生冲突时，按以下顺序取信：

1. **源码**：`SDK_PYTHON/fx_robot.py`、`SDK_PYTHON/fx_kine.py`、`SDK_PYTHON/fx_interference.py`
2. 本文件 `AI_GUIDE.md`
3. API 文档：`python_doc_contrl.md`、`python_doc_kine.md`、`c++_doc_contrl.md`、`c++_doc_kine.md`
4. `README.md`、`DEMO_*/readme.md`、示例代码

### 0.2 硬性规则

- **只使用本文件或源码中真实存在的接口名。** 不要根据命名习惯推测接口。
- **不要虚构返回值、参数个数、单位或取值范围。** 不确定时在代码里加校验，或明确标注 `TODO: verify`。
- **示例不是生产代码。** 所有 demo 的 `sleep`、速度百分比、刚度阻尼都是测试值。
- **涉及真实机器人运动的代码必须包含**：连接检查、清错、速度/加速度限制、到位判断、`release_robot()`。
- **不要同时生成“连接 APP”和“SDK 连接”的操作说明**，二者会争抢端口。
- **不要建议用户跳过 `release_robot()`。**
- 生成 C++ 代码时注意：**`_A` = 左臂，`_B` = 右臂**；Python 用字符 `'A'` / `'B'`。

### 0.3 版本与环境

| 项 | 值 |
|---|---|
| 控制协议 | 100343 及以上（100343 之前的控制器用旧 `contrlSDK`） |
| Python | ≥ 3.10 |
| 平台 | Windows 10/11，Ubuntu 18.04–24.04，x86_64 |
| 默认机器人 IP | `192.168.1.190`（示例值，按现场修改） |
| 通信 | UDP，1 kHz |
| 机型配置 | `CommonConfig/*.MvKDCfg`，目录内必须只有一个文件 |

---

## 1. 项目结构（生成代码前先定位文件）

```
SDK_PYTHON/
├── fx_robot.py        # 控制 SDK（两个类：Marvin_Robot / Concise_Marvin_Robot）
├── fx_kine.py         # 计算 SDK（Marvin_Kine）
├── fx_interference.py # 干涉检测（Marvin_Interference）
├── libMarvinSDK.dll/.so
├── libKine.dll/.so
└── libInterfCheck.dll/.so

DEMO_PYTHON/           # Python 示例（推荐参考）
DEMO_C++/              # C++ 示例
contrlSDK100343/       # 控制库源码（当前维护）
kinematicsSDK/         # 计算库源码
interferenceCheck/     # 干涉检测源码
CommonConfig/          # 机型配置
```

Python 导入方式（示例脚本都在仓库根目录的子目录中运行）：

```python
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from SDK_PYTHON.fx_robot import Marvin_Robot, Concise_Marvin_Robot, DCSS, FTCmd
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara, ArmsSynchronousPlanningParams
```

---

## 2. 术语、索引与单位

| 概念 | 规则 |
|---|---|
| 左臂 / 右臂 | Python：`'A'` / `'B'`；数据索引：`outputs[0]` / `outputs[1]`；计算：`arm_type` / `RobotSerial` = `0` / `1` |
| 关节 | 7 个，单位 **度** |
| 笛卡尔位姿 | `[X, Y, Z, A, B, C]`，位置 **mm**，姿态 **度** |
| 力 / 力矩 | N / N·m |
| 速度/加速度 | 控制模式参数为 **百分比 0~100**；规划接口为 mm/s、mm/s² |
| 规划频率 | 控制下发常用 50 Hz；基频 1000 Hz，`1000 % freq == 0` |
| 点集对象 | `CPointSet`，ctypes 的 `c_void_p`，用完必须销毁 |
| 坐标系 | 默认法兰/基座；设置工具后正逆解到 TCP；再设用户坐标系则到用户坐标系 |

---

## 3. 控制 SDK：两个类怎么选

`fx_robot.py` 中有两个类，**接口能力相同，用法不同**：

| 类 | 用法 | 适合 |
|---|---|---|
| `Marvin_Robot` | 原接口：参数设置必须包在 `clear_set()` 和 `send_cmd()` 之间 | 需要精细控制、批量下发 |
| `Concise_Marvin_Robot` | 简明式：一个调用完成“设参数 + 切模式”，内部检查错误 | 快速开发和业务脚本 |

两者的功能覆盖相近但**方法名不同**；下面分别给出清单，不要混用。

**不要混用两个类的实例去控制同一个机器人。**

### 3.1 标准生命周期

```
Marvin_Robot()
  → connect(ip)
  → subscribe() 确认 frame_serial 在刷新
  → check_error_and_clear(dcss)
  → [可选] set_tool()
  → 设置模式参数 → 切状态 → 等待 1 s
  → 下发指令 / 执行规划
  → 等待到位
  → 下使能（可选）
  → release_robot()   ← 必须
```

### 3.2 原接口的“攒批”规则

所有 `set_*`、`clear_error`、`set_state` 等**不会立即发送**，必须：

```python
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.set_state('A', 1)
robot.send_cmd()          # 到这里才真正发给控制器
```

需要确认控制器已响应时：

```python
robot.clear_set()
robot.set_state('A', 1)
robot.send_cmd_wait_response(100)   # 100 ms 超时
```

例外（直接生效，不需要 `clear_set/send_cmd`）：
`setPln_joint`、`setPln_Cart`、`setPln_joint_AB`、`setPln_Cart_AB`、`stopPln_AB`、`send_pvt_file`、`set_user_specified_data` 等内部自行发送的接口。以源码是否调用 `OnSetSend` 为准。

需要批处理但容易漏掉的接口：`set_drag_space`、`collect_data`、`stop_collect_data`、`set_pvt_id`。

---

## 4. 状态机与模式切换

### 4.1 状态值

| 值 | 含义 |
|---|---|
| `0` | 下伺服 / 复位 |
| `1` | 位置跟随 |
| `2` | PVT |
| `3` | 扭矩（阻抗 / 力控） |
| `4` | 协作释放（零力） |
| `100` | 错误，需要清错 |
| `101/102/103` | 正在切换到位置/PVT/扭矩（正常瞬态） |

读取位置：`sub_data['states'][0]['cur_state']`（A），`[1]`（B）。

### 4.2 切模式模板

```python
# 位置模式
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.send_cmd(); time.sleep(0.2)
robot.clear_set()
robot.set_state('A', 1)
robot.send_cmd(); time.sleep(1.0)

# 关节阻抗
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.set_joint_kd_params('A', K=[12,12,12,10,9,9,7], D=[0.3]*7)
robot.send_cmd(); time.sleep(0.2)
robot.clear_set()
robot.set_state('A', 3)
robot.set_impedance_type('A', 1)
robot.send_cmd(); time.sleep(1.0)

# 笛卡尔阻抗
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.set_cart_kd_params('A', K=[8000,8000,8000,300,300,300,20], D=[0.3]*7, type=2)
robot.send_cmd(); time.sleep(0.2)
robot.clear_set()
robot.set_state('A', 3)
robot.set_impedance_type('A', 2)
robot.send_cmd(); time.sleep(1.0)

# 力控
robot.clear_set()
robot.set_force_control_params('A', fcType=0, fxDirection=[0,0,1,0,0,0],
                               fcCtrlpara=[0]*7, fcAdjLmt=10.0)
robot.send_cmd(); time.sleep(0.2)
robot.clear_set()
robot.set_state('A', 3)
robot.set_impedance_type('A', 3)
robot.send_cmd(); time.sleep(1.0)
robot.clear_set()
robot.set_force_cmd('A', 5.0)
robot.send_cmd()
```

### 4.3 安全规则

- 从 `0` 切到任何模式：等待 ≥ 1 s。
- 扭矩模式必须保证工具动力学参数正确，否则可能切不进去（错误码 6/7）。
- 关节阻抗刚度范围约 `0~22 N·m/deg`；低刚度配低阻尼更稳。
- 笛卡尔阻抗平移刚度建议 ≤ 3000，旋转 ≤ 100（文档建议值），零空间 ≤ 20 或 20~100 视控制器版本。
- 力控范围 `fcAdjLmt` 单位 mm，先用 5~10 mm 验证。
- 协作释放后需要回 `0` 再重新上使能。

---

## 5. 实时订阅数据（唯一数据来源）

```python
dcss = DCSS()                       # 必须复用同一个实例
data = robot.subscribe(dcss)
```

### 5.1 顶层结构

```python
{
  'states':  [ {'cur_state': 0, 'cmd_state': 0, 'err_code': 0},
               {'cur_state': 0, 'cmd_state': 0, 'err_code': 0} ],
  'outputs': [ {...}, {...} ],   # A / B
  'inputs':  [ {...}, {...} ],   # A / B
  'ParaName': [...], 'ParaType': [...], 'ParaIns': [...],
  'ParaValueI': [...], 'ParaValueF': [...],
  'ParaCmdSerial': [...], 'ParaRetSerial': [...]
}
```

### 5.2 `outputs[i]` 字段（A=`0`，B=`1`）

| 字段 | 含义 | 单位 |
|---|---|---|
| `frame_serial` | 输出帧序号，用于判断数据是否刷新 | — |
| `traj_state` | 规划状态：`0x00` 无轨迹 / `0x01` 接收中 / `0x02` 已接收 / `>=0x03` 运行中 | — |
| `low_speed_flag` | 低速标志（各关节速度 < 0.5°/s 时为 1） | — |
| `tip_di` | 末端拖动按钮信号 | — |
| `fb_joint_pos` | 反馈关节位置 | deg |
| `fb_joint_vel` | 反馈关节速度 | deg/s |
| `fb_joint_posE` | 外编位置 | deg |
| `fb_joint_cmd` | 位置指令 | deg |
| `fb_joint_cToq` | 电流 | 千分比 |
| `fb_joint_sToq` | 传感器扭矩 | N·m |
| `fb_joint_them` | 关节温度 | 以控制器定义为准 |
| `est_joint_firc` | 关节摩擦力估计值；`[0]` 也被用作自定义数据类别编码 | — |
| `est_joint_firc_dot` | 关节力扰动估计值微分；自定义数据 `116~120/216~220` 的真实数据在这里 | — |
| `est_joint_force` | 关节外力估计值 | N·m |
| `est_cart_fn` | 末端六维力扰动估计；陀螺仪 `300~302` 数据在这里 | N / N·m |

### 5.3 到位判断的三种方式

1. `low_speed_flag == 1`（最省事）；
2. `fb_joint_pos` 与目标逐关节比较，差值 < 0.1°；
3. `traj_state == b'\x00'`（规划轨迹执行结束）。

注意：`traj_state`、`tip_di`、`low_speed_flag` 是 **bytes**（`b'\x00'`），比较时不要写成 `0`。

### 5.4 自定义数据通道

- `set_user_specified_data(arm, category)` 的 `arm` 只能是 `'A'` / `'B'` / `'AB'`。
- 同一通道重复下发相同类别时，`m_EST_Joint_Firc` 不刷新；需要验证“最新类别”时先切到别的类别再切回。
- 类别表见 `DEMO_PYTHON/showcase_validate_get_dcss_data_from_user_inputCMD.py` 顶部。

---

## 6. 计算 SDK（Marvin_Kine）

### 6.1 初始化（每个手臂独立一份）

```python
kk_a = Marvin_Kine()
cfg_a = kk_a.load_config(arm_type=0, config_path='CommonConfig/ccs_m6_40.MvKDCfg')
kk_a.initial_kine(cfg_a['TYPE'][0], cfg_a['DH'][0], cfg_a['PNVA'][0], cfg_a['BD'][0])

kk_b = Marvin_Kine()
cfg_b = kk_b.load_config(arm_type=1, config_path='CommonConfig/ccs_m6_40.MvKDCfg')
kk_b.initial_kine(cfg_b['TYPE'][1], cfg_b['DH'][1], cfg_b['PNVA'][1], cfg_b['BD'][1])
```

- `load_config` 返回的是**双臂**参数；A 臂用索引 `0`，B 臂用索引 `1`。
- **配置错误时不会报错，只会算出错误结果。** 必须核对机型版本。
- 工具：`set_tool_kine(4x4)`、`remove_tool_kine()`；用户坐标系：`set_user_frame_kine(4x4)`、`remove_user_frame_kine()`。
- 正解默认到法兰；设工具后到 TCP；再设用户坐标系则到用户坐标系。

### 6.2 正逆解

```python
mat = kk.fk(joints)                     # 4x4
xyzabc = kk.mat4x4_to_xyzabc(mat)       # [6]
mat4 = kk.xyzabc_to_mat4x4(xyzabc)      # 4x4

sp = FX_InvKineSolvePara()
sp.set_input_ik_target_tcp(mat)
sp.set_input_ik_ref_joint(ref_joints)   # 不能全 0
sp.set_input_ik_zsp_type(0)             # 0: 欧氏距离最近；1: 臂角平面最近
ok = kk.ik(sp)
q = sp.get_output_ret_joint()
```

失败诊断顺序：
`m_Output_IsOutRange` → `m_Output_IsDeg` → `m_Output_IsJntExd` / `m_Output_JntExdTags` → 参考关节是否合理。

### 6.3 规划接口

| 方法 | 返回 | 说明 |
|---|---|---|
| `movL(start,end,ref,vel,acc,freq,save_path)` | `bool` | 写文件；频率 500 Hz；速度/加速度 0.1~500 |
| `movLA(start,end,ref,vel,acc,freq,dimension=7)` | `(points, pset)` | 在线点集；失败 `([], None)` |
| `movL_KeepJ(start,end,vel,acc,freq,save_path)` | `bool` | 关节构型约束，写文件 |
| `movL_KeepJA(start,end,vel,acc,freq,dimension=7)` | `(points, pset)` | 关节构型约束，在线点集 |
| `multi_movL_set_start(...)` | `bool` | 多段起点 |
| `multi_movL_next_point(...)` | `bool` | 追加途经点 |
| `multi_movL_get_points(dimension=7)` | `(points, pset)` | 取回点集 |
| `mov_target(start,end,ref,vel,acc,freq,dimension=7)` | `(points, pset)` | 直线优先、关节兜底 |
| `movL_synchronous(aspp, dimension=7)` | `(data0, pset0, data1, pset1)` | 双臂同步直线（躯干固定） |
| `jnt_tau_to_ee_tau(q, tau)` | `list[6]` 或 `False` | 关节力矩 → 末端六维力 |
| `cal_ee_linear_vel(joints, angvel)` | `float`；失败返回 `-1` | 末端线速度 |

**点集生命周期：**

```python
points, pset = kk.movLA(...)
if pset is None:
    ...  # 规划失败
robot.setPln_Cart('A', pset)
# 等轨迹结束后
kk.destroy_point_set(pset)
```

`movL_synchronous` 的参数 `ArmsSynchronousPlanningParams`：

```python
aspp = ArmsSynchronousPlanningParams(
    Skye_Body_Start_PRR=[0, 0, 0],
    Arm0_Start_XYZABC=start_a, Arm0_End_XYZABC=end_a, Arm0_Ref_Joints=ref_a,
    Arm0_ZSP_Type=0, Arm0_ZSP_Para=[0]*6,
    Arm1_Start_XYZABC=start_b, Arm1_End_XYZABC=end_b, Arm1_Ref_Joints=ref_b,
    Arm1_ZSP_Type=0, Arm1_ZSP_Para=[0]*6,
    Freq=50, Vel=50.0, Acc=100.0, Sync_Type=0, World_Co_Flag=False)
data_a, pset_a, data_b, pset_b = kk_a.movL_synchronous(aspp)
```

前置条件：两个手臂都已完成 `initial_kine`；`World_Co_Flag=True` 时位姿基于机体坐标系并提供 PRR。

### 6.4 选择哪种运动方式

```
只需要到某个关节角
  → 位置模式 + set_joint_cmd_pose / Concise set_joint_position_cmd

末端需要走直线 / 抑制抖动
  → 计算 SDK 生成点集（movLA / movL_KeepJA / multi_movL / mov_target）
  → robot.setPln_Cart('A'/'B', pset) 或 setPln_Cart_AB(pset_a, pset_b)

已经有一条示教/离线的 PVT 轨迹
  → 切 PVT 状态 → send_pvt_file → set_pvt_id

双臂同时开始（不保证同时结束）
  → setPln_joint_AB / setPln_Cart_AB
```

---

## 7. 常用任务配方

### 7.1 连接 + 验证数据通道 + 清错

```python
robot = Marvin_Robot()
if not robot.connect('192.168.1.190'):
    raise SystemExit('connect failed')

dcss = DCSS()
last, ok = None, False
for _ in range(20):
    data = robot.subscribe(dcss)
    s = data['outputs'][0]['frame_serial']
    ok = ok or (s != 0 and s != last)
    last = s
    time.sleep(0.01)
if not ok:
    robot.release_robot()
    raise SystemExit('no feedback')

robot.check_error_and_clear(dcss)
```

### 7.2 位置模式移动到关节角

```python
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.send_cmd(); time.sleep(0.2)

robot.clear_set()
robot.set_state('A', 1)
robot.send_cmd(); time.sleep(1.0)

target = [0, 0, 0, 0, 0, 0, 0]
robot.clear_set()
robot.set_joint_cmd_pose('A', target)
robot.send_cmd()

for _ in range(600):
    d = robot.subscribe(dcss)['outputs'][0]
    if all(abs(a-b) < 0.1 for a, b in zip(d['fb_joint_pos'], target)):
        break
    time.sleep(0.02)
```

### 7.3 单臂直线规划并执行

```python
kk = Marvin_Kine()
cfg = kk.load_config(0, 'CommonConfig/ccs_m6_40.MvKDCfg')
kk.initial_kine(cfg['TYPE'][0], cfg['DH'][0], cfg['PNVA'][0], cfg['BD'][0])

points, pset = kk.movLA(start_xyzabc, end_xyzabc, ref_joints,
                        vel=50, acc=100, freq=50)
if pset is None:
    raise RuntimeError('planning failed')

try:
    robot.setPln_Cart('A', pset)
    for _ in range(1000):
        d = robot.subscribe(dcss)['outputs'][0]
        if d['traj_state'] == b'\x00':
            break
        time.sleep(0.2)
finally:
    kk.destroy_point_set(pset)
```

### 7.4 双臂同步直线

```python
data_a, pset_a, data_b, pset_b = kk_a.movL_synchronous(aspp)
if pset_a is None or pset_b is None:
    raise RuntimeError('dual-arm planning failed')

try:
    robot.setPln_Cart_AB(pset_a, pset_b)
    for _ in range(1000):
        d = robot.subscribe(dcss)
        if (d['outputs'][0]['traj_state'] == b'\x00' and
            d['outputs'][1]['traj_state'] == b'\x00'):
            break
        time.sleep(0.2)
finally:
    kk_a.destroy_point_set(pset_a)
    kk_b.destroy_point_set(pset_b)
```

### 7.5 数据采集

```python
# 采集 A/B 双臂关节位置，14 列，1000 行
ids = [0,1,2,3,4,5,6, 100,101,102,103,104,105,106] + [0]*21
robot.clear_set()
robot.collect_data(targetNum=14, targetID=ids, recordNum=1000)
robot.send_cmd()
# ... 运动/等待 ...
robot.clear_set()
robot.stop_collect_data()
robot.send_cmd()
robot.save_collected_data_as_csv_to_path('/tmp/data.csv')
```

### 7.6 拖动

```python
robot.clear_set()
robot.set_drag_space('A', 1)     # 1 关节拖动；2/3/4 = X/Y/Z；5 = 旋转
robot.send_cmd()

# 使用完毕必须退出
robot.clear_set()
robot.set_drag_space('A', 0)
robot.send_cmd()
```

切到另一种拖动前必须先 `0` 退出。

### 7.7 末端 485 / CAN

```python
robot.clear_485_cache('A')
robot.set_485_data('A', data=bytes([...]), size_int=len(data), com=1)  # 1:CAN 2:COM1 3:COM2
tag, recv = robot.get_485_data('A', com=1)
```

具体协议必须以末端模组厂商手册为准。

---

## 8. 参数范围与默认值

| 参数 | 范围 / 建议 |
|---|---|
| `velRatio` / `AccRatio` | 0~100；示例用 10，生产验证后再提高 |
| 关节阻抗 `K[7]` | 0~约 22 N·m/deg；建议低刚度起步 |
| 关节阻抗 `D[7]` | 0~1；建议 0.3 |
| 笛卡尔阻抗 `K[7]` | 文档建议平移 ≤1200~3000、旋转 ≤100~600、零空间 20~100（随控制器版本）；先用低值 |
| 笛卡尔阻抗 `D[7]` | 0~1 |
| 力控 `fcAdjLmt` | mm；建议先 5~10 |
| `movL/movLA` vel | 0.1~500 mm/s |
| `movL/movLA` acc | 0.1~500 mm/s² |
| 规划 `Freq` | 需满足 `1000 % Freq == 0`，常用 50 |
| `recordNum` | ≥1000，≤1,000,000 |
| PVT `serial/id` | 0~99 |

---

## 9. 常见反模式（生成代码时禁止）

| 反模式 | 后果 | 正确做法 |
|---|---|---|
| `connect()` 后不 `release_robot()` | 其他程序无法连接 | `try/finally` 中释放 |
| 原接口 `set_*` 后不 `send_cmd()` | 参数从未下发 | `clear_set → set_* → send_cmd` |
| 把 `traj_state` 和 `0` 比较 | 永远不成立 | 和 `b'\x00'` 比较 |
| 逆解 `ref_joints` 全 0 | 解算失败或构型跳变 | 用当前关节角做参考 |
| 用错配置文件版本 | 结果看似正常但错误 | 核对 CCS 3.1/4.0、SRS |
| `pset` 不销毁 | 内存持续增长 | `destroy_point_set` |
| 同一通道重复下发相同自定义类别后读编码 | 编码不刷新 | 先切其他类别再切回 |
| 笛卡尔拖动直接从一个方向切另一个 | 拖动混乱 | 先 `dgType=0` 退出 |
| PVT 文件用 CRLF | 控制器加载失败 | 转 LF |
| 用 `sleep` 代替到位判断 | 生产节拍不稳 | 订阅 `fb_joint_pos` / `low_speed_flag` |
| 未设工具就切扭矩模式 | 切换失败（错误码 6/7） | 先 `set_tool` |
| 同时运行 APP 和 SDK | 端口/数据冲突 | 只保留一个 |

---

## 10. 生成代码后的自检清单

- [ ] 导入路径正确（`SDK_PYTHON.fx_robot` / `SDK_PYTHON.fx_kine`）。
- [ ] 使用了真实存在的类和方法名（对照本文件第 11 节）。
- [ ] `DCSS()` 只创建一次并复用。
- [ ] 连接后有数据刷新检查。
- [ ] 有清错步骤。
- [ ] 速度/加速度有安全限制。
- [ ] 模式切换后有等待。
- [ ] 规划点集在 `finally` 中销毁。
- [ ] 所有退出路径都调用 `release_robot()`。
- [ ] 没有把示例参数当成生产参数。
- [ ] 涉及双臂时 A/B 索引没有写反。

---

## 11. API 清单（真实存在）

### 11.1 `Marvin_Robot`（原接口）

**生命周期 / 系统**
`connect`, `release_robot`, `subscribe`, `check_error_and_clear`, `SDK_version`, `check_sdk_type_compat`,
`update_SDK`, `download_sdk_log`, `get_robot_name`, `set_system_time`, `reboot`, `help`

**参数 / 日志**
`get_param`, `set_param`, `save_para_file`, `log_switch`, `local_log_switch`

**指令批处理**
`clear_set`, `send_cmd`, `send_cmd_wait_response`

**状态 / 错误**
`set_state`, `set_impedance_type`, `soft_stop`, `servo_reset`, `get_servo_error_code`, `clear_error`,
`reset_6dof_force_sensor`

**模式参数**
`set_vel_acc`, `set_tool`, `set_joint_kd_params`, `set_cart_kd_params`, `set_force_control_params`,
`set_EefCart_control_params`, `set_drag_space`, `set_PD_vel_est_step`

**指令**
`set_joint_cmd_pose`, `set_force_cmd`, `ft_arm_control`

**规划执行**
`pln_init`, `setPln_joint`, `setPln_Cart`, `setPln_joint_AB`, `setPln_Cart_AB`,
`stopRunPln_joint`, `stopPln_AB`

**PVT / 文件**
`send_pvt_file`, `set_pvt_id`, `send_file`, `receive_file`, `get_tool_info`

**数据 / 末端通讯**
`collect_data`, `stop_collect_data`, `save_collected_data_to_path`,
`save_collected_data_as_csv_to_path`, `set_485_data`, `get_485_data`, `clear_485_cache`,
`set_user_specified_data`

**其他**
`stop_running`

### 11.2 `Concise_Marvin_Robot`

`connect`, `subscribe`, `check_arms_error`, `check_servo_error`, `check_error_and_clear`,
`release_robot`, `SDK_version`, `update_SDK`, `download_sdk_log`, `get_param`, `set_param`,
`save_para_file`, `start_collect_data`, `stop_collect_data`, `save_gather_data`,
`save_gather_data_as_csv_to_path`, `soft_stop`, `servo_reset`, `get_servo_error_code`,
`set_position_state`, `set_imp_joint_state`, `set_imp_cart_state`, `set_imp_force_state`,
`set_force_cmd`, `ft_arm_control`, `set_user_specified_data`, `set_joint_position_cmd`,
`pln_init`, `run_pln_joint`, `run_pln_cart`, `stop_pln`, `setPln_joint_AB`,
`setPln_Cart_AB`, `stopPln_AB`, `send_pvt`, `run_pvt`, `set_joint_drag`, `set_cart_drag`,
`exit_drag`, `set_tool`, `disable`, `clear_ch_data`, `get_ch_data`, `set_ch_data`,
`get_tool_info`, `set_vel_est_step`, `get_robot_name`, `reset_6dof_force_sensor`,
`set_system_time`, `reboot`, `stop_running`, `help`

### 11.3 `Marvin_Kine`

`log_switch`, `load_config`, `initial_kine`, `set_tool_kine`, `remove_tool_kine`,
`set_user_frame_kine`, `remove_user_frame_kine`, `fk`, `fk_nsp`, `ik`, `ik_nsp`,
`joints2JacobMatrix`, `mat4x4_to_xyzabc`, `xyzabc_to_mat4x4`, `mat4x4_to_mat1x16`,
`calculate_end_xyzabc`, `movL`, `movLA`, `movL_KeepJ`, `movL_KeepJA`,
`mov_target`, `movL_synchronous`, `multi_movL_set_start`, `multi_movL_next_point`,
`multi_movL_get_points`, `identify_tool_dyn`, `identify_tool_dyn_m20`,
`jnt_tau_to_ee_tau`, `cal_ee_linear_vel`, `create_point_set`, `destroy_point_set`,
`get_point_set_data`, `help`

### 11.4 `Marvin_Interference`

`create`, `destroy`, `init`, `update_cord`, `update_convex`, `calc_interf_distance`,
`get_com`, `get_cords_num`, `get_mass`, `get_min_span`, `get_interf_span`, `on_is_interf`

### 11.5 C++ 简明式接口对应

| Python（Concise） | C++ |
|---|---|
| `connect` | `Connect(ip1,ip2,ip3,ip4,log_switch)` |
| `set_position_state` | `SetJointMode(arm, velRatio, AccRatio)` |
| `set_imp_joint_state` | `SetImpJointMode(arm, velRatio, AccRatio, K, D)` |
| `set_imp_cart_state` | `SetImpCartMode(arm, velRatio, AccRatio, K, D, RotType, CartCtrlPara)` |
| `set_imp_force_state` | `SetImpForceMode(arm, fxDir, fcAdjLmt)` |
| `set_joint_position_cmd` | `SetJointPostionCmd(arm, joint)` |
| `set_force_cmd` | `SetForceCmd(arm, force)` |
| `run_pln_joint` | `RunPlnJoint(arm, start, stop, vel_ratio, acc_ratio)` |
| `run_pln_cart` | `RunPlnCart(arm, pset)` |
| `stop_pln` | `StopPln(arm)` |
| `send_pvt` | `SendPVT(arm, file, serial)` |
| `run_pvt` | `RunPVT(arm, id)` |
| `set_joint_drag` / `set_cart_drag` / `exit_drag` | `SetJointDrag` / `SetCartDrag` / `ExitDrag` |
| `disable` | `Disable(arm)` |
| 释放 | `OnRelease()` |

C++ 原接口：`OnClearSet()` → `OnSetXxx_A/_B(...)` → `OnSetSend()`；订阅 `OnGetBuf(&dcss)`。

---

## 12. 文档与示例索引

| 内容 | 文件 |
|---|---|
| 总览 / 快速开始 | [README.md](README.md) |
| Python 控制 API | [python_doc_contrl.md](python_doc_contrl.md) |
| Python 计算 API | [python_doc_kine.md](python_doc_kine.md) |
| C++ 控制 API | [c++_doc_contrl.md](c++_doc_contrl.md) |
| C++ 计算 API | [c++_doc_kine.md](c++_doc_kine.md) |
| Python 示例索引 | [DEMO_PYTHON/readme.md](DEMO_PYTHON/readme.md) |
| C++ 示例索引 | [DEMO_C++/readme.md](DEMO_C++/readme.md) |
| Python 包说明 | [SDK_PYTHON/readme.md](SDK_PYTHON/readme.md) |
| 上位机 | [MarvinPlatform_EN/README.md](MarvinPlatform_EN/README.md) |
| 状态码 / 错误码 / 故障排查 | [TROUBLESHOOTING.md](TROUBLESHOOTING.md) |

**生成新示例前，先找最接近的现有示例并以其调用顺序为模板。** 不要从零“设计”调用流程。

---

## 13. 文档维护规则（给维护者 / AI）

接口或示例变更时，按以下顺序同步，避免文档漂移：

1. 先改源码（`SDK_PYTHON/*.py`、`kinematicsSDK/*.h`、`contrlSDK100343/*.h`）。
2. 更新本文件 `AI_GUIDE.md` 的 API 清单与约束。
3. 更新对应 API 文档（Python/C++ 各一份）。
4. 更新 `DEMO_*/readme.md` 的示例索引（新增/重命名/删除示例时）。
5. 如果新增了控制模式、规划方式或必读限制，更新根 `README.md` 的核心概念/速查表。
6. 重新编译动态库，并确认 `SDK_PYTHON/`、`DEMO_C++/` 中的库已替换。

**命名变更检查清单：**

- C++ 导出符号（`FxRobot.h` / `MarvinSDK.h`）
- Python 方法名（`fx_kine.py` / `fx_robot.py`）
- 示例文件名与示例顶部注释
- 文档中的接口名、示例索引、交叉链接

> 历史教训：双臂同步接口曾从 `FX_Robot_PLN_MOVL_DualArm_FixBody` 更名为
> `FX_Robot_PLN_MOVL_Arms_Synchronous`，Python 方法为 `movL_synchronous`。
> 旧名不应再出现在新代码或文档中。
