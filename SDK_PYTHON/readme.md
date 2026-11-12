# SDK_PYTHON — Python SDK 包

> 这是 MARVIN 机器人的 Python 包：控制 + 计算 + 干涉检测。
> 不需要 `pip install`，把仓库根目录加入 `sys.path` 即可导入。

## 一、包内文件

| 文件 | 作用 |
|---|---|
| `fx_robot.py` | 控制 SDK：`Marvin_Robot`（原接口）、`Concise_Marvin_Robot`（简明式）、`DCSS`、`FTCmd` |
| `fx_kine.py` | 计算 SDK：`Marvin_Kine`、`FX_InvKineSolvePara`、`ArmsSynchronousPlanningParams` |
| `fx_interference.py` | 干涉检测：`Marvin_Interference` |
| `libMarvinSDK.dll/.so` | 控制动态库（由 `contrlSDK100343` 编译） |
| `libKine.dll/.so` | 计算动态库（由 `kinematicsSDK` 编译） |
| `libInterfCheck.dll/.so` | 干涉检测动态库（由 `interferenceCheck` 编译） |

**动态库必须与当前源码版本一致。** 更新代码后重新编译并覆盖这里的库文件。

## 二、导入方式

示例脚本位于 `DEMO_PYTHON/`，通常这样导入：

```python
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SDK_PYTHON.fx_robot import Marvin_Robot, Concise_Marvin_Robot, DCSS, FTCmd
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara, ArmsSynchronousPlanningParams
from SDK_PYTHON.fx_interference import Marvin_Interference
```

## 三、最小示例

```python
import time
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

robot = Marvin_Robot()
dcss = DCSS()

if not robot.connect('192.168.1.190'):
    raise SystemExit('connect failed')

data = robot.subscribe(dcss)          # 持续调用以获取最新状态
print(data['outputs'][0]['fb_joint_pos'])

robot.release_robot()                 # 必须释放
```

## 四、控制模块：两个类

### 4.1 `Marvin_Robot`（原接口）

参数设置遵循“攒批”：

```python
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.set_state('A', 1)
robot.send_cmd()
```

主要能力分组：

| 分组 | 方法 |
|---|---|
| 连接/数据 | `connect`, `release_robot`, `subscribe`, `check_error_and_clear`, `SDK_version` |
| 参数 | `get_param`, `set_param`, `save_para_file`, `log_switch`, `local_log_switch` |
| 状态/错误 | `set_state`, `set_impedance_type`, `soft_stop`, `servo_reset`, `get_servo_error_code`, `clear_error` |
| 模式参数 | `set_vel_acc`, `set_tool`, `set_joint_kd_params`, `set_cart_kd_params`, `set_force_control_params`, `set_EefCart_control_params`, `set_drag_space`, `set_PD_vel_est_step` |
| 指令 | `set_joint_cmd_pose`, `set_force_cmd`, `ft_arm_control` |
| 规划执行 | `pln_init`, `setPln_joint`, `setPln_Cart`, `setPln_joint_AB`, `setPln_Cart_AB`, `stopPln_AB`, `stopRunPln_joint` |
| PVT | `send_pvt_file`, `set_pvt_id` |
| 采集 | `collect_data`, `stop_collect_data`, `save_collected_data_to_path`, `save_collected_data_as_csv_to_path` |
| 末端通讯 | `set_485_data`, `get_485_data`, `clear_485_cache`, `set_user_specified_data` |
| 文件/系统 | `send_file`, `receive_file`, `update_SDK`, `download_sdk_log`, `set_system_time`, `reboot`, `stop_running` |

### 4.2 `Concise_Marvin_Robot`（简明式）

一个方法完成“设参数 + 切模式”，内部处理错误检查，适合业务脚本：

`set_position_state`, `set_imp_joint_state`, `set_imp_cart_state`, `set_imp_force_state`,
`set_joint_position_cmd`, `set_force_cmd`, `set_joint_drag`, `set_cart_drag`, `exit_drag`,
`run_pln_joint`, `run_pln_cart`, `stop_pln`, `setPln_joint_AB`, `setPln_Cart_AB`,
`send_pvt`, `run_pvt`, `disable`, `start_collect_data`, `get_ch_data`, `set_ch_data` 等。

完整清单见 [AI_GUIDE.md](../AI_GUIDE.md) 第 11 节或源码。

## 五、计算模块 `Marvin_Kine`

```python
kk = Marvin_Kine()
cfg = kk.load_config(arm_type=0, config_path='CommonConfig/ccs_m6_40.MvKDCfg')
kk.initial_kine(cfg['TYPE'][0], cfg['DH'][0], cfg['PNVA'][0], cfg['BD'][0])

print(kk.fk([0]*7))                       # 正解
points, pset = kk.movLA(start, end, ref, 50, 100, 50)   # 规划
kk.destroy_point_set(pset)                # 用完销毁
```

- 左臂 `arm_type=0`，右臂 `arm_type=1`，并且取 `cfg[...][0]` / `cfg[...][1]` 对应参数。
- 规划点集 `pset` 交给 `robot.setPln_Cart` / `setPln_Cart_AB` 执行。
- 所有 API 以源码和 [python_doc_kine.md](../python_doc_kine.md) 为准。

## 六、干涉检测 `Marvin_Interference`

流程：`create` → `init` → `update_cord` → `calc_interf_distance` → `on_is_interf` → `destroy`。
示例：[`DEMO_PYTHON/showcase_drag_interference.py`](../DEMO_PYTHON/showcase_drag_interference.py)。

## 七、常见问题

| 问题 | 原因 |
|---|---|
| `OSError: 找不到指定模块` | 动态库缺失，或依赖的 VC/MinGW 运行库缺失 |
| 导入成功但订阅全 0 | 未连接成功、防火墙、网段不对、APP 占用 |
| 设置参数无效 | 原接口未 `send_cmd()` |
| `movL_synchronous` 报未导出 | `SDK_PYTHON/` 中 `libKine` 不是最新版本 |
| 退出后其他程序连不上 | 未调用 `release_robot()` |

## 八、编译动态库

见仓库根目录 [README.md](../README.md#8-编译与库更新) 或 `DEMO_C++/readme.md`。
自动化脚本：

```bash
./marvinSDK_ubuntu_100343.sh      # Linux
marvinSDK_windows_100343.bat      # Windows
```

## 相关文档

- [SDK 总览](../README.md)
- [AI 开发指南](../AI_GUIDE.md)
- [Python 控制 API](../python_doc_contrl.md)
- [Python 计算 API](../python_doc_kine.md)
- [Python 示例索引](../DEMO_PYTHON/readme.md)
