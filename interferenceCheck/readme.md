# interferenceCheck — 干涉检测（碰撞检测）

> 基于当前关节角，计算双臂/连杆之间的最小距离和干涉状态。
> 依赖运动学库 `libKine`，Python 封装为 `SDK_PYTHON/fx_interference.py`。

## 目录

```
interferenceCheck/
├── FxCfg/                 # 配置基类
├── Interf/                # 干涉计算源码 + Interference.h + 预编译库
└── InterferenceCfg/       # 机型几何配置
    ├── CordDef_*.Cord
    ├── ConvexDef_*.Convex
    ├── CalInputDef.Joints
    ├── CalLinkDef.Links
    └── InterfDef.Interf
```

## Python 使用流程

```python
from SDK_PYTHON.fx_interference import Marvin_Interference

ic = Marvin_Interference()
ic.create()
ic.init(cord_def, cal_link_def, input_def, convex_def, ic_def)  # 传入上述配置
ic.update_cord(cord_num, joint_data)
ic.calc_interf_distance()
is_collision = ic.on_is_interf(thresholds)
ic.destroy()
```

完整示例：[`DEMO_PYTHON/showcase_drag_interference.py`](../DEMO_PYTHON/showcase_drag_interference.py)。

## 编译

干涉检测库需要 `Interf/*.cpp` + `FxCfg/CfgBase.cpp`，并依赖运动学库中的矩阵函数。

```bash
# Linux（so）
cd interferenceCheck/Interf
g++ *.cpp ../FxCfg/CfgBase.cpp -Wall -O2 -fPIC -shared -o libInterfCheck.so

# Windows（C++ 用 dll）
g++ *.cpp ../FxCfg/CfgBase.cpp -Wall -O2 -fPIC -shared -o libInterfCheck.dll -DCMPL_WIN

# Windows（Python 用 dll）
g++ *.cpp ../FxCfg/CfgBase.cpp -Wall -O2 -shared -o libInterfCheck.dll \
    -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++
```

也可以直接运行根目录的 `marvinSDK_ubuntu_100343.sh` / `marvinSDK_windows_100343.bat`，
它们会连同控制库、计算库一起编译并复制到 `SDK_PYTHON/` 和 `DEMO_C++/`。

## 注意

- 必须在运动学库完成 `initial_kine` 之后使用。
- 几何配置（`InterferenceCfg/`）必须与机型匹配。
- 检测结果用于辅助安全判断，不能替代硬件安全防护。
