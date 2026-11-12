# kinematicsSDK — 运动学与规划库

> 负责：正解、逆解、零空间、雅可比、直线/多段/双臂规划、工具动力学辨识。
> **不连接机器人**，只依赖机型配置文件。对应 Python 封装：`SDK_PYTHON/fx_kine.py`。

## 一、目录内容

| 文件 | 说明 |
|---|---|
| `FxRobot.h` | C++ 接口头文件 |
| `FXKineCommon.h` | 公共类型与 `ArmsSynchronousPlanningParams` 等结构体 |
| `AxisPln.cpp/.h` | 规划实现 |
| `makefile` / `makefile_dll` | Linux 编译脚本 |

## 二、编译

```bash
# Linux
cd kinematicsSDK
g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.so -lpthread -lrt

# Windows（C++ 用）
g++ *.cpp -Wall -O2 -shared -o libKine.dll

# Windows（Python 用）
g++ *.cpp -Wall -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC \
    -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
```

编译产物复制到 `SDK_PYTHON/`（Python）或 `DEMO_C++/`（C++）。

## 三、使用前必须确认

1. **机型配置文件正确。** `CommonConfig/*.MvKDCfg`：
   - CCS 6kg：3.1 用 `ccs_m6_31.MvKDCfg`，4.0 用 `ccs_m6_40.MvKDCfg`；
   - CCS 3kg：`ccs_m3.MvKDCfg`；
   - SRS：`srs.MvKDCfg`。
   - 配置错了可能“运行正常但结果错误”。
2. **左右臂独立初始化。** A = 0，B = 1；`load_config` 返回的是双臂参数，初始化时取对应索引。
3. **带工具时先设置工具**，否则正逆解算到法兰而不是 TCP。

## 四、文档

- [C++ 计算 SDK](../c++_doc_kine.md)
- [Python 计算 SDK](../python_doc_kine.md)
- [AI 开发指南](../AI_GUIDE.md)
- 示例：[DEMO_PYTHON](../DEMO_PYTHON/readme.md) / [DEMO_C++](../DEMO_C++/readme.md)
