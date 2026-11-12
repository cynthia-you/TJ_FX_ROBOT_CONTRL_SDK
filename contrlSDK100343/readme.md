# contrlSDK100343 — 控制 SDK 源码（当前维护版本）

> 负责：连接机器人、切换控制模式、下发指令、订阅状态、采集数据、PVT、末端通讯。
> 对应 Python 封装：`SDK_PYTHON/fx_robot.py`。

## 一、使用前提

- 控制器系统版本 **100343 及以上**。
- 100343 之前的控制器使用旧的 `contrlSDK/`（已停止维护）。
- 100343 版本修改了通信协议并增加校验，**不向下兼容**。

## 二、目录内容

| 文件 | 说明 |
|---|---|
| `MarvinSDK.h` | C++ 接口头文件（原接口 + 简明式接口） |
| `MarvinSDK.cpp` | 导出接口实现 |
| `Robot.cpp/.h` | 机器人通信与状态管理 |
| `FxRtCSDef.h` | `DCSS` / `RT_IN` / `RT_OUT` 等通信结构体定义 |
| `makefile` / `makefile_dll` | Linux 编译脚本 |

## 三、编译

### Linux（.so）

```bash
cd contrlSDK100343
g++ *.cpp -Wall -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN
# 或
make -f makefile
```

### Windows / MinGW（C++ 用 .dll）

```bash
g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN
```

### Windows / Python 用 .dll（静态链接运行库）

```bash
g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll \
    -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC \
    -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
```

编译完成后，把动态库复制到实际运行目录：

- Python：`SDK_PYTHON/`
- C++ 示例：`DEMO_C++/`

也可以直接运行根目录的 `marvinSDK_windows_100343.bat` / `marvinSDK_ubuntu_100343.sh`。

## 四、接口文档

- [C++ 控制 SDK](../c++_doc_contrl.md)
- [Python 控制 SDK](../python_doc_contrl.md)
- [AI 开发指南](../AI_GUIDE.md)
- 示例：[DEMO_C++](../DEMO_C++/readme.md) / [DEMO_PYTHON](../DEMO_PYTHON/readme.md)

## 五、注意

- 使用原接口时，参数设置必须位于 `OnClearSet()` 和 `OnSetSend()` 之间。
- 左右臂后缀：`_A` 左臂，`_B` 右臂。
- 连接后必须 `OnRelease()`。
