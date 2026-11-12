# contrlSDK — 控制 SDK 旧版本（不再维护）

> ⚠️ 该目录对应 **100343 之前的控制协议**，已停止维护。
> 新项目请使用 [`contrlSDK100343/`](../contrlSDK100343/readme.md)。

## 适用范围

- 仅用于旧控制器固件（100343 以下）。
- 100343 及以上的控制器**不能**使用该版本。

## 编译（仅作维护参考）

```bash
# Linux
cd contrlSDK
g++ *.cpp -Wall -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN

# Windows（C++）
g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN

# Windows（Python 用）
g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll \
    -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC \
    -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
```

## 文档

接口与 100343 版本基本一致，差异见 [SDK 总览](../README.md) 的“SDK 更新”说明。
