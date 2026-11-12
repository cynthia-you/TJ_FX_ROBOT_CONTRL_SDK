# MarvinPlatform — 开源上位机 APP

> 用于手动调试、验证参数、运行示例动作的图形界面。
> **APP 和 SDK 不能同时连接同一台机器人**（端口占用）。
>
> 本文件只介绍 APP；**SDK 总览请从 [../README.md](../README.md) 开始**。

## 一、首次使用前

1. 在 `CommonConfig/config/` 中放入**唯一一个**与机型匹配的配置文件：
   - CCS 6kg 3.1：`ccs_m6_31.MvKDCfg`
   - CCS 6kg 4.0：`ccs_m6_40.MvKDCfg`
   - CCS 3kg：`ccs_m3.MvKDCfg`
   - SRS：`srs.MvKDCfg`
   - 多个 `*.MvKDCfg` 会导致解析错误。
2. 确认 `SDK_PYTHON/` 下是最新编译的：
   - Windows：`libMarvinSDK.dll`、`libKine.dll`、`libInterfCheck.dll`
   - Linux：`libMarvinSDK.so`、`libKine.so`、`libInterfCheck.so`
3. 电脑与控制器同网段，能 `ping` 通。

## 二、运行

### 直接运行可执行文件

- Windows：`MarvinPlatform_win_100343.exe`
- Linux：`MarvinPlatform_linux_100343`

### 从源码运行

```bash
python -m pip install pillow
python ui_EN.py
```

要求 Python ≥ 3.10。

### 打包

```bash
python setup.py
```

## 三、功能

| 区域 | 功能 |
|---|---|
| 连接 / 状态 | 连接机器人、查看关节状态与错误、清错、上/下使能 |
| 位置控制 | 关节位置跟随、速度/加速度百分比 |
| 阻抗控制 | 关节阻抗、笛卡尔阻抗、刚度/阻尼设置 |
| 力控 | 力控参数、目标力 |
| 拖动示教 | 关节拖动、笛卡尔拖动（X/Y/Z/R） |
| PVT | 上传轨迹、运行指定 ID |
| 数据 | 1kHz 数据采集、保存 CSV |
| 参数 | 读取/设置/保存 `robot.ini` 参数 |
| 运动规划 | 关节/笛卡尔规划、双臂协同、双臂同步直线（Dual-arm synchronous） |
| 末端通讯 | 485 / CAN 数据收发 |
| 用户自定义数据 | 六维力、重力、工具动力学、刚度、IMU 等通道 |

## 四、使用建议

1. 先用 APP 熟悉机器人动作逻辑，再写代码。
2. 首次动作把速度/加速度设为 10%。
3. 切换模式后等待约 1 秒再下发指令。
4. 调试完成后先断开 APP，再运行 SDK 程序。
5. 阻抗/力控参数是参考值，不同控制器版本可能需要调整。

## 五、常见问题

| 现象 | 处理 |
|---|---|
| 软件连接失败 | 检查是否已有其他进程/APP 占用；重启控制器后等待 30~60 秒 |
| 数据不刷新 | 检查网段、防火墙；确认控制器已收到上位机数据 |
| 切换模式失败 | 先清错；确认已设置工具动力学参数 |
| 配置文件报错 | 确认 `config/` 下只有一个 `*.MvKDCfg` 且机型匹配 |
| Linux 无法运行 | 确认 `SDK_PYTHON/` 下为 Linux `.so`，并已重新编译 |

## 六、相关文档

- [SDK 总览](../README.md)
- [AI 开发指南](../AI_GUIDE.md)
- [Python 控制 SDK](../python_doc_contrl.md)
- 软件使用说明 PPT：`天机Marvin系列MarvinPlatform软件使用说明2601.pptx`（同目录）
