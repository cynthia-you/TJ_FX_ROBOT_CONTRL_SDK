# 故障排查与错误码

> 本文档汇总机器人状态、错误码、常见问题和版本兼容说明。
> 快速问题定位请先看 [README.md](README.md) 的“常见问题速查”。

---

## 一、机器人状态（`cur_state`）

### Python

```python
sub = robot.subscribe(dcss)
a_state = sub['states'][0]['cur_state']   # A 臂
b_state = sub['states'][1]['cur_state']   # B 臂
a_err   = sub['states'][0]['err_code']
```

| 值 | 含义 |
|---|---|
| `0` | 下伺服 |
| `1` | 位置跟随 |
| `2` | PVT |
| `3` | 扭矩（阻抗 / 力控） |
| `4` | 协作释放（Python 文档中部分旧版本未列出） |
| `100` | 有错误，需要清错 |
| `101` | 正在切换到位置（正常瞬态） |
| `102` | 正在切换到 PVT（正常瞬态） |
| `103` | 正在切换到扭矩（正常瞬态） |

### C++

读取 `dcss.m_State[0].m_CurState` / `dcss.m_State[1].m_CurState`，值同上。

---

## 二、手臂错误码（`err_code` / `m_ERRCode`）

> 十进制原始值；Python 和 APP 已转换为十六进制显示，C++ 拿到的是原始值。
> 伺服具体故障请对照 `JMDT驱动器报警故障和原因措施说明书`。

| 值 | 名称 | 含义 / 常见原因 |
|---|---|---|
| 1 | `ARM_ERR_BusPhysicAbnoraml` | 总线拓扑异常；EtherCAT 通讯断开等 |
| 2 | `ARM_ERR_ServoError` | 伺服故障；某轴故障、轴参数错误、轴通讯错误 |
| 3 | `ARM_ERR_InvalidPVT` | PVT 异常；文件数据错误/长度不符，或调度导致数据交互错误 |
| 4 | `ARM_ERR_RequestPositionMode` | 请求进位置失败；伺服初始化/切换状态/编码器异常、急停 |
| 5 | `ARM_ERR_PositionModeOK` | 进位置失败；伺服反馈模式切换失败、电机/内存/手臂数量配置错误 |
| 6 | `ARM_ERR_RequestSensorMode` | 请求进扭矩失败；未设工具动力学、轴外力检测不通过、内外编检查不通过 |
| 7 | `ARM_ERR_SensorModeOK` | 进扭矩失败；原因同上 |
| 8 | `ARM_ERR_RequestEnableServo` | 请求上伺服失败 |
| 9 | `ARM_ERR_EnableServoOK` | 上伺服失败 |
| 10 | `ARM_ERR_RequestDisableServo` | 请求下伺服失败 |
| 11 | `ARM_ERR_DisableServoOK` | 下伺服失败 |
| 12 | `ARM_ERR_InvalidSubState` | 内部错误；内存/调度/变量计算/空指针 |
| 13 | `ARM_ERR_Emcy` | 急停 |
| 14 | `ARM_DYNA_FLOAT_NO_GYRO` | 配置了浮动基座但没有 IMU 硬件 |
| 15 | `ARM_ERR_PdoAbnormal` | PDO 工作不正常 |
| 16 | `ARM_ERR_AxisIsVirtual` | 虚拟轴；控制器启动时未检测到从站，检查手臂与控制器是否同时上电 |

### 清错流程

```python
robot.check_error_and_clear(dcss)      # 检查并清错（推荐）

# 或手动指定手臂清错
robot.clear_set()
robot.clear_error('A')
robot.clear_error('B')
robot.send_cmd()
```

急停后机器人会自动下伺服，需要**清错 → 重新上使能**。

---

## 三、伺服错误码

```python
# Python：返回 7 个关节的错误列表（十六进制）
errs = robot.get_servo_error_code('A', lang='CN')
```

- 对照 `JMDT驱动器报警故障和原因措施说明书_1121200064_V1.0.pdf`；
- 软件和 Python 已转十六进制，C++ 原始接口返回十进制整数；
- 单个关节软复位：`robot.servo_reset('A', axis)`，`axis` 为 0~6。

---

## 四、常见问题

### 4.1 连接相关

**Q：ping 不通 / 连接失败？**
- 网线是否插好；
- 电脑是否设置成与控制器同一网段的静态 IP；
- 是否有其他进程（MarvinPlatform、旧 Python 进程、ROS）占用了连接；
- 机器人是否已 `release`。

### 4.2 订阅相关

**Q：订阅到的数据全是 0？**
- 必须先 `connect`，再等待约 0.5 s 订阅；
- 检查 `frame_serial` 是否持续变化；
- 检查防火墙；
- 是否有其他进程占用订阅。

### 4.3 回调 / 高频调用

**Q：回调只有第一次生效？**
连接/释放不需要高频回调。运动指令可以低于 1 kHz 频率发送，高频伺服来不及响应会报错。

### 4.4 到位判断

```python
# 方式一：低速标志（各关节速度 < 0.5°/s 时为 1）
sub['outputs'][0]['low_speed_flag']

# 方式二：逐关节比较
sub['outputs'][0]['fb_joint_pos']
```

C++：`dcss.m_Out[0].m_LowSpdFlag` / `m_FB_Joint_Pos`。

### 4.5 急停后指令不响应

急停会自动下伺服：先清错，再重新上使能/切模式。

### 4.6 工具动力学辨识

| 现象 | 原因 / 处理 |
|---|---|
| `ret=4`，采集时间不够 | 控制器与 SDK 版本兼容问题；使用 100343007 以上系统 + SDK |
| 加载轨迹后不运行，提示 `Load ... failed` / `no pvt file[0] to run` | 文件被 Windows 编辑器改成了 CRLF；转成 LF |

### 4.7 PVT 轨迹不运行

- 文件换行符必须是 **LF**；
- 机器人必须先运动到轨迹起点；
- PVT ID 必须与上传时一致；
- 检查控制器日志。

### 4.8 末端夹爪 / 灵巧手

- 目前支持 Modbus 485 和 CAN/CANFD；
- **不要直接把 demo 指令发给夹爪/灵巧手**，协议不一致可能导致模组死机；
- 先用供应商说明书和测试软件确认指令，再用 SDK 透传；
- CAN ID 位宽注意：32 位 ID `0x01` 发送为 `01 00 00 00`；64 位 ID `0x01` 发送为 `01 00`。

### 4.9 规划相关

| 现象 | 处理 |
|---|---|
| 逆解失败 | 看 `IsOutRange` / `IsDeg` / `IsJntExd`；参考关节不能全 0 |
| 轨迹执行结束判断不到 | `traj_state` 是 bytes，和 `b'\x00'` 比较 |
| 执行后内存增长 | `destroy_point_set(pset)` |
| 关节抖动 | 改用 `setPln_joint` / `movLA` 规划方式下发 |

---

## 五、控制器版本与兼容

### 5.1 版本号格式

`1003xx`，例如 `100335`：大版本 `1003`，子版本 `35`。

```python
ret, version = robot.get_param('int', 'VERSION')
```

### 5.2 100343 版本要求

- SDK 必须使用 `contrlSDK100343`；
- 不支持 100343 之前的控制器（如 100341/100342）；
- 100343 修改了通信协议并增加校验，**不向下兼容**；
- 旧脚本无需改代码，重新编译并替换动态库即可。

### 5.3 版本更新摘要

| 版本 | 更新 |
|---|---|
| 1003_34 | 内外编清零、编码器清错；支持仅位置模式控制（`R.A0.BASIC.CtrlType` / `R.A1.BASIC.CtrlType`，1=仅位置） |
| 1003_35 | 内外编码器检测；修复伺服出错后所有轴全部下使能 |
| 1003_37 | 任意状态下的轴外力检测，可用于计算末端外力 |
| 100343 | 新通信协议 + 校验，防多线程指令混乱 |

升级包与参数发布在仓库 Releases 页面。

---

## 六、相关文档

- [SDK 总览](README.md)
- [AI 开发指南](AI_GUIDE.md)
- [Python 控制 SDK](python_doc_contrl.md)
- [C++ 控制 SDK](c++_doc_contrl.md)
- [Python 示例](DEMO_PYTHON/readme.md) / [C++ 示例](DEMO_C++/readme.md)
