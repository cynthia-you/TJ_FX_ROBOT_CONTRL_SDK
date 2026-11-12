# AGENTS.md — 给代码代理的入口

> 在本仓库中生成、修改或审查代码前，**先读 [AI_GUIDE.md](AI_GUIDE.md)**。
> AI_GUIDE 包含真实 API 清单、调用顺序、单位、安全约束和常见反模式。

## 必须遵守

1. **只使用真实存在的接口。** 以 `SDK_PYTHON/fx_robot.py`、`SDK_PYTHON/fx_kine.py`、
   `kinematicsSDK/FxRobot.h`、`contrlSDK100343/MarvinSDK.h` 为准。
2. **不要编造参数、返回值、单位或示例文件名。** 不确定就查源码或标记 `TODO: verify`。
3. **涉及真实机器人运动的代码必须包含**：连接检查、清错、速度/加速度限制、到位判断、
   `release_robot()` / `OnRelease()`。
4. **原控制接口必须批处理**：`clear_set()` → `set_*` → `send_cmd()`。
5. **左右臂索引不能混**：Python `'A'`/`'B'`，数组 `outputs[0]`/`outputs[1]`，计算 `arm_type`/`RobotSerial` = `0`/`1`。
6. **规划点集要释放**：`destroy_point_set(pset)`。
7. **不要修改用户未要求的动态库、可执行文件或无关业务代码。**

## 文档路由

| 需要 | 文件 |
|---|---|
| 总览 / 快速开始 | [README.md](README.md) |
| 权威开发规则 / API 清单 | [AI_GUIDE.md](AI_GUIDE.md) |
| 控制 API（Python / C++） | [python_doc_contrl.md](python_doc_contrl.md) / [c++_doc_contrl.md](c++_doc_contrl.md) |
| 计算 API（Python / C++） | [python_doc_kine.md](python_doc_kine.md) / [c++_doc_kine.md](c++_doc_kine.md) |
| 示例索引 | [DEMO_PYTHON/readme.md](DEMO_PYTHON/readme.md) / [DEMO_C++/readme.md](DEMO_C++/readme.md) |
| 错误码 / 故障排查 | [TROUBLESHOOTING.md](TROUBLESHOOTING.md) |
| 上位机 | [MarvinPlatform_EN/README.md](MarvinPlatform_EN/README.md) |

## 生成新示例时

优先复制最接近的现有示例的调用顺序（连接 → 清错 → 模式 → 指令 → 到位 → 释放），
再替换参数。不要从零设计流程。

生成完成后，对照 AI_GUIDE 第 10 节的自检清单逐项检查。
