# bench-layout-v2 标定与经验策略快照

用户于 2026-11-15 导出，源代码为 `b1d3766412144e0f500b35d279389e66b3a65c27`。这是当前可运行版本的布局数据归档；拉取代码不会覆盖本机活动标定。

- 活动 revision：`eef40afc17fa4533a4d4198c0edf2676`。
- 标定指纹：`05368598e3e0f107d87f4f4d174a2cfaf88fdf9efffd91f97b0fbccb9dd7eed4`，与策略文件名一致。
- 导出时单格边长 **24.94 mm**，参考 RGB **1280 × 720**。
- 重新计算拟合 RMS **5.444 mm**，独立 V1 误差 **3.708 mm**，通过当前标定检查。
- 策略 **v17**；包含抓取 XY 补偿、125 mm 抬升等当前参数。
- 相机内参来自 SDK 活动彩色 profile，原始 `verified=false` 保留，不伪造独立验证。
- 用户已确认该代码版本抓起过球，但导出未能确定对应试次；本包没有成功试次 state.json，不能证明 v17 就是那轮参数，也不能证明投放成功。缺项见 `EXPORT_MANIFEST.json`。

## 加载与恢复

运行时从 `~/.evoarm/rsi/calibrations/<setup_id>/active-session.json` 选择 revision；没有指针时才读取旧式根目录 session.json。RSI 策略按标定指纹加载。相机、底座和桌面默认保持该布局；移动后需另建标定。

在原机器上继续运行无需导入。本快照保留原文件字节、记录、指纹和源路径，清单包含 SHA256。不可为了迁移而直接改写 session.json 的 records，否则策略指纹会变化。

如要在另一运行环境恢复，先备份原有同名目录，再明确导入本快照的 `calibrations/`、`pick_place_policies/` 和对应 `experiments/` 到 `~/.evoarm/rsi/`。参考视频和其他原始实验文件未随包提供；图片为 session.png。原文件中的 Mac 绝对来源路径属于溯源信息，不保证在新机器上可读。恢复标定与恢复实验视频是两件事。

此快照不包含模型 token、auth.json、串口授权或运动开关。任何导入均不应自动连接或移动机械臂。
