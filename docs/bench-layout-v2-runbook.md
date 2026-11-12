# bench-layout-v2：新标定与首轮实物试验交接

本流程使用已完成的空棋盘试次 `80b87a1602554583a4ec0386922f2465`，实测单格24.0mm。
不使用旧XY标定或Home/Hover/Grasp/Place。OpenCode只运行已有程序、操作设备和回传结果，不开发代码。

## 1. 让实际控制服务加载新代码

在本机原仓库fetch `origin/feature/rsi-grok-pick-place-10s`，更新或创建独立worktree；保留原分支、未提交改动和所有本机数据。
仅更新worktree不会更新已经运行的服务。记录当前8765端口进程的PID、启动目录和实际commit，再停止该控制服务。
相机服务保持独立运行；不要启动两个串口所有者。现场人员看住机械臂并清空扫掠区后，使用原有普通用户Python环境，从新worktree启动：

```bash
EVOARM_ENABLE_MOTION=1 EVOARM_ENABLE_TEACH=0 EVOARM_ALLOWED_JOINTS=1,2,3,4,5,6 \
python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

随后通过现有控制台连接实际可见的串口（上次为 `/dev/cu.usbserial-130`）。打开串口可能触发固件初始化，因此连接时需要现场人员在场。
不执行官方复位、自动故障恢复或扭矩释放。出现fault、pending或端口丢失则回传真实状态，不反复重连。

```bash
python -m evoarm_rsi.task_agent doctor --output /tmp/evoarm-handoff-calibration.json
```

只在fresh feedback通过、fault和pending均为空后继续。新接口是 `GET /api/calibration/stationary-sample`；它持有控制器串口锁，连续读三次反馈，不开启串口、不解锁、不发送运动、不修改旧标定。

## 2. 从既有录像生成五点图

在新worktree、普通用户环境运行：

```bash
python -m evoarm_rsi.bench_calibration init \
  "$HOME/.evoarm/rsi/experiments/80b87a1602554583a4ec0386922f2465" \
  --setup-id bench-layout-v2 --square-mm 24 \
  --output "$HOME/.evoarm/rsi/calibrations/bench-layout-v2/session.json"
```

输出同目录的 `session.png`、`session-reference.json` 和 `session.json`。已有session时直接使用status续采，不再次init。
图中9×6角点的列/行从0开始；P1=(1,1)、P2=(7,1)、P3=(7,4)、P4=(1,4)，V1=(4,2)。这些是待现场验证可达的采点位置，不是运动目标指令。
相机、棋盘和底座必须保持录制空棋盘时的布置；若已移动，使用新的setup版本重新采空棋盘。

## 3. 逐点人工对准，程序只采反馈

先显示 `session.png`，请现场人员用现有手动控制把**官方图中标黄的末端参考点**对准P1，并轻触棋盘表面、不施压。
不以夹爪中心、球心或其他指尖替代；不能仅在上方悬停，否则本次棋盘表面Z对应关系不成立。
参照[官方坐标说明及末端示意图](https://docs.waveshare.net/RoArm-M3/Advanced-Tutorials/)：X向前、Y向机械臂自身左侧、Z向上，毫米单位；原点位于肩部高度而非桌面，故桌面Z可能为负。
程序记录的是T1051编码器正运动学反馈；`tit`是末端Pitch，`t`是腕部关节角，不混用。固定末端参考点与实际夹球中心的偏移仍需另行验证。
五点需保持相同Pitch/Roll和夹爪开合姿态。现场操作者需要决定每次手动移动的方向和避让路径；不能从旧标定计算自主目标。
若任一点不可达或要发生干涉，保留已有点并回报；不要强行下探，也不要自行更换参考点。

现场确认P1已对准且机械臂静止后：

```bash
python -m evoarm_rsi.bench_calibration capture \
  --session "$HOME/.evoarm/rsi/calibrations/bench-layout-v2/session.json" \
  --point P1 --aligned
```

`--aligned`只表示现场已核实该物理对准，不会触发移动。依次对准并将 `--point` 换成P2、P3、P4、V1再采样；**不能保持原姿态连续运行五条采点命令**。
每点记录棋盘坐标、原始像素、三次T1051 XYZ/关节角/扭矩/Pitch/Roll/G、时间戳与独立原始采样文件。
XYZ或关节仍在变化时不记录该点。重新采已记录点需显式加 `--replace`，先前记录仍归档。

断点续采：

```bash
python -m evoarm_rsi.bench_calibration status \
  --session "$HOME/.evoarm/rsi/calibrations/bench-layout-v2/session.json"
```

## 4. 拟合与独立验证

```bash
python -m evoarm_rsi.bench_calibration fit \
  --session "$HOME/.evoarm/rsi/calibrations/bench-layout-v2/session.json" \
  --output "$HOME/.evoarm/rsi/calibrations/bench-layout-v2/candidate.json"
```

P1–P4用带毫米尺度的刚体变换拟合棋盘平面到固件末端坐标，V1不参与拟合。
记录训练RMS/最大误差、V1误差和棋盘面在机器人系中的平面方程。
暂用已有工程门槛6/9/10mm验收，失败时检查实际对准和物理布置，不自动放宽。
即使通过也只标记 `planar_fit_validated`；不自动覆盖 `robot_xy_calibration.json`，不冒充已验证RGB-D空间外参、抓球TCP或整臂避障。

## 5. 首轮抓放和调整需要的真实数据

回传新诊断、`session.json`、`candidate.json`、`session.png`，以及实际运行服务的commit。
接着把黄色球和可移动黑漏斗放入棋盘附近已验证的工作区，保持相机/棋盘/底座固定，通过既有相机服务录10秒当前场景：

```bash
python -m evoarm_rsi.camera_service record --seconds 10
python -m evoarm_rsi.task_agent inspect /实际返回的录像目录 \
  --setup-id bench-layout-v2 --output /tmp/evoarm-current-scene.json
```

Grok配置使用公司现有环境；缺配置只报告变量名，不回传凭据。
回传场景录像/RGB-D/内参及inspect结果后，开发侧结合五点数据校验坐标与深度，补齐本次工具/球体/漏斗口几何并生成首条试验路径。
真实执行适配器尚未接通，不能把当前plan输出或inspect框直接转成串口动作。

首次实物试验先验证一条完整抓放。以真实事件时间和10秒录像判定抓起、入漏斗与耗时，保存反馈轨迹；Grok结合本布置历史给出一个有界参数调整，再验证下一次。
先比较同一球/漏斗位置的重复试次，再换位置检查是否仍有效。当前实现保留调整建议，不自动晋升策略或再次驱动电机；这一步要在新几何和执行链路验证后接通。
