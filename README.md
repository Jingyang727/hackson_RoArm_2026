# EvoArm · RoArm-M3-S 工作区

当前目标：公司 Grok 观察黄色球与可移动黑漏斗，在已验证的桌面工作区内完成抓起、入漏斗，并以10秒为完整动作目标。

本分支提供 **RSI自动抓放与学习**：RGB-D定位黄球与漏斗，录制10秒抓放，直接调用公司Grok API判断结果、更新参数，失败后独立执行官方T100复位并重试，成功后保存策略。网页保留 **坐标验证**、**手动标定** 和 **抓取试验**；自动任务从抓取试验页启动，主界面不要求人工复核每一轮。

[坐标验证预览](docs/images/coordinates-preview.png) · [手动标定预览](docs/images/manual-calibration-preview.png) · [抓取页面预览](docs/images/grasp-ui-preview.png)（均为模拟画面与反馈）

[移动控制 checkpoint 与 RSI 接口约定](docs/rsi-control-checkpoint.md)：后续RSI复用当前已验收的XYZ/T104执行链、T1051反馈与人工T100复位入口。坐标图支持“左右镜像”，只影响显示和选点映射。

[RSI v6：前后实帧与修正效果](docs/rsi-effect-learning-v6.md)。默认`grok-4.7-build`，直接调用公司API。将前后对应动作的图像、实际参数和反馈交给有状态学习循环，明确判断更近、更远、越过或没变化，再决定试验步长或撤销修正。网页保留实时过程与Grok公开回复。

## 当前状态

- [RSI v10：执行诊断与防重复学习](docs/rsi-execution-learning-v10.md)：修复视觉子Agent相对路径读图权限，CLI默认输出摘要；合并高处接近、抬升时切换运输姿态。超时和抬升不足可按时序/反馈做单项执行试验；未复核或未解决的同策略失败不原样重试。10秒窗口与几何检查保留。

- [RSI v9：现场失败重放与启动修复](docs/rsi-reported-failure-v9.md)：修复抓取俯仰强加给运输/投放造成的漏解，按各阶段姿态重算指尖目标；实际注册 `/roarm-rsi` 命令，提供新鲜readiness与明确的armed授权协议。上传两段失败录像重放可生成80°抓取、50°运输/投放路径；实际实物效果仍待现场验证。

- [RSI v8：先抬升路径与两层Agent](docs/rsi-route-agent-v8.md)：完整抓放采用抬升→高处平移→竖直下探，球径与指尖偏移单独计算，漏斗上方停稳释放。OpenCode主Agent维护上下文和数值决策，Grok只读子Agent只看实际图片给定性结果，结论绑定本轮图片包并保存。

- [实时棋盘重新标定与误差诊断](docs/live-recalibration.md)：直接采集当前棋盘、检测54个角点并开始新五点记录；显示逐点误差方向、实际行列、重采前后值与已采用的实测尺寸。无参考时也能看实时画面；新标定单独保存，检测失败不回退旧布局，重启后可续采。

- [RSI v7：完整向量修正与 OpenCode 执行 Agent](docs/rsi-agent-tools-v7.md)：修复 ±30mm 逐轴裁剪造成的步长和方向失真，隔离旧错误补偿；新增 `roarm-rsi` 项目 Agent、HTTP 操作工具和按标定持久经验。网页可独立“采集当前物体坐标（不运动）”，不依赖完整抓放规划，避免拿旧位置验证已移动物体。可沿用 OpenCode 公司模型登录。网页直接 API 循环继续可用，两种编排选一种启动。

- [XYZ坐标验证](docs/coordinate-validation.md)：在三维图上选XY、拖动高度生成XYZ（也可手填），预览先抬升→高处平移→竖直到点的完整路径，每段发送一次T104并等待反馈。自动模式优先保持当前俯仰，必要时计算其他俯仰并明确显示起点→终点；也可选择保持俯仰。已移除旧训练范围、选点网格截断、固定25mm门槛和额外收腕流程，保留真实几何、桌面与路径检查。保持旋转与合法夹爪开度，保留原始反馈，显示逐轴误差并保存独立的实物验收。预览不要求锁定扭矩，执行前仍确认六关节锁定；页面显示原始状态和禁用原因。另可查看已录制深度图并将表面点转换到机械臂坐标。
- 官方复位：坐标页“恢复官方初始姿态”先展示目标角度，确认静止且阻挡解除后做只读稳定复核，再经原有串口所有者发送一次T100；显示反馈进度，持久化请求编号防止重发，原XYZ预览失效。有动作未完成时不能叠加；复位不规划避障，不能作为急停或强行脱困。

- 五点标定：复用已有Gemini空棋盘录像，手动对准并采三组静止反馈；四点拟合、第五点独立检验，逐点保存并支持续采。
- 实时对准：相机画面同时叠加五点位置，当前点高亮、已记录点打勾；标记不依赖每帧检测完整棋盘，支持遮挡。断流时暂停采点，恢复扭矩操作仍可用。
- 手动方式：使用现有全部释放接口，持续支撑机械臂，连续记录五点；无需每个点重新上锁，五点结束再恢复扭矩。
- 释放的是全部六关节，旋转与夹爪也会松开。页面显示相对首点的角度差，超出5°时要求核对并调整回同一触点。俯仰尽量一致，差异较大时保留采点并提示。
- [RSI自动闭环](docs/rsi-effect-learning-v6.md)：点击“开始自动抓放”后自动观察、抓放、直接API分析和参数学习。规划或动作异常直接通过已有控制器复位到官方姿态，复位不依赖出错的IK、相机或标定。规划失败后从官方姿态重试一次；真正不可达时停止在官方姿态。默认最多5轮，成功即停止。每轮有ZIP备份与Langfuse待同步记录，失败规划和复位也有独立诊断包。
- 固定布局：相机、机械臂底座与桌面默认保持不变，直到用户报告变化。保存的五点标定继续使用，棋盘可撤走，运行时不再要求或重新检查棋盘角点。
- 标定后[从当前姿态起步](docs/bench-first-motion.md)：低位检查抬臂，高位跳过多余动作；官方范围内不再额外收腕。`already_clear` 后继续目标规划，无需回到标定触点或固定示教姿态。

**改动覆盖本机网页与任务编排，官方板载网页、固件和串口驱动未修改。** 继续复用已有串口所有者和扭矩接口；自动任务期间阻止其他入口插入运动，不新增扭矩命令。

已核对的官方20260701源码中，T210释放会先回位，再释放全部舵机；恢复扭矩也可能使机械臂移动。页面会在切换前明确说明，并要求现场支撑及检查运动范围，不把现有功能描述成原地释放或部分关节锁定。

## 本机运行

从现有独立worktree更新 `feature/rsi-pick-place-loop-v1`（基于main的`checkpoint-xyz-control-v1`），保留原工作目录、未提交改动与所有旧标定数据。代码更新后实际运行的服务也必须从新worktree启动。

```bash
python -m pip install -r requirements.txt -r requirements-vision.txt
EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=0 EVOARM_SETUP_ID=bench-layout-v2 \
python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

需要UI实际抓放时，将上述 `EVOARM_ENABLE_MOTION=0` 改为 `1` 后启动，打开 http://127.0.0.1:8765/experiments 。已经验收过的坐标控制和保存标定继续沿用，无需重新采点。开启实物模式本身不下发抓取；用户确认范围并启动后才进入自动任务。详见[浏览器操作说明](docs/rsi-effect-learning-v6.md)。

打开 http://127.0.0.1:8765/calibration 。页面不会自动连接串口或切换扭矩；连接操作可能触发固件初始化，需要现场看住机械臂。

相机继续使用独立服务，只有该相机进程按实际需要使用sudo。已有相机health返回 `live_preview: true` 和 `recording_request_ids: true` 时直接沿用；仅能力缺失时先停止旧服务再启动新版，保持一个SDK所有者。控制UI和Agent保持普通用户权限。实时画面、快照和RGB-D录像共用一个SDK采集线程；切换到参考图或快照时明确标明非实时。详见[本机更新与实时预览说明](docs/bench-layout-v2-runbook.md)。

## 现场流程

- [RSI前后对照、自动抓放、策略学习与本地更新](docs/rsi-effect-learning-v6.md)
- [bench-layout-v2 当前标定与首轮试验](docs/bench-layout-v2-runbook.md)
- [Grok抓放规划、10秒计时与历史检索](docs/grok-pick-place-v1.md)
- [旧控制台说明归档](docs/legacy-control-console-v0.4.md)

新版标定生成独立文件，不覆盖 `robot_xy_calibration.json` 或旧示教点。用户报告相机、底座或桌面空间关系改变后，采用新布置版本重新标定；撤走或移动棋盘本身不使已保存的变换失效。

## 验证

```bash
python -m unittest discover -s tests -v
node --test tests/*.test.cjs
```

单元测试不连接真实机械臂。可另用 `tests/browser_manual_calibration.py` 、`python -m tests.browser_grasp_ui` 和 `python -m tests.browser_coordinates` 在本地UI服务上运行Chromium验收；所有设备接口均被模拟拦截。所有真机操作仍需现场执行。
