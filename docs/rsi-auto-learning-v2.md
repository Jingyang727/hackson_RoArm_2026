# RSI自动闭环：本机OpenCode/Grok、恢复、学习与备份

分支仍为`feature/rsi-pick-place-loop-v1`，从main的`checkpoint-xyz-control-v1`发展。本次API版本为2，`/experiments`主界面改为自动学习；旧版单次prepare/run/review接口作为兼容接口保留。

## 现场更新

用户已确认：OpenCode 1.19.0，使用`bottlerocket/grok-4.7-build`。它是默认观察模型，复用本机公司provider和凭据；不新增模型网关，不切换到其他模型。不要求在聊天粘贴密钥。

转给本地OpenCode：

```text
更新 feature/rsi-pick-place-loop-v1 的当前服务工作区。
沿用原Gemini Python环境，安装 requirements.txt 与 requirements-vision.txt。
在停止旧网页服务后执行 python -m evoarm_rsi.rsi_journal backup-existing，为既有执行记录补齐备份；此命令不接触机械臂或相机。
保留五点标定和现有独立相机服务，只重启普通用户8765网页服务。
EVOARM_ENABLE_MOTION=1 EVOARM_ENABLE_TEACH=1 EVOARM_SETUP_ID=bench-layout-v2
EVOARM_OPENCODE_MODEL=bottlerocket/grok-4.7-build
只读检查 /api/bench/rsi/state：api_version=2，grok.configured=true。
如果找不到opencode，设置EVOARM_OPENCODE_BIN为 command -v opencode 的绝对路径后重启网页服务。
不要在更新或验收时发送运动、复位或扭矩切换；由我在网页点击开始自动抓放。
```

程序会从PATH、`~/.opencode/bin`、`~/.bun/bin`以及常用Homebrew目录寻找OpenCode。启动示例：

```bash
EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=1 EVOARM_SETUP_ID=bench-layout-v2 \
EVOARM_OPENCODE_MODEL=bottlerocket/grok-4.7-build \
/Users/jyang51/miniconda3/envs/evoarm/bin/python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

继续使用现有RGB-D相机；相机、底座和桌面不变就不重新标定，棋盘可撤走。球在桌面上，漏斗开口和夹爪应可见，开始时夹爪为空。目前不需要增加USB相机；只有真实复核多次因同一处遮挡无法判断时，再考虑侧视辅助。

## 界面与自动流程

打开`http://127.0.0.1:8765/experiments`。主界面只有工作区画面、开始、停止、当前阶段和Grok结论。深度诊断与原始记录折叠显示，无需用户填写抓没抓到或偏向哪里。

勾选一次现场范围确认，点击“开始自动抓放”。默认最多5轮，成功即停止；可在“尝试次数”中改为1、3、5或10轮。程序依次：

1. 记录2秒当前RGB-D，用已有标定定位球和当前漏斗，生成完整路径。
2. 首轮由Grok核对标出的球、漏斗和空夹爪；无法确认就停在运动前。后续有漏斗歧义时也重新核对。
3. 启动带请求ID的10秒RGB-D录像，接近球时同步张爪，下降、合抓、提升、运输、释放。若XYZ已停稳但夹爪未张到位，专用T106完成张爪后才下降。
4. 保存视频与反馈，从录像选取按时间排序的实际画面，由OpenCode自动调用公司Grok判断抓空、推球、滑落、投偏、弹出或成功。图像和时间索引明确记录；不把它写成模型逐帧看完了整个MP4。
5. 根据Grok结果更新下一轮参数。失败后检查新鲜静止反馈，先抬离桌面，再经已有串口所有者发送一次官方T100，恢复后重新定位并开始下一轮。
6. 如果可能仍夹着球，先保持夹持、移到重新确认的漏斗上方释放，再回位。会补采恢复后的RGB-D给Grok确认球的位置；不能确认时保存证据并停止，避免再把漏斗内的球当作桌面目标。

停止会取消后续动作、分析、回位和重试；已经被固件接受的动作可能继续。通信、传输或扭矩反馈失效时不能确认机械臂状态，程序停止自动恢复；普通到位残差不再要求人逐轮去“检查异常”。

## 本轮修正的执行问题

旧版四宫格的黑块来自凑偶数张关键帧的空白填充，已删除，关键帧图只包含真实帧。

旧版额外规定“到位偏差超过12mm就终止”，会在下降后尚未闭爪时中断。自动试验改成检查XYZ和六关节是否停稳，残差原样记录并交给下一轮学习，不以12mm作为抓取门槛。停稳后仍从**实际反馈姿态**重新检查下一段IK、官方关节范围和桌面路径；没有恢复旧训练范围或固定25mm目标高度限制。

抓取接触后，旧版把目标退回实测角度，可能丢失夹持预紧。本版默认只增加一次2°角度预紧，并在提升和运输时保持该目标。Grok若判断滑落来自夹持不足，可使下一轮预紧增加1°，最多5°，始终不越过官方夹爪范围；也可改为降低运输速度。没有新增扭矩命令或力传感器，**角度预紧不是测得的夹持力**。

## Grok判断与参数记忆

Grok输出结果、方向、偏差幅度`small/medium/large/unknown`、球的位置、置信度、简短证据与建议调整类别。它不发送电机指令，不填写绝对毫米值，也不编辑代码。

- XY偏差的小/中/大幅度对应下一轮2/4/6mm试验步长，先用相机变换转换方向；步长不冒充视觉测量精度。
- 太高或太低调整抓取高度；确认抓空但看不清方向时，只探索降低2mm。
- 推球提高抓取高度；滑落按Grok建议调整预紧或运输速度；弹出降低投放高度。
- 一轮只更新对应参数。看不清时不伪造成功或盲目连续抓取。
- 策略按布置与标定指纹保存，保留每次参数前后值、来源和依据。成功时额外保存`successful_policy`和统计；下一次启动继续使用保存参数。

XY学习修正仍有±30mm搜索域，高度与预紧也有明确搜索域；它们不限制球可选的工作区。成功一次不代表全工作区百发百中，需要换球位和漏斗位置继续测成功率。

OpenCode每次使用`--agent rsi-observer --model bottlerocket/grok-4.7-build --format json`，并用调用级配置禁止全部工具。公司provider配置保留，解析的只有最终JSON，推理内容和凭据不保存。真实公司模型调用仍需本机验证；云端验收使用模拟模型与机械臂。

## 录像、深度和10秒

运输T104 `spd=0.65`、最后下降`0.30`；T104没有acc字段。专用夹爪T106使用`spd=1000、acc=20`。模型分析、回位与保存不计入抓放动作的10秒目标。录像窗口将结束而动作尚未完成时，停止下发后续步骤，完整记录已执行内容；不以HTTP成功宣称抓放成功。

深度图的颜色表示表面**到相机**的距离，单位毫米；图例显示本轮颜色区间。黑色表示缺少深度，不表示黑色物体。球面深度用于拟合34mm球心；漏斗口仍以RGB中心、桌面平面和45.4mm实测高度定位，洞内桌面深度不当作口沿高度。用户无需读深度图来指导运动。

## 每轮数据、备份和Langfuse接口

每轮都保留录像、深度及内参、逐帧时间戳、预览、计划、原始反馈与逐段残差、实际写入尝试、Grok请求上下文和最终判断、参数版本及前后值、恢复动作、新鲜静止反馈、当时标定快照及服务代码版本。动作没完成也保存，不只记录成功。

- `~/.evoarm/rsi/pick_place_runs/<plan_id>/`：状态、追加事件日志、原始执行/复核/恢复报告。
- `~/.evoarm/rsi/experiments/<capture_id>/`：原RGB-D与时间戳；浏览器H.264副本独立保留。
- `~/.evoarm/rsi/backups/<plan_id>.zip`：包含观察、执行及已有恢复证据的完整快照，每个文件附SHA-256清单，打包后校验。网页可下载。
- `~/.evoarm/rsi/langfuse-outbox/<plan_id>.json`：`evoarm.rsi.trace.v1`契约，含稳定本地trace_id、输入参数、结果、复核、恢复及备份哈希；当前`export_status=pending`、`remote_submitted=false`，没有联网提交。
- `~/.evoarm/rsi/learning_sessions/<session_id>/state.json`：自动任务进度；试次和策略也有独立持久化文件。

`rsi_journal.pending_exports()`读取待同步记录，`publish_trace(packet, client)`接受已配置Langfuse客户端，在动作窗口外显式导出。未来同步失败只影响遥测，不会重放机械臂动作；导出方按本地trace_id做对账与去重。Langfuse不会自动把日志放进模型，当前Grok每轮已显式检索最近反馈与保存策略。

主要接口：`GET /api/bench/rsi/state`（版本2）、`POST /learning/start`（declaration与max_attempts）、`POST /learning/stop`、`GET /history`、`GET /context/<plan_id>`。媒体与备份为`GET /plans/<plan_id>/{video,raw-video,keyframes,depth,archive}`。页面刷新、状态GET和历史查询不会启动或恢复运动；服务重启不自动续跑旧任务。
