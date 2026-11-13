# 一期：Grok 观察、抓放评估与历史检索

目标：黄色球和纯黑色漏斗都可以改变位置；在已验证的桌面工作区内完成抓球、转运、入漏斗。验收依据是视频中可见的结果，目标动作时长不超过10秒。毫米仅作为内部坐标单位，不是本期成功指标。

**bench-layout-v2当前操作**：[手动标定与首轮实物试验交接](bench-layout-v2-runbook.md)。界面已收敛为手动标定与试验记录，移除逐关节步进、快速示教和固定抓放位置页面。沿用现有全部释放/恢复扭矩接口；本次不改官方固件或板载网页。新的只读采点入口支持在人工持续支撑、全部扭矩释放时连续记录五点；不复用或覆盖旧标定。

本分支已接通[首轮实物抓取与提升](bench-first-grasp.md)：从本机当前RGB-D与五点标定生成路径，实际张爪、下降、闭爪、提升并录10秒。OpenCode自身的公司Grok核对预览并评估关键帧，不需要再配置第二个模型HTTP网关。`already_clear` 后继续抓取入口，不停在起步检查。漏斗投放及通用空间抓放尚未验收。

## 当前方案不使用任务位置示教

新任务规划器 `task_planner.plan_from_world` 不读取 `pick_poses_v2.json`，也不要求Home、Hover、Grasp、Place。球和漏斗目标取自本轮经过标定的场景，接近/转运/释放/撤离高度由当前球、漏斗口、桌面和经过验证的夹爪净空计算。漏斗移动后重新观察，不手填或沿用固定Place。

仍然需要设备标定：相机到机械臂坐标关系、桌面高度、夹爪TCP与尺寸、开合角和可用工作区。这是让设备理解空间，不是逐个示教任务目标。棋盘到机器人重新采样的几何对应点只用于校准，不是抓放路径。

```bash
python -m evoarm_rsi.task_agent plan --world /path/to/fresh-world.json \
  --geometry /path/to/verified-geometry.json --output /tmp/evoarm-task-plan.json
```

这两个JSON应由真实感知/标定适配器提供，不能从示例值或Grok猜测值手工制造“verified”。世界状态需含同一主机的真实时间戳、匹配的setup/calibration版本、当前TCP、球心和半径、漏斗口中心与口径；几何需含机器人基座TCP坐标系、桌面高度、夹爪参数、已验证工作区及观测最大时效 `max_scene_age_s`。模型处理旧图不能把该时间戳刷新为“现在”；真实执行前需要当前观测复核目标。当前 `inspect` 只生成图像框和表面深度，尚未输出这套经过验证的机器人世界状态。

规划输出是高层技能序列，`taught_positions_used=false`、`motion_authorized=false`、`execution_eligible=false`。后续执行适配器还必须检查整臂逆运动学/扫掠空间、TCP到固件末端参考点的转换及实测耗时，不能直接把高层技能当T104串口指令。

上述通用规划器仍保持只读。首轮实物尝试可在 `/experiments` 点击观察、核对预览并执行，或使用 `grasp_once prepare/execute` 的同一专用受限链路，按当前图像、已有平面标定、已知球径及默认工具机构检查路径；棋盘/深度不完整与球径偏差作为感知警告进入本轮记录，有明确坐标冲突时拒绝；其实验性夹取偏移不伪装为全局已验证TCP。

世界状态还需真实 `current_gripper_rad`。初始抬升保持该夹爪角度，达到净空后再执行张爪；不在桌面触点同时张爪和抬升。

## 相机、棋盘或底座移动后的处理

旧文档中的“不要重复拍空棋盘”只适用于布置完全没有变化的情况。已知相机移动时，旧像素到棋盘映射失效；棋盘或机械臂底座也移动时，旧棋盘到机械臂XY映射同样失效。先停用旧目标和旧路径，给新布置分配新的setup版本，例如 `bench-layout-v2`；旧历史保留，不混入新布置的空间补偿。

先固定新布置，备份当前Gemini参考、XY标定、标定草稿、示教和抓取配置。备份是为了保留记录，不能把文件存在当作本次标定仍有效。不要自动删除或覆盖旧文件，也不要向运行中的旧标定服务发送运动指令。

第一步仅通过已有Gemini相机服务录制10秒**完整空棋盘**，应能看到9×6内部角点。拍摄前由现场人员确认棋盘无遮挡；不移动机械臂去清空视野。拍摄无效时报告，等待现场调整。

确认实际单格边长后，在独立候选路径生成新参考（将示例中的路径、数值替换为本次真实值）：

```bash
python -m evoarm_rsi.board_reference /path/to/NEW/recording \
  --square-mm 24 --output /tmp/evoarm-new-board-reference.json
```

`24`不是要求值，必须采用实测结果。该命令只生成候选参考，不更新机器人XY，不授权运动。相机仅改变安装位置时，SDK内参不一定改变；仍需核对实际分辨率、设备和RGB/深度对齐。相机到棋盘/机器人的外部几何必须重建。

第二步用**本次新参考**通过 `bench_calibration` 建立四点棋盘平面到固件末端坐标的拟合，加一个独立验证点。五点使用同一个物理棋盘原点/轴方向和同一个官方末端点，旋转/开合保持一致，俯仰尽量一致但允许变化并记录提示。末端点轻触棋盘而非悬停，才能同时采集棋盘表面的XYZ对应关系。重新摆放后的旧P1～V1记录不能复用，原始记录保留为历史。五个位置需要现场操作，不能让OpenCode根据旧目标自主摆臂。平面验证通过仍不代表RGB-D空间外参、抓球TCP或整臂路径已经验收。

最后核对新桌面布置下的工具抓取高度、Hover/转运净空和漏斗入口；旧示教坐标只供比对。可移动漏斗始终要在当前画面中重新识别。新标定通过前不把 `/calibration` 已保存状态或旧Place快照当作可执行凭据。

## 分工

- Grok：公司批准的模型，识别黄球和当前黑色漏斗开口，结合RGB-D与历史判断结果，提出下一轮参数候选。
- OpenCode：在本机操作已有机械臂/相机服务，运行发布好的程序，回传实测记录；不开发代码。
- `ArmService`：唯一串口所有者。所有动作仍经过它的授权、限位、反馈与故障处理。
- `camera_service`：仅相机进程在确有需要时使用sudo，普通用户通过现有UNIX socket访问。Agent和控制器不提权。
- 本地 `TaskMemory`：每次Grok调用前检索同一 `setup_id` 的历史结果。相机、棋盘或机器人基座改变后应更新setup版本。
- Langfuse：可选的历史后端；配置后会真正查询并注入Grok上下文。故障时使用本地记录，不影响电机控制。

## 本次新入口

安装普通依赖和视觉依赖，使用公司已有的Grok环境配置。不要将凭据提交到Git或贴到聊天。

```bash
python -m pip install -r requirements.txt -r requirements-vision.txt
python -m unittest discover -s tests -v
python -m evoarm_rsi.task_agent doctor --output /tmp/evoarm-handoff.json
```

`doctor` 只调用当前控制器的GET接口、读取本地标定/示教数据及相机健康状态。它不打开串口、不发运动指令、不解锁、不切换扭矩、不修改标定。机器人正在执行动作时跳过反馈采样。

新诊断还会列出系统可见端口及控制器声称连接的端口，并区分 `connected` 和 `fresh_robot_feedback_verified`。前者只是原服务仍持有连接对象，不能证明设备仍在线；历史扭矩字段也不等于新的实际反馈。端口消失时先排查设备/旧句柄，不把它归因于肩部执行偏差。

历史 `/teach` 的 `Unexpected token 'I', "Internal S"...` 来自前端把HTTP500纯文本当成JSON。POSIX `termios.error(6, 'Device not configured')` 不属于 `OSError`；串口层将其规范化为通信错误、接口返回JSON409。现在旧页面跳转至新版手动标定，新页面同样对非JSON500显示HTTP状态和接口路径。请求不会自动重试，失败的运动传输会撤销授权并保留故障。修复错误处理不等于恢复串口连接。

代码更新到独立worktree不改变已运行的服务。真实服务加载修复前，先核对其工作目录和端口；不要为清除错误而反复点解锁、重启或拔插USB。串口重新打开/USB复位可能触发固件初始化，需现场确认完整运动空间后单独安排。

普通用户调用已有相机服务，记录10秒场景，再对结果目录分析：

```bash
python -m evoarm_rsi.camera_service health
python -m evoarm_rsi.camera_service record --seconds 10
python -m evoarm_rsi.task_agent inspect /path/to/returned/trial/folder \
  --setup-id bench-v1 --output /tmp/evoarm-scene.json
```

`inspect` 发给公司Grok的是同一帧RGB、空间深度网格和检索出的历史摘要；随后对模型识别的黄球、漏斗开口分别计算局部深度。图像与深度按真实时间戳配对。深度空洞输出unknown，不视为空闲空间。漏斗内部的深度不冒充漏斗边缘或释放高度；球表面也不冒充球心。

识别结果输出 `motion_authorized=false`、`robot_targets_available=false`。下一步需要确认当前相机坐标到机械臂坐标的变换，以及漏斗口的几何与释放高度；可移动漏斗不能沿用旧Place姿态。SDK内参未验证时不输出可用机器人XYZ。

## 10秒预算

动作预算从第一条实际动作开始，到释放并完成必要撤离为止。相机预热、场景定位和Grok事后分析独立计时。模型分析延迟不能从视频长度直接推定。

采集上限20fps不等于真实帧率。新录像会根据帧时间戳核对容器帧率；偏差超过1%时离线修正平均播放速度，并将原始MP4保留为 `rgb-capture-original.mp4`。帧顺序和索引不变，逐帧时间戳仍是实际时间依据；平均帧率不能消除采样抖动。

已有录像无需重拍，可以单独生成正确平均播放速度的副本：

```bash
python -m evoarm_rsi.video_timing /path/to/existing/trial --output /tmp/rgb-realtime.mp4
```

当前控制器保持至少0.8秒的指令间隔。因此14个阶段仅间隔下限就是10.4秒，尚未包含最后阶段运动与反馈延迟。`timing_preflight` 会显式报告这种不可满足的计划；不会靠提高速度、忽略反馈或放宽故障判断冒充达标。

10秒内真实抓放能力仍需本地实测。超时后停止后续指令不能保证中止固件中已经执行的动作，软件时限不能充当硬件急停。

## 真实试次的评估契约

以后由本地执行适配器生成以下报告；时间必须来自真实事件，不得手填伪造。`start` 是首次运动，`finish` 是完成撤离，`close/lift/release` 等事件用于选取对应视频帧。反馈轨迹、目标与实际指令仍保留在原始记录中。

```json
{
  "kind": "pick_place_trial_v1",
  "simulated": false,
  "capture_trial_id": "the_actual_recording_trial_id",
  "setup_id": "bench-v1",
  "grasp_attempted": true,
  "placement_attempted": true,
  "motion_error": null,
  "policy": {"offset_x_mm": 0},
  "events": [
    {"stage": "start", "host_monotonic_ns": 1000000000},
    {"stage": "close", "host_monotonic_ns": 3000000000},
    {"stage": "lift", "host_monotonic_ns": 4000000000},
    {"stage": "release", "host_monotonic_ns": 8000000000},
    {"stage": "finish", "host_monotonic_ns": 10000000000}
  ]
}
```

上面的数字仅说明数据格式，不能作为真实试次使用。

```bash
python -m evoarm_rsi.task_agent review /path/to/real/capture \
  --setup-id bench-v1 --motion-report /path/to/real/motion-report.json \
  --output /tmp/evoarm-task-review.json
```

评估区分 `pickup_observed`、`ball_entered_funnel`、`visual_task_success`、`within_10s` 和 `goal_met`。缺失关键视角、没有投放尝试或无法确认时保留unknown。动作报错时禁止产生自动补偿建议。

候选最多修改一个受限参数；目前只持久化建议，不自动修改控制策略或再次驱动机械臂。任务闭环的物理执行和策略晋升尚待接入、验收。相同目标上的一次偶然成功不能算整个工作区已经改善。

## Langfuse回读与导出

本地记忆无需Langfuse依赖或网络。将Langfuse用于本项目时，配置公司允许的HTTPS实例：

```text
EVOARM_LANGFUSE_URL=<approved HTTPS instance>
LANGFUSE_PUBLIC_KEY=<project public key>
LANGFUSE_SECRET_KEY=<project secret key>
```

回读使用 `/api/public/v2/observations`，提取 `evoarm.pick_place` 的结构化输出并按setup版本过滤，最多给Grok最近5次摘要。远端输出经过相同字段验证。不会把历史提示词当作运动授权。

显式导出在动作窗口之外进行，使用可选的Langfuse OTEL SDK：

```bash
python -m pip install -r requirements-agent.txt
python -m evoarm_rsi.task_agent sync ACTUAL_TRIAL_ID
```

依赖公司Langfuse v4或兼容的Cloud API。`flush`提交完成不保证立即可查询。未配置、网络失败或缺少可选SDK时，本地结果仍保留。当前导出的是任务结果摘要，完整LLM调用span/成本追踪可以后续接入；不会自动上传原始视频。

## OpenCode首次交接：只读，不做动作

1. 在已有本机仓库 `git fetch origin`，将 `origin/feature/rsi-grok-pick-place-10s` 放到独立的detached worktree运行新命令。保留现有控制服务、所在分支及未提交改动；不重启服务，不stash/reset。
2. 使用已能运行EvoArm的普通用户Python环境，在新worktree执行 `task_agent doctor`。回传报告及机械臂型号、固件版本（知道多少写多少）。不读取或复制 `.env`、公司token。
3. 若原相机服务已启动，调用 `camera_service record --seconds 10`，返回实际目录、相机内参文件、录像首/末帧和一次当前场景图，图里同时包含黄球、黑色漏斗开口和夹爪。不重新拍空棋盘，不覆盖既有五点XY标定。
4. 有普通用户公司Grok环境时可运行 `inspect`，回传 `/tmp/evoarm-scene.json`。公司配置不完整就报告缺少哪个变量名，不报告变量值。
5. 首轮不要执行运动、复位、扭矩切换或肩部试探。先让开发侧根据真实状态、几何数据与现有故障，生成下一轮明确的操作指令。
