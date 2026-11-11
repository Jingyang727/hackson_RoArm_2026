# 一期：Grok 观察、抓放评估与历史检索

目标：黄色球和纯黑色漏斗都可以改变位置；在已验证的桌面工作区内完成抓球、转运、入漏斗。验收依据是视频中可见的结果，目标动作时长不超过10秒。毫米仅作为内部坐标单位，不是本期成功指标。

本分支实现了**无运动的感知与评估链路**，不是已验收的自动抓放系统。还需要本机标定数据、当前场景图片和控制响应测量，才能接通动态目标规划与真实执行。原有控制器、夹爪限位和独立 sudo 相机服务保持原有职责。

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
