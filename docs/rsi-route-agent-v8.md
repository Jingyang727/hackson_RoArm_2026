# RSI v8：先抬升的抓放路径与 OpenCode/Grok 分工

本文记录v8设计。最新启动入口、授权协议与独立投放姿态见 [RSI v9](rsi-reported-failure-v9.md)。

更新 `feature/rsi-pick-place-loop-v1` 并从正在运行的worktree重启普通用户8765服务，沿用原Gemini Python、相机服务、当前实际使用的标定及尺寸。v8时状态 `/api/bench/rsi/state` 为 `api_version=8`，计划 `route_policy.version=2`。更新验收先读取状态与规划预览，现场由用户开始物理任务。云端没有移动机械臂。

## 抓放路径

旧抓放仅在起点很低时抬升，运输到漏斗的T104同时做XY和下降；上一段实际落点有偏差，也会把原本竖直的下一段变成斜线。现在：

1. 保持实际XY和工具姿态先竖直抬离桌面。已处于高处时优先沿高处通道；IK需要降低高处高度时，也先在原XY竖直调整。
2. 在通行高度平移到球顶上方，途中张爪；开度未兑现时，通过专用夹爪接口确认后再下探。
3. 在球上方竖直调整到接近姿态，再竖直下探、合抓、竖直抬升。
4. 在高处平移到漏斗中心上方，独立竖直到释放高度，读取静止反馈后额外停留0.2秒，再释放。

所有途径点和连续段都检查IK、现有关节与桌面间隙。通行下限由漏斗高度、球半径和留空量推导，只约束抓放横移阶段，不改变坐标验证页的用户选点范围。默认通行目标约100mm离桌；这是抓放路径参数，不是最低可抓取高度。执行时竖直段锚定最新反馈XY，保留名义与实际指令目标；实测仍低于横移所需间隙时，不发送贴桌斜线。

34mm球的定位值是球心。规划明确保存球顶、期望夹持中心和固定指尖目标，沿用当前实验指尖偏移19mm。先在球顶上方留空再下降，让球进入两指之间，不把球心裸值直接当指尖目标。漏斗释放时的期望球心在开口中心上方，球底留出release_clearance_mm；固定指尖仍单独计算偏移。正常释放没有下探到漏斗内部的动作。

真实工具外形、舵机挠曲和摩擦仍需现场核对；软件几何与模拟通过不等于已验证任意障碍碰撞。录像保持10秒，分析与官方回位单独计时；不能为赶10秒合并成贴桌斜线。路径与实际用时、未完成原因均保留。

坐标验证页也默认发送 `path_mode=lift`，先原XY抬高，经目标上方再竖直到点；三维预览显示完整折线路径。每段只发一次T104，等待反馈后再继续；撤销授权阻止后续途径点，传输不确定不重发。路径失败保留已执行的段与反馈。状态 `coordinate_api_version=6`。底层未指定path_mode的原单段API保留兼容，网页使用新分段模式；目标本身仍可选桌面以上任意几何可达高度，无新增25mm目标门槛。

## 两层 Agent

在本机项目目录重开OpenCode，选择主Agent `roarm-rsi`。它与只读子Agent `roarm-vision` 均复用 `bottlerocket/grok-4.7-build` 登录，无需另配模型API。

- 主OpenCode Agent：保存任务、标定、相机/机械臂视角关系、历史策略与经验；做方向转换、选择数值试验、调用现有控制服务。
- Grok视觉子Agent：只收到实际图片、阶段和可比前轮的图像证据；仅能读取项目证据目录的JPEG，没有执行、代码修改、网络或继续派任务的权限。看图给定性结论，建议也按原图方向，不需要知道机械臂摆放、XYZ、外参或数值策略。

`vision PLAN_ID` 生成图片包并下载到项目 `.evoarm-agent-evidence/`，主Agent把返回的 `vision_task` 原样交给Task工具的 `roarm-vision`。每轮用新任务，不传主上下文。场景包返回ready/note；结果包返回结果、观察偏向、建议类别、置信度、画面依据及前后改善判断。缺图和遮挡用unknown。

主Agent维护面对面视角的左右对应，再用标定工具计算XYZ。验收标准始终是：Grok建议画面左移，实际录像中的夹爪就应在画面向左靠近目标。画面上下映射到桌面平面，抬升/下探独立控制高度；已转换的机器人XYZ不额外手写全局Y反号。

## 一轮工具调用

```text
context → prepare → vision（交只读子Agent检查场景）
→ run（附场景图片包与ready结论）→ wait
→ vision（交只读子Agent检查本轮及可比前轮实帧）
→ decision（主Agent选一个参数试验）→ remember
→ 失败时reset/wait → 下一轮context/prepare
```

`run` 使用 `--execute --vision-packet-id <场景包编号> --scene-json '<子Agent场景JSON>'`。原有网页直接API自动循环保留，使用同一个新路径；与OpenCode编排选一种启动。

`decision --json` 示例：

```json
{
  "plan_id":"本轮32位ID",
  "vision_packet_id":"本轮16位结果包编号",
  "visual_review":{
    "outcome":"grasp_missed","direction":"image_right","ball_state":"table",
    "magnitude":"medium","adjustment":"pickup_xy","confidence":0.8,"note":"闭爪在球画面右侧，提升后球仍在桌面",
    "comparison":{"reference_plan_id":null,"effect":"not_compared","movement":"unknown","note":"首轮"}
  },
  "requested_step_mm":12,
  "reason":"选择12mm可见试验，方向转换后通过标定工具生成下一轮参数；不是模型测量误差"
}
```

也可用 `parameter_trial:{"parameter":"grasp_z_offset_mm","value":0}` 替代XY步长。每轮只改一个参数；可选夹持高度、指尖偏移、张闭角、预紧、速度、投放间隙，需与实际失败类型对应，并通过现有参数和完整路径检查。视觉结论原样保存，不得由主Agent改写；图片包绑定本轮实际录像和标定。相机视角关系只给主Agent，直接API视觉请求也不再传机器人坐标/策略作为判断上下文。

已保存的主Agent经验每轮读取；go按已授权任务继续，未另指定时最多3轮、成功即停。画面判断、主Agent理由、实际参数进入本轮归档和网页学习记录；浏览器过程流展示阶段与结论，子Agent原生生成过程在OpenCode中可见。ZIP、原视频/RGB-D与Langfuse待同步出口沿用。
