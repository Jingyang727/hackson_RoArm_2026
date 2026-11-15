# RSI v7：完整修正向量与 OpenCode 执行 Agent

在 `feature/rsi-pick-place-loop-v1` 更新，状态接口 `api_version=7`。只重启普通用户的 8765 服务；保留原 Gemini Python、独立相机服务、标定和试次。网页现有直接 API 自动循环仍可用；它不会自动启动 OpenCode。要使用本次新增的执行 Agent，在项目目录打开本机 OpenCode，选 `roarm-rsi`，无需额外配置模型 API。使用已有 `bottlerocket/grok-4.7-build` 登录，先激活原 Gemini Python，确认 `python` 能导入本项目。

## 修复和旧数据

上传会话中申请 8mm 的修正，被旧版 XY 各轴 ±30mm 裁剪成约 1.389mm，像素位移偏离预期约 80°。此前检查只看到微小的同向分量就放行，导致继续误学。

现在移除这项补偿裁剪，使用当前补偿位置的相机投影 Jacobian 计算完整 XY 向量。保存前同时核对向量长度和投影方向，不能再把申请 8mm、实际约 1.4mm 记作有效修正。未变好不会标为“已验证改善”。自动算法初始建议 4/8/12mm；有证据后增减，自动单次最多 16mm。执行 Agent 可明确选择其他正数试验步长，完整路径必须通过已有 IK、官方关节与桌面检查。这些建议不是用户坐标工作区限制。

首次显式 `prepare` 时识别旧版饱和且被截短的学习参数，先备份完整策略，再仅重置受影响 XY 补偿到有成功证据的基线或零。保留其他参数、原始 reviews 和所有标定，不伪造新试次。`baseline_resets` 保留来源、版本、前后值及备份路径；新计划核对这条修复谱系。只读查询不触发此重置。

## 图像方向与机器人坐标

`direction` 是夹爪相对目标的观察偏差，原图偏右 → 纠正向左，偏上 → 纠正向下。`correction_direction` 可显式提交，程序验证二者相反。如果原自然语言已给纠正建议，就不再取反。机器人 XYZ 由保存标定计算，不能额外镜像一次。坐标页的镜像按钮只改显示。

## 工具与现场启动

项目 Agent 定义在 [.opencode/agents/roarm-rsi.md](../.opencode/agents/roarm-rsi.md)，按 OpenCode 项目 Markdown Agent 配置加载。它有读取图片与执行工具的能力，禁用代码编辑；不是把 OpenCode 当作无工具的模型代理。

可向本地 OpenCode 转发：

```text
更新并从目标 worktree 重启普通用户网页服务，沿用相机与标定。只读确认 /api/bench/rsi/state 的 api_version=7。
在激活原 Gemini Python 的该项目目录重开 OpenCode，选择 roarm-rsi Agent。
先执行 python -m evoarm_rsi.agent_tools context；用 observe 新采当前球心、漏斗口 XYZ 与带标记快照，勿复用旧试次坐标。
本次更新验收先不执行抓放、复位或扭矩切换，等用户在现场开始任务。
```

随后用户可对该 Agent 说：“现场已清空，允许抓放、失败后恢复官方姿态并连续试最多 3 轮。先看实际画面，再执行，逐轮说明变化。”现有授权范围内无需每个子动作重新询问。

```bash
python -m evoarm_rsi.agent_tools context
python -m evoarm_rsi.agent_tools observe
python -m evoarm_rsi.agent_tools prepare
python -m evoarm_rsi.agent_tools objects
python -m evoarm_rsi.agent_tools evidence <plan_id>
python -m evoarm_rsi.agent_tools run <plan_id> --execute
python -m evoarm_rsi.agent_tools wait
python -m evoarm_rsi.agent_tools evidence <plan_id>
```

`review --json` 的示例（内容必须来自本轮实际观察，不能照抄判定）：

```json
{
  "plan_id": "本轮32位ID",
  "outcome": "grasp_missed",
  "direction": "image_right",
  "correction_direction": "image_left",
  "magnitude": "medium",
  "adjustment": "pickup_xy",
  "requested_step_mm": 12,
  "note": "看过闭爪和提升帧，球仍在桌面，夹爪在原图右侧；选择12mm可见试验，不是测量误差。"
}
```

可用 `review --file 文件`；Agent 禁止编辑文件时使用内联 JSON。若 context 的 comparison.available=true，必须对照实际图片，附上 `comparison`：reference_plan_id、effect、movement、note，含义与 v6 相同。每个真实试次只记账一次，过期状态/参数不会重复应用。

持久策略沿用 `~/.evoarm/rsi/pick_place_policies/<setup>/<标定指纹>.json`；Agent 经验在 `~/.evoarm/rsi/agent_memory/<setup>/<标定指纹>.json`，每轮由 context 重载。供 Agent 读取的图片副本在项目 `.evoarm-agent-evidence/<试次>/`（已加入 gitignore，避免跨工作目录读取图片反复询问），原始图片继续保存在试次备份。自然语言笔记不能覆盖工具方向协议；不能保证模型永不犯错，所以执行边界仍检查几何和参数谱系。网页进度显示实际动作；OpenCode 的决策文字显示在其会话，保存后的 review 会进入网页学习记录。

## 坐标来源与手动验证

当前执行路径的球/漏斗 XYZ 不由 Grok 输出。球：OpenCV 黄色区域、对齐深度的已知半径球面拟合、相机到机器人标定；缺深度时明确回退桌面与球径。漏斗：OpenCV 黑色开口轮廓、标定桌面、实测高度 45.4mm；黑洞中心的深度不当作开口高度。

`objects` 返回最后一次观察的物体中心、来源、深度证据、试次状态与有效期，不冒充实时定位。上传记录最后一轮球心 `(338.477,57.195,-100.466)`、漏斗口中心 `(382.549,-232.479,-71.716)` mm，只适用于那一轮。球心在实体内部；固件 XYZ 指固定指尖，不是张爪中心。手动验证先在已有坐标页预览物体上方目标；上方不是自动保证可达，尤其漏斗远端需检查 IK。

目前备份只有会话 state/events，没有实际录像与完整单轮 dispatch。完整向量 bug 可由数据与回归测试复现；实物运输方向还需对照新预览/录像，不能用软件投影自洽宣布已经现场纠正。

## 坐标验收必须使用同一摆放

用户随后报告用输入 `(338.48,-70,-70)` 触碰球、`(300,306,-40)` 触碰漏斗，随后明确：物体已经不在旧录像原位。因此，这些数值不能与旧试次 XYZ 相减来认定镜像、偏差或标定失败；也不能据此拟合、覆盖标定。相机/底座/桌面固定，并不代表球和漏斗固定。

两条坐标记录 `d7152c840bb1441baf1043cba5dcaab7`、`6d3b5c8cc9a14b33a91659ed4d04fe75` 的正 Y 输入与正 Y 反馈一致，没有发现运动接口反号。已报告的约8–20mm实际到位偏差仍是真实执行问题，应由同一时刻、同一物体摆放与明确工具接触点的测试来量化。

用抓取试验页工作区下面的“采集当前物体坐标（不运动）”→“重新定位黄球和漏斗”，或 `python -m evoarm_rsi.agent_tools observe`。此入口只采2秒当前 RGB-D，不读取机械臂反馈、不规划抓放、不下发运动，也不改变策略；不依赖机械臂连接、扭矩、公司 API 或完整抓放路径可达。返回带标记快照、采集时间、试次ID、当前球心与漏斗口坐标，以及标定指纹。失败时不沿用旧坐标。先看图确认识别的是正确物体，再保持物体原位，到现有坐标页预览/验证。

`coordinate-evidence` 只读保存的坐标测试输入/反馈/俯仰与五点记录。球心与表面触点、漏斗开口中心与边沿、固定指尖与张爪中心都需分开。不要用粗略的两组表面触碰重建完整三维外参。当前矩阵可复算旧轮漏斗口，五点射线回算误差约1.4–6.4mm；这只是存档内部一致性，现场定位仍需本次新快照验证。
