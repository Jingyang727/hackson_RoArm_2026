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
先执行 python -m evoarm_rsi.agent_tools context 和 objects，回传球心、漏斗口 XYZ 与来源。
本次更新验收先不执行抓放、复位或扭矩切换，等用户在现场开始任务。
```

随后用户可对该 Agent 说：“现场已清空，允许抓放、失败后恢复官方姿态并连续试最多 3 轮。先看实际画面，再执行，逐轮说明变化。”现有授权范围内无需每个子动作重新询问。

```bash
python -m evoarm_rsi.agent_tools context
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

## 用户新增触碰实测：尚未解释的定位矛盾

用户随后报告用输入 `(338.48,-70,-70)` 触碰球、`(300,306,-40)` 触碰漏斗。它们相对上面的估计差值分别为 `(0,-127.19,+30.47)` 和 `(-82.55,+538.48,+31.72)` mm。只翻转估计 Y 后仍有约 12.81mm、73.52mm 的 Y 差，且漏斗 X 仍差 82.55mm，因此不据此实现一个全局 Y 取反。两处 Z 都约提高31mm；球心到球顶只有17mm，仍需核对实际接触部位、俯仰和指尖参考。

这些目前是操作输入与肉眼触碰报告，尚缺同次实际反馈。`python -m evoarm_rsi.agent_tools coordinate-evidence` 返回保存的输入/反馈/俯仰与当前五点记录，完全只读。先确认物体未移动、计划与当前标定指纹相同、命令与实际反馈一致，再诊断相机图像方向、外参或工具参考点。两组近似表面触碰不足以重建完整三维外参，不覆盖标定，也不宣称该定位矛盾已解决。

后续回传两条坐标记录：`d7152c840bb1441baf1043cba5dcaab7` 目标 `(300,306,-40)`，实际 `(307.73,293.88,-52.14)`；`6d3b5c8cc9a14b33a91659ed4d04fe75` 目标 `(382.55,306,-40)`，实际 `(391.78,285.93,-57.37)`，均 settled_with_offset。正 Y 输入与正 Y 反馈一致，没有反号证据。约8–20mm执行误差应与五百多毫米的视觉/接触位置矛盾分开调查；球的负 Y 输入尚无对应实测反馈。由两物体输入位置得到的相对向量两个分量均与估计相反，也要核对整体图像方向/角点对应，而非直接改一个轴。
