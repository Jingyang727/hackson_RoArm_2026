# RSI v4：相机方向、参数修正与实际应用记录

> 此文记录 v4。当前实时过程展示与更新步骤见 [RSI v5](rsi-live-process-v5.md)。

分支 `feature/rsi-pick-place-loop-v1`，`GET /api/bench/rsi/state` 的 `api_version=4`。沿用已保存标定、同一个机械臂控制器和独立相机服务。

## 转交本地 OpenCode

```text
更新当前运行 worktree 的 feature/rsi-pick-place-loop-v1。
保留 ~/.evoarm/rsi/ 下的标定、策略和试次，保留现有独立相机服务。
使用原 Gemini Python 安装 requirements.txt 与 requirements-vision.txt，只重启普通用户 8765 网页服务。
保留 EVOARM_ENABLE_MOTION=1、EVOARM_ENABLE_TEACH=1、EVOARM_SETUP_ID=bench-layout-v2。
只读检查 /api/bench/rsi/state：api_version=4、grok.backend=company_api、grok.configured=true。
如未配置，用原 Python 运行 python -m evoarm_rsi.company_api，只回传安全诊断 JSON，不输出凭据文件内容。
更新验收期间不发送运动、复位或扭矩指令。由用户打开 /experiments 启动自动任务。
```

模型仍默认 `grok-4.7-build`，直接调用公司 Chat Completions API。配置来源按优先顺序为：

1. 显式进程变量 `EVOARM_COMPANY_LLM_BASE_URL`、`EVOARM_COMPANY_LLM_TOKEN`、`EVOARM_COMPANY_LLM_MODEL`。
2. `~/.evoarm/company_llm.env`。支持上述变量以及 `OPENAI_BASE_URL` / `OPENAI_API_BASE`、`OPENAI_API_KEY` / `GROK_API_KEY`、`OPENAI_MODEL`。按字面值解析 `KEY=value`，不执行 shell、不展开命令、不修改环境或原文件。
3. 已有用户级 OpenCode provider 配置与 `~/.local/share/opencode/auth.json` 的 `bottlerocket` 凭据。只读取文件，不运行 OpenCode。已知公司默认地址为 `https://inference.tesla.com/v1`。

可通过 `EVOARM_COMPANY_LLM_ENV_FILE` 选择其他可信本地 env 文件。密钥不进入网页状态、模型上下文或试次备份。不自动向公共模型回退。

## 两种方向不能混用

Grok 的 `direction=image_right` 表示**夹爪在目标的原始画面右侧**，并不是建议向右移动。程序先选择反向画面修正，再通过相机标定计算机器人 XY 增量。原始 RGB 的 u 向右、v 向下；坐标页的镜像仅用于展示。

相机与机械臂相对摆放时，机器人 +Y 完全可能投影到画面左侧，画面上下也不能直接解释为机器人 Z 正负。不得因肉眼称呼“左/右”或某个 XYZ 正负号，就再翻转一次坐标轴。

用户提供的试次中，球像素为 `(574.1, 208.2)`，漏斗为 `(929.1, 199.5)`，因此识别出的漏斗在**画面右侧**。规划抓取与释放名义指尖的 Y 分别约 `+68.6`、`−224.0` mm。这个组合不能证明漏斗识别或运输方向错误。

用之前上传的标定离线计算，空抓报告 `image_right` 时的 2mm 试验修正是机器人 XY 约 `[-0.347, +1.970]` mm，投影约为画面 `[-2.387, +0.001]` 像素，即向画面左侧移动。这是计算核对，不是对当前实物运动的验收。

页面在实时图上圈出本轮观察时识别的黄球、漏斗，并显示漏斗在球的画面哪一侧。圆圈不是实时跟踪；球运动后可能离开原圈。原始观察另存为 `observation.jpg`，Grok 场景复核同时查看原图与标注图，不能把软件标签当作事实。

## 定位与学习分别做什么

黄球由 RGB 黄色区域定位；实际对齐的球面深度参与已知半径 17mm 的球心拟合，再转换为机器人 XYZ。深度不足时明确退回保存桌面与球径估计。黑色漏斗目前由暗色轮廓、圆形程度和尺寸筛选，口中心像素通过保存的桌面与 45.4mm 漏斗高度定位；孔内测到的深度不能当作口沿高度。多个暗色物体仍可能导致选错，圈出的目标和 Grok 的场景确认用于复核这个问题。

Grok 主要报告是否抓住、是否入漏斗、夹爪偏哪边和粗略程度。它不计算绝对机器人 XYZ，也不直接下发命令。校准几何负责换算；RSI 保存并修改经验参数。

每轮复核现在保存完整 `before` / `after`，以及 `proposal`：观察方向、反向修正方向、试验步长、机器人 XY 增量、相机模型预测的像素变化、改变的参数和比较原因。网页可展开查看参数变化；历史也显示每轮改了什么。

同一位置连续同向空抓，不再一直固定加 2mm：下一轮探索步长逐次增加 2mm，最多 8mm。方向反转时缩小上一轮步长，最低 1mm；粗略偏差明显增大时撤销上一轮该项调整，回到官方姿态后停止盲目叠加。这里的数值是学习试验参数，绝不是 Grok 测得的误差，也不是对用户 XYZ 选点新增限制。

比较旧轮次只使用相近物体位置的历史。每次只调整对应参数项，不翻转标定；补偿范围仍沿用已有策略。到参数边界而没有新变化，或候选补偿投影没有朝正确画面方向移动时，恢复后停止重复相同策略并记录原因。实际改善仍以随后录像为准。

新计划保存 `policy_application`，记录策略摘要、来源试次和参数核对结果。执行前再次核对当前策略、计划参数及摘要；执行收据也保存该记录，避免“记录已学习但动作仍用旧参数”。已有增加 `squeeze_deg` 的迁移兼容保留。

合抓、提升等事件与均匀采样落到同一帧时，保留全部标签；模型最多查看八张关键帧时优先保留合抓、提升、释放和异常，而不是只按时间均匀抽取。请求保存实际选中的帧索引和时间。

官方 T100 自动恢复、10 秒动作录像、本地 ZIP 备份与待同步 Langfuse 记录继续沿用。网页会识别旧版后端并提示重启，避免新静态页面连到旧进程。云端验收使用模拟设备和模型；真实公司接口、当前目标识别和抓放效果需在本机验证。

本次验证：428 项 Python 测试、12 项 Node 测试，以及 Chromium 桌面与 390px 移动布局验收通过。覆盖面对面相机的方向转换、连续同向错误增大步长、方向反转减步、回滚、不产生有效变化时回位、旧参数计划拒绝、关键帧事件保留、env 字面解析与凭据不出现在安全诊断中。
