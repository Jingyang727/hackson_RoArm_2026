# Qwen 视觉、大口沿定位与20秒录像

基线：成功抓起过球的代码 `b1d3766` 已合入 main；PR #12 将当前标定和匹配的经验策略 v17 合入 main `9528c9d`。优化从该 main 新建 `feature/rsi-qwen-rgbd-20s`，没有覆盖基线。

## 模型与策略的分工

- **Qwen3.8-27B-FP8** 接收实际图片，输出抓住与否、球在桌面/夹爪/漏斗、夹爪在目标的画面哪侧，以及定性的调整类别。
- **程序**依据相机到机械臂的标定，把画面方向转换为机器人 XY 修正，并选择单项试验参数，累计保存到对应标定的策略文件。
- 程序继续使用已验收的 T104 XYZ、T106 开合、T1051 反馈和 T100 恢复链。Qwen 不生成绝对 XYZ，也不直接发运动指令；当前没有第二个决策大模型，因此不需要 DeepSeek 来改坐标。
- 图像不镜像。画面方向通过标定换算，不向所有机器人指令额外施加 Y 取反。

内部 `grok-*` 文件名、`company_grok` 来源名及事件名保留，便于读取历史记录。网页显示“视觉模型”和实际模型 ID。

## 本机切换 API 和 token

使用用户给出的 OpenAI-compatible Chat Completions 地址：

```text
https://inference-bottlerocket.tesla.cn/v1
```

配置模板为 [`config/company_llm.brcn.env.example`](../config/company_llm.brcn.env.example)。首次复制成 `~/.evoarm/company_llm.brcn.env`，在本机填入 `BRCN_9901_API_KEY`；也支持 `BRCN_API_KEY`。不要将实际 token 放进仓库、指令输出或聊天。保留原 `company_llm.env` 便于切回，服务启动时明确设置：

```bash
export EVOARM_COMPANY_LLM_ENV_FILE="$HOME/.evoarm/company_llm.brcn.env"
/Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.company_api
```

模板模型为 `Qwen3.8-27B-FP8`，关闭 thinking，temperature=0.2。模型、地址、token 都来自本机配置，可以再换。进程中已有的 `EVOARM_COMPANY_LLM_BASE_URL`、`EVOARM_COMPANY_LLM_TOKEN`、`EVOARM_COMPANY_LLM_MODEL` 优先于文件，切换时应检查旧进程启动环境；不要输出 token 值。新网关不会自动继承 US OpenCode 登录 token。

图像是 JPEG data-URI `image_url`，一次请求传全部时序帧。直接连接网关，不使用环境代理，不把凭据跟随 HTTP 重定向发送。流式回复继续展示；只在网关明确拒绝 stream 参数时改用一次非流式请求。认证失败不重试。

上述配置检查只确认配置完整，不能证明该模型真的支持图片。部署后可先用已有工作区预览图做一次只读识图：

```bash
/Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.company_api \
  --probe-image /absolute/path/to/workspace-preview.jpg
```

该命令只读图片、调用选定视觉 API、保存请求证据和结果；不连接或移动机械臂。公司网关连通性和该模型实际多图能力需在本机验证。

服务仍使用现有 Python、端口和相机进程，例如在当前 worktree：

```bash
EVOARM_COMPANY_LLM_ENV_FILE="$HOME/.evoarm/company_llm.brcn.env" \
EVOARM_ENABLE_MOTION=1 EVOARM_ENABLE_TEACH=1 \
/Users/jyang51/miniconda3/envs/evoarm/bin/python -m uvicorn control_ui.main:app \
  --host 127.0.0.1 --port 8765
```

只重启已确认属于本项目的旧网页进程，避免同时占用端口或串口。相机 `/record` 已支持20秒，继续沿用当前独立、可能需要 sudo 的 Gemini 相机服务。

## 漏斗大开口

旧版使用整块黑色轮廓的重心，底座和耳片会拉偏中心；前端还用固定半径画位置圈。

现在把可见轮廓反投影到标定的口沿平面，在毫米坐标中稳健拟合大圆，使用76mm开口内径约束候选尺寸。小颈口不作为大开口；显示的是投影后的76mm可用开口轮廓和圆心。

对同一 RGB 帧配对的深度，只取口沿附近、与45.4mm测量高度一致的样本。深度必须覆盖至少半圈，并通过半径、残差和高度分布检查，才用来修正中心与高度。孔内、桌面和底座深度不当作口沿高度；黑色表面无可靠深度时，明确回退到 RGB 口沿 + 保存桌面 + 实测高度。

运输路径使用的漏斗障碍高度至少保留实测45.4mm，避免深度采到略低的内沿后降低通行间隙。

`scene.funnel` 保存 `mouth_boundary_px`、`rim_fit`、`depth_evidence`，网页诊断显示是否实际用了深度。自动投放目标仍是**球心在大开口中心上方**；固件 XYZ 是指尖坐标，因此会按姿态扣除指尖到球心的偏移，不能直接把球心 XYZ 当指尖 XYZ。

## 录像和模型图片

- 每轮完整 RGB-D 录像 **20秒**。停止发送后续动作的窗口改为 **19.6秒**；10秒保留为性能目标及 `within_10s` 指标，超过10秒不会因此截掉释放阶段。
- 定位、深度与标定继续用原始 **1280×720** profile；相机未降分辨率。
- 只缩小模型输入副本，含方向说明边框的最终图片**长边最多768px**，JPEG quality=82。常见全景从1280×720变为768×432内容，加边框后为768×550；局部对照图保留清晰细节。
- 场景检查通常1–2张。结果分析一次最多**6张**，优先闭爪、抬升、释放、前/末帧。对照时使用1张前轮对应动作图，加最多5张本轮图；恢复后的补充图也占用同一预算。
- `model-frames/raw-*` 保留原始证据副本；`scene/review-*` 是实际发给模型的压缩副本，记录尺寸、缩放、质量。`vision-*-metrics.json` / `vision-metrics.json` 记录准备耗时、输入大小、首段文本耗时和 API 总耗时。

## 本机验收

1. 打开 `/experiments`，确认页面显示选定 Qwen 模型；`/api/bench/rsi/state` 应含 `visual_pipeline_version=2`、`recording_duration_s=20`、`model_long_edge_px=768`。
2. 先点“重新定位黄球和漏斗”，不运动，检查大开口轮廓、圆心与深度来源。物体保持原位。
3. 再从网页执行一轮，确认释放阶段、20秒录像和模型结论；比较实际模型图片与时序，不凭 planned 路径判断已执行。
4. 新策略仍按原标定指纹保存，原标定文件不改写。归档的 v17 没有对应已确认成功试次，见 [标定快照说明](../calibrations/bench-layout-v2/2026-11-15/README.md)。

离线与历史 RGB-D 回放能检验几何、时序与协议；它们不替代当前布局的实物投放验收。
