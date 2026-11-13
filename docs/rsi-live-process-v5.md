# RSI v5：实时过程与 Grok 公开回复

分支 `feature/rsi-pick-place-loop-v1`；`GET /api/bench/rsi/state` 返回 `api_version=5`。相机、标定、策略和机械臂执行接口继续沿用。

## 本地更新

```text
更新正在运行的 worktree 的 feature/rsi-pick-place-loop-v1。
使用原 Gemini Python；只重启普通用户 8765 网页服务，保留现有相机服务和 ~/.evoarm/rsi 数据。
保持 EVOARM_ENABLE_MOTION=1、EVOARM_ENABLE_TEACH=1、EVOARM_SETUP_ID=bench-layout-v2。
只读确认 /api/bench/rsi/state 的 api_version=5、grok.backend=company_api、grok.configured=true。
打开 /experiments，页面加载和实时过程连接不应触发抓放或复位；由用户点击开始。
不输出 company_llm.env 或 auth.json 的内容。
```

公司 API 配置与 v4 相同：优先显式环境变量，其次 `~/.evoarm/company_llm.env`，再复用已有用户级 provider/auth 配置，默认公司地址 `https://inference.tesla.com/v1`，模型 `grok-4.7-build`。不运行 OpenCode CLI，不新增相机或串口所有者。

## Grok 返回什么

开始前的场景检查返回 `ready` 和画面说明。录像复核返回：

| 字段 | 含义 |
| --- | --- |
| `outcome` | 成功入漏斗、空抓、推走球、滑落、投放偏移、弹出、执行失败或无法判断 |
| `ball_state` | 球在桌面、夹爪、漏斗中，或无法确认 |
| `direction` | 夹爪相对目标在原始画面的左、右、上、下，或抓取／投放偏高、偏低 |
| `magnitude` | 小、中、大或不确定的粗略偏差 |
| `adjustment` | 保留策略，或建议调整抓取 XY、高度、夹持预紧、运输速度、投放 XY／高度 |
| `confidence` | 模型自报置信度，并非实测成功率 |
| `note` | 可见画面依据 |

方向仍是**观察到夹爪偏在哪里**，不是机器人运动指令。程序用标定计算反向修正与有界试验参数，并在下一轮计划中核对它们确实生效。Grok 不提供绝对毫米误差或直接控制机械臂。

提示词现在要求先用一两句中文说明可见结果与定性调整建议，再返回唯一结果 JSON。界面实时显示这段公开回复；完整公开文本可展开。只读取模型的 `content`，不展示或保存 `reasoning_content`、工具调用等其他字段。

## 页面如何看

“抓取试验”页增加 **RSI 过程**，按轮次显示观察、规划、下降、合抓、提升、运输、释放、录像分析、程序修正和回位／重试等阶段。动作阶段表示执行进度，物理成功仍由录像判断；控制器到位不能证明已夹住球。

分析时会显示“Grok 正在流式回复，本轮结论尚未确认”。例如：

```text
第 1 轮：下降 → 合抓 → 提升 → 分析录像
Grok：球仍在桌面，夹爪在球的画面右侧合拢，建议向画面左侧修正。
程序采用的修正：将画面左移经标定转换为机械臂 XY 增量，保存参数 v2。
恢复官方姿态 → 第 2 轮重新定位 → 核对并使用 v2 → 再次尝试。
```

右侧结论同时列出结果、球的位置、观察方向、粗略偏差、建议类别、模型自报置信度和画面依据。“本轮参数变化”仍可查看数值前后对照。

**未完成或未通过格式校验的回复只显示，不更新策略。** 只有完整回复通过既有校验后，才计算、记录并应用下一轮经验参数。断流、格式错误、停止等情况沿用现有恢复与停止机制。

## 流式接口与兼容

后端向公司 Chat Completions API 请求 `stream=true`，逐个转发公开文本片段。浏览器通过只读 `GET /api/bench/rsi/learning/events` 接收 SSE，事件带会话 ID、序号、轮次和时间。首次连接发送快照；重连使用事件游标补发，缓冲过期则恢复快照。刷新或重连只恢复显示，不启动任务、不重发机械臂指令。浏览器断开不等于用户点击“停止”。

页面保留状态刷新作为显示兜底。若服务返回完整 JSON，则明确显示等待完整回复，不模拟逐字输出。只有接口明确以 HTTP 400／422 拒绝 `stream` 参数时，才改用非流式请求一次；认证失败、其他 HTTP 错误及中途断流不自动重试。也可在本机设置 `EVOARM_COMPANY_LLM_STREAM=0` 使用完整回复模式。

取消后，迟到的网络片段不再进入显示、策略或控制流程。公开事件日志位于 `~/.evoarm/rsi/learning_sessions/<session_id>/events.jsonl`，包含在既有任务诊断 ZIP 中。各试次保存 `grok-scene-output.txt` / `grok-output.txt` 公开回复，与原录像、RGB-D、校验后的 Grok 结果、策略变化及 Langfuse 待同步记录一起保留。

自动测试覆盖流式片段在 HTTP 结束前到达、中文片段、忽略 reasoning 字段、断流不产生完成结果、停止后的迟到片段、非流式兼容、SSE 游标补发及刷新不重放动作。浏览器验收使用模拟设备与模型；公司接口是否实时吐出片段，需要本机实测，页面会按实际返回方式显示。
