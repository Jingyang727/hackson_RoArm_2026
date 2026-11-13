# RSI v3：直接公司Grok API与独立官方复位

> 此文记录 v3。当前版本及本地更新步骤见 [RSI v4](rsi-direction-policy-v4.md)。

当前分支`feature/rsi-pick-place-loop-v1`，状态接口版本3。相机、底座、桌面和已有标定继续沿用。

## 本地更新

```text
更新当前服务worktree的 feature/rsi-pick-place-loop-v1。
使用原Gemini Python安装 requirements.txt 与 requirements-vision.txt。
保留标定和独立相机服务，只重启普通用户8765网页服务。
保持 EVOARM_ENABLE_MOTION=1、EVOARM_ENABLE_TEACH=1、EVOARM_SETUP_ID=bench-layout-v2。
只读检查 /api/bench/rsi/state：api_version=3，grok.backend=company_api，grok.configured=true。
若配置不完整，用原Python运行 python -m evoarm_rsi.company_api，回传安全诊断JSON；不输出密钥。
由用户在 /experiments 点击开始；更新验收期间不要代发运动或扭矩切换。
```

模型默认仍为用户选定的`grok-4.7-build`。程序直接向公司接口发送Chat Completions HTTP请求，不运行OpenCode CLI、代理进程或工具Agent。

配置优先使用本机环境变量`EVOARM_COMPANY_LLM_BASE_URL`、`EVOARM_COMPANY_LLM_TOKEN`、`EVOARM_COMPANY_LLM_MODEL`。未显式配置时，只读复用现有`bottlerocket` provider 的`baseURL`、模型ID和本机凭据：支持OpenCode JSON/JSONC配置、环境引用、凭据文件引用及已有auth.json。只读取配置，不调用OpenCode。当前支持OpenAI兼容Chat Completions协议；不猜测其他私有SDK协议。

密钥不出现在状态接口、日志或备份中，不需要粘贴到聊天。接口地址要求HTTPS且不将凭据放入URL；禁止携带凭据自动跟随重定向。不回退到公共模型。停止任务后，迟到的HTTP回复不会启动动作。

Grok收到时序JPEG关键帧、时间和反馈上下文，返回结果、方向、粗略幅度及定性调整类别。忽略reasoning_content，只有经过校验的最终JSON能影响下一轮参数；请求中没有电机工具。

## 这次复位修正

旧版要求先保持当前俯仰抬升，再去漏斗处理可能夹持的球，最后才T100。近工作区边缘时，保持64.2°抬高可能要求约392mm连杆距离，超过模型约383.2mm可达上限，因此恢复本身也可能被IK卡住。另一个漏洞是新一轮prepare失败时，异常处理仍引用上一轮已恢复的试次，跳过了新异常的复位。

现在复位完全独立：

1. 确认同一控制器所有者、用户未停止，并读取三次新鲜的锁定、静止反馈。
2. 如果已经处于官方姿态，记录已到位，不重复发送。
3. 否则通过现有串口所有者发送一次官方`T100`，轮询确认。没有XYZ抬升，没有漏斗定位，不依赖标定或IK解。
4. 如夹爪可能仍夹着球，先在当前已清空的工作台位置张爪，再T100；下一轮重新定位球。不会为了回位再规划一条去漏斗的路径。

规划失败、执行失败或模型分析失败都使用**本次异常独立的请求ID**。运动错误会先复位，再等待Grok分析，不把机械臂留在下降姿态等待模型。规划失败会复位后重新定位、规划一次；若仍不可达，任务停止时也保持官方姿态，并显示实际几何原因。没有放宽真实IK或官方关节范围。

旧的软件故障名不会永久阻止复位；只要新鲜反馈已证明连接、扭矩和静止状态恢复，就可发一个新的官方复位请求。通信仍不可用、扭矩未确认、仍在运动、用户停止或T100本身传输不确定时，不叠加命令或重发T100。

页面增加“恢复官方初始姿态”，它不要求Grok已配置，仍使用同一现场范围确认。对应`POST /api/bench/rsi/learning/reset`，只接受declaration，不接受关节角、任意命令或XYZ。

## 记录与验收

原试次视频、深度、反馈、Grok判断、策略与ZIP备份继续保留。未分析的旧记录显示“未分析”，避免误以为后台仍在等待运行。

规划失败尚未生成抓放计划时，也会保存错误、当前图、已有RGB-D观察及独立T100收据，打包到`~/.evoarm/rsi/backups/session-<session_id>.zip`。网页“下载本次任务诊断”可下载；它不会被计作一次额外的物理抓球。每次T100的写入尝试、传输确认和反馈确认分别记录。

云端测试使用模拟机械臂、模拟公司HTTP服务；包括64.2°低位远端规划失败后直接T100再规划、上一轮已恢复后的新异常、无Grok配置的复位按钮、API多模态请求及取消后的迟到回复。真实公司接口和机械臂需在本机验收。
