# RSI v9：现场失败重放、独立投放姿态与真实启动入口

本文记录v9修复。后续三轮执行超时、读图权限及未形成学习闭环的修复见 [RSI v10](rsi-execution-learning-v10.md)。

本版针对2026-11-14上传的诊断包修复。RSI接口版本9，路径版本3；坐标页接口仍为6。沿用当前活动标定、尺寸、模型登录、独立相机服务和原Gemini Python。更新验收不执行实物运动。

## 这次失败发生在哪

两次录像经原定位代码重放，球心约为 `(370.8,-53.0,-104.2)` mm，漏斗口约为 `(366.6,274.1,-72.5)` mm。失败时的静止反馈约为 `(358.9,-6.1,187.5)` mm，俯仰5.89°。这些与稍后采集的当前姿态不同；文字摘要里的近基座XYZ、水平桌面也不是包内原始API返回值。本次按实际录像、标定与恢复前静止反馈重放，不据摘要覆盖标定。

原路径在抓起后的transport段继续强制80°俯仰，到漏斗上方需要约448mm腕部连杆距离，超出该姿态的几何范围。496.2mm、388.4mm则是其他候选在起点高处转腕失败，并非同一个球坐标的误差。原错误只拼接字符串，丢失了阶段和XYZ。

现在独立搜索运输/投放俯仰，逐段检查完整XYZ与姿态插值。这两段录像均能保留80°抓取、采用50°运输与投放。投放指尖目标随俯仰重新计算，避免改了角度却仍用旧指尖坐标，最终夹持中心继续位于漏斗口上方。高处转移、竖直下探/提升、球径与指尖补偿、停稳释放均保留。

没有扩大关节范围或取消桌面检查。搜索失败明确标为候选路径未通过，附阶段、XYZ、俯仰和原因，不等同于目标物体绝对不可达。失败前先保存planning-input.json和calibration-snapshot.json；完整失败备份也包含它们。

## OpenCode 1.19.0 入口

此前只有agents文件，没有同名command。现在实际注册 `.opencode/commands/roarm-rsi.md`，选择主Agent roarm-rsi、模型bottlerocket/grok-4.7-build，subtask=false。只读子Agent仍为roarm-vision。

在运行服务的同一个项目目录重开OpenCode：

```text
/roarm-rsi
```

无参数只读取readiness和context，不连接设备、不复位、不开始抓放。原来选择的是Sisyphus/Build时，此命令会切换到项目定义的roarm-rsi。`opencode agent list`应列出roarm-rsi(primary)、roarm-vision(subagent)。如同名命令被本机插件覆盖，先核对当前项目与Agent列表；也可从主Agent选择器选择roarm-rsi。不是另装一个skill。

本次在隔离目录使用官方OpenCode 1.19.0二进制验证：agent list枚举两项，debug config中的command.roarm-rsi实际绑定正确的agent/model/subtask。没有调用公司模型或连接硬件；本机插件对会话选择的影响仍需现场核对。

## armed=false 正常

新工具 `python -m evoarm_rsi.agent_tools readiness` 读取本次反馈，返回控制服务实际代码目录与提交、客户端目录、连接、六关节状态、只读开关、标定以及明确阻止准备的原因。连接已存在时不要求重连；读取失败不拿旧health缓存冒充新反馈。

用户授权一个抓放任务，与内部临时armed标志分开。待机armed=false、0秒不阻止prepare；run会在完整检查后调用现有unlock_once取得临时授权。没有网页独立解锁按钮。motion_enabled=false才表示后端只读模式。readiness本身不建立连接、不授权或发送运动。

更新后先运行 `/roarm-rsi` 做只读核对。现场准备就绪且用户授权后，用 `/roarm-rsi go` 进入最多3轮、成功即停的闭环。每轮重读当前状态和经验，读取新场景，再交视觉子Agent检查；不执行诊断包里的旧坐标。网页直接API循环仍为另一个启动方式，不与OpenCode同时启动。

## 验证范围

上传两段RGB-D已在云端用原定位流程重放；数值提取为tests/fixtures/reported_rsi_layout.json，覆盖两次失败时起点和后来的坐标页姿态。回归检查逐段IK、官方关节范围、桌面间隙以及不同俯仰下的投放中心。另验证armed=false时run自动取得授权、readiness读取失败/断开不虚报就绪、失败规划输入持久化。

这证明已修复的这组输入能生成经过检查的路径，不证明实物抓放成功或已达到10秒。机械挠曲、抓球接触和视频判断还需现场执行后评估，继续使用逐轮录像与记录。
