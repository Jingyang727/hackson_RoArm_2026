# 一次多图的RSI视觉复核

2026-11-14现场记录已经确认：计划567750da的视觉任务共6帧，子任务耗时128秒；6次文件读取单次275–452毫秒，首末读取跨度约31秒，不能将整个窗口当作磁盘读图时间。最后生成约50秒，348字符的JSON完整返回，step-finish.reason=stop，不是数据库或模型截断。官方复位及反馈确认实际7.16秒，不是30秒。这些是现场事实，不能用Agent的后续口述覆盖。

旧链路把图片路径发给视觉子Agent，再由模型选择read工具，可能多次往返。现在`vision`下载并校验本轮图片后，生成含逐行`@.evoarm-agent-evidence/.../frame-XX.jpg`引用的vision_task。主Agent将它原样传给Task；OpenCode 1.19.0在首次模型请求前展开全部引用，并把独立图片一起送给视觉模型。不是缩小为联系表，也不是逐帧调用模型。

保留本轮最多6帧，另有可比前轮时最多加2帧。每张图内写入序号、CURRENT/PREVIOUS、阶段与实际相对秒数，任务文字也保留时间清单。OpenCode内部文件解析并发执行，不依靠附件到达顺序判断时序。原始录像、原始图片、散列和绑定继续留存。任务元数据中的非图片`@`字符转义，避免误附加其他文件。

视觉子Agent无执行或读文件工具：直接看已附图，只返回一个短JSON。加载失败或看不清时保留unknown。主Agent继续负责上下文、执行诊断、数值试验与控制接口；不能把软件等待当成机械运动耗时，也不能把正常stop且完整的JSON当作截断后重开视觉任务。

## 模型可独立更换

项目根目录[opencode.json](../opencode.json)只配置视觉Agent：

```json
{"agent":{"roarm-vision":{"model":"bottlerocket/grok-4.7-build-fast"}}}
```

这是用户本轮同意试用的公司模型。主执行Agent仍用grok-4.7-build。将来修改这个配置值并重开OpenCode即可，不改Python、不重标定、不改公司凭据或API地址。fast的真实读图质量、公司服务排队及响应耗时仍须本地只读复核验证，离线测试不能证明它一定更快或判断一定正确。

## 验证与更新

官方1.19.0二进制配合本地模拟模型运行真实Task流程，确认首次视觉请求同时包含6张图片、0次视觉工具调用、完整JSON回传主Agent；项目路径包含空格也通过。只验证传输，不调用公司模型或接入硬件。可重复运行：

```text
python -m tests.opencode_vision_attachments --opencode /path/to/opencode --output /tmp/roarm-vision-check
```

更新普通网页服务和本地OpenCode使用的同一worktree后重开OpenCode。RSI API仍为10，路径版本仍为4，视觉包版本为2。网页服务继续使用原Gemini Python，显式保留EVOARM_ENABLE_MOTION=1、EVOARM_ENABLE_TEACH=1、EVOARM_SETUP_ID=bench-layout-v2；相机、标定、已学参数和试次保留。

已有完整视觉JSON时先提交对应decision，不重开该轮视觉任务。需要单独验收新模型时，可对已有失败轮次只读调用vision，将vision_task完整交给视觉子Agent：确认实际采用fast、首个视觉请求已有全部图片、没有read调用、JSON完整。也可以在下一轮场景检查中验收。不要为验收读图重放抓取或复位；继续原任务时只执行尚未完成的授权轮数。transport.prepare_elapsed_s只表示工具下载准备耗时，不是模型推理时间；模型请求和总任务耗时分别由OpenCode记录。
