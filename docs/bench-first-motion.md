# bench-layout-v2：标定后的第一次真实自动动作

五点已经完成，不重采，不使用Home/Hover/Grasp/Place。新的场景规划器尚未接通完整抓放执行器；旧 `grasp_run` 依赖旧示教，不能用于本布置。
本入口执行抓取准备中的首个离桌动作，记录 `grasp_attempted=false`、`placement_attempted=false`，不计作抓取成功。

## 转发给本地 OpenCode

只运行已有程序与操作设备，不开发代码。用户已要求继续实物测试，在现场看护，人手退出工作区；连接串口可能触发初始化，先检查整臂活动范围。保持相机、棋盘、底座固定，保留全部标定与本地改动。

1. 更新独立worktree的 `feature/rsi-grok-pick-place-10s`。先停止旧网页服务，从新版worktree用普通用户启动：

   ```bash
   EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=1 EVOARM_SETUP_ID=bench-layout-v2 \
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
   ```

   只有一个串口所有者，不发送扭矩切换或复位命令。
2. 停止旧相机服务，从同一新版worktree启动一个：

   ```bash
   sudo /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.camera_service serve
   ```

   普通用户检查 `python -m evoarm_rsi.camera_service health`，须有 `recording_request_ids=true`。这让动作能确认录制的是它自己的10秒录像；旧服务缺少该字段时不会发运动。
3. 通过已有服务连接机械臂：

   ```bash
   curl --fail-with-body --silent --show-error -X POST http://127.0.0.1:8765/api/auto-connect
   ```

   确认 `connected=true`，再读取 `/api/status`、`/api/health`：新鲜T1051、六关节扭矩均为1、fault/pending为空。无法唯一确定串口时回传 `/api/ports`，不猜测或反复重连。连接后姿态改变时，以新反馈为准。
   获取 `/api/bench/live-frame?after=0` 保存当前JPEG，检查夹爪周围的短程抬起走廊无遮挡。
4. 从同一新版worktree运行只读预测：

   ```bash
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.bench_start > /tmp/evoarm-first-lift-plan.json
   ```

   它使用本次五点与当前反馈，不覆盖标定。上次姿态预计J3肘关节减5°、末端升高约14mm、水平移动约20mm；实际以本次预测为准。预测拒绝时回传错误及反馈，不改阈值、不复位、不换旧示教。
5. 现场看护就绪、预测通过后，给现有控制器一次授权并执行一次动作，不再要求用户标定或操作旧示教界面：

   ```bash
   curl --fail-with-body --silent --show-error -X POST http://127.0.0.1:8765/api/unlock-once \
     -H 'Content-Type: application/json' --data '{"declaration":"已检查设备并清空运动区域"}'
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.bench_start --execute > /tmp/evoarm-first-lift-result.json
   ```

   先确认授权成功再执行。程序等本次10秒RGB-D录像开始后，只通过既有 `/api/step` 发一次J3 -5°。不下发其余关节目标、不复位、反向返回或重发。反馈需证明实际抬起，到位或出错后撤销运动授权，扭矩保持不变。
   启动拒绝时调用 `/api/disarm` 撤销本轮授权并回传原因；写入运动意图后失败时不重跑、不删 `first-motion/lift-attempt.json`，避免响应丢失后的重复运动。
6. 回传 `/tmp/evoarm-first-lift-result.json`、当前JPEG、实际服务提交；打包 `capture_trial_id` 对应录像目录，含 `pregrasp-lift.json`、RGB、深度和时间戳。另回传 `calibrations/bench-layout-v2/first-motion/latest-report.json`。
   顺带确认OpenCode自身是否能通过公司Grok分析图片，或只是项目环境变量未配置；只回传可用状态和非秘密的模型名，不贴密钥。

下一步根据本次上电后的实际位置和图像继续接入张爪、接近、闭爪提升及当前漏斗投放。已有五点继续使用，本动作反馈不是新示教。

## 范围

官方FK采用20260701默认edge-tool尺寸，与当前固件XYZ/Pitch交叉核对。它不等于完整抓取中心标定或任意路径的碰撞验证。
10秒录像须覆盖动作及到位反馈；分别记录动作耗时、录像跨度和覆盖结果。完整抓放10秒目标尚无实测成绩，不把准备动作计作完整试次。
