# 首轮实物抓取：观察、张爪、接近、闭爪、提升

`bench_start` 只是起步判断。`already_clear` 表示无需额外抬升，**接下来必须继续本页的 prepare → 看预览 → execute**，不能把起步检查当作抓取后结束。
本页会真正发送机械臂动作，执行一次抓取与约50mm提升，录制10秒RGB-D。此轮不包含漏斗投放；是否抓起由录像确认，不由编码器角度宣布。

## 给本机 OpenCode 的完整任务

用户已要求尽快实际尝试抓球。你是本机控制Agent，使用自身已批准的Grok分析图片，不开发代码。无需配置第二个HTTP模型网关，`company_grok_configured=false`不阻止本流程。不要在准备检查后结束任务或再次要求用户采五点。

1. 更新独立worktree的 `feature/rsi-grok-pick-place-10s`，保留本地改动、五点标定及所有录像。只重启普通用户网页控制服务；沿用已经支持录像ID的相机服务，不重启相机、不新开SDK进程。

   ```bash
   EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=1 EVOARM_SETUP_ID=bench-layout-v2 \
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
   ```

   串口只由此服务持有。先现场看护并检查可能初始化的活动范围；未连接时调用已有 `POST /api/auto-connect`，读取新鲜T1051，确认扭矩均为1、fault/pending为空。重连后使用实际新姿态，不要求用户手动复刻标定俯仰或触点。
2. 从同一新版worktree用普通用户运行：

   ```bash
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.grasp_once prepare --output /tmp/evoarm-grasp-plan.json
   ```

   这一步录2秒当前RGB-D、核对可见棋盘与尺度、识别稳定黄球并计算路径，不发运动。程序直接读取本次五点，不使用旧任务示教。球需完整位于棋盘内部角点围成的区域，作为首轮受限试验范围。
3. 用你的Grok图片能力读取返回的 `preview_file`，核对：识别的是当前34mm黄色球；末端安装为原装夹爪；图中下降十字是靠机器人一侧的固定指尖目标，夹爪开口应包住球；高位对齐、下降和提升的整臂扫过范围没有漏斗、支架、人手或其他障碍。
   这是带明确实验参数的首轮抓取，夹取偏移尚未全局验收，允许失败并留下数据。若识别或路径不符，回传预览和具体原因，不放宽限位、不猜角度、不复位。
4. 现场人手退出运动范围、可及时断电且路径核对通过后，在用户现有指令授权范围内继续执行，不要又停在 `already_clear` 或只读结果：

   ```bash
   curl --fail-with-body --silent --show-error -X POST http://127.0.0.1:8765/api/unlock-once \
     -H 'Content-Type: application/json' --data '{"declaration":"已检查设备并清空运动区域"}'
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.grasp_once execute --plan-file /tmp/evoarm-grasp-plan.json --output /tmp/evoarm-grasp-result.json
   ```

   必须确认授权成功。`execute` 才是真实执行入口：等待本次10秒录像开始后，高位张爪对齐、到球上方约50mm处悬停并核对上电后的实际位置，再短程下降、闭爪、提升约5cm。悬停前重新核对黄球位置，后续短程下降连续执行以允许正常夹爪遮挡；每段核对实际反馈，一条指令传输不确定时不会重发。普通手动限位、速度和故障锁存保持原样。
   正常闭爪到位或“稳定角度+负载变化”的接触候选仅允许继续一次观察提升，不证明抓取成功。接触候选需单独POST保持实测夹角；GET状态接口不会发送任何电机命令。
   计划3分钟后失效；如已失效，重新 prepare 并看新预览，不移动标定物。执行收据存在时不重跑同一个计划、不删除收据；出错后撤销运动授权，不自动回位或切扭矩。
5. 查看返回的 `keyframes_file` 和同目录 `evidence-*.jpg`，结合10秒RGB-D与反馈，用自身Grok判断黄球是否随夹爪离开桌面、是否滑落。模糊或遮挡时写无法确认，不能仅凭闭爪角度写成功。
   回传 `/tmp/evoarm-grasp-result.json`、该关键帧图及实际服务提交，打包 `capture_trial_id` 对应实验目录。目录含 `grasp-plan.json`、`grasp-trial.json`、`motion-report.json`、原视频、深度和逐帧时间戳。若失败，给出可见原因和下一轮一个小幅修正建议。

## 实验边界

本次采用34mm球径、默认原装edge-tool几何、实验性固定指尖偏移与夹角，保持原相机内参的未全局验证标志。通过可见棋盘、球径和深度一致性检查，只支持这次局部有现场看护的尝试，不声称完成任意空间TCP或全场景碰撞验收。
Cartesian速度沿用0.12；不为凑10秒提高速度或取消反馈。超时停止后续指令，已发出的固件运动不能由撤销授权立即中止。视频与动作时间分开实测，未完成或录像未覆盖时明确报告，不宣称达标。
试验记录页会显示实际闭爪尝试和动作结果；视觉抓取结果未经Grok/人工评估时保持未知。成功抓起后再接当前黑漏斗的投放路径。
