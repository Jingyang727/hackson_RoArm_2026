# 首轮实物抓取：观察、张爪、接近、闭爪、提升

`bench_start` 只是起步判断。`already_clear` 表示无需额外抬升，**接下来必须继续本页的 prepare → 看预览 → execute**，不能把起步检查当作抓取后结束。
本页会真正发送机械臂动作，执行一次抓取与约50mm提升，录制10秒RGB-D。此轮不包含漏斗投放；是否抓起由录像确认，不由编码器角度宣布。

## 浏览器操作（推荐）

让OpenCode更新 `feature/rsi-grok-pick-place-10s` 并从该worktree重启**普通用户网页服务**：

```bash
EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=1 EVOARM_SETUP_ID=bench-layout-v2 \
/Users/jyang51/miniconda3/envs/evoarm/bin/python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

保留五点标定与现有相机服务。相机health应包含 `recording_request_ids: true`，无需为了网页按钮再开相机SDK或使用sudo启动网页。OpenCode完成更新、启动和接口检查即可，**这次不要替用户调用解锁、执行、释放扭矩或复位**。

用户打开 http://127.0.0.1:8765/experiments ：

1. 尚未连接时点击“连接机械臂”，在现场看护可能发生的串口初始化；已有连接无需重连。
2. 点击“1 · 观察并规划”。服务录2秒当前RGB-D，显示黄球、指尖目标与路径预览，本步骤不发送运动。球完整位于棋盘内部角点围成的区域，作为首轮试验范围。
3. 核对识别的是当前34mm黄球，原装夹爪的开口能包住球，整臂扫过范围没有漏斗、支架或人手。勾选现场看护与清场确认后，点击“2 · 执行一次抓取”。该点击自动申请本次运动授权、开始10秒录像、调用同一抓取执行器，无需另开终端解锁。
4. 查看实时画面、各段进度与最终关键帧。刷新或关掉网页不会重新执行，也不会停止正在进行的服务器任务；重新打开页面可接着查看。“停止后续动作”会阻止后续指令，已接受的动作可能继续，紧急情况应切断运动电源。
5. 本轮完成的是抓取与提升约5cm，不含漏斗投放。机械臂到位并不等于抓起，视觉结果在Grok或人工核对录像前保留“无法确认”。记录自动显示在同页下方，原RGB-D和轨迹继续保存。

预览需实际加载成功并由用户勾选后才能执行；计划3分钟失效，执行过的收据不允许重跑；页面加载、状态轮询与刷新均不申请运动授权。错误后不自动重试动作。

### 黄色球尺寸核对失败

预期像素直径由34mm球径、相机内参与标定平面处的相机距离估算；识别直径来自黄色轮廓。旧外接圆容易被附着的反光或噪点撑大，现改用轮廓面积等效直径核对，保留外接圆供诊断，**允许偏差仍为±22%**。

HTTP409会在页内显示预期/实测像素直径、偏差、叠加图与黄色掩膜，诊断同时保存在本次2秒观察目录的 `ball-size-diagnostic.json`、`ball-size-overlay.jpg`、`ball-size-mask.png`。不能仅凭“尺寸不符”认定球摆错；先看是否轮廓混入其他黄色区域、球被遮挡，或相机/棋盘发生移动、距离估算不一致。未通过时不会授权或发送运动，不应盲目放宽阈值。

## OpenCode代执行入口（命令行可选）

仅在用户要求由OpenCode代操作时使用下述CLI流程；用户在UI操作时，OpenCode不要同时调用这些执行接口。

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
