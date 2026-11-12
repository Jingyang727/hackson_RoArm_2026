# bench-layout-v2：从当前姿态起步

五点已完成，保持相机、棋盘和底座固定，不重采、不使用旧Home/Hover/Grasp/Place。
无需把机械臂放回标定角点，也无需在高位维持标点时的俯仰。起步状态由新鲜T1051和本次桌面模型判断。
本入口只处理准备动作，完整抓放执行器仍在接入中；`grasp_attempted=false`，不计作抓取成功。

## 起步状态与动作

- `already_clear`：末端离桌面至少100mm、关节在项目训练范围内。跳过多余抬升；水平或垂直朝向都可能接受。无需为本入口授权或发送动作，接下来进行本轮场景规划。
- `high_wrist_recovery`：末端已经在100mm以上，仅腕部处于训练范围±60°之外、但仍在官方±90°范围内。闭合夹爪，检查整段回收路径，最多只发送一条朝训练范围内部的腕部指令。其他关节必须在训练范围内。
- `low_pose_lift`：当前高度不足100mm时，检查一次受限肘关节抬升；不再要求末端恰好位于棋盘接触点。贴近桌面时仍需核对触点与工具姿态。
- `blocked=true`：输出拒绝原因和实际反馈。不让用户猜一个新姿态，不放宽限位、复位或回旧示教。

例如整体Pitch约80°、腕部约81°、离桌约130mm，可检查高位腕部回收。动作可能伴随数厘米的横向扫过；目标和轨迹必须按当前反馈重新计算并检查现场画面，它不是新的示教点。
正常手动控制的±60°限位保持不变。恢复接口仅支持上述高位、向内、单次、不超过30°的路径；不允许用户传入电机角度目标。

## 转发给本地 OpenCode

只运行已有程序与操作设备，不开发代码。用户现场看护，人手退出工作区，能及时断电。

1. 更新独立worktree的 `feature/rsi-grok-pick-place-10s`，保留本地改动及全部标定。仅重启普通用户网页控制服务，从新版代码启动：

   ```bash
   EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=1 EVOARM_SETUP_ID=bench-layout-v2 \
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
   ```

   只有一个串口所有者。串口关闭重开可能触发设备初始化，需要现场看护；重连后按新反馈重新分类，不要求用户再手动摆成垂直姿态。
2. 沿用已验收、含 `recording_request_ids=true` 的相机服务，无需重启它或另外打开SDK。
   读取 `/api/health`，未连接时才调用 `POST /api/auto-connect`，再读取新鲜 `/api/status`。确认六关节扭矩为1、fault/pending为空；不发送扭矩切换、复位或旧示教命令。
3. 获取 `/api/bench/live-frame?after=0` 保存当前JPEG。运行只读预测：

   ```bash
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.bench_start > /tmp/evoarm-start-plan.json
   ```

   读JSON中的 `start_mode`、`motion_required`、`plan.predicted_path`。`blocked=true` 时回传实际状态与原因，不继续。`already_clear` 是正确结果，不是失败：无需授权或多余抬臂，更不能要求用户回到棋盘触点。
4. 仅在 `motion_required=true` 且预测通过时，通过OpenCode自身的公司Grok结合当前图片检查整段路径，并由现场看护确认运动空间。高位腕部回收会有约7cm的横向扫过，不能只检查指尖最终位置。
   授权成功后运行一次：

   ```bash
   curl --fail-with-body --silent --show-error -X POST http://127.0.0.1:8765/api/unlock-once \
     -H 'Content-Type: application/json' --data '{"declaration":"已检查设备并清空运动区域"}'
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.bench_start --execute > /tmp/evoarm-start-result.json
   ```

   程序等待本次10秒RGB-D录像开始后，只执行预测对应的一条动作。高位恢复由新版控制服务在串口锁内重新计算，并核对标定、当前关节、录像ID和授权，不接受外部角度目标。需要 `/api/bench/motion/capabilities` 返回 `high_wrist_recovery=true`。
   到位或异常后撤销运动授权，扭矩保持。HTTP响应丢失或反馈不确定时不重发、不反向返回；保留 `first-motion/lift-attempt.json`。
5. 回传计划JSON、执行结果（若有）、新鲜T1051和当前JPEG。执行产生的 `capture_trial_id` 对应目录包含RGB、深度、帧时间戳和 `pregrasp-lift.json`（兼容旧文件名，内容含起步类型）。
   若重连后已经处于 `already_clear`，回传该状态，并通过已有相机服务录10秒当前球、漏斗与夹爪场景，供接近与抓取规划；不要为了制造动作而重新调整机械臂。

## 边界

100mm是本次起步检查的净空标准，不是要求用户示教固定高度。正常起步不需要某个固定XYZ；关节角、整臂形状和实际障碍仍需核对，任意姿态不自动等于有无碰撞的路径。
[官方20260701源码](https://www.waveshare.net/w/upload/4/48/RoArm-M3_example_20260701.zip)的腕部命令在±90°夹限，模型采用默认edge-tool尺寸，并与固件XYZ/Pitch交叉核对。高位回收的初始状态可在±60°训练范围之外，目标在其内；这不是把正常训练范围扩大到±90°。
已有五点原文件保持不变。本动作仍不是完整抓取或投放，下一步依据新位置、画面和工具几何继续规划。
