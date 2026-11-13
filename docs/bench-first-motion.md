# bench-layout-v2：从当前姿态起步

五点已完成，相机、底座和桌面默认固定，棋盘可撤走，不重采、不使用旧Home/Hover/Grasp/Place。
无需把机械臂放回标定角点，也无需在高位维持标点时的俯仰。起步状态由新鲜T1051和本次桌面模型判断。
本入口只处理抓取前的辅助准备，`grasp_attempted=false`，不计作抓取成功。当前先按[坐标验证](coordinate-validation.md)由用户在UI测试点位，不自动进入抓取。

## 起步状态与动作

- `already_clear`：末端离桌面至少100mm、关节在官方接口范围内。跳过多余动作；水平或垂直朝向都可能接受。无需为本入口授权或发送动作。
- `low_pose_lift`：当前高度不足100mm时，检查一次受限肘关节抬升；不再要求末端恰好位于棋盘接触点。贴近桌面时仍需核对触点与工具姿态。
- `blocked=true`：输出拒绝原因和实际反馈。不让用户猜一个新姿态，不放宽限位、复位或回旧示教。

例如整体Pitch约80°、腕部约81°、离桌约130mm，若其余检查通过即为`already_clear`。旧的`high_wrist_recovery`只为绕开项目±60°训练范围而存在，现已取消。`/api/bench/motion/start-once`保留停用响应，不读写设备；不能再据旧文档把腕部额外收回55°。

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
4. 仅在 `motion_required=true`（低位抬升）且预测通过时，通过OpenCode自身的公司Grok结合当前图片检查整段路径，并由现场看护确认运动空间。
   授权成功后运行一次：

   ```bash
   curl --fail-with-body --silent --show-error -X POST http://127.0.0.1:8765/api/unlock-once \
     -H 'Content-Type: application/json' --data '{"declaration":"已检查设备并清空运动区域"}'
   /Users/jyang51/miniconda3/envs/evoarm/bin/python -m evoarm_rsi.bench_start --execute > /tmp/evoarm-start-result.json
   ```

   程序等待本次10秒RGB-D录像开始后，只执行低位抬升预测对应的一条动作。高位状态不执行额外动作，也不需要恢复接口或新的运动授权。
   到位或异常后撤销运动授权，扭矩保持。HTTP响应丢失或反馈不确定时不重发、不反向返回；保留 `first-motion/lift-attempt.json`。
5. 回传计划JSON、执行结果（若有）、新鲜T1051和当前JPEG。执行产生的 `capture_trial_id` 对应目录包含RGB、深度、帧时间戳和 `pregrasp-lift.json`（兼容旧文件名，内容含起步类型）。
   若重连后已经处于 `already_clear`，回传该状态，并通过已有相机服务录10秒当前球、漏斗与夹爪场景，供接近与抓取规划；不要为了制造动作而重新调整机械臂。

## 边界

100mm仅是本辅助抓取起步检查的净空标准，不是XYZ页面的最低高度。坐标页直接从当前反馈规划，不先执行此入口。关节角、整臂形状和实际障碍仍需核对。
[官方20260701源码](https://www.waveshare.net/w/upload/4/48/RoArm-M3_example_20260701.zip)的腕部命令在±90°夹限；全项目现统一使用官方名义范围，模型采用默认edge-tool尺寸并与固件XYZ/Pitch交叉核对。
已有五点原文件保持不变。本动作仍不是完整抓取或投放，下一步依据新位置、画面和工具几何继续规划。
