# EvoArm 快速示教（v2 快照）

本页面在 `/teach`：单关节**绝对目标角度滑块** + 5°/10°/20° 快速调整；拖动只改目标，点击“执行到目标角度”才发送**一条**指令。

## 启动

```bash
git fetch origin
git switch --track origin/feature/evoarm-fast-pose-teaching
source .venv/bin/activate
python -m unittest discover -s tests -v
EVOARM_ENABLE_MOTION=1 EVOARM_ALLOWED_JOINTS=1,2,3,4,5,6 \
 python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

打开 `http://127.0.0.1:8765/teach`。旧的 `/pick` 页面也提供入口。

## 使用流程

1. 清理**整条机械臂**和线缆的扫掠空间，固定底座/摄像头。确认有独立切断动力电源的方法。网页登录解锁120秒；无需每步再弹窗。
2. 将目标角度滑块拖到合适位置后**点击执行**。单次最多改变30°，按统一官方接口范围限位、15°/秒速度、20°/秒²加速度；每次只允许一条运动指令，等待真实反馈。并非碰撞检测；切勿将软件“禁止后续动作”当作急停。
3. 在实际经过验证的姿态依次点击保存：Home（初始）、Grasp（真实抓球）、Hover（球上方预备）、Place（实际放球）。
4. 夹爪张开/闭合也可用 J6 滑块，分别记录 Gripper Open / Close；记录时仅保存夹爪角度，不保存XYZ。
5. 完成后可复制快照，用于下一步审核新路径。

保存地址：`~/.evoarm/pick_poses_v2.json`。**不会覆盖**
`~/.evoarm/robot_xy_calibration.json` 或 `~/.evoarm/pick_profile.json`。

**重要：新版 v2 快照目前只是可复现的示教数据，不会被旧版 /pick 自动执行器消费。** 旧版固定 Pitch 和单一 travel Z 的抓放序列仍不适合直接运行；需要下一步基于 Home→Hover→Grasp→Hover→Place 的不同姿态重新设计路径并做硬件验收。

未确认固件支持单关节独立释放扭矩，故本版不开放部分 Torque OFF。所有关节正常保持 Torque ON，只有选中的关节收到目标运动指令；负载与齿隙仍可带来偏差。
