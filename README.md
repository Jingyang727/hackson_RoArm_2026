# EvoArm — Agentic Robotic Self-Improvement

RoArm-M3 + Mac M1 的机器人实验项目。

## 开发约定

- `main` 为基线分支；实验功能通过独立分支和 PR 提交。
- 真实机械臂控制应从只读状态联调开始，严禁长按连续运动，运动前检查实际工作空间并保持可切断电源。
- 当前开发分支 `feature/evoarm-control-ui` 提供中文安全控制台（默认只读）。

> 软件限位不是碰撞避免；真实硬件测试必须有人监护。
