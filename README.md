# EvoArm · RoArm-M3-S 工作区

当前目标：公司 Grok 观察黄色球与可移动黑漏斗，在已验证的桌面工作区内完成抓起、入漏斗，并以10秒为完整动作目标。

界面现在只保留 **手动标定** 和 **试验记录**。旧控制台的逐关节5°/10°调整、快速示教、固定抓放位置和复位控件已移除；旧页面地址会跳转到新工作区。机械臂执行后端保留，供后续受控执行适配器使用。

[查看新版界面预览（模拟棋盘与反馈）](docs/images/manual-calibration-preview.png)

## 当前状态

- 五点标定：复用已有Gemini空棋盘录像，手动对准并采三组静止反馈；四点拟合、第五点独立检验，逐点保存并支持续采。
- 实时对准：相机画面同时叠加五点位置，当前点高亮、已记录点打勾；标记不依赖每帧检测完整棋盘，支持遮挡。断流时暂停采点，恢复扭矩操作仍可用。
- 手动方式：使用现有全部释放接口，持续支撑机械臂，连续记录五点；无需每个点重新上锁，五点结束再恢复扭矩。
- 释放的是全部六关节，旋转与夹爪也会松开。页面显示相对首点的角度差，超出5°时要求核对并调整回同一触点。俯仰尽量一致，差异较大时保留采点并提示。
- 场景识别、Grok结果评估、本地历史和可选Langfuse回读已实现；真实抓放执行和自动策略调整仍待几何与路径验收。

**改动涉及本机网页、只读采点与独立相机服务。官方板载网页、固件、串口驱动和原有扭矩控制实现未修改。** 页面调用现有 `/api/teach/torque`，不发送新的固件命令。

已核对的官方20260701源码中，T210释放会先回位，再释放全部舵机；恢复扭矩也可能使机械臂移动。页面会在切换前明确说明，并要求现场支撑及检查运动范围，不把现有功能描述成原地释放或部分关节锁定。

## 本机运行

从现有独立worktree更新 `feature/rsi-grok-pick-place-10s`，保留原工作目录、未提交改动与所有旧标定数据。代码更新后实际运行的服务也必须从新worktree启动。

```bash
python -m pip install -r requirements.txt -r requirements-vision.txt
EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=0 EVOARM_SETUP_ID=bench-layout-v2 \
python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

打开 http://127.0.0.1:8765/calibration 。页面不会自动连接串口或切换扭矩；连接操作可能触发固件初始化，需要现场看住机械臂。

相机继续使用独立服务，只有该相机进程按实际需要使用sudo。**更新时先停止旧相机服务，再从新版worktree重启一个服务**，其health应返回 `live_preview: true`。控制UI和Agent保持普通用户权限。实时画面、快照和RGB-D录像共用一个SDK采集线程；切换到参考图或快照时明确标明非实时。详见[本机更新与实时预览说明](docs/bench-layout-v2-runbook.md)。

## 现场流程

- [bench-layout-v2 当前标定与首轮试验](docs/bench-layout-v2-runbook.md)
- [Grok抓放规划、10秒计时与历史检索](docs/grok-pick-place-v1.md)
- [旧控制台说明归档](docs/legacy-control-console-v0.4.md)

新版标定生成独立文件，不覆盖 `robot_xy_calibration.json` 或旧示教点。相机、棋盘或底座移动后必须采用新的布置版本重新标定。

## 验证

```bash
python -m unittest discover -s tests -v
node --test tests/*.test.cjs
```

单元测试不连接真实机械臂。可另用 `tests/browser_manual_calibration.py` 在本地UI服务上运行Chromium验收；所有设备接口均被模拟拦截。所有真机操作仍需现场执行。
