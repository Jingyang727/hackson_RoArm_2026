# EvoArm · RoArm-M3-S 工作区

当前目标：公司 Grok 观察黄色球与可移动黑漏斗，在已验证的桌面工作区内完成抓起、入漏斗，并以10秒为完整动作目标。

界面现在只保留 **手动标定** 和 **抓取试验**。旧控制台的逐关节5°/10°调整、快速示教、固定抓放位置和复位控件已移除；旧页面地址会跳转到新工作区。抓取页通过已有执行后端完成一次受控抓取与提升。

[手动标定预览](docs/images/manual-calibration-preview.png) · [抓取页面预览](docs/images/grasp-ui-preview.png)（均为模拟画面与反馈）

## 当前状态

- 五点标定：复用已有Gemini空棋盘录像，手动对准并采三组静止反馈；四点拟合、第五点独立检验，逐点保存并支持续采。
- 实时对准：相机画面同时叠加五点位置，当前点高亮、已记录点打勾；标记不依赖每帧检测完整棋盘，支持遮挡。断流时暂停采点，恢复扭矩操作仍可用。
- 手动方式：使用现有全部释放接口，持续支撑机械臂，连续记录五点；无需每个点重新上锁，五点结束再恢复扭矩。
- 释放的是全部六关节，旋转与夹爪也会松开。页面显示相对首点的角度差，超出5°时要求核对并调整回同一触点。俯仰尽量一致，差异较大时保留采点并提示。
- [首轮实物抓取](docs/bench-first-grasp.md)支持浏览器操作：点“观察并规划”、核对预览、再点“执行一次抓取”；显示实时画面、动作进度、停止后续动作和关键帧。当前RGB-D定位黄球，高位张爪、接近、闭爪、提升约5cm，保留10秒录像和反馈。用户可在UI核对预览，OpenCode自身的公司Grok也可评估关键帧；不要求第二个模型网关。漏斗投放与后续自动策略调整仍待接入。
- RSI感知策略：已有五点标定继续用于每轮平面定位；棋盘遮挡、稀疏深度、球径偏差和超出棋盘内框作为估计警告，允许核对预览后试抓。明确坐标冲突、目标缺失/移动与运动约束仍会拒绝。感知估计与动作结果一同保存，供下一轮分析。
- 标定后[从当前姿态起步](docs/bench-first-motion.md)：低位检查受限抬臂，高位跳过多余抬升；仅腕部越出训练范围时检查一次高位向内回收。`already_clear` 后继续抓取入口，无需回到标定触点或固定示教姿态。

**改动涉及本机网页、只读采点与独立相机服务。官方板载网页、固件、串口驱动和原有扭矩控制实现未修改。** 页面调用现有 `/api/teach/torque`，不发送新的固件命令。

已核对的官方20260701源码中，T210释放会先回位，再释放全部舵机；恢复扭矩也可能使机械臂移动。页面会在切换前明确说明，并要求现场支撑及检查运动范围，不把现有功能描述成原地释放或部分关节锁定。

## 本机运行

从现有独立worktree更新 `feature/rsi-grok-pick-place-10s`，保留原工作目录、未提交改动与所有旧标定数据。代码更新后实际运行的服务也必须从新worktree启动。

```bash
python -m pip install -r requirements.txt -r requirements-vision.txt
EVOARM_ENABLE_TEACH=1 EVOARM_ENABLE_MOTION=0 EVOARM_SETUP_ID=bench-layout-v2 \
python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

需要UI实际抓取时，将上述 `EVOARM_ENABLE_MOTION=0` 改为 `1` 后启动，打开 http://127.0.0.1:8765/experiments 。开启实物模式本身不下发抓取，只有核对预览并点击执行才申请运动授权。详见[浏览器操作说明](docs/bench-first-grasp.md)。

打开 http://127.0.0.1:8765/calibration 。页面不会自动连接串口或切换扭矩；连接操作可能触发固件初始化，需要现场看住机械臂。

相机继续使用独立服务，只有该相机进程按实际需要使用sudo。已有相机health返回 `live_preview: true` 和 `recording_request_ids: true` 时直接沿用；仅能力缺失时先停止旧服务再启动新版，保持一个SDK所有者。控制UI和Agent保持普通用户权限。实时画面、快照和RGB-D录像共用一个SDK采集线程；切换到参考图或快照时明确标明非实时。详见[本机更新与实时预览说明](docs/bench-layout-v2-runbook.md)。

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

单元测试不连接真实机械臂。可另用 `tests/browser_manual_calibration.py` 和 `python -m tests.browser_grasp_ui` 在本地UI服务上运行Chromium验收；所有设备接口均被模拟拦截。所有真机操作仍需现场执行。
