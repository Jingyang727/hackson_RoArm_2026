# EvoArm Control Center v0.1

RoArm-M3-S / M3-Pro 的 **Mac M1 中文本地控制台**。使用 Python、FastAPI、pyserial，保持无需 Jetson 的单机架构。

> **安全警告：** 曾发生机械臂碰撞显示器的事故。先断电检查关节、夹爪、线缆、底座，移除工作区障碍物。机械臂上电可能自动回中。软件限位不能保证无碰撞，关闭网页、断开 USB 或停止 Python 进程也不等于物理急停。

## 功能

- 读取 Mac 上 `/dev/cu.*` 串口
- 查询官方状态指令 `{"T":105}`，显示 `T=1051` 反馈，每约2秒刷新
- 全中文六关节反馈和说明
- **默认只读：运行命令不会使能运动**
- 经现场硬件检查后，可显式允许一次一个 ±1° 关节移动（服务端开关 + 单次解锁 + 限位 + 冷却）
- 不支持长按连续转动、不支持一键回中，不接受任意 JSON 指令

## 安装（Mac）

```bash
git clone https://github.com/Jingyang727/hackson_RoArm_2026.git
cd hackson_RoArm_2026
git switch feature/evoarm-control-ui
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m unittest discover -s tests -v
python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

打开 <http://127.0.0.1:8765>，仅测试连接与状态显示。串口若已被之前的 CLI 或厂家软件占用，先关闭那个进程。机械臂仍需使用独立电源。不要将服务绑定到 `0.0.0.0`。

## 运动模式（默认禁止，当前不建议在碰撞后的首次验证中启用）

只有完成现场检查并确认机械臂动作映射、关节反馈单位与活动空间后，才考虑运行：

```bash
EVOARM_ENABLE_MOTION=1 python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

浏览器逐次确认解锁，每个操作仅 ±1°，2秒冷却。允许操作不代表机械臂一定没有碰撞风险；尤其不要把软件停止当成安全急停。

本分支的 `T=121` 关节命令是依据现有 RoArm-M3 资料做的初版实现，**尚未用当前真实机械臂实测运动安全性**。确认姿态限制、固件兼容和机械零位前，不建议使能。

## Git 协作

工作分支：`feature/evoarm-control-ui`。请通过 PR 审核后再合并到 main。以后的 Observer / Scientist / RSI 代码继续独立开发，避免覆盖机器人控制层。

官方文档：https://www.waveshare.net/wiki/RoArm-M3

## 俯视相机预览（无需安装 OpenCV）

本分支还提供 `http://127.0.0.1:8765/camera`：Mac 浏览器直接查看 USB 摄像头的实时画面、选择设备并保存一帧 PNG。此功能不需要 Jetson，也不会触发任何机械臂运动。首次使用时在浏览器里允许摄像头权限。

安装、标定和安全检查见 [docs/overhead-camera.md](docs/overhead-camera.md)。
