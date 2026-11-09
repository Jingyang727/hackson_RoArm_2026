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

## 安全手动微步进（新功能，需实物检查后使用）

**此前发生过碰撞显示器的事故。** 上电和运动前移走显示器等物品，检查机械臂夹爪、连杆、供电和底座稳定性；确保有可立即切断机械臂电源的硬件手段。软件关节限位不能避免所有碰撞。

默认 `python -m uvicorn ...` 运行仍处于 **只读模式**。若已经实地检查完毕，确认有足够的安全运动空间，可从 Mac 终端先结束旧服务，手动启用最受限的试验：

```bash
EVOARM_ENABLE_MOTION=1 EVOARM_ALLOWED_JOINTS=6 \
python -m uvicorn control_ui.main:app --host 127.0.0.1 --port 8765
```

访问 `http://127.0.0.1:8765`，选串口连接、等待角度反馈、点击 **确认安全 · 仅授权一次**，然后测试夹爪 `-1°`（夹爪减小角度为张开方向）。每个动作只能执行一次、每次还需弹窗确认。再次动作需要重新解锁。**请勿连续点击。**

- 初次仅允许6号夹爪动作；底座、肩部、肘部、两个手腕关节全部锁定。
- 已确认官方 `T=121` 使用 **角度**，而 `T=1051` 的 `b/s/e/t/r/g` 反馈使用 **弧度**。前后端负责单位转换。
- 单步只能是 +1° 或 -1°，夹爪的软件临时测试范围为 160°–180°（官方实际机械范围更大），移动限速 `spd=3°/s`、`acc=10°/s²`。
- 超出软件范围、未查询到状态、关节未授权、关节扭矩锁明确关闭等情况拒绝指令。
- 打开网页 **不会自动使能运动**。关闭网页/拔下 USB 也 **不能**阻止已发送的机械臂运动，遇到危险请使用真正的电源级停机手段。
- 目前没有接入自动视觉抓取或 XYZ 自动运动；其他关节需要完成单独的方向、零位和无障碍空间验证后再开放。

### 俯视相机高度

`/camera` 仍然能在 Mac 直接预览 USB 相机。可以手动升降支架，直到桌面抓取区和标定棋盘格完整可见，再锁紧支架。
**每次改变摄像头高度、俯仰或平面位置，都应重新进行桌面标定**。拍摄画面不等于机器人坐标系，未经标定不能自动控制机械臂抓取。

### 当前开发分支

`feature/evoarm-safe-manual-jog` 建立在 `feature/evoarm-overhead-camera` 上，继承最新的 ESP32 串口启动等待修复。
