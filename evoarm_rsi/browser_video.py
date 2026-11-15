"""Offline H.264 playback copy; preserve raw camera evidence and timestamps."""
from pathlib import Path
import shutil
import subprocess
import threading
import uuid


_lock = threading.Lock()


def browser_video(folder):
    folder = Path(folder)
    source = folder / 'rgb.mp4'
    output = folder / 'rgb-browser.mp4'
    if not source.is_file() or source.is_symlink():
        raise ValueError('原录像不存在')
    with _lock:
        if output.is_symlink():
            raise ValueError('回放文件路径无效')
        if output.is_file() and output.stat().st_size:
            return output
        try:
            import imageio_ffmpeg
            encoder = imageio_ffmpeg.get_ffmpeg_exe()
        except (ImportError, RuntimeError):
            encoder = shutil.which('ffmpeg')
        if not encoder:
            raise RuntimeError('缺少浏览器回放编码器，请安装 requirements-vision.txt；原录像仍可下载')
        temp = folder / f'.rgb-browser-{uuid.uuid4().hex}.mp4'
        try:
            result = subprocess.run([
                encoder, '-hide_banner', '-loglevel', 'error', '-nostdin',
                '-i', str(source), '-an', '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2',
                '-c:v', 'libx264', '-threads', '1', '-preset', 'veryfast',
                '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                str(temp),
            ], capture_output=True, timeout=30, check=False)
            if result.returncode or not temp.is_file() or not temp.stat().st_size:
                raise RuntimeError('浏览器回放转码失败；原录像和逐帧时间戳已保留')
            temp.replace(output)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError('浏览器回放转码超时；原录像已保留') from exc
        finally:
            temp.unlink(missing_ok=True)
        return output
