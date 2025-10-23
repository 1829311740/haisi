import cv2
import serial
import serial.tools.list_ports
from PyQt5.QtGui import QDoubleValidator, QIntValidator, QFont, QIcon
import time
import os
import numpy as np
import random
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QComboBox,
    QLineEdit,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QMessageBox,
    QProgressBar,
    QTextEdit,
    QScrollArea,
    QGroupBox,
    QCheckBox,
    QDialog,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QFrame,
    QGridLayout,
    QSplitter,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QEventLoop
import json
from datetime import datetime
import shutil
import sys
import re
import threading
import ctypes
import platform


# ==================== 高精度时间与工具 ====================
class WinHighResTimer:
    """
    在 Windows 上把系统时钟精度提高到 1ms，退出时恢复。
    其他平台下保持空操作。
    """

    def __enter__(self):
        if platform.system() == "Windows":
            try:
                self._winmm = ctypes.WinDLL("winmm")
                self._winmm.timeBeginPeriod(1)
            except Exception:
                pass
        return self

    def __exit__(self, exc_type, exc, tb):
        if platform.system() == "Windows":
            try:
                self._winmm.timeEndPeriod(1)
            except Exception:
                pass


def _parse_duration_to_ms(text: str) -> float:
    """
    解析一个时长字符串为毫秒(float)。
    支持格式：
      - 纯数字：'1500' -> 1500ms（默认 ms）
      - '1500ms' / '1500.5ms'
      - '1.5s' / '0.25s'
    """
    s = text.strip().lower()
    if s.endswith("ms"):
        return float(s[:-2].strip())
    if s.endswith("s"):
        return float(s[:-1].strip()) * 1000.0
    return float(s)


def parse_range_to_ms_pair(text: str) -> tuple:
    """
    解析范围字符串 'a-b' 为 (min_ms, max_ms)，单位规则同上。
    """
    parts = [p for p in re.split(r"\s*-\s*", text.strip()) if p]
    if len(parts) != 2:
        raise ValueError("延迟范围格式错误，应为 a-b（支持 ms 或 s）")
    a_ms = _parse_duration_to_ms(parts[0])
    b_ms = _parse_duration_to_ms(parts[1])
    if a_ms < 0 or b_ms < a_ms:
        raise ValueError("延迟范围必须满足：最小值 ≥ 0 且 最大值 ≥ 最小值")
    return (a_ms, b_ms)


def precise_sleep_ms(ms: float, cancel_check=None):
    """
    高精度睡眠（毫秒）。使用单调时钟 + 绝对时刻，防止累计漂移。
    cancel_check: 可传入函数，返回 True 则提前退出（用于线程结束）
    """
    if ms <= 0:
        return
    start_ns = time.perf_counter_ns()
    target_ns = start_ns + int(ms * 1_000_000)

    while True:
        if cancel_check and cancel_check():
            return
        now_ns = time.perf_counter_ns()
        remain_ns = target_ns - now_ns
        if remain_ns <= 0:
            break
        if remain_ns > 2_000_000:
            sleep_ms = (remain_ns - 1_000_000) / 1_000_000.0
            time.sleep(sleep_ms / 1000.0)
        else:
            time.sleep(0)


def qt_wait_ms(ms: int):
    """
    在主线程中非阻塞等待 ms（不卡 UI）。
    """
    loop = QEventLoop()
    t = QTimer()
    t.setSingleShot(True)
    t.timeout.connect(loop.quit)
    t.start(int(ms))
    loop.exec_()


# ==================== 相机全局与线程安全封装 ====================
cap = None
cap_lock = threading.RLock()


def _open_camera_cross_platform(index: int = 0):
    """跨平台打开摄像头。Windows 用 CAP_DSHOW，Linux 优先 CAP_V4L2，其它平台用默认。"""
    try:
        if platform.system() == "Windows":
            camera = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        elif platform.system() == "Linux":
            # 有些环境 CAP_V4L2 更稳定；失败则回退默认
            camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if not camera.isOpened():
                camera.release()
                camera = cv2.VideoCapture(index)
        else:
            camera = cv2.VideoCapture(index)
        return camera
    except Exception:
        return cv2.VideoCapture(index)


def camera_open(width: int = 1280, height: int = 720) -> bool:
    global cap
    with cap_lock:
        try:
            if cap and cap.isOpened():
                return True
            camera = _open_camera_cross_platform(0)
            if camera.isOpened():
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap = camera
                return True
            else:
                try:
                    camera.release()
                except Exception:
                    pass
                cap = None
                return False
        except Exception:
            cap = None
            return False


def camera_close():
    global cap
    with cap_lock:
        try:
            if cap and cap.isOpened():
                cap.release()
        except Exception:
            pass
        finally:
            cap = None


def camera_read():
    """线程安全读取一帧。返回 (ret, frame)。"""
    global cap
    with cap_lock:
        if not cap or not cap.isOpened():
            return False, None
        try:
            return cap.read()
        except Exception:
            return False, None


def camera_flush(n: int = 5, delay_ms: int = 10):
    for _ in range(n):
        _ = camera_read()
        precise_sleep_ms(delay_ms)


# ==================== 运行时全局 ====================
running = False
regions = []
drawing = False
start_point = None
end_point = None
regions_selected = False
image_folder = "captured_images"
error_images_folder = "error_images"
reference_image_folder = "Reference_image"
reference_colors = []
COLOR_THRESHOLD_DEFAULT = 80
BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400]

# ==================== 继电器命令表（1/2/3/4 路） ====================
RELAY_CMDS = {
    1: {"on": bytearray([0xA0, 0x01, 0x01, 0xA2]), "off": bytearray([0xA0, 0x01, 0x00, 0xA1])},
    2: {"on": bytearray([0xA0, 0x02, 0x01, 0xA3]), "off": bytearray([0xA0, 0x02, 0x00, 0xA2])},
    3: {"on": bytearray([0xA0, 0x03, 0x01, 0xA4]), "off": bytearray([0xA0, 0x03, 0x00, 0xA3])},
    4: {"on": bytearray([0xA0, 0x04, 0x01, 0xA5]), "off": bytearray([0xA0, 0x04, 0x00, 0xA4])},
}


class CaptureErrorThread(QThread):
    """屏幕异常连续拍照线程（线程安全相机读）"""

    capture_status = pyqtSignal(str)
    capture_completed = pyqtSignal()

    def __init__(self, interval_ms: int, duration_ms: int, abnormal_filename: str):
        super().__init__()
        self.interval_ms = int(interval_ms)
        self.duration_ms = int(duration_ms)
        self.abnormal_filename = abnormal_filename
        self._stop_event = threading.Event()

    def run(self):
        if not camera_open():
            self.capture_status.emit("摄像头未初始化，无法进行连续拍照")
            return

        error_folder = f"{error_images_folder}/error_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        try:
            os.makedirs(error_folder, exist_ok=True)
        except Exception as e:
            self.capture_status.emit(f"创建目录失败: {e}")
            return

        try:
            if self.abnormal_filename and os.path.exists(self.abnormal_filename):
                ref_filename = os.path.join(error_folder, "original_error.jpg")
                shutil.copy(self.abnormal_filename, ref_filename)
                self.capture_status.emit(f"已复制初始异常图片到 {ref_filename}")
        except Exception as e:
            self.capture_status.emit(f"复制原始异常图片失败: {e}")

        total_captures = max(1, int(self.duration_ms / max(1, self.interval_ms)))
        self.capture_status.emit(
            f"开始连续拍照: 每{self.interval_ms}毫秒一次，共{total_captures}张"
        )

        start_time = time.time()
        end_time = start_time + (self.duration_ms / 1000.0)
        count = 0

        while time.time() < end_time and not self._stop_event.is_set():
            try:
                ret, frame = camera_read()
                if not ret or frame is None:
                    self.capture_status.emit("读取摄像头画面失败")
                    break

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = os.path.join(error_folder, f"error_{timestamp}.jpg")
                try:
                    cv2.imwrite(filename, frame)
                except Exception as e:
                    self.capture_status.emit(f"保存图片失败: {e}")
                    break

                count += 1
                elapsed = time.time() - start_time
                self.capture_status.emit(
                    f"已连续拍摄 {count}/{total_captures} 张 (用时{elapsed:.1f}秒)"
                )

                next_capture_time = start_time + ((count * self.interval_ms) / 1000.0)
                wait_time_ms = max(0.0, (next_capture_time - time.time()) * 1000.0)
                precise_sleep_ms(wait_time_ms, cancel_check=self._stop_event.is_set)
            except Exception as e:
                self.capture_status.emit(f"连续拍照出错: {e}")
                break

        self.capture_status.emit(f"连续拍照完成，共拍摄 {count} 张图片")
        self.capture_completed.emit()

    def stop(self):
        self._stop_event.set()


class CameraThread(QThread):
    camera_error = pyqtSignal(str)
    camera_ready = pyqtSignal()

    def __init__(self):
        super().__init__()

    def run(self):
        if camera_open(1280, 720):
            self.camera_ready.emit()
        else:
            self.camera_error.emit("摄像头初始化失败，请检查设备连接！")

    def stop(self):
        camera_close()


class TestThread(QThread):
    test_error = pyqtSignal(str)
    test_result = pyqtSignal(str)
    abnormal_detected = pyqtSignal(str, str, float)

    def __init__(
        self,
        parent,
        port,
        baudrate,
        startup_delay_range_ms,
        shutdown_delay_range_ms,
        region_count,
        threshold,
        relay_channels: list,
        channel_times=None,
        capture_image=True,
    ):
        super().__init__(parent)
        self.parent = parent
        self.port = port
        self.baudrate = baudrate
        self.startup_delay_range = startup_delay_range_ms
        self.shutdown_delay_range = shutdown_delay_range_ms
        self.region_count = region_count
        self.threshold = threshold
        self.relay_channels = [ch for ch in relay_channels if ch in RELAY_CMDS]
        if not self.relay_channels:
            raise ValueError("未选择有效的继电器通道")
        self.should_power_off = True
        self._stop_event = threading.Event()
        self.channel_times = channel_times or {}
        self.capture_image = capture_image

    def _write_with_retry(self, cmd, MAX_RETRIES=3, RETRY_DELAY_MS=1000):
        for attempt in range(MAX_RETRIES):
            try:
                self.parent.ser.write(cmd)
                return True
            except serial.SerialTimeoutException:
                if attempt < MAX_RETRIES - 1:
                    self.test_result.emit(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 命令发送超时，重试 {attempt + 1}/3 ..."
                    )
                    precise_sleep_ms(
                        RETRY_DELAY_MS,
                        cancel_check=lambda: (not running) or self._stop_event.is_set(),
                    )
                else:
                    return False
            except Exception as e:
                if attempt < MAX_RETRIES - 1:
                    self.test_result.emit(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 命令发送错误: {e}，重试 {attempt + 1}/3 ..."
                    )
                    precise_sleep_ms(
                        RETRY_DELAY_MS,
                        cancel_check=lambda: (not running) or self._stop_event.is_set(),
                    )
                else:
                    return False

    def run(self):
        global running
        if self.capture_image and not camera_open():
            self.test_error.emit("摄像头未初始化")
            return
        if self.capture_image and len(regions) != self.region_count:
            self.test_error.emit(f"请先选择 {self.region_count} 个框选区域")
            return
        if not self.parent.ser or not self.parent.ser.is_open:
            self.test_error.emit("串口未连接")
            return

        MAX_RETRIES = 3
        RETRY_DELAY_MS = 1000

        try:
            with WinHighResTimer():
                while running and not self._stop_event.is_set():
                    channel_delays = {}
                    has_custom_times = any(
                        ch in self.channel_times and self.channel_times[ch]
                        for ch in self.relay_channels
                    )

                    if not has_custom_times:
                        startup_delay = random.uniform(
                            self.startup_delay_range[0], self.startup_delay_range[1]
                        )
                        shutdown_delay = random.uniform(
                            self.shutdown_delay_range[0], self.shutdown_delay_range[1]
                        )
                        for ch in self.relay_channels:
                            channel_delays[ch] = (startup_delay, shutdown_delay)
                    else:
                        common_startup = random.uniform(
                            self.startup_delay_range[0], self.startup_delay_range[1]
                        )
                        common_shutdown = random.uniform(
                            self.shutdown_delay_range[0], self.shutdown_delay_range[1]
                        )
                        for ch in self.relay_channels:
                            if ch in self.channel_times and self.channel_times[ch]:
                                ch_startup_min, ch_startup_max, ch_shutdown_min, ch_shutdown_max = (
                                    self.channel_times[ch]
                                )
                                ch_startup = random.uniform(ch_startup_min, ch_startup_max)
                                ch_shutdown = random.uniform(ch_shutdown_min, ch_shutdown_max)
                                channel_delays[ch] = (ch_startup, ch_shutdown)
                            else:
                                channel_delays[ch] = (common_startup, common_shutdown)

                    ok = True
                    for ch in self.relay_channels:
                        if not self._write_with_retry(RELAY_CMDS[ch]["on"], MAX_RETRIES, RETRY_DELAY_MS):
                            ok = False
                            self.test_error.emit(
                                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 通道{ch} 上电失败（重试耗尽）"
                            )
                            break
                        precise_sleep_ms(
                            5,
                            cancel_check=lambda: (not running) or self._stop_event.is_set(),
                        )
                    if not ok:
                        return

                    power_on_time = time.time()
                    self.test_result.emit(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] (通道{','.join(map(str,self.relay_channels))}) 已上电"
                    )

                    if self.capture_image:
                        max_startup_delay = max(delay[0] for delay in channel_delays.values())
                        self.test_result.emit(
                            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 等待 {max_startup_delay:.0f} ms ..."
                        )
                        precise_sleep_ms(
                            max_startup_delay,
                            cancel_check=lambda: (not running) or self._stop_event.is_set(),
                        )
                        if not running or self._stop_event.is_set():
                            break

                        camera_flush(5, 10)
                        ret, frame = camera_read()
                        if not ret or frame is None:
                            self.test_error.emit(
                                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法捕获画面"
                            )
                            precise_sleep_ms(
                                1000,
                                cancel_check=lambda: (not running) or self._stop_event.is_set(),
                            )
                            continue

                        try:
                            os.makedirs(image_folder, exist_ok=True)
                        except Exception:
                            pass

                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                        filename = os.path.join(image_folder, f"capture_{timestamp}.jpg")
                        try:
                            cv2.imwrite(filename, frame)
                        except Exception:
                            pass
                        self.test_result.emit(
                            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已保存检测照片: {filename}"
                        )

                        for i, region in enumerate(regions):
                            is_abnormal, similarity, avg_rgb = self.is_abnormal(
                                frame, region, reference_colors[i], self.threshold
                            )
                            self.test_result.emit(
                                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 区域{i+1} RGB: {avg_rgb}, 参考 RGB: {reference_colors[i]}, 相似度: {similarity:.1f}%"
                            )

                            if is_abnormal:
                                # 不下电，停止循环，并开始连续拍照（由主线程处理）
                                self.abnormal_detected.emit(f"{i+1}", filename, similarity)
                                self.test_error.emit(
                                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 屏幕{i+1}异常，停止测试，异常照片: {filename}"
                                )
                                running = False
                                self.should_power_off = False
                                break
                    else:
                        for ch in self.relay_channels:
                            startup_delay, _ = channel_delays[ch]
                            self.test_result.emit(
                                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 通道{ch} 等待上电时间 {startup_delay:.0f} ms..."
                            )

                    if not running or self._stop_event.is_set():
                        break

                    for ch in self.relay_channels:
                        elapsed_ms = (time.time() - power_on_time) * 1000
                        startup_delay, _ = channel_delays[ch]
                        remain_ms = max(0, startup_delay - elapsed_ms)
                        if remain_ms > 0:
                            self.test_result.emit(
                                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 通道{ch} 继续等待 {remain_ms:.0f} ms..."
                            )
                            precise_sleep_ms(
                                remain_ms,
                                cancel_check=lambda: (not running) or self._stop_event.is_set(),
                            )

                    for ch in self.relay_channels:
                        if not self._write_with_retry(RELAY_CMDS[ch]["off"], MAX_RETRIES, RETRY_DELAY_MS):
                            self.test_error.emit(
                                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 通道{ch} 下电失败（重试耗尽）"
                            )
                            return
                        precise_sleep_ms(
                            5,
                            cancel_check=lambda: (not running) or self._stop_event.is_set(),
                        )

                    max_shutdown_delay = 0
                    for ch in self.relay_channels:
                        _, shutdown_delay = channel_delays[ch]
                        max_shutdown_delay = max(max_shutdown_delay, shutdown_delay)

                    self.test_result.emit(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] (通道{','.join(map(str,self.relay_channels))}) 已全部下电，等待 {max_shutdown_delay:.0f} ms ..."
                    )
                    precise_sleep_ms(
                        max_shutdown_delay,
                        cancel_check=lambda: (not running) or self._stop_event.is_set(),
                    )

        except Exception as e:
            self.test_error.emit(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 测试异常: {e}"
            )
        finally:
            if self.parent.ser and self.parent.ser.is_open:
                try:
                    if self.should_power_off:
                        for ch in self.relay_channels:
                            try:
                                self.parent.ser.write(RELAY_CMDS[ch]["off"])
                                precise_sleep_ms(5)
                            except Exception:
                                pass
                        self.test_result.emit(
                            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] (通道{','.join(map(str,self.relay_channels))}) 已执行收尾下电"
                        )
                    try:
                        self.parent.ser.close()
                    except Exception:
                        pass
                    self.parent.ser = None
                except Exception as e:
                    self.test_result.emit(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 关闭串口失败: {e}"
                    )

    def is_abnormal(self, frame, region, ref_color, threshold):
        x, y, w, h = region
        roi = frame[y : y + h, x : x + w]
        avg_rgb = np.mean(roi, axis=(0, 1))
        diff = np.abs(avg_rgb - ref_color)
        max_diff = np.max(diff)
        similarity = (1 - max_diff / 255) * 100
        return similarity < threshold, similarity, avg_rgb


class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("使用说明")
        self.resize(900, 700)
        self.setStyleSheet(
            """
            QDialog { background-color: #1e1e2e; }
            QTabWidget { background-color: #1e1e2e; }
            QTabWidget::pane { border: 1px solid #3b82f6; border-radius: 5px; background-color: #2a2a3a; }
            QTabBar::tab { background-color: #2a2a3a; color: white; padding: 8px 15px; border-top-left-radius: 5px; border-top-right-radius: 5px; margin-right: 2px; font-size: 14px; }
            QTabBar::tab:selected { background-color: #3b82f6; font-weight: bold; }
            QTextEdit { background-color: #2a2a3a; color: #e0e0e0; border: none; font-size: 16px; font-family: Microsoft YaHei; }
            QPushButton { background-color: #3b82f6; color: white; border-radius: 12px; padding: 10px; font-size: 16px; font-family: Microsoft YaHei; min-height: 40px; margin: 10px; }
            QPushButton:hover { background-color: #60a5fa; }
            """
        )

        layout = QVBoxLayout()
        tab_widget = QTabWidget()

        intro_tab = QWidget()
        intro_layout = QVBoxLayout()
        intro_text = QTextEdit()
        intro_text.setReadOnly(True)
        intro_text.setHtml(
            """
            <h2>通用上下电压测工具介绍</h2>
            <p>本工具用于电子设备的自动上下电测试，通过串口控制继电器模拟上下电，并可通过摄像头捕获屏幕画面进行异常检测。</p>
            <ul>
              <li>支持4路继电器通道控制</li>
              <li>多路并行控制</li>
              <li>屏幕区域选择与异常检测</li>
              <li>通道独立时间</li>
              <li>仅上下电模式</li>
              <li>异常时连续拍照</li>
              <li>毫秒级时间控制</li>
            </ul>
            """
        )
        intro_layout.addWidget(intro_text)
        intro_tab.setLayout(intro_layout)

        usage_tab = QWidget()
        usage_layout = QVBoxLayout()
        usage_text = QTextEdit()
        usage_text.setReadOnly(True)
        usage_text.setHtml(
            """
            <h2>操作说明</h2>
            <ol>
              <li>选择串口与通道</li>
              <li>设定时间参数</li>
              <li>选择屏幕区域并拍摄参考颜色</li>
              <li>开始测试</li>
            </ol>
            <p>当检测到异常时，系统会停止测试循环，保持设备上电（不执行下电），并按照设置在后台进行连续拍照。</p>
            """
        )
        usage_layout.addWidget(usage_text)
        usage_tab.setLayout(usage_layout)

        tab_widget.addTab(intro_tab, "基本介绍")
        tab_widget.addTab(usage_tab, "操作说明")

        layout.addWidget(tab_widget)
        close_btn = QPushButton("关闭")
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignCenter)
        self.setLayout(layout)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("通用上下电压测工具 v2.5")
        self.setMinimumSize(1000, 680)
        try:
            self.setWindowIcon(QIcon("icon.ico"))
        except Exception:
            pass

        screen = QApplication.primaryScreen()
        dpi = screen.logicalDotsPerInch() if screen else 96
        self.ui_scale = dpi / 96.0

        base_font = QFont("Microsoft YaHei")
        base_font.setPointSizeF(10 * self.ui_scale)
        QApplication.instance().setFont(base_font)

        self.setStyleSheet(
            f"""
            QMainWindow {{ background-color: #12121a; }}
            QWidget {{ color: #e6eef8; font-family: 'Microsoft YaHei','Helvetica','Arial'; }}
            QPushButton {{ background-color: #2563eb; color: white; border-radius: 8px; padding: {int(8*self.ui_scale)}px {int(10*self.ui_scale)}px; font-size: {int(13*self.ui_scale)}px; min-height: {int(34*self.ui_scale)}px; }}
            QPushButton:hover {{ background-color: #3b82f6; }}
            QPushButton:disabled {{ background-color: #444b55; color: #9aa4b2; }}
            QComboBox, QLineEdit {{ background-color: #151521; color: #e6eef8; border: 1px solid #2b6df6; padding: {int(6*self.ui_scale)}px; border-radius: 6px; font-size: {int(13*self.ui_scale)}px; min-height: {int(30*self.ui_scale)}px; }}
            QLabel {{ color: #e6eef8; font-size: {int(13*self.ui_scale)}px; }}
            QLabel#statusLabel {{ color: #34d399; font-size: {int(14*self.ui_scale)}px; font-weight: bold; }}
            QLabel#errorLabel {{ color: #fb7185; font-size: {int(14*self.ui_scale)}px; font-weight: bold; }}
            QProgressBar {{ border: 1px solid #2b6df6; border-radius: 6px; background-color: #0f0f14; text-align: center; font-size: {int(13*self.ui_scale)}px; color: #e6eef8; min-height: {int(24*self.ui_scale)}px; }}
            QProgressBar::chunk {{ background-color: #2b6df6; border-radius: 5px; }}
            QTextEdit {{ background-color: #0f0f14; color: #dfe8f8; border: 1px solid #243b6f; border-radius: 6px; font-size: {int(13*self.ui_scale)}px; font-family: 'Consolas','Microsoft YaHei'; }}
            QScrollArea {{ background-color: transparent; border: none; }}
            QScrollBar:vertical {{ border: none; background: #0f0f14; width: {int(12*self.ui_scale)}px; margin: 0px; }}
            QScrollBar::handle:vertical {{ background: #2b6df6; min-height: {int(30*self.ui_scale)}px; border-radius: 6px; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: transparent; }}
            QMessageBox {{ background-color: #12121a; color: #e6eef8; font-size: {int(13*self.ui_scale)}px; }}
            QTableWidget {{ background-color: #0f0f14; color: #e6eef8; border: 1px solid #2b6df6; border-radius: 6px; font-size: {int(13*self.ui_scale)}px; gridline-color: #2f3540; }}
            QHeaderView::section {{ background-color: #2b6df6; color: white; padding: 6px; font-size: {int(13*self.ui_scale)}px; border: none; }}
            QFrame#separator {{ background-color: #2b6df6; max-height: 1px; min-height: 1px; }}
            QGroupBox {{ color: #e6eef8; font-size: {int(13*self.ui_scale)}px; font-weight: bold; border: 1px solid #2b6df6; border-radius: 6px; margin-top: 12px; padding-top: 12px; }}
            QCheckBox {{ color: #e6eef8; font-size: {int(13*self.ui_scale)}px; spacing: 8px; }}
            QCheckBox::indicator {{ width: {int(18*self.ui_scale)}px; height: {int(18*self.ui_scale)}px; }}
            QPushButton#startButton {{ background-color: #10b981; font-weight: bold; font-size: {int(14*self.ui_scale)}px; }}
            QPushButton#startButton:hover {{ background-color: #34d399; }}
            QPushButton#stopButton {{ background-color: #ef4444; font-weight: bold; font-size: {int(14*self.ui_scale)}px; }}
            QPushButton#stopButton:hover {{ background-color: #f87171; }}
            QPushButton#clearButton {{ background-color: #f97316; }}
            QPushButton#clearButton:hover {{ background-color: #fb923c; }}
            """
        )

        self.ser = None
        self.camera_thread = None
        self.test_thread = None
        self.continuous_capture_thread = None

        self.setup_ui()
        self.load_regions()
        self.load_channel_settings()
        QApplication.instance().setWheelScrollLines(3)
        self.update_serial_ports()

    # ---------------- UI ----------------
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_splitter = QSplitter(Qt.Horizontal)
        main_layout = QHBoxLayout(central_widget)
        main_layout.addWidget(main_splitter)
        main_layout.setContentsMargins(8, 8, 8, 8)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_scroll.setMinimumWidth(int(340 * self.ui_scale))
        left_scroll.setMaximumWidth(int(700 * self.ui_scale))

        left_panel = QWidget()
        left_panel.setStyleSheet("background-color: transparent;")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(
            int(16 * self.ui_scale), int(16 * self.ui_scale), int(12 * self.ui_scale), int(16 * self.ui_scale)
        )
        left_layout.setSpacing(int(12 * self.ui_scale))

        top_layout = QHBoxLayout()
        title_label = QLabel("通用上下电压测工具")
        title_label_font = QFont()
        title_label_font.setPointSizeF(16 * self.ui_scale)
        title_label_font.setBold(True)
        title_label.setFont(title_label_font)
        top_layout.addWidget(title_label)

        version_label = QLabel("v2.5")
        version_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        top_layout.addWidget(version_label)
        left_layout.addLayout(top_layout)

        separator = QFrame()
        separator.setObjectName("separator")
        separator.setFrameShape(QFrame.HLine)
        left_layout.addWidget(separator)

        serial_group = QGroupBox("串口设置")
        serial_layout = QGridLayout()
        serial_layout.setVerticalSpacing(8)
        serial_layout.setHorizontalSpacing(8)

        serial_layout.addWidget(QLabel("串口设备:"), 0, 0)
        self.serial_combo = QComboBox()
        serial_layout.addWidget(self.serial_combo, 0, 1, 1, 2)

        serial_layout.addWidget(QLabel("波特率:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems([str(baud) for baud in BAUD_RATES])
        self.baud_combo.setCurrentText("9600")
        serial_layout.addWidget(self.baud_combo, 1, 1, 1, 2)

        self.refresh_button = QPushButton("刷新串口")
        serial_layout.addWidget(self.refresh_button, 2, 0, 1, 3)
        self.refresh_button.clicked.connect(self.refresh_devices)

        serial_group.setLayout(serial_layout)
        left_layout.addWidget(serial_group)

        time_group = QGroupBox("时间设置")
        time_layout = QGridLayout()
        time_layout.setVerticalSpacing(8)
        time_layout.setHorizontalSpacing(8)

        time_layout.addWidget(QLabel("上电延迟范围:"), 0, 0)
        self.startup_delay_range_input = QLineEdit("6000-8000")
        time_layout.addWidget(self.startup_delay_range_input, 0, 1)
        time_layout.addWidget(QLabel("(毫秒)"), 0, 2)

        time_layout.addWidget(QLabel("下电等待范围:"), 1, 0)
        self.shutdown_delay_range_input = QLineEdit("1000-3000")
        time_layout.addWidget(self.shutdown_delay_range_input, 1, 1)
        time_layout.addWidget(QLabel("(毫秒)"), 1, 2)

        time_group.setLayout(time_layout)
        left_layout.addWidget(time_group)

        detect_group = QGroupBox("检测参数")
        detect_layout = QGridLayout()
        detect_layout.setVerticalSpacing(8)
        detect_layout.setHorizontalSpacing(8)

        detect_layout.addWidget(QLabel("区域数量:"), 0, 0)
        self.region_count_input = QLineEdit("2")
        self.region_count_input.setValidator(QIntValidator(1, 10))
        detect_layout.addWidget(self.region_count_input, 0, 1)

        detect_layout.addWidget(QLabel("相似度阈值:"), 1, 0)
        self.threshold_input = QLineEdit(str(COLOR_THRESHOLD_DEFAULT))
        self.threshold_input.setValidator(QIntValidator(0, 100))
        detect_layout.addWidget(self.threshold_input, 1, 1)
        detect_layout.addWidget(QLabel("%"), 1, 2)

        detect_group.setLayout(detect_layout)
        left_layout.addWidget(detect_group)

        capture_group = QGroupBox("异常检测连续拍照设置")
        capture_layout = QGridLayout()
        capture_layout.setVerticalSpacing(8)
        capture_layout.setHorizontalSpacing(8)

        capture_layout.addWidget(QLabel("拍照间隔(毫秒):"), 0, 0)
        self.error_capture_interval = QLineEdit("500")
        self.error_capture_interval.setValidator(QIntValidator(50, 10000))
        capture_layout.addWidget(self.error_capture_interval, 0, 1)

        capture_layout.addWidget(QLabel("持续时间(毫秒):"), 1, 0)
        self.error_capture_duration = QLineEdit("10000")
        self.error_capture_duration.setValidator(QIntValidator(1000, 60000))
        capture_layout.addWidget(self.error_capture_duration, 1, 1)

        capture_group.setLayout(capture_layout)
        left_layout.addWidget(capture_group)

        relay_group = QGroupBox("继电器通道选择")
        relay_layout = QGridLayout()
        relay_layout.setVerticalSpacing(10)
        relay_layout.setHorizontalSpacing(12)

        self.cb_ch1 = QCheckBox("通道 1")
        self.cb_ch2 = QCheckBox("通道 2")
        self.cb_ch3 = QCheckBox("通道 3")
        self.cb_ch4 = QCheckBox("通道 4")
        self.cb_ch1.setChecked(True)

        relay_layout.addWidget(self.cb_ch1, 0, 0)
        relay_layout.addWidget(self.cb_ch2, 0, 1)
        relay_layout.addWidget(self.cb_ch3, 1, 0)
        relay_layout.addWidget(self.cb_ch4, 1, 1)

        relay_group.setLayout(relay_layout)
        left_layout.addWidget(relay_group)

        button_group = QGroupBox("操作控制")
        button_layout = QGridLayout()
        button_layout.setContentsMargins(8, 8, 8, 8)
        button_layout.setHorizontalSpacing(10)
        button_layout.setVerticalSpacing(10)

        self.select_button = QPushButton("选择屏幕区域")
        self.capture_ref_button = QPushButton("拍摄参考颜色")
        self.start_button = QPushButton("开始测试")
        self.start_button.setObjectName("startButton")
        self.stop_button = QPushButton("停止测试")
        self.stop_button.setObjectName("stopButton")
        self.power_only_button = QPushButton("仅上下电控制")
        self.clear_button = QPushButton("清除保存图片")
        self.clear_button.setObjectName("clearButton")

        button_layout.addWidget(self.select_button, 0, 0)
        button_layout.addWidget(self.capture_ref_button, 0, 1)
        button_layout.addWidget(self.start_button, 1, 0)
        button_layout.addWidget(self.stop_button, 1, 1)
        button_layout.addWidget(self.power_only_button, 2, 0)
        button_layout.addWidget(self.clear_button, 2, 1)

        button_group.setLayout(button_layout)
        left_layout.addWidget(button_group)

        self.help_button = QPushButton("使用说明")
        left_layout.addWidget(self.help_button)

        self.status_label = QLabel("就绪")
        self.status_label.setObjectName("statusLabel")
        self.status_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.status_label)

        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        left_layout.addWidget(self.progress_bar)

        left_layout.addStretch(1)
        left_scroll.setWidget(left_panel)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(
            int(12 * self.ui_scale), int(12 * self.ui_scale), int(12 * self.ui_scale), int(12 * self.ui_scale)
        )
        right_layout.setSpacing(int(12 * self.ui_scale))

        channel_time_group = QGroupBox("通道独立时间设置（留空则使用统一时间）")
        channel_time_layout = QVBoxLayout()
        channel_time_layout.setContentsMargins(8, 8, 8, 8)

        self.channel_table = QTableWidget(4, 5)
        self.channel_table.setHorizontalHeaderLabels(
            ["通道", "上电最小时间(ms)", "上电最大时间(ms)", "下电最小时间(ms)", "下电最大时间(ms)"]
        )
        self.channel_table.verticalHeader().setVisible(False)
        self.channel_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.channel_table.setMinimumHeight(int(180 * self.ui_scale))

        for i in range(4):
            channel_item = QTableWidgetItem(f"通道 {i+1}")
            channel_item.setFlags(channel_item.flags() & ~Qt.ItemIsEditable)
            channel_item.setTextAlignment(Qt.AlignCenter)
            self.channel_table.setItem(i, 0, channel_item)
            for j in range(1, 5):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.channel_table.setItem(i, j, item)

        channel_time_layout.addWidget(self.channel_table)
        self.save_settings_button = QPushButton("保存通道设置")
        channel_time_layout.addWidget(self.save_settings_button, alignment=Qt.AlignRight)

        channel_time_group.setLayout(channel_time_layout)
        right_layout.addWidget(channel_time_group)

        log_group = QGroupBox("运行状态日志")
        log_layout = QVBoxLayout()
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.status_display.setMinimumHeight(int(300 * self.ui_scale))
        log_layout.addWidget(self.status_display)
        log_group.setLayout(log_layout)
        right_layout.addWidget(log_group, 1)

        main_splitter.addWidget(left_scroll)
        main_splitter.addWidget(right_widget)
        main_splitter.setStretchFactor(0, 0)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([int(380 * self.ui_scale), int(900 * self.ui_scale)])

        self.select_button.clicked.connect(self.select_regions)
        self.capture_ref_button.clicked.connect(self.capture_reference_colors)
        self.start_button.clicked.connect(self.start_test)
        self.stop_button.clicked.connect(self.stop_test)
        self.power_only_button.clicked.connect(self.power_only_test)
        self.clear_button.clicked.connect(self.clear_images)
        self.help_button.clicked.connect(self.show_help)
        self.save_settings_button.clicked.connect(self.save_channel_settings)

    # ---------------- 功能 ----------------
    def show_help(self):
        help_dialog = HelpDialog(self)
        help_dialog.exec_()

    def append_status(self, message):
        self.status_display.append(message)
        self.status_display.ensureCursorVisible()
        status_text = message.split("] ")[-1] if "] " in message else message
        if any(key in message for key in ["错误", "失败", "异常"]):
            self.status_label.setObjectName("errorLabel")
            self.status_label.setText(status_text)
        else:
            self.status_label.setObjectName("statusLabel")
            self.status_label.setText(status_text)
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def update_serial_ports(self):
        self.serial_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.serial_combo.addItems(ports if ports else ["无可用串口"])
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 串口列表已更新"
        )

    def refresh_devices(self):
        self.update_serial_ports()

    def _selected_channels(self) -> list:
        channels = []
        if self.cb_ch1.isChecked():
            channels.append(1)
        if self.cb_ch2.isChecked():
            channels.append(2)
        if self.cb_ch3.isChecked():
            channels.append(3)
        if self.cb_ch4.isChecked():
            channels.append(4)
        return channels

    def get_channel_time_settings(self):
        channel_times = {}
        for row in range(self.channel_table.rowCount()):
            channel = row + 1
            startup_min = (
                self.channel_table.item(row, 1).text().strip()
                if self.channel_table.item(row, 1)
                else ""
            )
            startup_max = (
                self.channel_table.item(row, 2).text().strip()
                if self.channel_table.item(row, 2)
                else ""
            )
            shutdown_min = (
                self.channel_table.item(row, 3).text().strip()
                if self.channel_table.item(row, 3)
                else ""
            )
            shutdown_max = (
                self.channel_table.item(row, 4).text().strip()
                if self.channel_table.item(row, 4)
                else ""
            )

            if startup_min and startup_max and shutdown_min and shutdown_max:
                try:
                    startup_min_val = float(startup_min)
                    startup_max_val = float(startup_max)
                    shutdown_min_val = float(shutdown_min)
                    shutdown_max_val = float(shutdown_max)
                    if (
                        startup_min_val <= startup_max_val
                        and shutdown_min_val <= shutdown_max_val
                        and startup_min_val >= 0
                        and shutdown_min_val >= 0
                    ):
                        channel_times[channel] = (
                            startup_min_val,
                            startup_max_val,
                            shutdown_min_val,
                            shutdown_max_val,
                        )
                    else:
                        self.append_status(
                            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 警告: 通道{channel}时间设置无效"
                        )
                except ValueError:
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 警告: 通道{channel}时间设置必须为数字"
                    )
        return channel_times

    def save_channel_settings(self):
        channel_times = self.get_channel_time_settings()
        serializable_data = {str(ch): list(times) for ch, times in channel_times.items()}
        try:
            with open("channel_settings.json", "w") as f:
                json.dump(serializable_data, f)
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 通道时间设置已保存"
            )
        except Exception as e:
            self.show_error(f"保存通道设置失败: {e}")

    def load_channel_settings(self):
        try:
            if os.path.exists("channel_settings.json"):
                with open("channel_settings.json", "r") as f:
                    data = json.load(f)
                for channel_str, times in data.items():
                    try:
                        channel = int(channel_str)
                        row = channel - 1
                        if 0 <= row < self.channel_table.rowCount():
                            for col in range(1, 5):
                                if col - 1 < len(times):
                                    self.channel_table.setItem(
                                        row, col, QTableWidgetItem(str(times[col - 1]))
                                    )
                    except ValueError:
                        continue
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已加载通道时间设置"
                )
        except Exception as e:
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 加载通道设置失败: {e}"
            )

    def init_camera(self):
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在初始化摄像头..."
        )
        if self.camera_thread:
            self.camera_thread.stop()
        self.camera_thread = CameraThread()
        self.camera_thread.camera_error.connect(self.show_error)
        self.camera_thread.camera_ready.connect(
            lambda: self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 摄像头已初始化"
            )
        )
        self.camera_thread.start()

    def init_serial(self):
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在初始化串口..."
        )
        port = self.serial_combo.currentText()
        if port == "无可用串口":
            self.show_error("无可用串口，请检查设备连接")
            return False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 尝试打开串口 {port}，波特率 {self.baud_combo.currentText()}，第 {attempt + 1} 次"
                )
                self.ser = serial.Serial(
                    port, int(self.baud_combo.currentText()), timeout=1, write_timeout=1
                )
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 串口已连接"
                )
                return True
            except serial.SerialException as e:
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 错误初始化失败: {e}"
                )
                if attempt == max_attempts - 1:
                    self.show_error(f"串口被占用或无法访问: {e}")
                    return False
                qt_wait_ms(2000)
        return True

    def load_regions(self):
        global regions
        if os.path.exists("regions.txt"):
            try:
                with open("regions.txt", "r") as f:
                    data = json.load(f)
                    regions = data.get("regions", [])
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已加载 {len(regions)} 个区域"
                    )
            except Exception as e:
                self.show_error(f"加载区域失败: {e}")

    def save_regions(self):
        try:
            with open("regions.txt", "w") as f:
                json.dump({"regions": regions}, f)
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 区域已保存"
            )
        except Exception as e:
            self.show_error(f"保存区域失败: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        global drawing, start_point, end_point, regions, regions_selected
        try:
            region_count = int(self.region_count_input.text().strip())
            if region_count <= 0:
                raise ValueError
        except ValueError:
            self.show_error("区域数量必须为有效的正整数")
            cv2.destroyWindow("Select Regions")
            return
        if regions_selected or len(regions) >= region_count:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            start_point = (x, y)
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在绘制矩形框..."
            )
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            end_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            end_point = (x, y)
            x1, y1 = start_point
            x2, y2 = end_point
            x, y = min(x1, x2), min(y1, y2)
            w, h = abs(x2 - x1), abs(y2 - y1)
            if w > 0 and h > 0:
                regions.append((x, y, w, h))
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已选择 {len(regions)}/{region_count} 个区域"
                )
                if len(regions) >= region_count:
                    regions_selected = True
                    self.save_regions()
                    cv2.setMouseCallback("Select Regions", lambda *args: None)
                    cv2.destroyWindow("Select Regions")
                    self.send_power_off()
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 区域选择完成"
                    )
            else:
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无效的区域选择，请重新绘制"
                )

    def _channels_or_error(self):
        chs = self._selected_channels()
        if not chs:
            self.show_error("请至少选择一个继电器通道")
            return None
        return chs

    def send_power_on(self):
        chs = self._channels_or_error()
        if chs is None:
            return False
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在发送上电指令...(通道{','.join(map(str, chs))})"
        )
        if not self.ser or not self.ser.is_open:
            if not self.init_serial():
                return False
        try:
            for ch in chs:
                self.ser.write(RELAY_CMDS[ch]["on"])
                qt_wait_ms(100)
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] (通道{','.join(map(str, chs))}) 开关已上电"
            )
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设备已上电，等待屏幕稳定..."
            )
            return True
        except Exception as e:
            self.show_error(f"上电失败: {e}")
            return False

    def send_power_off(self):
        chs = self._selected_channels()
        if not chs:
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 未选择通道，跳过下电"
            )
            return
        if not self.ser or not self.ser.is_open:
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 串口未连接，跳过下电"
            )
            return
        try:
            for ch in chs:
                self.ser.write(RELAY_CMDS[ch]["off"])
                qt_wait_ms(100)
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] (通道{','.join(map(str, chs))}) 开关已下电"
            )
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 设备已下电"
            )
        except Exception as e:
            self.show_error(f"下电失败: {e}")
        finally:
            if self.ser and self.ser.is_open:
                try:
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 关闭串口 {self.ser.port}"
                    )
                    self.ser.close()
                except Exception as e:
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 关闭串口失败: {e}"
                    )
                self.ser = None

    def select_regions(self):
        try:
            region_count = int(self.region_count_input.text().strip())
            startup_min_ms, startup_max_ms = parse_range_to_ms_pair(
                self.startup_delay_range_input.text()
            )
            if region_count <= 0:
                raise ValueError("区域数量必须为正整数")
        except ValueError as e:
            self.show_error(str(e) if str(e) else "区域数量和拍照延迟范围必须为有效数字")
            return

        if not self.send_power_on():
            return
        rand_ms = random.uniform(startup_min_ms, startup_max_ms)
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 等待 {rand_ms:.0f} ms 以稳定屏幕..."
        )
        qt_wait_ms(int(rand_ms))

        self.init_camera()
        self.camera_thread.wait()
        if not camera_open():
            self.send_power_off()
            return

        global regions, regions_selected, drawing, start_point, end_point
        regions = []
        regions_selected = False
        drawing = False
        start_point = None
        end_point = None

        cv2.namedWindow("Select Regions")
        cv2.resizeWindow("Select Regions", 1280, 720)
        cv2.setMouseCallback("Select Regions", self.mouse_callback)
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 请依次选择屏幕区域：点击并拖动鼠标绘制矩形框"
        )

        while len(regions) < region_count and cv2.getWindowProperty("Select Regions", cv2.WND_PROP_VISIBLE) > 0:
            ret, frame = camera_read()
            if not ret or frame is None:
                self.show_error("无法捕获画面")
                break
            display_frame = frame.copy()
            if drawing and start_point and end_point:
                cv2.rectangle(display_frame, start_point, end_point, (0, 255, 0), 2)
            for i, region in enumerate(regions):
                x, y, w, h = region
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(
                    display_frame,
                    f"Region {i+1}",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )
            cv2.imshow("Select Regions", display_frame)
            cv2.waitKey(1)

        if len(regions) < region_count:
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 区域选择未完成"
            )
            regions = []
        self.camera_thread.stop()
        cv2.destroyAllWindows()
        self.send_power_off()

    def capture_reference_colors(self):
        try:
            region_count = int(self.region_count_input.text().strip())
            startup_min_ms, startup_max_ms = parse_range_to_ms_pair(
                self.startup_delay_range_input.text()
            )
            if region_count <= 0:
                raise ValueError("区域数量必须为正整数")
        except ValueError as e:
            self.show_error(str(e) if str(e) else "区域数量和拍照延迟范围必须为有效数字")
            return
        if len(regions) != region_count:
            self.show_error(f"请先选择 {region_count} 个框选区域")
            return

        if not self.send_power_on():
            return
        rand_ms = random.uniform(startup_min_ms, startup_max_ms)
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 等待 {rand_ms:.0f} ms 以稳定屏幕..."
        )
        qt_wait_ms(int(rand_ms))

        self.init_camera()
        self.camera_thread.wait()
        if not camera_open():
            self.send_power_off()
            return

        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在清除摄像头缓冲..."
        )
        camera_flush(5, 10)
        ret, frame = camera_read()
        if not ret or frame is None:
            self.show_error("无法捕获参考画面")
            self.camera_thread.stop()
            self.send_power_off()
            return

        global reference_colors
        reference_colors = []
        for i, region in enumerate(regions):
            x, y, w, h = region
            roi = frame[y : y + h, x : x + w]
            avg_rgb = np.mean(roi, axis=(0, 1))
            reference_colors.append(avg_rgb)
            self.append_status(
                f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 区域{i+1} 参考颜色 RGB: {avg_rgb}"
            )

        try:
            os.makedirs(reference_image_folder, exist_ok=True)
            filename = os.path.join(reference_image_folder, "reference.png")
            cv2.imwrite(filename, frame)
        except Exception:
            filename = ""

        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 参考颜色已捕获，保存至 {filename}"
        )
        self.camera_thread.stop()
        self.send_power_off()

    def power_only_test(self):
        global running
        if running:
            self.show_error("测试已在运行")
            return
        try:
            startup_min_ms, startup_max_ms = parse_range_to_ms_pair(
                self.startup_delay_range_input.text()
            )
            shutdown_min_ms, shutdown_max_ms = parse_range_to_ms_pair(
                self.shutdown_delay_range_input.text()
            )
        except ValueError as e:
            self.show_error(str(e) if str(e).startswith("延迟范围") or str(e) else "请输入有效的时间参数")
            return

        relay_channels = self._selected_channels()
        if not relay_channels:
            self.show_error("请至少选择一个继电器通道")
            return

        if not self.init_serial():
            return

        running = True
        channel_times = self.get_channel_time_settings()

        self.test_thread = TestThread(
            self,
            self.serial_combo.currentText(),
            int(self.baud_combo.currentText()),
            (startup_min_ms, startup_max_ms),
            (shutdown_min_ms, shutdown_max_ms),
            0,
            0,
            relay_channels,
            channel_times=channel_times,
            capture_image=False,
        )
        self.test_thread.test_error.connect(self.show_error)
        self.test_thread.test_result.connect(self.append_status)
        self.test_thread.start()

    def handle_abnormal_detection(self, area_id, error_image, similarity):
        if not os.path.exists(error_images_folder):
            os.makedirs(error_images_folder, exist_ok=True)

        # 等待测试线程退出主循环，避免并发读取
        def _start_capture_when_ready():
            if self.test_thread and self.test_thread.isRunning():
                QTimer.singleShot(100, _start_capture_when_ready)
                return
            try:
                interval_ms = int(self.error_capture_interval.text())
                duration_ms = int(self.error_capture_duration.text())
                if interval_ms < 50:
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 拍照间隔太短，最小为50毫秒"
                    )
                    return
                if duration_ms < 1000:
                    self.append_status(
                        f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 拍照持续时间太短，最小为1000毫秒"
                    )
                    return
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 检测到屏幕区域 {area_id} 异常，相似度 {similarity:.1f}%，开始连续拍照"
                )
                self.continuous_capture_thread = CaptureErrorThread(
                    interval_ms, duration_ms, error_image
                )
                self.continuous_capture_thread.capture_status.connect(self.append_status)
                self.continuous_capture_thread.capture_completed.connect(self.on_capture_completed)
                self.continuous_capture_thread.start()
            except ValueError:
                self.append_status(
                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无效的连续拍照设置参数"
                )

        QTimer.singleShot(0, _start_capture_when_ready)

    def on_capture_completed(self):
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 异常连续拍照完成"
        )

    def start_test(self):
        global running
        if running:
            self.show_error("测试已在运行")
            return
        try:
            region_count = int(self.region_count_input.text().strip())
            startup_min_ms, startup_max_ms = parse_range_to_ms_pair(
                self.startup_delay_range_input.text()
            )
            shutdown_min_ms, shutdown_max_ms = parse_range_to_ms_pair(
                self.shutdown_delay_range_input.text()
            )
            threshold = int(self.threshold_input.text().strip())
            if region_count <= 0:
                raise ValueError("区域数量必须为正整数")
        except ValueError as e:
            self.show_error(str(e) if str(e).startswith("延迟范围") or str(e) else "请输入有效的参数")
            return
        if len(regions) != region_count:
            self.show_error(f"请先选择 {region_count} 个框选区域")
            return
        if not reference_colors:
            self.show_error("请先拍摄参考颜色")
            return
        relay_channels = self._selected_channels()
        if not relay_channels:
            self.show_error("请至少选择一个继电器通道")
            return

        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在初始化摄像头..."
        )
        self.init_camera()
        self.camera_thread.wait()
        if not camera_open():
            self.show_error("摄像头初始化失败")
            return

        if not self.init_serial():
            if self.camera_thread:
                self.camera_thread.stop()
            return

        running = True
        channel_times = self.get_channel_time_settings()

        self.test_thread = TestThread(
            self,
            self.serial_combo.currentText(),
            int(self.baud_combo.currentText()),
            (startup_min_ms, startup_max_ms),
            (shutdown_min_ms, shutdown_max_ms),
            region_count,
            threshold,
            relay_channels,
            channel_times=channel_times,
        )
        self.test_thread.test_error.connect(self.show_error)
        self.test_thread.test_result.connect(self.append_status)
        self.test_thread.abnormal_detected.connect(self.handle_abnormal_detection)
        self.test_thread.start()

    def stop_test(self):
        global running
        running = False
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 测试已停止"
        )
        if hasattr(self, "test_thread") and self.test_thread:
            try:
                self.test_thread._stop_event.set()
            except Exception:
                pass
            self.test_thread.wait()
        if self.camera_thread:
            self.camera_thread.stop()
        self.send_power_off()

    def clear_images(self):
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 正在清除图片..."
        )
        self.progress_bar.setVisible(True)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)

        total_files = 0
        cleared_files = 0
        folders_to_check = [image_folder, reference_image_folder, error_images_folder]

        for folder in folders_to_check:
            if os.path.exists(folder):
                if os.path.isdir(folder):
                    total_files += len(
                        [f for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
                    )
                    for root, dirs, files in os.walk(folder):
                        if root != folder:
                            total_files += len(files)

        for folder in folders_to_check:
            if os.path.exists(folder):
                try:
                    for root, dirs, files in os.walk(folder, topdown=False):
                        for file in files:
                            file_path = os.path.join(root, file)
                            try:
                                os.remove(file_path)
                                cleared_files += 1
                                if total_files > 0:
                                    progress = int((cleared_files / total_files) * 100)
                                    self.progress_bar.setValue(progress)
                            except Exception as e:
                                self.append_status(
                                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法删除文件 {file_path}: {e}"
                                )
                        if root != folder:
                            try:
                                os.rmdir(root)
                            except Exception as e:
                                self.append_status(
                                    f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法删除目录 {root}: {e}"
                                )
                    try:
                        os.rmdir(folder)
                        self.append_status(
                            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {folder} 文件夹已清除"
                        )
                    except Exception as e:
                        self.append_status(
                            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 无法删除主目录 {folder}: {e}"
                        )
                except Exception as e:
                    self.show_error(f"清除 {folder} 失败: {e}")

        self.progress_bar.setValue(100)
        qt_wait_ms(500)
        self.progress_bar.setVisible(False)
        self.append_status(
            f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 已清除 {cleared_files} 个图片文件"
        )

    def show_error(self, message):
        if not message:
            message = "未知错误，请检查设备连接或程序日志"
        if not message.startswith("["):
            message = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] 错误: {message}"
        msg_box = QMessageBox()
        msg_box.setWindowTitle("错误")
        msg_box.setText(message.split("] ")[-1] if "] " in message else message)
        msg_box.exec_()
        self.append_status(message)

    def closeEvent(self, event):
        global running
        running = False
        # 停止测试线程
        if hasattr(self, "test_thread") and self.test_thread:
            try:
                self.test_thread._stop_event.set()
            except Exception:
                pass
            self.test_thread.wait()
        # 停止连续拍照线程
        if hasattr(self, "continuous_capture_thread") and self.continuous_capture_thread:
            try:
                self.continuous_capture_thread.stop()
                self.continuous_capture_thread.wait()
            except Exception:
                pass
        # 关闭相机
        if self.camera_thread:
            try:
                self.camera_thread.stop()
            except Exception:
                pass
        camera_close()
        # 关闭串口
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        event.accept()


if __name__ == "__main__":
    with WinHighResTimer():
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())
