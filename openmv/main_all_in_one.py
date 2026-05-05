import time

import image
import sensor
from pyb import LED
from pyb import UART
from pyb import millis

try:
    import display
except ImportError:
    display = None

try:
    import lcd
except ImportError:
    lcd = None


class config:
    # OpenMV 图像采集、识别、串口输出的全局配置。
    # 图像分辨率，QQVGA 计算量更小，适合高帧率识别。
    FRAME_SIZE = "QQQVGA"
    # 像素格式，RGB565 便于做颜色阈值检测。
    PIXFORMAT = "RGB565"

    # 是否锁定自动增益，锁定后颜色阈值更稳定。
    LOCK_AUTO_GAIN = True
    # 是否锁定自动白平衡，锁定后环境颜色变化影响更小。
    LOCK_AUTO_WHITEBAL = True
    # 是否锁定曝光；为 True 时使用下面的固定曝光值。
    LOCK_AUTO_EXPOSURE = False
    # 固定曝光时间，单位 us。
    EXPOSURE_US = 12000

    # OpenMV 使用的串口号。
    UART_PORT = 3
    # 串口波特率，需要和 STM32 端一致。
    UART_BAUDRATE = 115200
    # 串口字符超时，影响底层发送等待行为。
    UART_TIMEOUT_CHAR = 2

    # 是否绘制识别框、十字等调试图形。
    DRAW_DEBUG = True
    # 是否在图像左上角绘制状态文字。
    DRAW_STATUS_BAR = True
    # 是否打开补光灯。
    FILL_LIGHT_ENABLE = True
    # 打开 RGB 三色灯，避免误开红外灯导致画面偏紫。
    FILL_LIGHT_LED_IDS = (1, 2, 3)
    # 串口发送周期，单位 ms。
    SEND_FRAME_INTERVAL_MS = 30

    # 是否把当前识别画面同步显示到外接屏幕。
    DISPLAY_ENABLE = True
    # 优先尝试的显示后端：AUTO / RGB / SPI。
    # OpenMV 官方 SPI LCD 建议固定为 SPI，避免先探测 RGB 屏浪费时间。
    DISPLAY_PREFERRED = "SPI"
    # RGB 屏输出方向；竖屏安装时改成 True。
    DISPLAY_PORTRAIT = False
    # 屏幕刷新率，过高会增加带宽占用。
    DISPLAY_REFRESH = 30
    # 背光亮度百分比，0~100。
    DISPLAY_BACKLIGHT_PERCENT = 100
    # SPI 屏参数；只有回退到 SPI 显示时才会使用。
    DISPLAY_SPI_WIDTH = 128
    DISPLAY_SPI_HEIGHT = 160
    DISPLAY_SPI_BGR = False
    DISPLAY_SPI_BYTE_SWAP = False

    # 目标短暂丢失后继续保留的时间，单位 ms。
    TRACK_HOLD_MS = 120
    # 平滑系数，越大越跟随新值，越小越平稳。
    TRACK_SMOOTH_ALPHA = 0.35

    # 橙色圆的 LAB 阈值。
    ORANGE_THRESHOLDS = [
    (20, 75, 18, 65, 20, 75),
    (28, 40, 12, 61, -8, 47),
    (28, 49, -1, 26, 26, 37),
    (48, 70, 2, 24, 8, 38),
    (73, 81, 6, 23, 13, 34),
    (72, 87, 1, 18, 13, 53),
    (18, 82, 3, 32, 1, 20),
    ]

    # 火焰的 LAB 阈值，可同时覆盖不同亮度和色温的火焰区域。
    FLAME_THRESHOLDS = [
        (18, 95, 20, 80, 15, 80),
        (35, 100, -10, 30, 20, 80),
        (17, 35, 16, 39, 0, 53),
        (32, 53, 29, 64, 52, -20),
        (21, 39, 6, 26, -5, 44),
    ]

    # find_blobs 的像素数门限，过小的色块直接忽略。
    ORANGE_PIXELS_THRESHOLD = 120
    # find_blobs 的面积门限。
    ORANGE_AREA_THRESHOLD = 120
    # 色块合并边距，允许相邻区域合并成一个目标。
    ORANGE_MERGE_MARGIN = 8
    # 目标最小宽度。
    ORANGE_MIN_WIDTH = 10
    # 目标最小高度。
    ORANGE_MIN_HEIGHT = 10
    # 外接框长宽比下限，限制形状不要太扁。
    ORANGE_ASPECT_MIN = 0.82
    # 外接框长宽比上限，限制形状不要太长。
    ORANGE_ASPECT_MAX = 1.22
    # 密度下限，过滤内部过空的噪声区域。
    ORANGE_DENSITY_MIN = 0.58
    # 圆点允许的最大细长度，越小越接近圆。
    ORANGE_ELONGATION_MAX = 0.45
    # 圆点最小圆形度，用来和火焰做互斥。
    ORANGE_ROUNDNESS_MIN = 0.78
    # 圆点内部亮度标准差上限，过于斑驳的暖色块更像火焰。
    ORANGE_L_STDDEV_MAX = 30
    # 圆点上被激光笔打到时，允许更高一点的亮度起伏，但仍限制在可控范围内。
    ORANGE_L_STDDEV_WITH_LASER_MAX = 46
    # 圆点和上一帧形状越接近，越倾向继续判成圆点。
    ORANGE_STABLE_BONUS_BASE = 0.60
    # 圆点平均 b 值需要保持足够偏黄橙，避免纯红激光点混入。
    ORANGE_WARM_SCORE_MIN = 0.34
    # 只有足够大、足够圆、整体仍偏橙的候选，才放宽激光干扰下的纹理限制。
    ORANGE_LASER_TOLERANT_AREA = 180
    ORANGE_LASER_TOLERANT_CIRCLE_SCORE = 0.82
    ORANGE_LASER_TOLERANT_WARM_SCORE = 0.55
    # 上一帧已经确认是圆点时，如果当前位置突然被判成“近圆的火焰块”，优先拉回圆点。
    ORANGE_HISTORY_RECLASSIFY_DISTANCE = 24
    ORANGE_HISTORY_RECLASSIFY_AREA_MIN_RATIO = 0.45
    ORANGE_HISTORY_RECLASSIFY_AREA_MAX_RATIO = 2.80
    ORANGE_HISTORY_RECLASSIFY_CIRCLE_SCORE = 0.68
    ORANGE_HISTORY_RECLASSIFY_WARM_SCORE = 0.10
    # 平台快速移动时，首帧看到圆点可能已经被激光染红；此时只要整体仍明显更像圆实体，
    # 也要优先按圆点处理，避免还没建立历史就被火焰分支抢走。
    ORANGE_RED_CIRCLE_AREA_MIN = 150
    ORANGE_RED_CIRCLE_CIRCLE_SCORE = 0.72
    ORANGE_RED_CIRCLE_DENSITY_MIN = 0.52
    ORANGE_RED_CIRCLE_ROUNDNESS_MIN = 0.55
    ORANGE_RED_CIRCLE_TEXTURE_MAX = 0.42

    # 火焰检测的像素数门限。
    FLAME_PIXELS_THRESHOLD = 80
    # 火焰检测的面积门限。
    FLAME_AREA_THRESHOLD = 80
    # 火焰色块合并边距。
    FLAME_MERGE_MARGIN = 12
    # 火焰目标最小宽度。
    FLAME_MIN_WIDTH = 8
    # 火焰目标最小高度。
    FLAME_MIN_HEIGHT = 8
    # 火焰外接框长宽比下限。
    FLAME_ASPECT_MIN = 0.45
    # 火焰外接框长宽比上限。
    FLAME_ASPECT_MAX = 1.80
    # 火焰密度下限。
    FLAME_DENSITY_MIN = 0.20
    # 火焰细长度上限，过于细长时视为噪声。
    FLAME_ELONGATION_MAX = 0.92
    # 火焰中心点距离画面边缘过近时直接忽略，避免把白纸外杂点当成目标。
    FLAME_EDGE_MARGIN_X = 14
    FLAME_EDGE_MARGIN_Y = 10
    # 火焰新目标需要连续检测到几帧才确认，抑制单帧闪烁误判。
    FLAME_CONFIRM_FRAMES = 2
    # 连续确认时允许的中心点最大跳变像素。
    FLAME_CONFIRM_DISTANCE = 16
    # 火焰内部亮度标准差下限，内部过于均匀的暖色块更可能是圆点。
    FLAME_L_STDDEV_MIN = 8
    # 火焰默认保留少量动态分，避免首次出现时因为没有历史而被压制。
    FLAME_FLICKER_BASE = 0.25
    # 火焰整体仍应带一定黄橙分量；太偏纯红且面积又小，更像激光点。
    FLAME_WARM_SCORE_MIN = 0.12
    FLAME_LASER_REJECT_AREA_MAX = 260

    # 红色激光点常表现为：面积较小、很亮、a 分量高、b 分量不高、形状偏紧凑。
    LASER_AREA_MAX = 260
    LASER_MAX_WIDTH = 20
    LASER_MAX_HEIGHT = 20
    LASER_ASPECT_MIN = 0.40
    LASER_ASPECT_MAX = 2.10
    LASER_DENSITY_MIN = 0.30
    LASER_ROUNDNESS_MIN = 0.45
    LASER_L_MEAN_MIN = 45
    LASER_A_MEAN_MIN = 26
    LASER_B_MEAN_MAX = 18

    # 用 blob 区域平均 b 值估计“偏黄橙”程度；激光点通常 b 值明显低于橙色目标和火焰。
    WARM_B_MEAN_MIN = 0
    WARM_B_MEAN_MAX = 32

    # 两类目标中心过近时，视为同一暖色物体的冲突候选。
    TARGET_CONFLICT_DISTANCE = 18
    # 冲突时分数至少拉开这么多才保留更强的一类，否则按不确定处理。
    TARGET_CONFLICT_SCORE_MARGIN = 10

    # 串口包头字节 0。
    PACKET_HEADER_0 = 0xAA
    # 串口包头字节 1。
    PACKET_HEADER_1 = 0x55

    # 双目标固定帧格式，总长度 27 字节：
    # [0]  0xAA
    # [1]  0x55
    # [2]  seq
    # [3]  flags: bit0=orange_valid, bit1=orange_stale, bit2=flame_valid, bit3=flame_stale
    # [4]  orange_quality
    # [5]  orange_cx low
    # [6]  orange_cx high
    # [7]  orange_cy low
    # [8]  orange_cy high
    # [9]  orange_ex low
    # [10] orange_ex high
    # [11] orange_ey low
    # [12] orange_ey high
    # [13] orange_area low
    # [14] orange_area high
    # [15] flame_quality
    # [16] flame_cx low
    # [17] flame_cx high
    # [18] flame_cy low
    # [19] flame_cy high
    # [20] flame_ex low
    # [21] flame_ex high
    # [22] flame_ey low
    # [23] flame_ey high
    # [24] flame_area low
    # [25] flame_area high
    # [26] crc8


# 目标类型枚举，供检测、跟踪、串口发送共用。
TARGET_NONE = 0
TARGET_ORANGE_CIRCLE = 1
TARGET_FLAME = 2


class VisionTarget:
    # 统一描述当前选中的视觉目标，后续跟踪、显示、串口发送都使用这个结构。
    def __init__(self,
                 kind=TARGET_NONE,
                 valid=False,
                 cx=0,
                 cy=0,
                 ex=0,
                 ey=0,
                 area=0,
                 angle=0,
                 quality=0,
                 stale=False,
                 box_w=0,
                 box_h=0):
        self.kind = kind
        self.valid = valid
        self.cx = int(cx)
        self.cy = int(cy)
        self.ex = int(ex)
        self.ey = int(ey)
        self.area = int(area)
        self.angle = int(angle)
        self.quality = int(quality)
        self.stale = bool(stale)
        self.box_w = int(box_w)
        self.box_h = int(box_h)

    def copy(self):
        return VisionTarget(self.kind,
                            self.valid,
                            self.cx,
                            self.cy,
                            self.ex,
                            self.ey,
                            self.area,
                            self.angle,
                            self.quality,
                            self.stale,
                            self.box_w,
                            self.box_h)


def invalid_target():
    # 返回一个统一的“无目标”对象，避免各处自己造默认值。
    return VisionTarget()


def clamp_i16(value):
    # 限制到 int16 范围，避免打包时溢出。
    if value > 32767:
        return 32767
    if value < -32768:
        return -32768
    return int(value)


def clamp_u16(value):
    # 限制到 uint16 范围。
    if value < 0:
        return 0
    if value > 65535:
        return 65535
    return int(value)


def clamp_u8(value):
    # 限制到 uint8 范围。
    if value < 0:
        return 0
    if value > 255:
        return 255
    return int(value)


def _append_u8(buffer, value):
    # 以 1 字节无符号数写入缓冲区。
    buffer.append(clamp_u8(value))


def _append_u16(buffer, value):
    # 以小端格式写入 2 字节无符号数。
    value = clamp_u16(value)
    buffer.append(value & 0xFF)
    buffer.append((value >> 8) & 0xFF)


def _append_i16(buffer, value):
    # 以小端格式写入 2 字节有符号数，负数按补码编码。
    value = clamp_i16(value)
    if value < 0:
        value = 0x10000 + value
    buffer.append(value & 0xFF)
    buffer.append((value >> 8) & 0xFF)


def _crc8_xor(data):
    # 使用简单异或校验，STM32 侧实现代价低。
    crc = 0
    for byte in data:
        crc ^= (byte & 0xFF)
    return crc & 0xFF


class VisionUart:
    def __init__(self):
        # 初始化串口并维护一个循环递增的序号字段。
        self._uart = UART(config.UART_PORT, config.UART_BAUDRATE, timeout_char=config.UART_TIMEOUT_CHAR)
        self._seq = 0

    def _append_target_payload(self, packet, target):
        _append_u8(packet, target.quality)
        _append_u16(packet, target.cx)
        _append_u16(packet, target.cy)
        _append_i16(packet, target.ex)
        _append_i16(packet, target.ey)
        _append_u16(packet, target.area)

    def send_targets(self, orange_target, flame_target):
        # 按固定双目标协议打包，STM32 侧按顺序取橙色圆和火焰即可。
        packet = bytearray()
        packet.append(config.PACKET_HEADER_0)
        packet.append(config.PACKET_HEADER_1)
        _append_u8(packet, self._seq)

        flags = 0
        if orange_target.valid:
            flags |= 0x01
        if orange_target.stale:
            flags |= 0x02
        if flame_target.valid:
            flags |= 0x04
        if flame_target.stale:
            flags |= 0x08
        _append_u8(packet, flags)

        self._append_target_payload(packet, orange_target)
        self._append_target_payload(packet, flame_target)
        _append_u8(packet, _crc8_xor(packet[2:]))

        self._uart.write(packet)
        self._seq = (self._seq + 1) & 0xFF


class FrameDisplay:
    # 封装外接屏显示，优先适配 H7 Plus 常用的 RGB 屏，失败后再回退到 SPI/lcd 接口。
    def __init__(self):
        self._driver = None
        self._backend = None
        self._hint = image.CENTER | image.SCALE_ASPECT_KEEP
        if config.DISPLAY_ENABLE:
            self._init_display()

    def _set_backlight(self):
        level = max(0, min(100, int(config.DISPLAY_BACKLIGHT_PERCENT)))

        if self._backend == "display" and hasattr(self._driver, "backlight"):
            self._driver.backlight(level)
            return

        if self._backend == "lcd":
            lcd.set_backlight((level * 255) // 100)

    def _try_rgb_display(self):
        if display is None:
            return False

        self._driver = display.RGBDisplay(refresh=config.DISPLAY_REFRESH,
                                          portrait=config.DISPLAY_PORTRAIT)
        self._backend = "display"
        self._set_backlight()
        return True

    def _try_spi_display(self):
        if display is None:
            return False

        self._driver = display.SPIDisplay(width=config.DISPLAY_SPI_WIDTH,
                                          height=config.DISPLAY_SPI_HEIGHT,
                                          refresh=config.DISPLAY_REFRESH,
                                          bgr=config.DISPLAY_SPI_BGR,
                                          byte_swap=config.DISPLAY_SPI_BYTE_SWAP)
        self._backend = "display"
        self._set_backlight()
        return True

    def _try_lcd_display(self):
        if lcd is None:
            return False

        lcd.init(lcd.LCD_DISPLAY, refresh=config.DISPLAY_REFRESH)
        self._driver = lcd
        self._backend = "lcd"
        self._set_backlight()
        return True

    def _try_lcd_spi(self):
        if lcd is None:
            return False

        lcd.init(lcd.LCD_SHIELD,
                 width=config.DISPLAY_SPI_WIDTH,
                 height=config.DISPLAY_SPI_HEIGHT,
                 refresh=config.DISPLAY_REFRESH,
                 bgr=config.DISPLAY_SPI_BGR,
                 byte_reverse=config.DISPLAY_SPI_BYTE_SWAP)
        self._driver = lcd
        self._backend = "lcd"
        self._set_backlight()
        return True

    def _init_display(self):
        preferred = config.DISPLAY_PREFERRED
        if preferred == "RGB":
            attempts = (self._try_rgb_display, self._try_lcd_display)
        elif preferred == "SPI":
            attempts = (self._try_spi_display, self._try_lcd_spi)
        else:
            attempts = (self._try_rgb_display,
                        self._try_lcd_display,
                        self._try_spi_display,
                        self._try_lcd_spi)

        for attempt in attempts:
            try:
                if attempt():
                    return
            except Exception:
                self._driver = None
                self._backend = None

    def show(self, img):
        if self._driver is None:
            return

        try:
            if self._backend == "display":
                self._driver.write(img, hint=self._hint)
            else:
                self._driver.display(img,
                                     x_size=self._driver.width(),
                                     y_size=self._driver.height(),
                                     hint=self._hint)
        except Exception:
            # 显示异常时静默降级，避免影响识别和串口发送主流程。
            self._driver = None
            self._backend = None


def _lerp_int(old_value, new_value, alpha):
    # 线性插值，用于降低检测结果帧间抖动。
    return int(old_value + alpha * (new_value - old_value))


class TargetTracker:
    def __init__(self):
        self._target = invalid_target()
        self._last_update_ms = 0

    def latest(self, now_ms, max_age_ms=None):
        # 读取最近一次确认的目标；超过允许年龄后返回无效，避免拿过期历史做重分类。
        if self._target.valid:
            if max_age_ms is not None and (now_ms - self._last_update_ms) > max_age_ms:
                return invalid_target()
            latest_target = self._target.copy()
            latest_target.stale = False
            return latest_target
        return invalid_target()

    def update(self, new_target, now_ms):
        if new_target is not None and new_target.valid:
            if self._target.valid and self._target.kind == new_target.kind:
                # 同类目标连续出现时做简单平滑，降低检测抖动。
                smoothed = VisionTarget(kind=new_target.kind,
                                        valid=True,
                                        cx=_lerp_int(self._target.cx, new_target.cx, config.TRACK_SMOOTH_ALPHA),
                                        cy=_lerp_int(self._target.cy, new_target.cy, config.TRACK_SMOOTH_ALPHA),
                                        ex=_lerp_int(self._target.ex, new_target.ex, config.TRACK_SMOOTH_ALPHA),
                                        ey=_lerp_int(self._target.ey, new_target.ey, config.TRACK_SMOOTH_ALPHA),
                                        area=_lerp_int(self._target.area, new_target.area, config.TRACK_SMOOTH_ALPHA),
                                        angle=_lerp_int(self._target.angle, new_target.angle, config.TRACK_SMOOTH_ALPHA),
                                        quality=max(self._target.quality, new_target.quality),
                                        stale=False,
                                        box_w=_lerp_int(self._target.box_w, new_target.box_w, config.TRACK_SMOOTH_ALPHA),
                                        box_h=_lerp_int(self._target.box_h, new_target.box_h, config.TRACK_SMOOTH_ALPHA))
                self._target = smoothed
            else:
                self._target = new_target.copy()
                self._target.stale = False

            self._last_update_ms = now_ms
            return self._target.copy()

        if self._target.valid and (now_ms - self._last_update_ms) <= config.TRACK_HOLD_MS:
            # 目标短暂丢失时继续沿用上一帧结果，避免串口数据瞬间清零。
            held_target = self._target.copy()
            held_target.stale = True
            return held_target

        self._target = invalid_target()
        self._last_update_ms = now_ms
        return self._target.copy()


class ConfirmedTargetTracker(TargetTracker):
    # 在普通跟踪基础上增加“连续确认”能力，适合抑制短暂闪现的误检。
    def __init__(self, confirm_frames, confirm_distance):
        TargetTracker.__init__(self)
        self._confirm_frames = confirm_frames
        self._confirm_distance = confirm_distance
        self._candidate = invalid_target()
        self._candidate_count = 0

    def _is_close_target(self, target_a, target_b):
        if (not target_a.valid) or (not target_b.valid):
            return False
        if target_a.kind != target_b.kind:
            return False
        if abs(target_a.cx - target_b.cx) > self._confirm_distance:
            return False
        if abs(target_a.cy - target_b.cy) > self._confirm_distance:
            return False
        return True

    def _remember_candidate(self, target):
        if self._is_close_target(self._candidate, target):
            self._candidate_count += 1
        else:
            self._candidate = target.copy()
            self._candidate_count = 1

    def update(self, new_target, now_ms):
        if self._confirm_frames <= 1:
            return TargetTracker.update(self, new_target, now_ms)

        if new_target is not None and new_target.valid:
            if self._target.valid and self._is_close_target(self._target, new_target):
                self._candidate = new_target.copy()
                self._candidate_count = self._confirm_frames
                return TargetTracker.update(self, new_target, now_ms)

            self._remember_candidate(new_target)
            if self._candidate_count < self._confirm_frames:
                return TargetTracker.update(self, invalid_target(), now_ms)

            confirmed_target = self._candidate.copy()
            return TargetTracker.update(self, confirmed_target, now_ms)

        self._candidate = invalid_target()
        self._candidate_count = 0
        return TargetTracker.update(self, new_target, now_ms)


def _safe_density(blob):
    # 优先使用固件自带 density()，没有时退化为手动估算。
    try:
        return blob.density()
    except Exception:
        rect_area = blob.w() * blob.h()
        if rect_area <= 0:
            return 0.0
        return float(blob.pixels()) / float(rect_area)


def _safe_elongation(blob):
    # 优先使用固件自带 elongation()，没有时用长宽比近似。
    try:
        return blob.elongation()
    except Exception:
        w = blob.w()
        h = blob.h()
        if w <= 0 or h <= 0:
            return 1.0
        short_side = w if w < h else h
        long_side = h if h > w else w
        return 1.0 - (float(short_side) / float(long_side))


def _safe_roundness(blob):
    # 优先使用固件自带 roundness()，没有时退化为基于长宽比的近似值。
    try:
        return blob.roundness()
    except Exception:
        w = blob.w()
        h = blob.h()
        if w <= 0 or h <= 0:
            return 0.0
        short_side = w if w < h else h
        long_side = h if h > w else w
        return float(short_side) / float(long_side)


def _norm_error(value, full_scale):
    # 将像素偏差归一化到约 -1000 到 1000，便于 STM32 直接控制。
    if full_scale <= 0:
        return 0
    return int((1000 * value) / full_scale)


def _blob_area(blob):
    # 优先使用真实像素面积，失败时退化为外接框面积。
    try:
        return int(blob.pixels())
    except Exception:
        return int(blob.w() * blob.h())


def _safe_l_stdev(img, blob):
    # 统计候选框内亮度起伏，圆点通常更均匀，火焰通常更斑驳。
    try:
        stats = img.get_statistics(roi=blob.rect())
        return float(stats.l_stdev())
    except Exception:
        return 0.0


def _safe_blob_stats(img, blob):
    # 统一读取候选框统计量，便于同时使用均值和标准差做判别。
    try:
        return img.get_statistics(roi=blob.rect())
    except Exception:
        return None


def _stats_l_stdev(stats):
    if stats is None:
        return 0.0
    try:
        return float(stats.l_stdev())
    except Exception:
        return 0.0


def _stats_lab_means(stats):
    if stats is None:
        return 0.0, 0.0, 0.0
    try:
        return float(stats.l_mean()), float(stats.a_mean()), float(stats.b_mean())
    except Exception:
        return 0.0, 0.0, 0.0


def _circle_shape_score(aspect, density, elongation, roundness):
    # 将“像圆”的程度压成 0 到 1，便于和纹理、时序信息一起加权。
    aspect_score = 1.0 - min(1.0, abs(1.0 - aspect))
    return max(0.0,
               min(1.0,
                   aspect_score * 0.28 + density * 0.24 + roundness * 0.30 + (1.0 - elongation) * 0.18))


def _texture_score(l_stdev):
    # 亮度标准差越大，内部纹理越复杂，更像真实火焰。
    return max(0.0, min(1.0, float(l_stdev) / 32.0))


def _warm_b_score(b_mean):
    # b 值越高越偏黄橙，越低越偏纯红；可用来抑制红激光干扰。
    span = float(config.WARM_B_MEAN_MAX - config.WARM_B_MEAN_MIN)
    if span <= 0:
        return 0.0
    return max(0.0, min(1.0, float(b_mean - config.WARM_B_MEAN_MIN) / span))


def _looks_like_laser_spot(area, w, h, aspect, density, roundness, l_mean, a_mean, b_mean):
    # 红色激光点通常是小而亮、偏纯红、外形较紧凑的高光斑。
    if area <= 0 or area > config.LASER_AREA_MAX:
        return False
    if w > config.LASER_MAX_WIDTH or h > config.LASER_MAX_HEIGHT:
        return False
    if aspect < config.LASER_ASPECT_MIN or aspect > config.LASER_ASPECT_MAX:
        return False
    if density < config.LASER_DENSITY_MIN:
        return False
    if roundness < config.LASER_ROUNDNESS_MIN:
        return False
    if l_mean < config.LASER_L_MEAN_MIN:
        return False
    if a_mean < config.LASER_A_MEAN_MIN:
        return False
    if b_mean > config.LASER_B_MEAN_MAX:
        return False
    return True


def _feature_delta(current_metrics, last_metrics):
    # 只比较和目标形状相关的特征，不比较中心点，适配移动摄像头场景。
    if current_metrics is None or last_metrics is None:
        return 0.0

    return (abs(current_metrics["aspect"] - last_metrics["aspect"]) * 1.7 +
            abs(current_metrics["density"] - last_metrics["density"]) * 1.8 +
            abs(current_metrics["roundness"] - last_metrics["roundness"]) * 1.6 +
            abs(current_metrics["elongation"] - last_metrics["elongation"]) * 1.2 +
            (abs(current_metrics["l_stdev"] - last_metrics["l_stdev"]) / 35.0))


def _targets_conflict(target_a, target_b):
    if (not target_a.valid) or (not target_b.valid):
        return False
    if abs(target_a.cx - target_b.cx) > config.TARGET_CONFLICT_DISTANCE:
        return False
    if abs(target_a.cy - target_b.cy) > config.TARGET_CONFLICT_DISTANCE:
        return False
    return True


def _area_ratio(reference_area, current_area):
    # 返回当前面积相对上一帧面积的比例，便于按配置直接判断上下限。
    if reference_area <= 0 or current_area <= 0:
        return 999.0
    return float(current_area) / float(reference_area)


def _should_reclassify_flame_as_orange(last_orange_target, flame_target, flame_metrics):
    # 当上一帧这里就是圆点，而本帧只是被激光照红，优先保持圆点身份。
    if (not last_orange_target.valid) or (not flame_target.valid) or (flame_metrics is None):
        return False

    if abs(last_orange_target.cx - flame_target.cx) > config.ORANGE_HISTORY_RECLASSIFY_DISTANCE:
        return False
    if abs(last_orange_target.cy - flame_target.cy) > config.ORANGE_HISTORY_RECLASSIFY_DISTANCE:
        return False

    area_ratio = _area_ratio(last_orange_target.area, flame_target.area)
    if area_ratio < config.ORANGE_HISTORY_RECLASSIFY_AREA_MIN_RATIO:
        return False
    if area_ratio > config.ORANGE_HISTORY_RECLASSIFY_AREA_MAX_RATIO:
        return False

    circle_score = flame_metrics.get("circle_score", 0.0)
    warm_score = flame_metrics.get("warm_score", 0.0)
    if circle_score < config.ORANGE_HISTORY_RECLASSIFY_CIRCLE_SCORE:
        return False
    if warm_score < config.ORANGE_HISTORY_RECLASSIFY_WARM_SCORE:
        return False
    return True


def _reclassify_flame_to_orange(last_orange_target, flame_target, flame_metrics):
    # 保留当前帧的位置和面积，但把类别拉回圆点，避免整颗圆点翻成火焰。
    circle_score = flame_metrics.get("circle_score", 0.0)
    warm_score = flame_metrics.get("warm_score", 0.0)
    quality = int(min(100,
                      max(flame_target.quality,
                          40 + circle_score * 28 + warm_score * 18)))
    return VisionTarget(kind=TARGET_ORANGE_CIRCLE,
                        valid=True,
                        cx=flame_target.cx,
                        cy=flame_target.cy,
                        ex=flame_target.ex,
                        ey=flame_target.ey,
                        area=flame_target.area,
                        angle=last_orange_target.angle,
                        quality=quality,
                        stale=False,
                        box_w=flame_target.box_w,
                        box_h=flame_target.box_h)


def _should_promote_flame_to_orange(flame_target, flame_metrics):
    # 没有历史圆点时，也允许把“首帧就被激光染红的圆”从火焰候选里拉回圆点。
    if (not flame_target.valid) or (flame_metrics is None):
        return False

    area = flame_target.area
    density = flame_metrics.get("density", 0.0)
    roundness = flame_metrics.get("roundness", 0.0)
    circle_score = flame_metrics.get("circle_score", 0.0)
    texture_score = flame_metrics.get("texture_score", 1.0)

    if area < config.ORANGE_RED_CIRCLE_AREA_MIN:
        return False
    if density < config.ORANGE_RED_CIRCLE_DENSITY_MIN:
        return False
    if roundness < config.ORANGE_RED_CIRCLE_ROUNDNESS_MIN:
        return False
    if circle_score < config.ORANGE_RED_CIRCLE_CIRCLE_SCORE:
        return False
    if texture_score > config.ORANGE_RED_CIRCLE_TEXTURE_MAX:
        return False
    return True


def _promote_flame_to_orange(flame_target, flame_metrics):
    circle_score = flame_metrics.get("circle_score", 0.0)
    density = flame_metrics.get("density", 0.0)
    roundness = flame_metrics.get("roundness", 0.0)
    texture_score = flame_metrics.get("texture_score", 1.0)
    warm_score = flame_metrics.get("warm_score", 0.0)
    uniformity_score = max(0.0, 1.0 - texture_score)
    quality = int(min(100,
                      max(flame_target.quality,
                          density * 24 + roundness * 18 + circle_score * 24 +
                          uniformity_score * 18 + warm_score * 6)))
    return VisionTarget(kind=TARGET_ORANGE_CIRCLE,
                        valid=True,
                        cx=flame_target.cx,
                        cy=flame_target.cy,
                        ex=flame_target.ex,
                        ey=flame_target.ey,
                        area=flame_target.area,
                        angle=0,
                        quality=quality,
                        stale=False,
                        box_w=flame_target.box_w,
                        box_h=flame_target.box_h)


def _orange_metrics_from_flame_metrics(flame_metrics):
    return {
        "aspect": flame_metrics.get("aspect", 1.0),
        "density": flame_metrics.get("density", 0.0),
        "elongation": flame_metrics.get("elongation", 1.0),
        "roundness": flame_metrics.get("roundness", 0.0),
        "l_stdev": flame_metrics.get("l_stdev", 0.0),
        "circle_score": flame_metrics.get("circle_score", 0.0),
        "uniformity_score": max(0.0, 1.0 - flame_metrics.get("texture_score", 1.0)),
        "warm_score": flame_metrics.get("warm_score", 0.0),
    }


def _resolve_target_conflict(orange_target, orange_metrics, flame_target, flame_metrics):
    # 两类目标指向同一个暖色块时，只有明显更强的一类才能保留，否则两边都不报。
    if not _targets_conflict(orange_target, flame_target):
        return orange_target, flame_target

    orange_score = orange_target.quality
    flame_score = flame_target.quality

    if orange_metrics is not None:
        orange_score += int(orange_metrics["circle_score"] * 15 + orange_metrics["uniformity_score"] * 10)
    if flame_metrics is not None:
        flame_score += int(flame_metrics["texture_score"] * 12 + flame_metrics["flicker_score"] * 14)

    if orange_score >= (flame_score + config.TARGET_CONFLICT_SCORE_MARGIN):
        return orange_target, invalid_target()
    if flame_score >= (orange_score + config.TARGET_CONFLICT_SCORE_MARGIN):
        return invalid_target(), flame_target
    return invalid_target(), invalid_target()


def _draw_target_debug(img, orange_target, flame_target):
    # 只绘制冲突消解后的最终目标，避免同一物体被画成两类标记。
    if not config.DRAW_DEBUG:
        return

    if orange_target.valid:
        radius = max(4, (orange_target.box_w + orange_target.box_h) // 4)
        img.draw_circle(orange_target.cx, orange_target.cy, radius, color=(255, 128, 0), thickness=2)
        img.draw_cross(orange_target.cx, orange_target.cy, color=(255, 128, 0))

    if flame_target.valid:
        left = max(0, flame_target.cx - (flame_target.box_w // 2))
        top = max(0, flame_target.cy - (flame_target.box_h // 2))
        width = max(2, flame_target.box_w)
        height = max(2, flame_target.box_h)
        img.draw_rectangle(left, top, width, height, color=(255, 0, 0), thickness=2)
        img.draw_cross(flame_target.cx, flame_target.cy, color=(255, 255, 0))


def _looks_like_orange_circle_shape(aspect, density, elongation, roundness):
    # 圆点必须同时满足接近方形、足够致密、细长度低、圆形度高。
    if aspect < config.ORANGE_ASPECT_MIN or aspect > config.ORANGE_ASPECT_MAX:
        return False
    if density < config.ORANGE_DENSITY_MIN:
        return False
    if elongation > config.ORANGE_ELONGATION_MAX:
        return False
    if roundness < config.ORANGE_ROUNDNESS_MIN:
        return False
    return True


class OrangeCircleDetector:
    def __init__(self):
        self._last_metrics = None

    def detect(self, img):
        # 检测橙色圆形目标，并返回评分最高的一个。
        blobs = img.find_blobs(config.ORANGE_THRESHOLDS,
                               pixels_threshold=config.ORANGE_PIXELS_THRESHOLD,
                               area_threshold=config.ORANGE_AREA_THRESHOLD,
                               merge=True,
                               margin=config.ORANGE_MERGE_MARGIN)
        best_target = None
        best_metrics = None
        best_score = -1
        img_cx = img.width() // 2
        img_cy = img.height() // 2

        for blob in blobs:
            w = blob.w()
            h = blob.h()
            if w < config.ORANGE_MIN_WIDTH or h < config.ORANGE_MIN_HEIGHT:
                continue

            aspect = float(w) / float(h)
            density = _safe_density(blob)
            elongation = _safe_elongation(blob)
            roundness = _safe_roundness(blob)
            if not _looks_like_orange_circle_shape(aspect, density, elongation, roundness):
                continue

            area = _blob_area(blob)
            circle_score = _circle_shape_score(aspect, density, elongation, roundness)
            stats = _safe_blob_stats(img, blob)
            l_stdev = _stats_l_stdev(stats)
            l_mean, a_mean, b_mean = _stats_lab_means(stats)
            warm_score = _warm_b_score(b_mean)
            if warm_score < config.ORANGE_WARM_SCORE_MIN:
                continue
            if _looks_like_laser_spot(area, w, h, aspect, density, roundness, l_mean, a_mean, b_mean):
                continue

            allowed_l_stdev = config.ORANGE_L_STDDEV_MAX
            if (area >= config.ORANGE_LASER_TOLERANT_AREA and
                    circle_score >= config.ORANGE_LASER_TOLERANT_CIRCLE_SCORE and
                    warm_score >= config.ORANGE_LASER_TOLERANT_WARM_SCORE):
                allowed_l_stdev = config.ORANGE_L_STDDEV_WITH_LASER_MAX
            if l_stdev > allowed_l_stdev:
                continue

            uniformity_score = 1.0 - _texture_score(l_stdev)
            metrics = {
                "aspect": aspect,
                "density": density,
                "elongation": elongation,
                "roundness": roundness,
                "l_stdev": l_stdev,
                "l_mean": l_mean,
                "a_mean": a_mean,
                "b_mean": b_mean,
                "circle_score": circle_score,
                "uniformity_score": uniformity_score,
                "warm_score": warm_score,
            }

            stability_score = config.ORANGE_STABLE_BONUS_BASE
            if self._last_metrics is not None:
                stability_score = max(0.0, min(1.0, 1.0 - _feature_delta(metrics, self._last_metrics)))

            # 综合面积、密度、接近圆形程度选出最优候选。
            aspect_score = 1.0 - abs(1.0 - aspect)
            score = int(area * (0.30 + density) * (0.35 + aspect_score) * (0.30 + circle_score) *
                        (0.30 + uniformity_score) * (0.35 + stability_score) * (0.35 + warm_score))
            quality = int(min(100,
                              max(0,
                                  density * 26 + aspect_score * 18 + roundness * 18 +
                                  (1.0 - elongation) * 12 + circle_score * 12 +
                                  uniformity_score * 8 + stability_score * 6 + warm_score * 8)))

            target = VisionTarget(kind=TARGET_ORANGE_CIRCLE,
                                  valid=True,
                                  cx=blob.cx(),
                                  cy=blob.cy(),
                                  # ex/ey 是相对画面中心的归一化偏差，范围大致在 -1000 到 1000。
                                  ex=_norm_error(blob.cx() - img_cx, img.width() // 2),
                                  ey=_norm_error(blob.cy() - img_cy, img.height() // 2),
                                  area=area,
                                  angle=0,
                                  quality=quality,
                                  box_w=w,
                                  box_h=h)

            if score > best_score:
                best_score = score
                best_target = target
                best_metrics = metrics

        self._last_metrics = best_metrics

        if best_target is None:
            return invalid_target(), None
        return best_target, best_metrics


class FlameDetector:
    def __init__(self):
        self._last_metrics = None

    def detect(self, img):
        # 检测火焰目标，并返回评分最高的一个。
        blobs = img.find_blobs(config.FLAME_THRESHOLDS,
                               pixels_threshold=config.FLAME_PIXELS_THRESHOLD,
                               area_threshold=config.FLAME_AREA_THRESHOLD,
                               merge=True,
                               margin=config.FLAME_MERGE_MARGIN)
        best_target = None
        best_metrics = None
        best_score = -1
        img_cx = img.width() // 2
        img_cy = img.height() // 2

        for blob in blobs:
            w = blob.w()
            h = blob.h()
            if w < config.FLAME_MIN_WIDTH or h < config.FLAME_MIN_HEIGHT:
                continue

            cx = blob.cx()
            cy = blob.cy()
            if cx < config.FLAME_EDGE_MARGIN_X or cx > (img.width() - config.FLAME_EDGE_MARGIN_X):
                continue
            if cy < config.FLAME_EDGE_MARGIN_Y or cy > (img.height() - config.FLAME_EDGE_MARGIN_Y):
                continue

            # 火焰形状允许比圆更自由，但仍限制过扁或过宽的候选。
            aspect = float(w) / float(h)
            if aspect < config.FLAME_ASPECT_MIN or aspect > config.FLAME_ASPECT_MAX:
                continue

            density = _safe_density(blob)
            if density < config.FLAME_DENSITY_MIN:
                continue

            # 过度细长的亮色区域更像噪声或反光，不作为火焰目标。
            elongation = _safe_elongation(blob)
            if elongation > config.FLAME_ELONGATION_MAX:
                continue

            roundness = _safe_roundness(blob)
            # 近圆且高密度的暖色块更像圆点，直接从火焰候选里排除。
            if _looks_like_orange_circle_shape(aspect, density, elongation, roundness):
                continue

            area = _blob_area(blob)
            stats = _safe_blob_stats(img, blob)
            l_stdev = _stats_l_stdev(stats)
            l_mean, a_mean, b_mean = _stats_lab_means(stats)
            warm_score = _warm_b_score(b_mean)
            if (_looks_like_laser_spot(area, w, h, aspect, density, roundness, l_mean, a_mean, b_mean) or
                    (area <= config.FLAME_LASER_REJECT_AREA_MAX and warm_score < config.FLAME_WARM_SCORE_MIN)):
                continue
            if l_stdev < config.FLAME_L_STDDEV_MIN:
                continue

            texture_score = _texture_score(l_stdev)
            metrics = {
                "aspect": aspect,
                "density": density,
                "elongation": elongation,
                "roundness": roundness,
                "l_stdev": l_stdev,
                "l_mean": l_mean,
                "a_mean": a_mean,
                "b_mean": b_mean,
                "circle_score": _circle_shape_score(aspect, density, elongation, roundness),
                "texture_score": texture_score,
                "flicker_score": config.FLAME_FLICKER_BASE,
                "warm_score": warm_score,
            }

            flicker_score = config.FLAME_FLICKER_BASE
            if self._last_metrics is not None:
                flicker_score = max(config.FLAME_FLICKER_BASE,
                                    min(1.0, _feature_delta(metrics, self._last_metrics)))
            metrics["flicker_score"] = flicker_score

            # 结合面积、暖色形状特征和密度来选最像火焰的目标。
            warm_shape_score = 1.0 - min(1.0, abs(0.75 - aspect))
            score = int(area * (0.35 + warm_shape_score) * (0.45 + density) *
                        (0.35 + texture_score) * (0.30 + flicker_score) * (0.20 + warm_score))
            quality = int(min(100,
                              max(0,
                                  density * 32 + warm_shape_score * 20 + (1.0 - elongation) * 12 +
                                  texture_score * 20 + flicker_score * 16 + warm_score * 10)))

            target = VisionTarget(kind=TARGET_FLAME,
                                  valid=True,
                                  cx=cx,
                                  cy=cy,
                                  ex=_norm_error(cx - img_cx, img.width() // 2),
                                  ey=_norm_error(cy - img_cy, img.height() // 2),
                                  area=area,
                                  angle=0,
                                  quality=quality,
                                  box_w=w,
                                  box_h=h)

            if score > best_score:
                best_score = score
                best_target = target
                best_metrics = metrics

        self._last_metrics = best_metrics

        if best_target is None:
            return invalid_target(), None
        return best_target, best_metrics


def _init_sensor():
    # 初始化摄像头格式、分辨率以及自动曝光/白平衡等参数。
    sensor.reset()

    if config.PIXFORMAT == "RGB565":
        sensor.set_pixformat(sensor.RGB565)
    else:
        sensor.set_pixformat(sensor.GRAYSCALE)

    if config.FRAME_SIZE == "QQVGA":
        sensor.set_framesize(sensor.QQVGA)
    else:
        sensor.set_framesize(sensor.QVGA)

    sensor.skip_frames(time=1500)

    if config.LOCK_AUTO_GAIN:
        sensor.set_auto_gain(False)
    if config.LOCK_AUTO_WHITEBAL:
        sensor.set_auto_whitebal(False)
    if config.LOCK_AUTO_EXPOSURE:
        sensor.set_auto_exposure(False, exposure_us=config.EXPOSURE_US)

    if config.FILL_LIGHT_ENABLE:
        try:
            for led_id in config.FILL_LIGHT_LED_IDS:
                LED(led_id).on()
        except Exception:
            pass


def _draw_status(img, orange_target, flame_target, fps_value):
    # 在图像上叠加双目标状态，便于现场确认串口实际发送内容。
    if not config.DRAW_STATUS_BAR:
        return

    orange_state = "1"
    if not orange_target.valid:
        orange_state = "0"
    elif orange_target.stale:
        orange_state = "S"

    flame_state = "1"
    if not flame_target.valid:
        flame_state = "0"
    elif flame_target.stale:
        flame_state = "S"

    line0 = "FPS=%d O:%s Q=%d" % (int(fps_value), orange_state, orange_target.quality)
    line1 = "OX=%d OY=%d" % (orange_target.cx, orange_target.cy)
    line2 = "F:%s Q=%d FX=%d" % (flame_state, flame_target.quality, flame_target.cx)
    line3 = "FY=%d" % (flame_target.cy)
    img.draw_string(2, 2, line0, color=(255, 255, 255), scale=1)
    img.draw_string(2, 14, line1, color=(255, 255, 255), scale=1)
    img.draw_string(2, 26, line2, color=(255, 255, 255), scale=1)
    img.draw_string(2, 38, line3, color=(255, 255, 255), scale=1)


def main():
    # 主循环：采图 -> 双目标检测 -> 分别跟踪 -> 定时发送双目标帧。
    _init_sensor()

    clock = time.clock()
    orange_detector = OrangeCircleDetector()
    flame_detector = FlameDetector()
    orange_tracker = TargetTracker()
    flame_tracker = ConfirmedTargetTracker(config.FLAME_CONFIRM_FRAMES,
                                           config.FLAME_CONFIRM_DISTANCE)
    frame_display = FrameDisplay()
    uart = VisionUart()
    last_send_ms = 0

    while True:
        clock.tick()
        img = sensor.snapshot()
        now_ms = millis()
        last_orange_target = orange_tracker.latest(now_ms, config.TRACK_HOLD_MS)

        # 每帧同时检测两类目标，并在进入跟踪前做一次冲突消解。
        orange_raw, orange_metrics = orange_detector.detect(img)
        flame_raw, flame_metrics = flame_detector.detect(img)
        if (not orange_raw.valid) and _should_reclassify_flame_as_orange(last_orange_target, flame_raw, flame_metrics):
            orange_raw = _reclassify_flame_to_orange(last_orange_target, flame_raw, flame_metrics)
            orange_metrics = _orange_metrics_from_flame_metrics(flame_metrics)
            flame_raw = invalid_target()
            flame_metrics = None
        elif (not orange_raw.valid) and _should_promote_flame_to_orange(flame_raw, flame_metrics):
            orange_raw = _promote_flame_to_orange(flame_raw, flame_metrics)
            orange_metrics = _orange_metrics_from_flame_metrics(flame_metrics)
            flame_raw = invalid_target()
            flame_metrics = None
        orange_raw, flame_raw = _resolve_target_conflict(orange_raw, orange_metrics, flame_raw, flame_metrics)
        _draw_target_debug(img, orange_raw, flame_raw)

        orange_target = orange_tracker.update(orange_raw, now_ms)
        flame_target = flame_tracker.update(flame_raw, now_ms)

        _draw_status(img, orange_target, flame_target, clock.fps())
        frame_display.show(img)

        if (now_ms - last_send_ms) >= config.SEND_FRAME_INTERVAL_MS:
            # 无目标时 flags 会为 0x00，两个目标的数据区保持为 0。
            uart.send_targets(orange_target, flame_target)
            last_send_ms = now_ms


main()
