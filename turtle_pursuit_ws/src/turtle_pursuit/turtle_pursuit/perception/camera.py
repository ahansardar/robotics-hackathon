from dataclasses import dataclass
import math
import statistics
import struct

from turtle_pursuit.common.geometry import Pose2D


@dataclass
class CameraDetection:
    bearing: float
    distance: float | None
    confidence: float


def _rgb_indices(encoding):
    encoding = encoding.lower()
    if encoding in ('rgb8', 'rgba8'):
        return (0, 1, 2, 4 if encoding == 'rgba8' else 3)
    if encoding in ('bgr8', 'bgra8'):
        return (2, 1, 0, 4 if encoding == 'bgra8' else 3)
    return None


def _depth_value(message, x, y):
    if message is None or not (0 <= x < message.width and 0 <= y < message.height):
        return None
    encoding = message.encoding.lower()
    endian = '>' if message.is_bigendian else '<'
    if encoding == '32fc1':
        offset = y*message.step+x*4
        return struct.unpack_from(endian+'f', message.data, offset)[0]
    if encoding in ('16uc1', 'mono16'):
        offset = y*message.step+x*2
        return struct.unpack_from(endian+'H', message.data, offset)[0]/1000.0
    return None


def detect_colored_target(image, depth, camera_info, target_color):
    """Locate the red/blue role marker and recover bearing plus RGB-D range."""
    layout = _rgb_indices(image.encoding)
    if layout is None or image.width <= 0 or image.height <= 0:
        return None
    ri, gi, bi, channels = layout
    stride = max(1, min(image.width, image.height)//80)
    xs = []
    ys = []
    for y in range(0, image.height, stride):
        row = y*image.step
        for x in range(0, image.width, stride):
            offset = row+x*channels
            if offset+channels > len(image.data):
                continue
            r = image.data[offset+ri]
            g = image.data[offset+gi]
            b = image.data[offset+bi]
            # The ratio-only tests below classify saturated orange as "red"
            # (e.g. an arena obstacle painted diffuse (1, 0.55, 0.08), which is
            # roughly RGB 255/140/20: 255 > 1.45*140 and 255 > 1.35*20 both
            # hold). A true red marker has a low green channel; orange does
            # not, so cap it explicitly rather than relying on ratios alone.
            # Same reasoning caps blue's red channel against a magenta/purple
            # false positive.
            red = r >= 100 and r > 1.45*g and r > 1.35*b and g <= 110
            blue = b >= 90 and b > 1.25*g and b > 1.45*r and r <= 110
            if (target_color == 'red' and red) or (target_color == 'blue' and blue):
                xs.append(x)
                ys.append(y)
    if len(xs) < 4:
        return None
    cx = sum(xs)/len(xs)
    cy = sum(ys)/len(ys)
    fx = camera_info.k[0] if camera_info is not None and camera_info.k[0] > 0 else image.width/(2*math.tan(1.25/2))
    optical_cx = camera_info.k[2] if camera_info is not None and camera_info.k[2] > 0 else image.width/2
    bearing = -math.atan2(cx-optical_cx, fx)
    depths = []
    for dy in range(-3, 4):
        for dx in range(-3, 4):
            value = _depth_value(depth, int(cx)+dx, int(cy)+dy)
            if value is not None and math.isfinite(value) and .20 < value < 20.0:
                depths.append(value)
    distance = statistics.median(depths) if depths else None
    sampled = max(1, ((image.width+stride-1)//stride)*((image.height+stride-1)//stride))
    return CameraDetection(bearing, distance, min(1.0, len(xs)/max(12.0, sampled*.01)))


def detection_to_world(observer, detection, previous=None, max_speed=None):
    """Project a bearing/range detection into a world pose.

    If `previous` (the target's last trusted world pose) and `max_speed` are
    both given, a detection implying a physically impossible jump since then
    is rejected (returns None) instead of silently overwriting a good
    estimate with a spurious color match. This is defense in depth alongside
    the tightened color thresholds above: any object that happens to share a
    hue with the role marker -- not just the specific obstacle color already
    fixed above -- could otherwise hijack the tracked pose.
    """
    if detection is None or detection.distance is None:
        return None
    heading = observer.yaw+detection.bearing
    x = observer.x+detection.distance*math.cos(heading)
    y = observer.y+detection.distance*math.sin(heading)
    if previous is not None and max_speed is not None:
        dt = observer.stamp-previous.stamp
        if dt > 1e-3 and math.hypot(x-previous.x, y-previous.y)/dt > max_speed:
            return None
    return Pose2D(x, y, heading, observer.stamp)
