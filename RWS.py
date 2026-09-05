import cv2
import numpy as np
import serial
import time

# ============================================================
# SERIAL SETTINGS
# ============================================================
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
time.sleep(2)

# ============================================================
# CAMERA SETTINGS
# ============================================================
CAMERA_INDEX = 0

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30

cap = cv2.VideoCapture(CAMERA_INDEX)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Try to lock camera exposure and white balance.
# Not every camera supports these values, but it is safe to call them.
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # manual exposure on many Linux/V4L2 cameras
cap.set(cv2.CAP_PROP_EXPOSURE, -6)          # adjust if image is too dark/bright
cap.set(cv2.CAP_PROP_AUTO_WB, 0)            # disable auto white balance
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4500)  # fixed white balance if supported

# ============================================================
# SERVO LIMITS
# ============================================================
PAN_MIN = 0.0
PAN_MAX = 180.0

TILT_MIN = 0.0
TILT_MAX = 180.0

PAN_CENTER = 90.0
TILT_CENTER = 90.0

pan_angle = PAN_CENTER
tilt_angle = TILT_CENTER

# Change these if movement direction is wrong
PAN_DIR = -1.0
TILT_DIR = 1.0

# ============================================================
# CAMERA FIELD OF VIEW
# ============================================================
# Adjust these according to your camera lens.
# Common USB camera: around 55-70 horizontal FOV.
# Pi camera modules vary depending on lens.
HORIZONTAL_FOV_DEG = 60.0
VERTICAL_FOV_DEG = 45.0

# FOV correction strength.
# Higher = faster centering, but more overshoot risk.
ANGLE_GAIN_X = 0.55
ANGLE_GAIN_Y = 0.55

# Maximum command speed from Python.
# ESP32 also smooths the servo, but this prevents Python from sending aggressive jumps.
MAX_PAN_COMMAND_DPS = 85.0
MAX_TILT_COMMAND_DPS = 70.0

# ============================================================
# TRACKING SETTINGS
# ============================================================
TRACKING_ENABLED = True

# Smaller = more accurate, but more jitter risk.
DEADZONE_PX = 6

# Target centroid smoothing.
# Higher = smoother, lower = faster.
TARGET_ALPHA = 0.70

# Ignore tiny contour noise.
MIN_AREA = 550
MAX_AREA = 200000

# Shape filtering.
MIN_CIRCULARITY = 0.25
MIN_ASPECT_RATIO = 0.25
MAX_ASPECT_RATIO = 4.0

# ============================================================
# SEARCH MODE
# ============================================================
SEARCH_WHEN_LOST = True
SEARCH_SPEED_DPS = 35.0
LOST_BEFORE_SEARCH = 0.45

scan_dir = 1

# ============================================================
# SERIAL SEND SETTINGS
# ============================================================
SEND_INTERVAL = 1.0 / 30.0
last_send = 0.0

last_sent_pan = None
last_sent_tilt = None

# Send again only if angle changed by this much.
SEND_CHANGE_THRESHOLD_DEG = 0.05

# ============================================================
# RED HSV COLOR RANGE
# ============================================================
# Red wraps around HSV hue, so two ranges are required.
lower_red1 = np.array([0, 110, 60])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 110, 60])
upper_red2 = np.array([180, 255, 255])

kernel_open = np.ones((5, 5), np.uint8)
kernel_close = np.ones((7, 7), np.uint8)

# ============================================================
# STATE
# ============================================================
target_smooth = None
last_target_time = time.time()

prev_loop_time = time.time()
fps = 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


def move_toward(current, target, max_step):
    error = target - current

    if abs(error) <= max_step:
        return target

    if error > 0:
        return current + max_step
    else:
        return current - max_step


def get_frame_center(frame):
    h, w = frame.shape[:2]
    return np.array([w // 2, h // 2])


def build_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = mask1 | mask2

    # Clean noise
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
    mask = cv2.dilate(mask, kernel_open, iterations=1)

    return mask


def find_best_target(mask, previous_target=None):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []

    for c in contours:
        area = cv2.contourArea(c)

        if area < MIN_AREA or area > MAX_AREA:
            continue

        x, y, bw, bh = cv2.boundingRect(c)

        if bw <= 0 or bh <= 0:
            continue

        aspect_ratio = bw / float(bh)

        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue

        perimeter = cv2.arcLength(c, True)

        if perimeter <= 0:
            continue

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)

        if circularity < MIN_CIRCULARITY:
            continue

        M = cv2.moments(c)

        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Main score prefers large clean target.
        score = area * circularity

        # If we already have a target, prefer the object near previous position.
        if previous_target is not None:
            px, py = previous_target
            dist = np.hypot(cx - px, cy - py)
            score -= dist * 2.0

        candidates.append({
            "score": score,
            "cx": cx,
            "cy": cy,
            "area": area,
            "box": (x, y, bw, bh),
            "circularity": circularity,
            "aspect_ratio": aspect_ratio
        })

    if not candidates:
        return None

    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates[0]


def pixel_error_to_angle_error(err_x, err_y, frame_w, frame_h):
    """
    Converts pixel error into camera angle error using camera field of view.

    Example:
    If the object is at the right edge of the frame,
    horizontal angle error is about HORIZONTAL_FOV_DEG / 2.
    """
    angle_error_x = (err_x / (frame_w / 2.0)) * (HORIZONTAL_FOV_DEG / 2.0)
    angle_error_y = (err_y / (frame_h / 2.0)) * (VERTICAL_FOV_DEG / 2.0)

    return angle_error_x, angle_error_y


def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    return value


def send_servo_angles(pan, tilt, force=False):
    global last_send, last_sent_pan, last_sent_tilt

    now = time.time()

    if not force and now - last_send < SEND_INTERVAL:
        return

    pan = clamp(pan, PAN_MIN, PAN_MAX)
    tilt = clamp(tilt, TILT_MIN, TILT_MAX)

    changed = (
        last_sent_pan is None or
        last_sent_tilt is None or
        abs(pan - last_sent_pan) >= SEND_CHANGE_THRESHOLD_DEG or
        abs(tilt - last_sent_tilt) >= SEND_CHANGE_THRESHOLD_DEG
    )

    if changed or force:
        # Decimal servo command.
        # ESP32 must parse floats:
        # Expected format: A,90.25,88.70
        ser.write(f"A,{pan:.2f},{tilt:.2f}\n".encode("utf-8"))

        last_sent_pan = pan
        last_sent_tilt = tilt
        last_send = now


# Send center command at startup
send_servo_angles(pan_angle, tilt_angle, force=True)

print("Stable FOV-based tracking started")
print("Keys:")
print("  y = tracking on/off")
print("  c = center servos")
print("  q = quit")

while True:
    ret, frame = cap.read()

    if not ret:
        continue

    now = time.time()
    dt = now - prev_loop_time
    prev_loop_time = now

    if dt <= 0.0 or dt > 0.2:
        dt = 1.0 / CAMERA_FPS

    fps = 1.0 / dt

    h, w = frame.shape[:2]
    center = get_frame_center(frame)

    mask = build_red_mask(frame)

    previous_target = None
    if target_smooth is not None:
        previous_target = (target_smooth[0], target_smooth[1])

    target = find_best_target(mask, previous_target)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    if key == ord("y"):
        TRACKING_ENABLED = not TRACKING_ENABLED
        print("TRACKING ON" if TRACKING_ENABLED else "TRACKING OFF")

    if key == ord("c"):
        pan_angle = PAN_CENTER
        tilt_angle = TILT_CENTER
        target_smooth = None
        send_servo_angles(pan_angle, tilt_angle, force=True)
        print("CENTERED")

    target_found = target is not None

    if target_found:
        raw_cx = target["cx"]
        raw_cy = target["cy"]
        x, y, bw, bh = target["box"]

        last_target_time = now

        raw_target = np.array([float(raw_cx), float(raw_cy)])

        if target_smooth is None:
            target_smooth = raw_target
        else:
            target_smooth = (TARGET_ALPHA * target_smooth) + ((1.0 - TARGET_ALPHA) * raw_target)

        sx = int(target_smooth[0])
        sy = int(target_smooth[1])

        err_x = sx - center[0]
        err_y = sy - center[1]

        err_x = apply_deadzone(err_x, DEADZONE_PX)
        err_y = apply_deadzone(err_y, DEADZONE_PX)

        if TRACKING_ENABLED:
            angle_error_x, angle_error_y = pixel_error_to_angle_error(err_x, err_y, w, h)

            desired_pan = pan_angle + (PAN_DIR * angle_error_x * ANGLE_GAIN_X)
            desired_tilt = tilt_angle + (TILT_DIR * angle_error_y * ANGLE_GAIN_Y)

            desired_pan = clamp(desired_pan, PAN_MIN, PAN_MAX)
            desired_tilt = clamp(desired_tilt, TILT_MIN, TILT_MAX)

            max_pan_step = MAX_PAN_COMMAND_DPS * dt
            max_tilt_step = MAX_TILT_COMMAND_DPS * dt

            pan_angle = move_toward(pan_angle, desired_pan, max_pan_step)
            tilt_angle = move_toward(tilt_angle, desired_tilt, max_tilt_step)

            pan_angle = clamp(pan_angle, PAN_MIN, PAN_MAX)
            tilt_angle = clamp(tilt_angle, TILT_MIN, TILT_MAX)

            send_servo_angles(pan_angle, tilt_angle)

        # Draw target UI
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(frame, (raw_cx, raw_cy), 5, (0, 0, 255), -1)
        cv2.circle(frame, (sx, sy), 7, (255, 0, 255), 2)

        cv2.putText(frame, f"Area: {int(target['area'])}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    else:
        # Reset target memory after object is gone for some time.
        if now - last_target_time > 1.0:
            target_smooth = None

        # Search mode: pan left-right using full servo range.
        if TRACKING_ENABLED and SEARCH_WHEN_LOST:
            lost_time = now - last_target_time

            if lost_time > LOST_BEFORE_SEARCH:
                pan_angle += scan_dir * SEARCH_SPEED_DPS * dt

                if pan_angle >= PAN_MAX:
                    pan_angle = PAN_MAX
                    scan_dir = -1

                elif pan_angle <= PAN_MIN:
                    pan_angle = PAN_MIN
                    scan_dir = 1

                # Slowly bring tilt back to center during search.
                tilt_center_step = 30.0 * dt
                tilt_angle = move_toward(tilt_angle, TILT_CENTER, tilt_center_step)

                pan_angle = clamp(pan_angle, PAN_MIN, PAN_MAX)
                tilt_angle = clamp(tilt_angle, TILT_MIN, TILT_MAX)

                send_servo_angles(pan_angle, tilt_angle)

    # ========================================================
    # UI DISPLAY
    # ========================================================
    cv2.circle(frame, (center[0], center[1]), 5, (255, 0, 0), -1)
    cv2.circle(frame, (center[0], center[1]), DEADZONE_PX, (255, 0, 0), 1)

    status_text = "TRACKING ON" if TRACKING_ENABLED else "TRACKING OFF"
    status_color = (0, 255, 0) if TRACKING_ENABLED else (0, 0, 255)

    target_text = "TARGET FOUND" if target_found else "TARGET LOST"
    target_color = (0, 255, 0) if target_found else (0, 0, 255)

    cv2.putText(frame, status_text, (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

    cv2.putText(frame, target_text, (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, target_color, 2)

    cv2.putText(frame, f"PAN: {pan_angle:.2f}  TILT: {tilt_angle:.2f}", (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    cv2.putText(frame, f"FPS: {int(fps)}", (20, 135),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    cv2.putText(frame, f"FOV: {HORIZONTAL_FOV_DEG:.1f} x {VERTICAL_FOV_DEG:.1f}", (20, 165),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

    cv2.imshow("Accurate FOV-Based Tracking", frame)
    cv2.imshow("Red Mask", mask)

cap.release()
ser.close()
cv2.destroyAllWindows()
