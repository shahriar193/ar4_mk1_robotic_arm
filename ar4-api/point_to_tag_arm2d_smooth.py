#!/usr/bin/env python3
"""
point_to_tag_arm2d_smooth.py

Continuous, smooth AprilTag tracking for 2-DOF arm:

- Uses Arm2D (Teensy firmware unchanged).
- Continuously reads camera frames.
- For each frame:
    * Detect AprilTag 36h11 ID 0.
    * Compute pixel error from image center.
    * If error is outside a small deadband:
          - Compute a SMALL angle step d_th1, d_th2.
          - Update internal angles and call arm.move_math(th1, th2).
    * If error is small: do nothing (hold pose).

Key ideas for smoothness:
  - No "coarse + refine" bursts.
  - No long sleeps after moves.
  - Low gain (deg per pixel) + small per-command max step.
  - Optional minimum period between commands (~20 Hz).
"""

import time
import numpy as np
import cv2
from pupil_apriltags import Detector

from arm2d import Arm2D  # your existing API

# ===================== CAMERA + TAG CONFIG =====================

CAM_INDEX = 0  # adjust if needed

TAG_FAMILY     = "tag36h11"
PRIMARY_TAG_ID = 0
TAG_SIZE_M     = 0.025  # 25 mm tag

# Camera intrinsics (your calibration)
K = np.array([
    [565.02316947,   0.0,         303.68874689],
    [  0.0,         563.48245081, 250.54622527],
    [  0.0,           0.0,           1.0       ]
], dtype=np.float32)

DIST_COEFFS = np.array(
    [-0.11169773, 0.52121116, 0.00134473, 0.00237634, -0.93294836],
    dtype=np.float32
)

fx = float(K[0, 0])
fy = float(K[1, 1])
cx = float(K[0, 2])
cy = float(K[1, 2])
CAMERA_PARAMS = (fx, fy, cx, cy)

DISPLAY_SCALE = 1.0  # 1.0 = original

# ===================== CONTROL PARAMS (TUNE HERE) =====================

# Proportional gains: how many degrees per pixel of error (per update)
KP_YAW_DEG_PER_PX   = 0.02   # J1 (yaw)
KP_PITCH_DEG_PER_PX = 0.02   # J2 (pitch)

# Maximum step per command (deg). Smaller => smoother, but slower.
MAX_STEP_DEG = 1.7

# Deadband around the image center (px). Inside this → no motion.
CENTER_DEADBAND_PX = 10.0

# Minimum time between commands (s). ~0.05 = up to 20 Hz updates.
MIN_COMMAND_PERIOD_S = 0.01


# ===================== APRILTAG DETECTOR =====================

def create_detector():
    return Detector(
        families=TAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        debug=False
    )


# ===================== UTIL: DETECT TAG & ERROR =====================

def detect_tag_and_error(frame, detector):
    """
    Detect AprilTag ID PRIMARY_TAG_ID and compute pixel error from image center.

    Returns:
        (tag_found, err_x, err_y, annotated_frame)
        - tag_found: bool
        - err_x: tag_x - center_x (pixels)  (RIGHT positive)
        - err_y: tag_y - center_y (pixels)  (DOWN positive)
    """
    if DISPLAY_SCALE != 1.0:
        frame = cv2.resize(
            frame,
            None,
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE,
            interpolation=cv2.INTER_LINEAR
        )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    cx_img = w / 2.0
    cy_img = h / 2.0

    tags = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE_M
    )

    # Draw image center
    cv2.circle(frame, (int(cx_img), int(cy_img)), 4, (255, 0, 0), -1)

    target_tag = None
    for tag in tags:
        if tag.tag_id == PRIMARY_TAG_ID:
            target_tag = tag
            break

    if target_tag is None:
        cv2.putText(
            frame,
            f"Tag {PRIMARY_TAG_ID} not visible",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )
        return False, 0.0, 0.0, frame

    cx_tag, cy_tag = target_tag.center

    # Draw tag outline and center
    pts = target_tag.corners.astype(int)
    for i in range(4):
        pt1 = tuple(pts[i])
        pt2 = tuple(pts[(i + 1) % 4])
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
    cv2.circle(frame, (int(cx_tag), int(cy_tag)), 4, (0, 0, 255), -1)

    err_x = cx_tag - cx_img   # >0 => tag RIGHT
    err_y = cy_tag - cy_img   # >0 => tag DOWN

    cv2.putText(
        frame,
        f"err_x={err_x:.1f} px, err_y={err_y:.1f} px",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    return True, err_x, err_y, frame


# ===================== MAIN LOOP (CONTINUOUS SMALL CORRECTIONS) =====================

def main():
    # --- Arm initialization ---
    arm = Arm2D()  # uses your TeensyLink config

    # Get current math angles from status(). If parsing fails, fall back to a default.
    status = arm.status().get("parsed")
    if status and "j1_math" in status and "j2_math" in status:
        th1 = float(status["j1_math"])
        th2 = float(status["j2_math"])
        print(f"[INFO] Start from current pose: J1={th1:.1f}, J2={th2:.1f}")
    else:
        th1 = 0.0
        th2 = 20.0
        print(f"[INFO] Could not parse status; moving to default start pose: "
              f"J1={th1:.1f}, J2={th2:.1f}")
        arm.move_math(th1, th2)

    # --- Camera & detector ---
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera", CAM_INDEX)
        arm.close()
        return

    detector = create_detector()
    print(
        f"[INFO] Smooth continuous tracking running. "
        f"Family={TAG_FAMILY}, ID={PRIMARY_TAG_ID}"
    )

    last_cmd_time = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame")
            continue

        tag_found, err_x, err_y, disp = detect_tag_and_error(frame, detector)

        if tag_found:
            max_err = max(abs(err_x), abs(err_y))

            if max_err > CENTER_DEADBAND_PX:
                # Throttle command rate
                now = time.time()
                if now - last_cmd_time >= MIN_COMMAND_PERIOD_S:
                    # --------- Convert pixel error -> small angle steps ---------
                    # Your sign convention:
                    #  - If tag is LEFT  (err_x < 0) -> J1 should increase (math θ1 > 0)
                    #  - If tag is RIGHT (err_x > 0) -> J1 should decrease
                    # So: d_th1 = -K * err_x
                    d_th1 = -KP_YAW_DEG_PER_PX   * err_x

                    # For pitch:
                    #  - If tag is UP   (err_y < 0) -> J2 should increase (math θ2 > 0)
                    #  - If tag is DOWN (err_y > 0) -> J2 should decrease
                    # So: d_th2 = -K * err_y
                    d_th2 = -KP_PITCH_DEG_PER_PX * err_y

                    # Clamp each step to keep motion smooth
                    d_th1 = float(np.clip(d_th1, -MAX_STEP_DEG, MAX_STEP_DEG))
                    d_th2 = float(np.clip(d_th2, -MAX_STEP_DEG, MAX_STEP_DEG))

                    th1 += d_th1
                    th2 += d_th2

                    print(
                        f"[CTRL] err_x={err_x:.1f}, err_y={err_y:.1f} "
                        f"-> d_th1={d_th1:.2f}, d_th2={d_th2:.2f} "
                        f"-> J1={th1:.2f}, J2={th2:.2f}"
                    )

                    arm.move_math(th1, th2)
                    last_cmd_time = now

                    cv2.putText(
                        disp,
                        "Moving (correction)",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )
                else:
                    cv2.putText(
                        disp,
                        "Waiting (rate limit)",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2
                    )
            else:
                cv2.putText(
                    disp,
                    "Centered (within deadband)",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
        else:
            # Tag not found
            cv2.putText(
                disp,
                "Tag lost",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow("EE camera - smooth tracking", disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("[INFO] Quit requested.")
            break

    cap.release()
    cv2.destroyAllWindows()
    try:
        arm.close()
    except Exception:
        pass
    print("[INFO] Camera and Arm2D closed.")


if __name__ == "__main__":
    main()
