#!/usr/bin/env python3
"""
point_to_tag_arm2d_vel.py

Continuous, velocity-based AprilTag tracking for 2-DOF arm.

- Uses new Teensy "V v1 v2" command (math-angle velocities, deg/s).
- No IK, no position jumps: just joint velocities.
- As long as the tag is visible, we run a low-gain visual servo:
      err_x, err_y (pixels) -> v1, v2 (deg/s)
- Teensy firmware handles smooth stepping in the background.

Controls:
    - 'q' in the OpenCV window to quit.
"""

import time
import numpy as np
import cv2
from pupil_apriltags import Detector

from arm2d import Arm2D  # must include set_velocity_math()


# ===================== CAMERA + TAG CONFIG =====================

CAM_INDEX      = 0
TAG_FAMILY     = "tag36h11"
PRIMARY_TAG_ID = 0
TAG_SIZE_M     = 0.030  # 30 mm tag

# Camera intrinsics
K = np.array([
    [565.02316947,   0.0,         303.68874689],
    [  0.0,         563.48245081, 250.54622527],
    [  0.0,           0.0,           1.0       ]
], dtype=np.float32)

DIST_COEFFS = np.array(
    [-0.11169773, 0.52121116, 0.00134473, 0.00237634, -0.93294836],
    dtype=np.float32
)
fx = float(K[0, 0]); fy = float(K[1, 1])
cx = float(K[0, 2]); cy = float(K[1, 2])
CAMERA_PARAMS = (fx, fy, cx, cy)

DISPLAY_SCALE = 1.0  # 1.0 = original


# ===================== VELOCITY CONTROL GAINS =====================

# Gains: deg/s per pixel
KV_YAW_DEG_S_PER_PX   = 0.25   # For J1 (yaw)
KV_PITCH_DEG_S_PER_PX = 0.25   # For J2 (pitch)

# Firmware-side hard cap is MAX_DEG_PER_SEC; we keep a margin.
MAX_FW_DEG_PER_SEC = 15.0
VEL_CLAMP = 0.8 * MAX_FW_DEG_PER_SEC  # e.g. 12 deg/s

# Pixel deadband around center: inside this, command 0 velocity
CENTER_DEADBAND_PX = 6.0

# Command rate: max ~1 / MIN_SEND_PERIOD_S
MIN_SEND_PERIOD_S = 0.03  # ~33 Hz


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


# ===================== TAG DETECTION + ERROR =====================

def detect_tag_and_error(frame, detector):
    """
    Detect AprilTag and compute pixel error from image center.

    Returns:
        tag_found, err_x, err_y, annotated_frame
    """
    if DISPLAY_SCALE != 1.0:
        frame = cv2.resize(
            frame, None,
            fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
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

    cv2.circle(frame, (int(cx_img), int(cy_img)), 4, (255, 0, 0), -1)

    target_tag = None
    for tag in tags:
        if tag.tag_id == PRIMARY_TAG_ID:
            target_tag = tag
            break

    if target_tag is None:
        cv2.putText(frame,
                    f"Tag {PRIMARY_TAG_ID} not visible",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)
        return False, 0.0, 0.0, frame

    cx_tag, cy_tag = target_tag.center

    pts = target_tag.corners.astype(int)
    for i in range(4):
        pt1 = tuple(pts[i])
        pt2 = tuple(pts[(i + 1) % 4])
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
    cv2.circle(frame, (int(cx_tag), int(cy_tag)), 4, (0, 0, 255), -1)

    err_x = cx_tag - cx_img   # >0 => tag right
    err_y = cy_tag - cy_img   # >0 => tag down

    cv2.putText(frame,
                f"err_x={err_x:.1f}, err_y={err_y:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2)

    return True, err_x, err_y, frame


# ===================== MAIN: VELOCITY-BASED TRACKING =====================

def main():
    arm = Arm2D()

    # Optional: start from current pose, just to konw.
    st = arm.status().get("parsed")
    if st:
        print(f"[STATUS] Start: J1={st.get('j1_math', 'NA'):.2f} deg, "
              f"J2={st.get('j2_math', 'NA'):.2f} deg")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera", CAM_INDEX)
        arm.close()
        return

    detector = create_detector()
    print("[INFO] Velocity-based tracking started.")
    print("[INFO] Press 'q' in the window to quit.")

    last_send_time = 0.0
    moving = False  # track whether we are currently sending non-zero velocities

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Failed to grab frame")
                continue

            tag_found, err_x, err_y, disp = detect_tag_and_error(frame, detector)

            v1_cmd = 0.0
            v2_cmd = 0.0

            if tag_found:
                max_err = max(abs(err_x), abs(err_y))

                if max_err > CENTER_DEADBAND_PX:
                    # ---- Convert pixel error to math-angle velocities ----
                    # Sign convention:
                    #   J1 (yaw): if tag is LEFT  (err_x < 0), we want J1 to INCREASE (math++)
                    #             if tag is RIGHT (err_x > 0), we want J1 to DECREASE
                    #   => v1 = -K * err_x
                    v1_cmd = -KV_YAW_DEG_S_PER_PX * err_x

                    #   J2 (pitch): if tag is UP   (err_y < 0), we want J2 to INCREASE (math++)
                    #               if tag is DOWN (err_y > 0), we want J2 to DECREASE
                    #   => v2 = -K * err_y
                    v2_cmd = -KV_PITCH_DEG_S_PER_PX * err_y

                    # Clamp to safe range
                    v1_cmd = float(np.clip(v1_cmd, -VEL_CLAMP, VEL_CLAMP))
                    v2_cmd = float(np.clip(v2_cmd, -VEL_CLAMP, VEL_CLAMP))

                    cv2.putText(disp,
                                f"v1={v1_cmd:.1f} deg/s, v2={v2_cmd:.1f} deg/s",
                                (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(disp,
                                "Centered (deadband)",
                                (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)
            else:
                cv2.putText(disp,
                            "Tag lost (command 0 vel)",
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)

            # ---- Send velocity command at limited rate ----
            now = time.time()
            if now - last_send_time >= MIN_SEND_PERIOD_S:
                arm.set_velocity_math(v1_cmd, v2_cmd)
                last_send_time = now
                moving = (abs(v1_cmd) > 1e-3 or abs(v2_cmd) > 1e-3)

            cv2.imshow("EE camera - velocity tracking", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[INFO] Quit requested.")
                break

    finally:
        # Ensure we stop the arm
        try:
            arm.set_velocity_math(0.0, 0.0)
        except Exception:
            pass

        cap.release()
        cv2.destroyAllWindows()
        try:
            arm.close()
        except Exception:
            pass
        print("[INFO] Camera and Arm2D closed.")


if __name__ == "__main__":
    main()
