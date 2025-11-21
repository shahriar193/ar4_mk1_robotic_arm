#!/usr/bin/env python3
"""
point_to_tag_arm2d.py

Use the end-effector camera to point the 2-DOF arm toward AprilTag 36h11 ID 0.

- Uses the existing Arm2D Python API (no manual serial port handling).
- No homing is called from here (you can home separately if needed).
- Behavior:
    * The camera detects AprilTag ID 0.
    * If the tag is left   in the image -> J1 math angle increases (positive).
    * If the tag is right  in the image -> J1 math angle decreases (negative).
    * If the tag is up     in the image -> J2 math angle increases (positive).
    * If the tag is down   in the image -> J2 math angle decreases (negative).
    * Small corrections are applied repeatedly until the tag is near the center.
"""

import time
import numpy as np
import cv2
from pupil_apriltags import Detector

from arm2d import Arm2D   # uses your existing API


# ===================== CAMERA + TAG CONFIG =====================

CAM_INDEX = 0  # adjust if needed

TAG_FAMILY     = "tag36h11"
PRIMARY_TAG_ID = 0
TAG_SIZE_M     = 0.025  # 25 mm tag; change if you used a different size

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

# Intrinsics for AprilTag pose estimation
fx = float(K[0, 0])
fy = float(K[1, 1])
cx = float(K[0, 2])
cy = float(K[1, 2])
CAMERA_PARAMS = (fx, fy, cx, cy)

# ===================== CONTROL PARAMS =====================

# Degrees of joint change per pixel of error
Kp_YAW_DEG_PER_PX   = 0.02   # J1: horizontal correction
Kp_PITCH_DEG_PER_PX = 0.02   # J2: vertical correction

CENTER_THRESH_PX = 15        # when |err_x|, |err_y| < this, consider the tag centered
MAX_STEP_DEG     = 2.0       # clamp per-cycle change for safety
CONTROL_HZ       = 5.0       # how often we send new joint targets

DISPLAY_SCALE    = 1.0       # 1.0 = original; <1.0 to shrink display window


# ===================== MAIN =====================

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


def main():
    # ----- Initialize Arm2D (no homing here) -----
    arm = Arm2D()   # this will open serial using your existing logic

    # Optionally: you could call arm.initialize() here if homing works reliably
    # arm.initialize()

    # We'll keep our *internal* estimate of math angles here.
    # Start from something reasonable; or from your usual resting pose.
    th1 = 0.0
    th2 = 20.0

    print(f"[INFO] Starting servo from math angles J1={th1:.1f} deg, J2={th2:.1f} deg")
    arm.move_math(th1, th2)

    # ----- Camera and tag detector -----
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera index", CAM_INDEX)
        arm.close()
        return

    detector = create_detector()
    print(f"[INFO] AprilTag detector ready (family={TAG_FAMILY}, ID={PRIMARY_TAG_ID})")

    dt = 1.0 / CONTROL_HZ
    last_cmd_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Failed to grab frame")
                continue

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

            # Detect tags
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

            if target_tag is not None:
                # Center of tag in pixels
                cx_tag, cy_tag = target_tag.center

                # Draw tag corners and center
                pts = target_tag.corners.astype(int)
                for i in range(4):
                    pt1 = tuple(pts[i])
                    pt2 = tuple(pts[(i + 1) % 4])
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
                cv2.circle(frame, (int(cx_tag), int(cy_tag)), 4, (0, 0, 255), -1)

                # Pixel error: tag relative to image center
                err_x = cx_tag - cx_img   # >0 => tag to the RIGHT
                err_y = cy_tag - cy_img   # >0 => tag BELOW

                cv2.putText(
                    frame,
                    f"Tag {PRIMARY_TAG_ID} err_x={err_x:.1f} err_y={err_y:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

                now = time.time()
                if now - last_cmd_time >= dt:
                    last_cmd_time = now

                    if abs(err_x) < CENTER_THRESH_PX and abs(err_y) < CENTER_THRESH_PX:
                        cv2.putText(
                            frame,
                            "CENTERED",
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2
                        )
                        # Just hold current angles
                    else:
                        # Your sign convention:
                        # - If object is LEFT  -> positive angle
                        # - If object is RIGHT -> negative angle
                        #   err_x = tag_x - cx; LEFT => err_x < 0 => want d_th1 > 0
                        d_th1 = -Kp_YAW_DEG_PER_PX * err_x

                        # - If object is UP   -> positive angle
                        # - If object is DOWN -> negative angle
                        #   err_y = tag_y - cy; UP => err_y < 0 => want d_th2 > 0
                        d_th2 = -Kp_PITCH_DEG_PER_PX * err_y

                        # Clamp per-step changes
                        d_th1 = float(np.clip(d_th1, -MAX_STEP_DEG, MAX_STEP_DEG))
                        d_th2 = float(np.clip(d_th2, -MAX_STEP_DEG, MAX_STEP_DEG))

                        th1 += d_th1
                        th2 += d_th2

                        print(
                            f"[CMD] err_x={err_x:.1f} err_y={err_y:.1f} "
                            f"-> d_th1={d_th1:.3f}, d_th2={d_th2:.3f} | "
                            f"J1={th1:.2f}, J2={th2:.2f}"
                        )

                        # Use your existing API, same as demo_moves.py
                        res = arm.move_math(th1, th2)
                        # Optional: print replies for debugging
                        # print("\n".join(res.reply))

            else:
                cv2.putText(
                    frame,
                    f"Tag {PRIMARY_TAG_ID} not visible",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

            cv2.imshow("EE camera - point to AprilTag", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[INFO] Quitting (q pressed).")
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        try:
            arm.close()
        except Exception:
            pass
        print("[INFO] Camera and Arm2D closed.")


if __name__ == "__main__":
    main()
