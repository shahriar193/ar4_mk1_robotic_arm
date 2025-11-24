#!/usr/bin/env python3
"""
point_to_tag_arm2d.py

Continuous tracking version (Option A in a loop):

- Uses the existing Arm2D Python API (no manual serial port handling).
- No homing is called from here (you can home separately if needed).

Behavior:
  * Continuously read camera frames and detect AprilTag 36h11 ID 0.
  * If the tag is near the center -> do nothing (hold pose).
  * If the tag moves away from the center by more than a threshold:
        - Run ONE coarse move (big smooth arm.move_math()).
        - Run a few small refinement steps.
        - Return to monitoring.
  * If, later, the tag moves again -> repeat.
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

fx = float(K[0, 0])
fy = float(K[1, 1])
cx = float(K[0, 2])
cy = float(K[1, 2])
CAMERA_PARAMS = (fx, fy, cx, cy)

DISPLAY_SCALE = 1.0  # 1.0 = original


# ===================== CONTROL PARAMS =====================

# Coarse move: one big sweep (for smooth motion)
COARSE_KP_YAW_DEG_PER_PX   = 0.06
COARSE_KP_PITCH_DEG_PER_PX = 0.06
COARSE_MAX_DEG             = 60.0

# Refinement: small clean-up around center
REFINE_KP_YAW_DEG_PER_PX   = 0.01
REFINE_KP_PITCH_DEG_PER_PX = 0.01
REFINE_MAX_DEG             = 3.0
REFINE_CENTER_THRESH_PX    = 10.0
REFINE_MAX_ITERS           = 5

# When monitoring: how far tag can drift before we trigger a new correction
TRIGGER_ERR_THRESH_PX = 25.0   # if max(|err_x|,|err_y|) > this -> run coarse+refine


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
        f"Tag {PRIMARY_TAG_ID} err_x={err_x:.1f} err_y={err_y:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    return True, err_x, err_y, frame


# ===================== COARSE + REFINE SEQUENCE =====================

def coarse_and_refine(arm, th1, th2, detector, cap, initial_err_x, initial_err_y):
    """
    Run ONE coarse move + small refinement, starting from angles (th1, th2).

    Uses:
      - initial_err_x, initial_err_y from the triggering frame.
      - reads its own frames from 'cap' during refinement.

    Returns:
      updated (th1, th2)
    """
    # ---------- COARSE MOVE ----------
    err_x = initial_err_x
    err_y = initial_err_y

    print(f"[COARSE] err_x={err_x:.1f}, err_y={err_y:.1f}")

    # Your sign convention:
    # If object is LEFT  -> positive angle
    # If object is RIGHT -> negative angle
    #   err_x = tag_x - cx; LEFT => err_x < 0 => want d_th1 > 0 → d_th1 = -K * err_x
    d_th1 = -COARSE_KP_YAW_DEG_PER_PX   * err_x

    # If object is UP   -> positive angle
    # If object is DOWN -> negative angle
    #   err_y = tag_y - cy; UP => err_y < 0 => want d_th2 > 0 → d_th2 = -K * err_y
    d_th2 = -COARSE_KP_PITCH_DEG_PER_PX * err_y

    d_th1 = float(np.clip(d_th1, -COARSE_MAX_DEG, COARSE_MAX_DEG))
    d_th2 = float(np.clip(d_th2, -COARSE_MAX_DEG, COARSE_MAX_DEG))

    th1_target = th1 + d_th1
    th2_target = th2 + d_th2

    print(
        f"[COARSE] d_th1={d_th1:.2f}, d_th2={d_th2:.2f} "
        f"-> target J1={th1_target:.2f}, J2={th2_target:.2f}"
    )

    arm.move_math(th1_target, th2_target)
    th1, th2 = th1_target, th2_target

    # Let the move settle a bit
    time.sleep(0.5)

    # ---------- REFINEMENT ----------
    print("[INFO] Refinement phase...")
    for i in range(REFINE_MAX_ITERS):
        ret, frame = cap.read()
        if not ret:
            print("[REFINE] Failed to grab frame")
            continue

        tag_found, err_x, err_y, disp = detect_tag_and_error(frame, detector)

        if tag_found:
            cv2.putText(
                disp,
                f"Refine {i+1}/{REFINE_MAX_ITERS}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
        else:
            cv2.putText(
                disp,
                f"Refine {i+1}/{REFINE_MAX_ITERS} - tag lost",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow("EE camera - tracking", disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("[INFO] Quit during refinement requested.")
            return th1, th2, True  # signal to quit main loop

        if not tag_found:
            print("[REFINE] Tag not visible; skipping.")
            continue

        if abs(err_x) < REFINE_CENTER_THRESH_PX and abs(err_y) < REFINE_CENTER_THRESH_PX:
            print(
                f"[REFINE] Centered (err_x={err_x:.1f}, err_y={err_y:.1f}), "
                "stopping refinement."
            )
            break

        d_th1_r = -REFINE_KP_YAW_DEG_PER_PX   * err_x
        d_th2_r = -REFINE_KP_PITCH_DEG_PER_PX * err_y

        d_th1_r = float(np.clip(d_th1_r, -REFINE_MAX_DEG, REFINE_MAX_DEG))
        d_th2_r = float(np.clip(d_th2_r, -REFINE_MAX_DEG, REFINE_MAX_DEG))

        th1 += d_th1_r
        th2 += d_th2_r

        print(
            f"[REFINE] iter {i+1}: err_x={err_x:.1f}, err_y={err_y:.1f} "
            f"-> d_th1={d_th1_r:.2f}, d_th2={d_th2_r:.2f} "
            f"-> J1={th1:.2f}, J2={th2:.2f}"
        )

        arm.move_math(th1, th2)
        time.sleep(0.2)

    return th1, th2, False  # updated angles, no quit


# ===================== MAIN LOOP =====================

def main():
    # --- Arm initialization ---
    arm = Arm2D()   # uses your existing serial config

    # Start from a reasonable pose
    th1 = 0.0
    th2 = 20.0
    print(f"[INFO] Moving to start pose: J1={th1:.1f}, J2={th2:.1f}")
    arm.move_math(th1, th2)

    # --- Camera & detector ---
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera", CAM_INDEX)
        arm.close()
        return

    detector = create_detector()
    print(
        f"[INFO] Continuous tracking running. "
        f"Family={TAG_FAMILY}, ID={PRIMARY_TAG_ID}"
    )

    quit_flag = False

    while not quit_flag:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame (main loop)")
            continue

        tag_found, err_x, err_y, disp = detect_tag_and_error(frame, detector)

        if tag_found:
            max_err = max(abs(err_x), abs(err_y))

            if max_err > TRIGGER_ERR_THRESH_PX:
                print(
                    f"[MAIN] Error {max_err:.1f} px > {TRIGGER_ERR_THRESH_PX} px -> "
                    f"run coarse+refine."
                )
                th1, th2, quit_flag = coarse_and_refine(
                    arm, th1, th2, detector, cap, err_x, err_y
                )
                # After coarse+refine, loop continues and we monitor again.
                continue
            else:
                cv2.putText(
                    disp,
                    "OK (within tolerance)",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
        else:
            # Tag not found: just display info
            pass

        cv2.imshow("EE camera - tracking", disp)
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
