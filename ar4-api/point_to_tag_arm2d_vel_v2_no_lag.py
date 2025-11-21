#!/usr/bin/env python3
"""
point_to_tag_arm2d_vel_threaded.py

Velocity-based AprilTag tracking for the 2-DOF arm, with threaded camera capture.

Architecture:
    - Capture thread:
        * Reads frames from the camera as fast as possible.
        * Always overwrites a shared 'latest_frame' (no backlog).
    - Main thread:
        * Copies latest_frame (if any), runs AprilTag detection,
          computes pixel error, converts to joint velocities (deg/s),
          and sends a velocity command to the Teensy via Arm2D.
        * Displays the annotated frame.

Result:
    - Even if detection + serial are slow, we never accumulate a camera buffer.
    - We always process the MOST RECENT frame => no multi-second lag.
"""

import time
import threading

import numpy as np
import cv2
from pupil_apriltags import Detector

from arm2d import Arm2D   # uses your existing API with set_velocity_math()


# ===================== CAMERA + TAG CONFIG =====================

CAM_INDEX      = 0
TAG_FAMILY     = "tag36h11"
PRIMARY_TAG_ID = 0
TAG_SIZE_M     = 0.030  # 30 mm tag

# Camera intrinsics (from your calibration)
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

# Downscale factor for display + detection; 1.0 = full resolution.
DISPLAY_SCALE = 1.0


# ===================== VELOCITY CONTROL GAINS =====================

# Gains: deg/s per pixel
KV_YAW_DEG_S_PER_PX   = 0.25   # J1
KV_PITCH_DEG_S_PER_PX = 0.25   # J2

MAX_FW_DEG_PER_SEC = 15.0      # must match firmware
VEL_CLAMP          = 0.8 * MAX_FW_DEG_PER_SEC  # 80% of max

CENTER_DEADBAND_PX = 6.0       # within this error, command zero velocity
MIN_SEND_PERIOD_S  = 0.04      # ~25 Hz velocity updates


# ===================== APRILTAG DETECTOR =====================

def create_detector():
    # Slight decimation for speed; adjust as needed.
    return Detector(
        families=TAG_FAMILY,
        nthreads=4,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        debug=False
    )


# ===================== TAG DETECTION + ERROR =====================

def detect_tag_and_error(frame, detector):
    """
    From a BGR frame, detect AprilTag PRIMARY_TAG_ID and compute pixel error.

    Returns:
        tag_found (bool),
        err_x (float): tag_x - cx_image (pixels, RIGHT positive),
        err_y (float): tag_y - cy_image (pixels, DOWN positive),
        disp (BGR image with overlays)
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
        estimate_tag_pose=False,  # just need center for velocity control
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE_M,
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
            2,
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

    err_x = cx_tag - cx_img   # >0 => tag right
    err_y = cy_tag - cy_img   # >0 => tag down

    cv2.putText(
        frame,
        f"err_x={err_x:.1f}, err_y={err_y:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    return True, err_x, err_y, frame


# ===================== THREAD: CAMERA CAPTURE =====================

def camera_capture_loop(cap, state, lock):
    """
    Continuously grab frames from cap and store the latest in state["latest_frame"].

    This thread does *nothing* heavy: no AprilTag, no serial I/O.
    It simply drains the camera buffer so we never fall behind.
    """
    while state["running"]:
        ret, frame = cap.read()
        if not ret:
            # You could add a small sleep to avoid busy looping on failure,
            # but typically this is fine if camera is working.
            continue

        with lock:
            state["latest_frame"] = frame


# ===================== MAIN CONTROL LOOP (runs in main thread) =====================

def main():
    # ----- Arm -----
    arm = Arm2D()
    st = arm.status().get("parsed")
    if st:
        print(
            f"[STATUS] Start: J1={st.get('j1_math', 0.0):.2f} deg, "
            f"J2={st.get('j2_math', 0.0):.2f} deg"
        )

    # ----- Camera -----
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera", CAM_INDEX)
        arm.close()
        return

    # Optional: try to limit internal buffering (works on some backends)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    state = {
        "running": True,
        "latest_frame": None,
    }
    lock = threading.Lock()

    # Start capture thread
    capture_thread = threading.Thread(
        target=camera_capture_loop,
        args=(cap, state, lock),
        daemon=True,
    )
    capture_thread.start()

    detector = create_detector()
    print("[INFO] Threaded velocity-based tag tracking started.")
    print("[INFO] Press 'q' in the window to quit.")

    last_send_time = 0.0

    try:
        while True:
            # Copy the most recent frame (if any)
            with lock:
                if state["latest_frame"] is None:
                    frame_copy = None
                else:
                    frame_copy = state["latest_frame"].copy()

            if frame_copy is None:
                # No frame yet; wait a bit
                time.sleep(0.01)
                continue

            tag_found, err_x, err_y, disp = detect_tag_and_error(frame_copy, detector)

            # Default zero velocities
            v1_cmd = 0.0
            v2_cmd = 0.0

            if tag_found:
                max_err = max(abs(err_x), abs(err_y))

                if max_err > CENTER_DEADBAND_PX:
                    # J1 yaw: tag left (err_x < 0) => want math angle to INCREASE => v1 > 0
                    v1_cmd = -KV_YAW_DEG_S_PER_PX * err_x
                    # J2 pitch: tag up (err_y < 0) => want math angle to INCREASE => v2 > 0
                    v2_cmd = -KV_PITCH_DEG_S_PER_PX * err_y

                    v1_cmd = float(np.clip(v1_cmd, -VEL_CLAMP, VEL_CLAMP))
                    v2_cmd = float(np.clip(v2_cmd, -VEL_CLAMP, VEL_CLAMP))

                    cv2.putText(
                        disp,
                        f"v1={v1_cmd:.1f}, v2={v2_cmd:.1f} deg/s",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                else:
                    cv2.putText(
                        disp,
                        "Centered (deadband)",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
            else:
                cv2.putText(
                    disp,
                    "Tag lost (zero vel)",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )

            # Send velocity commands at fixed rate
            now = time.time()
            if now - last_send_time >= MIN_SEND_PERIOD_S:
                # Uses Arm2D.set_velocity_math() which internally calls TeensyLink.send_command
                # with short timeouts; it should not block long enough to cause lag now.
                arm.set_velocity_math(v1_cmd, v2_cmd)
                last_send_time = now

            cv2.imshow("EE camera - threaded velocity tracking", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[INFO] Quit requested.")
                break

    finally:
        # Stop robot motion
        try:
            arm.set_velocity_math(0.0, 0.0)
        except Exception:
            pass

        # Stop threads and clean up
        state["running"] = False
        capture_thread.join(timeout=1.0)

        cap.release()
        cv2.destroyAllWindows()
        try:
            arm.close()
        except Exception:
            pass

        print("[INFO] Camera and Arm2D closed.")


if __name__ == "__main__":
    main()
