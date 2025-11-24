#!/usr/bin/env python3
"""
go_close_to_tag_auto.py

Automatic coarse move of the 2-DOF arm toward an AprilTag using IK.

Behavior (no key press needed except 'q' to quit):
  - Continuously read camera frames.
  - Detect AprilTag 36h11 ID 0 (size 30 mm).
  - Estimate tag 3D pose in camera frame.
  - Transform tag pose into robot base frame using T_BASE_CAM (YOU must fill).
  - Pick a goal point some distance in front of tag along its +Z axis.
  - If the goal moves enough (e.g. > 20 mm from last), call arm.move_xyz(x,y,z)
    for a single smooth IK motion on Teensy.

This is a coarse "go near the object" behavior using your existing IK on Teensy.
Later you can switch to image-based tracking for fine centering if needed.
"""

import time
import numpy as np
import cv2
from pupil_apriltags import Detector

from arm2d import Arm2D  # your existing API


# ===================== CAMERA + TAG CONFIG =====================

CAM_INDEX      = 0
TAG_FAMILY     = "tag36h11"
PRIMARY_TAG_ID = 0
TAG_SIZE_M     = 0.030  # 30 mm tag (updated)

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


# ===================== BASE -> CAMERA EXTRINSICS (YOU FILL THIS) =====================

def make_T(R, t):
    """Build 4x4 homogeneous transform from R (3x3) and t (3,) in mm."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = t
    return T

# >>> IMPORTANT <<<
# T_BASE_CAM: 4x4 matrix mapping camera coords to base coords:
#   p_base = T_BASE_CAM @ [p_cam; 1]
#
# You MUST replace R_base_cam and t_base_cam_mm with your measured/estimated values.
# For now it's identity (meaning base frame == camera frame), which is WRONG in
# real life but okay as a placeholder while you calibrate.
R_base_cam = np.eye(3, dtype=np.float64)         # TODO: replace with your rotation
t_base_cam_mm = np.array([0.0, 0.0, 0.0])        # TODO: replace with your translation (mm)

T_BASE_CAM = make_T(R_base_cam, t_base_cam_mm)


# ===================== CONTROL PARAMS =====================

STANDOFF_MM = 80.0          # Distance to stay away from tag along its normal
REPLAN_THRESH_MM = 20.0     # Only send a new IK move if goal moved > this
REPLAN_MIN_PERIOD_S = 0.8   # Don't replan more often than this (sec)


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


# ===================== GEOMETRY: GOAL FROM TAG POSE =====================

def compute_goal_from_tag(tag) -> np.ndarray:
    """
    Compute coarse EE goal in base frame (mm) given an AprilTag detection.

    Steps:
      - Get tag.pose_R, tag.pose_t (tag->camera, in meters).
      - Convert translation to mm, build T_CAM_TAG (tag->camera).
      - Compose with T_BASE_CAM to get T_BASE_TAG (tag->base).
      - Take tag origin in base frame.
      - Get tag +Z axis in base frame.
      - Goal = tag_origin - STANDOFF_MM * z_axis (stay in front of tag).
    """
    R_cam_tag = np.array(tag.pose_R, dtype=np.float64)
    t_cam_tag_m = np.array(tag.pose_t, dtype=np.float64).reshape(3)
    t_cam_tag_mm = 1000.0 * t_cam_tag_m

    T_CAM_TAG = make_T(R_cam_tag, t_cam_tag_mm)    # tag -> camera
    T_BASE_TAG = T_BASE_CAM @ T_CAM_TAG            # tag -> base

    p_tag_base = T_BASE_TAG[:3, 3]
    R_base_tag = T_BASE_TAG[:3, :3]
    z_axis_base = R_base_tag[:, 2]  # tag +Z in base

    # Goal: stand off along tag's outward normal (camera side)
    p_goal_base = p_tag_base - STANDOFF_MM * z_axis_base
    return p_goal_base


# ===================== MAIN =====================

def main():
    arm = Arm2D()

    # Just print initial status if available
    st = arm.status().get("parsed")
    if st:
        print(f"[STATUS] Start: J1={st.get('j1_math', 'NA'):.2f} deg, "
              f"J2={st.get('j2_math', 'NA'):.2f} deg, "
              f"X={st.get('x_mm', 'NA')} mm, Y={st.get('y_mm', 'NA')} mm, "
              f"Z={st.get('z_mm', 'NA')} mm")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open camera", CAM_INDEX)
        arm.close()
        return

    detector = create_detector()
    print(f"[INFO] Automatic coarse tracking: family={TAG_FAMILY}, ID={PRIMARY_TAG_ID}")
    print("[INFO] Press 'q' in the window to quit.")

    last_goal = None
    last_replan_time = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame")
            continue

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
        cv2.circle(frame, (int(cx_img), int(cy_img)), 4, (255, 0, 0), -1)

        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M
        )

        target_tag = None
        for tag in tags:
            if tag.tag_id == PRIMARY_TAG_ID:
                target_tag = tag
                break

        if target_tag is not None:
            # Draw tag
            pts = target_tag.corners.astype(int)
            for i in range(4):
                pt1 = tuple(pts[i])
                pt2 = tuple(pts[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            cx_tag, cy_tag = target_tag.center
            cv2.circle(frame, (int(cx_tag), int(cy_tag)), 4, (0, 0, 255), -1)

            cv2.putText(
                frame,
                f"Tag {PRIMARY_TAG_ID} visible",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            # --- Compute new goal from tag pose ---
            p_goal = compute_goal_from_tag(target_tag)  # (3,) in mm
            x_mm, y_mm, z_mm = p_goal.tolist()

            # Decide whether to send a new IK move
            do_replan = False
            now = time.time()

            if last_goal is None:
                do_replan = True
            else:
                dist = np.linalg.norm(p_goal - last_goal)
                if dist > REPLAN_THRESH_MM and (now - last_replan_time) >= REPLAN_MIN_PERIOD_S:
                    do_replan = True

            if do_replan:
                print(f"[GOAL] x={x_mm:.1f} mm, y={y_mm:.1f} mm, z={z_mm:.1f} mm")
                res = arm.move_xyz(x_mm, y_mm, z_mm)
                if res.ok:
                    print("[INFO] move_xyz OK")
                else:
                    print("[WARN] move_xyz may be blocked:")
                    for ln in res.reply:
                        print("   ", ln)

                last_goal = p_goal
                last_replan_time = now

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

        cv2.imshow("Camera - automatic coarse go-to-tag", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("[INFO] Quit.")
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
