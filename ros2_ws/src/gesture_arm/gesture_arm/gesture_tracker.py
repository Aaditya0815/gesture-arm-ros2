#!/usr/bin/env python3
"""
MediaPipe Pose & Hands Tracker — "Digital Twin Safe-Teleop System" v13
Orangewood Labs Training — Individual Project
Author: Aaditya Sabharwal | TIET RAI 2026

v13 Changes:
  - Removed torso_yaw (no longer needed — IK handles j1 from Cartesian target)
  - Delta Cartesian control in publisher gives true puppet feel
  - All v12 improvements retained: no EMA, better wrist gains, j4 fix
"""

import cv2
import mediapipe as mp
import numpy as np
import math
import socket
import json
import signal
import sys
import time
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

UDP_HOST = '127.0.0.1'
UDP_PORT = 9870

# Smoothing factors
# POS_ALPHA removed — publisher's 1€ filter handles all position smoothing.
# Double-smoothing (EMA here + 1€ in publisher) caused lag that felt like jitter/inversion.
GRIPPER_ALPHA = 0.30

# ── v10: Scale-invariant fist threshold ──
# OLD: absolute spread < 0.12 (breaks when user steps far from camera)
# NEW: (thumb_tip → index_tip distance) / (wrist → middle_mcp distance) < threshold
# This ratio is scale-invariant — works at any camera distance.
FIST_RATIO_THRESHOLD = 0.35

# ── v14: Hysteresis mode switching thresholds ──
# Entry: wrist must be clearly above shoulder to enter TELEOP
# Exit:  wrist must drop significantly below shoulder to leave TELEOP
# This prevents accidental mode switches when lowering elbow during control.
TELEOP_ENTER_THRESH = 0.10   # wrist must be this far ABOVE shoulder to enter
TELEOP_EXIT_THRESH  = 0.15   # wrist must be this far BELOW shoulder to exit
STOP_ENTER_THRESH   = 0.10   # left wrist above left shoulder to trigger STOP
MODE_BUFFER_SIZE    = 20     # frames of mode history (was 15)
TELEOP_CONSENSUS    = 12     # frames needed to ENTER teleop (was 8)
AUTO_CONSENSUS      = 15     # frames needed to EXIT to auto (was 10) — harder to leave
STOP_CONSENSUS      = 8      # frames needed for emergency stop — fast

# ─────────────────────────────────────────────────────────────────────────────
# Fist Detection (Scale-Invariant)
# ─────────────────────────────────────────────────────────────────────────────

def is_fist(hand_landmarks, threshold=FIST_RATIO_THRESHOLD):
    if hand_landmarks is None:
        return False

    lm = hand_landmarks.landmark

    def dist3d(a, b):
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    pinch_dist = dist3d(lm[4],  lm[8])   # thumb tip → index tip
    hand_size  = dist3d(lm[0],  lm[9])   # wrist → middle MCP

    if hand_size < 1e-6:
        return False

    return (pinch_dist / hand_size) < threshold

# ─────────────────────────────────────────────────────────────────────────────
# Wrist Orientation Extraction
# ─────────────────────────────────────────────────────────────────────────────

def extract_wrist_orientation(hand_landmarks):
    DEFAULT = (0.0, -math.pi / 2, -math.pi / 2)

    if hand_landmarks is None:
        return DEFAULT

    lm = hand_landmarks.landmark

    wrist     = np.array([lm[0].x,  lm[0].y,  lm[0].z])
    idx_mcp   = np.array([lm[5].x,  lm[5].y,  lm[5].z])
    pinky_mcp = np.array([lm[17].x, lm[17].y, lm[17].z])

    v_lateral = idx_mcp - pinky_mcp
    v_forward = idx_mcp - wrist

    palm_normal = np.cross(v_lateral, v_forward)

    def safe_norm(v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else v

    x_axis = safe_norm(v_lateral)
    z_axis = safe_norm(palm_normal)
    y_axis = np.cross(z_axis, x_axis)

    R = np.column_stack([x_axis, y_axis, z_axis])

    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
    yaw   = math.atan2( R[1, 0],  R[0, 0])
    roll  = math.atan2( R[2, 1],  R[2, 2])

    # Wrist gains — v14 values (user-tested, felt good)
    wrist_roll  = yaw   * 0.8                       # j4 — doorknob rotation
    wrist_pitch = (-math.pi / 2) + pitch * 1.5      # j5 — palm tilt up/down
    wrist_yaw   = (-math.pi / 2) + roll  * 1.5      # j6 — wrist side tilt

    wrist_roll  = max(-math.pi,     min(math.pi,     wrist_roll))
    wrist_pitch = max(-math.pi,     min(math.pi / 4, wrist_pitch))
    wrist_yaw   = max(-math.pi,     min(math.pi,     wrist_yaw))

    return (wrist_roll, wrist_pitch, wrist_yaw)

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("═══════════════════════════════════════════════════")
    print("  Normalized Spatial Tracker v10")
    print("  Orangewood Labs | Aaditya Sabharwal | TIET RAI")
    print("  Fixes: Scale-invariant fist + 6-DOF wrist")
    print("═══════════════════════════════════════════════════")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    mp_pose = mp.solutions.pose
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        smooth_landmarks=True,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.5,
    )
    
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.5
    )

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera.")
        sys.exit(1)

    # States
    spatial_pos = np.array([0.0, 0.0, 0.0]) # [user_x, user_y, user_z]
    gripper = 1.0
    mode_buffer = deque(maxlen=MODE_BUFFER_SIZE)
    active_mode = "AUTO"
    mode_locked = False  # v14: hysteresis lock prevents accidental exits
    prev_time = time.time()
    fps = 0.0
    fist_ratio = 1.0    # For HUD display
    wrist_orientation = (0.0, -math.pi / 2, -math.pi / 2)

    def signal_handler(sig, frame):
        cap.release()
        cv2.destroyAllWindows()
        sock.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        ret, frame = cap.read()
        if not ret: continue

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(rgb)
        hand_results = hands.process(rgb)
        
        

        raw_mode = "AUTO"
        
        # ── 1. Pose Processing (Arm Position) ──
        if pose_results.pose_landmarks:
            world = pose_results.pose_world_landmarks.landmark
            img_lm = pose_results.pose_landmarks.landmark

            r_shoulder = img_lm[12]
            r_wrist = img_lm[16]
            l_shoulder = img_lm[11]
            l_wrist = img_lm[15]
            
            r_shoulder_w = world[12]
            r_wrist_w    = world[16]

            # ── v14: Hysteresis mode detection ──
            # STOP always has priority (safety)
            if l_wrist.y < l_shoulder.y - STOP_ENTER_THRESH:
                raw_mode = "STOP"
            elif active_mode == "TELEOP" and mode_locked:
                # LOCKED in TELEOP: stay unless wrist drops WELL below shoulder
                if r_wrist.y > r_shoulder.y + TELEOP_EXIT_THRESH:
                    raw_mode = "AUTO"   # deliberate exit gesture
                else:
                    raw_mode = "TELEOP"  # stay locked — safe zone
            else:
                # Not locked: use entry threshold
                if r_wrist.y < r_shoulder.y - TELEOP_ENTER_THRESH:
                    raw_mode = "TELEOP"
                else:
                    raw_mode = "AUTO"

            mode_buffer.append(raw_mode)

            # Consensus with different thresholds per mode
            stop_count   = mode_buffer.count("STOP")
            teleop_count = mode_buffer.count("TELEOP")
            auto_count   = mode_buffer.count("AUTO")

            if stop_count >= STOP_CONSENSUS:
                active_mode = "STOP"
                mode_locked = False
            elif teleop_count >= TELEOP_CONSENSUS:
                active_mode = "TELEOP"
                mode_locked = True   # Lock in!
            elif auto_count >= AUTO_CONSENSUS:
                active_mode = "AUTO"
                mode_locked = False

            if active_mode == "TELEOP":
                # True 3D Spatial Vector in meters (Right, Up, Forward)
                # MP World X: positive to user's left. So Right = shoulder.x - wrist.x
                # MP World Y: positive down. So Up = shoulder.y - wrist.y
                # MP World Z: positive away (behind). So Forward = shoulder.z - wrist.z
                
                user_x = r_shoulder_w.x - r_wrist_w.x
                user_y = r_shoulder_w.y - r_wrist_w.y
                user_z = r_shoulder_w.z - r_wrist_w.z

                # v12: NO EMA here — publisher's 1€ filter handles all smoothing.
                # Old: spatial_pos = POS_ALPHA * target_vec + (1-POS_ALPHA) * spatial_pos
                # That created double-smoothing lag that felt like jitter/inversion.
                spatial_pos = np.array([user_x, user_y, user_z])

            if pose_results.pose_landmarks:
                mp_draw.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            
        # ── 2. Hand Processing (Fist + Wrist Orientation) ──
        if active_mode == "TELEOP" and hand_results.multi_hand_landmarks:
            hand_lm = hand_results.multi_hand_landmarks[0]
            
            fist_detected = is_fist(hand_lm)
            target_gripper = 0.0 if fist_detected else 1.0
            gripper = GRIPPER_ALPHA * target_gripper + (1 - GRIPPER_ALPHA) * gripper

            wrist_orientation = extract_wrist_orientation(hand_lm)
            
            # For HUD display
            lm = hand_lm.landmark
            def d3(a, b): return math.sqrt((lm[a].x-lm[b].x)**2+(lm[a].y-lm[b].y)**2+(lm[a].z-lm[b].z)**2)
            hand_size = d3(0, 9)
            fist_ratio = d3(4, 8) / hand_size if hand_size > 1e-6 else 1.0
            
            mp_draw.draw_landmarks(frame, hand_lm, mp_hands.HAND_CONNECTIONS)
        elif active_mode == "TELEOP" and not hand_results.multi_hand_landmarks:
            # If hand is lost entirely (e.g. out of frame), slowly open gripper for safety
            gripper = GRIPPER_ALPHA * 1.0 + (1 - GRIPPER_ALPHA) * gripper

        display = cv2.flip(frame, 1)
        
        # ── 3. UDP Send ──
        data = json.dumps({
            'spatial': [round(float(spatial_pos[0]), 4), round(float(spatial_pos[1]), 4), round(float(spatial_pos[2]), 4)],
            'mode':    active_mode,
            'gripper': round(gripper, 3),
            'wrist':   [round(wrist_orientation[0], 4), round(wrist_orientation[1], 4), round(wrist_orientation[2], 4)]
        }).encode('utf-8')
        sock.sendto(data, (UDP_HOST, UDP_PORT))

        # ── 4. HUD ──
        now = time.time()
        dt = now - prev_time
        prev_time = now
        if dt > 0:
            fps = 0.1 * (1.0/dt) + 0.9 * fps

        h, w_img = display.shape[:2]
        cv2.rectangle(display, (0, 0), (w_img, 60), (0, 0, 0), -1)

        mode_cols = {"AUTO": (0,200,0), "TELEOP": (0,165,255), "STOP": (0,0,255)}
        mode_label = f"MODE: {active_mode}"
        if mode_locked and active_mode == "TELEOP":
            mode_label += " [LOCKED]"
        cv2.putText(display, mode_label, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_cols.get(active_mode, (255,255,255)), 2)
        cv2.putText(display, f"FPS: {fps:.0f}", (w_img - 140, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)

        grip_text = "CLOSED" if gripper < 0.5 else "OPEN"
        grip_col = (0,0,255) if gripper < 0.5 else (0,255,0)
        cv2.putText(display, f"Grip: {grip_text}", (w_img - 180, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, grip_col, 2)

        if active_mode == "TELEOP":
            info = f"X(R):{spatial_pos[0]:.2f} Y(U):{spatial_pos[1]:.2f} Z(F):{spatial_pos[2]:.2f}"
            cv2.putText(display, info, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,220,100), 1)
            cv2.putText(display, f"Fist Ratio: {fist_ratio:.2f} (< {FIST_RATIO_THRESHOLD})", (10, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,255), 1)
            pitch_deg = math.degrees(wrist_orientation[1])
            yaw_deg   = math.degrees(wrist_orientation[2])
            roll_deg  = math.degrees(wrist_orientation[0])
            cv2.putText(display, f"Wrist R:{roll_deg:.0f} P:{pitch_deg:.0f} Y:{yaw_deg:.0f}", (10, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100,255,200), 1)
        
        cv2.imshow("Spatial Telepresence | Aaditya Sabharwal", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()

if __name__ == '__main__':
    main()
