#!/usr/bin/env python3
"""
ROS2 Joint State Bridge + Virtual Pick & Place Environment ? v11
Orangewood Labs Training ? Individual Project
Author: Aaditya Sabharwal | TIET RAI 2026

v11 TELEOP OVERHAUL ? Cylindrical Coordinate Control:
  - j1 (base) driven DIRECTLY from horizontal arm angle, not from XY IK
  - This gives full 360? base rotation (was capped at ~70? before)
  - Inversion fixed: arm-right = robot-right
  - j2/j3 driven from reach + height (natural and intuitive)
  - 1? filter + JointInterpolator retained
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

import socket
import json
import math
import random
import numpy as np

UDP_HOST    = '127.0.0.1'
UDP_PORT    = 9870
BUFFER_SIZE = 1024

# ???????????????????????????????????????????????????
# UR5e Geometry
# ???????????????????????????????????????????????????
D1        = 0.1625
L1        = 0.425
L2        = 0.3922
MAX_REACH = L1 + L2 - 0.05   # 0.762 ? safe max, avoids full-extension singularity
MIN_REACH = 0.15              # Comfortable minimum ? avoids fully-folded singularity

PICK_DISTANCE  = 0.15    # v14: 0.12 -> 0.15 for more forgiving grabs
PLACE_DISTANCE = 0.15
CUBE_SIZE      = 0.05

# ???????????????????????????????????????????????????
# 1? Filter
# ???????????????????????????????????????????????????

class OneEuroFilter:
    def __init__(self, freq=30.0, min_cutoff=1.2, beta=0.06, d_cutoff=1.0):
        self.freq       = freq
        self.min_cutoff = min_cutoff
        self.beta       = beta
        self.d_cutoff   = d_cutoff
        self._x_prev    = None
        self._dx_prev   = 0.0

    def _alpha(self, cutoff):
        tau = 1.0 / (2 * math.pi * cutoff)
        te  = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x):
        if self._x_prev is None:
            self._x_prev = x
            return x
        dx     = (x - self._x_prev) * self.freq
        a_d    = self._alpha(self.d_cutoff)
        dx_hat = a_d * dx + (1 - a_d) * self._dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a      = self._alpha(cutoff)
        x_hat  = a * x + (1 - a) * self._x_prev
        self._x_prev  = x_hat
        self._dx_prev = dx_hat
        return x_hat

    def reset(self):
        self._x_prev  = None
        self._dx_prev = 0.0


class PoseFilter:
    def __init__(self, freq=30.0):
        # v14: Position axes — more responsive (higher min_cutoff, moderate beta)
        self.fx  = OneEuroFilter(freq, min_cutoff=0.8, beta=0.15)
        self.fy  = OneEuroFilter(freq, min_cutoff=0.8, beta=0.15)
        self.fz  = OneEuroFilter(freq, min_cutoff=0.8, beta=0.15)
        # Lateral (left/right sweep): smooth — low beta kills jitter
        self.fx_lat = OneEuroFilter(freq, min_cutoff=0.35, beta=0.05)
        # Wrist orientation: responsive with moderate smoothing
        self.fr  = OneEuroFilter(freq, min_cutoff=0.5, beta=0.10)
        self.fp  = OneEuroFilter(freq, min_cutoff=0.5, beta=0.10)
        self.fyw = OneEuroFilter(freq, min_cutoff=0.5, beta=0.10)
        # Torso yaw: very smooth
        self.ft  = OneEuroFilter(freq, min_cutoff=0.3, beta=0.03)

    def filter_position(self, x, y, z):
        return self.fx(x), self.fy(y), self.fz(z)

    def filter_lateral(self, x):
        return self.fx_lat(x)

    def filter_orientation(self, roll, pitch, yaw):
        return self.fr(roll), self.fp(pitch), self.fyw(yaw)

    def filter_torso(self, torso_yaw):
        return self.ft(torso_yaw)

    def reset(self):
        for f in [self.fx, self.fy, self.fz, self.fx_lat,
                  self.fr, self.fp, self.fyw, self.ft]:
            f.reset()


# ???????????????????????????????????????????????????
# Joint Interpolator ? Kills snappy movement
# ???????????????????????????????????????????????????

class JointInterpolator:
    def __init__(self, n_joints=6, max_rate_rad_s=2.0, freq=30.0):
        self.max_delta    = max_rate_rad_s / freq
        self.current      = np.zeros(n_joints)
        self.target       = np.zeros(n_joints)
        self._initialized = False

    def set_target(self, joints):
        self.target = np.array(joints, dtype=float)
        if not self._initialized:
            self.current      = self.target.copy()
            self._initialized = True

    def step(self):
        diff = self.target - self.current
        for i in range(len(diff)):
            while diff[i] >  math.pi: diff[i] -= 2 * math.pi
            while diff[i] < -math.pi: diff[i] += 2 * math.pi
        step = np.clip(diff, -self.max_delta, self.max_delta)
        self.current += step
        return self.current.tolist()

    def reset(self, joints=None):
        if joints is not None:
            self.current      = np.array(joints, dtype=float)
            self.target       = self.current.copy()
            self._initialized = True
        else:
            self._initialized = False


# ???????????????????????????????????????????????????
# Forward Kinematics
# ???????????????????????????????????????????????????

def _Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def _Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

def _Txyz(x, y, z):
    m = np.eye(4); m[0,3] = x; m[1,3] = y; m[2,3] = z; return m

def forward_kinematics(j):
    pi = math.pi
    # The official ur_description URDF rotates the base by pi around Z!
    # We MUST include this so our iterative IK solver matches RViz exactly.
    T  = _Rz(pi)
    T  = T @ _Txyz(0, 0, 0.1625) @ _Rz(j[0])
    T  = T @ _Rx(pi/2) @ _Rz(j[1])
    T  = T @ _Txyz(-0.425, 0, 0) @ _Rz(j[2])
    T  = T @ _Txyz(-0.3922, 0, 0.1333) @ _Rz(j[3])
    T  = T @ _Txyz(0, -0.0997, 0) @ _Rx(pi/2) @ _Rz(j[4])
    T  = T @ _Txyz(0, 0.0996, 0) @ _Rz(pi) @ _Ry(pi) @ _Rx(pi/2) @ _Rz(j[5])
    return T[:3, 3].tolist()


# ???????????????????????????????????????????????????
# IK ? kept for AUTO mode (unchanged)
# ???????????????????????????????????????????????????

_last_j1 = 0.0
J1_DEADZONE = 0.08

def _raw_ik(tx, ty, tz,
            wrist_pitch=-math.pi/2,
            wrist_yaw=0.0):
    global _last_j1
    pi = math.pi
    L1, L2, D1 = 0.425, 0.3922, 0.1625
    
    # Since URDF rotates base by pi, we rotate the target by -pi internally
    tx_int = -tx
    ty_int = -ty
    
    r_horiz = math.sqrt(tx_int*tx_int + ty_int*ty_int)
    if r_horiz > J1_DEADZONE:
        j1 = math.atan2(ty_int, tx_int)
        _last_j1 = j1
    else:
        j1 = _last_j1

    r = max(0.01, r_horiz)
    h = tz - D1
    
    d = math.sqrt(r*r + h*h)
    MAX_REACH = L1 + L2 - 0.01
    MIN_REACH = 0.15
    if d > MAX_REACH:
        sc = MAX_REACH / d; r *= sc; h *= sc
    elif d < MIN_REACH:
        sc = MIN_REACH / d; r *= sc; h *= sc

    cos_j3 = (r*r + h*h - L1*L1 - L2*L2) / (2*L1*L2)
    cos_j3 = max(-1.0, min(1.0, cos_j3))
    
    # Mode 4 IK: To reach FORWARD (elbow pointing down in the internal frame)
    # The internal arm normally extends along -X. But j1 points the internal +X 
    # to the target. So we treat the target as being at -r, and invert j3.
    j3 = -math.acos(cos_j3)
    
    beta = math.atan2(h, -r)
    psi  = math.atan2(L2*math.sin(j3), L1 + L2*math.cos(j3))
    j2   = -(beta + psi)

    # Normalize j2 to (-pi, pi]
    if j2 > pi: j2 -= 2*pi
    if j2 < -pi: j2 += 2*pi

    # Auto-compute wrist_1 so gripper always points straight down
    wrist_roll = -(j2 + j3) - pi/2

    return [j1, j2, j3, wrist_roll, wrist_pitch, wrist_yaw]


def inverse_kinematics(tx, ty, tz,
                        wrist_pitch=-math.pi/2,
                        wrist_yaw=0.0,
                        iterations=5):
    target   = np.array([tx, ty, tz])
    adjusted = target.copy()
    joints   = _raw_ik(tx, ty, tz, wrist_pitch, wrist_yaw)
    for _ in range(iterations):
        actual   = np.array(forward_kinematics(joints))
        error    = target - actual
        adjusted = adjusted + error * 0.8
        joints   = _raw_ik(float(adjusted[0]), float(adjusted[1]), float(adjusted[2]),
                            wrist_pitch, wrist_yaw)
    pi = math.pi
    joints[0] = max(-pi,  min(pi,   joints[0]))
    joints[1] = max(-pi,  min(pi/2, joints[1]))
    joints[2] = max(-pi,  min(pi,   joints[2]))
    return joints


# ???????????????????????????????????????????????????
# AUTO Mode smooth interpolation
# ???????????????????????????????????????????????????

def cosine_interp(start, end, t):
    progress = 0.5 * (1.0 - math.cos(math.pi * t))
    return [s + progress * (e - s) for s, e in zip(start, end)]



# ---------------------------------------------------
# Main Node
# ---------------------------------------------------

# -- Scene geometry --------------------------------
BOX_POS        = [-0.30, -0.55, 0.00]
BOX_W, BOX_D, BOX_H = 0.32, 0.32, 0.14
BOX_WALL_T     = 0.015

CUBE_SPAWN_POSITIONS = [
    [-0.42, -0.12, 0.05],
    [-0.52, -0.24, 0.05],
    [-0.58, -0.36, 0.05],
    [-0.52, -0.48, 0.05],
    [-0.42, -0.60, 0.05],
]

GRIP_CLOSE_THRESH = 0.30
GRIP_OPEN_THRESH  = 0.70


class JointBridgeNode(Node):
    def __init__(self):
        super().__init__('gesture_arm_node')
        self.get_logger().info('[v13] Delta Cartesian TELEOP | Drop-box scene')

        self.joint_pub  = self.create_publisher(
            JointState,  '/joint_states',              10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.033, self.control_loop)

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint',      'wrist_2_joint',       'wrist_3_joint',
        ]

        self.joint_positions   = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.mode              = 'AUTO'
        self.gripper           = 1.0
        self.target_xyz        = [-0.40, -0.30, 0.30]
        self.spatial_pos       = [0.0, 0.0, 0.0]
        self.wrist_orientation = (-math.pi / 2, 0.0)  # (wrist_pitch, wrist_yaw)
        self.ee_pos            = [-0.40, -0.30, 0.30]
        self.tf_ok             = False
        self.time_counter      = 0.0

        self._teleop_ref_hand = None
        self._teleop_ref_ee   = None

        self.pose_filter  = PoseFilter(freq=30.0)
        self.joint_interp = JointInterpolator(max_rate_rad_s=4.5, freq=30.0)
        self.joint_interp.reset(self.joint_positions)

        self._teleop_recal_timer = 0.0
        self._joint_ema = None          # v16: joint-level EMA for post-IK smoothing
        self._prev_target = None        # v16: velocity dead zone tracking

        self._init_scene()
        self.get_logger().info('Scene ready (right side) | Awaiting tracker...')

    def _init_scene(self):
        colours = [
            [0.95, 0.20, 0.20],
            [0.20, 0.55, 0.95],
            [0.20, 0.90, 0.30],
            [0.95, 0.75, 0.10],
            [0.75, 0.20, 0.90],
        ]
        labels = ['Red', 'Blue', 'Green', 'Yellow', 'Purple']
        
        self.cubes = []
        for i in range(5):
            # Scatter cubes randomly on the left side, away from the box
            cx = random.uniform(-0.65, -0.35)
            cy = random.uniform(-0.55, -0.05)
            self.cubes.append({
                'pos':    [cx, cy, 0.05],
                'color':  colours[i],
                'label':  labels[i],
                'held':   False,
                'placed': False,
            })
            
        self.held_cube            = None
        self.score                = 0
        self.done_time            = 0.0
        self.auto_state           = 'SCAN'
        self.auto_target_cube     = -1
        self.auto_progress        = 0.0
        self.auto_gripper_target  = 1.0
        self.auto_speed           = 0.55
        self.auto_waypoint_start  = list(self.joint_positions)
        self.auto_waypoint_end    = list(self.joint_positions)

    def _reset_scene(self):
        self._init_scene()
        self.joint_interp.reset(self.joint_positions)
        self.get_logger().info('[RESET] Scene reset!')

    def control_loop(self):
        dt = 0.033
        self.time_counter += dt

        latest_data = None
        while True:
            try:
                data, _ = self.sock.recvfrom(BUFFER_SIZE)
                latest_data = data
            except BlockingIOError:
                break

        if latest_data is not None:
            try:
                msg      = json.loads(latest_data.decode('utf-8'))
                new_mode = msg.get('mode', 'AUTO')
                if new_mode in ('AUTO', 'STOP', 'TELEOP'):
                    if new_mode != self.mode:
                        self.get_logger().info(
                            f'[MODE] {self.mode} -> {new_mode}')
                        if new_mode == 'AUTO':
                            self._reset_auto()
                        elif new_mode == 'TELEOP':
                            self.joint_interp.reset(self.joint_positions)
                            self.pose_filter.reset()
                            self._teleop_ref_hand = None
                            self._teleop_ref_ee   = None
                        elif new_mode == 'STOP':
                            # Freeze at EXACT current position
                            self._stop_frozen_joints = list(self.joint_positions)
                            self.joint_interp.reset(self.joint_positions)
                            self._stop_cube_released = False

                    if 'gripper' in msg:
                        g = float(msg['gripper'])
                        if not math.isnan(g):
                            self.gripper = max(0.0, min(1.0, g))

                    if 'spatial' in msg:
                        s = msg['spatial']
                        if len(s) == 3:
                            self.spatial_pos = s

                    if 'wrist' in msg:
                        w = msg['wrist']
                        if len(w) >= 2:
                            self.wrist_orientation = (
                                float(w[1]), float(w[2]) if len(w) > 2 else 0.0)

                self.mode = new_mode
            except (json.JSONDecodeError, KeyError):
                pass

        if   self.mode == 'TELEOP': self._teleop_step()
        elif self.mode == 'AUTO':   self._auto_step(dt)
        elif self.mode == 'STOP':   self._stop_step()

        # Use TF for the gripper ball so it sits exactly on the URDF tool0 frame.
        # Fall back to FK while TF hasn't warmed up yet.
        fk_pos = forward_kinematics(self.joint_positions)
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time())
            self.ee_pos = [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ]
        except Exception:
            self.ee_pos = fk_pos

        # Cube attachment always uses FK so it tracks commanded pose instantly
        # (TF has a small lag that causes cubes to "float" slightly behind)
        self._fk_pos = fk_pos

        self._update_gripper()
        self._publish_joint_states()
        self._publish_markers()

    def _stop_step(self):
        """Hold arm at the EXACT position it was when STOP was triggered."""
        # Lock joint positions to the frozen snapshot
        if hasattr(self, '_stop_frozen_joints'):
            self.joint_positions = list(self._stop_frozen_joints)

        # Safety: release held cube (only once)
        if not getattr(self, '_stop_cube_released', True):
            self._stop_cube_released = True
            if self.held_cube is not None:
                cube = self.cubes[self.held_cube]
                cube['held'] = False
                self.get_logger().info(f'[E-STOP] Released {cube["label"]}')
                self.held_cube = None

        # Slowly open gripper
        self.gripper = min(1.0, self.gripper + 0.05)

        # Clear teleop state so re-entering TELEOP recalibrates fresh
        self._teleop_ref_hand = None
        self._teleop_ref_ee   = None
        self._joint_ema       = None
        self._prev_target     = None

    def _teleop_step(self):
        SCALE_XZ = 1.8
        SCALE_Y  = 3.0
        LAT_DEAD = 0.02
        RECAL_INTERVAL = 5.0
        RECAL_RATE     = 0.02
        VEL_DEAD = 0.003     # velocity dead zone (m) — below this, arm freezes completely
        JOINT_EMA = 0.35     # joint-level EMA alpha (0=frozen, 1=instant, 0.35=smooth blend)

        user_x, user_y, user_z = self.spatial_pos

        # Lateral axis: dedicated smoother filter
        fx_lat = self.pose_filter.filter_lateral(user_x)
        # Forward/vertical: standard filter
        _, fy, fz = self.pose_filter.filter_position(user_x, user_y, user_z)

        if self._teleop_ref_hand is None:
            self._teleop_ref_hand = [fx_lat, fy, fz]
            self._teleop_ref_ee   = list(self.ee_pos)
            self._teleop_recal_timer = 0.0
            self._prev_target = list(self.ee_pos)
            self._joint_ema = None
            self.get_logger().info(
                f'[TELEOP] Calibrated | '
                f'hand=({fx_lat:.2f},{fy:.2f},{fz:.2f}) '
                f'EE=({self.ee_pos[0]:.2f},{self.ee_pos[1]:.2f},{self.ee_pos[2]:.2f})')

        # Periodic re-calibration
        self._teleop_recal_timer += 0.033
        if self._teleop_recal_timer >= RECAL_INTERVAL:
            self._teleop_recal_timer = 0.0
            self._teleop_ref_hand[0] += (fx_lat - self._teleop_ref_hand[0]) * RECAL_RATE
            self._teleop_ref_hand[1] += (fy - self._teleop_ref_hand[1]) * RECAL_RATE
            self._teleop_ref_hand[2] += (fz - self._teleop_ref_hand[2]) * RECAL_RATE

        # Lateral delta with deadzone
        dx_raw = fx_lat - self._teleop_ref_hand[0]
        dx = math.copysign(max(0.0, abs(dx_raw) - LAT_DEAD), dx_raw)

        dy = fy  - self._teleop_ref_hand[1]
        dz = fz  - self._teleop_ref_hand[2]

        ref_x, ref_y, ref_z = self._teleop_ref_ee
        target_x = max(-0.75, min(-0.08, ref_x - dz * SCALE_XZ))
        target_y = max(-0.78, min( 0.78, ref_y - dx * SCALE_Y))
        target_z = max( 0.02, min( 0.75, ref_z + dy * SCALE_XZ))

        # Fix 1: Velocity dead zone — if hand barely moved, freeze target completely
        if self._prev_target is not None:
            delta = math.sqrt(
                (target_x - self._prev_target[0])**2 +
                (target_y - self._prev_target[1])**2 +
                (target_z - self._prev_target[2])**2)
            if delta < VEL_DEAD:
                target_x, target_y, target_z = self._prev_target
        self._prev_target = [target_x, target_y, target_z]

        # Snap assist
        if self.gripper < 0.6 and self.held_cube is None:
            best_dist = 0.18
            best_cube_pos = None
            for cube in self.cubes:
                if cube['held'] or cube['placed']:
                    continue
                d = self._dist([target_x, target_y, target_z], cube['pos'])
                if d < best_dist:
                    best_dist = d
                    best_cube_pos = cube['pos']
            if best_cube_pos is not None:
                snap_strength = 0.20
                target_x += (best_cube_pos[0] - target_x) * snap_strength
                target_y += (best_cube_pos[1] - target_y) * snap_strength
                target_z += (best_cube_pos[2] - target_z) * snap_strength

        w_p, w_y = self.wrist_orientation
        fw_pitch, fw_yaw = self.pose_filter.filter_orientation(0.0, w_p, w_y)[1:]

        # Fix 4: More IK iterations (5 → 8) for better accuracy
        joints = inverse_kinematics(
            target_x, target_y, target_z,
            fw_pitch, fw_yaw, iterations=8)

        pi = math.pi
        joints[0] = max(-pi,   min(pi,   joints[0]))
        joints[1] = max(-pi,   min(pi/2, joints[1]))
        joints[2] = max(-pi,   min(pi,   joints[2]))
        joints[3] = max(-pi,   min(pi,   joints[3]))
        joints[4] = max(-pi,   min(pi/2, joints[4]))
        joints[5] = max(-pi,   min(pi,   joints[5]))

        # Fix 3: Joint-level EMA — smooths out IK discontinuities
        if self._joint_ema is None:
            self._joint_ema = list(joints)
        else:
            for i in range(6):
                self._joint_ema[i] += JOINT_EMA * (joints[i] - self._joint_ema[i])

        self.joint_interp.set_target(self._joint_ema)
        self.joint_positions = self.joint_interp.step()
        self.target_xyz = [target_x, target_y, target_z]

    def _reset_auto(self):
        self.auto_state          = 'SCAN'
        self.auto_target_cube    = -1
        self.auto_progress       = 0.0
        self.auto_gripper_target = 1.0
        self.gripper             = 1.0

    def _find_next_cube(self):
        for i, c in enumerate(self.cubes):
            if not c['placed'] and not c['held']:
                return i
        return -1

    def _set_waypoint(self, tx, ty, tz):
        self.auto_waypoint_start = list(self.joint_positions)
        self.auto_waypoint_end   = inverse_kinematics(tx, ty, tz)
        self.auto_progress       = 0.0

    def _advance(self, speed_mult=1.0):
        self.auto_progress += 0.033 * self.auto_speed * speed_mult
        if self.auto_progress >= 1.0:
            self.auto_progress = 1.0
        self.joint_positions = cosine_interp(
            self.auto_waypoint_start, self.auto_waypoint_end,
            self.auto_progress)
        return self.auto_progress >= 1.0

    def _auto_step(self, dt):
        if self.auto_state == 'SCAN':
            idx = self._find_next_cube()
            if idx < 0:
                self.auto_state = 'DONE'
                self.done_time  = self.time_counter
                return
            self.auto_target_cube    = idx
            self.auto_gripper_target = 1.0
            cube = self.cubes[idx]
            self._set_waypoint(
                cube['pos'][0], cube['pos'][1], cube['pos'][2] + 0.22)
            self.auto_state = 'LIFT_ABOVE'
            self.get_logger().info(
                f'[AUTO] Targeting {cube["label"]} @ {cube["pos"]}')

        elif self.auto_state == 'LIFT_ABOVE':
            if self._advance(1.0):
                cube = self.cubes[self.auto_target_cube]
                self._set_waypoint(
                    cube['pos'][0], cube['pos'][1], cube['pos'][2] + 0.03)
                self.auto_state = 'DESCEND_PICK'
                self.get_logger().info(
                    f'[AUTO] Descending to {cube["label"]}')

        elif self.auto_state == 'DESCEND_PICK':
            if self._advance(0.55):
                self.auto_gripper_target = 0.0
                self.auto_progress       = 0.0
                self.auto_state = 'GRIPPING'
                self.get_logger().info(
                    f'[AUTO] Gripping {self.cubes[self.auto_target_cube]["label"]}')

        elif self.auto_state == 'GRIPPING':
            self.auto_progress += 0.033 * 2.0
            if self.auto_progress >= 1.0:
                cube           = self.cubes[self.auto_target_cube]
                cube['held']   = True
                self.held_cube = self.auto_target_cube
                self._set_waypoint(
                    cube['pos'][0], cube['pos'][1], cube['pos'][2] + 0.28)
                self.auto_state = 'LIFT_CARRY'
                self.get_logger().info(
                    f'[AUTO] Lifting {cube["label"]}')

        elif self.auto_state == 'LIFT_CARRY':
            if self._advance(0.8):
                bx, by, bz = BOX_POS
                self._set_waypoint(bx, by, bz + BOX_H + 0.20)
                self.auto_state = 'MOVE_TO_BOX'
                self.get_logger().info('[AUTO] Carrying to drop box')

        elif self.auto_state == 'MOVE_TO_BOX':
            if self._advance(0.85):
                bx, by, bz = BOX_POS
                self._set_waypoint(bx, by, bz + BOX_H * 0.5)
                self.auto_state = 'DESCEND_DROP'
                self.get_logger().info('[AUTO] Descending into box')

        elif self.auto_state == 'DESCEND_DROP':
            if self._advance(0.45):
                self.auto_gripper_target = 1.0
                self.auto_progress       = 0.0
                self.auto_state = 'RELEASING'
                self.get_logger().info('[AUTO] Releasing cube into box')

        elif self.auto_state == 'RELEASING':
            self.auto_progress += 0.033 * 1.8
            if self.auto_progress >= 1.0:
                cube = self.cubes[self.auto_target_cube]
                cube['held']   = False
                cube['placed'] = True
                cube['pos']    = [
                    BOX_POS[0],
                    BOX_POS[1],
                    BOX_POS[2] + 0.03 + self.score * 0.055,
                ]
                self.held_cube = None
                self.score    += 1
                self.get_logger().info(
                    f'[OK] {cube["label"]} placed! Score: {self.score}/{len(self.cubes)}')
                self.auto_waypoint_start = list(self.joint_positions)
                self.auto_waypoint_end   = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
                self.auto_progress       = 0.0
                self.auto_state = 'HOME'

        elif self.auto_state == 'HOME':
            if self._advance(1.2):
                self.auto_state = 'SCAN'

        elif self.auto_state == 'DONE':
            t = self.time_counter
            self.joint_positions = [
                math.sin(t * 0.7) * 0.6,
                -1.2 + math.cos(t * 0.5) * 0.25,
                1.5  + math.sin(t * 0.9) * 0.15,
                -1.57, -1.57, 0.0,
            ]
            if self.time_counter - self.done_time > 6.0:
                self._reset_scene()

        if self.mode == 'AUTO':
            self.gripper = 0.8 * self.gripper + 0.2 * self.auto_gripper_target

        if self.held_cube is not None:
            self.cubes[self.held_cube]['pos'] = list(getattr(self, '_fk_pos', self.ee_pos))

    def _update_gripper(self):
        if self.mode != 'TELEOP':
            return
        if self.gripper < GRIP_CLOSE_THRESH:
            if self.held_cube is None:
                for i, cube in enumerate(self.cubes):
                    if cube['held'] or cube['placed']:
                        continue
                    if self._dist(self.ee_pos, cube['pos']) < PICK_DISTANCE:
                        cube['held']   = True
                        self.held_cube = i
                        self.get_logger().info(f'[PICK] {cube["label"]}!')
                        break
        elif self.gripper > GRIP_OPEN_THRESH:
            if self.held_cube is not None:
                cube         = self.cubes[self.held_cube]
                cube['held'] = False
                in_box = (
                    abs(cube['pos'][0] - BOX_POS[0]) < BOX_W / 2 and
                    abs(cube['pos'][1] - BOX_POS[1]) < BOX_D / 2 and
                    cube['pos'][2] < BOX_POS[2] + BOX_H + 0.12
                )
                if in_box:
                    cube['placed'] = True
                    cube['pos']    = [
                        BOX_POS[0], BOX_POS[1],
                        BOX_POS[2] + 0.03 + self.score * 0.055,
                    ]
                    self.score += 1
                    self.get_logger().info(
                        f'[OK] {cube["label"]} in box! Score: {self.score}/{len(self.cubes)}')
                else:
                    self.get_logger().info(f'[DROP] {cube["label"]}')
                self.held_cube = None

        if self.held_cube is not None:
            self.cubes[self.held_cube]['pos'] = list(getattr(self, '_fk_pos', self.ee_pos))

    def _publish_markers(self):
        ma    = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 0

        # v15: Premium floor — soft dark grid
        f = self._mk('env', mid, Marker.CUBE, stamp); mid += 1
        f.pose.position.x = -0.25
        f.pose.position.y = -0.35
        f.pose.position.z = -0.008
        f.scale.x, f.scale.y, f.scale.z = 2.0, 2.0, 0.006
        f.color.r, f.color.g, f.color.b, f.color.a = 0.12, 0.13, 0.16, 0.70
        ma.markers.append(f)

        # v15: Grid lines on floor (subtle)
        for gi in range(-4, 5):
            gl = self._mk('grid', mid, Marker.CUBE, stamp); mid += 1
            gl.pose.position.x = -0.25
            gl.pose.position.y = -0.35 + gi * 0.20
            gl.pose.position.z = -0.004
            gl.scale.x, gl.scale.y, gl.scale.z = 2.0, 0.003, 0.001
            gl.color.r, gl.color.g, gl.color.b, gl.color.a = 0.25, 0.28, 0.32, 0.30
            ma.markers.append(gl)
            gl2 = self._mk('grid', mid, Marker.CUBE, stamp); mid += 1
            gl2.pose.position.x = -0.25 + gi * 0.20
            gl2.pose.position.y = -0.35
            gl2.pose.position.z = -0.004
            gl2.scale.x, gl2.scale.y, gl2.scale.z = 0.003, 2.0, 0.001
            gl2.color.r, gl2.color.g, gl2.color.b, gl2.color.a = 0.25, 0.28, 0.32, 0.30
            ma.markers.append(gl2)

        # v15: Sleek metallic table
        tb = self._mk('env', mid, Marker.CUBE, stamp); mid += 1
        tb.pose.position.x = -0.47
        tb.pose.position.y = -0.36
        tb.pose.position.z = -0.015
        tb.scale.x, tb.scale.y, tb.scale.z = 0.70, 0.85, 0.025
        tb.color.r, tb.color.g, tb.color.b, tb.color.a = 0.30, 0.32, 0.35, 0.95
        ma.markers.append(tb)
        # Table edge highlight
        te = self._mk('env', mid, Marker.CUBE, stamp); mid += 1
        te.pose.position.x = -0.47
        te.pose.position.y = -0.36
        te.pose.position.z = -0.002
        te.scale.x, te.scale.y, te.scale.z = 0.71, 0.86, 0.003
        te.color.r, te.color.g, te.color.b, te.color.a = 0.45, 0.50, 0.55, 0.60
        ma.markers.append(te)

        # Drop Box (floor + 4 walls)
        bx, by, bz = BOX_POS
        w, d, h, t = BOX_W, BOX_D, BOX_H, BOX_WALL_T
        # v15: Sleek teal box
        bc = (0.15, 0.70, 0.75)
        ba = 0.55

        def box_panel(x, y, z, sx, sy, sz, alpha=ba):
            nonlocal mid
            p = self._mk('box', mid, Marker.CUBE, stamp)
            mid += 1
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = z
            p.scale.x, p.scale.y, p.scale.z = sx, sy, sz
            p.color.r, p.color.g, p.color.b = bc
            p.color.a = alpha
            return p

        ma.markers.append(box_panel(bx, by, bz + t/2, w, d, t, alpha=0.95))
        ma.markers.append(box_panel(bx + w/2 - t/2, by, bz + h/2, t, d, h))
        ma.markers.append(box_panel(bx - w/2 + t/2, by, bz + h/2, t, d, h))
        ma.markers.append(box_panel(bx, by + d/2 - t/2, bz + h/2, w, t, h))
        ma.markers.append(box_panel(bx, by - d/2 + t/2, bz + h/2, w, t, h))

        # Box label
        bl = self._mk('box', mid, Marker.TEXT_VIEW_FACING, stamp); mid += 1
        bl.pose.position.x = bx
        bl.pose.position.y = by
        bl.pose.position.z = bz + h + 0.08
        bl.scale.z = 0.04
        bl.color.r, bl.color.g, bl.color.b, bl.color.a = 1.0, 0.7, 0.1, 1.0
        bl.text = f'DROP BOX  {self.score}/{len(self.cubes)}'
        ma.markers.append(bl)

        # Cubes (with v14 proximity glow)
        for i, cube in enumerate(self.cubes):
            m = self._mk('cubes', i, Marker.CUBE, stamp)
            m.pose.position.x = cube['pos'][0]
            m.pose.position.y = cube['pos'][1]
            m.pose.position.z = cube['pos'][2]
            m.scale.x = m.scale.y = m.scale.z = CUBE_SIZE
            m.color.r, m.color.g, m.color.b = cube['color']
            m.color.a = 0.45 if cube['placed'] else 1.0
            ma.markers.append(m)

            cl = self._mk('cube_labels', i, Marker.TEXT_VIEW_FACING, stamp)
            cl.pose.position.x = cube['pos'][0]
            cl.pose.position.y = cube['pos'][1]
            cl.pose.position.z = cube['pos'][2] + 0.06
            cl.scale.z = 0.025
            cl.color.r, cl.color.g, cl.color.b = cube['color']
            cl.color.a = 0.9
            cl.text = cube['label'] + (' [OK]' if cube['placed'] else '')

            # v14: Proximity glow ring — shows when EE is close enough to grab
            if self.mode == 'TELEOP' and not cube['placed'] and not cube['held']:
                dist_to_ee = self._dist(self.ee_pos, cube['pos'])
                if dist_to_ee < 0.22:  # within approach range
                    glow = self._mk('glow', i, Marker.CYLINDER, stamp)
                    glow.pose.position.x = cube['pos'][0]
                    glow.pose.position.y = cube['pos'][1]
                    glow.pose.position.z = cube['pos'][2] - 0.01
                    glow_size = 0.12 if dist_to_ee < PICK_DISTANCE else 0.08
                    glow.scale.x = glow.scale.y = glow_size
                    glow.scale.z = 0.005
                    if dist_to_ee < PICK_DISTANCE:
                        glow.color.r, glow.color.g, glow.color.b = 0.0, 1.0, 0.3
                        cl.text = cube['label'] + ' [GRAB!]'
                    else:
                        glow.color.r, glow.color.g, glow.color.b = 1.0, 1.0, 0.2
                    glow.color.a = 0.5
                    ma.markers.append(glow)

            ma.markers.append(cl)

        # v15: Two-finger gripper visualization
        grip_val = max(0.0, min(1.0, float(self.gripper)))
        if math.isnan(grip_val): grip_val = 1.0
        finger_spread = 0.01 + grip_val * 0.025  # fingers spread apart when open
        finger_color = (1.0, 0.15, 0.15) if grip_val < 0.5 else (0.15, 1.0, 0.30)

        # Gripper base (small cylinder at wrist)
        gb = self._mk('gripper', 0, Marker.CYLINDER, stamp)
        gb.pose.position.x = self.ee_pos[0]
        gb.pose.position.y = self.ee_pos[1]
        gb.pose.position.z = self.ee_pos[2] + 0.015
        gb.scale.x = gb.scale.y = 0.025
        gb.scale.z = 0.02
        gb.color.r, gb.color.g, gb.color.b = 0.4, 0.42, 0.45
        gb.color.a = 0.9
        ma.markers.append(gb)

        # Left finger
        lf = self._mk('gripper', 1, Marker.CUBE, stamp)
        lf.pose.position.x = self.ee_pos[0]
        lf.pose.position.y = self.ee_pos[1] - finger_spread
        lf.pose.position.z = self.ee_pos[2] - 0.01
        lf.scale.x, lf.scale.y, lf.scale.z = 0.008, 0.005, 0.035
        lf.color.r, lf.color.g, lf.color.b = finger_color
        lf.color.a = 0.90
        ma.markers.append(lf)

        # Right finger
        rf = self._mk('gripper', 2, Marker.CUBE, stamp)
        rf.pose.position.x = self.ee_pos[0]
        rf.pose.position.y = self.ee_pos[1] + finger_spread
        rf.pose.position.z = self.ee_pos[2] - 0.01
        rf.scale.x, rf.scale.y, rf.scale.z = 0.008, 0.005, 0.035
        rf.color.r, rf.color.g, rf.color.b = finger_color
        rf.color.a = 0.90
        ma.markers.append(rf)

        # Grip status dot (small sphere between fingers)
        gd = self._mk('gripper', 3, Marker.SPHERE, stamp)
        gd.pose.position.x = self.ee_pos[0]
        gd.pose.position.y = self.ee_pos[1]
        gd.pose.position.z = self.ee_pos[2] - 0.015
        gd.scale.x = gd.scale.y = gd.scale.z = 0.012
        gd.color.r, gd.color.g, gd.color.b = finger_color
        gd.color.a = 0.7
        ma.markers.append(gd)

        # v14: Target ghost sphere — shows where arm is TRYING to go in TELEOP
        if self.mode == 'TELEOP':
            ghost = self._mk('ghost', 0, Marker.SPHERE, stamp)
            ghost.pose.position.x = self.target_xyz[0]
            ghost.pose.position.y = self.target_xyz[1]
            ghost.pose.position.z = self.target_xyz[2]
            ghost.scale.x = ghost.scale.y = ghost.scale.z = 0.04
            ghost.color.r, ghost.color.g, ghost.color.b = 0.3, 0.7, 1.0
            ghost.color.a = 0.35
            ma.markers.append(ghost)

        # Status HUD
        st = self._mk('status', 0, Marker.TEXT_VIEW_FACING, stamp)
        st.pose.position.x = 0.0
        st.pose.position.y = 0.0
        st.pose.position.z = 0.90
        st.scale.z = 0.055
        st.color.r, st.color.g, st.color.b, st.color.a = 1.0, 1.0, 1.0, 0.90
        grip_str = 'CLOSED' if self.gripper < 0.5 else 'OPEN'
        if self.mode == 'AUTO':
            lbl = self.auto_state
            if 0 <= self.auto_target_cube < len(self.cubes):
                if self.auto_state not in ('DONE', 'HOME', 'SCAN'):
                    lbl += f' -> {self.cubes[self.auto_target_cube]["label"]}'
            st.text = f'AUTO: {lbl}'
        elif self.mode == 'STOP':
            st.text = f'[E-STOP] | {grip_str}'
        else:
            ee = self.ee_pos
            st.text = f'TELEOP | EE({ee[0]:.2f},{ee[1]:.2f},{ee[2]:.2f}) | {grip_str}'
        ma.markers.append(st)

        sc = self._mk('status', 1, Marker.TEXT_VIEW_FACING, stamp)
        sc.pose.position.x = 0.0
        sc.pose.position.y = 0.0
        sc.pose.position.z = 0.78
        sc.scale.z = 0.048
        if self.score == len(self.cubes):
            sc.color.r, sc.color.g, sc.color.b, sc.color.a = 0.2, 1.0, 0.3, 1.0
            sc.text = '*** ALL PLACED! ***'
        else:
            sc.color.r, sc.color.g, sc.color.b, sc.color.a = 1.0, 0.85, 0.2, 0.9
            sc.text = f'Score: {self.score}/{len(self.cubes)}'
        ma.markers.append(sc)

        self.marker_pub.publish(ma)

    def _mk(self, ns, marker_id, marker_type, stamp):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp    = stamp
        m.ns              = ns
        m.id              = marker_id
        m.type            = marker_type
        m.action          = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec    = 0
        return m

    def _make_marker(self, ns, marker_id, marker_type, stamp):
        return self._mk(ns, marker_id, marker_type, stamp)

    def _publish_joint_states(self):
        msg          = JointState()
        msg.header   = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self.joint_names
        msg.position = [float(p) for p in self.joint_positions]
        self.joint_pub.publish(msg)

    @staticmethod
    def _dist(a, b):
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

    @staticmethod
    def _dist_2d(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def destroy_node(self):
        self.get_logger().info('Shutting down.')
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
