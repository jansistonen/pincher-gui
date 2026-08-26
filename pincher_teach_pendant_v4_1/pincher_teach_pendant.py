#!/usr/bin/env python3
import argparse, json, math, sys, time
from dataclasses import dataclass, asdict
from typing import Dict, List

from PyQt5.QtCore import QObject, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QPalette
from PyQt5.QtWidgets import (
    QApplication, QAbstractItemView, QDoubleSpinBox, QFileDialog, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QMainWindow, QMessageBox,
    QPushButton, QSlider, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

JOINTS = ["waist", "shoulder", "elbow", "wrist_angle"]
DEMO_LIMITS = {
    "waist": (-180, 180),
    "shoulder": (-105, 105),
    "elbow": (-115, 85),
    "wrist_angle": (-95, 115),
}
DEMO_SLEEP = {
    "waist": 0.0,
    "shoulder": -80.0,
    "elbow": 70.0,
    "wrist_angle": 30.0,
}
VALID_GRIPPER_ACTIONS = ("KEEP", "OPEN", "CLOSE")
GRIPPER_ACTION_SETTLE_MS = 800


@dataclass
class PoseStep:
    name: str
    waist: float
    shoulder: float
    elbow: float
    wrist_angle: float
    move_time: float = 2.0
    wait_time: float = 0.5
    gripper: str = "KEEP"

    def joints(self):
        return {j: getattr(self, j) for j in JOINTS}


class RobotBackend(QObject):
    joint_state_changed = pyqtSignal(dict)
    gripper_state_changed = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    motion_finished = pyqtSignal()
    error_occurred = pyqtSignal(str)

    def get_joint_positions_deg(self):
        raise NotImplementedError

    def get_joint_limits_deg(self):
        raise NotImplementedError

    def get_home_pose_deg(self):
        return {j: 0.0 for j in JOINTS}

    def get_sleep_pose_deg(self):
        raise NotImplementedError

    def move_joints_deg(self, targets, move_time=2.0):
        raise NotImplementedError

    def command_gripper(self, action):
        raise NotImplementedError

    def stop_gripper(self):
        raise NotImplementedError

    def set_gripper_pressure(self, percent):
        raise NotImplementedError

    def get_gripper_program_state(self):
        raise NotImplementedError

    def get_ee_pose(self):
        """Return actual x/y/z [m] and pitch_deg, or None if unavailable."""
        return None

    def get_cartesian_target_pose(self):
        """Return the maintained Cartesian jog target, if available."""
        return self.get_ee_pose()

    def reset_cartesian_target(self):
        return self.get_ee_pose()

    def jog_cartesian(self, dx=0.0, dy=0.0, dz=0.0, dpitch_deg=0.0, move_time=0.7):
        self.error_occurred.emit("Cartesian jogging is not available in this backend.")
        return False

    def hold_current(self):
        raise NotImplementedError

    def close(self):
        pass


class DemoBackend(RobotBackend):
    def __init__(self):
        super().__init__()
        self.cur = {j: 0.0 for j in JOINTS}
        self.start = self.cur.copy()
        self.target = self.cur.copy()
        self.step = 0
        self.steps = 1
        self.gripper_state = "OPEN"
        self.gripper_pressure = 50
        self.demo_ee = {"x": 0.25, "y": 0.0, "z": 0.15, "pitch_deg": 0.0}
        self.demo_cart_target = self.demo_ee.copy()
        self.timer = QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._tick)

    def get_joint_positions_deg(self):
        return self.cur.copy()

    def get_joint_limits_deg(self):
        return DEMO_LIMITS.copy()

    def get_sleep_pose_deg(self):
        return DEMO_SLEEP.copy()

    def move_joints_deg(self, targets, move_time=2.0):
        for j in JOINTS:
            lo, hi = DEMO_LIMITS[j]
            if not lo <= targets[j] <= hi:
                self.error_occurred.emit(f"{j}: target outside limits")
                return False
        self.start = self.cur.copy()
        self.target = targets.copy()
        self.step = 0
        self.steps = max(1, int(max(.2, move_time) * 1000 / 20))
        self.timer.start()
        self.status_changed.emit("MOVING (DEMO)")
        return True

    def _tick(self):
        self.step += 1
        t = min(1.0, self.step / self.steps)
        s = t * t * (3 - 2 * t)
        for j in JOINTS:
            self.cur[j] = self.start[j] + (self.target[j] - self.start[j]) * s
        self.joint_state_changed.emit(self.cur.copy())
        if t >= 1:
            self.timer.stop()
            self.status_changed.emit("READY (DEMO)")
            self.motion_finished.emit()

    def command_gripper(self, action):
        action = action.upper()
        if action not in ("OPEN", "CLOSE"):
            return False
        self.gripper_state = action
        self.gripper_state_changed.emit(action)
        return True

    def stop_gripper(self):
        return True

    def set_gripper_pressure(self, percent):
        self.gripper_pressure = max(0, min(100, int(percent)))

    def get_gripper_program_state(self):
        return self.gripper_state

    def get_ee_pose(self):
        return self.demo_ee.copy()

    def get_cartesian_target_pose(self):
        return self.demo_cart_target.copy()

    def reset_cartesian_target(self):
        self.demo_cart_target = self.demo_ee.copy()
        return self.demo_cart_target.copy()

    def jog_cartesian(self, dx=0.0, dy=0.0, dz=0.0, dpitch_deg=0.0, move_time=0.7):
        # Apply increments to a maintained target so +step followed by -step
        # returns exactly to the previous Cartesian target.
        target = self.demo_cart_target.copy()
        target["x"] += float(dx)
        target["y"] += float(dy)
        target["z"] += float(dz)
        target["pitch_deg"] = max(
            -85.0, min(85.0, target["pitch_deg"] + float(dpitch_deg))
        )
        self.demo_cart_target = target
        self.demo_ee = target.copy()
        self.status_changed.emit("CARTESIAN JOG (DEMO)")
        return True

    def hold_current(self):
        self.timer.stop()
        self.status_changed.emit("HOLD (DEMO)")


class Ros2PincherBackend(RobotBackend):
    """ROS 2 Humble backend using Interbotix xs_sdk topics/services directly."""

    def __init__(self, robot_name="px100"):
        super().__init__()
        try:
            import rclpy
            import numpy as np
            import modern_robotics as mr
            import interbotix_common_modules.angle_manipulation as ang
            from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
            from sensor_msgs.msg import JointState
            from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
            from interbotix_xs_msgs.srv import RegisterValues, RobotInfo
        except ImportError as e:
            raise RuntimeError(
                "ROS2/Interbotix Python modules not found. Source /opt/ros/humble "
                "and ~/interbotix_ws/install/setup.bash first."
            ) from e

        self.rclpy = rclpy
        self.np = np
        self.mr = mr
        self.ang = ang
        self.robot_des = getattr(mrd, robot_name)
        self.JointGroupCommand = JointGroupCommand
        self.JointSingleCommand = JointSingleCommand
        self.RegisterValues = RegisterValues
        self.RobotInfo = RobotInfo
        self.ns = f"/{robot_name}"

        self.current_rad = {}
        self.current_deg = {}
        self.last_msg = 0.0

        # Cartesian jog state. Keeping an explicit target is important:
        # X+ 5 mm then X- 5 mm must return to the same target instead of
        # accumulating measurement/interpolation error from an in-flight arm.
        self.cartesian_target_pose = None
        self.pending_cartesian_target = None
        self.motion_busy = False

        # Gripper follows the same default pressure mapping as the official
        # Interbotix Python interface: 0% -> 150 PWM, 100% -> 350 PWM.
        self.gripper_pressure_lower = 150.0
        self.gripper_pressure_upper = 350.0
        self.gripper_pressure_percent = 50
        self.gripper_pwm = 250.0
        self.gripper_effort = 0.0
        self.gripper_moving = False
        self.last_gripper_action = None
        self.left_finger_name = None
        self.left_finger_pos = None
        self.left_finger_lower = None
        self.left_finger_upper = None

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("pincher_teach_pendant")
        self.pub = self.node.create_publisher(
            JointGroupCommand, f"{self.ns}/commands/joint_group", 10
        )
        self.gripper_pub = self.node.create_publisher(
            JointSingleCommand, f"{self.ns}/commands/joint_single", 10
        )
        self.node.create_subscription(
            JointState, f"{self.ns}/joint_states", self._js_cb, 10
        )
        self.info_cli = self.node.create_client(
            RobotInfo, f"{self.ns}/get_robot_info"
        )
        self.reg_cli = self.node.create_client(
            RegisterValues, f"{self.ns}/set_motor_registers"
        )

        self._read_arm_info()
        self._read_gripper_info()
        self._wait_joint_states()
        self.reset_cartesian_target()

        self.spin_timer = QTimer()
        self.spin_timer.setInterval(10)
        self.spin_timer.timeout.connect(self._spin)
        self.spin_timer.start()

        self.motion_timer = QTimer()
        self.motion_timer.setSingleShot(True)
        self.motion_timer.timeout.connect(self._motion_done)

        self.status_timer = QTimer()
        self.status_timer.setInterval(500)
        self.status_timer.timeout.connect(self._connection_check)
        self.status_timer.start()

        self.set_gripper_pressure(50)

    def _spin(self):
        if self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def _call_robot_info(self, cmd_type, name):
        if not self.info_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                f"{self.ns}/get_robot_info not found. Start xsarm_control first."
            )
        req = self.RobotInfo.Request()
        req.cmd_type = cmd_type
        req.name = name
        fut = self.info_cli.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"Could not read RobotInfo for {name} from xs_sdk.")
        return fut.result()

    def _read_arm_info(self):
        x = self._call_robot_info("group", "arm")
        self.order = list(x.joint_names)
        if len(self.order) != 4 or any(j not in self.order for j in JOINTS):
            raise RuntimeError(f"Unexpected arm joint list: {self.order}")
        if x.profile_type != "time" or x.mode != "position":
            raise RuntimeError(
                f"Arm must use position + time profile "
                f"(got mode={x.mode}, profile={x.profile_type})."
            )
        self.lo = dict(zip(self.order, x.joint_lower_limits))
        self.hi = dict(zip(self.order, x.joint_upper_limits))
        self.vel = dict(zip(self.order, x.joint_velocity_limits))
        self.sleep = dict(zip(self.order, x.joint_sleep_positions))

    def _read_gripper_info(self):
        x = self._call_robot_info("single", "gripper")
        if x.mode != "pwm":
            raise RuntimeError(
                f"Gripper operating mode is '{x.mode}', expected 'pwm'. "
                "Use the standard PX100 mode configuration."
            )
        if not x.joint_names:
            raise RuntimeError("xs_sdk returned no finger joint information for gripper.")
        self.left_finger_name = x.joint_names[0]
        self.left_finger_lower = float(x.joint_lower_limits[0])
        self.left_finger_upper = float(x.joint_upper_limits[0])

    def _wait_joint_states(self):
        end = time.monotonic() + 3
        while not self.current_rad and time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=.05)
        if not self.current_rad:
            raise RuntimeError(
                f"No data from {self.ns}/joint_states. Check matching "
                "ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY."
            )

    def _publish_gripper_effort(self, effort):
        msg = self.JointSingleCommand()
        msg.name = "gripper"
        msg.cmd = float(effort)
        self.gripper_pub.publish(msg)
        self.gripper_effort = float(effort)

    def _js_cb(self, msg):
        p = dict(zip(msg.name, msg.position))
        if all(j in p for j in JOINTS):
            self.current_rad = {j: float(p[j]) for j in JOINTS}
            self.current_deg = {
                j: math.degrees(self.current_rad[j]) for j in JOINTS
            }
            self.last_msg = time.monotonic()
            self.joint_state_changed.emit(self.current_deg.copy())

        if self.left_finger_name in p:
            self.left_finger_pos = float(p[self.left_finger_name])
            if self.last_gripper_action is None:
                midpoint = (self.left_finger_lower + self.left_finger_upper) / 2.0
                self.last_gripper_action = (
                    "OPEN" if self.left_finger_pos >= midpoint else "CLOSE"
                )
                self.gripper_state_changed.emit(self.last_gripper_action)

            # Equivalent safety behavior to the Interbotix gripper interface:
            # stop PWM when the mechanical finger limit is reached.
            if self.gripper_moving:
                if (
                    self.gripper_effort > 0
                    and self.left_finger_pos >= self.left_finger_upper
                ) or (
                    self.gripper_effort < 0
                    and self.left_finger_pos <= self.left_finger_lower
                ):
                    self._publish_gripper_effort(0.0)
                    self.gripper_moving = False

    def _connection_check(self):
        if self.last_msg and time.monotonic() - self.last_msg > 1.0:
            self.status_changed.emit("ROBOT DATA LOST")

    def get_joint_positions_deg(self):
        return self.current_deg.copy()

    def get_joint_limits_deg(self):
        return {
            j: (math.degrees(self.lo[j]), math.degrees(self.hi[j]))
            for j in JOINTS
        }

    def get_sleep_pose_deg(self):
        return {j: math.degrees(self.sleep[j]) for j in JOINTS}

    def set_gripper_pressure(self, percent):
        percent = max(0, min(100, int(percent)))
        self.gripper_pressure_percent = percent
        fraction = percent / 100.0
        self.gripper_pwm = self.gripper_pressure_lower + fraction * (
            self.gripper_pressure_upper - self.gripper_pressure_lower
        )

    def command_gripper(self, action):
        action = action.upper()
        if action not in ("OPEN", "CLOSE"):
            self.error_occurred.emit(f"Unknown gripper action: {action}")
            return False
        if self.left_finger_pos is None:
            self.error_occurred.emit("No gripper finger position available.")
            return False

        effort = self.gripper_pwm if action == "OPEN" else -self.gripper_pwm

        # Do not drive farther if already at the corresponding mechanical limit.
        if (
            effort > 0 and self.left_finger_pos >= self.left_finger_upper
        ) or (
            effort < 0 and self.left_finger_pos <= self.left_finger_lower
        ):
            self._publish_gripper_effort(0.0)
            self.gripper_moving = False
            self.last_gripper_action = action
            self.gripper_state_changed.emit(action)
            return True

        self._publish_gripper_effort(effort)
        self.gripper_moving = True
        self.last_gripper_action = action
        self.gripper_state_changed.emit(action)
        return True

    def stop_gripper(self):
        self._publish_gripper_effort(0.0)
        self.gripper_moving = False
        return True

    def get_gripper_program_state(self):
        if self.last_gripper_action in ("OPEN", "CLOSE"):
            return self.last_gripper_action
        return "KEEP"

    def get_ee_pose(self):
        if not self.current_rad:
            return None
        try:
            q = [self.current_rad[j] for j in self.order]
            T = self.mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, q)
            rpy = self.ang.rotation_matrix_to_euler_angles(T[:3, :3])
            return {
                "x": float(T[0, 3]),
                "y": float(T[1, 3]),
                "z": float(T[2, 3]),
                "pitch_deg": math.degrees(float(rpy[1])),
            }
        except Exception:
            return None

    def get_cartesian_target_pose(self):
        if self.cartesian_target_pose is None:
            self.reset_cartesian_target()
        return (
            self.cartesian_target_pose.copy()
            if self.cartesian_target_pose is not None
            else None
        )

    def reset_cartesian_target(self):
        pose = self.get_ee_pose()
        if pose is not None:
            self.cartesian_target_pose = pose.copy()
        return (
            self.cartesian_target_pose.copy()
            if self.cartesian_target_pose is not None
            else None
        )

    def _wrap_ik_solution(self, theta_list):
        theta = (self.np.array(theta_list, dtype=float) + math.pi) % (
            2.0 * math.pi
        ) - math.pi
        for i, joint in enumerate(self.order):
            if round(theta[i], 3) < round(self.lo[joint], 3):
                theta[i] += 2.0 * math.pi
            elif round(theta[i], 3) > round(self.hi[joint], 3):
                theta[i] -= 2.0 * math.pi
        return theta

    @staticmethod
    def _joint_distance_sq(candidate, current):
        """Squared wrapped angular distance; smaller means less joint flipping."""
        total = 0.0
        for a, b in zip(candidate, current):
            d = (float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi
            total += d * d
        return total

    def jog_cartesian(
        self,
        dx=0.0,
        dy=0.0,
        dz=0.0,
        dpitch_deg=0.0,
        move_time=0.7,
    ):
        """
        Small base-frame Cartesian jog for the 4-DOF PX100.

        v4.1 changes:
        - increments are applied to a maintained Cartesian TARGET, not to a
          potentially intermediate measured pose;
        - all valid IK candidates are considered;
        - the solution closest to the current joint configuration is selected.

        Therefore a completed X+ step followed by the same X- step should
        return to the original Cartesian target (within normal servo accuracy).
        """
        if not self.current_rad:
            self.error_occurred.emit("No current joint state available.")
            return False

        if self.motion_busy:
            self.error_occurred.emit(
                "Arm is still moving. Wait for the current move to finish "
                "before another Cartesian jog."
            )
            return False

        try:
            if self.cartesian_target_pose is None:
                self.reset_cartesian_target()
            if self.cartesian_target_pose is None:
                self.error_occurred.emit("Could not determine current end-effector pose.")
                return False

            target_pose = self.cartesian_target_pose.copy()
            target_pose["x"] += float(dx)
            target_pose["y"] += float(dy)
            target_pose["z"] += float(dz)
            target_pose["pitch_deg"] += float(dpitch_deg)

            pitch = math.radians(target_pose["pitch_deg"])
            pitch_limit = math.radians(85.0)
            if not -pitch_limit <= pitch <= pitch_limit:
                self.error_occurred.emit(
                    "Cartesian pitch limited to ±85° in this GUI."
                )
                return False

            x = target_pose["x"]
            y = target_pose["y"]
            z = target_pose["z"]

            # Same convention used by Interbotix set_ee_pose_components()
            # for arms with fewer than 6 DOF.
            yaw = math.atan2(y, x)

            T_target = self.np.identity(4)
            T_target[:3, :3] = self.ang.euler_angles_to_rotation_matrix(
                [0.0, pitch, yaw]
            )
            T_target[:3, 3] = [x, y, z]

            q_current = [self.current_rad[j] for j in self.order]

            guesses = [
                q_current,
                [0.0] * len(self.order),
                [math.radians(-120.0)] + [0.0] * (len(self.order) - 1),
                [math.radians(120.0)] + [0.0] * (len(self.order) - 1),
            ]

            valid_candidates = []
            last_reason = "No IK solution."

            for guess in guesses:
                theta, success = self.mr.IKinSpace(
                    Slist=self.robot_des.Slist,
                    M=self.robot_des.M,
                    T=T_target,
                    thetalist0=guess,
                    eomg=0.001,
                    ev=0.001,
                )
                if not success:
                    continue

                theta = self._wrap_ik_solution(theta)
                target_rad = {
                    joint: float(theta[i])
                    for i, joint in enumerate(self.order)
                }

                reason = self._validate(target_rad, float(move_time))
                if reason is None:
                    distance = self._joint_distance_sq(theta, q_current)
                    valid_candidates.append((distance, target_rad))
                else:
                    last_reason = reason

            if not valid_candidates:
                self.error_occurred.emit(
                    "No valid Cartesian pose found. Try a smaller jog or move "
                    f"away from a workspace/joint limit. ({last_reason})"
                )
                return False

            # Continuity: choose the valid IK branch requiring the smallest
            # joint change from the measured current configuration.
            valid_candidates.sort(key=lambda item: item[0])
            target_rad = valid_candidates[0][1]
            target_deg = {
                joint: math.degrees(target_rad[joint])
                for joint in JOINTS
            }

            # Save this only if the command is actually accepted.
            self.pending_cartesian_target = target_pose.copy()
            ok = self.move_joints_deg(target_deg, float(move_time))
            if not ok:
                self.pending_cartesian_target = None
                return False
            return True

        except Exception as exc:
            self.pending_cartesian_target = None
            self.error_occurred.emit(f"Cartesian IK failed: {exc}")
            return False

    def _set_reg(self, reg, value):
        if not self.reg_cli.wait_for_service(timeout_sec=1.0):
            self.error_occurred.emit("set_motor_registers service unavailable")
            return False
        req = self.RegisterValues.Request()
        req.cmd_type = "group"
        req.name = "arm"
        req.reg = reg
        req.value = int(value)
        fut = self.reg_cli.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=1.5)
        if not fut.done() or fut.result() is None:
            self.error_occurred.emit(f"Failed to set {reg}")
            return False
        return True

    def _validate(self, target_rad, move_time):
        if not self.current_rad:
            return "No current joint state available."
        if move_time < 0.2:
            return "Move time must be at least 0.2 s."
        for j in self.order:
            t = target_rad[j]
            if not self.lo[j] <= t <= self.hi[j]:
                return (
                    f"{j}: target {math.degrees(t):.1f}° outside robot limit "
                    f"{math.degrees(self.lo[j]):.1f}...{math.degrees(self.hi[j]):.1f}°."
                )
            need = abs(t - self.current_rad[j]) / move_time
            if self.vel[j] > 0 and need > self.vel[j]:
                return f"{j}: requested move is too fast. Increase Move time."
        return None

    def move_joints_deg(self, targets, move_time=2.0):
        target_rad = {j: math.radians(float(targets[j])) for j in JOINTS}
        reason = self._validate(target_rad, float(move_time))
        if reason:
            self.error_occurred.emit(reason)
            return False

        accel = min(.3, float(move_time) / 2.0)
        if not self._set_reg("Profile_Velocity", int(float(move_time) * 1000)):
            return False
        if not self._set_reg("Profile_Acceleration", int(accel * 1000)):
            return False

        msg = self.JointGroupCommand()
        msg.name = "arm"
        msg.cmd = [target_rad[j] for j in self.order]
        self.pub.publish(msg)

        self.motion_busy = True
        self.status_changed.emit("REAL ROBOT / MOVING")
        self.motion_timer.start(int((float(move_time) + .12) * 1000))
        return True

    def _motion_done(self):
        self.motion_busy = False

        if self.pending_cartesian_target is not None:
            # Preserve the exact commanded target. This guarantees that an
            # equal opposite jog returns to the previous Cartesian target.
            self.cartesian_target_pose = self.pending_cartesian_target.copy()
            self.pending_cartesian_target = None
        else:
            # Any normal joint/Home/Sleep/program move invalidates the old
            # Cartesian target. Resynchronize from measured FK.
            self.reset_cartesian_target()

        self.status_changed.emit("REAL ROBOT / READY")
        self.motion_finished.emit()

    def hold_current(self):
        if self.current_deg:
            self.motion_timer.stop()
            self.move_joints_deg(self.current_deg.copy(), .3)
            self.status_changed.emit("HOLD CURRENT (NOT E-STOP)")

    def close(self):
        # Do not intentionally release an object on GUI exit. Stop only an OPEN command;
        # a CLOSE command may be holding an object, so leave the driver state untouched.
        for t in (
            getattr(self, "spin_timer", None),
            getattr(self, "motion_timer", None),
            getattr(self, "status_timer", None),
        ):
            if t:
                t.stop()
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


class MainWindow(QMainWindow):
    def __init__(self, backend, mode):
        super().__init__()
        self.backend = backend
        self.mode = mode
        self.controls = {}
        self.actual = {}
        self.program: List[PoseStep] = []
        self.running = False
        self.idx = 0
        self.targets_initialized = False

        self.wait_timer = QTimer()
        self.wait_timer.setSingleShot(True)
        self.wait_timer.timeout.connect(self._run_next)

        self.gripper_settle_timer = QTimer()
        self.gripper_settle_timer.setSingleShot(True)
        self.gripper_settle_timer.timeout.connect(self._after_gripper_action)

        backend.joint_state_changed.connect(self._joint_update)
        backend.gripper_state_changed.connect(self._gripper_update)
        backend.status_changed.connect(self._status)
        backend.motion_finished.connect(self._motion_finished)
        backend.error_occurred.connect(
            lambda s: QMessageBox.warning(self, "Command rejected", s)
        )

        self.limits = backend.get_joint_limits_deg()
        self.active_program_row = None
        self.setWindowTitle(f"PincherX 100 Teach Pendant v4.1 - {mode}")
        self.resize(1240, 860)
        self._ui()
        self._joint_update(backend.get_joint_positions_deg())
        self._gripper_update(backend.get_gripper_program_state())
        self._status(f"{mode} / READY")

    def _ui(self):
        c = QWidget()
        self.setCentralWidget(c)
        root = QVBoxLayout(c)
        top = QHBoxLayout()
        root.addLayout(top)

        box = QGroupBox("Robot")
        v = QVBoxLayout(box)
        self.status = QLabel()
        self.status.setStyleSheet("font-weight:bold;font-size:16px")
        v.addWidget(self.status)

        for text, fn in [
            ("HOME", lambda: self._command(self.backend.get_home_pose_deg(), 2.0)),
            ("SLEEP", lambda: self._command(self.backend.get_sleep_pose_deg(), 2.5)),
            ("COPY ACTUAL → TARGET", self._copy_actual),
            ("HOLD CURRENT", self._hold),
        ]:
            b = QPushButton(text)
            b.clicked.connect(fn)
            v.addWidget(b)
        note = QLabel("HOLD / STOP PROGRAM are not emergency-stop functions.")
        note.setWordWrap(True)
        v.addWidget(note)
        v.addStretch()
        top.addWidget(box, 1)

        jbox = QGroupBox("Joint control")
        g = QGridLayout(jbox)
        for col, text in enumerate(["Joint", "Actual", "Target", "Target slider", "Jog"]):
            g.addWidget(QLabel(text), 0, col)
        for r, j in enumerate(JOINTS, 1):
            lo, hi = self.limits[j]
            g.addWidget(QLabel(j.replace("_", " ").title()), r, 0)
            a = QLabel("---°")
            a.setStyleSheet("font-weight:bold")
            g.addWidget(a, r, 1)
            self.actual[j] = a

            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(1)
            s.setSuffix("°")
            sl = QSlider(Qt.Horizontal)
            sl.setRange(int(math.floor(lo * 10)), int(math.ceil(hi * 10)))
            sl.valueChanged.connect(lambda x, sp=s: sp.setValue(x / 10))
            s.valueChanged.connect(lambda x, q=sl: q.setValue(int(round(x * 10))))

            h = QHBoxLayout()
            bm = QPushButton("-1°")
            bp = QPushButton("+1°")
            bm.clicked.connect(lambda _, jj=j: self._jog(jj, -1))
            bp.clicked.connect(lambda _, jj=j: self._jog(jj, 1))
            h.addWidget(bm)
            h.addWidget(bp)

            g.addWidget(s, r, 2)
            g.addWidget(sl, r, 3)
            g.addLayout(h, r, 4)
            self.controls[j] = (s, sl)

        h = QHBoxLayout()
        h.addWidget(QLabel("Manual move time:"))
        self.manual = QDoubleSpinBox()
        self.manual.setRange(.2, 10)
        self.manual.setValue(1.5)
        self.manual.setSuffix(" s")
        h.addWidget(self.manual)
        b = QPushButton("MOVE TO TARGET ANGLES")
        b.clicked.connect(lambda: self._command(self._targets(), self.manual.value()))
        h.addWidget(b)
        g.addLayout(h, 5, 1, 1, 4)
        top.addWidget(jbox, 3)

        gripper_box = QGroupBox("Gripper")
        gv = QVBoxLayout(gripper_box)
        self.gripper_actual = QLabel("State: ---")
        self.gripper_actual.setStyleSheet("font-weight:bold;font-size:14px")
        gv.addWidget(self.gripper_actual)

        gh = QHBoxLayout()
        open_btn = QPushButton("OPEN")
        close_btn = QPushButton("CLOSE")
        open_btn.clicked.connect(lambda: self.backend.command_gripper("OPEN"))
        close_btn.clicked.connect(lambda: self.backend.command_gripper("CLOSE"))
        gh.addWidget(open_btn)
        gh.addWidget(close_btn)
        gv.addLayout(gh)

        stop_gripper_btn = QPushButton("STOP GRIPPER")
        stop_gripper_btn.clicked.connect(self.backend.stop_gripper)
        gv.addWidget(stop_gripper_btn)

        gv.addWidget(QLabel("Grip pressure"))
        self.pressure_slider = QSlider(Qt.Horizontal)
        self.pressure_slider.setRange(0, 100)
        self.pressure_slider.setValue(50)
        self.pressure_slider.valueChanged.connect(self._pressure_changed)
        gv.addWidget(self.pressure_slider)
        self.pressure_label = QLabel("50%  (~250 PWM)")
        gv.addWidget(self.pressure_label)
        gn = QLabel("OPEN/CLOSE uses PWM effort. CLOSE can keep holding an object until OPEN or STOP GRIPPER is commanded.")
        gn.setWordWrap(True)
        gv.addWidget(gn)
        gv.addStretch()
        top.addWidget(gripper_box, 1)

        # Cartesian / end-effector jog. These commands solve IK and finally
        # send normal joint targets, so saved programs remain joint-based.
        cbox = QGroupBox("Cartesian jog (base frame / IK)")
        cg = QGridLayout(cbox)

        self.ee_x = QLabel("Actual X: --- mm")
        self.ee_y = QLabel("Actual Y: --- mm")
        self.ee_z = QLabel("Actual Z: --- mm")
        self.ee_pitch = QLabel("Actual Pitch: ---°")
        self.ee_target = QLabel("Target: ---")
        for label in (self.ee_x, self.ee_y, self.ee_z, self.ee_pitch):
            label.setStyleSheet("font-weight:bold")
        self.ee_target.setStyleSheet("font-weight:bold")
        cg.addWidget(self.ee_x, 0, 0)
        cg.addWidget(self.ee_y, 0, 1)
        cg.addWidget(self.ee_z, 0, 2)
        cg.addWidget(self.ee_pitch, 0, 3)
        cg.addWidget(self.ee_target, 1, 0, 1, 3)

        reset_cart = QPushButton("RESET TARGET = ACTUAL")
        reset_cart.clicked.connect(self._reset_cartesian_target)
        cg.addWidget(reset_cart, 1, 3)

        cg.addWidget(QLabel("XYZ jog step:"), 2, 0)
        self.cart_step = QDoubleSpinBox()
        self.cart_step.setRange(1.0, 25.0)
        self.cart_step.setValue(5.0)
        self.cart_step.setSingleStep(1.0)
        self.cart_step.setSuffix(" mm")
        cg.addWidget(self.cart_step, 2, 1)

        cg.addWidget(QLabel("Pitch jog step:"), 2, 2)
        self.pitch_step = QDoubleSpinBox()
        self.pitch_step.setRange(0.5, 10.0)
        self.pitch_step.setValue(2.0)
        self.pitch_step.setSingleStep(0.5)
        self.pitch_step.setSuffix("°")
        cg.addWidget(self.pitch_step, 2, 3)

        cart_buttons = [
            ("X −", -1, 0, 0, 0, 3, 0),
            ("X +",  1, 0, 0, 0, 3, 1),
            ("Y −",  0,-1, 0, 0, 3, 2),
            ("Y +",  0, 1, 0, 0, 3, 3),
            ("Z −",  0, 0,-1, 0, 4, 0),
            ("Z +",  0, 0, 1, 0, 4, 1),
            ("Pitch −", 0, 0, 0,-1, 4, 2),
            ("Pitch +", 0, 0, 0, 1, 4, 3),
        ]
        for label, sx, sy, sz, sp, row, col in cart_buttons:
            btn = QPushButton(label)
            btn.clicked.connect(
                lambda _, ax=sx, ay=sy, az=sz, ap=sp:
                    self._cartesian_jog(ax, ay, az, ap)
            )
            cg.addWidget(btn, row, col)

        cart_note = QLabel(
            "PX100 is 4-DOF. X/Y/Z are base-frame target changes solved with IK; "
            "Pitch changes tool tilt. A jog is not guaranteed to follow a perfectly "
            "straight Cartesian path. Start with 5 mm / 2° steps."
        )
        cart_note.setWordWrap(True)
        cg.addWidget(cart_note, 5, 0, 1, 4)
        root.addWidget(cbox)

        pbox = QGroupBox("Program")
        pv = QVBoxLayout(pbox)

        self.execution_label = QLabel("Program idle")
        self.execution_label.setStyleSheet("font-weight:bold;font-size:14px")
        pv.addWidget(self.execution_label)

        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels([
            "Name", "Waist", "Shoulder", "Elbow", "Wrist", "Gripper", "Move [s]", "Wait [s]"
        ])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        pv.addWidget(self.table)

        help_label = QLabel("Gripper column: OPEN, CLOSE or KEEP. The gripper action is executed after the arm reaches that pose.")
        help_label.setWordWrap(True)
        pv.addWidget(help_label)

        h1 = QHBoxLayout()
        for text, fn in [
            ("SAVE ACTUAL POSITION", self._save_pose),
            ("DELETE", self._delete),
            ("↑", lambda: self._move_row(-1)),
            ("↓", lambda: self._move_row(1)),
        ]:
            b = QPushButton(text)
            b.clicked.connect(fn)
            h1.addWidget(b)
        pv.addLayout(h1)

        h2 = QHBoxLayout()
        for text, fn in [
            ("▶ RUN PROGRAM", self._run),
            ("■ STOP PROGRAM", self._stop_program),
            ("SAVE JSON", self._save_json),
            ("LOAD JSON", self._load_json),
        ]:
            b = QPushButton(text)
            b.clicked.connect(fn)
            h2.addWidget(b)
        pv.addLayout(h2)
        root.addWidget(pbox, 2)

    def _pressure_changed(self, value):
        self.backend.set_gripper_pressure(value)
        pwm = 150 + (value / 100.0) * 200
        self.pressure_label.setText(f"{value}%  (~{pwm:.0f} PWM)")

    def _targets(self):
        return {j: self.controls[j][0].value() for j in JOINTS}

    def _copy_actual(self):
        x = self.backend.get_joint_positions_deg()
        for j in JOINTS:
            if j in x:
                self.controls[j][0].setValue(x[j])

    def _command(self, p, t):
        if self.backend.move_joints_deg(p, t):
            for j in JOINTS:
                self.controls[j][0].setValue(p[j])

    def _jog(self, j, d):
        x = self.backend.get_joint_positions_deg()
        if not x:
            return
        lo, hi = self.limits[j]
        x[j] = max(lo, min(hi, x[j] + d))
        self._command(x, .5)

    def _joint_update(self, x):
        if not x:
            return
        for j in JOINTS:
            self.actual[j].setText(f"{x[j]:.1f}°")
        if not self.targets_initialized:
            self.targets_initialized = True
            self._copy_actual()
        self._update_ee_display()

    def _update_ee_display(self):
        pose = self.backend.get_ee_pose()
        if not pose:
            self.ee_x.setText("X: --- mm")
            self.ee_y.setText("Y: --- mm")
            self.ee_z.setText("Z: --- mm")
            self.ee_pitch.setText("Pitch: ---°")
            return
        self.ee_x.setText(f"Actual X: {pose['x'] * 1000.0:.1f} mm")
        self.ee_y.setText(f"Actual Y: {pose['y'] * 1000.0:.1f} mm")
        self.ee_z.setText(f"Actual Z: {pose['z'] * 1000.0:.1f} mm")
        self.ee_pitch.setText(f"Actual Pitch: {pose['pitch_deg']:.1f}°")

        target = self.backend.get_cartesian_target_pose()
        if target:
            self.ee_target.setText(
                "Target: "
                f"X {target['x'] * 1000.0:.1f} | "
                f"Y {target['y'] * 1000.0:.1f} | "
                f"Z {target['z'] * 1000.0:.1f} mm | "
                f"Pitch {target['pitch_deg']:.1f}°"
            )
        else:
            self.ee_target.setText("Target: ---")

    def _reset_cartesian_target(self):
        self.backend.reset_cartesian_target()
        self._update_ee_display()

    def _cartesian_jog(self, sx, sy, sz, sp):
        step_m = self.cart_step.value() / 1000.0
        pitch_deg = self.pitch_step.value()
        ok = self.backend.jog_cartesian(
            dx=sx * step_m,
            dy=sy * step_m,
            dz=sz * step_m,
            dpitch_deg=sp * pitch_deg,
            move_time=max(0.5, self.manual.value()),
        )
        if ok:
            # During motion the maintained target is pending; Actual will catch up.
            pending = getattr(self.backend, "pending_cartesian_target", None)
            if pending:
                self.ee_target.setText(
                    "Target: "
                    f"X {pending['x'] * 1000.0:.1f} | "
                    f"Y {pending['y'] * 1000.0:.1f} | "
                    f"Z {pending['z'] * 1000.0:.1f} mm | "
                    f"Pitch {pending['pitch_deg']:.1f}°"
                )

    def _gripper_update(self, state):
        self.gripper_actual.setText(f"State: {state}")

    def _status(self, s):
        self.status.setText(s)

    def _hold(self):
        self.running = False
        self.wait_timer.stop()
        self.gripper_settle_timer.stop()
        self._clear_active_program_row()
        self.execution_label.setText("Program interrupted by HOLD CURRENT")
        self.backend.hold_current()

    def _save_pose(self):
        x = self.backend.get_joint_positions_deg()
        if not x:
            return
        self.program.append(PoseStep(
            name=f"Pose {len(self.program) + 1}",
            waist=x["waist"],
            shoulder=x["shoulder"],
            elbow=x["elbow"],
            wrist_angle=x["wrist_angle"],
            gripper=self.backend.get_gripper_program_state(),
        ))
        self._refresh()
        self.table.selectRow(len(self.program) - 1)

    def _refresh(self):
        self.table.setRowCount(len(self.program))
        for r, p in enumerate(self.program):
            vals = [
                p.name,
                f"{p.waist:.1f}",
                f"{p.shoulder:.1f}",
                f"{p.elbow:.1f}",
                f"{p.wrist_angle:.1f}",
                p.gripper,
                f"{p.move_time:.2f}",
                f"{p.wait_time:.2f}",
            ]
            for c, val in enumerate(vals):
                self.table.setItem(r, c, QTableWidgetItem(val))

    def _sync(self):
        for r in range(self.table.rowCount()):
            try:
                gripper = self.table.item(r, 5).text().strip().upper()
                if gripper not in VALID_GRIPPER_ACTIONS:
                    raise ValueError(
                        f"Row {r + 1}: Gripper must be OPEN, CLOSE or KEEP."
                    )
                p = PoseStep(
                    name=self.table.item(r, 0).text(),
                    waist=float(self.table.item(r, 1).text()),
                    shoulder=float(self.table.item(r, 2).text()),
                    elbow=float(self.table.item(r, 3).text()),
                    wrist_angle=float(self.table.item(r, 4).text()),
                    gripper=gripper,
                    move_time=float(self.table.item(r, 6).text()),
                    wait_time=float(self.table.item(r, 7).text()),
                )
            except AttributeError:
                raise ValueError(f"Missing value on row {r + 1}")
            except ValueError as e:
                if str(e).startswith("Row "):
                    raise
                raise ValueError(f"Invalid numeric value on row {r + 1}")

            if p.move_time < .2 or p.wait_time < 0:
                raise ValueError(f"Invalid timing on row {r + 1}")
            self.program[r] = p

    def _delete(self):
        r = self.table.currentRow()
        if r >= 0:
            self.program.pop(r)
            self._refresh()

    def _move_row(self, d):
        r = self.table.currentRow()
        n = r + d
        if r < 0 or n < 0 or n >= len(self.program):
            return
        self.program[r], self.program[n] = self.program[n], self.program[r]
        self._refresh()
        self.table.selectRow(n)

    def _clear_active_program_row(self):
        if self.active_program_row is None:
            return
        row = self.active_program_row
        if 0 <= row < self.table.rowCount():
            for c in range(self.table.columnCount()):
                item = self.table.item(row, c)
                if item is None:
                    continue
                item.setBackground(QBrush())
                item.setForeground(QBrush())
                font = item.font()
                font.setBold(False)
                item.setFont(font)
        self.active_program_row = None

    def _highlight_program_row(self, row, phase):
        self._clear_active_program_row()
        if row < 0 or row >= self.table.rowCount():
            return

        self.active_program_row = row
        bg = self.table.palette().brush(QPalette.Highlight)
        fg = self.table.palette().brush(QPalette.HighlightedText)

        for c in range(self.table.columnCount()):
            item = self.table.item(row, c)
            if item is None:
                continue
            item.setBackground(bg)
            item.setForeground(fg)
            font = item.font()
            font.setBold(True)
            item.setFont(font)

        self.table.scrollToItem(self.table.item(row, 0))
        pose = self.program[row]
        self.execution_label.setText(
            f"▶ Step {row + 1}/{len(self.program)} — {pose.name} — {phase}"
        )

    def _run(self):
        if not self.program:
            return
        try:
            self._sync()
        except Exception as e:
            QMessageBox.warning(self, "Program error", str(e))
            return
        self.running = True
        self.idx = 0
        self._run_next()

    def _run_next(self):
        if not self.running:
            return
        if self.idx >= len(self.program):
            self.running = False
            self._clear_active_program_row()
            self.execution_label.setText("✓ Program finished")
            self._status(f"{self.mode} / PROGRAM FINISHED")
            return

        p = self.program[self.idx]
        self._highlight_program_row(self.idx, "MOVING ARM")
        self._status(f"RUNNING {self.idx + 1}/{len(self.program)}: {p.name}")
        if not self.backend.move_joints_deg(p.joints(), p.move_time):
            self.running = False
            self._clear_active_program_row()
            self.execution_label.setText("Program stopped: arm command rejected")

    def _motion_finished(self):
        if not self.running:
            return
        p = self.program[self.idx]
        if p.gripper in ("OPEN", "CLOSE"):
            self._highlight_program_row(self.idx, f"GRIPPER {p.gripper}")
            if not self.backend.command_gripper(p.gripper):
                self.running = False
                self._clear_active_program_row()
                self.execution_label.setText("Program stopped: gripper command rejected")
                return
            self.gripper_settle_timer.start(GRIPPER_ACTION_SETTLE_MS)
        else:
            self._after_gripper_action()

    def _after_gripper_action(self):
        if not self.running:
            return
        p = self.program[self.idx]
        if p.wait_time > 0:
            self._highlight_program_row(self.idx, f"WAIT {p.wait_time:.2f} s")
        self.idx += 1
        self.wait_timer.start(int(p.wait_time * 1000))

    def _stop_program(self):
        self.running = False
        self.wait_timer.stop()
        self.gripper_settle_timer.stop()
        self._clear_active_program_row()
        self.execution_label.setText("■ Program stopped")
        self._status(
            f"{self.mode} / PROGRAM STOPPED (CURRENT MOVE / GRIP MAY CONTINUE)"
        )

    def _save_json(self):
        try:
            self._sync()
        except Exception as e:
            QMessageBox.warning(self, "Program error", str(e))
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save program", "", "JSON (*.json)"
        )
        if not path:
            return
        if not path.endswith(".json"):
            path += ".json"
        with open(path, "w") as f:
            json.dump({
                "format": "pincher_teach_pendant_v4_1",
                "robot": "px100",
                "angle_unit": "degrees",
                "poses": [asdict(p) for p in self.program],
            }, f, indent=2)

    def _load_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load program", "", "JSON (*.json)"
        )
        if not path:
            return
        try:
            with open(path) as f:
                d = json.load(f)
            self.program = [PoseStep(**x) for x in d["poses"]]
            self._refresh()
        except Exception as e:
            QMessageBox.warning(self, "Load failed", str(e))

    def closeEvent(self, e):
        self.running = False
        self.wait_timer.stop()
        self.gripper_settle_timer.stop()
        self.backend.close()
        e.accept()


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--real", action="store_true")
    g.add_argument("--demo", action="store_true")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    try:
        backend = Ros2PincherBackend() if args.real else DemoBackend()
        mode = "REAL ROBOT" if args.real else "DEMO MODE"
        w = MainWindow(backend, mode)
        w.show()
        sys.exit(app.exec_())
    except Exception as e:
        QMessageBox.critical(None, "Startup failed", str(e))
        sys.exit(1)


if __name__ == "__main__":
    main()
