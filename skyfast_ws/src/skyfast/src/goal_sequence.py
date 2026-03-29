#!/usr/bin/env python3

import math
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget


def yaw_deg_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def publish_goal(pub, x, y, z, yaw_deg=0.0):
    msg = PoseStamped()
    msg.header.frame_id = "camera_init"
    msg.header.stamp = rospy.Time.now()

    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    qx, qy, qz, qw = yaw_deg_to_quaternion(yaw_deg)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    pub.publish(msg)


def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


class OdomTracker:
    def __init__(self):
        self.position = None
        self._last_log_time = 0.0

    def callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def wait_for_odom(self):
        rospy.loginfo("Waiting for odometry...")
        while not rospy.is_shutdown() and self.position is None:
            rospy.sleep(0.05)

        rospy.loginfo(
            "Odometry ready: x=%.2f y=%.2f z=%.2f",
            self.position[0],
            self.position[1],
            self.position[2],
        )

    def wait_until_reached(self, target, tolerance, timeout):
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.position is not None:
                dist = distance(self.position, target)

                now = rospy.Time.now().to_sec()
                if now - self._last_log_time > 1.0:
                    rospy.logdebug("Distance to goal: %.2f", dist)
                    self._last_log_time = now

                if dist <= tolerance:
                    return True

            if timeout > 0 and (rospy.Time.now() - start).to_sec() > timeout:
                return False

            rate.sleep()

        return False


class GoalMover:
    def __init__(self):
        # --- GOAL PUB ---
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )

        # --- MAVROS SETPOINT PUB ---
        self.sp_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )

        # --- STATE ---
        self.latest_cmd = None
        self.lock = threading.Lock()
        self.current_yaw = 0.0

        # --- SUBSCRIBERS ---
        self.odom_tracker = OdomTracker()
        rospy.Subscriber("/Odometry", Odometry, self.odom_tracker.callback, queue_size=1)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.pos_cmd_cb, queue_size=1)

        # --- PARAMS ---
        self.reach_tolerance = 0.2
        self.reach_timeout = 60.0

    def pos_cmd_cb(self, msg):
        with self.lock:
            self.latest_cmd = msg

    def publish_setpoint_loop(self):
        rate = rospy.Rate(5)  # 🔥 CHANGE TO 20Hz if using PX4 OFFBOARD

        rospy.loginfo("Setpoint publishing loop started")

        while not rospy.is_shutdown():
            with self.lock:
                cmd = self.latest_cmd

            if cmd is not None:
                sp = PositionTarget()
                sp.header.stamp = rospy.Time.now()

                sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                sp.type_mask = (
                    PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY |
                    PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW_RATE
                )

                sp.position.x = cmd.position.x
                sp.position.y = cmd.position.y
                sp.position.z = cmd.position.z

                sp.velocity.x = cmd.velocity.x
                sp.velocity.y = cmd.velocity.y
                sp.velocity.z = cmd.velocity.z

                sp.yaw = math.radians(self.current_yaw)

                self.sp_pub.publish(sp)

            rate.sleep()

    def wait_ready(self):
        rospy.sleep(2.0)
        self.odom_tracker.wait_for_odom()

    def move(self, x, y, z, yaw_deg=0.0):
        self.current_yaw = yaw_deg

        rospy.loginfo(
            "Move: x=%.2f y=%.2f z=%.2f yaw=%.1f",
            x, y, z, yaw_deg
        )

        publish_goal(self.goal_pub, x, y, z, yaw_deg)

        reached = self.odom_tracker.wait_until_reached(
            (x, y, z),
            self.reach_tolerance,
            self.reach_timeout,
        )

        if reached:
            rospy.loginfo("Reached goal")
        else:
            rospy.logwarn("Timeout reaching goal")


def main():
    rospy.init_node("goal_sequence")

    mover = GoalMover()
    mover.wait_ready()

    # ✅ Start 10Hz publishing thread
    threading.Thread(
        target=mover.publish_setpoint_loop,
        daemon=True
    ).start()

    while not rospy.is_shutdown():
        mover.move(0.0, 0.0, 1.0, 0)
        mover.move(1.0, 0.0, 1.0, 0)
        mover.move(1.0, 1.0, 1.0, 0)
        mover.move(0.0, 1.0, 1.0, 0)
        mover.move(0.0, 0.0, 1.0, 0)


if __name__ == "__main__":
    main()