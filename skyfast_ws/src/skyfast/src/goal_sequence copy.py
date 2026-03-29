#!/usr/bin/env python3

import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw_deg_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def publish_goal(pub, x, y, z, yaw_deg=0.0):
    msg = PoseStamped()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    qx, qy, qz, qw = yaw_deg_to_quaternion(yaw_deg)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    rospy.logdebug(
        "Publishing goal: x=%.2f y=%.2f z=%.2f yaw=%.1fdeg q=(%.3f, %.3f, %.3f, %.3f)",
        x,
        y,
        z,
        yaw_deg,
        qx,
        qy,
        qz,
        qw,
    )
    pub.publish(msg)


class GoalMover:
    def __init__(self):
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        self.odom_tracker = OdomTracker()
        rospy.Subscriber("/Odometry", Odometry, self.odom_tracker.callback, queue_size=1)
        self.reach_tolerance = 2
        self.reach_timeout = 60.0

    def wait_ready(self):
        rospy.sleep(2.0)
        self.odom_tracker.wait_for_odom()

    def move(self, x, y, z, yaw_deg=0.0):
        rospy.loginfo("Move request: x=%.2f y=%.2f z=%.2f yaw=%.1fdeg", x, y, z, yaw_deg)
        publish_goal(self.pub, x, y, z, yaw_deg)
        reached = self.odom_tracker.wait_until_reached((x, y, z), self.reach_tolerance, self.reach_timeout)
        if not reached:
            rospy.logwarn("Timeout waiting for goal (%s, %s, %s, yaw=%s deg)", x, y, z, yaw_deg)
        else:
            rospy.loginfo("Reached goal: x=%.2f y=%.2f z=%.2f yaw=%.1fdeg", x, y, z, yaw_deg)


def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class OdomTracker:
    def __init__(self):
        self.position = None
        self._last_progress_log = 0.0

    def callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def wait_for_odom(self):
        rospy.loginfo("Waiting for odometry on /visual_slam/odom...")
        while not rospy.is_shutdown() and self.position is None:
            rospy.sleep(0.05)
        if self.position is not None:
            rospy.loginfo(
                "Odometry ready at x=%.2f y=%.2f z=%.2f",
                self.position[0],
                self.position[1],
                self.position[2],
            )

    def wait_until_reached(self, target, tolerance, timeout):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.position is not None:
                current_dist = distance(self.position, target)
                now_sec = rospy.Time.now().to_sec()
                if now_sec - self._last_progress_log >= 1.0:
                    rospy.logdebug(
                        "Goal progress: target=(%.2f, %.2f, %.2f) current=(%.2f, %.2f, %.2f) dist=%.2f tol=%.2f",
                        target[0],
                        target[1],
                        target[2],
                        self.position[0],
                        self.position[1],
                        self.position[2],
                        current_dist,
                        tolerance,
                    )
                    self._last_progress_log = now_sec
                if current_dist <= tolerance:
                    return True
            if timeout > 0 and (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
        return False


def main():
    rospy.init_node("goal_sequence")
    rospy.loginfo("goal_sequence node started")
    rospy.loginfo("Enable debug output with: rosrun skyfast goal_sequence.py __log_level:=debug")

    loop = True
    mover = GoalMover()
    mover.wait_ready()

    while not rospy.is_shutdown():
        mover.move(6.0, -5.0, 1.1, 180)

        mover.move(-6.0, -6.0, 1.1, 180.0)
        mover.move(-8.0, -5.0, 1.1, 135.0)

        mover.move(-5.5, 4.0, 1.1, 0.0)
        mover.move(-3.5, 4.0, 1.1, 0.0)

        mover.move(13.5, 4.0, 1.1, 0.0)
        mover.move(15.5, 3.0, 1.1, -45.0)

        mover.move(13.0, -5.0, 1.1, 180.0)
        mover.move(11.0, -5.0, 1.1, 180.0)

        mover.move(8.5, -5.0, 1.1, 180.0)

        if not loop:
            break


if __name__ == "__main__":
    main()
