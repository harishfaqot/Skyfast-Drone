#!/usr/bin/env python3
import rospy
import math
import tf
import time
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandLong, SetMode, CommandTOL, CommandHome
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D, Twist


pi_2 = 3.141592654 / 2.0
deg2rad = 3.141592654/180

class PIDController:
    def __init__(self, Kp, Ki, Kd, windup_max=None, windup_min=None, decay_factor=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.windup_max = windup_max
        self.windup_min = windup_min
        self.decay_factor = decay_factor

    def update(self, process_variable):
        error = process_variable
        self.integral += error
        
        # Decreasing anti-windup
        if self.windup_max is not None:
            max_windup = self.windup_max / (1 + abs(error) * self.decay_factor)
            min_windup = -max_windup
            self.integral = max(min(self.integral, max_windup), min_windup)
        
        # print(self.integral)
        
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        output = max(-1, min(output, 1))
        return output

class MavController:
    def __init__(self):
        rospy.init_node("mission_fast_indoor_node")

        # --- STATE VARIABLES (Wajib ada agar tidak nyangkut saat menunggu) ---
        self.current_pose = None
        self.current_odom = None
        self.current_yaw = 0.0
        
        # --- KONFIGURASI TOPIC ---
        self.pose_topic = '/mavros/local_position/pose'
        self.odom_topic = '/mavros/local_position/odom'
        self.pos_cmd_topic = '/planning/pos_cmd'

        rospy.loginfo("Indoor Mission Controller Started!")

        # --- SUBSCRIBERS (Lengkapi keduanya) ---
        # 1. Subscribe ke Pose (Sering digunakan Ego Planner untuk koordinat target)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        # 2. Subscribe ke Odom (Untuk feedback kecepatan/posisi presisi)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        # 3. Bridge Planner -> Drone
        rospy.Subscriber(self.pos_cmd_topic, PositionCommand, self.pos_cmd_cb)
        
        self.target_sub = rospy.Subscriber('/pos_target', Pose2D, self.pos_target)

        # --- PUBLISHERS ---
        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.waypoint_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        # --- SERVICES ---
        rospy.loginfo("Waiting for MAVROS services...")
        self.setup_services()

        # --- WAIT FOR DATA (Logika agar tidak jalan sebelum posisi fix) ---
        self.wait_for_vins_data()

    def setup_services(self):
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5)
            self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            self.mavros_cmd_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        except rospy.ROSException:
            rospy.logerr("MAVROS Services not found!")

    def wait_for_vins_data(self):
        rospy.loginfo("Waiting for VINS position from MAVROS...")
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.sleep(0.2)
        rospy.loginfo("VINS Position received! Ready to go.")

    def pose_callback(self, msg):
        self.current_pose = msg
        # Tambahkan logika konversi Yaw jika perlu di sini

    def odom_callback(self, msg):
        self.current_odom = msg

    def pos_cmd_cb(self, msg):
        if not getattr(self, 'do_indoor', True):
            return

        sp = PositionTarget()
        sp.header = msg.header
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
                     | PositionTarget.IGNORE_YAW_RATE

        sp.position.x = msg.position.x
        sp.position.y = msg.position.y
        sp.position.z = msg.position.z
        sp.velocity.x = msg.velocity.x
        sp.velocity.y = msg.velocity.y
        sp.velocity.z = msg.velocity.z
        sp.yaw = self.current_yaw

        self.sp_pub.publish(sp)

    # ======================
    # Servo Control
    # ======================
    def set_servo(self, pin, pwm):
        response = self.mavros_cmd_service(broadcast=True,
                                      command=183,
                                      confirmation=0,
                                      param1=pin,
                                      param2=pwm,
                                      param3=0,
                                      param4=0,
                                      param5=0,
                                      param6=0,
                                      param7=0)
        print (response)

    # ======================
    # Command Functions
    # ======================
    def arm(self):
        return self.arm_service(True)

    def disarm(self):
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        self.mode_service(custom_mode="4")  # GUIDED NO GPS
        self.arm()
        print(self.takeoff_service(altitude=height))

    def land(self):
        self.mode_service(custom_mode="9")  # LAND
        self.disarm()

    def switch_gps(self):
        self.mode_service(custom_mode="3")  # GUIDED NO GPS
        self.arm()

    def reached_goal(self, goal, tol=0.2):
        if self.current_pose is None:
            return False

        dx = self.current_pose.pose.position.x - goal[0]
        dy = self.current_pose.pose.position.y - goal[1]
        dz = self.current_pose.pose.position.z - goal[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        rospy.loginfo(f"Distance to goal: {distance:.2f}m")
        rospy.loginfo(f"[OUTDOOR] Current Position x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, z={self.current_pose.pose.position.z}")

        return distance < tol

    def publish_waypoint_indoor(self, x, y, z, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "camera_init"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

        rospy.loginfo(f"Publishing waypoint: x={x}, y={y}, z={z}, yaw={yaw}")
        self.waypoint_pub.publish(goal)
        return goal

    def publish_waypoint_outdoor(self, x, y, z, yaw):
            sp = PositionTarget()
            sp.header.frame_id = "map"
            sp.header.stamp = rospy.Time.now()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

            sp.type_mask = (
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

            sp.velocity.x = 0.1
            sp.velocity.y = 0.1
            sp.velocity.z = 0.1
            sp.position.x = x
            sp.position.y = y
            sp.position.z = z
            sp.yaw = yaw

            self.sp_pub.publish(sp)

            rospy.loginfo(f"[OUTDOOR] Send direct waypoint x={x}, y={y}, z={z}, yaw={yaw}")

    def switch_to_outdoor(self):
        try:
            self.waypoint_pub.unregister()
        except:
            pass

        self.do_indoor = False
        rospy.loginfo("Switching to outdoor mission... Ego Planner stopped.")



    def go_to_waypoint_indoor(self, x, y, z, yaw):
        """Pergi ke waypoint tunggal"""
        self.current_yaw = yaw
        self.publish_waypoint_indoor(x, y, z, yaw)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.reached_goal((x, y, z)):
            self.publish_waypoint_indoor(x, y, z, yaw)
            rate.sleep()

        rospy.loginfo("Waypoint reached!")


    def go_to_waypoint_outdoor(self, x, y, z, yaw):
        self.current_yaw = yaw
        self.publish_waypoint_outdoor(x, y, z, yaw)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.reached_goal((x, y, z), tol = 1):
            self.publish_waypoint_outdoor(x, y, z, yaw)
            rate.sleep()

        rospy.loginfo("Outdoor waypoint reached!")


    def pos_target(self, pose_msg):
        # Extract the x and y values from the received Pose2D message
        self.posx = pose_msg.x
        self.posy = pose_msg.y
        self.theta = pose_msg.theta
        self.target = True
        self.last_msg_time = time.time()
    
    def set_vel(self, vx, vy, vz, yaw, heading):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        yaw_rad = math.radians(heading)
        N = vx * math.cos(yaw_rad)  + vy * math.sin(yaw_rad)
        E = -vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)

        cmd_vel.linear.x = N/3.281
        cmd_vel.linear.y = E/3.281
        cmd_vel.linear.z = vz/3.281

        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = yaw

        self.cmd_vel_pub.publish(cmd_vel)

    # --- Implementasi find_target menggunakan ego_planner (pos_target) ---
    def find_target(self, thr=1, head = 90):
        self.target = False
        
        while not rospy.is_shutdown():
            current_time = time.time()

            if current_time - self.last_msg_time > 2:
                self.target = False
                rospy.loginfo("Cannot find target")
                time.sleep(0.2)

            if self.target:
                output_x = pid_x_target.update(self.posx)
                output_y = pid_y_target.update(self.posy)

                print(f"x: {output_x:.3f}, y: {output_y:.3f}")

                if abs(output_x)<thr and abs(output_y)<thr:
                    print("Success, target Found!")
                    print("Dropping target!")
                    self.set_servo(14, 2400)
                    # break

                self.set_vel(output_x, output_y, 0, 0, head)
                time.sleep(0.2)
            else:
                print("Waiting for target")
                rospy.sleep(1.0)
  
    # ======================
    # Main Run
    # ======================
    def run(self):
        rospy.loginfo("Bismillahirrahmanirrahim - Starting mission")
        rospy.loginfo("Indoor Mission Starting")
        ## Indoor Mission
        self.go_to_waypoint_indoor(1.0, 0.0, 1, math.radians(0))
        self.go_to_waypoint_indoor(1.0, 1.0, 1, math.radians(0))
        self.go_to_waypoint_indoor(0.0, 1.0, 1, math.radians(0))
        self.go_to_waypoint_indoor(0.0, 0.0, 1, math.radians(0))


        


    
        rospy.loginfo("Mission complete! Landing...")
        self.land()
        rospy.spin()



pid_x_target = PIDController(Kp=0.001, Ki=0.0001, Kd=0.000, windup_max=10000, windup_min=-10000, decay_factor=0.01)
pid_y_target = PIDController(Kp=0.001, Ki=0.0001, Kd=0.000, windup_max=10000, windup_min=-10000, decay_factor=0.01)

if __name__ == "__main__":
    controller = MavController()
    controller.run()