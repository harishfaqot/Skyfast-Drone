#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def add_box(points, min_x, max_x, min_y, max_y, min_z, max_z, resolution):
    x = min_x
    while x <= max_x + 1e-6:
        y = min_y
        while y <= max_y + 1e-6:
            z = min_z
            while z <= max_z + 1e-6:
                points.append((x, y, z))
                z += resolution
            y += resolution
        x += resolution


def add_rotated_box(points, center_x, center_y, length, width, min_z, max_z, yaw, resolution):
    half_l = length * 0.5
    half_w = width * 0.5
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    lx = -half_l
    while lx <= half_l + 1e-6:
        ly = -half_w
        while ly <= half_w + 1e-6:
            x = center_x + lx * cos_y - ly * sin_y
            y = center_y + lx * sin_y + ly * cos_y
            z = min_z
            while z <= max_z + 1e-6:
                points.append((x, y, z))
                z += resolution
            ly += resolution
        lx += resolution


def add_gate_frame(points, center_x, center_y, gate_width, gate_height, frame_thickness, frame_depth, yaw, resolution):
    half_gate = gate_width * 0.5
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    left_cx = center_x - half_gate * cos_y
    left_cy = center_y - half_gate * sin_y
    right_cx = center_x + half_gate * cos_y
    right_cy = center_y + half_gate * sin_y

    add_rotated_box(points, left_cx, left_cy, frame_thickness, frame_depth, 0.0, gate_height, yaw, resolution)
    add_rotated_box(points, right_cx, right_cy, frame_thickness, frame_depth, 0.0, gate_height, yaw, resolution)
    add_rotated_box(
        points,
        center_x,
        center_y,
        gate_width + frame_thickness,
        frame_depth,
        gate_height,
        gate_height + frame_thickness,
        yaw,
        resolution,
    )


def main():
    rospy.init_node("custom_wall_gate_map")

    map_x_size = rospy.get_param("~map/x_size", 40.0)
    map_y_size = rospy.get_param("~map/y_size", 20.0)
    map_z_size = rospy.get_param("~map/z_size", 4.0)
    resolution = rospy.get_param("~map/resolution", 0.1)
    wall_length = rospy.get_param("~course/wall_length", 10.0)
    wall_thickness = rospy.get_param("~course/wall_thickness", 0.5)
    wall_height = rospy.get_param("~course/wall_height", 3.0)
    gate_width = rospy.get_param("~course/gate_width", 1.8)
    gate_height = rospy.get_param("~course/gate_height", 1.8)
    gate_frame_thickness = rospy.get_param("~course/gate_frame_thickness", 0.18)
    gate_depth = rospy.get_param("~course/gate_depth", 0.2)
    boundary_thickness = rospy.get_param("~course/boundary_thickness", 0.25)
    corridor_length = rospy.get_param("~course/corridor_length", 20.0)
    corridor_width = rospy.get_param("~course/corridor_width", 2.4)
    corridor_wall_thickness = rospy.get_param("~course/corridor_wall_thickness", 0.25)
    angled_path_length = rospy.get_param("~course/angled_path_length", 2.8)
    angled_path_width = rospy.get_param("~course/angled_path_width", 1.5)

    global_map_pub = rospy.Publisher("/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True)
    local_map_pub = rospy.Publisher("/map_generator/local_cloud", PointCloud2, queue_size=1, latch=True)

    points = []

    left = -map_x_size * 0.5
    right = map_x_size * 0.5
    bottom = -map_y_size * 0.5
    top = map_y_size * 0.5
    center = 0.0

    add_box(points, left, right, top - boundary_thickness, top, 0.0, wall_height, resolution)
    add_box(points, left, right, bottom, bottom + boundary_thickness, 0.0, wall_height, resolution)
    add_box(points, left, left + boundary_thickness, bottom, top, 0.0, wall_height, resolution)
    add_box(points, right - boundary_thickness, right, bottom, top, 0.0, wall_height, resolution)
    add_box(points, left + 14.0, right - 8.0, center - boundary_thickness / 2.0, center + boundary_thickness / 2.0, 0.0, wall_height, resolution)

    gate1_x = right - 9
    gate1_y = bottom + 5.0
    gate2_x = right - 14
    gate2_y = bottom + 5.0
    gate3_x = left + 14
    gate3_y = bottom + 5.0 - 1
    gate4_x = left + 14
    gate4_y = bottom + 5.0 + 1
    gate5_x = left + 16.5
    gate5_y = top - 5.0 - 1
    gate6_x = left + 16.5
    gate6_y = top - 5.0 + 1
    gate7_x = right - 6.5
    gate7_y = top - 5.0 - 1
    gate8_x = right - 6.5
    gate8_y = top - 5.0 + 1

    add_gate_frame(points, gate1_x, gate1_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate2_x, gate2_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate3_x, gate3_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate4_x, gate4_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate5_x, gate5_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate6_x, gate6_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate7_x, gate7_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)
    add_gate_frame(points, gate8_x, gate8_y, gate_width, gate_height, gate_frame_thickness, gate_depth, math.pi * 0.5, resolution)

    header = rospy.Header()
    header.frame_id = "world"
    cloud_msg = point_cloud2.create_cloud_xyz32(header, points)

    rate = rospy.Rate(1.0)
    print("Custom wall and gate map published with {} points.".format(len(points)))
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()
        global_map_pub.publish(cloud_msg)
        local_map_pub.publish(cloud_msg)
        rate.sleep()


if __name__ == "__main__":
    main()