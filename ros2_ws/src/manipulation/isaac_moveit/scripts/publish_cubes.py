#!/usr/bin/env python3
 
## Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class CubePublisher(Node):
    def __init__(self):
        super().__init__('cube_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_cubes)

        # Ensure 'use_sim_time' is set to True
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    def publish_cubes(self):
        marker_array = MarkerArray()
        for i in range(4):  # Adjust the range to 4 since we have 4 cubes
            marker = Marker()
            if i == 0:
                marker.header.frame_id = "nvidia_cube"
            else:
                marker.header.frame_id = f"nvidia_cube_{i:02}"
            # marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cubes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # marker.pose.orientation.w = 1.0
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    cube_publisher = CubePublisher()
    rclpy.spin(cube_publisher)
    cube_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()