#!/usr/bin/env python3

# Libraries ROS2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#Libraries Python
import pandas as pd
import numpy as np

# MACROS
path_to_csv = '/home/martin/SCARA-RI20252/src/csv'

class TrajectoryPlannerNode(Node):

    def __init__(self):
        super().__init__("trajectory_planner_node")

        # Info
        self.get_logger().info("Waiting for DXF")

        # Variables
        self.waypoints = pd.read_csv(path_to_csv)
        self.x = self.waypoints['x'].tolist()
        self.y = self.waypoints['y'].tolist()
        self.z = self.waypoints['z'].tolist()
        self.figure_type = self.waypoints['figure_type'].tolist()
        self.figure_id = self.waypoints['figure_id'].tolist()

        # Info
        self.get_logger().info(f"# Waypoints: {len(self.waypoints)}")

        # Tray Publisher
        self.trajectory_pub = self.create_publisher(Path, "planned_trajectory", 10)

        # Interpolation between waypoints
        self.interpolated_wp = self.interpolate(self.waypoints)
        self.get_logger().info(f"# Interpolated Waypoints: {len(self.interpolated_wp)}")

    # Interpolation
    def quintic_interpolation(self, q0, qf, T, N, v0=0.0, vf=0.0, a0=0.0, af=0.0):
        t = np.linspace(0, T, N)
        a0_c = q0
        a1_c = v0
        a2_c = a0 / 2
        a3_c = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T**2) / (2 * T**3)
        a4_c = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T**2) / (2 * T**4)
        a5_c = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T**2) / (2 * T**5)
        q = a0_c + a1_c * t + a2_c * t**2 + a3_c * t**3 + a4_c * t**4 + a5_c * t**5
        return q

    # Point Density for diffrent DXF entities
    def get_point_density(self, figure_type):
        if figure_type in ["LINE", "LWPOLYLINE", "POLYLINE"]:       # Lines and Polylines get more points because only 2 WP
            return 50    
        elif figure_type in ["CIRCLE", "ARC"]:                      # Circles and Arcs get fewer points because they have many WP
            return 3     
        elif figure_type == "SPLINE":                               # Splines get a medium number of points    
            return 15    
        else:
            return 20

    def interpolate(self, waypoints):
        points = pd.DataFrame(columns=['x', 'y', 'z', 'figure_type', 'figure_id'])

        # Check if waypoints is empty
        if waypoints.empty:
            self.get_logger().warn("No waypoints to interpolate.")
            return points

        # Variables
        F_start_point = waypoints.iloc[0]
        F_id = figure_start_point[4]

        for i in range(len(waypoints) - 1):
            # Current and next point
            actual_point = waypoints.iloc[i]
            next_point = waypoints.iloc[i + 1]

            # Extract data
            actual_point_id = actual_point['figure_id']
            next_point_id = next_point['figure_id']

            actual_point_type = actual_point['figure_type']


            if actual_point_id == next_point_id:                # Still in the same figure
                N = self.get_point_density(actual_point_type)   # Number of points based on figure type 
                T = 5.0                                         # DT parameter for interpolation

                # Interpolate between actual_point and next_point
                interpolated_x = self.quintic_interpolation(actual_point['x'], next_point['x'], T, N)
                interpolated_y = self.quintic_interpolation(actual_point['y'], next_point['y'], T, N)
                interpolated_z = self.quintic_interpolation(actual_point['z'], next_point['z'], T, N)

                # Add interpolated points to DataFrame
                pd.concat([points, pd.DataFrame({'x': interpolated_x, 'y': interpolated_y, 'z': interpolated_z, 'figure_type': actual_point_type, 'figure_id': actual_point_id})], ignore_index=True)

                self.get_logger().info("same_figure")

            else:
                self.get_logger().info("figure_change")
        return points

# Publish trajectory
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    node.trajectory_planner(node.final_pts)  # <- Publicar trayectoria interpolada
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
