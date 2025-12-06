# tb3_rnftsmc/controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
from .trajectory import trajectory_reference
from .rnftsmc import error_dynamics_rnftsmc, return_command_accelerations
from rclpy.qos import QoSProfile

def clamp(x, low, high):
    return max(low, min(high, x))

class RNFTSMCControllerReal(Node):
    def __init__(self):
        super().__init__('rnftsmc_controller_real')

        # --- Parameters
        self.declare_parameter('control_rate', 50.0)
        self.rate_hz = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.rate_hz

        # --- Control parameters (RNFTSMC)
        self.control_params = {
            'mu1': 1.0, 'mu2': 1.0, 'mu3': 1.0, 'mu4': 1.0,
            'Phi1': 0.5, 'Phi2': 0.5,
            'Theta1': 1.5, 'Theta2': 1.5,
            'kappa01': 0.1, 'kappa1': 0.1, 'kappa2': 0.1,
            'kappa02': 0.1, 'kappa3': 0.1, 'kappa4': 0.1,
            'a1': 1.0, 'a2': 1.0,
            'z1': 0.1, 'z2': 0.1,
            'epsilon': 0.05,
            'disturb_amp': 0.0, # No artificial disturbances in real robot
            'tiny': 1e-12
        }

        # --- Trajectory parameters
        self.traj_params = {'R_cercle': 0.5, 'centre_x': 0.0, 'centre_y': 0.0, 'omega_r': 0.30}

        # --- Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- Error state (e1,e2,e3,e4)
        xd0, yd0, dxd0, dyd0, _, _ = trajectory_reference(0.0, self.traj_params)
        self.e = np.array([self.x - xd0, self.y - yd0, -dxd0, -dyd0], dtype=float)

        # --- Subscribers & Publishers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.traj_pub = self.create_publisher(Marker, 'reference_trajectory', QoSProfile(depth=10))
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # --- Publish reference trajectory
        self.publish_reference_trajectory()

        # --- Timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.traj_timer = self.create_timer(0.5, self.publish_reference_trajectory)

        self.get_logger().info(f'RNFTSMC controller for TurtleBot3 started at {self.rate_hz} Hz')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def publish_reference_trajectory(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "reference_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        num_points = 100
        for i in range(num_points + 1):
            t = i * 2 * np.pi / num_points
            xd, yd, _, _, _, _ = trajectory_reference(t, self.traj_params)
            point = Point()
            point.x = xd
            point.y = yd
            point.z = 0.0
            marker.points.append(point)

        self.traj_pub.publish(marker)

    def control_loop(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        # --- Compute RNFTSMC error dynamics
        dedt = error_dynamics_rnftsmc(t, self.e, self.control_params, self.traj_params)
        self.e += dedt * self.dt

        # Récupérer la position réelle du robot (depuis l'odom)
        xd, yd, dxd, dyd, ddxd, ddyd = trajectory_reference(t, self.traj_params)

        # Mettre à jour l'erreur e en fonction de la position réelle
        self.e[0] = self.x - xd
        self.e[1] = self.y - yd
        # les vitesses restent les mêmes
        self.e[2] = self.e[2]  # ou éventuellement vx_est - dxd
        self.e[3] = self.e[3]  # ou éventuellement vy_est - dyd

        # --- Compute desired accelerations
        v_x_cmd, v_y_cmd = return_command_accelerations(t, self.e, self.control_params, self.traj_params)

        # --- Desired velocities in world frame
        vx_est = self.e[2] + dxd
        vy_est = self.e[3] + dyd
        vx_des = vx_est + v_x_cmd * self.dt
        vy_des = vy_est + v_y_cmd * self.dt

        # --- Transform to robot frame
        cos_t = np.cos(self.theta)
        sin_t = np.sin(self.theta)
        vx_robot = cos_t * vx_des + sin_t * vy_des
        vy_robot = -sin_t * vx_des + cos_t * vy_des

        # --- Compute linear and angular velocities
        v_linear = clamp(vx_robot, -0.22, 0.22)  # max linear speed TurtleBot3 Burger
        desired_yaw = np.arctan2(vy_des, vx_des + 1e-9)
        yaw_err = (desired_yaw - self.theta + np.pi) % (4*np.pi) - np.pi
        k_omega = 2.0
        v_angular = clamp(k_omega * yaw_err, -2.84, 2.84)  # max angular speed

        twist = Twist()
        twist.linear.x = float(v_linear)
        twist.angular.z = float(v_angular)
        self.cmd_pub.publish(twist)

        # --- Publish robot pose for visualization
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(self.theta / 2.0)
        pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.pose_pub.publish(pose)

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RNFTSMCControllerReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()