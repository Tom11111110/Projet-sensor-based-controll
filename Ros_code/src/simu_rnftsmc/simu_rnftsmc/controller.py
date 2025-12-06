import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster
import numpy as np
import math
from .trajectory import trajectory_reference
from .rnftsmc import error_dynamics_rnftsmc, return_command_accelerations
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path

def clamp(x, low, high):
    return max(low, min(high, x))

class RNFTSMCController(Node):
    def __init__(self):
        super().__init__('rnftsmc_controller')

        # --- Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('disturb_amp', 0.2)
        self.rate_hz = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.rate_hz

        # --- Control parameters
        self.control_params = {
            'mu1': 1.0, 'mu2': 1.0, 'mu3': 1.0, 'mu4': 1.0,
            'Phi1': 0.5, 'Phi2': 0.5,
            'Theta1': 1.5, 'Theta2': 1.5,
            'kappa01': 0.1, 'kappa1': 0.1, 'kappa2': 0.1,
            'kappa02': 0.1, 'kappa3': 0.1, 'kappa4': 0.1,
            'a1': 1.0, 'a2': 1.0,
            'z1': 0.1, 'z2': 0.1,
            'epsilon': 0.05,
            'disturb_amp': self.get_parameter('disturb_amp').value,
            'tiny': 1e-12
        }

        # --- Trajectory parameters
        self.traj_params = {'R_cercle': 2.0, 'centre_x': 0.0, 'centre_y': 0.0, 'omega_r': 1.0}

        # --- Initial robot pose
        self.x = 1.5
        self.y = 2.0
        self.theta = 0.0

        # --- Initial error state
        xd0, yd0, dxd0, dyd0, _, _ = trajectory_reference(0.0, self.traj_params)
        e1 = self.x - xd0
        e2 = self.y - yd0
        e3 = 0.0 - dxd0
        e4 = 0.0 - dyd0
        self.e = np.array([e1, e2, e3, e4], dtype=float)

        # --- Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.traj_pub = self.create_publisher(Marker, 'reference_trajectory', QoSProfile(depth=10))
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # --- Static TF for RViz
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # --- Publish initial reference trajectory
        self.publish_reference_trajectory()

        # --- Timers
        self.timer = self.create_timer(self.dt, self.control_loop)
        # Publish reference trajectory repeatedly for RViz
        self.traj_timer = self.create_timer(0.5, self.publish_reference_trajectory)

        self.get_logger().info(f'RNFTSMC controller started at {self.rate_hz} Hz')

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
        dedt = error_dynamics_rnftsmc(t, self.e, self.control_params, self.traj_params)
        self.e = self.e + dedt * self.dt

        xd, yd, dxd, dyd, ddxd, ddyd = trajectory_reference(t, self.traj_params)
        self.x = xd + self.e[0]
        self.y = yd + self.e[1]

        v_x_cmd, v_y_cmd = return_command_accelerations(t, self.e, self.control_params, self.traj_params)
        vx_est = self.e[2] + dxd
        vy_est = self.e[3] + dyd
        vx_des = vx_est + v_x_cmd * self.dt
        vy_des = vy_est + v_y_cmd * self.dt

        cos_t = np.cos(self.theta)
        sin_t = np.sin(self.theta)
        vx_robot = cos_t * vx_des + sin_t * vy_des
        vy_robot = -sin_t * vx_des + cos_t * vy_des

        v_linear = vx_robot
        desired_yaw = np.arctan2(vy_des, vx_des + 1e-9)
        yaw_err = (desired_yaw - self.theta + np.pi) % (2*np.pi) - np.pi
        k_omega = 2.0
        v_angular = clamp(k_omega * yaw_err, -2.0, 2.0)

        twist = Twist()
        twist.linear.x = float(v_linear)
        twist.angular.z = float(v_angular)
        self.cmd_pub.publish(twist)

        # Update robot pose
        self.theta += v_angular * self.dt
        self.x += v_linear * np.cos(self.theta) * self.dt
        self.y += v_linear * np.sin(self.theta) * self.dt

        # Publish pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(self.theta / 2.0)
        pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.pose_pub.publish(pose)

        # Update and publish Path for RViz
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RNFTSMCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()