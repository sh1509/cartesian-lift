import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from ur10e_ikpy_solver.ik_solver.ur10e_kinematics import UR10eIKSolver
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from visualization_msgs.msg import Marker

""" A ROS2 node for running Cartesian motion with a UR10e robot with a lift joint.
This node will handle the trajectory generation and execution for the UR10e robot with lift joint.
It will publish the trajectory points and markers for visualization."""

class CartesianMotionNode(Node):
    """ A ROS2 node for running Cartesian motion with a UR10e robot with a lift joint.
    This node will handle the trajectory generation and execution for the UR10e robot with lift joint
    It will publish the trajectory points and markers for visualization.
    """
    def __init__(self):
        """ Initialize the Cartesian Motion Node.
        This node will handle the trajectory generation and execution for the UR10e robot with lift joint. """
        super().__init__('cartesian_motion_node')
        self.get_logger().info("Initializing Cartesian Motion Node...")
        self.ik_solver = UR10eIKSolver()

        # Publisher
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)


        # State listener
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_lift = 0.0

        # Robot joint order
        self.joint_names = [
            'lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self.timer = self.create_timer(1.0, self.run_trajectory_once)
        self.joint_history = []
        self.ee_history = []

        ## Test the publication of Cartesian motion
        self.points_to_send = []
        self.current_point_index = 0
        self.position_tolerance = 0.05
        self.latest_joint_states = None
        self.publish_timer = self.create_timer(0.1, self.publish_next_point)

    def joint_state_callback(self, msg):
        """ Callback for joint state updates.
        This will update the current lift height and log the joint states. """
        self.latest_joint_states = msg
        self.get_logger().debug(f"Received joint states: {msg.name} positions: {msg.position}")
        if 'lift_joint' in msg.name:
            idx = msg.name.index('lift_joint')
            self.current_lift = msg.position[idx]

    def run_trajectory_once(self):
        """ Generate and publish a trajectory for the UR10e robot with lift joint.
        This method will create a trajectory that moves the end-effector in a straight line along the Z-axis,
        while also adjusting the lift joint."""
        self.timer.cancel()

        wall_x = 0.5
        y = 0.0

        waypoints = [
            # [wall_x, y, 0.8],
            # [wall_x, y, 1.2],
            [wall_x, y, 1.5],
            # [wall_x, y, 2.0],
            [wall_x, y, 2.0]
        ]
        total_time = 0.0

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        DT = 0.1  # seconds per segment
        self.get_logger().info(f"Preparing trajectory with {len(waypoints)} waypoints, total time: {total_time:.2f} seconds")
        for i in range(len(waypoints) - 1):
            # Interpolate between waypoints
            self.get_logger().info(f"Interpolating between waypoints {i} and {i + 1}")
            p1 = waypoints[i]
            p2 = waypoints[i + 1]

            # Build start and end poses
            T1 = np.eye(4)
            T2 = np.eye(4)
            T1[:3, 3] = p1
            T2[:3, 3] = p2
            T1[:3, :3] = T2[:3, :3] = R.from_euler('xyz', [0, 0, 0]).as_matrix()

            steps = 30  # Number of sub-steps (more = smoother, slower)
            interp_poses = self.interpolate_cartesian_path(T1, T2, steps)
            self.get_logger().info(f"Interpolating {len(interp_poses)} poses from {p1} to {p2}")
            for T in interp_poses:
                # Solve IK for each interpolated pose
                self.get_logger().info(f"Interpolated pose: {T[:3, 3]}")
                lift, joints = self.ik_solver.solve(T)
                if joints is None:
                    self.get_logger().warn("IK failed at one interpolated step.")
                    continue

                full_joints = [lift] + list(joints)
                p = JointTrajectoryPoint()
                p.positions = full_joints
                p.velocities = [0.0] * len(self.joint_names)
                p.accelerations = [0.0] * len(self.joint_names)
                p.time_from_start = rclpy.time.Duration(seconds=total_time).to_msg()
                msg.points.append(p)
                self.ee_history.append(T[:3, 3])
                self.joint_history.append(full_joints)
                total_time += DT  # fixed interval (dt = 0.1 means 10Hz, constant velocity)
        # Finalize the trajectory message
        self.points_to_send = msg.points
        self.current_point_index = 0
        self.get_logger().info("Prepared all trajectory points for sequential publishing.")

    def interpolate_cartesian_path(self, start_pose, end_pose, num_steps):
        """
        Linearly interpolate between two Cartesian poses in position only.
        Returns a list of 4x4 transformation matrices.
        """
        poses = []
        for i in range(num_steps):
            alpha = i / (num_steps - 1)
            pos = (1 - alpha) * start_pose[:3, 3] + alpha * end_pose[:3, 3]
            T = np.eye(4)
            T[:3, 3] = pos
            T[:3, :3] = start_pose[:3, :3]  # Keep same orientation
            poses.append(T)
        return poses

    def publish_next_point(self):
        """ Publish the next point in the trajectory.
        This will publish the next point in the trajectory and update the marker position.
        If all points have been published, it will log that and plot the velocities. """
        self.get_logger().debug(f"Publishing point {len(self.points_to_send)} length: {self.current_point_index}")
        if self.current_point_index == len(self.points_to_send):
            self.get_logger().debug("All points published.")
            self.plot_velocities()
            return

        x, y, z = self.ee_history[self.current_point_index]
        self.publish_marker(x, y, z, idx=self.current_point_index)
        self.publish_point(self.current_point_index)
        self.current_point_index += 1

    def publish_point(self, idx):
        """ Publish a single point in the trajectory.
        This will publish the joint positions for the given index and update the marker position."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [self.points_to_send[idx]]
        x, y, z = self.ee_history[idx]
        self.publish_marker(x, y, z, idx=idx)
        # Debug print: joint names and positions being published
        for name, pos in zip(msg.joint_names, msg.points[0].positions):
            self.get_logger().debug(f"  Joint: {name}, Position: {pos:.4f}")
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f"Published trajectory point {idx + 1}/{len(self.points_to_send)}")

    def robot_reached_goal(self, goal_positions):
        """ Check if the robot has reached the goal positions.
        This will compare the current joint positions with the goal positions and return True if within tolerance."""
        current_positions = []
        for joint_name in self.joint_names:
            if joint_name in self.latest_joint_states.name:
                idx = self.latest_joint_states.name.index(joint_name)
                current_positions.append(self.latest_joint_states.position[idx])
            else:
                return False

        diff = np.abs(np.array(current_positions) - np.array(goal_positions))
        return np.all(diff < self.position_tolerance)
    
    def publish_marker(self, x, y, z, idx=0):
        """ Publish a marker at the given position.
        This will create a marker at the specified position and publish it to the marker topic."""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_markers"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Yellow color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published marker at ({x}, {y}, {z}) with id {idx}")

    def plot_velocities(self):
        """ Plot the joint and end-effector velocities.
        This will calculate the joint and end-effector velocities from the history and plot them using matplotlib"""
        joint_history = np.array(self.joint_history)
        ee_history = np.array(self.ee_history)

        if len(joint_history) < 2:
            return

        dt = 0.1  # seconds per segment (same as used above)
        joint_velocities = np.diff(joint_history, axis=0) / dt
        ee_velocities = np.diff(ee_history, axis=0) / dt

        plt.figure(figsize=(12, 5))

        plt.subplot(1, 2, 1)
        plt.title("Joint Velocities")
        joint_labels = [
            "shoulder_pan (rad/s)",
            "shoulder_lift (rad/s)",
            "elbow (rad/s)",
            "wrist_1 (rad/s)",
            "wrist_2 (rad/s)",
            "wrist_3 (rad/s)",
            "lift (m/s)"
        ]

        # Clip to minimum length
        num_joints = min(joint_velocities.shape[1], len(joint_labels))
        colors = cm.get_cmap('tab10', num_joints)

        for i in range(num_joints):
            plt.plot(joint_velocities[:, i], color=colors(i), label=joint_labels[i], linewidth=2)
        plt.legend()
        plt.xlabel("Segment")
        plt.ylabel("rad/s or m/s")

        plt.subplot(1, 2, 2)
        plt.title("End Effector Velocities (XY)")
        plt.plot(ee_velocities[:, 0], label='X vel')
        plt.plot(ee_velocities[:, 1], label='Y vel')
        plt.legend()
        plt.xlabel("Segment")
        plt.ylabel("m/s")

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
