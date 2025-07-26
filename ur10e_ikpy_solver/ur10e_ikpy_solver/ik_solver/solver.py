import os
import numpy as np
from ikpy.chain import Chain

"""UR10e Inverse Kinematics Solver with Prismatic Lift Joint.
This module defines the kinematics and inverse kinematics solver for the UR10e robot,
which includes a prismatic lift joint as the first joint.
The UR10eKinematics class computes forward kinematics, and the UR10eIKSolver
class solves the inverse kinematics using optimization.
The IK solver minimizes the pose error between the desired end-effector pose and the computed pose
from the joint angles."""

class UR10eIKWithLift:
    """UR10e Inverse Kinematics Solver with a prismatic lift joint.
    This class uses the UR10eKinematics class to compute forward kinematics
    and solve inverse kinematics using optimization.
    The first joint is prismatic (lift), and the rest are revolute joints.
    The IK solver minimizes the pose error between the desired end-effector pose and the computed pose
    from the joint angles."""
    def __init__(self, urdf_path, max_lift=0.5, lift_step=0.01):
        """Initialize the UR10e IK solver with kinematics.
        This class uses the UR10eKinematics class to compute forward kinematics
        and solve inverse kinematics using optimization.
        The first joint is prismatic (lift), and the rest are revolute joints.
        Args:
            urdf_path (str): Path to the URDF file of the UR10e robot.
            max_lift (float): Maximum lift height in meters.
            lift_step (float): Step size for the prismatic joint in meters.
        """
        # Load UR10e arm kinematic chain (no lift joint)
        self.chain = Chain.from_urdf_file(urdf_path)

        joint_limits = [
            (-2*np.pi, 2*np.pi),
            (-2*np.pi, 2*np.pi),
            (-2*np.pi, 2*np.pi),
            (-2*np.pi, 2*np.pi),
            (-2*np.pi, 2*np.pi),
            (-2*np.pi, 2*np.pi),
            # Note: lift joint is handled outside this chain if prismatic transform is separate
        ]

        for i, limits in enumerate(joint_limits, start=1):
            if i < len(self.chain.links):
                self.chain.links[i].bounds = limits
        self.max_lift = max_lift
        self.lift_step = lift_step

    def get_lift_transform(self, lift_height):
        """4x4 transform for prismatic lift joint along Z axis"""
        T = np.eye(4)
        T[2, 3] = lift_height
        return T
    
    def forward_kinematics(self, joint_angles, lift):
        """Compute full end-effector transform including lift."""
        T_lift = self.get_lift_transform(lift)
        T_arm = self.chain.forward_kinematics(joint_angles)
        return T_lift @ T_arm

    def is_valid_solution(self, joint_angles):
        """Basic validity check:
           - Joint count matches chain
           - Joint angles within limits (simplified here)
        """
        angles = joint_angles[1:-1]
        for i, angle in enumerate(angles):
            link = self.chain.links[i+1]  # +1 because first is fixed
            if hasattr(link, 'bounds'):
                low, high = link.bounds
                if low is not None and angle < low:
                    return False
                if high is not None and angle > high:
                    return False
        return True

    def solve_ik(self, target_transform, initial_lift=0.0):
        """
        Solve IK for target end-effector pose, starting from current lift height.

        Parameters:
            target_transform (numpy.ndarray): 4x4 homogeneous target pose.
            initial_lift (float): starting Z height for lift (e.g., from joint state).

        Returns:
            (lift_value, joint_angles) or (None, None) if no solution.
        """
        # TODO try decreasing lift first if initial_lift is too high
        lift = max(0.0, initial_lift)  # Clamp to non-negative
        while lift <= self.max_lift:
            print(f"Trying lift: {lift:.3f}")
            T_lift = self.get_lift_transform(lift)
            T_target_in_base = np.linalg.inv(T_lift) @ target_transform
            ik_solution = self.chain.inverse_kinematics_frame(target=T_target_in_base)
            if self.is_valid_solution(ik_solution):
                joint_angles_cleaned = ik_solution[1:7]
                # Optional FK verification:
                T_fk = self.forward_kinematics(ik_solution, lift)
                if np.allclose(T_fk, target_transform, atol=1e-3):
                    return lift, joint_angles_cleaned
                print(f"No valid solution at lift {lift:.3f}, trying next step.")
            lift += self.lift_step
        return None, None