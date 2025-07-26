import numpy as np
from scipy.optimize import minimize

def dh_transform(a, alpha, d, theta):
    """Compute standard DH transformation matrix."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,      ca,     d],
        [0,        0,       0,     1]
    ])


class UR10eKinematics:
    """UR10e Kinematics with a prismatic lift joint.
    This class implements the forward kinematics for the UR10e robot with a prismatic lift joint.
    The first joint is prismatic (lift), and the rest are revolute joints.
    The DH parameters are defined in the constructor.
    The forward kinematics computes the end-effector pose given joint angles.
    """
    def __init__(self):
        """Initialize the UR10e kinematics with DH parameters.
        The first joint is prismatic (lift), and the rest are revolute joints.
        The DH parameters are defined as follows:
        [a, alpha, d, theta] where:
        - a: link length
        - alpha: link twist
        - d: link offset (fixed for revolute, variable for prismatic)
        - theta: joint angle (fixed for prismatic, variable for revolute)
        """
        # Add lift joint as prismatic (first row)
        # Format: [a, alpha, d, theta] — here theta=0 for prismatic
        self.dh_params = [
            [0.0, 0.0, 0.0, 0.0],               # Lift height (prismatic, d is variable)
            [0.0, np.pi/2, 0.181, 0.0],         # θ1
            [0.0, 0.0, 0.0, np.pi/2],           # θ2 + π/2
            [0.613, 0.0, 0.0, 0.0],             # θ3
            [0.572, 0.0, 0.174, -np.pi/2],      # θ4 - π/2
            [0.0, -np.pi/2, 0.120, 0.0],        # θ5
            [0.0, np.pi/2, 0.0, 0.0],           # θ6
        ]

    def forward_kinematics(self, q):
        """
        Compute the forward kinematics for the UR10e robot with a prismatic lift joint.
        Args:
        q: [lift_height, θ1, θ2, ..., θ6]
        Returns: 4x4 transformation matrix of end-effector
        """
        assert len(q) == 7, "Expected 7 joint values (1 prismatic + 6 revolute)"
        T = np.eye(4)
        for i in range(7):
            a, alpha, d_fixed, theta_fixed = self.dh_params[i]
            if i == 0:
                d = q[0]  # lift_height
                theta = 0
            else:
                d = d_fixed
                theta = q[i] + theta_fixed
            A_i = dh_transform(a, alpha, d, theta)
            T = T @ A_i
        return T

    def print_pose(self, T):
        """Print the pose in a readable format.
        Args:
        T: 4x4 transformation matrix"""
        position = T[:3, 3]
        R = T[:3, :3]
        print(f"Position (m): {position}")
        print("Rotation matrix:")
        print(np.round(R, 4))



class UR10eIKSolver:
    """UR10e Inverse Kinematics Solver with a prismatic lift joint.
    This class uses the UR10eKinematics class to compute forward kinematics
    and solve inverse kinematics using optimization.
    The first joint is prismatic (lift), and the rest are revolute joints.
    The IK solver minimizes the pose error between the desired end-effector pose and the computed pose
    from the joint angles."""
    def __init__(self):
        """Initialize the UR10e IK solver with kinematics.
        This class uses the UR10eKinematics class to compute forward kinematics
        and solve inverse kinematics using optimization.
        The first joint is prismatic (lift), and the rest are revolute joints.
        """
        self.kin = UR10eKinematics()

    def pose_error(self, q, T_goal):
        """Compute the pose error between the desired pose and the computed pose.
        Args:
        q: [lift_height, θ1, θ2, ..., θ6] joint angles
        T_goal: 4x4 target pose (desired end-effector pose)
        Returns: scalar error value (position + orientation error)
        """
        T_fk = self.kin.forward_kinematics(q)
        pos_fk = T_fk[:3, 3]
        pos_goal = T_goal[:3, 3]
        pos_error = np.linalg.norm(pos_goal - pos_fk)

        R_fk = T_fk[:3, :3]
        R_goal = T_goal[:3, :3]
        R_error = np.linalg.norm(R_goal - R_fk)

        return pos_error + 0.1 * R_error  # Weighting orientation error less

    def solve(self, T_goal, q_init=None):
        """Solve the inverse kinematics for the UR10e robot with a prismatic lift joint.
        Args:
        T_goal: 4x4 target pose (desired end-effector pose)
        q_init: Initial guess for joint angles (optional, defaults to zeros)
        Returns: (lift_height, [θ1, θ2, ..., θ6]) joint
            angles if successful, otherwise (None, None)"""
        if q_init is None:
            q_init = np.zeros(7)  # 1 lift + 6 revolute

        joint_bounds = [
            (0.0, 0.5),               # lift height (meters)
            (-np.pi/2, np.pi/2),      # θ1
            (-np.pi/4, np.pi/4),      # θ2
            (-np.pi/4, np.pi/4),      # θ3
            (-3.0, 3.0),              # θ4
            (-2.5, 2.5),              # θ5
            (-2*np.pi, 2*np.pi),      # θ6
        ]

        res = minimize(
            self.pose_error,
            q_init,
            args=(T_goal,),
            method='L-BFGS-B',
            bounds=joint_bounds,
            options={'maxiter': 500, 'ftol': 1e-6}
        )

        if res.success:
            q_sol = res.x
            lift = q_sol[0]
            joints = q_sol[1:]
            return lift, joints
        else:
            print("IK failed:", res.message)
            return None, None


def create_test_pose(x=0.4, y=0.0, z=1.0):
    """Create a simple 4x4 target pose with position and identity orientation."""
    T_goal = np.eye(4)
    T_goal[0, 3] = x
    T_goal[1, 3] = y
    T_goal[2, 3] = z
    return T_goal

def test_ik_solver():
    """Test the UR10e IK solver with a simple pose.
    This will create a target pose and solve for the joint angles."""
    ik_solver = UR10eIKSolver()
    T_goal = create_test_pose(x=0.6, y=0.1, z=1.2)

    lift, joint_angles = ik_solver.solve(T_goal)

    if lift is not None:
        print(f"IK Success!")
        print(f"Lift height: {lift:.4f} m")
        print(f"Joint angles (rad): {np.round(joint_angles, 4)}")

        # Optional: verify forward kinematics
        kin = UR10eKinematics()
        q = np.hstack(([lift], joint_angles))
        T_fk = kin.forward_kinematics(q)
        kin.print_pose(T_fk)

        # Position difference
        error = np.linalg.norm(T_fk[:3, 3] - T_goal[:3, 3])
        print(f"Position error: {error:.6f} m")
    else:
        print("IK Failed!")

def test_ik_solver2():
    """Test the UR10e IK solver with a different pose.
    This will create a target pose and solve for the joint angles."""
    ik_solver = UR10eIKSolver()
    T_goal = create_test_pose(x=0.5, y=0.0, z=1.5)  # Your desired pose

    lift, joint_angles = ik_solver.solve(T_goal)

    if lift is not None:
        print(f"IK Success!")
        print(f"Lift height: {lift:.4f} m")
        print(f"Joint angles (rad): {np.round(joint_angles, 4)}")

        kin = UR10eKinematics()
        q = np.hstack(([lift], joint_angles))
        T_fk = kin.forward_kinematics(q)
        kin.print_pose(T_fk)

        error = np.linalg.norm(T_fk[:3, 3] - T_goal[:3, 3])
        print(f"Position error: {error:.6f} m")
    else:
        print("IK Failed!")

if __name__ == "__main__":
    
    test_ik_solver2()


