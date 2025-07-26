import numpy as np
import os
import pytest
from ur10e_ikpy_solver.ik_solver.solver import UR10eIKWithLift
from ament_index_python.packages import get_package_share_directory

@pytest.fixture
def ik_solver():
    urdf_file = 'ur10e_without_lift.urdf'
    pkg = 'my_ur10e_custom_description'
    urdf_path = os.path.join(get_package_share_directory(pkg), 'urdf', urdf_file)
    return UR10eIKWithLift(urdf_path)

def test_ik_solution_found_at_zero_lift(ik_solver):
    # Target pose reachable without lift (identity)
    T_goal = np.eye(4)
    lift, joints = ik_solver.solve_ik(T_goal)
    assert lift is not None, "IK should find solution at zero lift"
    assert abs(lift) < 1e-5, "Lift should be near zero for reachable pose"
    assert len(joints) == len(ik_solver.chain.links)

def test_ik_solution_found_with_lift(ik_solver):
    # Target pose is far below robot base, so IK fails at zero lift but should succeed with lift
    # We simulate this by lowering target pose along Z axis
    T_goal = np.eye(4)
    T_goal[2, 3] = -0.6  # target 30cm below base frame
    lift, joints = ik_solver.solve_ik(T_goal)
    assert lift is not None, "IK should find solution with lift"
    assert lift > 0, "Lift should be > 0 for unreachable pose"
    assert len(joints) == len(ik_solver.chain.links)

def test_ik_no_solution(ik_solver):
    # Set a pose impossible even with max lift (far away in X)
    T_goal = np.eye(4)
    T_goal[0, 3] = 10.0  # 10 meters away, way out of reach
    lift, joints = ik_solver.solve_ik(T_goal)
    assert lift is None and joints is None, "No IK solution should be found for unreachable pose"

if __name__ == "__main__":
    pytest.main([__file__])
