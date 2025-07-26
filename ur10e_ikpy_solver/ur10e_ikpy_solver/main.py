from ik_solver.solver import UR10eIKWithLift
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np

def main():
    urdf_file = 'ur10e_without_lift.urdf'
    pkg = 'my_ur10e_custom_description'
    urdf_path = os.path.join(get_package_share_directory(pkg), 'urdf', urdf_file)


    ik_solver = UR10eIKWithLift(urdf_path)

    # Example target pose: identity (home pose)
    T_goal = np.eye(4)

    lift, joints = ik_solver.solve_ik(T_goal)
    if lift is not None:
        print(f"IK solution found with lift = {lift:.3f} m")
        print("Joint angles:", joints)
    else:
        print("No IK solution found within lift range")


if __name__ == '__main__':
    main()
