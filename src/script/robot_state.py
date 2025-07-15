from franky import (
    Affine,
    JointMotion,
    Measure,
    Reaction,
    Robot,
    CartesianStopMotion,
    CartesianMotion,
    RobotPose,
    RobotState,
    ReferenceType,
)
from argparse import ArgumentParser
def main():
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    args = parser.parse_args()

    left_robot = Robot(args.host)
    right_robot = Robot("172.16.1.2")
    left_robot.recover_from_errors()
    left_robot.relative_dynamics_factor = 0.1
    right_robot.recover_from_errors()
    right_robot.relative_dynamics_factor = 0.1
    left_state = left_robot.state
    print("\nPose: ", left_robot.current_pose)
    print("O_TT_E: ", left_state.O_T_EE)
    print("left_state Joints: ", left_state.q)
    print("Elbow: ", left_state.elbow)
    right_state = right_robot.state
    print("\nPose: ", right_robot.current_pose)
    print("O_TT_E: ", right_state.O_T_EE)
    print("right_state Joints: ", right_state.q)
    print("Elbow: ", right_state.elbow)

if __name__ == '__main__':
    main()