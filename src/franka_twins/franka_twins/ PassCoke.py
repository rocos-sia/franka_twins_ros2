import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from argparse import ArgumentParser
from franky import Robot, JointMotion


class MotionController:
    """
    独立类，封装机械臂动作循环
    """
    def __init__(self, robot, init_joints, target_joints):
        self.robot = robot
        self.init_joints = init_joints
        self.target_joints = target_joints
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.motion_loop)
        self.thread.daemon = True  # 确保主程序退出时线程自动退出

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def motion_loop(self):
        while not self.stop_event.is_set():
            # 这里就是循环动作
            self.robot.move(JointMotion(self.target_joints))
            self.robot.move(JointMotion(self.init_joints))
class SignalSubscriber(Node):
    def __init__(self, motion_controller):
        super().__init__('signal_subscriber')
        self.motion_controller = motion_controller
        self.subscription = self.create_subscription(
            String,
            'control_signal',
            self.listener_callback,
            10)
        self.get_logger().info("Subscribed to 'control_signal' topic.")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received signal: "{msg.data}"')
        if msg.data.lower() == "stop":
            self.motion_controller.stop()
            self.get_logger().info("Motion stopped by signal.")
        elif msg.data.lower() == "start":
            if not self.motion_controller.thread.is_alive():
                self.motion_controller = MotionController(
                    self.motion_controller.robot,
                    self.motion_controller.init_joints,
                    self.motion_controller.target_joints
                )
                self.motion_controller.start()
                self.get_logger().info("Motion started by signal.")
def main():
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    args = parser.parse_args()

    robot = Robot(args.host)
    robot.recover_from_errors()
    robot.relative_dynamics_factor = 0.1

    InitJoints = [0.12915286, -0.1195091, 0.84458355, -2.10961463,
                  0.01453559, 2.05610148, 0.08659532]
    TargetJoints = [-0.13397944, -0.05805722, -0.06326429, -2.10118137,
                    0.01519375, 1.99435689, -0.37010038]

    motion_controller = MotionController(robot, InitJoints, TargetJoints)
    motion_controller.start()

    rclpy.init()
    node = SignalSubscriber(motion_controller)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        motion_controller.stop()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()