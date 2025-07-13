import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from argparse import ArgumentParser
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

Current_step = 0
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
        self.stop_event.clear()
        
        self.thread.start()

    def stop(self):
        self.robot.stop()
        self.stop_event.set()
    def step0(self):
        print("Step 0: Move to init position")
        self.robot.move(JointMotion(self.target_joints))
    def step1(self):
        print("Step 1: Move to target position")
        self.robot.move(JointMotion(self.init_joints))
    
        
    def motion_loop(self):
        """
        按顺序执行步骤，如果 stop_event 被设置则中断
        """
        global Current_step
        print(f"Starting motion loop from step  " ,Current_step)
        steps = [self.step0, self.step1]
        while not self.stop_event.is_set():
            i = Current_step  # 从上次继续
            print(f"Starting motion loop from step  " ,Current_step)
            print(f"Total steps: {len(steps)}")
            while i < len(steps):
                # 每次循环先更新 current_step，表示“正在执行第i步”
                Current_step = i

                if self.stop_event.is_set():
                    print(f"Stopped before executing step {i}")
                    return

                print(f"Executing step {i}")
                try:
                    print(f"Executing step {i}: {steps[i].__name__}")
                    steps[i]()
                except Exception as e:
                    print(f"Motion interrupted or failed during step {i}: {e}")
                    # current_step 保持不变，下次从这一步继续
                    return

                # 成功执行完一步，推进到下一步
                i += 1

        # 所有步骤完成，重置
            Current_step = 0
            print("All steps completed.")
        # for i, step in enumerate(steps):
        #     if self.stop_event.is_set():
        #         print(f"Motion stopped before step {i}.")
        #         break
        #     print(f"Executing step {i}")
        #     step()
        # while not self.stop_event.is_set():
        #     for i in range(Current_step, len(steps)):
        #         if self.stop_event.is_set():
        #             print(f"Stopped at step {i}")
        #             Current_step = i
        #             return
        #         print(f"Executing step {i}")
        #         try:
        #             steps[i]()
        #         except Exception as e:
        #             print(f"Motion interrupted or failed: {e}")
        #             return  # 或者 break，结束线程
        #     Current_step = 0
        #     print("All steps completed.")

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