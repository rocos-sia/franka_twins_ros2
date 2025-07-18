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
from tosor_msgs.msg import GripperDistance
import time
Current_step = 0
class MotionController:
    """
    独立类，封装机械臂动作循环
    """
    def __init__(self, left_robot, right_robot,left_InitJoints,right_InitJoints, left_TargetJoints):
        self.left_robot = left_robot
        self.right_robot = right_robot
        self.right_InitJoints = right_InitJoints
        self.left_TargetJoints = left_TargetJoints
        self.left_InitJoints=left_InitJoints
        self.people_joints=[2.29221794 , 0.66549244, -0.34744742 ,-1.78037702,  0.12384098 , 2.65005514,0.4510016]
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.motion_loop)
        self.thread.daemon = True  # 确保主程序退出时线程自动退出
        

        self.gripper_node = rclpy.create_node("gripper_publisher")
        self.left_gripper_pub = self.gripper_node.create_publisher(
            GripperDistance,
            "/left/gripper/command",
            10
        )
        self.right_gripper_pub = self.gripper_node.create_publisher(
            GripperDistance,
            "/right/gripper/command",
            10)
        
        
        time.sleep(3)
    def _left_gripperCommand(self, left_distance):
        msg = GripperDistance()
        msg.distance = left_distance
        self.left_gripper_pub.publish(msg)
        self.left_gripper_pub.publish(msg)
        print(f"Left gripper command: {left_distance}")

    def _right_gripperCommand(self, right_distance):
        msg = GripperDistance()
        msg.distance = right_distance
        self.right_gripper_pub.publish(msg)
        self.left_gripper_pub.publish(msg)
        print(f"Right gripper command: {right_distance}")
    def start(self):
        self.stop_event.clear()
        
        self.thread.start()

    def stop(self):
        self.left_robot.stop()
        self.right_robot.stop()
        self.stop_event.set()
        time.sleep(0.2)
    def step0(self):
        print("Step 0: Move to init position")
        self._right_gripperCommand(0)  # 
        self._left_gripperCommand(1000)
        self.left_robot.move(JointMotion(self.left_InitJoints),asynchronous=True)
        
        self.right_robot.move(JointMotion(self.right_InitJoints))
        time.sleep(2)
    def step1(self):
        print("Step 1: Move to target position")
        self.left_robot.move(JointMotion(self.left_TargetJoints))
    def step2(self):
        print("Step 2 : Move to cola")
        target = Affine([0.0, 0.0, 0.03])
        motion_forward = CartesianMotion(target, reference_type=ReferenceType.Relative)
        self.left_robot.move(motion_forward)
        self._left_gripperCommand(400)
        time.sleep(1)
    def step3(self):
        print("Step 3 : Move to people")
        self.left_robot.stop()
        time.sleep(1)
        self.left_robot.move(JointMotion(self.people_joints))
        time.sleep(5)
        self._left_gripperCommand(700)
    
        
    def motion_loop(self):
        """
        按顺序执行步骤，如果 stop_event 被设置则中断
        """
        global Current_step
        print(f"Starting motion loop from step  " ,Current_step)
        steps = [self.step0, self.step1,self.step2,self.step3]
        
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
        # Current_step = 0
        print("All steps completed.")
        self.left_robot.stop()
        self.right_robot.stop()
        return

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
                    self.motion_controller.left_robot,
                    self.motion_controller.right_robot,
                    self.motion_controller.left_InitJoints,
                    self.motion_controller.right_InitJoints,
                    self.motion_controller.left_TargetJoints
                )
                self.motion_controller.start()
                self.get_logger().info("Motion started by signal.")
        else:
            pass  # 可以添加其他信号处理逻辑
def main():
    # parser = ArgumentParser()
    # parser.add_argument("--host", default="172.16.0.2", help="FCI IP of the robot")
    # args = parser.parse_args()

    left_robot = Robot("172.16.0.2")
    right_robot = Robot("172.16.1.2")
    left_robot.recover_from_errors()
    left_robot.relative_dynamics_factor = 0.08
    right_robot.recover_from_errors()
    right_robot.relative_dynamics_factor = 0.08

    right_InitJoints = [-1.367792,    0.01289397, -0.34383398 ,-2.65521047,  0.01256896 , 2.7578551,
 -0.73218845]
    left_TargetJoints =  [ 2.42424019 , 0.71202427, -0.19571876, -1.38322015,  0.12408109 , 2.10609979,
  0.56901092]
    left_InitJoints =  [ 1.57316892, -0.11782028 , 0.10805546, -2.70657421,  0.14477853,  2.674246,
 -1.01090839]
    
    rclpy.init()
    motion_controller = MotionController(left_robot, right_robot,left_InitJoints,right_InitJoints, left_TargetJoints)
    

    
    motion_controller.start()
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