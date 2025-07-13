import time
import math
from franky import Robot, JointMotion

robot = Robot("172.16.0.2")
robot.recover_from_errors()
robot.relative_dynamics_factor = 0.1
robot.recover_from_errors()
# 取当前位置
q_start = robot.current_joint_positions
print("Current joint positions:", q_start)
robot.relative_dynamics_factor = 0.1
state = robot.state
print("\nPose: ", robot.current_pose)
print("O_TT_E: ", state.O_T_EE)
print("Joints: ", state.q)
print("Elbow: ", state.elbow)

# 轨迹参数
amplitude = 0.2
frequency = 0.1

# 采样周期
dt = 0.001

# 控制时长
duration = 10.0
steps = int(duration / dt)

def generator():
    t0 = time.time()
    while True:
        t = time.time() - t0
        q_desired = [
            q_start[i] + amplitude * math.sin(2 * math.pi * frequency * t)
            for i in range(len(q_start))
        ]
        yield JointMotion(q_desired)

# 使用 join_motion 开始流式控制
robot.join_motion(generator(), control_rate=1/dt)
