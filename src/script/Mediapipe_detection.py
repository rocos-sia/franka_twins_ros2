import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp

# 初始化 Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 初始化 Mediapipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        results = pose.process(rgb_image)

        if results.pose_landmarks:
            # 获取骨盆中心 (通常用腰部中点)
            lm = results.pose_landmarks.landmark
            pelvis = lm[mp_pose.PoseLandmark.LEFT_HIP]
            x_px = int(pelvis.x * color_image.shape[1])
            y_px = int(pelvis.y * color_image.shape[0])
            # 限制坐标在合法范围内
            x_px = max(0, min(x_px, color_image.shape[1] - 1))
            y_px = max(0, min(y_px, color_image.shape[0] - 1))

            distance = depth_frame.get_distance(x_px, y_px)

            # 显示
            cv2.circle(color_image, (x_px, y_px), 6, (0,255,0), -1)
            cv2.putText(color_image, f"{distance:.2f} m", (x_px, y_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
            print(f"检测到人体，距离: {distance:.2f} 米")

            # 画骨骼
            mp.solutions.drawing_utils.draw_landmarks(
                color_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        cv2.imshow('Pose Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
