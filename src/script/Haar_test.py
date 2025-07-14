import pyrealsense2 as rs
import numpy as np
import cv2

# 创建pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动
pipeline.start(config)

# 加载OpenCV人脸分类器
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

try:
    while True:
        # 等待一帧
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 转numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 灰度化做人脸检测
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        # 遍历检测到的人脸
        for (x, y, w, h) in faces:
            # 在彩色图上画框
            cv2.rectangle(color_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            # 人脸中心点
            cx = x + w // 2
            cy = y + h // 2
            # 获取深度
            distance = depth_frame.get_distance(cx, cy)
            # 在画面上显示距离
            cv2.putText(color_image, f"{distance:.2f} m", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
            print(f"检测到人脸，距离: {distance:.2f} 米")

        # 显示画面
        cv2.imshow('RealSense Color Image', color_image)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()