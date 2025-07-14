#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time
class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector_node')

        # 发布器
        self.publisher_ = self.create_publisher(String, '/control_signal', 10)

        # 初始化 Realsense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 加载YOLO模型
        self.model = YOLO("yolov8n.pt",verbose=False)

        self.get_logger().info("YOLO Realsense Node Started")

        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        # YOLO推理
        results = self.model(color_image, verbose=False)

        person_detections = []

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls != 0:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cx = max(0, min(cx, depth_frame.get_width()-1))
                cy = max(0, min(cy, depth_frame.get_height()-1))
                distance = depth_frame.get_distance(cx, cy)
                person_detections.append({
                    "box": (x1, y1, x2, y2),
                    "center": (cx, cy),
                    "distance": distance
                })

        if person_detections:
            # 最近的人
            person_detections.sort(key=lambda d: d["distance"])
            nearest = person_detections[0]
            dist = nearest['distance']
            self.get_logger().info(f"最近的人距离: {dist:.2f} m")
            for det in person_detections:
                x1, y1, x2, y2 = det["box"]
                dist_viz = det["distance"]
                color = (0,255,0)
                if det == nearest:
                    color = (0,0,255)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(color_image, f"{dist_viz:.2f}m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            if dist < 1.5:
                current_state = "stop"
            else:
                current_state = "start"
        else:
            current_state = "start"

        # 如果状态发生变化才发布
        if current_state != getattr(self, "prev_state", None):
            msg = String()
            msg.data = current_state
            self.publisher_.publish(msg)
            self.publisher_.publish(msg)
            self.get_logger().info(f"状态变化，发布 {current_state} 指令")
            self.prev_state = current_state

        # 可视化（需要就取消注释）
        cv2.imshow("YOLO Detection", color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
