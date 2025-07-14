import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# 初始化 Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 加载YOLOv8人检测模型 (COCO pretrained)
model = YOLO("yolov8n.pt")  # 可换成 yolov8s.pt / yolov8m.pt / yolov8l.pt

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # YOLO推理
        results = model(color_image)

        # 记录所有人和距离
        person_detections = []

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls != 0:
                    continue  # 只要人 (COCO class 0)
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 中心点
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # 安全裁剪坐标
                cx = max(0, min(cx, depth_frame.get_width()-1))
                cy = max(0, min(cy, depth_frame.get_height()-1))

                distance = depth_frame.get_distance(cx, cy)
                person_detections.append({
                    "box": (x1, y1, x2, y2),
                    "center": (cx, cy),
                    "distance": distance
                })

        # 如果有人
        if person_detections:
            # 按距离排序
            person_detections.sort(key=lambda d: d["distance"])
            nearest = person_detections[0]
            for det in person_detections:
                x1, y1, x2, y2 = det["box"]
                dist = det["distance"]
                color = (0,255,0)
                if det==nearest:
                    color=(0,0,255)
                cv2.rectangle(color_image, (x1,y1), (x2,y2), color, 2)
                cv2.putText(color_image, f"{dist:.2f}m", (x1,y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            # print(f"最近的人距离: {nearest['distance']:.2f} 米")

        cv2.imshow("YOLOv8 Human Detection", color_image)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
