#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Int32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO  # YOLOv8 모델 로드
import warnings

warnings.filterwarnings("ignore", category=UserWarning, message="Unable to import Axes3D")

class ROS2VideoYOLONode(Node):
    def __init__(self):
        super().__init__('ros2_video_yolo_node')
        
        # ROS2 Publishers for detection results
        self.bbox_pub = self.create_publisher(Float32MultiArray, '/BBox', 10)
        self.class_id_pub = self.create_publisher(Int32, '/class_id', 10)
        self.class_name_pub = self.create_publisher(String, '/class_name', 10)
        self.score_pub = self.create_publisher(Float32, '/score', 10)
        
        # 이미지 구독자 생성
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            10)
        self.subscription  # 방지: linters가 사용되지 않는 변수를 경고하는 것 방지

        self.bridge = CvBridge()
        
        # CUDA 가용 여부 확인 후 모델 로드
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {device}")
        self.model = YOLO('weights/YOLO_0216.pt').to(device)
    
    def image_callback(self, msg):
        try:
            # "passthrough"로 변환하여 원본 포맷 유지
            frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

            # RGB 형식이면 BGR로 변환 (OpenCV 호환성 유지)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV: {e}")
            return
        
        try:
            # CUDA 디바이스로 변환하여 YOLO 실행
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            results = self.model(frame, device=device)

            for box in results[0].boxes:
                conf = box.conf[0]
                if conf >= 0.7:  # Confidence threshold for drawing the bounding box
                    x1, y1, x2, y2 = [float(coord) for coord in box.xyxy[0]]
                    cls = int(box.cls[0])
                    label = results[0].names[cls]

                    # Draw the bounding box and label
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), int(y1) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Publish detection results
                    bbox_msg = Float32MultiArray(data=[x1, y1, x2, y2])
                    self.bbox_pub.publish(bbox_msg)

                    class_id_msg = Int32(data=cls)
                    self.class_id_pub.publish(class_id_msg)

                    class_name_msg = String(data=label)
                    self.class_name_pub.publish(class_name_msg)

                    score_msg = Float32(data=float(conf))
                    self.score_pub.publish(score_msg)
        except Exception as e:
            self.get_logger().error(f"Error during YOLO detection: {e}")
            return
        
        # 결과 이미지 출력
        cv2.imshow("YOLOv8 Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            self.get_logger().info("Closing display window.")


def main(args=None):
    rclpy.init(args=args)
    node = ROS2VideoYOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.get_logger().info("Shutting down node.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
