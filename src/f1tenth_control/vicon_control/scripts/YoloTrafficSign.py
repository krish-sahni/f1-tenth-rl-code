#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloTrafficSignNode:
    def __init__(self):
        rospy.init_node('yolo_traffic_sign_node')

        # Load your YOLO model (edit path if needed)
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = rospy.Publisher('/yolo/annotated_image', RosImage, queue_size=1)
        self.sign_pub = rospy.Publisher('/yolo/traffic_sign', String, queue_size=1)

        # Subscriber
        rospy.Subscriber('/D435I/color/image_raw', RosImage, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo("YOLO Traffic Sign Node initialized.")
        rospy.spin()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(source=frame, device=0, imgsz=416, verbose=False)

            result = results[0]
            boxes = result.boxes

            if boxes.cls.numel() > 0:
                cls_ids = boxes.cls.cpu().numpy()
                confidences = boxes.conf.cpu().numpy()

                for cls_id, conf in zip(cls_ids, confidences):
                    label = result.names[int(cls_id)]

                    if label in ['stop sign', 'traffic light']:  # Add more if needed
                        rospy.loginfo(f"Detected {label} with confidence {conf:.2f}")
                        self.sign_pub.publish(label)

            # Publish annotated image to RViz
            annotated_frame = result.plot()
            ros_img = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ros_img.header = msg.header  # preserve time/frame_id
            self.image_pub.publish(ros_img)

        except Exception as e:
            rospy.logerr(f"Error in YOLO callback: {e}")

if __name__ == '__main__':
    try:
        YoloTrafficSignNode()
    except rospy.ROSInterruptException:
        pass
