#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os

class HybridRobotGreeter:
    def __init__(self):
        rospy.init_node("hybrid_greeter")
        
        # 1. Cập nhật đường dẫn weight (Sửa 'hieu' nếu tên user của bạn khác)
        path_to_weights = os.path.expanduser("~/catkin_ws/src/robot10/weights/best.pt")
        self.model = YOLO(path_to_weights)
        
        self.bridge = CvBridge()
        
        # Biến lưu khoảng cách từ Laser
        self.current_laser_dist = float('inf')
        self.has_greeted = False

        # 2. Subscriber: Đảm bảo /laser/scan khớp với file launch của bạn
        rospy.Subscriber("/laser/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        rospy.loginfo("🚀 Robot kết hợp Laser + Camera + Bounding Box đã sẵn sàng!")

    def laser_callback(self, msg):
        # Lấy khoảng cách ở góc chính giữa phía trước robot
        if len(msg.ranges) > 0:
            mid_index = len(msg.ranges) // 2
            dist = msg.ranges[mid_index]
            # Loại bỏ các giá trị nhiễu hoặc vô hạn
            if dist < msg.range_max and dist > msg.range_min:
                self.current_laser_dist = dist

    def image_callback(self, msg):
        # Chuyển đổi từ ROS Image sang OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Chạy YOLO nhận diện
        results = self.model(frame, conf=0.6, verbose=False)
        
        # --- VẼ BOUNDING BOX TỰ ĐỘNG ---
        # results[0].plot() sẽ trả về một ảnh đã có sẵn khung và nhãn
        annotated_frame = results[0].plot()
        
        found_person = False
        for box in results[0].boxes:
            label = self.model.names[int(box.cls[0])]
            
            if label == "person":
                found_person = True
                dist = self.current_laser_dist
                
                # Logic chào
                if 1.0 < dist < 2.5:
                    if not self.has_greeted:
                        rospy.loginfo(f"👋 Chào bạn! Tôi thấy bạn cách {dist:.2f}m")
                        self.has_greeted = True
                elif dist > 3.0:
                    self.has_greeted = False

        if not found_person:
            self.has_greeted = False

        # Hiển thị ảnh đã có Bounding Box
        cv2.imshow("Robot Vision - YOLO v8", annotated_frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        node = HybridRobotGreeter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()