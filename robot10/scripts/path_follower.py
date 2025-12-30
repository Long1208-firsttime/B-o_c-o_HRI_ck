#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Path
import tf.transformations
import math

class VisualPathFollower:
    def __init__(self):
        rospy.init_node('visual_path_follower')
        
        self.path = []
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Nhận điểm người dùng click trong RViz
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_callback)
        
        self.current_idx = 0
        self.tolerance = 0.2
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        
        rospy.Subscriber('/odom', rospy.AnyMsg, self.odom_callback)  # sẽ lấy pose từ odom
        
    def clicked_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.path.append((x, y))
        rospy.loginfo(f"Added waypoint: ({x:.2f}, {y:.2f}) | Total: {len(self.path)}")
        self.publish_path()
        
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = rospy.Time.now()
        
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        
    def odom_callback(self, msg):
        # Lấy pose hiện tại (cách đơn giản)
        import json
        from nav_msgs.msg import Odometry
        if msg._type == 'nav_msgs/Odometry':
            odom = Odometry().deserialize(msg._buff)
            self.current_x = odom.pose.pose.position.x
            self.current_y = odom.pose.pose.position.y
            q = odom.pose.pose.orientation
            self.current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.follow_path()
            
    def follow_path(self):
        if len(self.path) == 0 or self.current_idx >= len(self.path):
            return
            
        target_x, target_y = self.path[self.current_idx]
        
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.current_yaw
        angle_error = (angle_error + math.pi) % (2*math.pi) - math.pi
        
        twist = Twist()
        
        if dist < self.tolerance:
            self.current_idx += 1
            if self.current_idx >= len(self.path):
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_pub.publish(twist)
                rospy.loginfo("Đã hoàn thành đường đi!")
                return
        else:
            twist.linear.x = min(self.linear_speed, dist * 1.2)
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_error * 2))
            
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    follower = VisualPathFollower()
    rospy.spin()
