#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Topic mà plugin gazebo_ros_p3d đang lắng nghe
# (bạn đã đặt trong URDF là cmd_vel_flying hoặc cmd_pose, mình dùng cmd_vel_flying)
CMD_VEL_TOPIC = '/cmd_vel_flying'

class WASDFlyingControl:
    def __init__(self):
        rospy.init_node('wasd_flying_control', anonymous=True)
        self.pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        
        # Tốc độ mặc định (có thể chỉnh thoải mái)
        self.linear_speed = 1.5   # m/s (trôi tiến/lùi/trái/phải)
        self.angular_speed = 1.5  # rad/s (quay trái/phải)
        self.up_down_speed = 0.5  # m/s (bay lên/xuống nếu muốn)

        self.twist = Twist()
        self.settings = termios.tcgetattr(sys.stdin)

        rospy.loginfo("Điều khiển robot trôi tự do bằng WASD")
        rospy.loginfo("W/S   : Tiến / Lùi")
        rospy.loginfo("A/D   : Trái / Phải (trôi ngang)")
        rospy.loginfo("Q/E   : Quay trái / Quay phải")
        rospy.loginfo("Space : Dừng khẩn cấp")
        rospy.loginfo("R/F   : Bay lên / Hạ xuống (nếu muốn)")
        rospy.loginfo("Ctrl+C: Thoát")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key.lower()

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                # Reset vận tốc về 0 mỗi vòng
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.linear.z = 0.0
                self.twist.angular.x = 0.0
                self.twist.angular.y = 0.0
                self.twist.angular.z = 0.0

                if key == 'w':
                    self.twist.linear.x = self.linear_speed
                elif key == 's':
                    self.twist.linear.x = -self.linear_speed
                elif key == 'a':
                    self.twist.linear.y = self.linear_speed
                elif key == 'd':
                    self.twist.linear.y = -self.linear_speed
                elif key == 'q':
                    self.twist.angular.z = self.angular_speed
                elif key == 'e':
                    self.twist.angular.z = -self.angular_speed
                elif key == 'r':
                    self.twist.linear.z = self.up_down_speed   # Bay lên
                elif key == 'f':
                    self.twist.linear.z = -self.up_down_speed  # Hạ xuống
                elif key == ' ' or key == '\x03':  # Space hoặc Ctrl+C
                    self.twist = Twist()  # Dừng hoàn toàn

                # Publish lệnh
                self.pub.publish(self.twist)

        except Exception as e:
            rospy.logerr(e)
        finally:
            # Dừng robot khi thoát
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    rospy.init_node('wasd_flying_control', anonymous=True)
    controller = WASDFlyingControl()
    controller.run()
