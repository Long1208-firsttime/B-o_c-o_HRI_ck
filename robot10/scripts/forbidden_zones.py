#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PolygonStamped, Point32
from costmap_2d.srv import SetCostmap2D

def callback(poly):
    rospy.wait_for_service('/move_base/global_costmap/set_costmap')
    rospy.wait_for_service('/move_base/local_costmap/set_costmap')
    try:
        set_global = rospy.ServiceProxy('/move_base/global_costmap/set_costmap', SetCostmap2D)
        set_local = rospy.ServiceProxy('/move_base/local_costmap/set_costmap', SetCostmap2D)
        # Set cost 254 (lethal) cho polygon
        set_global(poly.polygon, 254, True)  # True để keep old costs
        set_local(poly.polygon, 254, True)
        rospy.loginfo("Added forbidden zone")
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('forbidden_zones_node')
    rospy.Subscriber('/forbidden_polygon', PolygonStamped, callback)
    rospy.spin()
