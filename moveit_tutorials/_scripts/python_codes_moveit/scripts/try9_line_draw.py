#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def draw_line_in_moveit():
    # Initialize ROS node
    rospy.init_node('moveit_draw_line', anonymous=True)

    # Create a publisher to publish markers to RViz
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Create a marker representing a line
    line_marker = Marker()
    line_marker.header.frame_id = "panda_link0"  # Adjust the frame_id according to your setup
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "line"
    line_marker.id = 0
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.scale.x = 1.0  # Line width
    line_marker.color.a = 1.0
    line_marker.color.r = 1.0  # Red color

    # Define points for the line
    point1 = Point()
    point1.x = 0.5
    point1.y = 0.5
    point1.z = 0.5

    point2 = Point()
    point2.x = 1.5
    point2.y = 1.5
    point2.z = 1.5

    line_marker.points.append(point1)
    line_marker.points.append(point2)

    # Publish the marker to RViz
    marker_publisher.publish(line_marker)

    rospy.sleep(10)  # Keep the visualization for a few seconds before closing

if __name__ == '__main__':
    try:
        draw_line_in_moveit()
    except rospy.ROSInterruptException:
        pass

