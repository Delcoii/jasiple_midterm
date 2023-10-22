#!/usr/bin/env python3


import rospy

from visualization_msgs.msg import Marker



def main() :

    rospy.init_node('turtle_camera', anonymous=True)
    loop_rate = rospy.Rate(60.)
    marker_pub = rospy.Publisher('turtle_floor', Marker, queue_size=10)

    turtle_floor = Marker()
    turtle_floor.id = 1
    turtle_floor.lifetime = rospy.Duration()
    turtle_floor.header.frame_id = "base"

    turtle_floor.type = turtle_floor.CUBE
    turtle_floor.action = turtle_floor.ADD
    turtle_floor.pose.position.x = 0.
    turtle_floor.pose.position.y = 0.
    turtle_floor.pose.position.z = 0.
    turtle_floor.pose.orientation.x = 0.
    turtle_floor.pose.orientation.y = 0.
    turtle_floor.pose.orientation.z = 0.
    turtle_floor.pose.orientation.w = 1.
    turtle_floor.scale.x = 10.
    turtle_floor.scale.y = 10
    turtle_floor.scale.z = 0.1
    turtle_floor.color.a = 1.
    

    while not rospy.is_shutdown():
        
        turtle_floor.color.r = 1.
        turtle_floor.color.g = 1.
        turtle_floor.color.b = 1.

        

        marker_pub.publish(turtle_floor)
        
        rospy.loginfo("aaa")
        
        loop_rate.sleep()
    





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass