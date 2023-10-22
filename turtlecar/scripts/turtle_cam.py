#!/usr/bin/env python3


import rospy
from visualization_msgs.msg import Marker
from turtlesim_msgs.msg import Color as TurtleRGB

g_red = 1
g_green = 1
g_blue = 1

def ChangeColorVal(x) :
    return (float)(x / 255)


def TurtleColorCallback(data) :
    global g_red
    global g_green
    global g_blue

    g_red = data.r
    g_green = data.g
    g_blue = data.b


def main() :
    global g_red
    global g_green
    global g_blue

    rospy.init_node('turtle_camera', anonymous=True)
    loop_rate = rospy.Rate(30)
    marker_pub = rospy.Publisher('/turtle_floor', Marker, queue_size=10)
    rospy.Subscriber("/turtle1/color_sensor", TurtleRGB, TurtleColorCallback)


    turtle_floor = Marker()
    turtle_floor.id = 1
    turtle_floor.lifetime = rospy.Duration()
    turtle_floor.header.frame_id = "base"
    turtle_floor.header.stamp = rospy.Time.now()

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
    turtle_floor.scale.y = 10.
    turtle_floor.scale.z = 1
    
    
    while not rospy.is_shutdown():
        turtle_floor.color.a = 1.
        turtle_floor.color.r = ChangeColorVal(g_red)
        turtle_floor.color.g = ChangeColorVal(g_green)
        turtle_floor.color.b = ChangeColorVal(g_blue)

        marker_pub.publish(turtle_floor)
        
        print("==============================")
        print("r :", g_red, "\ng :", g_green, "\nb :", g_blue)
        
        loop_rate.sleep()
        
    



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass