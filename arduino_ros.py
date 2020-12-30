import rospy
from geometry_msgs.msg import Twist
import tf

pub = rospy.Publisher('rpm_vel', Twist, queue_size=10)

rospy.init_node('cmd_pub', anonymous=True)
br = tf.TransformBroadcaster()


rate = rospy.Rate(15) # 10hz
inc = 0
direc = 1

'''
while not rospy.is_shutdown():
    a= Twist()
    a.linear.x = inc*100;
    a.linear.y = inc*100;
    inc+=direc
    if(inc>110):
        direc = -1
    elif(inc<=-110):
        direc = 1
    #rospy.logwarn(inc)
    pub.publish(a)
    rate.sleep()
'''
while not rospy.is_shutdown():
    a= Twist()
    a.angular.z= inc/180.*3.1415926535
    rospy.logwarn(inc)
    inc+=direc
    if(inc>=25):
        direc = -1
    elif(inc<=-25):
        direc = 1
    pub.publish(a)
    rate.sleep()
