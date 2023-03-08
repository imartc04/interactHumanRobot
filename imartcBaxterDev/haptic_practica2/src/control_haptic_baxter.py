#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class MyNode:
    def __init__(self):
        self.pub = rospy.Publisher('my_topic', String, queue_size=10)
        self.sub = rospy.Subscriber('/omniEthernet/pose', PoseStamped, self.callback)
        rospy.init_node('controlHapticBaxterImartc', anonymous=True)

        self.rate = rospy.Rate(10) # 10 Hz

        self.firstRead = True
        self.maxX = 0. 
        self.minX = 0. 
        self.maxY = 0. 
        self.minY = 0. 
        self.maxZ = 0. 
        self.minZ = 0. 
        

    def checkMaxAndMins(self, f_x, f_y, f_z):

        if self.firstRead == True:
            self.firstRead = False

            self.maxX = f_x 
            self.minX = f_x 
            self.maxY = f_y 
            self.minY = f_y
            self.maxZ = f_z
            self.minZ = f_z 

        else:
            self.maxX = max(self.maxX, f_x)
            self.minX = min(self.minX, f_x)
            self.maxY = max(self.maxY, f_y)
            self.minY = min(self.minY, f_y)
            self.maxZ = max(self.maxZ, f_z)
            self.minZ = min(self.minZ, f_z)
            
        print()
        print("Current max X:", self.maxX)
        print("Current min X:", self.minX)
        print("Current max Y:", self.maxY)
        print("Current min Y:", self.minY)
        print("Current max Z:", self.maxZ)
        print("Current min Z:", self.minZ)
        print()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data)

        #Pass haptic pose to baxter position of one hand
        l_x = data.pose.position.x
        l_y = data.pose.position.y
        l_z = data.pose.position.z

        self.checkMaxAndMins(l_x, l_y, l_z)

        
        

    def run(self):
        while not rospy.is_shutdown():
            msg = 'Hello ROS!'
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    node = MyNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
