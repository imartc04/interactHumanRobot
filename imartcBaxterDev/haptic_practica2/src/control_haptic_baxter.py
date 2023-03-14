#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import baxter_interface
import sys

class MyNode:
    def __init__(self):
        #self.pub = rospy.Publisher('my_topic', String, queue_size=10)
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
        
    
    def callbackMinMaxHaptic(self, data):

        """
        From 2023 haptic bags we get the next results
        Current max X: 0.23378638975162608
        Current min X: -0.2481581755214125
        Current max Y: 0.28992345652796514
        Current min Y: 0.016984542940453998
        Current max Z: 0.3258663271132756
        Current min Z: -0.039550315230442584
        
        """


        #Pass haptic pose to baxter position of one hand
        l_x = data.pose.position.x
        l_y = data.pose.position.y
        l_z = data.pose.position.z

        self.checkMaxAndMins(l_x, l_y, l_z)



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


    def linearInterpolation(self,f_minIn, f_maxIn, f_minOut, f_maxOut, f_in):
        return (f_maxOut - f_minOut)/(f_maxIn-f_minIn) * (f_in - f_minIn) + f_minOut


    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data)

        #Pass haptic pose to baxter position of one hand
        l_x = data.pose.position.x
        l_y = data.pose.position.y
        l_z = data.pose.position.z


        """
        Assign new values to joints

        Haptic axis : Joint
            X : S0
            Y : S1
            z : W2 

        """

        limb = baxter_interface.Limb('right')
        angles = limb.joint_angles()

        angles['right_s0']= self.linearInterpolation(-0.248,0.234,-1.7, 1.7, l_x)
        angles['right_s1']= self.linearInterpolation(0.017,0.290,-2.1, 1.1, l_y)
        angles['right_e0']=0.0
        angles['right_e1']=0.0
        angles['right_w0']=0.0
        angles['right_w1']=0.0
        angles['right_w2']= self.linearInterpolation(-0.04,0.326,-3.1, 3.1, l_z)

        # move the right arm to those joint angles
        print("Moving to new angles ", angles)
        limb.move_to_joint_positions(angles)

        
    def testMoveAngles(self, f_num):
        """
        Assign new values to joints

        Haptic axis : Joint
            X : S0
            Y : S1
            z : W2 

        """
        limb = baxter_interface.Limb('right')
        angles = limb.joint_angles()

        """
        Values detected with baxter simulator
        Joint:Max:Min
        S0:1.7:-1.7  Max value arm point to front
        S1:1.1:-2.1 Max value arm point down
        E0:3.1:-3.1  Max value rotation clockwise with axis where the arm is pointin to when it is straight 
        E1:2.7:-0.1 Max value arm straight point to body
        W0:3.1:-3.1 Max value rotation clockwise with axis where the arm is pointin to when it is straight 
        W1:2.2:-1.7 Max value arm point down
        W2:3.1:-3.1 Max value rotation clockwise with axis where the arm is pointin to when it is straight 
        """

        angles['right_s0']=0.0
        angles['right_s1']=0.0
        angles['right_e0']=0.0
        angles['right_e1']=0.0
        angles['right_w0']=0.0
        angles['right_w1']=0.0
        angles['right_w2']=f_num

        # print the joint angle command
        print(angles)

        # move the right arm to those joint angles
        print("Moving to new angles...")
        limb.move_to_joint_positions(angles)
        

    def run(self):
        while not rospy.is_shutdown():
            #self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    
    
    node = MyNode()
    #node.testMoveAngles(float(sys.argv[1]))
    try:
        
        node.run()
    except rospy.ROSInterruptException:
        pass
