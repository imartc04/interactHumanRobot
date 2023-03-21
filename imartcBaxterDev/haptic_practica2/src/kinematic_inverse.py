#!/usr/bin/python3
"""
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys
from geometry_msgs.msg import PoseStamped
import baxter_interface

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


def linearInterpolation(f_minIn, f_maxIn, f_minOut, f_maxOut, f_in):
    return (f_maxOut - f_minOut)/(f_maxIn-f_minIn) * (f_in - f_minIn) + f_minOut


class MyNode:
    def __init__(self):
        #self.pub = rospy.Publisher('my_topic', String, queue_size=10)
        self.sub = rospy.Subscriber('/omniEthernet/pose', PoseStamped, self.callback)
        rospy.init_node('controlBaxterInvKinematicImartc', anonymous=True)
        self.rate = rospy.Rate(10) # 10 Hz


    def run(self):
        while not rospy.is_shutdown():
            #self.pub.publish(msg)
            self.rate.sleep()


    def callback(self, data):


        #Obtain haptic position
        l_x = data.pose.position.x
        l_y = data.pose.position.y
        l_z = data.pose.position.z

        limb = "right"

        """
        Map hatpic space to baxter space 
        
        From 2023 haptic bags we get the next results for max/min values
        Current max X: 0.23378638975162608
        Current min X: -0.2481581755214125
        Current max Y: 0.28992345652796514
        Current min Y: 0.016984542940453998
        Current max Z: 0.3258663271132756
        Current min Z: -0.039550315230442584

        Robot max:mins 





        """
        l_x = linearInterpolation( -0.24, 0.24, -1, 1, l_x)
        l_y = linearInterpolation( -0.24, 0.24, -1, 1, l_y)
        l_z = linearInterpolation( -0.24, 0.24, -0.5, 0.5, l_z)

        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=l_x,
                        y=l_y,
                        z=l_z,
                    ),
                    orientation=Quaternion(
                        x=-0.366894936773,
                        y=0.885980397775,
                        z=0.108155782462,
                        w=0.262162481772,
                    ),
                ),
            ),
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=l_x,
                        y=l_y,
                        z=l_z,
                    ),
                    orientation=Quaternion(
                        x=0.367048116303,
                        y=0.885911751787,
                        z=-0.108908281936,
                        w=0.261868353356,
                    ),
                ),
            ),
        }

        ikreq.pose_stamp.append(poses[limb])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            #Obtain limb interface and set obtained
            limbIf = baxter_interface.Limb(limb)
            limbIf.move_to_joint_positions(limb_joints)


            print ("\nIK Joint Solution:\n", limb_joints)
            print ("------------------")
            print ("Response Message:\n", resp)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
    

if __name__ == '__main__':
    node = MyNode()
    #node.testMoveAngles(float(sys.argv[1]))
    try:
        
        node.run()
    except rospy.ROSInterruptException:
        pass