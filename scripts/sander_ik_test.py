#!/usr/bin/env python

"""
Sanders ik_comparison test python script
This script is written to be ran with the gazebo simulated Baxter
It will probably run on the robot live aswel, but is not tested on it
"""

#import various libraries
import argparse
import struct
import sys
import rospy

#import library to use time.sleep function
import time

#import necessary rosmessage parts
from geometry_msgs.msg import (
    	PoseStamped,
    	Pose,
    	Point,
    	Quaternion,
)
from std_msgs.msg import Header

#import necessary service messagestuff
#to build the request message for the IK service
from baxter_core_msgs.srv import (
    	SolvePositionIK,
    	SolvePositionIKRequest,
)


#def ik_test(beginpose, endpose):
def ik_test():
	rospy.init_node("Sander_ik_test_node")
 
	##preparing to call the IK service
	#store the name of the service in a variable for easier use
	servicename = "ExternalTools/right/PositionKinematicsNode/IKService"
	#create a rospy.serviceproxy to be able to call this service
	ikservice = rospy.ServiceProxy(servicename, SolvePositionIK)
	#create a blank requestmessage
	ikrequestmessage = SolvePositionIKRequest()
	#every request should have the correct timestamp:
                #the while loop is necessary because in simulator time rospy.time.now has
                #to be called in a short timespace after timepublication on /clock
       	now = rospy.Time.now()
       	count = 0
       	while(now.secs == 0):
             	now = rospy.Time.now()
               	count += 1
       	print('amount of rospy.Time.now() requests until non-zero output: ', count)

       	hdr = Header(stamp=now, frame_id='base')
	print(hdr)
	#oke the header is created

	#declaring all poses:
	poses = {
                'ik_example_pose': PoseStamped(
 	               header=hdr,
        	        pose=Pose(
                	        position=Point(
                        	        x=0.656982770038,
                                	y=-0.852598021641,
                                	z=0.0388609422173,
                        	),
                        	orientation=Quaternion(
                                	x=0.367048116303,
                                	y=0.885911751787,
                                	z=-0.108908281936,
                                	w=0.261868353356,
                        	),
                    	),
          	),
                'neutralpose': PoseStamped(
	                header=hdr,
	                pose=Pose(
	                        position=Point(
	                                x=0.573,
	                       		y=-0.181,
	                            	z=0.246,
	                        ),
	                        orientation=Quaternion(
	                            	x=-0.141,
	                            	y=0.990,
	                            	z=-0.012,
	                            	w=0.026,
	                        ),
	                ),
	  	),
	        'poseA': PoseStamped(
	                header=hdr,
       	        	pose=Pose(
	                        position=Point(
	                                x=0.1,
	                                y=0.51,
	                                z=0.723,
	                        ),
	                        orientation=Quaternion(
	                            	x=0,
	                            	y=1,
	                       		z=0,
	                            	w=0,
	                        ),              
			),
        	),
        #'triangledepositpose'
        #'squaredepositpose'
        #'circledepositpose'
        }
#	ikrequestmessage.pose_stamp.append(poses['poseA'])
	ikrequestmessage.pose_stamp.append(poses['ik_example_pose'])
#	print(ikrequestmessage)
	print(ikrequestmessage)

	#actually call the IK_service for motorpositions on the provided pose
        try:
                rospy.wait_for_service(servicename, 5.0)
                resp = ikservice(ikrequestmessage)
#                print('resp is')
#               	print(resp)
	#when a failure occurs, print the cause
        except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
	        return 1


        """
        revision the rest of this function
        """
	# Check if result valid, and type of seed ultimately used to get solution
    	# convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
                seed_str = {
                    ikrequestmessage.SEED_USER: 'User Provided Seed',
                    ikrequestmessage.SEED_CURRENT: 'Current Joint Angles',
                    ikrequestmessage.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
                print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                print "\nIK Joint Solution:\n", limb_joints
                print "------------------"
                print "Response Message:\n", resp
        else:
                print("INVALID POSE - No Valid Joint Solution Found.")

        return 0




def main():
	"""
	Sanders IK test

	Uses blabla
	"""
	
	#create and initialise a rosnode	
#	rospy.init_node("Sander_ik_test_node")
	#call the testroutine	
	ik_test()
	return "\n IK test executed succesfully"

if __name__ == '__main__':
    	sys.exit(main())

"""
THIS IS THE MESSAGE WE ARE FILLING IN TO DO AN IK REQUEST
baxter_core_msgs/SolvePositionIK
uint8 SEED_AUTO=0
uint8 SEED_USER=1
uint8 SEED_CURRENT=2
uint8 SEED_NS_MAP=3
geometry_msgs/PoseStamped[] pose_stamp
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
sensor_msgs/JointState[] seed_angles
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string[] name
  float64[] position
  float64[] velocity
  float64[] effort
uint8 seed_mode
"""



