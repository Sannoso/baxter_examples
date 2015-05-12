#!/usr/bin/env python

"""
Sanders ik_comparison test python script
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
       servicename = "ExternalTools/right/PositionKinemticsNode/IKService"
       #wait for the service to be available. startup_time or in use by something else
#      rospy.wait_for_service(servicename)#what is the name of IKService? does this work?
       #create a rospy.serviceproxy to be able to call this service
       ikservice = rospy.ServiceProxy(servicename, SolvePositionIK)
       ikrequestmessage = SolvePositionIKRequest()
       print('ikrequestemessage is: ', ikrequestmessage)
       print(ikrequestmessage)
	#every request should have the correct timestamp:
        #I'm making the header in a different function to ensure a correct timestamp
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
#	poses = {
	#continue with adding the endPose!


	return 1


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



