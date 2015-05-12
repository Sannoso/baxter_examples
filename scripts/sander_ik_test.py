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
	

def main():
	"""
	Sanders IK test

	Uses 
	"""
	
	rospy.init_node("Sander_ik_test_node")
	time.sleep(5)
	return "\n IK test executed succesfully"

if __name__ == '__main__':
    sys.exit(main())



