#!/usr/bin/env python

"""
Sanders ik_comparison test python script
"""

#import various libraries
import argparse
import struct
import sys
import rospy

#import necessary rosmessage parts
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

#import necessary service messagestuff
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


def main():
	"""
	Sanders IK test

	Uses 
	"""

	print "\n 42.43 seconds \n"
	return 0

if __name__ == '__main__':
    sys.exit(main())



