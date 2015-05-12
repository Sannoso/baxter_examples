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


