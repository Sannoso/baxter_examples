#!/usr/bin/env python

"""
Active Robots, Baxter Training Example
"""

import rospy
import math
import roslib
#orig lineroslib.load_manifest('training')    #we load the manifest of the package we are in
roslib.load_manifest('baxter_examples')
import baxter_interface
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

from moveit_commander import conversions
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





#callback function for camera subscriber, called by the camera subscriber for every frame.
def callback(data):

    #Republish the camera stream to the screen
    rospy.Publisher('/robot/xdisplay',Image, queue_size=1).publish(data)

    #Convert incoming image from a ROS image message to a CV image that open CV can process.
    cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    #Display the converted cv image, this is the raw camera feed data.
    cv2.imshow("Raw Camera Feed", cv_image)

    #Create an empty image variable, the same dimensions as our camera feed.
    gray = numpy.zeros((cv_image.shape), numpy.uint8)
    #Into this previously created empty image variable, place the grayscale conversion of our camera feed.
    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    #Display the grayscale image.
    cv2.imshow("Grayscale Conversion", gray)

    #Create another empty image variable.
    canny = numpy.zeros((cv_image.shape), numpy.uint8)
    #Fill the new image variable with a canny edge detection map of the greyscale image created earlier.
    canny = cv2.Canny(gray, 50, 150, 3)
    #Display the canny mapping.
    cv2.imshow("Canny Edge Detection", canny)

    #3ms wait
    cv2.waitKey(3)


def ik_solver_request(input_limb, input_pose):
    print "IK solver request:"

    #input error checking
    if len(input_pose) == 6:
        quaternion_pose = conversions.list_to_pose_stamped(input_pose, "base")
    elif len(input_pose) == 7:
        quaternion_pose = input_pose
    else:
        print """Invalid Pose List:
    Input Pose to function: ik_solver_request must be a list of:
    6 elements for an RPY pose, or
    7 elements for a Quaternion pose"""
        return

    if input_limb == "right" or input_limb == "left":
        limb = input_limb
    else:
        print """Invalid Limb:
    Input Limb to function: ik_solver_request must be a string:
    'right' or 'left'"""
        return

    #request/response handling
    node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(quaternion_pose)
    try:
        rospy.wait_for_service(node, 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" % (error_message,))
    if (ik_response.isValid[0]):
        print("PASS: Valid joint configuration found")
        #convert response to JP control dict
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        return limb_joints
    else:
        print("FAILED: No valid joint configuration for this pose found")





if __name__ == '__main__':
    rospy.init_node('activerobots_baxter_training_example', anonymous=True)

#create subscriber to the right hand camera, each frame recieved calls the callback function
camera_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,callback)

#Reading Sensors
#read joint angles
print baxter_interface.Limb('right').joint_angles()
#read button state
print baxter_interface.digital_io.DigitalIO('right_lower_button').state
#read IR rangefinder
print baxter_interface.analog_io.AnalogIO('right_hand_range').state()


#Writing to actuators
#define pi so we can use radians easily
pi = math.pi
#Enable the actuators
baxter_interface.RobotEnable().enable()

#send joint positions in radians
#origline rosbaxter_interface.Limb('right').move_to_joint_positions({
baxter_interface.Limb('right').move_to_joint_positions({
                                                        'right_s0': 0.0, 
                                                        'right_s1': 0.0, 
                                                        'right_e0': 0.0, 
                                                        'right_e1': 0.0, 
                                                        'right_w0': 0.0, 
                                                        'right_w1': 0.0, 
                                                        'right_w2': 0.0 })

baxter_interface.Limb('right').move_to_joint_positions({
                                                        'right_s0': 0.0, 
                                                        'right_s1': 0.0, 
                                                        'right_e0': pi/2, 
                                                        'right_e1': pi/2, 
                                                        'right_w0': 0.0, 
                                                        'right_w1': 0.0, 
                                                        'right_w2': -pi/2 })

baxter_interface.Limb('right').move_to_joint_positions({
                                                        'right_s0': -pi/4, 
                                                        'right_s1': -pi/4, 
                                                        'right_e0': 0.0, 
                                                        'right_e1': 3*pi/4, 
                                                        'right_w0': 0.0, 
                                                        'right_w1': 0.0, 
                                                        'right_w2': 0.0 })


#check if gripper is calibrated, if not, reboot it and calibrate
print baxter_interface.Gripper('right').calibrated(), " cali"
if baxter_interface.Gripper('right').calibrated() == False:
    print "cal"
    #baxter_interface.Gripper('right').reboot()
    baxter_interface.Gripper('right').calibrate()

#open gripper
baxter_interface.Gripper('right').open()
rospy.sleep(1)

#close gripper
baxter_interface.Gripper('right').close()
rospy.sleep(1)

#open gripper
baxter_interface.Gripper('right').open()
rospy.sleep(1)



#Lets do some IK

#        X     Y    Z    R  P  Y
#        f/b   l/r  u/d
#       |     metres    |radians| 
#pose = [0.8, -0.4, 0.4, 0, 0, 0]
pose = [0.3, -0.7, 0.4, pi, 0, pi]

#list of 6 floats [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw]

#send the pose to the IK solver service running on Baxter
angles = ik_solver_request('right', pose)
#execute the joint angles that the service returned
baxter_interface.Limb('right').move_to_joint_positions(angles)

#print some sensor readings
print baxter_interface.Limb('right').joint_angles()     #radians
print baxter_interface.Limb('right').joint_efforts()    #Nm


#move the arm downwards in the same x/y position until the IR rangefinder reaches threshold
#this is a very quick and dirty implementation and serves only to show arm/sensor interaction
#if you wanted to do this in a real environment you wouldnt implement it as a single loop!!!
while baxter_interface.analog_io.AnalogIO('right_hand_range').state() > 375 and pose[2] > -0.4:
    pose[2] += -0.05
    print pose
    print baxter_interface.analog_io.AnalogIO('right_hand_range').state()
    #send the pose to the IK solver service running on Baxter
    angles = ik_solver_request('right', pose)
    #execute the joint angles that the service returned
    baxter_interface.Limb('right').move_to_joint_positions(angles)

print baxter_interface.analog_io.AnalogIO('right_hand_range').state()



print "ended, now spinning, Ctrl-c to exit"
#prevents program from exiting, allowing subscribers and publishers to keep operating
#in our case that is the camera subscriber and the image processing callback function
rospy.spin()

