#!/usr/bin/env python

import roslib; roslib.load_manifest('kinova_control')
import rospy
import actionlib
import kinova_msgs.msg
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import time
import numpy as np
import pinocchio as pin
from math import *

# from robot_control_modules import *

prefix = 'j2n6s300'
nbJoints = 6

q = np.zeros(12)
x = np.zeros(3)
cube = np.zeros(3)

#JACO robot urdf model
filePath = '/home/bethany/catkin_ws/src/kinova-ros/kinova_description/urdf/' #NOT FLEXIBLE
filename = filePath + 'jaco.urdf'

def joint_state_callback(msg):
  global q, q_joint
  #[joint 1, joint 2, ... joint 6, joint finger 1, joint finger 2, etc..]
  q[:] = msg.position[:]

def link_state_callback(msg):
  global x
  x[:] = np.array([msg.pose[8].position.x, msg.pose[8].position.y, msg.pose[8].position.z])

def cube_pose_callback(msg):
  global cube
  cube[:] = np.array([msg.position.x, msg.position.y, msg.position.z])

def moveJoint(jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()  

def moveFingers (jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()    

def moveEndEffector(x_ref, model, data, rate):
  kp = 1.0
  epsilon = 0.1
  while (abs(x_ref - x) > epsilon):
    del_x = kp*(x_ref - x)

    # Calculate pseudo-inv Jacobian
    pin.computeJointJacobians(model, data, q)
    pin.framesForwardKinematics(model, data, q)
    J = pin.getFrameJacobian(model, data, model.getFrameId(prefix+"_joint_"+str(nbJoints)), pin.ReferenceFrame.LOCAL)
    Jinv = np.linalg.pinv(J[:3])

    # Calculate qdot = inv(J) * xdot
    del_q = np.dot(Jinv, del_x)
    # Integrate qdot
    q_ref = q[:6] + del_q[:6] * (0.5)
    moveJoint(q_ref, np.zeros(nbJoints), prefix, nbJoints)
    print('desired velocity: ', del_q)
    
    rate.sleep()

def hardcodePickandPlace():
  print('starting upright')
  moveJoint([0.0, 3.14, 3.14, 0.0, 0.0, 0.0], prefix, nbJoints)
  time.sleep(5)

  print('opening fingers')
  moveFingers([0, 0, 0], prefix, 3)
  time.sleep(5)

  print('picking up cube')
  moveJoint([pi/2, 2.0, 2.65, 0.0, 0.0, 0.0], prefix, nbJoints)
  time.sleep(5)
  print('lowering arm')
  moveJoint([pi/2, 1.0, 2.65, 0.0, 0.0, 0.0], prefix, nbJoints)
  time.sleep(5)

  print('closing fingers')
  moveFingers([1, 1, 1], prefix, 3)
  time.sleep(5)

  print('lifting cube')
  moveJoint([pi/2, 1.2, 2.65, 0.0, 0.0, 0.0], prefix, nbJoints)
  time.sleep(5)
  print('flipping cube')
  moveJoint([pi/2, 1.2, 2.65, 0.0, 0.0, pi], prefix, nbJoints)
  time.sleep(5)

  print('moving cube to goal')
  moveJoint([-pi, 1.0, 2.65, 0.0, 0.0, pi], prefix, nbJoints)
  time.sleep(20)

  print('opening fingers')
  moveFingers([0, 0, 0], prefix, 3)
  time.sleep(5)

  print('move back to upright position')
  moveJoint([0.0, 3.14, 3.14, 0.0, 0.0, 0.0], prefix, nbJoints)
  time.sleep(5)


if __name__ == '__main__':
    try:
        # prefix, nbJoints = argumentParser(None)
        rospy.init_node('pick_and_place', anonymous=True)

        # Define subscribers for joint and link states
        sub_joints = rospy.Subscriber("/" + prefix + "/joint_states", JointState, joint_state_callback)
        sub_links = rospy.Subscriber("/gazebo/link_states", LinkStates, link_state_callback)
        # Define subscriber for cube position
        sub_cube = rospy.Subscriber('/cube_pose/pose', Pose, cube_pose_callback)

        # Load the urdf model
        model = pin.buildModelFromUrdf(filename)
        data = model.createData()
       
        # Unpause the physics
        print('starting physics')
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        resp = unpause_gazebo()
        print('unpausing physics')

        rate = rospy.Rate(2)

        # moveEndEffector(cube, model, data, rate)
        hardcodePickandPlace()

        print('done')
    except rospy.ROSInterruptException:
        print "pick and place program interrupted before completion"

