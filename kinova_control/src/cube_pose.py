#!/usr/bin/env python

import roslib; roslib.load_manifest('kinova_control')
import rospy
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Float64MultiArray

def publishCubePose():
    # Create publisher
    pub = rospy.Publisher('/cube_pose/pose', Pose, queue_size=1)
    pub_up = rospy.Publisher('/cube_pose/up_vector', Float64MultiArray, queue_size=1)
    p = Pose() #pose data to be published
    o = Float64MultiArray() #Euler orientation data to be published

    # Set up server to get model states
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'cube'

    rate = rospy.Rate(2) #500ms

    while not rospy.is_shutdown():
        # Acquire pose from server
        result = get_model_srv(model)
        p = result.pose

        # Calculate up vector (quaternion -> euler)
        angles = tf.transformations.euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        o.data = angles #[roll, pitch, yaw]

        # Publish pose and Euler orientation
        pub.publish(p)
        pub_up.publish(o)

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('cube_pose', anonymous=True)
        publishCubePose()

    except rospy.ROSInterruptException:
        print "cube pose program interrupted before completion"