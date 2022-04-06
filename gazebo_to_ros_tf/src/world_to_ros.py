#!/usr/bin/env python

import math
from math import sin, cos, pi
import yaml
import rospy, rospkg
import tf
import tf_conversions
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, TransformStamped

class GazeboRos():
    def __init__(self):	
	rospack = rospkg.RosPack()
	rospack.list() 
	path = rospack.get_path('gazebo_to_ros_tf')
	self.yamlpath = path + '/config/data.yaml'
        with open(self.yamlpath) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            for key, value in data.items():
                if key == "odomtopic":
                    self.odomtopic = value
                if key == "odom_frame_name":
                    self.odom_frame = value
                if key == "baselink_frame_name":
                    self.baselink_frame = value
	rospy.init_node('odometry_publisher')
	#rospy.Subscriber(self.odomtopic, PoseStamped, self.cb_pose, queue_size=1)
	rospy.Subscriber(self.odomtopic, Odometry, self.cb_pose, queue_size=1)
	self.odom_pub = rospy.Publisher(self.odom_frame, Odometry, queue_size=1)
	tfBuffer = tf2_ros.Buffer()
	self.odom_broadcaster = tf2_ros.TransformBroadcaster()
	self.current_time = rospy.Time.now()
	self.last_time = rospy.Time.now()
        
	self.r = rospy.Rate(1000)

    def cb_pose(self, msg):
        self.local = msg
        self.localx = msg.pose.pose.position.x
        self.localy = msg.pose.pose.position.y
        self.localz = msg.pose.pose.position.z
	self.orix = msg.pose.pose.orientation.x
	self.oriy = msg.pose.pose.orientation.y
	self.oriz = msg.pose.pose.orientation.z
	self.oriw = msg.pose.pose.orientation.w
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        # print("roll is %f"  %roll)
        # print("pitch is %f" %pitch)
        self.yawdeg = self.yaw * (180/3.14)
        #print ("yaw is %f" %self.yawdeg)
        # print (self.local)
        # print (self.localx)
        #self.localxx = self.localy
        #self.localyy = -1*self.localx
        #print("first x, y")
        #print (self.localx, self.localy)
        #print("second x, y")
        #print (self.localx, self.localy)
        # print (self.localz) 
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
	r= rospy.Time.now()
        odom.header.stamp = r
	#print(self.odom_frame,self.baselink_frame)
        odom.header.frame_id = self.odom_frame;
        odom.child_frame_id = self.baselink_frame;
        # set the position
        odom.pose.pose.position.x = self.localx;
        odom.pose.pose.position.y = self.localy;
        odom.pose.pose.position.z = self.localz;
        odom.pose.pose.orientation.x = self.orix;
        odom.pose.pose.orientation.y = self.oriy;
        odom.pose.pose.orientation.z = self.oriz;
        odom.pose.pose.orientation.w = self.oriw;
	t = TransformStamped()
	t.header.stamp = r
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.baselink_frame
        t.transform.translation.x = self.localx
        t.transform.translation.y = self.localy
        t.transform.translation.z = self.localz
        #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = self.orix
        t.transform.rotation.y = self.oriy
        t.transform.rotation.z = self.oriz
        t.transform.rotation.w = self.oriw  
	# first, we'll publish the transform over tf
        #self.odom_broadcaster.sendTransform((self.localx, self.localy, self.localz), (self.orix, self.oriy, self.oriz, self.oriw), self.current_time, "base_link1", "odom1")
        self.odom_broadcaster.sendTransform(t)
        # publish the message
        self.odom_pub.publish(odom)
        last_time = r
        self.r.sleep()

    def listener(self):
	rospy.spin()
print("someeeeeeeeeething")
gz = GazeboRos()
gz.listener()
