#!/usr/bin/env python
#*****************************************************************************
# FroboMind Pose Estimator Node
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
This file wraps the FroboMind Pose 2D Estimator library into a ROS node.
Most documentation of the library is in pose_estimator.py

Revision
2013-05-16 LBL First version based on KJ's odometry_gnss_pose_estimator
"""
# ROS imports
import rospy,tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from msgs.msg import gpgga_tranmerc, IntArrayStamped
from math import pi, sqrt, atan2, asin
from pose_estimator import odometry_gnss_pose_preprocessor, odometry_pose_ekf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PoseEstimatorNode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		
		# Variables
		self.update_rate = 5
		self.first_odom_topic_received = False
		self.odometry_x_prev = 0.0
		self.odometry_y_prev = 0.0
		self.odometry_yaw_prev = 0.0
		self.latest_odo_update = 0.0
		self.latest_imu_update = 0.0
		self.acc_roll = 0.0
		self.acc_pitch = 0.0
				
		# Messages
		self.pose_msg = Odometry()
		self.quaternion = np.empty((4, ), dtype=np.float64) 
		
		# Get parameters
		self.pose_msg.header.frame_id = rospy.get_param("~frame_id", "base_link")
		self.pose_msg.child_frame_id = rospy.get_param("~child_frame_id", "odom")
		robot_max_velocity = float(rospy.get_param("~/robot_max_velocity", 1.0)) # Robot maximum velocity [m/s]

		self.odometry_var_dist = rospy.get_param("~odometry_distance_variance", 0.000001)
		self.odometry_var_yaw = rospy.get_param("~odometry_angular_variance", 0.000001)

		# Get topic names
		self.odom_topic = rospy.get_param("~odom_sub",'/fmKnowledge/odometry')
		self.imu_topic = rospy.get_param("~imu_sub",'/fmInformation/imu')
		self.line_pose_topic = rospy.get_param("~line_pose_sub",'/fmInformation/line_pose')
		self.lrs_pose_topic = rospy.get_param("~lrs_pose_sub",'/fmInformation/lrs_pose')
		self.marker_pose_topic = rospy.get_param("~marker_pose_sub",'/fmInformation/lrs_pose')
		
		self.pose_topic = rospy.get_param("~pose_pub",'/fmKnowledge/pose')
		
		# Setup subscription topic callbacks
		rospy.Subscriber(self.odom_topic, Odometry, self.on_odom_topic)
		rospy.Subscriber(self.imu_topic, Imu, self.on_imu_topic)
		rospy.Subscriber(self.line_pose_topic, Odometry, self.on_absolute_pose)
		rospy.Subscriber(self.lrs_pose_topic, Odometry, self.on_absolute_pose)
		rospy.Subscriber(self.marker_pose_topic, Odometry, self.on_absolute_pose)

		# setup publish topics
		self.pose_pub = rospy.Publisher(self.pose_topic, Odometry, queue_size=10)
		self.br = tf.TransformBroadcaster()

		# initialize estimator (preprocessing)
		self.pp = odometry_gnss_pose_preprocessor (robot_max_velocity)


		# initialize EKF
		self.ekf = odometry_pose_ekf()
		self.pose = [0.0, 0.0, 0.0]
		self.ekf.initial_guess (self.pose, self.odometry_var_dist, self.odometry_var_yaw)

		# Call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_odom_topic(self, msg):
		self.latest_odo_update = rospy.get_time()
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		self.quaternion[0] = msg.pose.pose.orientation.x
		self.quaternion[1] = msg.pose.pose.orientation.y
		self.quaternion[2] = msg.pose.pose.orientation.z
		self.quaternion[3] = msg.pose.pose.orientation.w
		(roll,pitch,yaw) = euler_from_quaternion(self.quaternion)

		# driving forwards or backwards?
		if (msg.twist.twist.linear.x >= 0):
			forward = True
		else:
			forward = False

		if self.first_odom_topic_received == True: # if we have received a first odom message
			# EKF system update (odometry)
			delta_dist =  sqrt((x-self.odometry_x_prev)**2 + (y-self.odometry_y_prev)**2)
			delta_angle = self.angle_diff (yaw, self.odometry_yaw_prev)
			self.pp.odometry_new_data (self.latest_odo_update, delta_dist, delta_angle, forward)
			self.pose = self.ekf.system_update (delta_dist, self.odometry_var_dist, delta_angle, self.odometry_var_yaw)

			# publish the estimated pose	
			self.publish_pose()

		# housekeeping
		self.first_odom_topic_received = True 
		self.odometry_x_prev = x
		self.odometry_y_prev = y
		self.odometry_yaw_prev = yaw

	def on_absolute_pose(self, msg):
		self.latest_odo_update = rospy.get_time()
		self.quaternion[0] = msg.pose.pose.orientation.x
		self.quaternion[1] = msg.pose.pose.orientation.y
		self.quaternion[2] = msg.pose.pose.orientation.z
		self.quaternion[3] = msg.pose.pose.orientation.w
		(roll,pitch,yaw) = euler_from_quaternion(self.quaternion)

		pos_variance = (msg.pose.covariance[0] + msg.pose.covariance[7])/2.0
		yaw_variance = msg.pose.covariance[35]
		# EKF system update
		
		if pos_variance == 0.0 :
			pos_variance = 0.000001
			
		if yaw_variance == 0.0 :
			yaw_variance = 0.000001
			
		self.pose = self.ekf.measurement_update ([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw], pos_variance, yaw_variance)

		# publish the estimated pose	
		self.publish_pose()


	def on_imu_topic(self, msg):
		self.latest_imu_update = rospy.get_time()
		self.pp.imu_new_data (self.latest_imu_update, msg.angular_velocity.z)

		# determine pitch and roll based on the accelerometers
		ax = msg.linear_acceleration.x
		ay = msg.linear_acceleration.y
		az = msg.linear_acceleration.z
		self.acc_pitch = atan2(-ay, sqrt(ax**2 + az**2))
		self.acc_roll = atan2(ax, az)


	def publish_pose(self):		
		self.pose_msg.header.stamp = rospy.Time.now()
		self.pose_msg.pose.pose.position.x = self.pose[0]
		self.pose_msg.pose.pose.position.y = self.pose[1]
		self.pose_msg.pose.pose.position.z = 0
		q = quaternion_from_euler (self.acc_roll, self.acc_pitch, self.pose[2])
		self.pose_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
		self.pose_pub.publish(self.pose_msg); # publish the pose message
		self.br.sendTransform((self.pose[0],self.pose[1],0), q, rospy.Time.now(), \
			self.pose_msg.header.frame_id, self.pose_msg.child_frame_id) # publish the transform message

	def updater(self):
		while not rospy.is_shutdown(): # updated at the rate defined by self.update_rate
			if self.first_odom_topic_received == False: # publish pose from here until odometry is received.
				self.publish_pose()

			# go back to sleep
			self.r.sleep()

	# return signed difference between new and old angle
	def angle_diff (self, angle_new, angle_old):
		diff = angle_new - angle_old
		while diff < -pi:
			diff += 2*pi
		while diff > pi:
			diff -= 2*pi
		return diff

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('odometry_gnss_pose_estimator_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = PoseEstimatorNode()
    except rospy.ROSInterruptException: pass

