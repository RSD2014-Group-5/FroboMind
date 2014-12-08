#!/usr/bin/env python
from rospy import init_node, get_param, loginfo, logerr, is_shutdown
#from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from signal import signal, SIGINT, SIG_DFL

import asyncore, socket, sys, time, rospy



#import nav_msgs
import tf

import geometry_msgs.msg
import nav_msgs.msg

import numpy as np

class TCPBridgeClient(asyncore.dispatcher):
	def __init__(self,host,port=9090, order=7):
		print self.__class__,"__INIT__"
		asyncore.dispatcher.__init__(self)
		self.order = order

		self.host, self.port = host, int(port)
		self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
		self.connect( (self.host, self.port) )
		self.buffer=""
		self.odom_topic = rospy.get_param("~odom_pub",'/fmInformation/marker_locator_pose')
		self.odom_pub = rospy.Publisher(self.odom_topic, nav_msgs.msg.Odometry, queue_size=10)
	
	def handle_connect(self):
		print 'Connected to {}:{}.'.format(self.host,str(self.port))

	def handle_close(self):
		print 'Connection closed.'
		self.close()

	def handdle_error(self):
		print self.__class__,"handle_error"

	def handle_read(self):
#         try:
              data = self.recv(1024)
              if not data:
                  return


              #print data
			  
              datalist = data.split(',')
              timestamp = datalist[2]
              for i in range( (len(datalist) - 3 )/4):
                  if str(self.order) == datalist[ 3 + i*4]:
                      x = np.double(datalist[4 + i*4])
                      y = np.double(datalist[5 + i*4])
                      th = np.double(datalist[6 + i*4])
                      
              """ create and pubish tranmerc """
              
#              geometry_msgs.Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
              odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
              q = geometry_msgs.msg.Quaternion(odom_quat[0],odom_quat[1],odom_quat[2],odom_quat[3])


              odom = nav_msgs.msg.Odometry()
#              odom.header.stamp.secs = int(timestamp)
              odom.header.frame_id = "odom"
            
              odom.pose.pose.position.x = x/100;
              odom.pose.pose.position.y = y/100;
              odom.pose.pose.position.z = 0.0;
#              print type(odom_quat)
              odom.pose.pose.orientation = q;
             
              odom.child_frame_id = "base_footprint";
              odom.twist.twist.linear.x = 0;
              odom.twist.twist.linear.y = 0;
              odom.twist.twist.angular.z = 0;
            
              self.odom_pub.publish(odom);

#         except:
#            print "error "
#            pass
         
 

	def writable(self):
		return len(self.buffer)>0

	def handle_write(self):
		sent = self.send(self.buffer)
		self.buffer = self.buffer[sent:]

# list of possible parameters ( with internal default values <-- get overwritten from parameter server and commandline)
port = 9090                             # integer (portnumber)
host = '192.168.1.50'                      # hostname / IP as string

if __name__ == "__main__":
    init_node("marker_locator_tcp_client" )
    order = rospy.get_param("~order", 7)
    print "Order is set to: " + str( order )

    signal(SIGINT, SIG_DFL)

    client = TCPBridgeClient(host,port,order)
    asyncore.loop()
