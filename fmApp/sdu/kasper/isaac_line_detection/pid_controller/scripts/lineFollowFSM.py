#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

import math
import numpy

from pid_controller.msg import line_points
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


lineData = [[3.0,2.0],[1.0,1.0]]

#class BehaviourFollowLine():
pid_enabled=False
error_enable=0


#Follow line using camera until Correct QR is found.
class StateFollowLineQR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      #  rospy.loginfo("in StateFollowLineQR.init")
        self.p = -0.5 #The further from the goal, the more power.
        self.i = -0.00 #If some external force is influencing the robot, I will slowly overpower this force
        self.d = -0.3 #Should counter the integrated part when an error of 0 is reoptained
        self.myCenter = [0,0]#[0,0] #Robot center located at (0,0)
        self.memCTError = 0        
        self.iCTError = 0
        self.PIDFreq = 20.0
        self.PIDPeriod = 1/self.PIDFreq
        self.PIDRate = rospy.Rate(self.PIDFreq) # 10hz
        self.publisher = rospy.Publisher('/fmCommand/cmd_vel', TwistStamped)
        self.publish_enabled= False

        

    def calcCTError(self,linePoints):
        #TODO:Handle changing lines or corners        
        
        #Use line knowledge, ax+b, form Isaac, given in robot coordinate system
        #Calculate orthogonal distance from robot center to line. 
            #Dist from point to line can be calculated by basic math formula
            #REF:http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
            #REF image:http://mathworld.wolfram.com/images/equations/Point-LineDistance2-Dimensional/NumberedEquation10.gif

        pointOnLine1 = linePoints[0]
        pointOnLine2 = linePoints[1]

        num = numpy.linalg.norm( numpy.linalg.det( [ numpy.subtract( pointOnLine2,pointOnLine1 ), numpy.subtract( pointOnLine2,self.myCenter ) ]  ) )
        denum = ( numpy.linalg.norm( numpy.subtract(pointOnLine2,pointOnLine1) ) )
        dist =  num/denum
        return dist
    
    def lineDirection(self, linePoints):
	#y value where checked the x sign
	y=0.5
	# COmpute line equation
	a=(linePoints[1][1]-linePoints[0][1])/(linePoints[1][0]-linePoints[0][0])
	
	b=linePoints[0][1]-a*linePoints[0][0]
	
	xval=(y-b)/a - 0.1

	if (xval < -0.3) or (xval > 0.3):
	    xval=0


	return xval
	  
	if xval > 0:
	  return 1
	else:
	  return -1
    
    def regulateErrorTo0(self,linePoints):
       # rospy.loginfo("in regto0")
        #TODO: Create ROS topic CONTROL_TWIST
        # CTError = self.calcCTError(linePoints)
        
        lineRelDirection = self.lineDirection(linePoints)
        #CTError = CTError * lineRelDirection
	CTError = lineRelDirection
        
        
        dCTError = error_enable * (CTError - self.memCTError)/(self.PIDPeriod)
        self.iCTError = error_enable*(self.iCTError + ((self.memCTError + CTError)/2)*(self.PIDPeriod))
        self.memCTError = error_enable*CTError
        
        
        
        #rospy.loginfo(lineRelDirection)
        rospy.loginfo(CTError)
        rospy.loginfo(self.iCTError)
        rospy.loginfo(dCTError)
        #rospy.loginfo(linePoints[0][0])
        #rospy.loginfo("p: " + CTError + ", i: " + self.iCTError + ", d: " + dCTError)

        controlSignal = self.p*CTError + self.i*self.iCTError + self.d*dCTError
        controlSignalTwist = controlSignal #!!!!!!!!!!!!!!!!HOW TO SEND TWIST?

	#rospy.loginfo(controlSignal)

        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = 0.25;                   # our forward speed
        twist.twist.linear.y = 0; twist.twist.linear.z = 0;     # we can't use these!        
        twist.twist.angular.x = 0; twist.twist.angular.y = 0;   #          or these!
        twist.twist.angular.z = controlSignal;    


        if pid_enabled :

	   # if controlSignal<1 :	
                self.publisher.publish(twist)        

        return controlSignalTwist
    
    def compareQRCode(self):
        #TODO: subscribe to QR topic        
        return 0
    
    def execute(self, userdata):
       # rospy.loginfo('Executing state StateFollowLineQR')
        #TODO: Get rsd_LineData from Isaacs code        
        
        #linePoints1 = [[1,3],[3,4]]       
        
        self.regulateErrorTo0(lineData)
        QRCodeCorrect = self.compareQRCode()
        self.PIDRate.sleep() #!!!!!!!!!!!!!!!how does this work with SMACH
        if QRCodeCorrect == 1:
            return 'outcome1'
        else:
            return 'outcome2'

     
        
 # define state Bar
class StateWaitAtQR(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state StateWaitAtQR')
        return 'outcome2' #loop this state  



# define state Bar
#class Bar(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['outcome2'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state BAR')
#        return 'outcome2'
        


def callback(data):
    #rospy.loginfo(data.data)
    #rospy.loginfo("%f y: %f" % (data.x1, data.y1))
    global lineData
    lineData = [[data.x1,data.y1],[data.x2,data.y2]]
   # rospy.loginfo(data.x1)
   
   
def callback_pid_enable(data):
    #rospy.loginfo(data.data)
    #rospy.loginfo("%f y: %f" % (data.x1, data.y1))
   
   # rospy.loginfo("Received from subscriber")  
   # rospy.loginfo(data.data)   
    
    global pid_enabled

    if (pid_enabled and not data.data):
        #publish 0 velocity comand
         publisher = rospy.Publisher('/fmCommand/cmd_vel', TwistStamped)        
         twist = TwistStamped()
         twist.header.stamp = rospy.Time.now()
         twist.twist.linear.x = 0;                   # our forward speed
         twist.twist.linear.y = 0; twist.twist.linear.z = 0;     # we can't use these!        
         twist.twist.angular.x = 0; twist.twist.angular.y = 0;   #          or these!
         twist.twist.angular.z = 0;    
         publisher.publish(twist)        
        

    pid_enabled=data.data

    # Dirty trick!!!
    # For reseting the error, what I do is multiplying the errormem by 0 when I am not publishing
    # this way, we avoid the error_i part from growing while we are not doing the line following.    
    global error_enable
    if data.data :
        error_enable=1
    else :
        error_enable=0
   
    
   
   
# main
def main():
    #rospy.init_node('smach_example_state_machine')

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('lineFollowFSM', anonymous=True)

    rospy.loginfo("Started")

    #publisher = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("line_node/line_points", line_points, callback)
    
    ## Subscriber for enabling or not the publishing
    rospy.Subscriber("line_action/pid_enable",Bool , callback_pid_enable)
    
    

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container

    with sm:
        # Add states to the container
        smach.StateMachine.add('STATE_FOLLOW_LINE_QR', StateFollowLineQR(), 
                               transitions={'outcome1':'STATE_WAIT_AT_QR', 
                                            'outcome2':'STATE_FOLLOW_LINE_QR'})
        smach.StateMachine.add('STATE_WAIT_AT_QR', StateWaitAtQR(), 
                               transitions={'outcome2':'STATE_WAIT_AT_QR'})

    sis = smach_ros.IntrospectionServer("msTest", sm, "/SM_ROOT")
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()  


if __name__ == '__main__':

   
    main()
