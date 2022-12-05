#!/usr/bin/env python
# coding=utf-8
import sys
import rospy
from basicmodutil_pkg import commBM
from std_msgs.msg import String

class Alexa2BasicModule:
    '''
    DESCRIPTION

    The purpose of this class is simply to serve as a translator between the
    commands issued by the Alexa-ROS node and the people-following basic mo-
    dule.
    '''
    def __init__(self):
        # Subscription to receive commands from the alexa-skill server
        self.__comm_sub = rospy.Subscriber('/speech_to_follow',String,self.__alexa_cb)

        # To publish function requests to the people-following basic module
        self.__comm_pub = rospy.Publisher('/master/output',String,queue_size=1)

    def __alexa_cb(self,data):
        # Build function request
        json_str = None
        if(data.data == 'follow-me'):
            json_str = commBM.writeFunCallFromRos('module_pf','followPeople',[])
        elif(data.data == 'stop'):
            json_str = commBM.writeFunCallFromRos('module_pf','stopFollowing',[])
        
        # Publish function request
        if json_str != None:
            self.__comm_pub.publish(String(json_str))

def main(args):
	rospy.init_node('alexa2basicmodule', anonymous=False)
	Alexa2BasicModule()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
