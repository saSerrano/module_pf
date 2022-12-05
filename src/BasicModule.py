#!/usr/bin/env python
# coding=utf-8
import sys
import rospy  # ROS
from basicmodutil_pkg import commBM # Basic module communication functions
from std_msgs.msg import String # Necessary for the basic-mod comunicates with the master node

class BasicMod:
    '''
    DESCRIPTION

    This class implements the basic module node for the people-following
    skill.
    '''
    def __init__(self,bm_name):
        #Name of the basic module this node will serve as bridge
        self.__basic_mod_name = bm_name

        #communication pub & sub
        self.__comm_pub = rospy.Publisher('/function/output',String,queue_size=1)
        self.__event_pub = rospy.Publisher('/event/output',String,queue_size=1)
        self.__comm_sub = rospy.Subscriber('/master/output',String,self.__comm_cb)

        # event-detection sub
        self.__event_sub = rospy.Subscriber('/module_pf/events',String,self.__event_cb)

        # send-commands to tracker & path-manager pub
        self.__command_pub = rospy.Publisher('/module_pf/commands',String,queue_size=1)

    def __comm_cb(self,data):
        #Parse the function-invocation message into a dictionary
        bm, func, msgs = commBM.readFunCall(data.data)

        #Check if this basic module is being requested
        if(bm == self.__basic_mod_name):
            if(func == 'followPeople'):
                self.__followPerson()
            elif(func == 'stopFollowing'):
                self.__stopFollowing()
        return

    def __event_cb(self,data):
        # Event type
        event_name = ''
        if data.data == 'found_person_to_follow':
            event_name = 'foundPerson'
        elif data.data == 'lost_followed_person':
            event_name = 'lostPerson'

        # Display event in console
        print(self.__basic_mod_name + '-event: '+event_name)

        # Send event notification
        mod_name = self.__basic_mod_name
        values = []
        names = []
        json_str = commBM.writeEventFromRos(mod_name,event_name,values,names)
        self.__event_pub.publish(String(json_str))

    def __followPerson(self):
        # Send follow-person command to the tracker and path-manager nodes
        self.__command_pub.publish(String('FollowPerson'))

        #Return the output-params to notify the action has been taken
        out_params = []
        names = []
        msg_str = commBM.writeMsgFromRos(out_params, names)
        out_msg = String(msg_str)
        self.__comm_pub.publish(out_msg)
    
    def __stopFollowing(self):
        # Send follow-person command to the tracker and path-manager nodes
        self.__command_pub.publish(String('StopFollowing'))

        #Return the output-params to notify the action has been taken
        out_params = []
        names = []
        msg_str = commBM.writeMsgFromRos(out_params, names)
        out_msg = String(msg_str)
        self.__comm_pub.publish(out_msg)

def main(args):
	basic_module_name = 'module_pf'
	rospy.init_node(basic_module_name, anonymous=False)
	BasicMod(basic_module_name)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
