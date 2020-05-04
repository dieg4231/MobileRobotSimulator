#!/usr/bin/env python
from planner_msgs.msg import *
from planner_msgs.srv import *
from interprete import intSpeech

import rospy

#Service for the task.
def wait_command(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_speech(req)
	return planning_cmdResponse(success, args)

def interpreter(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_int(req)
	return planning_cmdResponse(success, args)

def confirmation(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_conf(req)
	return planning_cmdResponse(success, args)

def get_task(req):
	print "Receive: [%s  %s]"%(req.name, req.params)
	(success, args) = intSpeech.cmd_task(req)
	return planning_cmdResponse(success, args)

def answer(req):
    print "Receive: [%s  %s]"%(req.name, req.params)
    (success, args) = intSpeech.answer(req)
    return planning_cmdResponse(success, args)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)[('go to the bathroom and find the sponge', 0.99000001)]
    test = [(data.hypothesis[0], 0.99000001)]
    print "Texto Reconocido: " + data.hypothesis[0]
    intSpeech.subsRecSpeech(test)

def main():

    rospy.init_node('planning_rm_services')
    
    ######## servicios para los primeros pasos del interprete
    rospy.Service('/planning_rm/wait_command', planning_cmd, wait_command)
    rospy.Service('/planning_rm/interpreter',planning_cmd, interpreter)
    rospy.Service('/planning_rm/confirmation', planning_cmd, confirmation)
    rospy.Service('/planning_rm/get_task', planning_cmd, get_task)
    rospy.Service('/planning_rm/answer', planning_cmd, answer)

    rospy.Subscriber("recognizedSpeech", RecognizedSpeech, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
