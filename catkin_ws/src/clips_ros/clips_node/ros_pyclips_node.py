#!/usr/bin/env python
import time, threading, os
import Tkinter as tk

import clipsFunctions
from clipsFunctions import clips, _clipsLock

import pyRobotics.BB as BB
from pyRobotics.Messages import Command, Response

from clips_ros.msg import *
from clips_ros.srv import *
from std_msgs.msg import Bool, String

import BBFunctions

import rospy
import rospkg

defaultTimeout = 2000
defaultAttempts = 1
logLevel = 1


def setLogLevelTest():
        _clipsLock.acquire()
        clips.SendCommand('(bind ?*logLevel* ' + 'getloglevel' + ')')
        #clipsFunctions.PrintOutput()
        _clipsLock.release()

def callbackCommandResponse(data):
    print "callbackCommandResponse name command:" + data.name
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    clipsFunctions.Assert('(BB_received "{0}" {1} {2} "{3}")'.format(data.name, data.id, data.successful, data.params))
    clipsFunctions.PrintOutput()
    clipsFunctions.Run(BBFunctions.gui.getRunTimes())
    clipsFunctions.PrintOutput()

def callbackCommandSendCommand(data):
    clips.SendCommand(data.command, True)
    clipsFunctions.PrintOutput()

def callbackCommandRunCLIPS(data):
    print "callbackCommandRUNCLIPS "
    clipsFunctions.Run('') # aqui se manda el numero de pasos que ejecutara CLIPS con [''] se ejecutan todos los pasos sin detenerse
    clipsFunctions.PrintOutput()

def callbackCommandClearCLIPS(data):
    clipsFunctions.Clear();
    print "Enviroment was clear"
    setLogLevelTest()

def callbackCommandResetCLIPS(data):
    clipsFunctions.Reset()
    print 'Facts were reset!'
    setLogLevelTest()

def callbackCommandFactCLIPS(data):
    print 'LIST OF FACTS'
    clipsFunctions.PrintFacts()

def callbackCommandRuleCLIPS(data):
    print 'LIST OF RULES'
    clipsFunctions.PrintRules()

def callbackCommandAgendaCLIPS(data):
    print 'AGENDA'
    clipsFunctions.PrintAgenda()

def callbackCommandSendCLIPS(data):
    print 'SEND COMMAND'
    _clipsLock.acquire()
    clips.SendCommand(data.data, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()

def callbackCommandSendAndRunClips(data):
    print 'SEND AND RUN COMMAND'
    _clipsLock.acquire()
    clips.SendCommand(data.data, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()
    clipsFunctions.Run('')
    clipsFunctions.PrintOutput()

def callbackCommandLoadCLIPS(data):
    print 'LOAD FILE'
    filePath = data.data
    if not filePath:
        print 'OPEN FILE, Click on the botton and select a file to be loaded.'
        return

    if filePath[-3:] == 'clp':
        _clipsLock.acquire()
        clips.BatchStar(filePath)
        clipsFunctions.PrintOutput()
        _clipsLock.release()
        print 'File Loaded!'
        return

    path = os.path.dirname(os.path.abspath(filePath))
    f = open(filePath, 'r')
    line = f.readline()
    _clipsLock.acquire()
    while line:
        clips.BatchStar((path + os.sep + line).strip())
        line = f.readline()
    f.close()

    clipsFunctions.PrintOutput()
    _clipsLock.release()

    print 'Files Loaded!'

    clipsFunctions.Reset()
    print 'Facts were reset!'
    setLogLevelTest()


def setCmdTimer(t, cmd, cmdId):
    t = threading.Thread(target=cmdTimerThread, args = (t, cmd, cmdId))
    t.daemon = True
    t.start()
    return True

def cmdTimerThread(t, cmd, cmdId):
    time.sleep(t/1000)
    clipsFunctions.Assert('(BB_timer "{0}" {1})'.format(cmd, cmdId))
    clipsFunctions.PrintOutput()
    #clipsFunctions.Run(gui.getRunTimes())
    #clipsFunctions.PrintOutput()

def setTimer(t, sym):
    t = threading.Thread(target=timerThread, args = (t, sym))
    t.daemon = True
    t.start()
    return True

def timerThread(t, sym):
    time.sleep(t/1000)
    clipsFunctions.Assert('(BB_timer {0})'.format(sym))
    clipsFunctions.PrintOutput()
    #clipsFunctions.Run(gui.getRunTimes())
    #clipsFunctions.PrintOutput()

def SendCommand(cmdName, params, timeout = defaultTimeout, attempts = defaultAttempts):
    global pubUnknown
    print 'Function name ' + cmdName
    cmd = Command(cmdName, params)
    func = fmap.get(cmdName)
    if func != None:
        func(cmd)
    else:
        request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
        pubUnknown.publish(request)
    return cmd._id

def str_query_KDB(req):
    print 'QUERY IN KDB ' + req.query
    _clipsLock.acquire()
    clips.SendCommand(req.query, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()
    clipsFunctions.Run('')
    result = str(clips.StdoutStream.Read())
    #if result.find("ros"):
    #    print 'Query response'
    #    r = result.split('ros')
        
    print 'RESULT OF QUERY= ' + result
    print ''
    return StrQueryKDBResponse(result)

def str_query(req):
    print 'Query in KDB' + req.query
    _clipsLock.acquire()
    clips.SendCommand(req.query, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()
    clipsFunctions.Run('')
    result = str(clips.StdoutStream.Read())
    if result.count('ROS') == 2:
        print 'ROS response'
        r = result.split('ROS')
        print r
        result = r[1]

    else:
        result = 'None'
        
    print 'Result of Query= ' + result
    print ''
    return StrQueryKDBResponse(result)

def init_KDB(req):
    print 'INIT KDB'
    print 'LOAD FILE'
    global file_gpsr
    rospack = rospkg.RosPack()
    #file_gpsr = rospack.get_path('simulator') + '/src/rules_base/oracle.dat'
    if not req.filePath:
        filePath = file_gpsr
    else:
        filepath = rospack.get_path('simulator') 
        #clips.BatchStar(filepath + os.sep + 'CLIPS' + os.sep + 'BB_interface.clp')
        filePath = filepath + req.filePath
        #filePath = req.filePath
    print 'Load file in path' + filePath
    if filePath[-3:] == 'clp':
        _clipsLock.acquire()
        clips.BatchStar(filePath)
        clipsFunctions.PrintOutput()
        _clipsLock.release()
        print 'File Loaded!'
        return

    path = os.path.dirname(os.path.abspath(filePath))
    f = open(filePath, 'r')
    line = f.readline()
    _clipsLock.acquire()
    while line:
        clips.BatchStar((path + os.sep + line).strip())
        line = f.readline()
    f.close()
    clipsFunctions.PrintOutput()
    _clipsLock.release()

    print 'Files Loaded!'

    clipsFunctions.Reset()
    print 'Facts were reset!'
    setLogLevelTest()
    if req.run == True:
        clipsFunctions.Run('')
    return InitKDBResponse()

def clear_KDB(req):
    print "CLEAR KDB"
    clipsFunctions.Clear()
    setLogLevelTest()

    return clearKDBResponse(True)


#def SendResponse(cmdName, cmd_id, result, response):
    #result = str(result).lower() not in ['false', '0']
    #r = Response(cmdName, result, response)
    #r._id = cmd_id
    #BB.Send(r)

def Initialize():
    clips.Memory.Conserve = True
    clips.Memory.EnvironmentErrorsEnabled = True
    
    clips.RegisterPythonFunction(SendCommand)
    clips.RegisterPythonFunction(setCmdTimer)
    clips.RegisterPythonFunction(setTimer)
    
    clips.BuildGlobal('defaultTimeout', defaultTimeout)
    clips.BuildGlobal('defaultAttempts', defaultAttempts)
    
    filePath = os.path.dirname(os.path.abspath(__file__))
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'BB_interface.clp')
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'functions.clp')
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'monitor.clp')
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'virbot_blackboard.clp')
    
    #file_gpsr = filePath + '/virbot_test/oracle.dat'
    rospack = rospkg.RosPack()
    file_gpsr = rospack.get_path('simulator') + '/src/rules_base/oracle.dat'
    print file_gpsr
    BBFunctions.gui.putFileName(file_gpsr)
    # Savage BBFunctions.gui.loadFile()
    BBFunctions.gui.reset()

#Funcions to fmap, this functions are publish to topics to do the tasks
def cmd_speech(cmd):
    global pubCmdSpeech
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdSpeech.publish(request)
    return cmd._id

def cmd_int(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdInt.publish(request)
    return cmd._id

def cmd_conf(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdConf.publish(request)
    return cmd._id

def cmd_task(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdGetTask.publish(request)
    return cmd._id

def goto(cmd):
    global pubCmdGoto
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdGoto.publish(request)
    print "send pub"
    return cmd._id

def answer(cmd):
    global pubCmdAnswer
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAnswer.publish(request)
    return cmd._id

def find_object(cmd):
    global pubCmdFindObject
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdFindObject.publish(request)
    return cmd._id

def ask_for(cmd):
    global pubCmdAskFor
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAskFor.publish(request)
    return cmd._id

def status_object(cmd):
    global pubCmdStatusObject
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdStatusObject.publish(request)
    return cmd._id

def move_actuator(cmd):
    global pubCmdMoveActuator
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdMoveActuator.publish(request)
    return cmd._id

def drop(cmd):
    global pubDrop
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubDrop.publish(request)
    return cmd._id

def ask_person(cmd):
    global pubCmdAskPerson
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAskPerson.publish(request)
    return cmd._id

def clips_ros(cmd):
    global pubClipsToRos 
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubClipsToRos.publish(request)
    return cmd._id

#Define the function map, this function are the functions that represent of task in the clips rules.
fmap = {
    'cmd_speech': cmd_speech,
    'cmd_int': cmd_int,
    'cmd_conf': cmd_conf,
    'cmd_task': cmd_task,
    'find_object': find_object,
    'move_actuator': move_actuator,
    #'grab': grab,
    'drop': drop,
    'status_object': status_object,
    'goto': goto,
    #'speak': speak,
    'ask_for': ask_for,
    'answer' : answer,
    'clips_ros': clips_ros,
    'ask_person': ask_person
    }

def quit():
    global tk
    tk.quit()

def main():

    global pubCmdSpeech, pubCmdInt, pubCmdConf, pubCmdGetTask, pubUnknown
    global pubCmdGoto, pubCmdAnswer, pubCmdFindObject, pubCmdAskFor, pubCmdStatusObject, pubCmdMoveActuator, pubDrop, pubCmdAskPerson, pubClipsToRos

    rospy.init_node('planning_rm')
    rospy.Subscriber("/planning_rm/command_response", PlanningCmdClips, callbackCommandResponse)
    rospy.Subscriber("/planning_rm/command_send_command", PlanningCmdSend, callbackCommandSendCommand)
    
    rospy.Subscriber("/planning_rm/command_runCLIPS",Bool, callbackCommandRunCLIPS)
    rospy.Subscriber("/planning_rm/command_resetCLIPS",Bool, callbackCommandResetCLIPS)
    rospy.Subscriber("/planning_rm/command_clearCLIPS",Bool, callbackCommandClearCLIPS)
    rospy.Subscriber("/planning_rm/command_factCLIPS",Bool, callbackCommandFactCLIPS)
    rospy.Subscriber("/planning_rm/command_ruleCLIPS",Bool, callbackCommandRuleCLIPS)
    rospy.Subscriber("/planning_rm/command_agendaCLIPS",Bool, callbackCommandAgendaCLIPS)

    rospy.Subscriber("/planning_rm/command_sendCLIPS",String, callbackCommandSendCLIPS)
    rospy.Subscriber("/planning_rm/command_sendAndRunCLIPS", String, callbackCommandSendAndRunClips)
    rospy.Subscriber("/planning_rm/command_loadCLIPS",String, callbackCommandLoadCLIPS)

    rospy.Service('/planning_rm/str_query_KDB', StrQueryKDB, str_query)
    rospy.Service('/planning_rm/init_kdb', InitKDB, init_KDB)
    rospy.Service('/planning_rm/clear_kdb', clearKDB, clear_KDB)

    pubCmdSpeech = rospy.Publisher('/planning_rm/cmd_speech', PlanningCmdClips, queue_size=1)
    pubCmdInt = rospy.Publisher('/planning_rm/cmd_int', PlanningCmdClips, queue_size=1)
    pubCmdConf = rospy.Publisher('/planning_rm/cmd_conf', PlanningCmdClips, queue_size=1)
    pubCmdGetTask = rospy.Publisher('/planning_rm/cmd_task', PlanningCmdClips, queue_size=1)
    pubCmdGoto = rospy.Publisher('/planning_rm/cmd_goto', PlanningCmdClips, queue_size=1)
    pubCmdAnswer = rospy.Publisher('/planning_rm/cmd_answer', PlanningCmdClips, queue_size=1)
    pubCmdFindObject = rospy.Publisher('/planning_rm/cmd_find_object', PlanningCmdClips, queue_size=1)
    pubCmdAskFor = rospy.Publisher('/planning_rm/cmd_ask_for', PlanningCmdClips, queue_size=1)
    pubCmdStatusObject = rospy.Publisher('/planning_rm/cmd_status_object', PlanningCmdClips, queue_size=1)
    pubCmdMoveActuator = rospy.Publisher('/planning_rm/cmd_move_actuator', PlanningCmdClips, queue_size=1)
    pubDrop = rospy.Publisher('/planning_rm/cmd_drop', PlanningCmdClips, queue_size=1)
    pubUnknown = rospy.Publisher('/planning_rm/cmd_unknown', PlanningCmdClips, queue_size=1)
    pubCmdAskPerson = rospy.Publisher('/planning_rm/cmd_ask_person', PlanningCmdClips, queue_size=1)
    
    pubClipsToRos = rospy.Publisher('/planning_rm/clips_to_ros', PlanningCmdClips, queue_size=1)

    Initialize()
    
    tk.mainloop()

if __name__ == "__main__":
    main()
