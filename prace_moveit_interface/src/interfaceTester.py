#!/usr/bin/env python
import roslib; roslib.load_manifest('prace_moveit_interface')
import rospy
import sys, getopt
from actionlib import *
from actionlib.msg import *
from prace_moveit_interface.msg import PtpAction, PtpGoal
from prace_moveit_interface.msg import LinAction, LinGoal
from prace_moveit_interface.msg import JointSpaceAction, JointSpaceGoal
from prace_moveit_interface.msg import ExecPlanAction, ExecPlanGoal

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

def main(argv):
    planning_type = ''
    goal_param = ''
    execute_motion = False
    try:
        opts, args = getopt.getopt(argv, "ha:p:g:e:", ["help","action_type=" "plan_type=", "goal=", "execute="])
    except getopt.GetoptError as err:
        print(err) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-h':
            print '-h --help: shows this message'
            print '-a --action_type: type of actioncall [move, plan, queue]'
            print '-p --plan_type: the type of movement [ptp, lin, js]'
            print '-g --goal: the param on param server'
            print '-e --execute: true if motion should be executed'
            sys.exit()
        elif opt in ("-a", "--action_type"):
            action_type = arg
        elif opt in ("-p", "--plan_type"):
            planning_type = arg
        elif opt in ("-g", "--goal"):
            goal_param = arg
        elif opt in ("-e", "--execute"):
            execute_motion = str2bool(arg)

    ptp_client_name = ''
    lin_client_name = ''
    js_client_name = ''
    exec_client_name = ''

    if action_type == "move":
        ptp_client_name = '/arm_controller/MovePTP'
        lin_client_name = '/arm_controller/MoveLin'
        js_client_name = '/arm_controller/MoveJointSpace'
    elif action_type == "plan":
        ptp_client_name = '/arm_controller/PlanPTP'
        lin_client_name = '/arm_controller/PlanLin'
        js_client_name = '/arm_controller/PlanJointSpace'
        exec_client_name = '/arm_controller/ExecPlan'
    elif action_type == "queue":
        ptp_client_name = '/arm_controller/QueuePtp'
        lin_client_name = '/arm_controller/QueueLin'
        js_client_name = '/arm_controller/QueueJointSpace'
        exec_client_name = '/arm_controller/ExecQueue'

            
    ptp_client = actionlib.SimpleActionClient(ptp_client_name, PtpAction)
    lin_client = actionlib.SimpleActionClient(lin_client_name, LinAction)
    js_client = actionlib.SimpleActionClient(js_client_name, JointSpaceAction)
    exec_client = actionlib.SimpleActionClient(exec_client_name, ExecPlanAction)
            
    if planning_type == "ptp":
        ptp_client.wait_for_server()
        goal = PtpGoal()
        genpy.message.fill_message_args(goal, [rospy.get_param(goal_param)])
        ptp_client.send_goal(goal)
        ptp_client.wait_for_result()
        
        print ptp_client.get_result()
        
    elif planning_type == "lin":
        lin_client.wait_for_server()
        goal = LinGoal()
        genpy.message.fill_message_args(goal, [rospy.get_param(goal_param)])
        lin_client.send_goal(goal)
        lin_client.wait_for_result()
        
        print lin_client.get_result()
        
    elif planning_type == "js":
        js_client.wait_for_server()
        goal = JointSpaceGoal()
        genpy.message.fill_message_args(goal, [rospy.get_param(goal_param)])
        js_client.send_goal(goal)
        js_client.wait_for_result()
        
        print js_client.get_result()
        
    if execute_motion is True:
        exec_client.wait_for_server()
        goal = ExecPlanGoal()
        genpy.message.fill_message_args(goal, [rospy.get_param('ExecuteQueue')])
        exec_client.send_goal(goal)
        exec_client.wait_for_result()
        
        print exec_client.get_result()


if __name__ == "__main__":
    try:
        rospy.init_node('interfaceTester')
        main(sys.argv[1:])        
        
            
    except rospy.ROSInterruptException:
        print "Exit"
