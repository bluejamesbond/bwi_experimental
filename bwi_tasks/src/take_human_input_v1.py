#!/usr/bin/python

import time
import rospy

import roslib; roslib.load_manifest('bwi_tasks')

import actionlib
from bwi_kr_execution.msg import *
import segbot_gui.srv
import bwi_rlg.srv

from multiprocessing import Process, Value, Array

# human_waiting = False
# curr_goal = []
# next_room = None

def task_guiding(doorname, client, dialog_handle):

    dialog_handle(0, "Follow me please. We are arriving soon. ", [], 0)

    goal = ExecutePlanGoal()
    rule = AspRule()
    fluent = AspFluent()
    
    fluent.name = "not facing"
    fluent.variables = [doorname]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    rospy.loginfo("Sending goal (doorname): " + doorname)
    client.send_goal(goal)

    client.wait_for_result()

    res = dialog_handle(0, "You have arrived. I am leaving. \n\nThank you!", \
                        [""], 10)
    rospy.sleep(10)

    human_waiting.value = False
    

def task_delivery(person, item, client, dialog_handle):

    dialog_handle(0, "Got you. I am working on it...", [], 0)

    goal = ExecutePlanGoal()
    rule = AspRule()
    fluent = AspFluent()
    
    # going to the shop first - there is no shop - going to Jesse's pod
    fluent.name = "not facing"
    fluent.variables = ["d3_504"]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    rospy.loginfo("Sending goal (doorname): " + fluent.variables[0])
    client.send_goal(goal)

    client.wait_for_result()

    res = dialog_handle(1, "May I have " + item + " please?", \
                 ["Sorry, we do not have that", "Loaded"], 60)

    hasLoaded = bool(res.index)

    # loaded item, now going to the person's place
    person_door = {'peter'      : 'd3_508', 
                   'dana'       : 'd3_510',
                   'ray'        : 'd3_512',
                   'raymond'    : 'd3_512',
                   'stacy'      : 'd3_502',
                   'kazunori'   : 'd3_402',
                   'matteo'     : 'd3_418',
                   'jivko'      : 'd3_432',
                   'shiqi'      : 'd3_420'}

    fluent.name = "not facing"
    fluent.variables = [person_door[person]]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    rospy.loginfo("Sending goal (doorname): " + person_door[person])
    client.send_goal(goal)
    client.wait_for_result()

    if hasLoaded == True:
        res = dialog_handle(1, "Here is your " + item + ". ", \
                 ["Unloaded"], 30)

    res = dialog_handle(0, "Delivery done. I am leaving. \n\nThank you!", \
                        [""], 20)
    rospy.sleep(20)

    human_waiting.value = False
    
def process_request(query, client, dialog_handle):

    print ("query: " + query)

    if (query.find("at") >= 0): # this is a guiding task! 

        query = query.replace("at(l", "d")
        query = query[:query.find(",")]

        # in case there are multiple doors to a room, select the first one
        if query.find("d3_414") > 0:
            query += "1"

        task_guiding(query, client, dialog_handle)

    elif (query.find("query") >= 0): # this is a question-asking task! 

        rospy.loginfo("Question-asking tasks will be treated as guiding ones.")
        query = query.replace("query(l", "d")
        query = query[:query.find(":")]

        if query.find("d3_414") > 0:
            query += "1"

        task_guiding(query, client, dialog_handle)

    elif (query.find("served") >= 0): # this is a delivery task! 
        # served(shiqi,coffee,n)

        person_name = query[query.find('(')+1 : query.find(',')]
        # remove the person name -> coffee,n)
        query = query[query.find(',')+1 : ]
        item_name = query[: query.find(',')]

        task_delivery(person_name, item_name, client, dialog_handle)


# option 
# 1: click me if you need help
# 2: please let me know your goal place
def gui_thread(human_waiting, curr_goal):
  
    rospy.init_node('human_input_gui_thread')

    rospy.loginfo("gui_thread started")

    rospy.wait_for_service('question_dialog')
    
    handle = rospy.ServiceProxy('question_dialog', \
                                segbot_gui.srv.QuestionDialog)

    while not rospy.is_shutdown():

        if human_waiting.value == True:
            rospy.sleep(2)
            continue

        res = handle(1, "Please click the button, if you need my help." + \
                     "\n\nI am moving to room 3." + curr_goal.value[3:], \
                     ["Button"], 2)

        while (res.index < 0):

            res = handle(1, "Please click the button, if you need my help." + \
                         "\n\nI am moving to room 3." + curr_goal.value[3:], \
                         ["Button"], 2)

        human_waiting.value = True
        res = handle(0, "I have to go to room 3." + curr_goal.value[3:] + \
                        " first." + \
                        "\n\nFollow me please, I will serve you in a moment.",\
                     ["Button"], 0)

    return True

def platform_thread(human_waiting, curr_goal):

    rospy.init_node('human_input_platform_thread')

    rospy.loginfo("platform_thread started")

    rospy.wait_for_service('question_dialog')

    rooms = ["414a", "414a", "414b", "414b", "416", "418", "420", "432"]
    doors = ["d3_414a1", "d3_414a2", "d3_414b1", "d3_414b2", "d3_416", \
             "d3_418", "d3_420", "d3_432"]
    
    client = actionlib.SimpleActionClient('/action_executor/execute_plan',\
                                          ExecutePlanAction)
    client.wait_for_server()

    dialog_handle = rospy.ServiceProxy('question_dialog',
                                       segbot_gui.srv.QuestionDialog)

    while not rospy.is_shutdown():

        if human_waiting.value == True:

            rospy.loginfo("Human is waiting. Let me see if I can help.")

            # commumication......
            # robot speaks first

            res_qd = dialog_handle(0, 
                     "I can do guiding and shopping tasks for you.", [""], 5)

            rospy.sleep(5)
            parser_handle = rospy.ServiceProxy('semantic_parser', 
                                        bwi_rlg.srv.SemanticParser)
            res_sp = parser_handle(0, res_qd.text)

            while len(res_sp.query) == 0:
                
                # take human feedback
                res_qd = dialog_handle(2, res_sp.output_text, doors, 0)

                # robot speaks back
                res_sp = parser_handle(0, res_qd.text)

            # now the robot has found the query from semantic parser

            process_request(res_sp.query, client, dialog_handle)

        else:

            rospy.loginfo("No one needs me. I will do a random walk.")

            loc = doors[int(time.time()) % len(rooms)]
            curr_goal.value = loc
            
            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            rospy.loginfo("sending goal: " + loc)
            client.send_goal(goal)
    
            client.wait_for_result()


    return 1

if __name__ == '__main__':

    human_waiting = Value('b', False)
    curr_goal = Array('c', "this is a very long string for nothing")

    p1 = Process(target = gui_thread, args = (human_waiting, curr_goal))
    p2 = Process(target = platform_thread, args = (human_waiting, curr_goal))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

