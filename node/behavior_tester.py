#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
import yaml

'''
Commands:
----------
rosrun rosmon rosmon --name-sim_launch project11_simulation sim_local.launch
rosrun mission_manager mission_manager_navigator.py __ns:=ben _map_frame:=ben/map
rosrun mesobot_behavior mesobot_behavior_node.py __ns:=ben
'''

trackplan = '''[{
    "label": "test mission",
    "type": "TrackLine",
    "children" : [    
                {"type":"Waypoint",
                "label":"waypoint0",
                "latitude": 43.076,
                "longitude": -70.708373},
                {"type":"Waypoint",
                "label":"waypoint1",
                "latitude": 43.076,
                "longitude": -70.70843},
                {"id": "drix-mesobot-behavior",
                "behaviorType": "mesobot",
                "type":"Behavior",
                "enabled": "true",
                "data": "key1: value1"},
                {"id": "BEV2",
                "type": "Behavior",
                "behaviorType": "behavior2_type",
                "enabled": "true",
                "data": "key1: value1"}   
    ]
 }]'''

surveyplan = '''[{
    "label": "test mission",
    "type": "SurveyArea",
    "children" : [    
                {"type":"Waypoint",
                "label":"waypoint0",
                "latitude": 43.076,
                "longitude": -70.708373},
                {"type":"Waypoint",
                "label":"waypoint1",
                "latitude": 43.076,
                "longitude": -70.70843},
                {"type":"Waypoint",
                "label":"waypoint2",
                "latitude": 43.075,
                "longitude": -70.70843},
                {"type":"Waypoint",
                "label":"waypoint3",
                "latitude": 43.075,
                "longitude": -70.708373},
                {"type": "Behavior",
                "id": "drix-mesobot-behavior",
                "behaviorType": "mesobot",
                "enabled": "true",
                "data": "key1: value1"},
                {"id": "BEV2",
                "type": "Behavior",
                "behaviorType": "behavior2", 
                "enabled": "true",
                "data": "key1:value1"}   
    ]
 }]'''

P = rospy.Publisher('/ben/project11/mission_manager/command',
                    String,
                    queue_size=10)
#S=String()
rospy.init_node('BehaviorTester', anonymous=False)

plan = surveyplan
plan = trackplan
J = json.dumps(plan)
S = 'replace_task mission_plan '
#S = 'debug_tasklist'
#S = 'append_task mission_plan '
#S += J
S += plan
print("Publishing:\n%s" % S)
P.publish(S)
