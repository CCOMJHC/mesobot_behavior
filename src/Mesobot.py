#!/usr/bin/env python3

import rospy
import py_trees
import py_trees_ros
import yaml

#####################################################################
# Import classes for each behavior tree condition or action:
#####################################################################
# Must import the blackboard module first!
from mesobot_blackboard import *
from mesobot_enable_action import *
from mesobot_emergency_condition import *
from mesobot_emergency_action1 import *
from mesobot_locationunknown_condition import *
from mesobot_onthesurface_condition import *
from mesobot_surfacesearch_action import *
from mesobot_submergedsearch_action import *
from mesobot_acousticsurveyincomplete_condition import *
from mesobot_EK80survey_action import *
from mesobot_notinposition_condition import *
from mesobot_reposition_action import *
from mesobot_executediveplan_action import *

from nav_msgs.msg import Odometry
from project11_msgs.msg import BehaviorInformation
from project11_navigation.msg import RunTasksAction, RunTasksGoal

import threading



#############################################################
# Utility Methods:
#############################################################
def shutdown(tree):
    tree.interrupt()

#############################################################
# Collecting data for the blackboard.
#############################################################

# The data2bb is a sequence item that triggers the behaviors
# for pulling data into the blackboard.
'''
data2bb = py_trees.composites.Sequence(
    name = "Data2BB",
    memory=False
)
'''
data2bb = py_trees.composites.Parallel(
    name = "Data2BB",
    memory=False,
    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
)

beh2BB = ToBB(
    name = 'behparam',
    blackboard = bhv_bb
)
'''
# An instance to write asv position data to the blackboard.
behavior2BB = ToLocalBlackboard(
    name="behaviorparam2BB",
    topic_name="/ben/project11/behaviors/mesobot/input",
    topic_type = BehaviorInformation,
    blackboard_variables={'behavior': None},
    blackboard = bhv_bb
)

asvpos2BB = ToLocalBlackboard(
    name="ASVData2BB",
    topic_name="/ben/project11/odom",
    topic_type=Odometry,
    blackboard_variables={'asv_pos': None},
    blackboard = bhv_bb
)
# An instance to write mesobot position data to the blackboard.
mesobotpos2BB = ToLocalBlackboard(
    name="MesobotData2BB",
    topic_name="/ben/project11/odom",  # place-holder.
    topic_type=Odometry,
    blackboard_variables={'mesobot_pos': None},
    blackboard=bhv_bb
)
'''




def CreateBehaviorTree(data,feedback_pub):
    #####################################################################
    # Create the conditions, actions, and composite logic to build the tree:
    #####################################################################
    print(data)
    print(feedback_pub)
    # The root of the behavior tree is implemented as a Parallel
    # composite item, which executes the branch which collects data for 
    # the blackboard in parallel with the rest of the behavior tree logic.
    root = py_trees.composites.Parallel(
        name="Mesobot",
        policy = py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
    )

    # This behavior allows the behavior to be enabled or disabled. 
    Enable = Enabler(name='Enabled?')

    # Emergency Condition
    EmergencyCondition = Emergency(name='Mesobot Emergency?',
                                feedback = feedback_pub)

    # Emergency Sequence:
    EmergencySequence = py_trees.composites.Sequence(
        name="Emergency"
    )
    InvertEmergencyCondition = py_trees.decorators.FailureIsSuccess(EmergencySequence)

    # Emergency Action Sequence:
    EmergencyActionSequence = py_trees.composites.Sequence(
        name="EmergencyActions"
    )
    # Emergency Actions
    EmergencyAct1 = EmergencyAction1(
        name='EmergencyAction1')

    # Operaterational Branch Sequence:
    OperationalSequence = py_trees.composites.Sequence(
        name="Operational",
    )

    LocationUnknownSequence = py_trees.composites.Sequence(
        name='Location',
    )

    LocationUnknownCondition = LocationUnknown(
        name="Location unknown?")

    SurfaceSearchSequence = py_trees.Sequence(
        name="SurfaceSearch"
        )

    SubmergedSearchSequence = py_trees.Sequence(
        name="Submerged Search"
    )

    MesobotOnSurface = MesobotOnSurfaceCondition(
        name = "Is Mesobot On Surface?"
    )

    SearchForMesobotOnSurface = MesobotSurfaceSearchAction(
        name="Search for Mesobot On Surf.",
        action_namespace = 'navigator/run_tasks',
        action_spec = RunTasksAction,
        action_goal = RunTasksGoal()
    )

    SearchForMesobotSubmerged = SubmergedSearchAction(
        name="Search for Mesobot Submerged"
    )

    AcousticSurveySequence = py_trees.Sequence(
        name = 'Acoustic Survey'
    )


    AcousticSurveyIncomplete = AcousticSurveyIncompleteCondition(
        name="Acoustic Survey Incomplete?"
    )
    DoEK80Survey = EK80SurveyAction(
        name="Do EK80 Survey."
    )

    MesobotNotInPositionSequence = py_trees.Sequence(
        name="MesobotNotInPosition"
    )

    MesobotNotInPosition = MesobotNotInPositionCondition(
        name="Is Mesobot not in the right position?"
    )
    RepositionMesobot = RepositionMesobotAction(
        name="Reposition Mesobot."
    )

    # Dive plan
    MesobotDivePlanSequence = py_trees.Sequence(
        name="MesobotDivePlanSequence"
    )

    ExecuteDivePlan = ExecuteMesobotDivePlanAction(
        name="Execute Mesobot Dive Plan"
    )

    #############################################################
    # Assemble the Behavior Tree:
    #############################################################

    # Build the behavior tree:
    root.add_children([data2bb,OperationalSequence])
    #data2bb.add_children([behavior2BB, asvpos2BB,
    #                      mesobotpos2BB])
    data2bb.add_children([beh2BB])

    OperationalSequence.add_children([Enable,InvertEmergencyCondition,
                                    LocationUnknownSequence,
                                    AcousticSurveySequence,
                                    MesobotNotInPositionSequence,
                                    MesobotDivePlanSequence
    ])
    # When the's no emergency condition, we want the operational sequence to
    # continue to run, so we invert it here.
    EmergencySequence.add_children([EmergencyCondition,
                                    EmergencyActionSequence])
    EmergencyActionSequence.add_child(EmergencyAct1)
    LocationUnknownSequence.add_children([LocationUnknownCondition,
                                        SurfaceSearchSequence,
                                        SubmergedSearchSequence])
    SurfaceSearchSequence.add_children([MesobotOnSurface,
                                        SearchForMesobotOnSurface])
    SubmergedSearchSequence.add_child(SearchForMesobotSubmerged)
    AcousticSurveySequence.add_children([AcousticSurveyIncomplete,
                                        DoEK80Survey])
    MesobotNotInPositionSequence.add_children([MesobotNotInPosition,
                                            RepositionMesobot])

    MesobotDivePlanSequence.add_child(ExecuteDivePlan)

    behavior_tree = py_trees_ros.trees.BehaviourTree(root)

    #py_trees.display.ascii_tree(root)

    return behavior_tree

