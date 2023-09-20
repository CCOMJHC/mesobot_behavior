#!/usr/bin/env python3

import rospy
import Mesobot
import functools
import py_trees
from project11_msgs.msg import BehaviorInformation
from project11_nav_msgs.msg import TaskInformation


class MesobotBehavior(object):
    def __init__(self):
        self.behavior_information = None

        self.feedback = None
        self.behavior_tree = None

        self.S = rospy.Subscriber('project11/behaviors/mesobot/input',
                        TaskInformation,
                        self.behaviorMsgCB,
                        queue_size=1)
        



    def behaviorMsgCB(self, data):

        # If the feedback publisher is up, then we've created the tree already
        # and this is an update to the behavior field. That update will be 
        # caught by the behavior tree itself and posted to the blackboard.
        if self.feedback is not None:
            return

        self.feedback = rospy.Publisher('project11/behaviors/mesobot/feedback',
                            BehaviorInformation,
                            queue_size=10)
        
        self.behavior_tree = Mesobot.CreateBehaviorTree(data,
                                                        self.feedback)

        if self.behavior_tree is None:
            rospy.loginfo("Behavior tree creation failed.")
            return
        if not self.behavior_tree.setup(timeout=15):
            py_trees.console.logerror("failed to setup the tree, aborting.")
        rospy.loginfo("Created mesobot behavior tree.")
        rospy.on_shutdown(functools.partial(Mesobot.shutdown,self.behavior_tree))

        print(py_trees.display.ascii_tree(self.behavior_tree.root))

        self.behavior_tree.tick_tock(500)
        
        #                             pre_tick_handler=self.updateBehaviorTree)
    
if __name__ == '__main__':
    rospy.init_node("MesobotBehavior")
    rospy.loginfo("Setting up Mesobot Behavior")
    bhv = MesobotBehavior()
    rospy.spin()


