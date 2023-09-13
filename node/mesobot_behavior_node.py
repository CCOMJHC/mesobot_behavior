#!/usr/bin/env python3

import rospy
import Mesobot
import functools
import py_trees
import sys

behavior_tree = Mesobot.CreateBehaviorTree()

rospy.init_node("MesobotBehavior")
rospy.on_shutdown(functools.partial(Mesobot.shutdown,behavior_tree))

if not behavior_tree.setup(timeout=15):
    py_trees.console.logerror("failed to setup the tree, aborting.")
    sys.exit(1)

print(py_trees.display.ascii_tree(behavior_tree.root))
behavior_tree.tick_tock(500)