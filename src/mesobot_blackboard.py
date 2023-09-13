import py_trees
import py_trees_ros
from nav_msgs.msg import Odometry

bhv_bb = py_trees.blackboard.Blackboard()

# This class inherits the py_trees_ros.subscribers.ToBlackboard object, 
# allowing one to create a blackboard separately and provide it, rather 
# than creating one on the fly. In this version of py_trees/py_trees_ros
# there is no Client object for access the blackboard created with the 
# standard ToBlackboard() object. 
class ToLocalBlackboard(py_trees_ros.subscribers.ToBlackboard):
    def __init__(self,blackboard,**kwargs):
        super(ToLocalBlackboard,self).__init__(**kwargs)
        self.ourblackboard = blackboard

    def setup(self, timeout):
        ret = super(ToLocalBlackboard,self).setup(timeout)
        # Resets the blackboard to our blackboard.
        self.blackboard = self.ourblackboard
        return ret
    
