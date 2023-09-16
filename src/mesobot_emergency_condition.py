import rospy
import py_trees
from mesobot_blackboard import bhv_bb
from project11_msgs.msg import BehaviorInformation

# Emergnecy Condition
class Emergency(py_trees.behaviour.Behaviour):
    def __init__(self,name,feedback):
        super(Emergency,self).__init__(name)
        self.blackboard = bhv_bb
        self.feedback_pub = feedback
        self.is_emergency = False

    def setup(self,timeout):
        '''How do we change the emergency status?'''
        feedback = BehaviorInformation()
        feedback.data = "Setup Emergency Condition."
        self.feedback_pub.publish(feedback)

        return True
    
    def initialise(self):
        pass

    def update(self):
        if self.blackboard.get('test.debug'):
            self.is_emergency = self.blackboard.get('test.emergency')
        if self.is_emergency:
            self.logger.debug("  %s [Emergency::update()]" % self.name)
            rospy.loginfo("Mesobot Emergency")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    def terminate(self,new_status):
        pass