import py_trees
import rospy
from mesobot_blackboard import bhv_bb

class LocationUnknown(py_trees.behaviour.Behaviour):
    '''Behavior to determine if Mesobot Location is known.'''
    def __init__(self,name):
        super(LocationUnknown,self).__init__(name)
        self.blackboard = bhv_bb
        
    def setup(self,timeout):
        return True
    def initialise(self):
        pass

    def mesobot_location_unknown(self):

        if self.blackboard.get("test.debug"):
            return self.blackboard.get("test.location_unknown")
        else:
            # Do it for reals.
            mesobot_position = self.blackboard.get("mesobot_odom")
            if mesobot_position is None:
                return True
            if mesobot_position.header.stamp < rospy.Time.now() - rospy.Duration(self.blackboard.get("location_timeout")):
                return True
        return False

    def update(self):
        '''Determine if the location of mesobot is known'''

        self.logger.debug("  %s [LocationUnknown::update()]" % self.name)
        if self.mesobot_location_unknown():
            bhv_bb.set('mesobot_location_unknown',True)
            return py_trees.common.Status.SUCCESS
        else:
            bhv_bb.set('mesobot_location_unknown',False)
            return py_trees.common.Status.FAILURE
    
    def terminate(self,new_status):
        pass