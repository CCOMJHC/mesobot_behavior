import py_trees
from mesobot_blackboard import bhv_bb

# Emergnecy Condition
class Emergency(py_trees.behaviour.Behaviour):
    def __init__(self,name):
        super(Emergency,self).__init__(name)
        self.blackboard = bhv_bb
        self.emergency_status = False

    def setup(self,timeout):
        '''How do we change the emergency status?'''
        return True
    
    def initialise(self):
        pass
    def update(self):
        if self.emergency_status:
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [Emergency::update()]" % self.name)
            return py_trees.common.Status.FAILURE
    def terminate(self,new_status):
        pass