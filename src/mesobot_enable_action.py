import py_trees
from mesobot_blackboard import bhv_bb

# Enable Behavior Condition
class Enabler(py_trees.behaviour.Behaviour):
    def __init__(self,name):
        super(Enabler,self).__init__(name)
        self.enabled = True
        self.blackboard = bhv_bb

    def setup(self,timeout):
        '''Must return True/False, not sure why.'''
        return True
    
    def initialise(self):
        pass
    def update(self):
        # Get BB enabled status to set self.enabled.
        if self.enabled:
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [Enabler::update()]" % self.name)
            return py_trees.common.Status.FAILURE
    def terminate(self,new_status):
        pass