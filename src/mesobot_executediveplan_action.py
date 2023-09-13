import py_trees
from mesobot_blackboard import bhv_bb

class ExecuteMesobotDivePlanAction(py_trees.behaviour.Behaviour):
    '''Behavior to determine if Mesobot Location is known.'''
    def __init__(self,name):
        super(ExecuteMesobotDivePlanAction,self).__init__(name)
        self.blackboard = bhv_bb
        
    def setup(self,timeout):
        return True
    def initialise(self):
        pass

    def update(self):
        '''Execute Mesobot Dive Plan.'''

        self.logger.debug("  %s [ExecuteMesobotDivePlanAction::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self,new_status):
        pass