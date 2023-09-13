import py_trees
from mesobot_blackboard import bhv_bb

class MesobotNotInPositionCondition(py_trees.behaviour.Behaviour):
    '''Behavior to determine if Mesobot Location is known.'''
    def __init__(self,name):
        super(MesobotNotInPositionCondition,self).__init__(name)
        self.blackboard = bhv_bb
        
    def setup(self,timeout):
        return True
    def initialise(self):
        pass

    def update(self):
        '''Determine if the acoustic survey is complete.'''

        self.logger.debug("  %s [MesobotNotInPositionCondition::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self,new_status):
        pass