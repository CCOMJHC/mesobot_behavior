import py_trees
from mesobot_blackboard import bhv_bb

class SubmergedSearchAction(py_trees.behaviour.Behaviour):
    '''Behavior to conduct search for a submerged mesobot.'''
    def __init__(self,name):
        super(SubmergedSearchAction,self).__init__(name)
        self.blackboard = bhv_bb

    def setup(self,timeout):
        return True

    def initialise(self):
        pass

    def update(self):
        '''Determine if the location of mesobot is known'''

        self.logger.debug("  %s [SubmergedSearchAction::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self,new_status):
        pass