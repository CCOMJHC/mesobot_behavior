import py_trees
from mesobot_blackboard import bhv_bb

class EK80SurveyAction(py_trees.behaviour.Behaviour):
    '''Behavior to conduct an EK80 Acoustic Survey for Mesobot.'''
    def __init__(self,name):
        super(EK80SurveyAction,self).__init__(name)
        self.blackboard = bhv_bb

    def setup(self,timeout):
        return True
    def initialise(self):
        pass

    def update(self):
        '''Do the Acoustic Survey.'''

        self.logger.debug("  %s [EK80SurveyAction::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self,new_status):
        pass