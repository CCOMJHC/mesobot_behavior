import py_trees
from mesobot_blackboard import bhv_bb

class AcousticSurveyIncompleteCondition(py_trees.behaviour.Behaviour):
    '''Behavior to determine if the acoustic survey is compelete.'''
    def __init__(self,name):
        super(AcousticSurveyIncompleteCondition,self).__init__(name)
        self.blackboard = bhv_bb

    def setup(self,timeout):
        return True
    def initialise(self):
        pass

    def update(self):
        '''Determine if the acoustic survey is complete.'''

        self.logger.debug("  %s [AcousticSurveyIncompleteCondition::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self,new_status):
        pass