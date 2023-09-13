import py_trees
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

    def check_mesobot_location(blackboard):
        pass

    def update(self):
        '''Determine if the location of mesobot is known'''
        self.logger.debug("  %s [LocationUnknown::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self,new_status):
        pass