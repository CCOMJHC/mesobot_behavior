import py_trees

# Emergency Action 1 (TBD)
class EmergencyAction1(py_trees.behaviour.Behaviour):
    '''First action in the event of an emergency.
    This action returns failure for now, which will 
    stop the behavior tree (I think)'''
    def __init__(self,blackboard=None, **kwargs):
        super(EmergencyAction1,self).__init__(**kwargs)
        self.blackboard = blackboard

    def setup(self,timeout):
        return True
    def initialise(self):
        pass
    def update(self):
        self.logger.debug("  %s [Emergency::update()]" % self.name)
        return py_trees.common.Status.RUNNING
    def terminate(self,new_status):
        pass