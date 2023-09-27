import py_trees

from mesobot_blackboard import bhv_bb

class MesobotOnSurfaceCondition(py_trees.behaviour.Behaviour):
    '''Behavior to determine if Mesobot Location is known.'''
    def __init__(self,name):
        super(MesobotOnSurfaceCondition,self).__init__(name)
        self.blackboard = bhv_bb
        
    def setup(self,timeout):
        return True
    def initialise(self):
        pass

    def check_mesobot_location(blackboard):
        pass

    def update(self):
        '''Determine if mesobot is expected to be on the surface.'''

        self.logger.debug("  %s [MesobotOnSurfaceCondition::update()]" % self.name)
        if self.blackboard.get('test.debug'):
            if self.blackboard.get('test.on_surface'):
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        
        # Put real code here.
        mesobot_position = self.blackboard.get("mesopos")
        if mesobot_position is None:
            return py_trees.common.Status.SUCCESS
        depth = -mesobot_position.pose.position.altitude
        if depth > self.blackboard.get("minimum_depth_considered_submerged"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


        
    
    def terminate(self,new_status):
        pass