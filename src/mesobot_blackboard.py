import rospy
import py_trees
import py_trees_ros
import threading
from project11_msgs.msg import BehaviorInformation
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
import time

bhv_bb = py_trees.blackboard.Blackboard()

# Set up some test variables to artificaily set states for debugging.
testvalues = {'debug': True,
              'emergency': False,
              'location_unknown':True,
              'on_surface': True}

for k,v in testvalues.items():
    bhv_bb.set('test.'+k,v, overwrite=True)

# This class inherits the py_trees_ros.subscribers.ToBlackboard object, 
# allowing one to create a blackboard separately and provide it, rather 
# than creating one on the fly. In this version of py_trees/py_trees_ros
# there is no Client object for access the blackboard created with the 
# standard ToBlackboard() object. 
class ToLocalBlackboard(py_trees_ros.subscribers.ToBlackboard):
    def __init__(self,blackboard,**kwargs):
        super(ToLocalBlackboard,self).__init__(**kwargs)
        self.ourblackboard = blackboard

    def setup(self, timeout):
        ret = super(ToLocalBlackboard,self).setup(timeout)
        # Resets the blackboard to our blackboard.
        self.blackboard = self.ourblackboard
        return ret
    
    def update(self,**kwargs):
        ret = super(ToLocalBlackboard,self).update(**kwargs)
        if self.name=='behaviorparam2BB':
            print(self.subscriber.get_num_connections())
            print(self.msg)
            print(self.topic_name)
            print(self.name + ":" + self.feedback_message)
            print(ret)
        #ret = py_trees.common.Status.SUCCESS
        return ret


class ToBB(py_trees.behaviour.Behaviour):
    ''' A class to write data from ROS messages to the py_trees blackboard.

    TODO: This was written out of frustration (see NOTE), and has all the 
    subscribers hard-coded. It should be rewritten, submitting topics and 
    message types to __init__().
    
    NOTE:
    This Behavior combines all the data acquisiton subscribers into one behavior 
    writing their data to the blackboard. In this version of py_trees_ros/py_trees
    every attempt to use ToBlackboard() above with multiple behaviors, one for each
    subscriber failed. Data would be written by the fast data rate topics but omitted
    for the slow data rate topics. If the slow data rate topic is placed first in the 
    sequence, no data would be written to the blackbaord at all. The cause for these
    problems might be in the architecture of the data acquisition branch of the tree
    (sequential vs parallel elements, memory=True/False, etc.), but every attempt 
    resulted in failed attempts to get data into the blackboard reliably. My 
    suspicion is that the thread lock (self.wireguard below) might have been
    monopolized by the high data rate topics and by combining the posting to the
    blackboard into a single threadlock here, the problem is solved. Not sure.
'''
    def __init__(self, blackboard,**kwargs):

        super(ToBB,self).__init__(**kwargs)
        # Debugging tool.
        self.feedback_messages = {'bhvinfo':None,
                                'mesopos':None,
                                'asvinfo':None}
        # Place where messages received by subscribers are held.
        self.msgs = {'bhvinfo':None,
                     'mesopos':None,
                     'asvinfo':None}
        
        self.blackboard = blackboard
        self.data_guard = threading.Lock()

        self.input_subscriber = None
        self.mesobot_subscriber = None
        self.asv_subscriber = None

        # Will get the whole message
        self.blackboard_variable_mapping = {"blackboard_variables":None}
        self.clearing_policy = None

    def setup(self,timeout):
        self.input_subscriber = rospy.Subscriber('project11/behaviors/mesobot/input',
                                      BehaviorInformation,
                                      self.inputCB,
                                      queue_size=10)
        self.mesobot_subscriber = rospy.Subscriber('/project11/mesobot/nav/position',
                                                   GeoPoseStamped,
                                                   self.mesoCB,
                                                   queue_size=10)
        self.asv_subscriber = rospy.Subscriber('project11/odom',
                                               Odometry,
                                               self.asvCB,
                                               queue_size=10)
        
        '''
        # Sets an empty set of data on setup.
        self.msgs['bhvinfo'] = BehaviorInformation()
        self.msgs['mesoinfo'] = GeoPoseStamped()
        self.msgs['asvinfo'] = Odometry()
        self.update()
        '''
        

        return True

    def inputCB(self,msg):
        #with self.data_guard:
        #    self.msgs['bhvinfo'] = msg
        self.msgs['bhvinfo'] = msg
        
    def mesoCB(self,msg):
        #with self.data_guard:
        #    self.msgs['mesoinfo'] = msg
        self.msgs['mesopos'] = msg

    def asvCB(self,msg):
        #with self.data_guard:
        #    self.msgs['asvinfo'] = msg
        self.msgs['asvinfo'] = msg
    
    def update(self):
        
        haveMsgs = False
        for k, msg in self.msgs.items():
            if msg is not None:
                haveMsgs = True
        if not haveMsgs:
            return py_trees.common.Status.RUNNING
        
        
        # Look at self.msgs for new items.
        with self.data_guard:
            for kk, msg in self.msgs.items():
                #print(kk)
                #print(msg)

                # This code is directly from py_trees_ros/subscribers.py with only
                # msg = self.msg changed. It should apply the messages to the
                # blackboard within one "data_guard" mutex rather than individual ones.
                if msg is None:
                    self.feedback_message = "no " + kk + " message received yet"
                    #return py_trees.common.Status.RUNNING
                else:
                    
                    self.blackboard.set(kk, msg, overwrite=True)
                    self.feedback_message = " saved incoming message: " + kk
                    #if kk == 'bhvinfo':
                    #    print(msg)
                    # this is of dubious worth, since the default setting of ClearingPolicy.ON_INITIALISE
                    # covers every use case that we can think of.
                    if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
                        msg = None
                    msg = None
                        
                #print(time.asctime() + self.feedback_message)

        return py_trees.common.Status.SUCCESS
