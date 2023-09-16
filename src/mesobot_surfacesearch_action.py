import rospy
import py_trees
import py_trees_ros
import actionlib
from geometry_msgs.msg import PoseStamped
from mesobot_blackboard import bhv_bb
from project11_nav_msgs.msg import TaskInformation
from project11_navigation.msg import RunTasksGoal
import project11
import yaml
import copy

class MesobotSurfaceSearchAction(py_trees_ros.actions.ActionClient):
    '''Behavior to Search for Mesobot when he's on the surface.
    
    This Behavior sends a search task to the navigator. It returns
    RUNNING while the navigator is navigating and then FAILURE when
    it's compelete. It will never return SUCCESS, unless Mesobot's 
    location is known, but that is handled by the "Location Unknown?"
    condition. 

    '''
    def __init__(self,**kwargs):
        super(MesobotSurfaceSearchAction,self).__init__(**kwargs)
        self.blackboard = bhv_bb
        #self.act_client = actionlib.SimpleActionClient('navigator/run_tasks', 
        #                                                     project11_navigation.msg.RunTasksAction)
        self.earth = project11.nav.EarthTransforms()
        self.sent_goal = False
        self.update_value = py_trees.common.Status.FAILURE
    '''    
    def setup(self,timeout):
        return True
    '''
    def initialise(self):

        self.action_goal = RunTasksGoal()
        # Send search task to Navigator here.
        task = TaskInformation()
        task.type = "survey_line"
        task.id = "mesobot_surface_search"
        task.priority = 3
        try:
            task.data = yaml.safe_dump({'speed': self.blackboard.bhvinfo.search_speed_kts * .0514444})
        except:
            task.data = yaml.safe_dump({'speed': "8.0"})

        # Dummy search code to test functionality.
        pt1 = PoseStamped()
        pt1.header.stamp = rospy.Time.now()
        try:
            pt1.header.frame_id = self.blackboard.asvinfo.header.frame_id
        except:
            rospy.loginfo('Failed to get reference frame for vehicle.')
            return py_trees.common.Status.FAILURE
        # Search at mesobot's last known location...
        pt1.pose = self.blackboard.mesoinfo.pose.pose
        task.poses.append(pt1)
        pt2 = PoseStamped()
        pt2.header = copy.deepcopy(pt1.header)
        pt2.pose = copy.deepcopy(self.blackboard.mesoinfo.pose.pose)
        pt2.pose.position.x = pt2.pose.position.x + 100
        pt2.pose.position.y = pt2.pose.position.y
        task.poses.append(pt2)
        pt3 = copy.deepcopy(pt2)
        pt3.pose.position.y = pt2.pose.position.y + 100
        task.poses.append(pt3)

        self.action_goal.tasks = [task]

                
    def update(self):
        '''Search for Mesobot.'''

         # TODO: Check for new search location in blackboard and reset goal.

        if self.sent_goal is False:
            rospy.loginfo("mesobot: sending surface search goal")
            rospy.loginfo(self.action_goal)

            self.action_client.send_goal(self.action_goal,
                        active_cb = self.navigatorActiveCallback,
                        feedback_cb = self.navigatorFeedbackCallback,
                        done_cb = self.navigatorDoneCallback)
            self.sent_goal = True
            return py_trees.common.Status.RUNNING

        self.logger.debug("  %s [SurfaceSearchAction::update()]" % self.name)

        # Return status is generally set in the search action callbacks.
        return self.update_value
    
    def navigatorActiveCallback(self):
        rospy.loginfo("navigator active - searching for mesobot on surface")
        
    def navigatorFeedbackCallback(self,data):

        # If the feedback doesn't have our tasks in the list, then either
        # our task was aborted or maybe the task completed and we missed it.
        if 'mesobot_surface_search' not in [t.id for t in data.tasks]:
                data.update_value = py_trees.common.Status.FAILURE
                self.sent_goal = False
                rospy.loginfo('surface search complete or aborted')

        # Otherwise if the task is still underway, set RUNNING, otherwise
        # set "FAILURE" until we get a new task or until location is known.
        for task in data.tasks:
            if task.id == 'mesobot_surface_search':
                if task.done == False:
                    self.update_value = py_trees.common.Status.RUNNING
                    rospy.rosloginfo('mesobot: surface search underway.')
                else:
                    data.update_value = py_trees.common.Status.FAILURE
                    self.sent_goal = False
                    rospy.loginfo('mesobot: surface search complete')

        pass

    def navigatorDoneCallback(self):
        rospy.loginfo("mesobot surface seach complete - extending search..")

        # Pretend the location is now known:
        bhv_bb.set('test.location_known',True)

        # Or hover:
        self.action_goal = RunTasksGoal()
        # Send search task to Navigator here.
        task = TaskInformation()
        task.type = "hover"
        task.id = "hover"
        task.priority = 3
        try:
            task.data = yaml.safe_dump({'speed': self.blackboard.bhvinfo.search_speed_kts * .0514444})
        except:
            task.data = yaml.safe_dump({'speed': "4.0"})

        print(self.blackboard.mesoinfo.pose.pose)
        # Dummy search code to test functionality.
        pt1 = PoseStamped()
        pt1.header.stamp = rospy.Time.now()
        try:
            pt1.header.frame_id = self.blackboard.asvinfo.header.frame_id
        except:
            rospy.loginfo('Failed to get reference frame for vehicle.')
            return py_trees.common.Status.FAILURE
        pt1.pose = self.blackboard.mesoinfo.pose.pose
        task.poses.append(pt1)

        # Send hover goal here...

        return 

    def terminate(self,new_status):
        pass  