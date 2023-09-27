import rospy
import py_trees
import py_trees_ros
import tf2_ros
import actionlib
from geometry_msgs.msg import PoseStamped, PointStamped
from mesobot_blackboard import bhv_bb
from project11_nav_msgs.msg import TaskInformation
from project11_navigation.msg import RunTasksGoal
import project11
from mission_manager.srv import TaskManagerCmd, TaskManagerCmdRequest
from mission_manager import track_patterns

import yaml
import copy
import math

class MesobotSurfaceSearchAction(py_trees.behaviour.Behaviour):
    '''Behavior to conduct search for a mesobot on the surface.'''
    def __init__(self,name):
        super(MesobotSurfaceSearchAction,self).__init__(name)
        self.blackboard = bhv_bb

    def setup(self,timeout):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        return False

    def initialise(self):
        self.search_task = None

    def update(self):
        '''Determine if the location of mesobot is known'''

        self.logger.debug("  %s [MesobotSurfaceSearchAction::update()]" % self.name)

        mesobot_position = self.blackboard.get("mesobot_odom")
        if mesobot_position is not None and mesobot_position.header.stamp > rospy.Time.now() - rospy.Duration(self.blackboard.get("location_timeout")):
            return py_trees.common.Status.SUCCESS
        
        if self.search_task is None:
            behavior_task = self.blackboard.get("bhvinfo")
            
            speed = self.blackboard.get("surface_search_speed")
            spacing = self.blackboard.get("surface_search_spacing")

            start_position =  PoseStamped()
            start_position.header = behavior_task.poses[0].header


            if mesobot_position is not None:
                mesobot_point = PointStamped()
                mesobot_point.header = mesobot_position.header
                mesobot_point.point = mesobot_position.pose.pose.position
                mesobot_in_asv_frame = self.tfBuffer.transform(mesobot_point, behavior_task.poses[0].header.frame_id)
                start_position.pose.position = mesobot_in_asv_frame.point
            else:
                sum_x = 0.0
                sum_y = 0.0
                count = 0.0
                for pose in behavior_task.poses:
                    sum_x += pose.pose.position.x
                    sum_y += pose.pose.position.y
                    count += 1.0
                start_position.pose.position.x = sum_x/count
                start_position.pose.position.y = sum_y/count

            max_d2 = 0.0
            for pose in behavior_task.poses:
                dx = pose.pose.position.x - start_position.pose.position.x
                dy = pose.pose.position.y - start_position.pose.position.y
                d2 = dx*dx + dy*dy
                max_d2 = max(d2, max_d2)

            max_distance = math.sqrt(max_d2)

            search = track_patterns.ExpandingBoxSearch(startLocation = start_position, loopSpacing = spacing, searchSpeedKts = speed*1.94384, maxSearchRadius=max_distance)
            self.search_task = search.create(behavior_task.id+'/surface_search_line')

            rospy.wait_for_service('mission_manager/task_manager', 0.5)
            task_manager = rospy.ServiceProxy('mission_manager/task_manager', TaskManagerCmd)
            task_manager_request = TaskManagerCmdRequest()
            task_manager_request.command = "update"
            task_manager_request.tasks.append(self.search_task)
            task_manager.call(task_manager_request)

        return py_trees.common.Status.RUNNING
    
    def terminate(self,new_status):
        pass

class OldMesobotSurfaceSearchAction(py_trees_ros.actions.ActionClient):
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
        self.have_position = False
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
        if self.blackboard.get('mesobot_odom') is None:
            rospy.logwarn("mesobot_behavior; No Mesobot position for search.")
            self.have_position = False
            return py_trees.common.Status.FAILURE
        else:
            self.have_postion = True

        pt1 = self.earth.geoToPose(
            self.blackboard.mesopos.pose.position.latitude, 
            self.blackboard.mesopos.pose.position.longitude) 
        

        #pt1.pose = self.blackboard.mesoinfo.pose.pose
        task.poses.append(pt1)
        pt2 = PoseStamped()
        pt2.header = copy.deepcopy(pt1.header)
        pt2.pose = copy.deepcopy(pt1.pose)
        pt2.pose.position.x = pt2.pose.position.x + 100.0
        pt2.pose.position.y = pt2.pose.position.y
        task.poses.append(pt2)
        pt3 = copy.deepcopy(pt2)
        pt3.pose.position.y = pt2.pose.position.y + 100.0
        task.poses.append(pt3)

        self.action_goal.tasks = [task]

                
    def update(self):
        '''Search for Mesobot.'''

         # TODO: Check for new search location in blackboard and reset goal.

        #if not self.have_position:
        #    return py_trees.common.Status.FAILURE
        
        print(self.sent_goal)
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
                self.update_value = py_trees.common.Status.FAILURE
                self.sent_goal = False
                rospy.loginfo('surface search complete or aborted')

        # Otherwise if the task is still underway, set RUNNING, otherwise
        # set "FAILURE" until we get a new task or until location is known.
        for task in data.tasks:
            if task.id == 'mesobot_surface_search':
                if task.done == False:
                    self.update_value = py_trees.common.Status.RUNNING
                    rospy.loginfo('mesobot: surface search underway.')
                else:
                    data.update_value = py_trees.common.Status.FAILURE
                    self.sent_goal = False
                    rospy.loginfo('mesobot: surface search complete')

        pass

    def navigatorDoneCallback(self,status,result):
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

        # Dummy search code to test functionality.
        pt1 = PoseStamped()
        pt1.header.stamp = rospy.Time.now()
        try:
            pt1.header.frame_id = self.blackboard.asvinfo.header.frame_id
        except:
            rospy.loginfo('Failed to get reference frame for vehicle.')
            return py_trees.common.Status.FAILURE
        
        pt1.pose = self.blackboard.mesopos.pose.pose
        task.poses.append(pt1)

        # Send hover goal here...

        return 

    def terminate(self,new_status):
        pass  