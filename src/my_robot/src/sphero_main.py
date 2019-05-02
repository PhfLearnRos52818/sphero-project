#! /usr/bin/env python

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from my_robot.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from cmd_vel_pub import CmdVelPub
from odometry_analysis import OdometryAnalysis
from odometry_analysis import check_if_out_maze

class ControlSphero(object):
    def __init__(self, goal_distance):
        self._goal_distance = goal_distance
        self.init_direction_service_client()
        self.init_rec_odom_action_client()
        self.init_move_sphero_publisher()
        self.crashed = False
        
    def init_direction_service_client(self, service_name = "/crash_direction_service"):
        rospy.loginfo('Waiting for Service Server')
        rospy.wait_for_service(service_name) # wait for the service client /gazebo/delete_model to be running
        rospy.loginfo('Service Server Found...')
        self._direction_service = rospy.ServiceProxy(service_name, Trigger) # create the connection to the service
        self._request_object = TriggerRequest()
        
    def make_direction_request(self):
        
        result = self._direction_service(self._request_object) # send the name of the object to be deleted by the service through the connection
        self.crashed = result.success
        return result.message
    
    def init_rec_odom_action_client(self):
        self._rec_odom_action_client = actionlib.SimpleActionClient('/rec_odom_as', record_odomAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for action Server')
        self._rec_odom_action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        self._rec_odom_action_goal = record_odomGoal()
    
    def send_goal_to_rec_odom_action_server(self):
        self._rec_odom_action_client.send_goal(self._rec_odom_action_goal, feedback_cb=self.rec_odom_feedback_callback)
        
    def rec_odom_feedback_callback(self,feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))
    
    def rec_odom_finished(self):
        
        has_finished = ( self._rec_odom_action_client.get_state() >= 2 )
        
        return has_finished
    
    def get_result_rec_odom(self):
        return self._rec_odom_action_client.get_result()
        
    def init_move_sphero_publisher(self):
        self._cmdvelpub_object = CmdVelPub()

    def move_sphero(self, direction):
        self._cmdvelpub_object.move_robot(direction)

    def got_out_maze(self, odom_result_array):
        return check_if_out_maze(self._goal_distance, odom_result_array)

    def is_crash(self):
        return self.crashed
  
rospy.init_node("sphero_main_node", log_level=rospy.INFO)
controlsphero_object = ControlSphero(goal_distance=2.0)
rate = rospy.Rate(10)

t = rospy.Duration(0.24, 0)
d = rospy.Duration(0.35, 0)
i = 1

controlsphero_object.send_goal_to_rec_odom_action_server()

while not controlsphero_object.rec_odom_finished():
    if controlsphero_object.is_crash():
        rospy.logwarn("="*20)
        #rospy.logwarn("CRASH! -> %"+str(i)*50)
        """if i != 4:
            controlsphero_object.move_sphero("backwards")
            rospy.sleep(d)"""

        controlsphero_object.move_sphero("stop")      
        rate.sleep()

        if i == 1:
            controlsphero_object.move_sphero("backwards")
            rospy.sleep(d)
            controlsphero_object.move_sphero("right")
            rospy.sleep(t)
        elif i == 2 :
            controlsphero_object.move_sphero("backwards")
            rospy.sleep(t)
            controlsphero_object.move_sphero("left")
            rospy.sleep(rospy.Duration(0.34, 0))
        elif i == 3:
            controlsphero_object.move_sphero("backwards")
            rospy.sleep(d)
            controlsphero_object.move_sphero("left")  
            rospy.sleep(rospy.Duration(0.24, 0))        
        elif i == 4:
            controlsphero_object.move_sphero("right")  
            rospy.sleep(rospy.Duration(0.25, 0))
        elif i == 5:
            controlsphero_object.move_sphero("forwards")
            rospy.sleep(rospy.Duration(0.25, 0))       
        else:
           pass
        
        i=i+1
        
        """controlsphero_object.move_sphero("stop")
        rate.sleep()"""
       
        """controlsphero_object.move_sphero("backwards")controlsphero_object.move_sphero("backwards")
            rospy.sleep(d)
        rate.sleep()

        controlsphero_object.move_sphero("stop")
        rate.sleep()

        controlsphero_object.move_sphero("right")"""
    
    direction_to_go = controlsphero_object.make_direction_request()
    rospy.loginfo(direction_to_go)
    controlsphero_object.move_sphero(direction_to_go)
    rate.sleep()


odom_result = controlsphero_object.get_result_rec_odom()
odom_result_array = odom_result.result_odom_array

if controlsphero_object.got_out_maze(odom_result_array):
    rospy.loginfo("Out of Maze")
else:
    rospy.loginfo("In Maze")

rospy.loginfo("Sphero Maze test Finished")
