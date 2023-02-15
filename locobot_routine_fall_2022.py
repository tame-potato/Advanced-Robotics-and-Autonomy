import math
import time
import roslib
import rospy
import actionLib
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250 use_perception:=true'
# Then change to this directory and type 'python pick_place_no_armtag.py'

#Move the robot to the given pose
def Move2Goal(x,y,w):
    # setup move_base action server client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    # setup current move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    
    # send goal to move_base action server
    client.send_goal(goal)
    
    # waits until the action is complete
    wait=client.wait_for_result()
    
    # checks if a result was received successfuly
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


#Pick the given cluster
def Pick(bot, cluster):
    # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link'
    # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    
    # move the robot back so it's centered and open the gripper
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.gripper.open()

    x, y, z = cluster["position"]
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
    bot.gripper.close()
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    
# Place at the given position and return the arm to sleep
def Place(bot, x, y, z):
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
    bot.gripper.open()
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':

    # Node initialization
    rospy.init_node('movebase_client_py')

    # bot setup
    bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
    
    # Move to position
    Move2Goal()
    
    # Move camera
    bot.camera.pan_tilt_move(0, 0.75)
    
    # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link'
    # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    
    # If clusters found successfully pick first from the left
    if success and len(clusters) > 0:
    	#Pick the item
    	Pick(bot, clusters[0])
    	
    	#Move arm to neutral resting position for trip
    	bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    else:
    	rospy.loginfo("Nothing to pick up!")
    	return
    	
    # Move camera back to good vantage point for movement
    bot.camera.pan_tilt_move(0, 0.75)
    
    # Move to second position
    Move2Goal()
    
    # Place the object at a fixed set of coordinates
    Place(bot, )
