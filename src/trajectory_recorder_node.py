#!/usr/bin/env python 

import rospy
import rospkg
import copy, pickle
import std_msgs
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from hlpr_manipulation_utils.arm_moveit2 import *

counter = 0
joint_traj = RobotTrajectory()
rospack = rospkg.RosPack()
cwd = rospack.get_path('kinesthetic_teaching_dmp') # Name the right package
arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')


def clean_joint_traj(msg):
	msg = copy.deepcopy(msg)
	for pt in msg.points:
		pt.velocities = []
		pt.accelerations = []
		pt.effort = []

	return msg


def traj_callback(msg):
	# joint_traj := RobotTrajectory msg
	global counter, cwd, joint_traj, arm

	# rospy.sleep(4) # Giving time to rviz to visualize
	joint_traj = copy.deepcopy(msg.trajectory[0])
	
	# Save joint trajectory
	joint_traj_saved = clean_joint_traj(joint_traj.joint_trajectory)

	# Create ee trajecotry
	ee_traj = []
	ref_frame = 'right_link_base'
	req_frame = 'right_link_7'
	for i, pt in enumerate(joint_traj_saved.points):
		ee_pose = arm.get_FK(root=ref_frame, req_frame=req_frame, joints=pt.positions)[0]
		ee_pose.header.seq = i
		ee_pose.header.stamp = pt.time_from_start.to_sec()
		ee_traj.append(copy.deepcopy(ee_pose))


	print "Recording Trajectories with Ref frame: ", ref_frame, "and eef_frame: ", req_frame	

	# Save the trajecotry
	filename = cwd + '/src/traj_data/demo_' + str(counter) + '.pickle'
	file = open(filename,'wb')
	pickle.dump(ee_traj, file) 
	file.close()

	counter += 1



if __name__=='__main__':
	rospy.init_node('eef_trajectory_recorder_node', anonymous=True)
	rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, traj_callback, queue_size = 1)
	rospy.spin()
	
