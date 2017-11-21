#!/usr/bin/env python 

import os
import rospy
import pickle
import numpy as np
import rospkg
import copy
from dmp.msg import *
from dmp.srv import *
from tf.transformations import *
from geometry_msgs.msg import *
from ar_track_alvar_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from hlpr_manipulation_utils.arm_moveit2 import *
from cartesian_trajectory_planner.trajopt_planner import *
from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

plot = False
goal_pose = geometry_msgs.msg.PoseStamped()
demo_quat = Quaternion(0,0,0,1)

def arPoseMarkerCallback(msg):
    global goal_pose
    global demo_quat

    if(len(msg.markers)>0):
        # marker Pose
        marker_pose_stamped = msg.markers[0].pose
        goal_pose = copy.deepcopy(marker_pose_stamped)
        goal_pose.pose.position.y += 0.4
        goal_pose.pose.orientation = demo_quat

def _nearest_equivalent_angle(desired, current):
    previous_rev = floor(current/(2*pi))
    next_rev = ceil(current/(2*pi))
    
    if fabs(current - previous_rev*2*pi) < fabs(current - next_rev*2*pi):
        current_rev = previous_rev
    else:
        current_rev = next_rev
        
    # Determine closestAngle
    low_val = (current_rev - 1)*2*pi + desired
    med_val = current_rev*2*pi + desired
    high_val = (current_rev + 1)*2*pi + desired
    
    if (fabs(current - low_val) <= fabs(current - med_val) and fabs(current - low_val) <= fabs(current - high_val)):
        return low_val
    elif (fabs(current - med_val) <= fabs(current - low_val) and fabs(current - med_val) <= fabs(current - high_val)):
        return med_val
    else:
        return high_val 


def _nearest_equivalent_joints(desired, current):
    return [_nearest_equivalent_angle(desired[i], current[i]) for i in range(len(desired))]

def _nearest_equivalent_joints_array(traj):
    new_traj = [traj[0]]
    for i in range(1,len(traj)):
        new_traj.append(_nearest_equivalent_joints(traj[i], new_traj[i-1]))
        #print "desired =", traj[i]
    #raw_input('')
    return np.array(new_traj)
    #return np.array([traj[0]] + [_nearest_equivalent_joints(traj[i], traj[i-1]) for i in range(1,len(traj))])

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;

#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;


def joint_array_to_traj(joint_array):
    traj = JointTrajectory()
    # traj.header.frame_id = frame_id
    traj.joint_names = ['right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6', 'right_joint_7']
    traj.points = []
    for i, joint in enumerate(joint_array):
        pt = JointTrajectoryPoint()
        pt.positions = joint
        traj.points.append(pt)

    return traj

def joint_traj_to_pose_array(traj, arm):
    ee_traj = []
    for i, pt in enumerate(traj.points):
        ee_pose = arm.get_FK(root='right_link_base', req_frame='right_link_7', joints=pt.positions)[0]
        ee_pose.header.seq = i
        ee_pose.header.stamp = pt.time_from_start.to_sec()
        ee_traj.append(copy.deepcopy(ee_pose))

    # print 'ee_traj', ee_traj
    return ee_traj

def process_for_dmp(traj):
    new_traj = []
    for pt in traj:
        if type(pt) is PoseStamped:
            pt = pt.pose
        euler = euler_from_quaternion([pt.orientation.x, pt.orientation.y, pt.orientation.z, pt.orientation.w])
        new_traj.append([pt.position.x, pt.position.y, pt.position.z] + list(euler))
    return new_traj

def process_for_dmp_quat(traj):
    new_traj = []
    for pt in traj:
        if type(pt) is PoseStamped:
            pt = pt.pose
        new_traj.append([pt.position.x, pt.position.y, pt.position.z, pt.orientation.x, pt.orientation.y, pt.orientation.z, pt.orientation.w])
    return new_traj

def process_for_playback(traj):
    new_traj = []
    points = traj.plan.points
    for pt in points:
        quat = quaternion_from_euler(pt.positions[3], pt.positions[4], pt.positions[5])
        pose = Pose()
        pose.position.x = pt.positions[0]
        pose.position.y = pt.positions[1]
        pose.position.z = pt.positions[2]
        pose.orientation.x = quat[0] 
        pose.orientation.y = quat[1] 
        pose.orientation.z = quat[2] 
        pose.orientation.w = quat[3] 
        new_traj.append(pose)
    return new_traj

def process_for_playback_quat(traj):
    new_traj = []
    points = traj.plan.points
    for pt in points:
        quat = np.array([pt.positions[3], pt.positions[4], pt.positions[5], pt.positions[6]])
        quat /= np.linalg.norm(quat)
        pose = Pose()
        pose.position.x = pt.positions[0]
        pose.position.y = pt.positions[1]
        pose.position.z = pt.positions[2]
        pose.orientation.x = quat[0] 
        pose.orientation.y = quat[1] 
        pose.orientation.z = quat[2] 
        pose.orientation.w = quat[3] 
        #pose.orientation = demo_quat
        new_traj.append(pose)
    return new_traj

def dmp_plan_to_array(traj):
    return [pt.positions for pt in traj.plan.points]

def merge_plans(plans):
    joint_traj = copy.deepcopy(plans[0].joint_trajectory)
    for i in range(1,len(plans)):
        joint_traj.points.extend(plans[i].joint_trajectory.points)
    return joint_traj

def goto_start(arm):
    start = [1.026, 4.847, 0.114, 4.884, 3.77, 4.479, -0.117]
    arm.move_to_joint_pose(start)

def plot_traj(traj, title):
    plt.figure()
    joints = ['joints_'+str(i) for i in range(1,8)]
    plt.plot(traj)
    plt.legend(joints)
    plt.title(title)

def plot_cart_trajs(trajs):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for traj in trajs:
        traj = np.array(traj)
        plt.plot(traj[:,0], traj[:,1], traj[:,2])

def plot_traj_err(traj_ref, traj, legend, title):
    plt.figure()
    err = np.array(traj) - np.array(traj_ref)
    plt.plot(err)
    plt.legend(legend)
    plt.title(title)

def main():
    global goal_pose
    global demo_quat

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, arPoseMarkerCallback)

    arm = ArmMoveIt(planning_frame='right_link_base', _arm_name='right')

    #goto_start(arm)

    rospack = rospkg.RosPack()
    cwd = '/home/vector/data/'

    filename = raw_input('Enter the unique (before _joint.pkl) name of the file: ')

    playback_file = cwd + filename + '_joint.pkl'

    print "Playing back trajectory file: ", playback_file

    #playback_traj = joint_traj_to_pose_array(pickle.load(open(playback_file))['full_plan'][0].joint_trajectory, arm)
    playback_traj = joint_traj_to_pose_array(merge_plans(pickle.load(open(playback_file))['full_plan'][1:]), arm)
    demo_quat = playback_traj[-1].pose.orientation
    #demo_quat = playback_traj[0].pose.orientation

    rospy.sleep(2)

    # Going to the initial point of the trajectory
    #start_pt = playback_traj[0]
    #print "start_pt", start_pt
    #raw_input( "Going to the start point of the recorded trajectory, Press Enter!")
    #arm.move_to_ee_pose(start_pt)
    #rospy.sleep(1)

    # learn dmp
    #dims = 6
    dims = 7
    dt = 1.0                
    K = 2500                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 10
    traj = process_for_dmp_quat(playback_traj)
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases) 

    makeSetActiveRequest(resp.dmp_list)

    # set new goal
    #x_0 = process_for_dmp([arm.get_FK(root='right_link_base', req_frame='right_link_7', joints=arm.get_current_pose())[0]])[0]
    x_0 = process_for_dmp_quat([arm.get_FK(root='right_link_base', req_frame='right_link_7', joints=arm.get_current_pose())[0]])[0]
    x_dot_0 = 7*[0.0]   
    t_0 = 0                
    #goal = process_for_dmp([goal_pose.pose])[0]
    #goal = process_for_dmp_quat([goal_pose.pose])[0]
    #goal = process_for_dmp([playback_traj[-1]])[0] 
    goal = process_for_dmp_quat([playback_traj[-1]])[0] 
    #goal[0] -= 0.45
    #goal_thresh = [0.05, 0.05, 0.05, 0.1, 0.1, 0.1]
    goal_thresh = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.01]
    seg_length = -1          #Plan until convergence to goal
    tau = 1*resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)
    #playback_plan = process_for_playback(plan)
    playback_plan = process_for_playback_quat(plan)
    print len(playback_plan)
    print playback_plan
    print "Goal Pose:", goal_pose.pose
    print "Demo Quat:", demo_quat
    print "Goal Euler:", goal

    urdf = 'package://cartesian_trajectory_planner/urdf/jaco_7dof.urdf'
    srdf = 'package://cartesian_trajectory_planner/urdf/jaco_7dof.srdf'

    cart_planner = TrajOptCartesianPlanner(urdf, srdf, viz=False)
    current_pose = arm.get_current_pose()
    cart_planner.setDOFs(current_pose)

    #cal_traj = cart_planner.plan('right_arm', playback_traj, link='right_link_7')
    cal_traj = cart_planner.plan('right_arm', playback_plan, link='right_link_7')
    #cal_traj = _nearest_equivalent_joints_array(cal_traj)

    if plot:
        plot_traj(cal_traj, '')
    joint_traj = joint_array_to_traj(cal_traj.tolist())

    if plot:
        dmp_plan = dmp_plan_to_array(plan)
        plan_traj = process_for_dmp_quat([arm.get_FK(root='right_link_base', req_frame='right_link_7', joints=joint_state)[0] for joint_state in cal_traj])
        #plot_traj_err(traj[1:], dmp_plan, ['x','y','z','qx','qy','qz','qw'], 'DMP Plan Error')
        #plot_traj_err(dmp_plan, plan_traj[1:], ['x','y','z','qx','qy','qz','qw'], 'TrajOpt Plan Error')  
        plot_cart_trajs([dmp_plan, plan_traj])

    if plot:
        plt.show()
    
    raw_input('Playing back the trajectory, Press Enter!')

    arm.execute_joint_trajectory(joint_traj)
    #print len(playback_plan)
    #print joint_traj
    #print "Goal Pose:", goal_pose.pose
    #print "Demo Quat:", demo_quat
    #print "Goal Euler:", goal

    # Shutdown
    print "shutting down!"
    # shut down moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__=='__main__':
    rospy.init_node('demo_manipulation', anonymous=True)
    main()
    # rospy.spin()
    
