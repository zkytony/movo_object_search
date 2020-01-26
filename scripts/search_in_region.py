#!/usr/bin/env python
# Object search within a region
#
# NOTE: This script should be run in a Python2 shell
# which has access to ROS packages. It expects launch
# files that will serve the rosparams to be already
# running.

import argparse
import rospy
import time
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from aruco_msgs.msg import MarkerArray as ArMarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Point
import message_filters
import tf
import subprocess
import os
import yaml
import math
import json
from action.waypoint import WaypointApply
from action.head_and_torso import TorsoJTAS
from action.action_type import ActionType
from scipy.spatial.transform import Rotation as scipyR
from topo_marker_publisher import PublishTopoMarkers, PublishSearchRegionMarkers
from ros_util import get_param

# Start a separate process to run POMDP; Use the virtualenv
VENV_PYTHON = "/home/kaiyuzh/pyenv/py37/bin/python"
POMDP_SCRIPT = "/home/kaiyuzh/repo/3d-moos-pomdp/moos3d/robot_demo/build_pomdp.py"

def euclidean_dist(p1, p2):
    return math.sqrt(sum([(a - b)** 2 for a, b in zip(p1, p2)]))

def read_pose_msg(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    return (x,y,z,qx,qy,qz,qw)


def list_arg(l):
    return " ".join(map(str,l))

def execute_action(action_info,
                   robot_state,
                   last_action_observation):
    """Exxecute the action described in action_info.
    navclient (SimpleActionClient) used to send goal
        to the navigation components."""
    obs_info = {"action_type": action_info["type"]}
    if action_info["type"] == ActionType.TopoMotionAction:
        # check source
        cur_robot_pose = wait_for_robot_pose()
        goal_tolerance = get_param('goal_tolerance')
        _gap = euclidean_dist(cur_robot_pose[:3], action_info["src_pose"])
        if _gap > goal_tolerance:
            rospy.logwarn("Robot is at %s, far from topo node location %s (dist=%.3f > %.3f); But won't stop moving."\
                          % (str(cur_robot_pose[:3]), str(action_info["src_pose"]), _gap, goal_tolerance))
        
        # navigate to destination
        position = action_info["dst_pose"]  # x,y,z
        # default rotation is (0,0,0) --> will look "forward"
        orientation = (0,0,0,1)  # equivalent to (0,0,0) in euler.
        if WaypointApply(position, orientation).status == WaypointApply.Status.SUCCESS:
            # Successfully moved robot; Return an observation about the robot state.
            rospy.loginfo("APPLIED MOTION ACTION SUCCESSFULLY")
            obs_info["status"] = "success"
        else:
            rospy.logwarn("FAILED TO APPLY MOTION ACTION")
            obs_info["status"] = "failed"
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_height()  # provides z pose.
        obs_info["objects_found"] = robot_state["objects_found"]
        obs_info["camera_direction"] = None  # consistent with transition model.

    elif action_info["type"] == ActionType.TorsoAction:
        torso_height = wait_for_torso_height()
        desired_height = torso_height + action_info["displacement"]
        TorsoJTAS.move(desired_height, current_height=torso_height)

        # Verify
        actual_height = wait_for_torso_height()
        if abs(actual_height - desired_height) > 1e-2:
            rospy.logerr("Torso did not move to desired height. (Desired: %.3f ; Actual: %.3f)"
                         % (actual_height, desired_height))
        else:
            rospy.loginfo("Torso motion complete. Now torso at %.3f" % actual_height)
        # Get observation about robot state
        obs_info["status"] = "success"
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = actual_height  # provides z pose.
        obs_info["objects_found"] = robot_state["objects_found"]
        obs_info["camera_direction"] = None  # consistent with transition model.        

    elif action_info["type"] == ActionType.LookAction:
        rotation = action_info["rotation"]
        cur_robot_pose = wait_for_robot_pose()
        # Rotate the robot
        position = cur_robot_pose[:3]
        orientation = tuple(map(float, scipyR.from_euler("xyz", rotation, degrees=True).as_quat()))
        if WaypointApply(position, orientation).status == WaypointApply.Status.SUCCESS:
            # Successfully moved robot; Return an observation about the robot state.
            rospy.loginfo("Robot rotate successfully")
            obs_info["status"] = "success"
        else:
            rospy.logwarn("Failed to rotate robot")
            obs_info["status"] = "failed"

        # robot state            
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_height()  # provides z pose.
        obs_info["objects_found"] = robot_state["objects_found"]
            
        if obs_info["status"] == "success":
            # Project field of view; Start the point cloud processing script,
            # one with ar tag detection, one without. Upon publishing the
            # volumetric observation, these scripts will save the observation
            # to a file.
            rospy.loginfo("Projecting field of view; Processing point cloud.")
            start_time = rospy.Time.now()
            voxels_dir = os.path.dirname(get_param('observation_file'))
            vpath_ar = os.path.join(voxels_dir, "voxels_ar.yaml")
            vpath = os.path.join(voxels_dir, "voxels.yaml")
            start_pcl_process(save_path=vpath_ar,
                              detect_ar=True)
            start_pcl_process(save_path=vpath,
                              detect_ar=False)
            # wait until files are present
            wait_time = max(1, get_param('observation_wait_time') - 3)
            observation_saved = False
            while rospy.Time.now() - start_time < rospy.Duration(wait_time):
                if os.path.exists(vpath):
                    observation_saved = True
                    break
                rospy.loginfo("Waiting for voxel observation file.")
                rospy.sleep(1.0)

            rospy.loginfo("Voxel observation saved.")
            if os.path.exists(vpath_ar):
                # use ar tag detection observation
                rospy.loginfo("Using the voxels that may contain AR tag labels.")
                with open(vpath_ar) as f:
                    voxels = yaml.load(f)
            else:
                with open(vpath) as f:
                    voxels = yaml.load(f)
            obs_info["camera_direction"] = orientation  # consistent with transition model.
            obs_info["voxels"] = voxels # volumetric observation
        else:
            # robot didn't reach the desired rotation; no observation received
            obs_info["camera_direction"] = None  # consistent with transition model.
            obs_info["voxels"] = {}
            rospy.logerror("No observation received because desired rotation not reached.")


    elif action_info["type"] == ActionType.DetectAction:
        # Based on last observation, mark objects as detected
        last_action, last_observation = last_action_observation
        if last_action is not None and last_observation is not None:
            target_object_ids = set(get_param('target_object_ids'))
            new_objects_found = set({})            
            if last_action["type"] == ActionType.LookAction:
                voxels = last_observation["voxels"]
                for voxel_pose in voxels:
                    _, label = voxels[voxel_pose]
                    if int(label) in target_object_ids:
                        new_objects_found.add(label)
                obs_info["objects_found"] = robot_state["objects_found"] | new_objects_found
        else:
            obs_info["objects_found"] = robot_state["objects_found"]
        # robot state
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_height()  # provides z pose.
        obs_info["camera_direction"] = obs_info["robot_pose"][3:]  # consistent with transition model.
    return obs_info
        
        
def start_pcl_process(save_path, detect_ar=False):
    search_space_dimension = get_param('search_space_dimension')
    search_space_resolution = get_param('search_space_resolution')    
    fov = get_param('fov')
    asp = get_param('aspect_ratio')
    near = get_param('near')
    far = get_param('far')
    sparsity = get_param("sparsity")
    occupied_threshold = get_param("occupied_threshold")
    mark_nearby = get_param("mark_nearby")
    assert type(mark_nearby) == bool
    marker_topic = get_param("marker_topic") #"/movo_pcl_processor/observation_markers"
    point_cloud_topic = get_param("point_cloud_topic")
    
    optional_args = []
    if detect_ar:
        optional_args.append("-M")
        marker_topic += "_ar"
        if mark_nearby:
            optional_args.append("-N")
    
    subprocess.Popen(["rosrun", "movo_object_search", "process_pcl.py",
                      "--save-path", str(save_path),
                      "--quit-when-saved",
                      "--point-cloud-topic", str(point_cloud_topic),
                      "--marker-topic", str(marker_topic),
                      "--resolution", str(search_space_resolution),
                      "--fov", str(fov),
                      "--asp", str(asp),
                      "--near", str(near),
                      "--far", str(far),
                      "--sparsity", str(sparsity),
                      "--occupied-threshold", str(occupied_threshold)]\
                     + optional_args)

def wait_for_robot_pose():
    robot_pose_topic = get_param('robot_pose_topic')
    msg = rospy.wait_for_message(robot_pose_topic, PoseWithCovarianceStamped, timeout=15)
    robot_pose = read_pose_msg(msg)
    return robot_pose

def wait_for_torso_height():
    torso_topic = get_param('torso_height_topic')  # /movo/linear_actuator/joint_states
    msg = rospy.wait_for_message(torso_topic, JointTrajectoryControllerState, timeout=15)
    assert msg.joint_names[0] == 'linear_joint', "Joint is not linear joint (not torso)."
    position = msg.actual.positions[0]
    return position

def main():
    rospy.init_node("movo_object_search_in_region",
                    anonymous=True)

    region_file = get_param("region_file")
    region_name = get_param("region_name")
    with open(region_file) as f:
        data = json.load(f)
        region_data = data["regions"][region_name]
    region_origin = tuple(map(float, region_data["origin"][:2]))
    search_space_dimension = int(region_data["dimension"])
    search_space_resolution = float(region_data["resolution"])
    
    # region_origin_x = get_param('region_origin_x')
    # region_origin_y = get_param('region_origin_y')
    # search_space_dimension = get_param('search_space_dimension')
    # search_space_resolution = get_param('search_space_resolution')
    
    target_object_ids = get_param('target_object_ids')  # a list
    _size = search_space_dimension * search_space_resolution
    rospy.loginfo("Total search space area: %.3f x %.3f x %.3f m^3"
                  % (_size, _size, _size))

    # files
    topo_map_file = get_param('topo_map_file')
    action_file = get_param('action_file')
    observation_file = get_param('observation_file')
    prior_file = get_param('prior_file')

    # clear exisiting action/observation files
    if os.path.exists(action_file):
        os.remove(action_file)
    if os.path.exists(observation_file):        
        os.remove(observation_file)  

    # other config
    observation_wait_time = get_param('observation_wait_time')
    action_wait_time = get_param('action_wait_time')    
    fov = get_param('fov')
    asp = get_param('aspect_ratio')
    near = get_param('near')
    far = get_param('far')
    torso_min = get_param('torso_min')
    torso_max = get_param('torso_max')

    # for volumetric observation
    sparsity = get_param("sparsity")
    occupied_threshold = get_param("occupied_threshold")
    mark_nearby = get_param("mark_nearby")
    
    # publish topo markers
    PublishTopoMarkers(topo_map_file, search_space_resolution)
    PublishSearchRegionMarkers(region_origin, search_space_dimension, search_space_resolution)

    # Listen to robot pose
    robot_pose = wait_for_robot_pose()

    subprocess.Popen([VENV_PYTHON, POMDP_SCRIPT,
                      # arguments
                      topo_map_file,
                      list_arg(robot_pose),
                      str(search_space_dimension),
                      list_arg(target_object_ids),
                      list_arg(region_origin),
                      str(search_space_resolution),
                      action_file,
                      observation_file,
                      "--torso-min", str(torso_min),
                      "--torso-max", str(torso_max),
                      "--wait-time", str(observation_wait_time),
                      "--prior-file", prior_file,
                      "--fov", str(fov),
                      "--asp", str(asp),
                      "--near", str(near),
                      "--far", str(far)])
    
    # Wait for an action and execute this action
    robot_state = {"objects_found": set({})}
    last_action_observation = (None, None)
    step = 0
    while True:
        rospy.loginfo("Waiting for action...(t=%d)" % step)
        start_time = rospy.Time.now()
        observation = None
        observation_issued = False
        while rospy.Time.now() - start_time < rospy.Duration(action_wait_time):
            if os.path.exists(action_file):
                rospy.loginfo("Got action! Executing action...")
                with open(action_file) as f:
                    action_info = yaml.load(f)

                # observation obtained from robot                    
                obs_info = execute_action(action_info,  
                                          robot_state,
                                          last_action_observation)
                robot_state["objects_found"] = obs_info["objects_found"]
                
                with open(observation_file, "w") as f:
                    yaml.dump(obs_info, f)
                    
                rospy.loginfo("Action executed. Observation written to file %s" % observation_file)
                last_action_observation = (action_info, obs_info)
                observation_issued = True

                # remove action file
                os.remove(action_file)
                break  # break the loop                
            else:
                rospy.loginfo("Waiting for POMDP action...")
                rospy.sleep(0.5)
        if not observation_issued:
            rospy.logerror("Timed out waiting for POMDP action.")
            break
        if robot_state["objects_found"] == set(target_object_ids):
            rospy.loginfo("Done! All objects found in this region")
            break
        step += 1

if __name__ == "__main__":
    main()
