#!/usr/bin/env python
#
### Listens to point cloud and process it
import argparse
import rospy
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from aruco_msgs.msg import MarkerArray as ArMarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
import message_filters
import tf
import util
import yaml

import time
from camera_model import FrustumCamera

# THE VOXEL TYPE; If > 0, it's an object id.
VOXEL_OCCUPIED = "occupied"
VOXEL_FREE = "free"
VOXEL_UNKNOWN = "unknown"

# A very good ROS Ask question about the point cloud message
# https://answers.ros.org/question/273182/trying-to-understand-pointcloud2-msg/
class PCLProcessor:
    """
    Subscribes to point cloud and ar tag topics, and publish volumetric
    observation as a result of processing the messages.

    The reason to use Subscriber inteader of Client/Service is because
    point cloud data and ar tag markers themselves are published via topics.
    """
    def __init__(self,
                 # frustum camera configuration
                 fov=90,
                 aspect_ratio=1,
                 near=1,
                 far=5,
                 resolution=0.5,  # m/grid cell
                 pcl_topic="/movo_camera/point_cloud/points",
                 marker_topic="/movo_pcl_processor/observation_markers",
                 artag_topic="/aruco_marker_publisher/markers",
                 voxel_marker_frame="movo_camera_color_optical_frame",
                 world_frame="map",
                 sparsity=1000,
                 occupied_threshold=5,
                 mark_nearby=False,  # mark voxels within 1 distance of the artag voxel as well.
                 mark_ar_tag=True,   # true if use message filter to process ar tag and point cloud messages together
                 save_path=None,     # (str) if save the volumetric observation to a file.
                 quit_when_saved=False):    # terminate when an observation has been saved.
        self._resolution = resolution
        self._sparsity = sparsity  # number of points to skip
        self._occupied_threshold = occupied_threshold
        self._voxel_marker_frame = voxel_marker_frame
        self._world_frame = world_frame
        self._mark_nearby = mark_nearby
        self._mark_ar_tag = mark_ar_tag
        self._cam = FrustumCamera(fov=fov, aspect_ratio=aspect_ratio,
                                  near=near, far=far)
        self._save_path = save_path
        self._quit_when_saved = quit_when_saved
        self._quit = False
        
        # Listen to tf
        self._tf_listener = tf.TransformListener()
        
        # The AR tag message and point cloud message are synchronized using
        # message filter. The point cloud message also has its own callback,
        # in case there is no AR tag detected.
        self._sub_pcl = message_filters.Subscriber(pcl_topic, PointCloud2)
        if self._mark_ar_tag:
            self._sub_artag = message_filters.Subscriber(artag_topic, ArMarkerArray)
            self._pcl_artag_ats = message_filters.ApproximateTimeSynchronizer([self._sub_pcl, self._sub_artag],
                                                                              20, 3)#queue_size=10, slop=1.0)
            self._pcl_artag_ats.registerCallback(self._pcl_artag_cb)            
        else:
            self._sub_pcl.registerCallback(self._pcl_cb)        
        self._processing_point_cloud = False
        
        # Publish processed point cloud
        self._pub_pcl = rospy.Publisher(marker_topic,
                                        MarkerArray,
                                        queue_size=10,
                                        latch=True)

    def _pcl_cb(self, msg):
        # We just process one point cloud message at a time.
        if self._processing_point_cloud:
            return
        else:
            self._processing_point_cloud = True
            voxels = self.process_cloud(msg)
            # saving
            if self._save_path is not None:
                wf_voxels = self._transform_worldframe(voxels)
                with open(self._save_path, "w") as f:
                    yaml.dump(wf_voxels, f)
                    if self._quit_when_saved:
                        self._quit = True
            # publish message
            msg = self.make_markers_msg(voxels)
            r = rospy.Rate(3) # 3 Hz
            self._pub_pcl.publish(msg)
            print("Published markers")            
            self._processing_point_cloud = False
            r.sleep()

    def _pcl_artag_cb(self, pcl_msg, artag_msg):
        """Called when received an artag message and a point cloud."""
        if self._processing_point_cloud:
            return
        else:
            self._processing_point_cloud = True
            voxels = self.process_cloud(pcl_msg)

            # Mark voxel at artag location as object
            for artag in artag_msg.markers:
                # Transform pose to voxel_marker_frame
                artag_pose = self._get_transform(self._voxel_marker_frame, artag.header.frame_id, artag.pose.pose)
                if artag_pose is False:
                    return # no transformed pose obtainable
                atx = artag_pose.position.x
                aty = artag_pose.position.y
                atz = artag_pose.position.z
                arvoxel_pose = (int(round(atx / self._resolution)),
                                int(round(aty / self._resolution)),
                                int(round(atz / self._resolution)))
                # (Approach1) Find the voxel_pose in the volume closest to above
                if not self._mark_nearby:
                    closest_voxel_pose = min(voxels, key=lambda voxel_pose: util.euclidean_dist(voxel_pose, arvoxel_pose))
                    voxels[closest_voxel_pose] = (closest_voxel_pose, artag.id)
                else:
                    # (Approach2) Mark all voxel_poses in the volume within a certain dist.
                    nearby_voxel_poses = {voxel_pose
                                          for voxel_pose in voxels
                                          if util.euclidean_dist(voxel_pose, arvoxel_pose) <= 1}
                    for voxel_pose in nearby_voxel_poses:
                        voxels[voxel_pose] = (voxel_pose, artag.id)

            # saving
            if self._save_path is not None:
                wf_voxels = self._transform_worldframe(voxels)
                with open(self._save_path, "w") as f:
                    yaml.dump(wf_voxels, f)
                    if self._quit_when_saved:
                        self._quit = True
                        
            msg = self.make_markers_msg(voxels)
            # publish message
            r = rospy.Rate(3) # 3 Hz
            self._pub_pcl.publish(msg)
            print("Published markers (with AR)")
            self._processing_point_cloud = False
            r.sleep()
    

    def point_in_volume(self, voxel, point):
        """Check if point (in point cloud) is inside the volume covered by voxel"""
        vx,vy,vz = voxel[:3]
        xmin = vx*self._resolution
        xmax = (vx+1)*self._resolution
        ymin = vy*self._resolution
        ymax = (vy+1)*self._resolution
        zmin = vz*self._resolution
        zmax = (vz+1)*self._resolution
        px, py, pz = point[:3]
        # print("%s | %s" % (voxel, point))
        if xmin <= px and px < xmax\
           and ymin <= py and py < ymax\
           and zmin <= pz and pz < zmax:
            return True
        else:
            return False
        
    def _get_transform(self, target_frame, source_frame, pose_msg):
        # http://wiki.ros.org/tf/TfUsingPython
        if self._tf_listener.frameExists(source_frame)\
           and self._tf_listener.frameExists(target_frame):
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = source_frame
            pose_stamped_msg.pose = pose_msg
            pose_stamped_transformed = self._tf_listener.transformPose(target_frame, pose_stamped_msg)
            return pose_stamped_transformed.pose
        else:
            rospy.logwarn("Frame %s or %s does not exist. (Check forward slash?)" % (target_frame, source_frame))
            return False

    def _transform_worldframe(self, voxels):
        # obtain transform
        (trans,rot) = self._tf_listener.lookupTransform(self._world_frame,
                                                        self._voxel_marker_frame,
                                                        rospy.Time(0))
        wf_voxels = {}
        for voxel_pose in voxels:
            x,y,z = voxel_pose
            wf_pose = (x + float(trans[0]),
                       y + float(trans[1]),
                       z + float(trans[2]))
            wf_voxels[wf_pose] = (wf_pose, voxels[voxel_pose][1])
        return wf_voxels
            

    def process_cloud(self, msg):
        # Iterate over the voxels in the FOV
        points = []
        for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            points.append(point)
        rospy.loginfo("(info)[Received %d points in point cloud]" % len(points))
        voxels = {}  # map from voxel_pose xyz to label
        parallel_occupied = {}
        for volume_voxel_pose in self._cam.volume:
            # Iterate over the whole point cloud sparsely
            i = 0
            count = 0
            occupied = False
            # In the camera model, robot looks at -z direction.
            # But robot actually looks at +z in the real camera.
            original_z = volume_voxel_pose[2]
            volume_voxel_pose[2] = abs(volume_voxel_pose[2])
            for point in points:
                if i % self._sparsity == 0:
                    if self.point_in_volume(volume_voxel_pose, point):
                        count += 1
                        if count > self._occupied_threshold:
                            occupied = True
                            break
                i += 1
            # forget about the homogenous coordinate; use xyz as key                
            voxel_pose = tuple((float(volume_voxel_pose[0]),
                                float(volume_voxel_pose[1]),
                                float(original_z)))
            if occupied:
                # xyz2 = (xyz[0], xyz[1], xyz[2]+1)  # TODO: HACK
                voxels[voxel_pose] = (voxel_pose, VOXEL_OCCUPIED)
                # voxels[xyz2] = (xyz2, VOXEL_UNKNOWN)

                parallel_point = self._cam.perspectiveTransform(voxel_pose[0], voxel_pose[1], voxel_pose[2],
                                                                (0,0,0,0,0,0))
                xy_key = (round(parallel_point[0], 2), round(parallel_point[1], 2))
                if xy_key not in parallel_occupied:
                    parallel_occupied[xy_key] = (occupied, parallel_point[2])
                else:
                    if parallel_point[2] > parallel_occupied[xy_key][1]:
                        parallel_occupied[xy_key] = (occupied, parallel_point[2])
            else:
                # didn't know if there are points there
                voxels[voxel_pose] = (voxel_pose, VOXEL_UNKNOWN)

        final_voxels = {}
        for voxel_pose in voxels:
            # Now, decide the label of each voxel
            voxel_pose_ros = (voxel_pose[0], voxel_pose[1], abs(voxel_pose[2]))
            _, label = voxels[voxel_pose]
            if label == VOXEL_OCCUPIED:
                # already occupied, so no change.
                final_voxels[voxel_pose_ros] = (voxel_pose_ros, VOXEL_OCCUPIED)
            else:
                assert label == VOXEL_UNKNOWN
                parallel_point = self._cam.perspectiveTransform(voxel_pose[0], voxel_pose[1], voxel_pose[2],
                                                                (0,0,0,0,0,0))
                xy_key = (round(parallel_point[0], 2), round(parallel_point[1], 2))
                if xy_key in parallel_occupied:
                    if parallel_point[2] > parallel_occupied[xy_key][1]:
                        # the voxel is closer to the camera, so it's free.
                        final_voxels[voxel_pose_ros] = (voxel_pose_ros, VOXEL_FREE)
                    else:
                        # otherwise --> it's unknown
                        final_voxels[voxel_pose_ros] = (voxel_pose_ros, VOXEL_UNKNOWN)
                else:
                    final_voxels[voxel_pose_ros] = (voxel_pose_ros, VOXEL_FREE)

        return final_voxels

    def _make_pose_msg(self, posit, orien):
        pose = Pose()
        pose.position.x = posit[0] * self._resolution
        pose.position.y = posit[1] * self._resolution
        pose.position.z = posit[2] * self._resolution
        pose.orientation.x = orien[0]
        pose.orientation.y = orien[1]
        pose.orientation.z = orien[2]
        pose.orientation.w = orien[3]
        return pose

    def make_markers_msg(self, voxels):
        """Convert voxels to Markers message for visualizatoin"""
        timestamp = rospy.Time.now()
        i = 0
        markers = []
        for voxel_pose in voxels:
            xyz, label = voxels[voxel_pose]
            
            h = Header()
            h.stamp = timestamp
            h.frame_id = self._voxel_marker_frame
            
            marker_msg = Marker()
            marker_msg.header = h
            marker_msg.type = 1  # CUBE
            marker_msg.ns = "volumetric_observation"
            marker_msg.id = i; i+=1
            marker_msg.action = 0 # add an object
            marker_msg.pose = self._make_pose_msg(xyz, [0,0,0,1])
            marker_msg.scale.x = self._resolution
            marker_msg.scale.y = self._resolution
            marker_msg.scale.z = self._resolution
            if label == VOXEL_OCCUPIED:  # red
                marker_msg.color.r = 0.8
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0
                marker_msg.color.a = 0.7
            elif label == VOXEL_FREE:  # cyan
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.8
                marker_msg.color.a = 0.2
            elif label == VOXEL_UNKNOWN:  # grey
                marker_msg.color.r = 0.8
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.8
                marker_msg.color.a = 0.7
            elif label >= 0:  # it's an object. Mark as Green
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.0
                marker_msg.color.a = 1.0
            else:
                raise ValueError("Unknown voxel label %s" % str(label))
            marker_msg.lifetime = rospy.Duration.from_sec(3.0)  # forever
            marker_msg.frame_locked = True
            markers.append(marker_msg)

        marker_array_msg = MarkerArray(markers)
        return marker_array_msg


def main():
    parser = argparse.ArgumentParser(description='Process Point Cloud as Volumetric Observation')
    parser.add_argument('-f', '--save-path', type=str)
    parser.add_argument('--quit-when-saved', action="store_true")    
    parser.add_argument('-p', '--point-cloud-topic', type=str,
                        default="/movo_camera/sd/points")
    parser.add_argument('-m', '--marker-topic', type=str,
                        default="/movo_pcl_processor/observation_markers")
    parser.add_argument('-M', '--mark-ar-tag', action="store_true")
    parser.add_argument('-N', '--mark-nearby', action="store_true")
    parser.add_argument('-r', '--resolution', type=float,
                        default=0.3,
                        help='resolution of search region (i.e. volume). Format, float')
    parser.add_argument('--fov', type=float,
                        help="FOV angle",
                        default=60)
    parser.add_argument('--asp', type=float,
                        help="aspect ratio",
                        default=1.0)
    parser.add_argument('--near', type=float,
                        help="near plane",
                        default=0.1)
    parser.add_argument('--far', type=float,
                        help="far plane",
                        default=7)  # this covers a range from about 0.32m - 4m
    parser.add_argument('--sparsity', type=float,
                        help="Sparsity of point cloud sampling",
                        default=500)
    parser.add_argument('--occupied-threshold', type=float,
                        help="Number of points needed in a cube to mark it as occupied",
                        default=5)
    args = parser.parse_args()
    rospy.init_node("movo_pcl_processor",
                    anonymous=True, disable_signals=True)

    # Some info about the Kinect
    # The Kinect has a range of around . 5m and 4.5m (1'8"-14'6".)
    #    (ref: https://docs.depthkit.tv/docs/kinect-for-windows-v2)
    # The old Kinect has a color image resolution of 640 x 480 pixels with a fov of
    # 62 x 48.6 degrees resulting in an average of about 10 x 10 pixels per degree. (see source 1)
    # The new Kinect has color image resolution of 1920 x 1080 pixels and a fov
    # of 84.1 x 53.8 resulting in an average of about 22 x 20 pixels per degree. (see source 2)
    proc = PCLProcessor(fov=args.fov, aspect_ratio=args.asp,
                        near=args.near, far=args.far, resolution=args.resolution,
                        sparsity=args.sparsity, occupied_threshold=args.occupied_threshold,
                        pcl_topic=args.point_cloud_topic,
                        marker_topic=args.marker_topic,
                        mark_nearby=args.mark_nearby,
                        mark_ar_tag=args.mark_ar_tag,
                        save_path=args.save_path,
                        quit_when_saved=args.quit_when_saved)
    while not proc._quit:
        rospy.spin()

if __name__ == "__main__":
    main()
