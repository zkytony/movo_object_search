### Listens to point cloud and process it
import rospy
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose

import time
from camera_model import FrustumCamera

VOXEL_OCCUPIED = 0  # TODO: Should replace this by object id
VOXEL_FREE = -1
VOXEL_UNKNOWN = -2

# A very good ROS Ask question about the point cloud message
# https://answers.ros.org/question/273182/trying-to-understand-pointcloud2-msg/
class PCLProcessor:
    def __init__(self,
                 # frustum camera configuration
                 fov=90,
                 aspect_ratio=1,
                 near=1,
                 far=5,
                 resolution=0.5,  # m/grid cell
                 topic="/movo_camera/point_cloud/points",
                 sparsity=1000,
                 occupied_threshold=5):
        self._topic = topic
        self._resolution = resolution
        self._sparsity = sparsity  # number of points to skip
        self._occupied_threshold = occupied_threshold
        self._cam = FrustumCamera(fov=fov, aspect_ratio=aspect_ratio,
                                  near=near, far=far)
        self._sub_pcl = rospy.Subscriber(topic, PointCloud2,
                                         self._pcl_cb)#, callback_args=(self._cam))
        self._processed_point_cloud = False

        # Publish processed point cloud
        self._pub_pcl = rospy.Publisher("/movo_pcl_processor/observation_markers",
                                        MarkerArray,
                                        queue_size=10,
                                        latch=True)

    def _pcl_cb(self, msg):
        # We just process one point cloud message.
        if self._processed_point_cloud:
            self._processed_point_cloud = False
        if not self._processed_point_cloud:
            voxels = self.process_cloud(msg)
            msg = self.make_markers_msg(voxels)
            # publish message
            r = rospy.Rate(3) # 3 Hz
            self._pub_pcl.publish(msg)
            print("Published markers")
            self._processed_point_cloud = True
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

    def process_cloud(self, msg):
        # Iterate over the voxels in the FOV
        points = []
        for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            points.append(point)
        voxels = []
        oalt = {}
        pesp_to_plel = {}  # map from xyz in perspective to xy key in parallel
        for xyz in self._cam.volume:
            # Iterate over the whole point cloud sparsely
            i = 0
            count = 0
            occupied = False
            # In the camera model, robot looks at -z direction.
            # But robot actually looks at +z in the real camera.
            original_z = xyz[2]
            xyz[2] = abs(xyz[2])
            for point in points:
                if i % self._sparsity == 0:
                    if self.point_in_volume(xyz, point):
                        count += 1
                        if count > self._occupied_threshold:
                            occupied = True
                            break
                i += 1
            if occupied:
                voxels.append((xyz, VOXEL_OCCUPIED))
                xyz2 = [xyz[0], xyz[1], xyz[2]+1]
                voxels.append((xyz2, VOXEL_UNKNOWN))
            else:
                voxels.append((xyz, VOXEL_FREE))
        #     # Project the voxel to parallel space and
        #     # obtain a mapping from xy_key to z_depth.
        #     # Then use this mapping to account for occlusions
        #     x,y,_ = map(float,xyz[:3]); z = original_z #-abs(xyz[2])  # camera model looks at -z direction
        #     parallel_point = self._cam.perspectiveTransform(x, y, z, (0,0,0,0,0,0))
        #                                                     # (0,0,0,0,0,0,1))  # point is already in camera space
        #     xy_key = (round(parallel_point[0], 2), round(parallel_point[1], 2))                                                            
        #     if(xy_key not in oalt.keys()):
        #         # primitive of oalt: { (x,y) : occupied (or, objid), cube_depth}
        #         oalt[xy_key] = (occupied, parallel_point[2])
        #     else:
        #         # update the z index if the point is closer
        #         oalt[xy_key] = (occupied, min(oalt[xy_key][1], parallel_point[2])) # since camera looks at +z direction
        #     pesp_to_plel[tuple(xyz)] = (xy_key, parallel_point[2])

        # # Figure out the final voxel labels (if the voxel is occluded,
        # # or free, or occupied) based on its z-index in the parallel
        # # projection space. There is some loss here, but, because the
        # # volumetric observation is already coarse, the loss should be acceptable.
        # output_voxels = []
        # for xyz, occupied in voxels:
        #     if occupied:
        #         voxel = (xyz, VOXEL_OCCUPIED)
        #     else:
        #         xy_key, z_point = pesp_to_plel[tuple(xyz)]
        #         if z_point > oalt[xy_key][1]: # and oalt[xy_key][0] is True:
        #             # the point is occluded
        #             voxel = (xyz, VOXEL_UNKNOWN)
        #         else:
        #             voxel = (xyz, VOXEL_FREE)
        #     output_voxels.append(voxel)
        return voxels

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
        for voxel in voxels:
            xyz, label = voxel
            
            h = Header()
            h.stamp = timestamp
            h.frame_id = "movo_camera_color_optical_frame"
            
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
                marker_msg.color.a = 0.1
            elif label == VOXEL_UNKNOWN:  # grey
                marker_msg.color.r = 0.8
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.8
                marker_msg.color.a = 0.7
            else:
                raise ValueError("Unknown voxel label %s" % str(label))
            marker_msg.lifetime = rospy.Duration.from_sec(3.0)  # forever
            marker_msg.frame_locked = True
            markers.append(marker_msg)

        marker_array_msg = MarkerArray(markers)
        return marker_array_msg


def main():
    rospy.init_node("movo_pcl_processor",
                    anonymous=True, disable_signals=True)

    # Some info about the Kinect
    # The Kinect has a range of around . 5m and 4.5m (1'8"-14'6".)
    #    (ref: https://docs.depthkit.tv/docs/kinect-for-windows-v2)
    # The old Kinect has a color image resolution of 640 x 480 pixels with a fov of
    # 62 x 48.6 degrees resulting in an average of about 10 x 10 pixels per degree. (see source 1)
    # The new Kinect has color image resolution of 1920 x 1080 pixels and a fov
    # of 84.1 x 53.8 resulting in an average of about 22 x 20 pixels per degree. (see source 2)
    proc = PCLProcessor(fov=60, aspect_ratio=1.0,
                        near=1.0, far=7, resolution=0.3,
                        sparsity=500, occupied_threshold=3)  # this covers a range from about 0.32m - 4m
    rospy.spin()

if __name__ == "__main__":
    main()
