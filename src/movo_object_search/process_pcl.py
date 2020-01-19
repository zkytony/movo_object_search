### Listens to point cloud and process it
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import time

# A very good ROS Ask question about the point cloud message
# https://answers.ros.org/question/273182/trying-to-understand-pointcloud2-msg/


class PCLProcessor:
    def __init__(self,
                 origin,  # pose of the origin of the gridworld
                 dims, # (w,l,h) of the gridworld
                 topic="/movo_camera/point_cloud/points"):
        self._origin = origin
        self._topic = topic
        self._sub_pcl = rospy.Subscriber(topic, PointCloud2,
                                         self._pcl_cb, callback_args=(self._origin, dims))
        self._received_point_cloud = False

    def _pcl_cb(self, msg, args):
        if not self._received_point_cloud:
            for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
                x,y,z,d = point
                print(x,y,z,d)
            self._received_point_cloud = True


def main():
    rospy.init_node("movo_pcl_processor",
                    anonymous=True, disable_signals=True)
    proc = PCLProcessor((0,0,0), (16,16,16))

    while not rospy.is_shutdown():
        if proc._received_point_cloud:
            break
        time.sleep(0.1)

if __name__ == "__main__":
    main()
