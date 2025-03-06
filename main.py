from typing import Literal, Optional

import numpy as np
from sklearn.neighbors import KDTree
import rospy


from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray

from irobman_project_lab_perception.srv import GetCubePoseEstimates

import argparse

parser = argparse.ArgumentParser(
    description="irobman-project-lab-perception"
)

parser.add_argument(
    "--sim_mode", type=bool, default=True, help="Whether to configure the node for simulation or the real robot"
)

# parse the arguments
args_cli = parser.parse_args()

print(f"SIM MODE: {args_cli.sim_mode}")


class Core:
    def __init__(self, strategy: Optional[str]):
        rospy.init_node("core", anonymous=False)
        rospy.sleep(2)
        rospy.wait_for_service("/get_cube_pose_estimates")

        self.strategy = strategy

        self.pub_pointcloud = rospy.Publisher("/filtered_point_cloud", PointCloud2, queue_size=10)
        self.pub_cube_pose = rospy.Publisher("/cube_pose", PoseArray, queue_size=10)
        

    def _fetch_new_cube_estimates(self):
        # cv subsciber
        try:
            get_cube_estimates = rospy.ServiceProxy("/get_cube_pose_estimates", GetCubePoseEstimates)
            response = get_cube_estimates()
            rospy.loginfo(f"Received Pointcloud: {response.pointcloud is not None}")
            rospy.loginfo(f"Received PoseArray: {response.cubeposes is not None}")
            
            # * Publish PointCloud for Debugging
            self.pub_pointcloud.publish(response.pointcloud)

            # * Update tracking of cube poses
            self._update_cube_pose_estimates(response.cubeposes)
            
            return response.pointcloud, response.cubeposes
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed! {e}")

    def _update_cube_pose_estimates(self, poses: PoseArray):
        # TODO: Need to track the cube poses and update their poses as new information becomes available
        # Need to publish this information everytime we update, i.e. whenever we access perception to give us information regarding the cubes
        # or when we mov the cubes (we know their translation relative to the gripper, so we can at least give an approximate position update)
        # or we just set the desired pose as the new pose

        # In the end publish the new poseArray 
        self.pub_cube_pose.publish(poses)

    def _get_cube_poses(self):
        # TODO
        # ! Cube Poses need to be the same every time!, 
        # ! i.e. cannot start numerating ba the length of the array, 
        # ! because number of cubes in vewport can vary depending on viewport pose , occlusion, reflecctions etc.
        pass



    def _get_cube_order(self):
        cube_order = []
        cube_poses = self._get_cube_poses()
        points = []
        labels = []

        for key, value in cube_poses.items():
            value.position.z = 0  # We dont care about height

            labels.append(key)
            points.append((value.position.x, value.position.y))

        # print(points)
        tree = KDTree(points)
        distances, _ = tree.query(points, k=2)  # Get nearest neighbors of points
        nearest_distances = distances[
            :, 1
        ]  # Get only nearest neighbor distances (not to self)
        sorted_indices = np.argsort(
            -nearest_distances
        )  # Sort indices based on nearest neighbor distance
        # print(nearest_distances)
        # Sort labels by rank (sorted_indices gives the order based on distance)
        cube_order = [labels[idx] for idx in sorted_indices]
        # print(cube_order)
        return cube_order


        # need robot command publisher
        # control_cube_pose_pub = rospy.Publisher("/panda_command/cube_poses")
        # control_next_cube_pub = rospy.Publisher("/panda_command/next_cube")

    def _get_ideal_tower_location(self):
        return self._get_cube_poses()[self._get_cube_order()[0]]
    
    def _get_cube_location_in_order(self):
        cube_order = self._get_cube_order()
        cube_poses = self._get_cube_poses()
        poses_in_order = []

        for cube in cube_order:
            poses_in_order.append(self._get_cube_location(cube))

        return np.array(poses_in_order)
    
    def build_tower(self):
        # TODO implement high-level logic that moves our manipulator
        # Want two modes of operation: general control:
        # 1: tell manipulator to go to overview position
        # 2: tell manipulator to pick and place a cube

        # We want some kind of feedback look, so that our planning node knows when each task has been finished
        # We want to wait for the manipulator to reach the overview position before we take an image of the cubes
        # We want our manipulator to reach the position close to the cube before we execute perception
        pass

if __name__ == "__main__":
    core = Core(strategy=None)
    pc, poses = core._fetch_new_cube_estimates()
    core._update_cube_pose_estimates(poses)


    




