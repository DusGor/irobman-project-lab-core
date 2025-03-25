from typing import Literal, Optional

import numpy as np
from sklearn.neighbors import KDTree
import rospy


from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
import geometry_msgs.msg
#from irobman_project_lab_perception.srv import GetCubePoseEstimates
from irobman_project_lab_control.msg import ManipulatorControlAction, ManipulatorControlGoal

import actionlib


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

        self.cube_poses = {} # Has to be filled by Perc
        self.strategy = strategy

        self.pub_pointcloud = rospy.Publisher("/filtered_point_cloud", PointCloud2, queue_size=10)
        self.pub_cube_pose = rospy.Publisher("/cube_pose", PoseArray, queue_size=10)
        
        self.client = actionlib.SimpleActionClient("manipulator_control", ManipulatorControlAction)
        rospy.loginfo("Waiting for manipulator control server ...")
        self.client.wait_for_server()
        rospy.loginfo("Manipulator Control Server reached!")


    def _get_cube_poses(self) -> dict:
        return self.cube_poses

    def _get_cube_order(self) -> list:

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
        return cube_order

    def _get_cube_location(self, cube):
        return self.cube_poses[cube]

    def _send_command(self, command: Literal["go_to_overview", "scan_cube", "pick", "place"], target_pose=None):
        goal = ManipulatorControlGoal()
        goal.command = command
        if target_pose is not None:
            goal.target_pose = target_pose
        
        self.client.send_goal(goal)
        rospy.loginfo(f"Sent command {command}")
        self.client.wait_for_result()
        result = self.client.get_result()

        if result.success:
            rospy.loginfo(f"Action '{command}' completed successfully!")
            return True
        else:
            rospy.logerr(f"Action 'command' failed!")
            return False


    def _fetch_new_cube_estimates(self):
        # cv subsciber
        try:
            get_cube_estimates = rospy.ServiceProxy("/get_cube_pose_estimates", GetCubePoseEstimates)
            response = get_cube_estimates()
            rospy.loginfo(f"Received PointCloud: {response.pointcloud is not None}")
            rospy.loginfo(f"Received PoseArray: {response.cubeposes is not None}")
            
            # * Publish PointCloud for Debugging
            for _ in range(10):
                self.pub_pointcloud.publish(response.pointcloud)
                rospy.sleep(1)

            # * Update tracking of cube poses
            for _ in range(10):
                self._update_cube_pose_estimates(response.cubeposes)
                rospy.sleep(1)

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


    def _get_cube_order(self) -> list:

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
        return cube_order





    def _get_ideal_tower_location(self) -> geometry_msgs.msg.Pose:
        
        cube_poses = self._get_cube_poses()
        cube_positions = [(pose.position.x, pose.position.y) for pose in cube_poses.values()]

        if not cube_positions:
            raise ValueError("No cube positions available!")
            
        # Define the search space (adjust based on table size)
        x_min, x_max = 0.4, 0.7  
        y_min, y_max = -0.15, 0.15 # Tried -0.3:0.3 here but arm grip was getting unstable when passing singularities
        grid_resolution = 200  # Number of points per axis
        
        # Create grid points for potential tower locations
        grid_x = np.linspace(x_min, x_max, grid_resolution)
        grid_y = np.linspace(y_min, y_max, grid_resolution)
        grid_points = np.array([(x, y) for x in grid_x for y in grid_y])
        
        # Use KDTree for fast nearest-neighbor search
        tree = KDTree(cube_positions)
        distances, _ = tree.query(grid_points)  # Find nearest cube distance for each grid point

        # Select the grid point with the maximum minimum distance (most isolated)
        best_index = np.argmax(distances)
        best_x, best_y = grid_points[best_index]

        # Convert Euler angles (0, π, 0) to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, math.pi, 0)

        # Create and return Pose object
        tower_pose = geometry_msgs.msg.Pose()
        tower_pose.position.x = best_x
        tower_pose.position.y = best_y
        tower_pose.position.z = 0.01  # Tower is on the table

        tower_pose.orientation.x = quaternion[0]
        tower_pose.orientation.y = quaternion[1]
        tower_pose.orientation.z = quaternion[2]
        tower_pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Ideal Tower Pose is... {tower_pose}")
        return tower_pose

    
    def _build_tower(
        self,
    ) -> bool:
        tower_pose = self._get_ideal_tower_location()
        
        i = 0
        for cube_name in self._get_cube_order():

            rospy.loginfo(f"Pick&Place for {cube_name}")
            self._send_command(command="pick", target_pose=self._get_cube_grasp(cube_name))

            tower_pose.position.z = 0.01 + self.CUBE_GRASP_Z + (i * (self.cube_size))
            rospy.loginfo("Initializing Place...")
            self.send_command(command="place", target_pose=tower_pose)

            i += 1

        return True


if __name__ == "__main__":
    print("Creating Core")
    core = Core(strategy=None)
    print("Core created")


    pc, poses = core._fetch_new_cube_estimates()
    core._update_cube_pose_estimates(poses)
    core._send_command("go_to_overview")
    core.build_tower()
    




