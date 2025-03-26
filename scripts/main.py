from typing import Literal, Optional

import numpy as np
from sklearn.neighbors import KDTree
import rospy


from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
import geometry_msgs.msg
from irobman_project_lab_perception.srv import GetCubePoseEstimates
from irobman_project_lab_control.msg import ManipulatorControlAction, ManipulatorControlGoal

import tf

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

        self.cube_poses: dict[str, geometry_msgs.msg.Pose] = {} # Has to be filled by Perc
        self.cube_pose_dict = {}  # Maps cube_id -> Pose
        self.next_cube_id = 0     # Next available cube id for a new cube.
        self.matching_threshold = 0.1  # Distance threshold in meters for matching cubes.
        self.strategy = strategy

        self.pub_pointcloud = rospy.Publisher("/filtered_point_cloud", PointCloud2, queue_size=10)
        self.pub_cube_pose = rospy.Publisher("/cube_pose", PoseArray, queue_size=10)
        
        self.client = actionlib.SimpleActionClient("manipulator_control", ManipulatorControlAction)
        rospy.loginfo("Waiting for manipulator control server ...")
        self.client.wait_for_server()
        rospy.loginfo("Manipulator Control Server reached!")


    def _get_cube_poses(self) -> dict:
        return self.cube_pose_dict

    # def _get_cube_order(self) -> list:

    #     cube_order = []
    #     cube_poses = self._get_cube_poses()
    #     points = []
    #     labels = []

    #     for key, value in cube_poses.items():
    #         value.position.z = 0  # We dont care about height

    #         labels.append(key)
    #         points.append((value.position.x, value.position.y))

    #     # print(points)
    #     tree = KDTree(points)
    #     distances, _ = tree.query(points, k=2)  # Get nearest neighbors of points
    #     nearest_distances = distances[
    #         :, 1
    #     ]  # Get only nearest neighbor distances (not to self)
    #     sorted_indices = np.argsort(
    #         -nearest_distances
    #     )  # Sort indices based on nearest neighbor distance
    #     # print(nearest_distances)
    #     # Sort labels by rank (sorted_indices gives the order based on distance)
    #     cube_order = [labels[idx] for idx in sorted_indices]
    #     return cube_order

    # def _get_cube_location(self, cube):
    #     return self.cube_poses[cube]

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

    def _get_cube_pose(self, cube_name: str) -> geometry_msgs.msg.Pose:
        return self.cube_pose_dict[cube_name]


    def _fetch_new_cube_estimates(self) -> tuple:
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

        # Keep track of which cube ids have been updated this round.
        updated_ids = set()

        # Process each new pose from perception.
        for new_pose in poses.poses:
            matched = False
            best_match_id = None
            best_match_distance = float('inf')

            # Attempt to match with an existing cube pose.
            for cube_id, existing_pose in self.cube_pose_dict.items():
                if cube_id in updated_ids:
                    continue
                dx = new_pose.position.x - existing_pose.position.x
                dy = new_pose.position.y - existing_pose.position.y
                dz = new_pose.position.z - existing_pose.position.z
                distance = np.sqrt(dx * dx + dy * dy + dz * dz)
                if distance < self.matching_threshold and distance < best_match_distance:
                    best_match_distance = distance
                    best_match_id = cube_id

            if best_match_id is not None:
                # Update the matched cube with the new pose.
                self.cube_pose_dict[best_match_id] = new_pose
                updated_ids.add(best_match_id)
                matched = True

            if not matched:
                # No match found; add as a new cube.
                self.cube_pose_dict[self.next_cube_id] = new_pose
                updated_ids.add(self.next_cube_id)
                self.next_cube_id += 1

        # Build a new PoseArray message with poses ordered by cube id for consistency.
        updated_pose_array = PoseArray()
        updated_pose_array.header = poses.header  # Alternatively, use rospy.Time.now() if needed.
        for cube_id in sorted(self.cube_pose_dict.keys()):
            updated_pose_array.poses.append(self.cube_pose_dict[cube_id])
        
        # Publish the updated PoseArray.
        self.pub_cube_pose.publish(updated_pose_array)

        # # TODO: Need to track the cube poses and update their poses as new information becomes available

        # def _update_cube(new_pose):
        #     for cube_name, old_pose in self.cube_poses.items():
        #         old = np.array([old_pose.position.x, old_pose.position.y, old_pose.position.z])
        #         new = np.array([new_pose.position.x, new_pose.position.y, new_pose.position.z])
        #         dist = np.linalg.norm(old - new)

        #         if dist < 0.03:
        #             print(f"Updating Cube {cube_name}!")
        #             self.cube_poses[cube_name] = new_pose
        #             return cube_name
        #     return None


        # # Need to publish this information everytime we update, i.e. whenever we access perception to give us information regarding the cubes
        # # or when we mov the cubes (we know their translation relative to the gripper, so we can at least give an approximate position update)
        # # or we just set the desired pose as the new pose

        # for new_pose in poses.poses:
        #     name = _update_cube(new_pose)
        #     if name is None:
        #         # If we make it to here, we did not find a match, so this must be a new cube
        #         cube_name = f"cube_{len(self.cube_poses)}"
        #         print(f"Adding new cube: {cube_name}")
        #         self.cube_poses[cube_name] = new_pose


        # # In the end publish the new poseArray 
        # new_poses = PoseArray()
        # new_poses.poses = list(self.cube_poses.values())
        # self.pub_cube_pose.publish(new_poses)

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
        quaternion = tf.transformations.quaternion_from_euler(0, np.pi, 0) # type: ignore

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
            self._send_command(command="scan_cube", target_pose=self._get_cube_pose(cube_name))
            _, poses = self._fetch_new_cube_estimates()
            self._update_cube_pose_estimates(poses)
            self._send_command(command="pick", target_pose=self._get_cube_pose(cube_name))
            
            tower_pose.position.z = 0.01 + 0.042 + 0.08 + (i * (0.042))
            rospy.loginfo("Initializing Place...")
            self._send_command(command="place", target_pose=tower_pose)

            i += 1

        return True


if __name__ == "__main__":
    print("Creating Core")
    core = Core(strategy=None)
    print("Core created")


    
    
    core._send_command("go_to_overview")
    pc, poses = core._fetch_new_cube_estimates()
    core._update_cube_pose_estimates(poses)
    core._build_tower()
    




