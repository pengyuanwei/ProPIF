import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum
import torch

# Curobo imports
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import JointState as CuroboJointState
from curobo.types.base import TensorDeviceType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.geom.sdf.world import CollisionCheckerType

# ROS message/service imports
from propif_msgs.srv import ExecuteJointTrajectory, GetRobotState
from propif_msgs.msg import PlaneInfo
from std_msgs.msg import String

class ControllerState(Enum):
    DETECTION = 0
    EXECUTION = 1
    RETURN_HOME = 2
    IDLE = 3

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        #! Load configuration Change this to your own config path
        self.curobo_robot_config = "/home/pengyuan/projects/mobile-levitator/ProPIF/configs/levi_pandaconfig.yaml"
        self.control_frequency = 100

        # Initialize state variables
        self.state = ControllerState.DETECTION
        self.detected_planes = []
        self.current_plane = None
        self.last_detection_time = None
        self.detection_start_time = None
        self.execution_index = 0
        self.execution_wait_start = None
        self.tensor_args = TensorDeviceType()
        # Directly initialize parameters
        self.start_position = [0.4, 0, 0.83]

        # Add trajectory execution status tracking
        self.trajectory_executing = False

        self.latest_robot_state = None

        #self.setup_robot_model()
        self.setup_motion_planner()

        self.get_logger().info('Control node initialized, starting in DETECTION phase')

    def get_robot_state(self):
        return self.latest_robot_state

    def get_initial_robot_state(self, timeout_sec=5.0):
        start = self.get_clock().now().nanoseconds / 1e9
        while self.latest_robot_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now().nanoseconds / 1e9) - start > timeout_sec:
                break
        return self.latest_robot_state

    def setup_robot_model(self):
        try:
            init_state = self.get_initial_robot_state()
            if not init_state:
                raise RuntimeError("Failed to get initial state")
            self.num_joints = 7
            self.home_joint_angles = list(init_state.joint_positions)
            
            # Use actual joint names from Panda robot
            self.joint_names = [f"panda_joint{i+1}" for i in range(self.num_joints)]
        except Exception as e:
            self.get_logger().error(f'Robot model error: {e}')
            raise

    def setup_motion_planner(self):
        try:
            world_config = {
                "cuboid": {
                    "table": {
                        "dims": [1.0, 1.0, 0.2],  # x, y, z
                        "pose": [3.0, 0.0, -0.1, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                    },
                },
            }
            self.get_logger().info('111')
            config = MotionGenConfig.load_from_robot_config(
                self.curobo_robot_config,
                world_config,
                interpolation_dt=0.01,
            )
            self.get_logger().info('222')
            self.motion_planner = MotionGen(config)
            self.get_logger().info('333')
            self.motion_planner.warmup()
            self.get_logger().info('444')
            
            self.get_logger().info('Motion planner initialized with custom configuration')
        except Exception as e:
            self.get_logger().error(f'Motion planner error: {e}')
            self.motion_planner = None



def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()