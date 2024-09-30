from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node

class Nav2CmdNode(Node):
    def __init__(self):
        super().__init__('nav2_cmd_node')
        self.navigator = BasicNavigator()

    def run(self):
        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 1.84
        initial_pose.pose.position.y = -9.05
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 3.14
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()
        #

        # Go to our demos first goal pose
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose1.pose.position.x = -0.12
        goal_pose1.pose.position.y = -2.35
        goal_pose1.pose.orientation.w = 3.14
        goal_pose1.pose.orientation.z = 0.0
        goal_poses.append(goal_pose1)
        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose2.pose.position.x = 1.86
        goal_pose2.pose.position.y = 2.56
        goal_pose2.pose.orientation.w = 0.97
        goal_pose2.pose.orientation.z = 0.0
        goal_poses.append(goal_pose2)
        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose3.pose.position.x = -3.84
        goal_pose3.pose.position.y = 2.64
        goal_pose3.pose.orientation.w = 2.78
        goal_pose3.pose.orientation.z = 0.0
        goal_poses.append(goal_pose3)

        self.get_logger().info("The executed script is here.")

        # Get the path, smooth it
        # path = self.navigator.getPath(initial_pose, goal_poses)
        # smoothed_path = self.navigator.smoothPath(path)
        self.get_logger().info("Here")

        # Follow path
        self.navigator.followWaypoints(goal_poses)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        self.navigator.lifecycleShutdown()

        exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()