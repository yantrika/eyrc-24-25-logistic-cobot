from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.84
    initial_pose.pose.position.y = -9.05
    initial_pose.pose.orientation.z = 0.1
    initial_pose.pose.orientation.w = 3.14
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    #Make a definition
    goal_poses = []

    # Go to our first goal pose
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.12
    goal_pose1.pose.position.y = -2.35
    goal_pose1.pose.orientation.w = 3.14
    goal_pose1.pose.orientation.z = 0.1
    goal_poses.append(goal_pose1)

    #Go to our second goal pose
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.86
    goal_pose2.pose.position.y = 2.56
    goal_pose2.pose.orientation.w = 0.97
    goal_pose2.pose.orientation.z = 0.1
    goal_poses.append(goal_pose2)

    #Go to our third goal pose
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.84
    goal_pose3.pose.position.y = 2.64
    goal_pose3.pose.orientation.w = 2.78
    goal_pose3.pose.orientation.z = 0.1
    goal_poses.append(goal_pose3)

    # Get the path, smooth it
    # path = navigator.getPath(initial_pose, goal_pose)
    # smoothed_path = navigator.smoothPath(path)

    # Follow path
    # navigator.followPath(smoothed_path)
    navigator.followWaypoints(goal_poses)
    i=0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print(
                'Feedback : \n',
                feedback
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()