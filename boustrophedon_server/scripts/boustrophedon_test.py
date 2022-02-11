#! /usr/bin/env/ python3 
import rospy
from geometry_msgs.msg import Pose, Point32
from boustrophedon_msgs.msg import PlanMowingPathActionGoal, PlanMowingPathActionResult
from std_srvs.srv import Trigger, TriggerResponse

class BoustrophedonTest():
    def __init__(self):
        rospy.loginfo("Node has been initialised")
        # robot_pose topic or amcl_pose topic is both fine
        rospy.Subscriber("robot_pose", Pose, self.robotPoseSubCB, queue_size=1)
        self.start_srv = rospy.Service('boustrophedon_start_srv', Trigger, self.startServiceCB)
        self.boustrophedon_action_server_goal_pub = rospy.Publisher('/plan_path/goal', PlanMowingPathActionGoal, queue_size=1)
        self.boustrophedon_action_server_goal_sub = rospy.Subscriber("/plan_path/result", PlanMowingPathActionResult, self.actionResult, queue_size=1)
        self.robot_pose = None

    def robotPoseSubCB(self, data):
        self.robot_pose = data

    def startServiceCB(self, data):
        if (self.robot_pose is None):
            rospy.logerr("The robot pose has not been published")
        action_goal_message = PlanMowingPathActionGoal()
        action_goal_message.goal.property.header.frame_id = "map"
        action_goal_message.goal.robot_position.header.frame_id = "map"
        action_goal_message.goal.property.header.stamp = rospy.Time.now()
        action_goal_message.goal.robot_position.header.stamp = rospy.Time.now()

        # To get these points, go to the top of rviz and click the + button and add Publish Point
        # Then echo /clicked_point in the terminal and use the Publish Point button to obtain the coordinates of the polygon
        points1 = Point32()
        points1.x = -6.0
        points1.y = -1.0
        action_goal_message.goal.property.polygon.points.append(points1)

        points2 = Point32()
        points2.x = -1.0
        points2.y = -1.0
        action_goal_message.goal.property.polygon.points.append(points2)

        points3 = Point32()
        points3.x = -4.0
        points3.y = -5.0
        action_goal_message.goal.property.polygon.points.append(points3)

        print(f"These are the points: {action_goal_message.goal.property.polygon.points}")

        # Add the current position of the robot
        action_goal_message.goal.robot_position.pose.position.x = self.robot_pose.position.x
        action_goal_message.goal.robot_position.pose.position.y = self.robot_pose.position.y
        action_goal_message.goal.robot_position.pose.position.z = self.robot_pose.position.z
        action_goal_message.goal.robot_position.pose.orientation.x = self.robot_pose.orientation.x
        action_goal_message.goal.robot_position.pose.orientation.y = self.robot_pose.orientation.y
        action_goal_message.goal.robot_position.pose.orientation.z = self.robot_pose.orientation.z
        action_goal_message.goal.robot_position.pose.orientation.w = self.robot_pose.orientation.w

        print("Published goal to the server")
        self.boustrophedon_action_server_goal_pub.publish(action_goal_message)
        return(TriggerResponse())

    def actionResult(self, data):
        print(data)


if __name__ == "__main__":
    rospy.init_node("boustrophedon_test")
    print('''
    Hello welcome to the boustrophedon test script. Ensure that roslaunch boustrophedon_server boustrophedon_server.launch is running 
    afterwhich proceed to call the following service: rosservice call /boustrophedon_start_srv "{}" 
    ''')
    some_instance = BoustrophedonTest()
    rospy.spin()