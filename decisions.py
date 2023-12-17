import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter


# The final exam is only about testing the rrt_star, though you can work with the 
# rrt itself too and observe the difference. 
from planner import A_STAR_PLANNER, RRT_PLANNER, RRT_STAR_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
from utilities import FileReader

class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)
        
        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)
        publishing_period=1/rate

        self.reachThreshold=0.1


        self.localizer=localization(kalmanFilter)


        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)


        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(POINT_PLANNER)
            return -1

        self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
        
        if motion_type in [RRT_PLANNER, RRT_STAR_PLANNER, A_STAR_PLANNER]:
            self.planner = planner(motion_type)
            
        else:            
            print("Error! you don't have this type of planner", file=sys.stderr)
            return -1
            

        print("waiting for your input position, use 2D nav goal in rviz2")

        # hint: if you set the self.goal in here, you can bypass the rviz goal selector
        # this can be useful if you don't want to use the map
        self.goal=self.planner.plan([0,0],[6,10])
    
    def designPathFor(self, msg: PoseStamped):
        
        spin_once(self.localizer)
        
        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        self.goal=self.planner.plan([self.localizer.getPose()[0], self.localizer.getPose()[1]],
                                     [msg.pose.position.x, msg.pose.position.y])

    
    def timerCallback(self):
        
        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom in timer callback msgs ....")
            return
        
        
        vel_msg=Twist()
        
        if self.goal is None:
            return
        
        if type(self.goal) == list:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else: 
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            self.planner.rrt_star.draw_graph()
            plt.plot([x for (x, y) in self.goal], [y for (x, y) in self.goal], 'r--', label="Planned Path")
            plt.grid(True)

            filename = "robotPose.csv"
            headers, values=FileReader(filename).read_file()
            
            time_list=[]
        
            first_stamp=values[0][-1]

            for val in values:
                time_list.append(val[-1] - first_stamp)

            # Isolate x and y poses of robot from data
            odom_x = [lin[-3] for lin in values]
            odom_y = [lin[-2] for lin in values]

            ekf_x = [lin[-5] for lin in values]
            ekf_y = [lin[-4] for lin in values]

            plt.plot(ekf_x, ekf_y, label = "EKF Trajectory")
            plt.plot(odom_x, odom_y, label = "Odom (True) Trajectory")
            plt.legend()
            plt.title("X vs Y Trajectory")
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.show()


            self.goal = None
            print("waiting for the new position input, use 2D nav goal on map")
            return
        
        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)

        
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        
        self.publisher.publish(vel_msg)

        # Comment out if not using rviz2
        # self.publishPathOnRviz2(self.goal)



    def publishPathOnRviz2(self, path):

        Path_ =  Path()

        Path_.header.frame_id ="map"
        Path_.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Set the orientation of the pose. Here, it's set to a default orientation.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            Path_.poses.append(pose)

        self.pathPublisher.publish(Path_)

import argparse
def main(args=None):
    
    
    init()
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    
    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=RRT_STAR_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)


    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")
        return

    except Exception as e:
        print(e.format_exc())
        return 




if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="trajectory")
    args = argParser.parse_args()

    main(args)
