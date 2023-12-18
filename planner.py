from mapUtilities import *
from a_star import *
from rrt_star import *
import time

POINT_PLANNER=0; A_STAR_PLANNER=1; RRT_PLANNER=2; RRT_STAR_PLANNER=3


# TODO Modify this class so that is uses the RRT* planner with virtual obstacles

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        # Corridor
        corridor_obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (9, 11, 1),
        (6, 12, 1),
        ]  # [x,y,size(radius)]

        # Bubble Mine Field
        mines_obstacle_list = [
            (0, 2, 1),
            (5, 2, 1),
            (10, 3, 1),
            (3, 5, 1),
            (1, 11, 2),
            (9, 5, 2),
            (10, 11, 1),
            (5, 8, 1),
        ]  # [x,y,size(radius)]

        self.rrt_star = RRTStar(
        # start=[0, 0], # Corridor 1
        # goal=[6, 10], # Corridor 1
        start=[6, 10], # Corridor 2
        goal=[0, 12], # Corridor 2
        # start=[0, 0], # Mines 1
        # goal=[6, 10], # Mines 1
        # start=[6, 10], # Mines 2
        # goal=[13, 0], # Mines 2
        rand_area=[-2, 15],
        obstacle_list=corridor_obstacle_list,
        expand_dis=0.5, # Halved to enable robot to better path into hallways
        path_resolution=0.5, # Halved since expand distance was halved
        connect_circle_dist=100.0,
        robot_radius=0.8)
        
    
    def trajectory_planner(self, startPoseCart, endPoseCart, type):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        #### If not using the map (no bonus), you can just call the function in rrt_star with the appropriate arguments and get the returned path
        #### then you can put the necessary measure to bypass the map stuff down here.
        # Map scaling factor (to save planning time)

        # TODO This is for A*, modify this part to use RRT*
        # path = search(self.costMap, startPose, endPose, scale_factor)
        
        # This is for RRT*
        start_time = time.time()
        # For now the RRT* goes to a hard coded location

        path = self.rrt_star.planning(animation=False)

        end_time = time.time()

        # This will display how much time the search algorithm needed to find a path
        print(f"the time took for rrt_star calculation was {end_time - start_time}")

        # TODO Smooth the path before returning it to the decision maker
        # this can be in form of a function that you can put in the utilities.py 
        # or add it as a method to the original rrt.py 

        # The path was smoothened in rrt_star.py in the smooth() function
        path.reverse() # reverse path to navigate in correct order

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

            # Draw final path
            if show_animation:
                self.rrt_star.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
                plt.grid(True)
                plt.show()

        return path[1:] # Don't return start of path (robot will be at start position already)


if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can do your test here ...

