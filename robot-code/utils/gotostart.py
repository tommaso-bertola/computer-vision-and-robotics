from utils.the_maze_runner import MazeRunner
from cycler import V
import ev3_dc as ev3
import numpy as np
from rich import print
from utils.tempo import *
from utils.magic import ordinator

class GoToStart:
    def __init__(self, data):
        self.positions = np.array(data.landmark_estimated_positions)
        self.robot_pose = data.robot_position
        # self.robot_theta = data.robot_theta
        self.ids = np.array(data.landmark_estimated_ids)
        self.target = None
        self.get_arrival_coords()
        print("TARGET:",self.target, "TYPE:", type(self.target))
        self.status = 0
        self.angle_limit = np.deg2rad(5)

    def find_extremes(self):
        mask_start_line = (self.ids < 100) & (self.ids > 0)
        positions_start_line = self.positions[mask_start_line]
        pos_start_line_ordered = np.array(
            ordinator(positions_start_line, max_distance=0.3))
        first = pos_start_line_ordered[0]
        last = pos_start_line_ordered[-1]

        radius = np.sqrt((first[0]-last[0])**2+(last[1]-first[1])**2)

        return (first, last, radius)

    def get_centers(self, first, last, r):
        x1, y1 = first
        x2, y2 = last
        r1 = r2 = r
        centerdx = x1 - x2
        centerdy = y1 - y2
        R = np.sqrt(centerdx**2 + centerdy**2)
        if not (abs(r1 - r2) <= R and R <= r1 + r2):
            """ No intersections """
            return []

        """ intersection(s) should exist """
        R2 = R**2
        R4 = R2**2
        a = (r1**2 - r2**2) / (2 * R2)
        r2r2 = r1**2 - r2**2
        c = np.sqrt(2 * (r1**2 + r2**2) / R2 - (r2r2**2) / R4 - 1)

        fx = (x1 + x2) / 2 + a * (x2 - x1)
        gx = c * (y2 - y1) / 2
        ix1 = fx + gx
        ix2 = fx - gx

        fy = (y1 + y2) / 2 + a * (y2 - y1)
        gy = c * (x1 - x2) / 2
        iy1 = fy + gy
        iy2 = fy - gy

        return [np.array([ix1, iy1]), np.array([ix2, iy2])]

    def get_arrival_coords(self):
        intersect_1, intersect_2 = self.get_centers(*self.find_extremes())
        d1 = np.sqrt(np.sum((self.robot_pose-intersect_1)**2))
        d2 = np.sqrt(np.sum((self.robot_pose-intersect_2)**2))

        if d1 < d2:
            self.target = intersect_1
        else:
            self.target = intersect_2


    def run(self, data):
        # get current position and orientation
        self.robot_pose = data.robot_position
        x_robot, y_robot = self.robot_pose
        robot_theta = data.robot_theta % (2*np.pi)
        y_target = self.target[1]
        x_target = self.target[0]
        # use to decide whether to only rotate
        distance_to_target = np.sqrt(np.sum((self.robot_pose-self.target)**2))

        if distance_to_target < 0.05:
            self.status = 2

        if self.status == 0:  # first rotation to target
            print("Entering status 0")
            angle_to_target = np.arctan2(y_target-y_robot, x_target-x_robot)
            print("angle to start", angle_to_target)
            print("abs difference", abs(angle_to_target-robot_theta))

            if (abs(angle_to_target-robot_theta) > self.angle_limit):
                return 3, -200, False  # speed, angle, end_condition
            else:
                self.status = 1
                return 0, 0, False

        elif self.status == 1:  # translation
            print("Entering status 1")
            return 10, 0, False

        elif self.status == 2:  # second roatation
            print("Entering status 2")
            if (abs(robot_theta) > self.angle_limit):
                return 15, -200, False  # speed, angle, end_condition
            else:
                return 0, 0, True
        else:
            raise Exception(
                'Something went horribly wrong in going to starting line')
            # return 0, 0, False