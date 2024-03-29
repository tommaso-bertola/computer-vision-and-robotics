import cv2
import time
import numpy as np
import sys
import jsonpickle
import pickle
from message import Message
from timeit import default_timer as timer
from numba import typed
from datetime import datetime

from publisher import Publisher
from rich import print

from utils.keypress_listener import KeypressListener
from utils.utils import load_config
from utils.robot_controller import RobotController
from utils.aruco_follower import Wanderer
from utils.path_follower import Runner
from utils.gotostart import GoToStart
from utils.tempo import *

from enum import Enum


class TaskPart(Enum):
    Manual = 0
    Exploration = 1
    ToStartLine = 2
    Race = 3
    Load = 4
    PrepareRace = 5


class Main():
    def __init__(self) -> None:

        self.config = load_config("config.yaml")

        self.robot = RobotController(self.config)
        self.keypress_listener = KeypressListener()
        self.publisher = Publisher()
        self.wanderer = Wanderer(self.config)
        self.starter = None

        self.DT = self.config.robot.delta_t  # delta time in seconds
        self.dt_ekfslam = 0

        self.speed = 0
        self.turn = 0
        self.new_speed = 0
        self.new_turn = 0

        self.manualMode = False
        self.is_running = True

        self.map = None

        self.mode = TaskPart.Manual

        self.run_loop()

    @timeit
    def run_loop(self):
        print("starting...")

        # control vehicle movement and visualize it
        with self.keypress_listener, self.publisher, self.robot:
            print("starting EKF SLAM...")

            print("READY!")
            print("[green]MODE: Manual")

            count = 0

            while self.is_running:
                time0 = timer()
                self.run(count, time0)

                elapsed_time = timer() - time0
                if elapsed_time <= self.DT:
                    dt = self.DT - elapsed_time
                    time.sleep(dt)  # moves while sleeping
                else:
                    print(f"[red]{count} dt = {elapsed_time}, RUN")
                    pass

                count += 1

            print("*** END PROGRAM ***")

    @timeit
    def run(self, count, time0):
        if not self.robot.recorder.playback:
            # read webcam and get distance from aruco markers
            _, raw_img, cam_fps, img_created = self.robot.camera.read()  # BGR color

            speed = self.speed
            turn = self.turn
        else:
            cam_fps = 0
            raw_img, speed, turn = next(self.robot.recorder.get_step)

        if raw_img is None:
            print("[red]image is None!")
            return

        if self.mode == TaskPart.Race:
            time_ekf = timer()
            draw_img = raw_img
            data = self.robot.run_ekf_slam(raw_img, fastmode=True)
            self.dt_ekfslam = timer()-time_ekf
        else:
            draw_img = raw_img.copy()
            data = self.robot.run_ekf_slam(raw_img, draw_img)

        self.parse_keypress(raw_img, count, data)

        # manual driving
        if self.mode == TaskPart.Manual:
            self.robot.move(self.speed, self.turn)

        # autonomous exploration with reactive control
        if self.mode == TaskPart.Exploration:
            self.speed, self.turn, end_reached = self.wanderer.tramp(data)
            self.robot.move(self.speed, self.turn)
            if end_reached > 5:
                print('END REACHED')
                self.speed, self.turn = 0, 0
                self.starter = GoToStart(data)
                self.mode = TaskPart.ToStartLine
        
        # check orientation when at start line 
        if self.mode == TaskPart.ToStartLine:
            print('I am in to start line')
            angle_to_start = np.rad2deg(self.starter.angle_to_start(data))
            print('angle to start', angle_to_start)
            if abs(angle_to_start) > 10:
                self.robot.vehicle.drive_turn(
                    angle_to_start, 0.05, speed=5).start(thread=False)

            # if start_line_reached:
            print('Switching to prepare race')
            self.mode = TaskPart.PrepareRace

        # race
        if self.mode == TaskPart.Race:
            self.speed, self.turn, end_reached = self.runner.run_path(
                data, self.dt_ekfslam)
            self.robot.move(self.speed, self.turn)
            if end_reached:
                self.mode = TaskPart.Manual
                self.speed = 0
                self.turn = 0
                self.robot.move(self.speed, self.turn)
                print("Oh yeah")

        if self.mode == TaskPart.PrepareRace:
            print('Preparing race')
            if self.starter == None:
                self.starter = GoToStart(data)

            start, end = self.starter.get_path_start_end()
            self.runner = Runner(data, start, end, self.config)
            self.mode = TaskPart.Race
            print('Race ready')

        if self.mode == TaskPart.Load:
            recalled_memories = self.load_state()
            self.robot.slam.load_map(*recalled_memories)
            self.mode = TaskPart.Manual

        msg = Message(
            id=count,
            timestamp=time0,
            start=True,

            landmark_ids=data.landmark_ids,
            landmark_rs=data.landmark_rs,
            landmark_alphas=data.landmark_alphas,
            landmark_positions=data.landmark_positions,

            landmark_estimated_ids=data.landmark_estimated_ids,
            landmark_estimated_positions=data.landmark_estimated_positions,
            landmark_estimated_stdevs=data.landmark_estimated_stdevs,

            robot_position=data.robot_position,
            robot_theta=data.robot_theta,
            robot_stdev=data.robot_stdev,

            text=f"cam fps: {cam_fps}\ntheta gyro: {data.theta_gyro}\ntheta robo: {data.robot_theta}"
        )

        msg_str = jsonpickle.encode(msg)
        self.publisher.publish_img(msg_str, draw_img)

    def save_state(self, data):
        data = {"positions": data.landmark_estimated_positions,
                "ids": data.landmark_estimated_ids,
                "robot_pose": data.robot_position,
                "robot_theta": data.robot_theta}
        with open('pathfinding/SLAM'+str(datetime.now().strftime("%Y%m%d_%H%M%S"))+'.pickle', 'wb') as pickle_file:
            pickle.dump(data, pickle_file)

    def load_state(self, pickle_file_path='SLAM_DUMP.pickle'):
        with open(pickle_file_path, 'rb') as file:
            loaded_data = pickle.load(file)
        ids = loaded_data['ids']
        index_to_ids = loaded_data['index_to_ids']
        n_ids = loaded_data['n_ids']
        mu = loaded_data['mu']
        sigma = loaded_data['sigma']
        # to avoid map destruction upon map loading and repositioning
        sigma[0, 0] = 100000000
        sigma[1, 1] = 100000000
        sigma[2, 2] = 100000000

        return (ids, index_to_ids, n_ids, mu, np.copy(sigma))

    @timeit
    def parse_keypress(self, raw_img, count, data):
        char = self.keypress_listener.get_keypress()

        turn_step = 40
        speed_step = 15

        if char == "a":
            if self.turn >= 0:
                self.new_turn = self.turn + turn_step
            else:
                self.new_turn = 0
            self.new_turn = min(self.new_turn, 200)
        elif char == "d":
            if self.turn <= 0:
                self.new_turn = self.turn - turn_step
            else:
                self.new_turn = 0
            self.new_turn = max(self.new_turn, -200)
        elif char == "w":
            if self.speed >= 0:
                self.new_speed = self.speed + speed_step
            else:
                self.new_speed = 0
            self.new_speed = min(self.new_speed, 100)
        elif char == "s":
            if self.speed <= 0:
                self.new_speed = self.speed - speed_step
            else:
                self.new_speed = 0
            self.new_speed = max(self.new_speed, -100)
        elif char == "c":
            self.new_speed = 0
            self.new_turn = 0
        elif char == "q":
            self.new_speed = 0
            self.new_turn = 0
        elif char == "k":
            self.new_speed = 0
            self.new_turn = 0
            self.is_running = False
        elif char == "z":
            self.new_speed = 4
            self.new_turn = 200
        elif char == "x":
            self.new_speed = 4
            self.new_turn = -200
        elif char == "m":
            self.new_speed = 0
            self.new_turn = 0
            self.mode = TaskPart.Manual
            print("[green]MODE: Manual")
        elif char == "r":
            self.mode = TaskPart.Race
            print("[green]MODE: Race")
        elif char == "l":
            self.mode = TaskPart.Load
            print("[green]MODE: Load map")
        elif char == "p":
            self.mode = TaskPart.PrepareRace
            print("[green]MODE: Prepare race path")
        elif char == "e":
            self.mode = TaskPart.Exploration
            print("[green]MODE: Exploration")
        elif char == "8":
            print("[green]MODE: Saveed picture")
            cv2.imwrite(
                'pics/pic_'+str(datetime.now().strftime("%Y%m%d_%H%M%S"))+'.png', raw_img)
        elif char == "j":
            self.save_state(data)
            print("[green]Saved ids and landmark positions")

        if self.mode == TaskPart.Manual:
            if self.speed != self.new_speed or self.turn != self.new_turn:
                self.speed = self.new_speed
                self.turn = self.new_turn
                print("speed:", self.speed, "turn:", self.turn)


if __name__ == '__main__':

    main = Main()
