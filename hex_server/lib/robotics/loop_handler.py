import threading
import numpy as np
import cv2
import lib.constants as consts
from lib.robotics.inverse_kinematics import IKSolver
from lib.robotics.pose_generation.pose_generator import PoseGenerator
import time

class LoopHandler:
    def __init__(self, pose_gen: PoseGenerator, comm):
        self.pose_gen = pose_gen
        self.comm = comm
        self.ik = IKSolver(self.pose_gen.pos)
        self.goal_reached = True
        self.travel_start_t = 0.0
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        while True:
            start_time = time.time()
            self.pose_gen.update()
            pos = self.pose_gen.pos
            thetas, reached = self.ik.calculate_joint_angles_dt(pos)

            if None not in thetas:
                for i, theta in enumerate(thetas):
                    self.comm.send_command(f"set_position {i} {theta}")
                    time.sleep(consts.DT / 4)

            if self.goal_reached and not reached:
                self.travel_start_t = time.time()
            elif not self.goal_reached and reached:
                print(f"Travel time: {time.time() - self.travel_start_t:.2f} sec")

            self.goal_reached = reached

            elapsed = time.time() - start_time
            if elapsed < consts.DT:
                time.sleep(consts.DT - elapsed)
