import threading
import numpy as np
import cv2
import lib.constants as consts
from lib.robotics.inverse_kinematics import IKSolver
from lib.robotics.pose_generation.pose_generator import PoseGenerator
import time

__STEP_BY_STEP__ = False

class LoopHandler:
    def __init__(self, pose_gen: PoseGenerator, comm):
        self.pose_gen = pose_gen
        self.comm = comm
        self.ik = IKSolver(self.pose_gen.pos)
        self.goal_reached = True
        self.travel_start_t = 0.0
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self, threaded=True):
        if threaded:
            self.thread.start()
        else:
            self.run()

    def run(self):
        while True:
            start_time = time.time()
            self.pose_gen.update()
            pos = self.pose_gen.pos
            thetas_list = []
            for i in range(len(pos)):
                pos_i = pos[i]
                thetas, reached = self.ik.calculate_joint_angles_dt(pos_i, step_by_step=__STEP_BY_STEP__)

                # thetas1 thetas2 thetas3 thetas4 thetas5...
                thetas_list.append(thetas)
                # for j, theta in enumerate(thetas):
                #     self.comm.send_command(f"set_position {i*3 + j} {theta:.2f}")
                #     time.sleep(consts.DT / 8)
            # flatten thetas_list
            thetas_list = [theta for sublist in thetas_list for theta in sublist]
            if None not in thetas_list:
                angles_separated_by_commas = ','.join(f"{theta:.2f}" for theta in thetas_list)
                self.comm.send_command(f"set_positions {angles_separated_by_commas}")

            # if self.goal_reached and not reached:
            #     self.travel_start_t = time.time()
            # elif not self.goal_reached and reached:
            #     print(f"Travel time: {time.time() - self.travel_start_t:.2f} sec")

            # self.goal_reached = reached

            # if __STEP_BY_STEP__:
            time.sleep(0.10)
            # input("Press Enter to continue...")
            # import pdb; pdb.set_trace()

            elapsed = time.time() - start_time
            if elapsed < consts.DT:
                time.sleep(consts.DT - elapsed)
