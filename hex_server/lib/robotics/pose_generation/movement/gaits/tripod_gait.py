import numpy as np
import lib.constants as consts
from lib.robotics.pose_generation.movement.gaits.gait_engine import GaitEngine, State

class TripodGaitEngine(GaitEngine):
    def __init__(self):
        super().__init__()

        self.leg_groups = np.array([[0, 2, 4], [1, 3, 5]], dtype=np.int32)

        self.last_direction = np.array([0.0, 0.0], dtype=np.float32)
        self.stp = 5.0
        self.lift = 3.0

        self.direction = self._generate_goal(self.last_direction)
        self.n_phases = 6

    def _generate_goal(self, direction_vec: np.ndarray):
        """Genera la secuencia de movimientos (6 fases) según la dirección"""
        # Genera avance y retroceso en dirección normalizada
        norm = np.linalg.norm(direction_vec)
        if norm == 0:
            dir_unit = np.array([0.0, 0.0])
        else:
            dir_unit = direction_vec / norm

        return dir_unit * self.stp / 2.0
 
    def _get_idling_targets(self, leg_positions: np.ndarray) -> dict:
        targets = leg_positions.copy()
        swing_legs = self.leg_groups[self.current_swing_group]


        if self.phase_counter%3 == 0:
            # make the z dimension equal to the INITIAL_POSITIONS_BODY height plus lift
            targets[swing_legs][2] = consts.INITIAL_POSITIONS_BODY[swing_legs][:,2] + self.lift
        elif self.phase_counter%3 == 1:
            targets[swing_legs] = consts.INITIAL_POSITIONS_BODY[swing_legs] + np.array([0, 0, self.lift])
        elif self.phase_counter%3 == 2:
            targets[swing_legs] = consts.INITIAL_POSITIONS_BODY[swing_legs]
            self.current_swing_group = 1 - self.current_swing_group
        return targets

    def get_step_targets(self, leg_positions: np.ndarray, direction_vec: np.ndarray, target_positions: np.ndarray, is_at_target: bool) -> dict:
        # print("Current state:", self.state)
        # return consts.INITIAL_POSITIONS_BODY.copy()
        print("State:", self.state, "Phase:", self.phase_counter, "Swing group:", self.current_swing_group)
        if not np.allclose(direction_vec, self.last_direction):
            self.direction = self._generate_goal(direction_vec)
            self.last_direction = direction_vec.copy()
            if np.linalg.norm(direction_vec) == 0:
                print("Switching to IDLING")
                self.state = State.IDLING
                self.phase_counter = 1
                self.target_positions = self._get_idling_targets(leg_positions)

        elif is_at_target and self.state == State.IDLING:
            self.phase_counter = (self.phase_counter + 1) % self.n_phases
            if self.phase_counter == 0:
                self.state = State.IDLE
            self.target_positions = self._get_idling_targets(leg_positions)


        if is_at_target and self.state == State.IDLE and np.linalg.norm(direction_vec) > 0:
            self.state = State.UNIDLING
            self.current_swing_group = 0
            self.phase_counter = 0

            swing_legs = self.leg_groups[self.current_swing_group]
            self.target_positions = leg_positions.copy()
            self.target_positions[swing_legs] += np.array([0, 0, self.lift])
            
        elif is_at_target and self.state == State.UNIDLING:
            self.state = State.WALKING

        if self.state == State.WALKING:

            # print("phase:", self.phase_counter, "swing group:", self.current_swing_group)
            # print("Targets before phase update:\n", self.target_positions[0:1])
            self.target_positions = consts.INITIAL_POSITIONS_BODY.copy()
            swing_legs = self.leg_groups[self.current_swing_group]
            stance_legs = self.leg_groups[1 - self.current_swing_group]

            if is_at_target:
                # print("\n\n\n-----------------------------------\n\n\n--------TARGETS---------\n\n\n")
                # print("Phase:", self.phase_counter, "Swing group:", self.current_swing_group)
                if self.phase_counter%3 == 2:
                    self.current_swing_group = 1 - self.current_swing_group
                self.phase_counter = (self.phase_counter + 1) % self.n_phases

            if self.phase_counter%3 == 0:
                self.target_positions[swing_legs] += np.append(-self.direction, self.lift)
                self.target_positions[stance_legs] += np.append(self.direction, 0.0)
            elif self.phase_counter%3 == 1:
                # import pdb; pdb.set_trace()
                self.target_positions[swing_legs] += np.append(self.direction, self.lift)
                self.target_positions[stance_legs] += np.append(-self.direction, 0.0)
            elif self.phase_counter%3 == 2:
                self.target_positions[swing_legs] += np.append(self.direction, 0.0)
                self.target_positions[stance_legs] += np.append(-self.direction, 0.0)

            
                # print("Phase:", self.phase_counter, "Swing group:", self.current_swing_group)
                # print("Targets:\n", self.target_positions)
                # import pdb; pdb.set_trace()
            # print("phase:", self.phase_counter, "swing group:", self.current_swing_group)
            # print("Targets after phase update:\n", self.target_positions[0:1])
            # print(targets)
            # print(self.target_positions)
            # import pdb; pdb.set_trace()

        return self.target_positions
