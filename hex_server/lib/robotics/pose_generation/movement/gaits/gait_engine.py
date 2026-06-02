import numpy as np
from enum import Enum, auto
import lib.constants as consts

class State(Enum):
    IDLE = auto()
    UNIDLING = auto()
    WALKING = auto()
    IDLING = auto()

class GaitEngine:
    """
    Base class for all gaits. Subclasses only need to define leg_groups.

    All position math is in body frame (consts.INITIAL_POSITIONS_BODY as home).
    rotation_speed is a signed float; positive = CCW viewed from above.
    The tangent contribution scales with ROT_SCALE so the PS-controller range
    (rx/10 → ±0.1) produces the same step magnitude as full-speed translation.
    """

    # Subclasses override these as class attributes
    leg_groups = []             # list of np.int32 arrays
    dead_legs = []              # list of leg indices that don't move in this gait
    stp = consts.STEP_LENGTH    # half-stroke in cm  (foot travels ±stp/2 from home)
    lift = consts.LIFT_HEIGHT   # foot lift height in cm
    ROT_SCALE = 50.0            # rotation_speed → gait units  (matches rx/10 range)
    step_phases = 3             # number of phases in a full step cycle (e.g. lift, forward, down)
    idle_phases = 2             # number of phases in a full idle cycle (e.g. lift, down)

    def __init__(self):
        self.phase_to_print = 0
        self.current_swing_group = 0
        self.phase_counter = 0
        self.state = State.IDLE
        self.target_positions = consts.INITIAL_POSITIONS_BODY.copy()

    # ------------------------------------------------------------------ helpers

    @property
    def _n_groups(self):
        return len(self.leg_groups)

    @property
    def _n_walk_phases(self):
        return self._n_groups * self.step_phases

    @property
    def _n_idle_phases(self):
        return self._n_groups * self.idle_phases

    def _per_leg_step(self, leg_idx, direction_vec, rotation_speed):
        """Per-leg step vector combining translation and rotation."""
        home = consts.INITIAL_POSITIONS_BODY[leg_idx]
        px, py = home[0], home[1]
        r = np.hypot(px, py)
        tangent = np.array([-py, px]) / r if r > 1e-6 else np.zeros(2)
        return (direction_vec + tangent * rotation_speed * self.ROT_SCALE) * self.stp / (2.0 * (self._n_groups - 1))

    # TODO - add a "swing height" parameter to leg_groups and interpolate z in _walk_targets for smoother motion?
    def _walk_targets(self, swing_group, phase_in_group, direction_vec, rotation_speed):
        targets = consts.INITIAL_POSITIONS_BODY.copy()
        swing_legs = self.leg_groups[swing_group]
        stance_legs = np.concatenate(
            [self.leg_groups[i] for i in range(self._n_groups) if i != swing_group]
        )

        for leg in swing_legs:
            d = self._per_leg_step(leg, direction_vec, rotation_speed)
            if phase_in_group == 0:
                targets[leg] = self.current_leg_positions[leg]
                targets[leg][2] = consts.INITIAL_POSITIONS_BODY[leg][2] + self.lift
            elif phase_in_group == 1:
                targets[leg] = consts.INITIAL_POSITIONS_BODY[leg] + np.array([ d[0],  d[1], self.lift])
            else:
                targets[leg] = consts.INITIAL_POSITIONS_BODY[leg] + np.array([ d[0],  d[1], 0.0])

        for leg in stance_legs:
            d = self._per_leg_step(leg, direction_vec, rotation_speed)
            if phase_in_group == 0:
                targets[leg] = self.current_leg_positions[leg]
            else:
                targets[leg] = consts.INITIAL_POSITIONS_BODY[leg] + np.array([-d[0], -d[1], 0.0])

            if leg == 1 and self.phase_to_print == phase_in_group:
                print("phase:", phase_in_group, "current:", self.current_leg_positions[leg], "target:", targets[leg], "d:", d)
                self.phase_to_print = (self.phase_to_print + 1) % 3
        return targets

    def _idle_targets(self):
        """Return legs to home one group at a time (lift then land)."""
        targets = consts.INITIAL_POSITIONS_BODY.copy()
        group_idx = (self.current_swing_group + self.phase_counter // self.idle_phases) % self._n_groups
        swing_legs = self.leg_groups[group_idx]
        if self.phase_counter % self.idle_phases == 0:   # lift phase
            targets[swing_legs] = (consts.INITIAL_POSITIONS_BODY[swing_legs]
                                   + np.array([0.0, 0.0, self.lift]))
        # odd phase: land at home (already the default in targets)
        return targets

    # ------------------------------------------------------------------ main

    def get_step_targets(self, leg_positions, direction_vec, rotation_speed,
                         is_at_target):
        """
        Called every control loop tick.
        leg_positions, target_positions: (6,3) in body frame.
        Returns (6,3) target positions in body frame.
        """

        self.current_leg_positions = leg_positions

        moving = (np.linalg.norm(direction_vec) > 0.01
                  or abs(rotation_speed) > 0.001)

        # --- state transitions ------------------------------------------------
        if self.state == State.IDLE and moving and is_at_target:
            self.state = State.UNIDLING
            self.current_swing_group = 0
            self.phase_counter = 0
            self.target_positions = consts.INITIAL_POSITIONS_BODY.copy()
            self.target_positions[self.leg_groups[0]] += np.array([0.0, 0.0, self.lift])

        elif self.state == State.UNIDLING and is_at_target:
            self.state = State.WALKING

        # walking → idling when input stops
        if self.state == State.WALKING and not moving:
            self.state = State.IDLING
            self.current_swing_group = self.current_swing_group
            self.phase_counter = 0
            self.target_positions = self._idle_targets()

        elif self.state == State.IDLING and is_at_target:
            self.phase_counter += 1
            if self.phase_counter >= self._n_idle_phases:
                self.state = State.IDLE
                self.target_positions = consts.INITIAL_POSITIONS_BODY.copy()
            else:
                self.target_positions = self._idle_targets()

        # --- walking tick -----------------------------------------------------
        if self.state == State.WALKING:
            if is_at_target:
                if self.phase_counter % self.step_phases == self.step_phases - 1:
                    self.current_swing_group = (self.current_swing_group + 1) % self._n_groups
                self.phase_counter = (self.phase_counter + 1) % self._n_walk_phases
            phase_in_group = self.phase_counter % self.step_phases
            self.target_positions = self._walk_targets(
                self.current_swing_group, phase_in_group, direction_vec, rotation_speed
            )

        if len(self.dead_legs) > 0:
            self.target_positions[self.dead_legs] = consts.INITIAL_POSITIONS_BODY[self.dead_legs] + np.array([0.0, 0.0, self.lift])

        return self.target_positions
