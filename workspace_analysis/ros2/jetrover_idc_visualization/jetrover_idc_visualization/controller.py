"""Pure fixed-sample playback state machine; never searches or interpolates motion."""
from typing import Any
import numpy as np
from .data import Catalog, Trajectory, PHASES
from .errors import ReplayError


class ReplayController:
    """Single-threaded owner; selection/reset are virtual jumps, not planned motions."""
    def __init__(self, catalog: Catalog):
        self.catalog = catalog
        self.state = 'STOPPED'; self.index = 0; self.revision = 0; self.sequence = None
        self.port_id = next(iter(catalog.ports)); self.goal_id = None; self.planner = None; self.phase = 'Path'
        self._first = False
        port = catalog.ports[self.port_id]
        if port.goals:
            goal = port.goals[0]
            self.select(self.port_id, goal.solution_id, next(iter(goal.paths)), 'Path')

    def select(self, port_id: str, goal_id: int | None, planner: str | None, phase: str) -> None:
        # Even an invalid switch cannot leave an old animation running.
        self.state = 'STOPPED'
        if port_id not in self.catalog.ports or phase not in PHASES:
            raise ReplayError('UNAVAILABLE_SELECTION', 'unknown port/phase')
        port = self.catalog.ports[port_id]
        sequence = None
        if port.goals:
            if type(goal_id) is not int: raise ReplayError('GOAL_ID', 'explicit saved goal ID required')
            goal = next((g for g in port.goals if g.solution_id == goal_id), None)
            if goal is None: raise ReplayError('GOAL_ID', 'unknown goal')
            sequence = goal.sequence(planner, phase)
        elif goal_id is not None or planner is not None:
            raise ReplayError('UNAVAILABLE_SELECTION', 'failed port has no playable goal/planner')
        self.port_id, self.goal_id, self.planner, self.phase = port_id, goal_id, planner, phase
        self.sequence = sequence; self.index = 0; self.revision += 1; self._first = False

    def play(self) -> None:
        if self.sequence is None: raise ReplayError('NO_VALID_TRAJECTORY', self.catalog.ports[self.port_id].detail)
        if self.state == 'FINISHED': self.index = 0
        self.state = 'PLAYING'; self._first = True

    def pause(self) -> None:
        if self.state != 'PLAYING': raise ReplayError('PLAYBACK_STATE', 'pause requires PLAYING')
        self.state = 'PAUSED'

    def resume(self) -> None:
        if self.state != 'PAUSED': raise ReplayError('PLAYBACK_STATE', 'resume requires PAUSED')
        self.state = 'PLAYING'; self._first = True

    def reset(self) -> None:
        self.state = 'STOPPED'; self.index = 0; self._first = False

    def tick(self) -> dict:
        if self.state == 'PLAYING':
            if self._first: self._first = False
            elif self.index < len(self.sequence.q)-1: self.index += 1
            if self.index == len(self.sequence.q)-1: self.state = 'FINISHED'
        return self.current()

    def current(self) -> dict:
        sequence = self.sequence
        return {'q':list(self.catalog.start_q) if sequence is None else sequence.q[self.index].tolist(),
                'tcp':self.catalog.start_tcp if sequence is None else sequence.tcp[self.index],
                'sample_index':self.index, 'total_samples':0 if sequence is None else len(sequence.q),
                'source_frame':None if sequence is None else sequence.source_frames[self.index],
                'query_id':None if sequence is None else int(sequence.query_id[self.index]),
                'current_phase':None if sequence is None else sequence.phases[self.index]}

    def status(self) -> dict:
        frame = self.current()
        return {'state':self.state, 'selection_revision':self.revision, 'port_id':self.port_id,
                'goal_id':self.goal_id, 'planner':self.planner, 'phase':self.phase,
                **{k:v for k,v in frame.items() if k != 'tcp'}, 'playable':self.sequence is not None,
                'task_status':self.catalog.ports[self.port_id].status,
                'task_detail':self.catalog.ports[self.port_id].detail,
                'usage':'RESEARCH_ONLY', 'time_parameterized':False, 'continuous_collision_proven':False}
