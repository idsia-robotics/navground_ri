from __future__ import annotations

import itertools
from typing import Any

import numpy as np
from navground import core, sim

from .task import Map, Task


class WaypointsWithPlannerTask(sim.Task, Task, name="WaypointsWithRIPlanner"
                               ):  # type: ignore[call-arg]

    def __init__(self):
        sim.Task.__init__(self)
        Task.__init__(self)

    @property
    @sim.register(False, "Whether to compute plans in a separate thread")
    def threaded(self) -> bool:
        return self._threaded

    @threaded.setter  # type: ignore[no-redef]
    def threaded(self, value: bool) -> None:
        self._threaded = value

    @property
    @sim.register(
        'bezier_4',
        "Type of curve to compute: one of 'line', 'bezier_4', 'bezier_6'")
    def curve(self) -> str:
        return self._curve

    @curve.setter  # type: ignore[no-redef]
    def curve(self, value: str) -> None:
        self._curve = value

    @property
    @sim.register([np.array((0.0, 0.0))], "A list of waypoints (in meters)")
    def waypoints(self) -> list[core.Vector2]:
        return self._waypoints

    # TODO(Jerome): complete
    @waypoints.setter  # type: ignore[no-redef]
    def waypoints(self, value: list[core.Vector2]) -> None:
        self._waypoints = value
        self._waypoints_iter = itertools.cycle(value)

    @property
    @sim.register(False, "Whether to loop over waypoints")
    def loop(self) -> bool:
        return self._loop

    @loop.setter  # type: ignore[no-redef]
    def loop(self, value: bool) -> None:
        self._loop = value

    @property
    @sim.register(1.0, "The waypoint tolerance")
    def tolerance(self) -> float:
        return self._tolerance

    @tolerance.setter  # type: ignore[no-redef]
    def tolerance(self, value: float) -> None:
        self._tolerance = value

    @property
    @sim.register('', "The IndoorGML map path")
    def map_path(self) -> str:
        return self._map_path

    @map_path.setter  # type: ignore[no-redef]
    def map_path(self, value: str) -> None:
        self._map_path = value

    @property
    @sim.register(100.0, "The IndoorGML map scale [units per meter]")
    def map_scale(self) -> float:
        return self._scale

    @map_scale.setter  # type: ignore[no-redef]
    def map_scale(self, value: float) -> None:
        self._scale = value

    @property
    @sim.register(0.0, "The path following horizon")
    def follow_horizon(self) -> float:
        return self._follow_horizon

    @follow_horizon.setter  # type: ignore[no-redef]
    def follow_horizon(self, value: float) -> None:
        self._follow_horizon = value

    @property
    @sim.register('', "The IndoorGML layer to use for planning")
    def layer_name(self) -> str:
        return self._layer_name

    @layer_name.setter  # type: ignore[no-redef]
    def layer_name(self, value: str) -> None:
        self._layer_name = value

    @property
    @sim.register(0.01, "The optimization tolerance")
    def opt_tol(self) -> float:
        return self._opt_tol

    @opt_tol.setter  # type: ignore[no-redef]
    def opt_tol(self, value: float) -> None:
        self._opt_tol = value

    def prepare(self, agent: sim.Agent, world: sim.World) -> None:
        if self.map_path:
            world_map = Map.from_file(self.map_path)
        else:
            try:
                world_map = world._world_map  # type: ignore
                self._scale = world._world_map_scale  # type: ignore
            except AttributeError:
                raise ValueError("No map provided")
        if not agent.behavior:
            raise ValueError("No behavior")
        self.setup(world_map=world_map,
                   horizon=agent.behavior.horizon,
                   safety_margin=agent.behavior.safety_margin,
                   optimal_speed=agent.behavior.optimal_speed)

    def update(self, agent: sim.Agent, world: sim.World, time: float) -> None:
        self.tick(agent.position, agent.orientation, agent.velocity, agent)

    def set_horizon(self, value: float, agent: sim.Agent, *args: Any,
                    **kwargs: Any) -> None:
        if agent.behavior:
            agent.behavior.horizon = value

    def set_safety_margin(self, value: float, agent: sim.Agent, *args: Any,
                          **kwargs: Any) -> None:
        if agent.behavior:
            agent.behavior.safety_margin = value

    def set_line_obstacles(self, value: list[tuple[core.Vector2,
                                                   core.Vector2]],
                           agent: sim.Agent, *args: Any,
                           **kwargs: Any) -> None:
        if (agent.behavior and isinstance(agent.behavior.environment_state,
                                          core.GeometricState)):
            agent.behavior.environment_state.line_obstacles = [
                core.LineSegment(p1, p2) for p1, p2 in value
            ]

    def set_target_velocity(self, value: core.Vector2, agent: sim.Agent, *args:
                            Any, **kwargs: Any) -> None:
        agent.controller.follow_velocity(value)
