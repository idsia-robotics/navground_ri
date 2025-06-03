from __future__ import annotations

import concurrent.futures
import itertools
import warnings
from collections.abc import Iterator
from typing import Any

import numpy as np
import shapely
from ri_planning.map.map import Map
from ri_planning.planning.bezier_planner import Planner
from ri_planning.planning.visibility_planner import \
    Planner as VisibilityPlanner
from ri_planning.utilities import Pose, unit
from shapely.ops import unary_union

warnings.filterwarnings("ignore")

Vector2 = np.ndarray


def plan_boundary(map_, plan):
    ss = set()
    layer_id = map_.geometric_layer.id
    for state in plan.states:
        ss |= map_.states[state].overlaps[layer_id]
        ss |= map_.states[state].inside[layer_id]
    ps = [map_.states[s].duality.geometry for s in ss]
    g = unary_union(ps)
    b = g.boundary.simplify(10)
    return b


class Task:

    _waypoints_iter: Iterator[Vector2]
    _planners: dict[tuple[str, str, float, float], Planner] = {}

    def __init__(self,
                 waypoints: list[Vector2] = [],
                 loop: bool = False,
                 tolerance: float = 1.0,
                 opt_tol: float = 0.01,
                 map_scale: float = 100.0,
                 map_path: str = '',
                 layer_name: str = '',
                 follow_horizon: float = 0.5,
                 path_tau: float = 5.0,
                 curve: str = 'bezier_4',
                 threaded: bool = False):
        super().__init__()
        self.waypoints = waypoints
        self.loop = loop
        self.tolerance = tolerance
        self.map_path = map_path
        self.layer_name = layer_name
        self.opt_tol = opt_tol
        self.map_scale = map_scale
        self.follow_horizon = follow_horizon
        self.path_tau = path_tau
        self.curve = curve
        self.threaded = threaded
        self.plan = None
        self.waypoint: np.ndarray | None = None
        self._horizon = 0.0
        self._safety_margin = 0.0
        self._optimal_speed = 0.0
        self._new_plan = False
        self.executor: concurrent.futures.ThreadPoolExecutor | None = None

    @property
    def threaded(self) -> bool:
        return self._threaded

    @threaded.setter  # type: ignore[no-redef]
    def threaded(self, value: bool) -> None:
        self._threaded = value

    @property
    def curve(self) -> str:
        return self._curve

    @curve.setter  # type: ignore[no-redef]
    def curve(self, value: str) -> None:
        self._curve = value

    @property
    def waypoints(self) -> list[Vector2]:
        return self._waypoints

    # TODO(Jerome): complete
    @waypoints.setter  # type: ignore[no-redef]
    def waypoints(self, value: list[Vector2]) -> None:
        self._waypoints = [np.asarray(p) for p in value]
        self._waypoints_iter = itertools.cycle(self._waypoints)

    @property
    def loop(self) -> bool:
        return self._loop

    @loop.setter  # type: ignore[no-redef]
    def loop(self, value: bool) -> None:
        self._loop = value

    @property
    def tolerance(self) -> float:
        return self._tolerance

    @tolerance.setter  # type: ignore[no-redef]
    def tolerance(self, value: float) -> None:
        self._tolerance = value

    @property
    def map_path(self) -> str:
        return self._map_path

    @map_path.setter  # type: ignore[no-redef]
    def map_path(self, value: str) -> None:
        self._map_path = value

    @property
    def map_scale(self) -> float:
        return self._scale

    @map_scale.setter  # type: ignore[no-redef]
    def map_scale(self, value: float) -> None:
        self._scale = value

    @property
    def follow_horizon(self) -> float:
        return self._follow_horizon

    @follow_horizon.setter  # type: ignore[no-redef]
    def follow_horizon(self, value: float) -> None:
        self._follow_horizon = value

    @property
    def layer_name(self) -> str:
        return self._layer_name

    @layer_name.setter  # type: ignore[no-redef]
    def layer_name(self, value: str) -> None:
        # print('set layer_name', value)
        self._layer_name = value

    @property
    def opt_tol(self) -> float:
        return self._opt_tol

    @opt_tol.setter  # type: ignore[no-redef]
    def opt_tol(self, value: float) -> None:
        self._opt_tol = value

    def done(self) -> bool:
        return False

    @classmethod
    def make_planner(cls, world_map: Map, layer_name: str, tol: float,
                     min_width: float) -> Planner:
        key = (world_map.source, layer_name, tol, min_width)
        if key not in cls._planners:
            layer = world_map[layer_name]
            cls._planners[key] = Planner(layer,
                                         tol=tol,
                                         sparse=True,
                                         use_cache=True,
                                         min_width=min_width)
        return cls._planners[key]

    def setup(self, world_map: Map, horizon: float, safety_margin: float,
              optimal_speed: float) -> None:
        self.planner = self.make_planner(world_map=world_map,
                                         layer_name=self.layer_name,
                                         tol=self.opt_tol,
                                         min_width=1. * self._scale)
        if self.threaded:
            self.executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=1)
        else:
            self.executor = None
        self._horizon = horizon
        self._safety_margin = safety_margin
        self._optimal_speed = optimal_speed

    def has_arrived(self, position: Vector2) -> bool:
        dist = float(np.linalg.norm(self.waypoint - position))
        return dist < self.tolerance

    def update_plan(self, position: Vector2, orientation: float) -> None:
        assert self.waypoint is not None, "Unexpected None waypoint in Task.update_plan"
        start = Pose(tuple(position * self._scale), orientation)
        end = Pose(tuple(self.waypoint * self._scale))
        if self.curve == 'line':
            self.plan = VisibilityPlanner.shortest_path(
                self.planner, start, end)
        else:
            self.plan = self.planner.shortest_path(start, end)
        self._new_plan = True

    def follow_plan(self, position: Vector2, speed: float) -> Vector2:
        delta_s = self.follow_horizon * self._scale
        assert self.plan is not None, "Unexpected None plan in Task.follow_plan"
        pose, dist = self.plan.project(position * self._scale, delta_s)
        # print('pose', pose, 'dist', dist)
        e = unit(pose.orientation)
        delta = np.asarray(pose.position) / self._scale - position
        l_speed = speed if dist > delta_s else speed * dist / delta_s
        v = e * l_speed + delta / self.path_tau
        n = np.linalg.norm(v)
        if n > 0:
            return v / n * speed
        return np.zeros(2)

    def tick(self, position: Vector2, orientation: float, velocity: Vector2, *args: Any,
             **kwargs: Any) -> None:
        if self.waypoint is None or self.has_arrived(position):
            self.waypoint = next(self._waypoints_iter)
            self.plan = None
            if self.executor:
                self.executor.submit(self.update_plan, position, orientation)
            else:
                self.update_plan(position, orientation)
        if not self.plan:
            velocity = np.zeros(2)
            self.set_horizon(self._horizon, *args, **kwargs)
            # self.set_safety_margin(self._safety_margin, *args, **kwargs)
        else:
            if self._new_plan:
                self.update_boundary(*args, **kwargs)
                self.has_updated_plan(*args, **kwargs)
                self._new_plan = False
            velocity = self.follow_plan(position, self._optimal_speed)
            self.set_horizon(self.horizon(position, velocity), *args, **kwargs)
            # self.set_safety_margin(self._safety_margin, *args, **kwargs)
        self.set_target_velocity(velocity, *args, **kwargs)

    def update_boundary(self, *args: Any, **kwargs: Any) -> None:
        self.boundary = plan_boundary(self.planner.layer.map, self.plan)
        line_obstacles = [
            (np.asarray(p1) / self._scale, np.asarray(p2) / self._scale)
            for p1, p2 in zip(self.boundary.coords, self.boundary.coords[1:])
        ]
        self.set_line_obstacles(line_obstacles, *args, **kwargs)

    def horizon(self, position: Vector2, velocity: Vector2) -> float:
        n = np.linalg.norm(velocity)
        if n > 0:
            p = self._scale * position
            a = shapely.Point(p)
            delta = self._horizon * self._scale * velocity / n
            line = shapely.LineString([p, p + delta])
            b = shapely.intersection(line, self.boundary)
            if not b.is_empty:
                return b.distance(a) / self._scale
        return self._horizon

    def safety_margin(self, velocity: Vector2) -> float:
        s = float(np.linalg.norm(velocity) / self._optimal_speed)
        min_margin = 0.01
        return min_margin + (self._safety_margin - min_margin) * s

    def set_horizon(self, value: float, *args: Any, **kwargs: Any) -> None:
        ...

    def set_safety_margin(self, value: float, *args: Any,
                          **kwargs: Any) -> None:
        ...

    def set_line_obstacles(self, value: list[tuple[Vector2, Vector2]],
                           *args: Any, **kwargs: Any) -> None:
        ...

    def set_target_velocity(self, value: Vector2, *args: Any,
                            **kwargs: Any) -> None:
        ...

    def has_updated_plan(self, *args: Any, **kwargs: Any) -> None:
        ...
