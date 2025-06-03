from __future__ import annotations

from collections.abc import Iterator

import numpy as np
import shapely as s
from navground import sim
from ri_planning.map.map import Map


def lines(wall: s.LineString, scale: float) -> Iterator[np.ndarray]:
    return (np.array(ps) / scale for ps in zip(wall.coords, wall.coords[1:]))


def get_walls(layer) -> list[s.LineString]:
    wall_cells = [c for c in layer.cells.values() if c.type == 'CellSpace']
    return [
        b.geometry for c in wall_cells for b in c.boundary
        if set(b.cells) - set(wall_cells)
    ]


class IndoorGMLScenario(sim.Scenario,
                        name="IndoorGML"):  # type: ignore[call-arg]

    def __init__(self, map_path: str = ''):
        super().__init__()
        self._map_path = map_path
        self._scale = 100.0

    def init_world(self, world: sim.World, seed: int | None = None) -> None:
        super().init_world(world, seed)
        world._world_map = Map.from_file(self._map_path)  # type: ignore
        world._world_map_scale = self._scale  # type: ignore
        layer = world._world_map.geometric_layer  # type: ignore
        walls = get_walls(layer)
        for wall in walls:
            for p1, p2 in lines(wall, self._scale):
                world.add_wall(sim.Wall(p1, p2))

    @property
    @sim.register('', "IndoorGML map path")
    def map_path(self) -> str:
        return self._map_path

    @map_path.setter  # type: ignore[no-redef]
    def map_path(self, value: str) -> None:
        self._map_path = value

    @property
    @sim.register(100.0, "IndoorGML map scale [units per meter]")
    def map_scale(self) -> float:
        return self._scale

    @map_scale.setter  # type: ignore[no-redef]
    def map_scale(self, value: float) -> None:
        self._scale = value
