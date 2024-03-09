# %%
import os
from abc import abstractmethod
from dataclasses import dataclass
from typing import NamedTuple, Optional, Union
from math import *
from global_params import *
from screwable_cylinder import ScrewableCylinder
import build123d as bd
import yacv_server as yacv

# ================== MODELLING ==================


Grid2DF = NamedTuple('Grid2DF', [('x', float), ('y', float)])
Grid2D = NamedTuple('Grid2D', [('x', int), ('y', int)])


@dataclass(kw_only=True)
class GridBase(bd.BasePartObject):
    dimensions: Grid2DF = Grid2DF(10, 10)
    repeat: Grid2D
    total_dimensions: Optional[Grid2DF] = None
    rounded: bool = True

    rotation: bd.RotationLike = (0, 0, 0)
    align: Union[bd.Align, tuple[bd.Align, bd.Align, bd.Align]] = None
    mode: bd.Mode = bd.Mode.PRIVATE

    def __post_init__(self):
        self.total_dimensions = self.total_dimensions or Grid2DF(
            self.dimensions.x * self.repeat.x, self.dimensions.y * self.repeat.y)
        with bd.BuildPart() as part:
            bd.add(self.build_sketch())
            bd.extrude(amount=self.sketch_depth)
        super().__init__(part=part.part, rotation=self.rotation,
                         align=self.align, mode=self.mode)

    def build_sketch(self, *workplanes, inverted: bool = False) -> bd.Sketch:
        with bd.BuildSketch(*workplanes) as sketch:
            if not inverted:
                if self.rounded:
                    bd.RectangleRounded(self.total_dimensions.x, self.total_dimensions.y,
                                     (self.dimensions.x + self.dimensions.y)/2 / 2)
                else:
                    bd.Rectangle(self.total_dimensions.x, self.total_dimensions.y)
            with bd.GridLocations(self.dimensions.x, self.dimensions.y, self.repeat.x, self.repeat.y):
                self._build_sketch(bd.Mode.SUBTRACT if not inverted else bd.Mode.ADD)
        return sketch.sketch

    @abstractmethod
    def _build_sketch(self, mode: bd.Mode):
        ...

    @property
    def sketch_depth(self):
        return wall


@dataclass(kw_only=True)
class GridNutHoles(GridBase):
    minor_radius = ScrewableCylinder.nut_inscribed_diameter / 2
    depth: float = ScrewableCylinder.nut_height

    def _build_sketch(self, mode: bd.Mode):
        major_radius = self.minor_radius / cos(radians(360/6/2))
        assert major_radius + tol < self.dimensions.x/2
        assert major_radius + tol < self.dimensions.y/2
        bd.RegularPolygon(major_radius, 6, mode=mode)

    @property
    def sketch_depth(self):
        return self.depth + tol


@dataclass(kw_only=True)
class GridScrewHeadHoles(GridBase):
    radius = ScrewableCylinder.screw_head_diameter/2
    depth: float = ScrewableCylinder.screw_head_height

    def _build_sketch(self, mode: bd.Mode):
        bd.Circle(self.radius, mode=mode)

    @property
    def sketch_depth(self):
        return self.depth + tol


@dataclass(kw_only=True)
class GridScrewThreadHoles(GridScrewHeadHoles):
    radius = ScrewableCylinder.screw_diameter/2
    wrapped_screw_length: float

    def _build_sketch(self, mode: bd.Mode):
        bd.Circle(self.radius, mode=mode)

    @property
    def sketch_depth(self):
        return self.wrapped_screw_length + tol


@dataclass(kw_only=True)
class GridStack(bd.BasePartObject):
    parts: list[bd.BasePartObject]

    rotation: bd.RotationLike = (0, 0, 0)
    align: Union[bd.Align, tuple[bd.Align, bd.Align, bd.Align]] = None
    mode: bd.Mode = bd.Mode.ADD

    def __post_init__(self):
        offset_z = 0
        with bd.BuildPart() as part:
            for subpart in self.parts:
                part_depth = subpart.bounding_box().size.Z
                bd.add(subpart.move(bd.Location((0, 0, offset_z))))
                offset_z += part_depth
        super().__init__(part=part.part, rotation=self.rotation,
                         align=self.align, mode=self.mode)


if __name__ == "__main__":
    conn_grid = GridStack(parts=[GridNutHoles(repeat=Grid2D(4, 3)), GridScrewThreadHoles(
        repeat=Grid2D(4, 7), wrapped_screw_length=8), GridScrewHeadHoles(repeat=Grid2D(4, 5))])
    if os.getenv('CI', '') != '':
        yacv.export_all('export')
    else:
        yacv.show_all()
