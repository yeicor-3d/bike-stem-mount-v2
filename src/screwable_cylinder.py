# %%
import os
from dataclasses import dataclass
from math import *
from typing import Union

import yacv_server as yacv

from global_params import *


# ================== MODELLING ==================


@dataclass(kw_only=True)
class ScrewableCylinder(bd.BasePartObject):
    screw_length: float = 12
    screw_diameter: float = 5  # M5
    screw_head_diameter: float = 8.5  # M5
    screw_head_height: float = 5  # M5
    nut_inscribed_diameter: float = 8  # M5
    nut_height: float = 4  # M5

    wall_size: float = wall
    round: bool = False

    rotation: bd.RotationLike = (0, 0, 0)
    align: Union[bd.Align, tuple[bd.Align, bd.Align, bd.Align]] = None
    mode: bd.Mode = bd.Mode.ADD

    def __post_init__(self):
        with bd.BuildPart() as part:
            total_height = self.screw_length + self.screw_head_height
            max_hole_diameter = max(
                self.screw_diameter + 2 * tol, self.screw_head_diameter + 2 * tol,
                (self.nut_inscribed_diameter + 2 * tol) / cos(radians(360 / 6 / 2)))
            # Core
            bd.Cylinder(max_hole_diameter / 2 + self.wall_size, total_height)
            if self.round:
                bd.fillet(bd.edges(), radius=self.wall_size)
            # Top hole
            with bd.BuildSketch(bd.faces() >> bd.Axis.Z):
                bd.Circle(self.screw_head_diameter / 2 + tol)
            bd.extrude(amount=-self.screw_head_height, mode=bd.Mode.SUBTRACT)
            # Screw hole
            bd.Cylinder(self.screw_diameter / 2 + tol,
                        self.screw_length, mode=bd.Mode.SUBTRACT)
            # Nut hole
            with bd.BuildSketch(bd.faces() << bd.Axis.Z):
                bd.RegularPolygon(self.nut_inscribed_diameter / 2 +
                                  tol, 6, major_radius=False)
            bd.extrude(amount=-self.nut_height, mode=bd.Mode.SUBTRACT)
        super().__init__(part=part.part, rotation=self.rotation,
                         align=self.align, mode=self.mode)


if __name__ == "__main__":
    screwable_cylinder = ScrewableCylinder(rotation=(0, 0, 90))
    yacv.show_all()
    if os.getenv('CI', '') != '':
        yacv.export_all(os.path.join(os.path.dirname(__file__), '..', 'export'))
