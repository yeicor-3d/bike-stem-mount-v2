# %%
from dataclasses import dataclass
from typing import Union
from build123d import *
from math import *
from global_params import *
from core import *

# ================== MODELLING ==================


@dataclass(kw_only=True)
class ModuleAllenBox(BasePartObject):
    """A module to store these allen keys: https://www.amazon.es/dp/B07ZMXQ68T/"""
    width = 40
    height = 20
    depth = 70

    round: bool = False
    rotation: RotationLike = (0, 0, 0)
    align: Union[Align, tuple[Align, Align, Align]] = None
    mode: Mode = Mode.ADD

    def __post_init__(self):
        with BuildPart(Plane.front) as part:
            Box(self.width+2*wall, self.height+2*wall, self.depth)
            Box(self.width, self.height, self.depth, mode=Mode.SUBTRACT)
        super().__init__(part=part.part, rotation=self.rotation,
                         align=self.align, mode=self.mode)


if __name__ == "__main__":
    show_or_export(ModuleAllenBox())
