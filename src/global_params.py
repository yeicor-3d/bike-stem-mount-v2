from typing import Callable
from build123d import *

# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number


def show_all() -> Callable[[], None]:
    import ocp_vscode
    ocp_vscode.set_defaults(reset_camera=ocp_vscode.Camera.CENTER,
                            measure_tools=True, render_joints=True)
    return ocp_vscode.show_all


def bbox_to_box(bb: BoundBox) -> Box:
    return Box(bb.size.X, bb.size.Y, bb.size.Z, mode=Mode.PRIVATE).translate(bb.center())
