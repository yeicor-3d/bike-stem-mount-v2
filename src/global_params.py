from build123d import *

# 3D printing basics
tol = 0.2 * MM  # Tolerance
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number


# Some common utilities

def bbox_to_box(bb: BoundBox) -> Box:
    return Box(bb.size.X, bb.size.Y, bb.size.Z, mode=Mode.PRIVATE).translate(bb.center())
