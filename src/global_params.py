import build123d as bd

# 3D printing basics
tol = 0.1 * bd.MM  # Tolerance (tighter than usual)
wall_min = 0.4 * bd.MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * bd.MM  # A small number


# Some common utilities

def bbox_to_box(bb: bd.BoundBox) -> bd.Box:
    return bd.Box(bb.size.X, bb.size.Y, bb.size.Z, mode=bd.Mode.PRIVATE).translate(bb.center())
