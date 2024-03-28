# %%
import os

from build123d import *
from yacv_server import show, show_all, export_all

from src.conn_grid import GridStack, GridScrewHeadHoles, GridScrewThreadHoles
from src.core import grid, grid_dim
from src.global_params import wall

# %% ================== MODELLING ==================

# https://www.amazon.es/dp/B07ZMXQ68T
box_min_width = 35.5 * MM
box_max_width = 43 * MM
box_min_length = 42 * MM
box_max_length = 70 * MM
box_height = 24.5 * MM
screw_max_diameter = 9 * MM

box_width = max(grid_dim[0], box_max_width)
box_length = max(grid_dim[1], box_max_length)

with BuildPart() as conn_core:
    with BuildPart(mode=Mode.PRIVATE) as grid_conn:
        GridStack(parts=[GridScrewHeadHoles(repeat=grid, total_dimensions=grid_dim, rounded=False),
                         GridScrewThreadHoles(repeat=grid, wrapped_screw_length=8 * MM,
                                              # Minimal screw length
                                              total_dimensions=grid_dim, rounded=False)])
    add(grid_conn)
    del grid_conn
    loc = Location(faces().group_by(Axis.Z)[0].edges().group_by(Axis.Y)[-1].edge().center())
    RigidJoint(label="conn_core", joint_location=loc * Location((0, 10, wall)))

with BuildPart() as box:
    Box(box_max_width + 2 * wall, box_length + 2 * wall, box_height + 2 * wall)
    Box(box_max_width + 2 * wall, box_length - 4 * wall, box_height, mode=Mode.SUBTRACT)
    with Locations(Location((0, -wall, 0))):
        Box(box_min_width, box_length + wall, box_height, mode=Mode.SUBTRACT)

    loc = Location(faces().group_by(Axis.Z)[-1].edges().group_by(Axis.Y)[-1].edge().center())
    RigidJoint(label="top_conn_core", joint_location=loc)

    del loc

box.joints["top_conn_core"].connect_to(conn_core.joints["conn_core"])
# Avoid screw collisions by extending screw holes vertically
all_faces = Compound([Face(wire) for wire in conn_core.faces().group_by(Axis.Z)[0].face().inner_wires()])
# noinspection PyTypeChecker
box.part -= extrude(all_faces.faces(), box_height * 2, both=True)
del all_faces

module_allen_box = conn_core.part + box.part
del conn_core, box

base_path = os.path.join(os.path.dirname(__file__), '..') if "__file__" in locals() else os.getcwd()
with open(os.path.join(base_path, 'assets', 'allen-lowpoly.glb'), 'rb') as f:
    allen_glb = f.read()

show_all()
show(allen_glb, names="allen_3d_scan", auto_clear=False)

# %% ================== EXPORT ==================

if __name__ == "__main__":
    import logging

    logging.basicConfig(level=logging.DEBUG)
    if os.getenv('CI', '') != '':
        export_all(os.path.join(os.path.dirname(__file__), '..', 'export'),
                   export_filter=lambda name, obj: name == 'module_allen_box')
