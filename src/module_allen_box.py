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
box_height = 23 * MM
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
    RigidJoint(label="conn_core", joint_location=faces().group_by(Axis.Z)[0].face().center_location)

with BuildPart() as box:
    Box(box_max_width + 2 * wall, box_length + 2 * wall, box_height + 2 * wall)
    Box(box_max_width + 2 * wall, box_length, box_height, mode=Mode.SUBTRACT)
    Box(box_min_width, box_length + 2 * wall, box_height, mode=Mode.SUBTRACT)
    Box(box_min_width, box_length, box_height + 2 * wall, mode=Mode.SUBTRACT)

    loc = faces().group_by(Axis.Z)[-1].face().center_location
    loc.orientation = Vector(loc.orientation.X + 180, loc.orientation.Y, loc.orientation.Z)
    RigidJoint(label="top_conn_core", joint_location=loc)
    del loc

conn_core.joints["conn_core"].connect_to(box.joints["top_conn_core"])
module_allen_box = conn_core.part + box.part
show(box.part)
del conn_core, box

# %% ================== EXPORT ==================

if __name__ == "__main__":
    import logging

    logging.basicConfig(level=logging.DEBUG)
    show_all()
    if os.getenv('CI', '') != '':
        export_all(os.path.join(os.path.dirname(__file__), '..', 'export'),
                   export_filter=lambda name, obj: name == 'module_allen_box')
