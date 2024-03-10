# %%
import os

from build123d import *
from yacv_server import show, show_all, export_all

from src.conn_grid import GridStack, GridScrewHeadHoles, GridScrewThreadHoles
from src.core import grid, grid_dim
from src.global_params import wall

# %% ================== MODELLING ==================

box_width = max(grid_dim[0], 40 * MM)
box_length = max(grid_dim[1], 35 * MM)
box_height = 20 * MM

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
    Box(box_width, box_length, box_height)
    Box(box_width - 2 * wall, box_length - 2 * wall, box_height, mode=Mode.SUBTRACT)
    Box(box_width - 2 * wall, box_length, box_height - 2 * wall, mode=Mode.SUBTRACT)
    Box(box_width, box_length - 2 * wall, box_height - 2 * wall, mode=Mode.SUBTRACT)

    loc = faces().group_by(Axis.Z)[-1].face().center_location
    loc.orientation = Vector(loc.orientation.X + 180, loc.orientation.Y, loc.orientation.Z)
    RigidJoint(label="top_conn_core", joint_location=loc)
    del loc

conn_core.joints["conn_core"].connect_to(box.joints["top_conn_core"])
part = conn_core.part + box.part
del conn_core, box
show(part, names='conn_core')

# %% ================== EXPORT ==================

if __name__ == "__main__":
    module_allen_box = part
    import logging

    logging.basicConfig(level=logging.DEBUG)
    show_all()
    if os.getenv('CI', '') != '':
        export_all(os.path.join(os.path.dirname(__file__), '..', 'export'),
                   export_filter=lambda name, obj: name == 'module_allen_box')
