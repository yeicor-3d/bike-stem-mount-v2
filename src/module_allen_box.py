# %%
import os

from build123d import *
from yacv_server import show, show_all, export_all

from src.conn_grid import GridStack, GridScrewHeadHoles, GridScrewThreadHoles
from src.core import grid, grid_dim
from src.global_params import wall, eps, tol

# %% ================== MODELLING ==================

# https://www.amazon.es/dp/B07ZMXQ68T
box_min_width = 35 * MM
box_max_width = 43 * MM
box_min_length = 42 * MM
box_max_length = 72.5 * MM
box_min_height = 2 * MM
box_max_height = 25 * MM
screw_max_diameter = 9 * MM
box_conn_offset = 10 * MM

with BuildPart() as conn_core:
    with BuildPart(mode=Mode.PRIVATE) as grid_conn:
        GridStack(parts=[GridScrewHeadHoles(repeat=grid, total_dimensions=grid_dim, rounded=False),
                         GridScrewThreadHoles(repeat=grid, wrapped_screw_length=8 * MM,
                                              # Minimal screw length
                                              total_dimensions=grid_dim, rounded=False)])
    add(grid_conn)
    del grid_conn
    loc = Location(faces().group_by(Axis.Z)[0].edges().group_by(Axis.Y)[-1].edge().center())
    RigidJoint(label="conn_core", joint_location=loc * Location((0, box_conn_offset, wall)))

with BuildPart() as box:
    # Make the main box with open front
    Box(box_max_width + 2 * wall, box_max_length + 2 * wall, box_max_height + 2 * wall)
    with Locations(Location((0, -wall, 0))):
        Box(box_max_width, box_max_length + wall, box_max_height, mode=Mode.SUBTRACT)

    loc = Location(faces().group_by(Axis.Z)[-1].edges().group_by(Axis.Y)[-1].edge().center())
    RigidJoint(label="top_conn_core", joint_location=loc)

    loc = Location(faces().group_by(Axis.Y)[0].face().center())
    RigidJoint(label="front_lid", joint_location=loc)

    # Make the insides tight
    tmp_edges = edges(Select.LAST).filter_by(Axis.Y)
    chamfer(tmp_edges, box_max_height / 2 - box_min_height)

    # Remove some material from the bottom
    tmp_edges = edges().group_by(Axis.Z)[0].filter_by(Axis.Y)
    chamfer(tmp_edges, box_max_height / 2 - box_min_height)

    # Add rails for the lid
    with Locations(Location((-box_max_width / 2 - wall / 2, -box_max_length / 2 + wall / 2 + wall, 0)),
                   Location((box_max_width / 2 + wall / 2, -box_max_length / 2 + wall / 2 + wall, 0))):
        Box(wall, wall, box_max_height + 2 * wall, mode=Mode.SUBTRACT)

    # Smooth rails for easier printing
    tmp_edges = edges(Select.LAST).filter_by(Axis.Z)
    chamfer([tmp_edges.group_by(Axis.X)[i] for i in [0, -1]], wall - eps)

    del loc

box.joints["top_conn_core"].connect_to(conn_core.joints["conn_core"])
# Avoid screw collisions by extending screw holes vertically
for wire in conn_core.faces().group_by(Axis.Z)[0].face().inner_wires():
    box.part -= extrude(Face(wire), box_max_height * 2, both=True)
del wire

module_allen_box = conn_core.part + box.part

# Ease 3D printing by adding a chamfer to the base of the connector
tmp_edges = module_allen_box.edges().filter_by(Axis.X).group_by(Axis.Y)[-3].group_by(Axis.Z)[0]
module_allen_box = chamfer(tmp_edges, box_conn_offset - eps, conn_core.part.bounding_box().size.Z - wall - eps)
del conn_core, tmp_edges

# Add a lid to the front of the open box
with BuildPart() as lid:
    # Core "box"
    with Locations(Location((0, 0, wall))):
        Box(box_max_width + 4 * wall + 2 * tol, 5 * wall + 2 * tol, box_max_height + 2 * wall + tol)
        with Locations(Location((0, wall, -wall / 2))):
            Box(box_max_width + 2 * wall + 2 * tol, 4.5 * wall + 2 * tol, box_max_height + wall + tol,
                mode=Mode.SUBTRACT)
    loc = Location(faces().group_by(Axis.Y)[0].face().center()) * Location((0, wall + wall / 2, -wall))
    RigidJoint(label="lid_box", joint_location=loc)
    del loc

    # Add rail extrusions
    tmp_edges = edges().filter_by(Axis.Z).group_by(Axis.Y)[-1]
    tmp_edges = [tmp_edges.group_by(Axis.X)[i].edge() for i in [1, -2]]
    tmp_faces = faces().filter_by(Axis.X)
    tmp_faces = [tmp_faces.group_by(Axis.X)[i].face().center_location.orientation for i in [1, -2]]
    off = wall / 2 + wall - tol
    sketch_locs = [Location((0, -off, 0)) * Location(edge.center(), tmp_faces[i]) for i, edge in enumerate(tmp_edges)]
    with BuildSketch(*sketch_locs):
        Rectangle(tmp_edges[0].length, wall - 2 * tol)
    extrude(amount=wall)

    # Smooth rails to engage better
    tmp_edges = edges(Select.LAST).filter_by(Axis.Z)
    chamfer([tmp_edges.group_by(Axis.X)[i] for i in [0, -1]], wall - eps)

    # Clicking mechanism
    click_size = 1 * wall
    with BuildSketch(*[Location((0, 0, - 2.5 * box_min_height - click_size)) * sk_loc for sk_loc in sketch_locs]):
        Rectangle(eps, wall - 2 * tol + eps)
    extrude(amount=wall + click_size)
    del sketch_locs

    # Smooth rails to engage better
    tmp_edges = edges(Select.LAST).filter_by(Axis.Y)
    chamfer([tmp_edges.group_by(Axis.X)[i] for i in [0, -1]], click_size - eps)
    del tmp_edges

    # TODO: Add hole for the allen key part that overlaps with the lid
    # TODO: Work on keeping camera position on scene reloads!

# Connect the lid to the box
box.joints["front_lid"].connect_to(lid.joints["lid_box"])
# lid.part.move(Location((0, -box_max_length / 2 - 2 * wall, 0)))
module_allen_box_lid = lid.part
del lid, box

# Also automatically show the allen 3D scan
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
                   export_filter=lambda name, obj: name.startswith('module_allen_box'))
