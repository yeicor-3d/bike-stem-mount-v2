from build123d import *

# try: # Optional, for visualizing the model in VSCode instead of CQ-editor or exporting to STL
#   from ocp_vscode import show_object, show_all, reset_show, set_port
#   set_port(3939)
# except ImportError:
#   pass


# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1  # Tolerance (tighter than usual)
wall_min = 0.4  # Minimum wall width
wall = wall_min * 3  # Recommended width for most walls of this print
eps = 1e-5  # A small number

# Measurements
screw_length = 1 * CM
screw_rad = 5/2 * MM
screw_shank_rad = 7/2 * MM
washer_outer_rad = 10/2 * MM
washer_depth = 1 * MM
nut_rad = 8/2 * MM  # Of the inscribed circle for the exterior hexagon
nut_depth = 4 * MM
button_rad = 30 * MM
button_fillet = (wall + washer_depth + nut_depth + wall) / 2 - eps # max


# ================== MODELLING ==================

# Outer shell of the whole button
obj = Circle(button_rad)
obj = extrude(obj, wall + washer_depth + nut_depth + wall)

# Remove the internal washer hole
washer = Location((0, 0, wall)) * Circle(washer_outer_rad)
washer = extrude(washer, washer_depth)
obj -= washer
del washer

# Remove the internal nut hole
nut = Location((0, 0, wall + washer_depth)) * RegularPolygon(nut_rad, 6)
nut = extrude(nut, nut_depth)
obj -= nut
del nut

# Add a nice fillet to the button
bb = obj.bounding_box()
outer_edge_length = obj.edges().group_by(SortBy.LENGTH)[-1][0].length
number_segments = 8
segment_width = outer_edge_length / number_segments / 2
segment_overhang = wall
extra_segments = Circle(bb.max.X + segment_overhang) - Circle(bb.max.X - wall)
extra_segments -= PolarLocations(button_rad, number_segments) * (Rectangle(bb.max.X * 2, segment_width))
obj_extra_segments = extrude(extra_segments, bb.max.Z)
obj += obj_extra_segments
del extra_segments
del obj_extra_segments
to_fillet = obj.edges().group_by(Axis.Z)
to_fillet = to_fillet[0] + to_fillet[-1]
obj = fillet(to_fillet, button_fillet)
del to_fillet


# ================== SHOWING/EXPORTING ==================

if 'reset_show' in locals() and 'show_all' in locals():
    reset_show()
    show_all()
elif 'show_object' in locals():
    show_object(obj, 'main-object-name')
else:
    obj.export_stl('main-object-name.stl')
