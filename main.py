from build123d import *

try: # Optional, for visualizing the model in VSCode instead of CQ-editor or exporting to STL
    import ocp_vscode
    ocp_vscode.set_port(3939)
except ImportError:
    pass


# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number

# Measurements
screw_shank_rad = 7/2 * MM
# screw_rad = 5/2 * MM # M5 screw, unused
washer_outer_rad = 10/2 * MM + tol
washer_depth = 1 * MM + 2 * tol
nut_rad = 8/2 * MM + tol  # Of the inscribed circle for the exterior hexagon
nut_depth = 4 * MM + 2 * tol
button_rad = 30 * MM

# Customization
button_fillet = (2 * wall + washer_depth + nut_depth) / 2 - tol
number_segments = 8
segment_overhang = wall


# ================== MODELLING ==================

# Outer shell of the whole button
obj = Circle(button_rad - segment_overhang)
obj = extrude(obj, wall + washer_depth + nut_depth + wall)

# Remove the shank of the screw
shank = Circle(screw_shank_rad)
shank = extrude(shank, wall)
obj -= shank
del shank

# Remove the internal washer hole
washer = Location((0, 0, wall)) * Circle(washer_outer_rad)
washer = extrude(washer, washer_depth)
obj -= washer
del washer

# Remove the internal nut hole
nut = Location((0, 0, wall + washer_depth)) * RegularPolygon(nut_rad, 6, major_radius=False)
nut = extrude(nut, nut_depth)
obj -= nut
del nut

# Add a nice fillet to the button
bb = obj.bounding_box()
outer_edge_length = obj.edges().group_by(SortBy.LENGTH)[-1][0].length
segment_width = outer_edge_length / number_segments / 2
extra_segments = Circle(bb.max.X + segment_overhang) - Circle(bb.max.X)
extra_segments -= PolarLocations(button_rad - segment_overhang, number_segments) * (Rectangle(bb.max.X * 2, segment_width))
obj_extra_segments = extrude(extra_segments, bb.max.Z)
obj += obj_extra_segments
del extra_segments
del obj_extra_segments
to_fillet = obj.edges().group_by(Axis.Z)
to_fillet = to_fillet[0] + to_fillet[-1]
to_fillet = to_fillet.group_by(SortBy.LENGTH)
to_fillet = to_fillet[0] + to_fillet[1] + to_fillet[2]
obj = fillet(to_fillet, button_fillet)
del to_fillet


# ================== SHOWING/EXPORTING ==================

export = True
try:
    if 'ocp_vscode' in locals():
        import socket
        tmp_socket = socket.socket()
        tmp_socket.connect(('localhost', ocp_vscode.get_port()))
        tmp_socket.close()
        ocp_vscode.reset_show()
        ocp_vscode.show_all()
        export = False # If the connection fails, export to STL instead
    elif 'show_object' in locals():
        show_object(obj, 'main-object-name')
except Exception as ex:
    print("Cannot show model, exporting to STL instead (%s)" % ex)
    
if export:
    obj.export_stl('main-object-name.stl')
