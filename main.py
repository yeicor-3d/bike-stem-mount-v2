from build123d import *


# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.2  # Tolerance
wall_min = 0.4  # Minimum wall width
wall = wall_min * 3  # Recommended width for most walls of this print
eps = 1e-5  # A small number

# Measurements
screw_length = 1 * CM
screw_diameter = 5 * MM
nut_diameter = 8 * MM  # Of the inscribed circle for the exterior hexagon
nut_height = 4 * MM
button_diameter = 30 * MM
button_fillet = 5 * MM


# ================== MODELLING ==================

# Build the button by layers from front-facing (on the XY plane) to back-facing (up the Z axis)
with BuildPart() as obj:
    # The hole for the nut
    with BuildSketch():
        Circle(radius=button_diameter / 2)
        RegularPolygon(nut_diameter / 2 + tol, major_radius=False, side_count=6, mode=Mode.SUBTRACT)
    extrude(amount=nut_height)

    # The inner wall in between the nut and the screw shank
    with BuildSketch(Plane.XY.offset(nut_height)):
        Circle(radius=button_diameter / 2)
        Circle(screw_diameter / 2 + tol, mode=Mode.SUBTRACT)
    extrude(amount=screw_length - nut_height)

    # The fillet, only on the bottom face

    fillet(obj.edges().group_by(Axis.Z)[0].sort_by(SortBy.LENGTH)[-1], button_fillet)


# ================== SHOWING/EXPORTING ==================

if 'show_object' in locals():
    show_object(obj, '<main-object-name>')
else:
    obj.part.export_stl('<main-object-name>.stl')
