# %%
import os
from math import cos, radians, sin, degrees, atan2

from build123d import *
from yacv_server import show_all, export_all

from src.conn_grid import GridBase, GridStack, GridScrewThreadHoles, Grid2D, Grid2DF, GridNutHoles
from src.global_params import wall, bbox_to_box, tol
from src.screwable_cylinder import ScrewableCylinder

stem_max_width = 38
stem_max_height = 38
stem_fillet = 10
stem_side_bulge = 1  # Removed as an arc from max_width/height until fillet point
stem_length = 30

pattern_side_len = GridBase.dimensions
grid = Grid2D(int((stem_max_width + 2 * wall) // pattern_side_len.x),
              int(stem_length // pattern_side_len.y))
grid_dim = Grid2DF(stem_max_width + 2 * wall, stem_length)


def build_core():
    # Prepare the stem_wrapper
    with BuildPart() as stem_wrapper:
        with BuildSketch(Plane.front):
            # Outer top/bottom/sides are flat
            Rectangle(stem_max_width + 2 * wall, stem_max_height + 2 * wall)
            # Remove inner profile
            with BuildLine(Plane.front):
                p1 = (stem_max_width / 2, 0)
                p2 = (stem_max_width / 2 - stem_side_bulge, stem_max_height /
                      2 - stem_side_bulge - stem_fillet)
                p3_base = (stem_max_width / 2 - stem_side_bulge - stem_fillet,
                           stem_max_height / 2 - stem_side_bulge - stem_fillet)
                p3 = (p3_base[0] + stem_fillet * cos(radians(45)),
                      p3_base[1] + stem_fillet * sin(radians(45)))
                p4 = (stem_max_width / 2 - stem_side_bulge - stem_fillet,
                      stem_max_height / 2 - stem_side_bulge)
                p5 = (0, stem_max_height / 2)
                Spline([p1, p2, p3, p4, p5], tangents=[(0, 1), (-1, 0)])
                mirror(about=Plane.YZ)
                mirror(about=Plane.XY)
            make_face(mode=Mode.SUBTRACT)
        extrude(amount=stem_length / 2, both=True)
        RigidJoint("right", stem_wrapper, faces().group_by(Axis.X)[-1].face().center_location)
    stem_wrapper = stem_wrapper.part

    # Prepare the screw hole adapter
    screw_hole_base = ScrewableCylinder()
    bb = screw_hole_base.bounding_box()
    eps_offset_loft = 0.01  # Causes broken geometry if too small
    RigidJoint("left", screw_hole_base, Location(
        (bb.min.X - eps_offset_loft, bb.center().Y, bb.center().Z), (0, 90, 0)))
    stem_wrapper.joints["right"].connect_to(screw_hole_base.joints["left"])

    with BuildPart() as core:
        # Add the placed screw_hole base
        add(screw_hole_base)

        # Attach the screw hole adapter to the stem_wrapper
        loft_bb: BoundBox = core.part.bounding_box()
        loft_bb.min.X -= loft_bb.size.X / 2
        loft_bb.min.X -= loft_bb.size.X / 2
        loft_screw_hole_face = screw_hole_base.faces().group_by(SortBy.AREA)[-1].face() & bbox_to_box(loft_bb)
        loft_stem_face = stem_wrapper.faces().group_by(Axis.X)[-1].face() & bbox_to_box(loft_bb)
        add(loft([loft_stem_face, loft_screw_hole_face]))
        assert core.part.is_valid()
        assert len(core.part.solids()) == 1
        core.part = core.part.clean()

        # Make it 3D printable by adding top and bottom supports
        for face_side in [-1, 1]:  # Bottom and top
            face_search = 0 if face_side < 0 else -1
            bottom_face = faces().group_by(Axis.Z)[face_search].face()
            extreme = stem_wrapper.bounding_box().min if face_side < 0 else stem_wrapper.bounding_box().max
            extreme.Z += face_side * (screw_hole_base.nut_height + 2 * tol)
            max_extrude = bottom_face.center().Z - extreme.Z
            extrude(bottom_face, amount=abs(max_extrude))
            assert core.part.is_valid()
            # Prepare a cut plane
            max_offset = bottom_face.bounding_box().size.X
            cut_plane_angle = degrees(atan2(max_extrude, max_offset))
            print(cut_plane_angle)
            bb = bottom_face.bounding_box()
            cut_plane = Plane(Location((bb.max.X, bb.center().Y, bb.center().Z), (0, -cut_plane_angle, 0)))
            split(bisect_by=cut_plane, keep=Keep.TOP if face_side < 0 else Keep.BOTTOM)
            assert core.part.is_valid()
            assert len(core.part.solids()) == 1
            # Fillet outer edges of supports
            new_face = faces().group_by(Axis.Z)[face_search].face()
            fillet(new_face.edges() - new_face.edges().group_by(Axis.X)[0], wall / 2.5)  # Finicky

        # Mirror to the other side
        mirror(about=Plane.YZ)

        # Add the core stem_wrapper
        add(stem_wrapper)

        # Add a top and bottom pattern to insert nuts to connect attachments, keeping it 3D-printable
        for face_side in [-1, 1]:  # Bottom and top
            face_search = 0 if face_side < 0 else -1
            bottom_face: Face = faces().filter_by(
                GeomType.PLANE).group_by(SortBy.AREA)[-1].group_by(Axis.Z)[face_search].face()
            nut_holes = GridNutHoles(repeat=grid, total_dimensions=grid_dim, rounded=False)
            with BuildPart(mode=Mode.PRIVATE) as grid_conn:
                GridStack(parts=[nut_holes,
                                 GridScrewThreadHoles(repeat=grid, wrapped_screw_length=wall,
                                                      # Minimal screw length
                                                      total_dimensions=grid_dim, rounded=False),
                                 ])
            place_at = Location((0, 0, bottom_face.center().Z), (0, 180 if face_side == -1 else 0, 0))
            grid_conn_placed = grid_conn.part.moved(place_at)
            add(grid_conn_placed)

            # Break inner top/bottom surfaces
            bottom_face = grid_conn.faces().group_by(Axis.Z)[0].face()
            bottom_face_inner_edges = bottom_face.edges() - bottom_face.outer_wire().edges()
            for wire in Wire.combine(bottom_face_inner_edges):
                extrude(Face(wire).move(place_at), amount=-stem_fillet, mode=Mode.SUBTRACT)

        # Final fillets
        to_fillet = edges().group_by(Axis.Y)[0] + edges().group_by(Axis.Y)[-1] + \
                    edges().group_by(Axis.Z)[-1] + edges().group_by(Axis.Z)[0]
        to_fillet -= to_fillet.filter_by(GeomType.CIRCLE)  # Inner top/bottom circles
        to_fillet -= to_fillet.filter_by(GeomType.BSPLINE)  # Inner section of stem
        fillet(to_fillet, wall / 1.01)

        # Cut the hole piece in two, to be connected by screws
        bb = core.part.bounding_box()
        Box(bb.size.X, bb.size.Y, 2 * wall, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    return core


if __name__ == "__main__":
    core = build_core()
    import logging

    logging.basicConfig(level=logging.DEBUG)
    show_all()
    if os.getenv('CI', '') != '':
        export_all(os.path.join(os.path.dirname(__file__), '..', 'export'))
