# %%
from dataclasses import dataclass
from math import *
from typing import Union
from build123d import *
from global_params import *
from screwable_cylinder import ScrewableCylinder


@dataclass(kw_only=True)
class Core(BasePartObject):

    stem_max_width = stem_max_height = 36.5
    stem_fillet = 6
    stem_side_bulge = 1  # Removed as an arc from max_width/height until fillet point
    stem_length = 36.5

    pattern_side_len = 12

    rotation: RotationLike = (0, 0, 0)
    align: Union[Align, tuple[Align, Align, Align]] = None
    mode: Mode = Mode.ADD

    def __post_init__(self):
        # Prepare the stem_wrapper
        with BuildPart() as stem_wrapper:
            with BuildSketch(Plane.front):
                # Outer top/bottom/sides are flat
                Rectangle(self.stem_max_width + 2 * wall, self.stem_max_height + 2 * wall)
                # Remove inner profile
                with BuildLine(Plane.front):
                    p1 = (self.stem_max_width/2, 0)
                    p2 = (self.stem_max_width/2 - self.stem_side_bulge, self.stem_max_height /
                          2-self.stem_side_bulge - self.stem_fillet)
                    p3_base = (self.stem_max_width/2-self.stem_side_bulge - self.stem_fillet,
                               self.stem_max_height/2-self.stem_side_bulge - self.stem_fillet)
                    p3 = (p3_base[0] + self.stem_fillet * cos(radians(45)),
                          p3_base[1] + self.stem_fillet * sin(radians(45)))
                    p4 = (self.stem_max_width/2-self.stem_side_bulge - self.stem_fillet,
                          self.stem_max_height/2 - self.stem_side_bulge)
                    p5 = (0, self.stem_max_height/2)
                    Spline([p1, p2, p3, p4, p5], tangents=[(0, 1), (-1, 0)])
                    mirror(about=Plane.YZ)
                    mirror(about=Plane.XY)
                make_face(mode=Mode.SUBTRACT)
            extrude(amount=self.stem_length/2, both=True)
            RigidJoint("right", stem_wrapper, faces().group_by(Axis.X)[-1].face().center_location)
        stem_wrapper = stem_wrapper.part

        # Prepare the screw hole adapter
        screw_hole_base = ScrewableCylinder()
        bb = screw_hole_base.bounding_box()
        eps_offset_loft = 0.01  # Causes broken geometry if too small
        RigidJoint("left", screw_hole_base, Location(
            (bb.min.X - eps_offset_loft, bb.center().Y, bb.center().Z), (0, 90, 0)))

        with BuildPart() as core:
            # Place the screw_hole base
            stem_wrapper.joints["right"].connect_to(screw_hole_base.joints["left"])
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
            del loft_screw_hole_face, loft_stem_face

            # Make it 3D printable by adding top and bottom supports
            for face_side in [-1, 1]:  # Bottom and top
                face_search = 0 if face_side < 0 else -1
                face = faces().group_by(Axis.Z)[face_search].face()
                extreme = stem_wrapper.bounding_box().min if face_side < 0 else stem_wrapper.bounding_box().max
                extreme.Z += face_side * (screw_hole_base.nut_height + tol - wall /
                                          2)  # Ignore fillet, but add top connectors
                max_extrude = face.center().Z - extreme.Z
                extrude(face, amount=abs(max_extrude))
                assert core.part.is_valid()
                del extreme
                # Prepare a cut plane
                max_offset = face.bounding_box().size.X
                cut_plane_angle = degrees(atan2(max_extrude, max_offset))
                print(cut_plane_angle)
                bb = face.bounding_box()
                cut_plane = Plane(Location((bb.max.X, bb.center().Y, bb.center().Z), (0, -cut_plane_angle, 0)))
                split(bisect_by=cut_plane, keep=Keep.TOP if face_side < 0 else Keep.BOTTOM)
                assert core.part.is_valid()
                assert len(core.part.solids()) == 1
                del face, cut_plane
                # Fillet outer edges of supports
                new_face = faces().group_by(Axis.Z)[face_search].face()
                # fillet(new_face.edges() - new_face.edges().group_by(Axis.X)[0], wall/2.5)  # Finicky
                del new_face

            # Mirror to the other side
            mirror(about=Plane.YZ)

            # Add the core stem_wrapper
            add(stem_wrapper)
            del stem_wrapper

            # Add a top and bottom pattern to insert nuts to connect attachments, keeping it 3D-printable
            nut_height = screw_hole_base.nut_height
            nut_inscribed_diameter = screw_hole_base.nut_inscribed_diameter
            nut_circumscribed_diameter = nut_inscribed_diameter / cos(radians(360/6/2))
            screw_diameter = screw_hole_base.screw_diameter
            del screw_hole_base
            for face_side in [-1, 1]:  # Bottom and top
                face_search = 0 if face_side < 0 else -1
                face: Face = faces().filter_by(
                    GeomType.PLANE).group_by(SortBy.AREA)[-1].group_by(Axis.Z)[face_search].face()
                work_area: BoundBox = face.bounding_box()
                min_pattern_side_len = nut_circumscribed_diameter + tol * 2 + wall * 2
                assert self.pattern_side_len >= min_pattern_side_len, "Pattern side too small (%f < %f)" % (
                    self.pattern_side_len, min_pattern_side_len)
                work_grid = GridLocations(self.pattern_side_len, self.pattern_side_len, int(
                    work_area.size.X // self.pattern_side_len), int(work_area.size.Y // self.pattern_side_len))
                # This is built by layers:
                # Layer -1: the nut (break inner top/bottom surfaces)
                with BuildSketch(face):
                    with work_grid:
                        RegularPolygon(nut_inscribed_diameter/2 + tol, 6, major_radius=False)
                extrude(amount=-self.stem_fillet, mode=Mode.SUBTRACT)
                draw_offset = -wall
                # Layer 1: the nut
                with BuildSketch(Plane(face.center_location).offset(draw_offset)):
                    work_area: BoundBox = face.bounding_box()
                    Rectangle(work_area.size.X, work_area.size.Y)  # Default to filled
                    with work_grid:
                        RegularPolygon(nut_inscribed_diameter/2 + tol, 6, major_radius=False, mode=Mode.SUBTRACT)
                extrude(amount=nut_height + tol)
                draw_offset += nut_height + tol
                # Layer 2: The hole for the screw
                with BuildSketch(Plane(face.center_location).offset(draw_offset)):
                    work_area: BoundBox = face.bounding_box()
                    Rectangle(work_area.size.X, work_area.size.Y)  # Default to filled
                    with work_grid:
                        Circle(screw_diameter/2 + tol, mode=Mode.SUBTRACT)
                extrude(amount=wall)
                del face, work_grid

            # Final filletings
            to_fillet = edges().group_by(Axis.Y)[0] + edges().group_by(Axis.Y)[-1] + \
                edges().group_by(Axis.Z)[-1] + edges().group_by(Axis.Z)[0]
            to_fillet -= to_fillet.filter_by(GeomType.CIRCLE)  # Inner top/bottom circles
            fillet(to_fillet, wall/2.01)
            del to_fillet

            # Cut the hole piece in two, to be connected by screws
            bb = core.part.bounding_box()
            Box(bb.size.X, bb.size.Y, 2 * wall, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        super().__init__(part=core.part, rotation=self.rotation,
                         align=self.align, mode=self.mode)


if __name__ == "__main__":
    part = Core()
    if 'show_object' in globals():  # Needed for CI / cq-editor
        show_object(part)  # type: ignore
    else:
        show_or_export(part)
