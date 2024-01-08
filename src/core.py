# %%
from math import *
from build123d import *
from global_params import *
from screwable_cylinder import ScrewableCylinder

# ================== PARAMETERS ==================

stem_width = stem_height = 35
stem_fillet = 5
stem_length = 30

# ================== MODELLING ==================

# Prepare the stem_wrapper
with BuildPart() as stem_wrapper:
    with BuildSketch(Plane.front):  # as cross_section
        RectangleRounded(stem_width + 2 * wall, stem_height + 2 * wall, wall/2 - eps)
        RectangleRounded(stem_width, stem_height, stem_fillet - wall, mode=Mode.SUBTRACT)
    extrude(amount=stem_length/2, both=True)
    fillet(faces().filter_by(Plane.XY).edges(), wall/2 - eps)
    RigidJoint("right", stem_wrapper, faces().group_by(Axis.X)[-1].face().center_location)
stem_wrapper = stem_wrapper.part

# Prepare the screw hole adapter
screw_hole_base = ScrewableCylinder()
bb = screw_hole_base.bounding_box()
eps_offset_loft = 0.1  # Causes broken geometry if too small
RigidJoint("left", screw_hole_base, Location((bb.min.X - eps_offset_loft, bb.center().Y, bb.center().Z), (0, 90, 0)))

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
    del loft_screw_hole_face, loft_stem_face, screw_hole_base

    # Make it 3D printable by adding top and bottom supports
    for face_side in [-1, 1]:  # Bottom and top
        face_search = 0 if face_side < 0 else -1
        face = faces().group_by(Axis.Z)[face_search].face()
        extreme = stem_wrapper.bounding_box().min if face_side < 0 else stem_wrapper.bounding_box().max
        extreme.Z -= face_side * wall/2  # Ignore fillet
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
        # Fillet some edges of supports
        fillet(faces().group_by(Axis.Z)[face_search].edges() - edges().group_by(Axis.X)[0], wall/2.01)

    # Mirror to the other side
    mirror(about=Plane.YZ)

    # Add the core stem_wrapper
    add(stem_wrapper)
    del stem_wrapper

    # Cut the hole piece in two, to be connected by screws
    bb = core.part.bounding_box()
    Box(bb.size.X, bb.size.Y, 2 * wall, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
core = core.part

# ================== SHOWING/EXPORTING ==================

export = True
try:
    if 'show_object' in locals():
        show_object(core, 'core')  # type: ignore
    else:
        import ocp_vscode
        ocp_vscode.set_defaults(render_joints=True, render_mates=True)
        show_all()()
    export = False
except Exception as ex:
    print("Cannot show model, exporting to STL instead (%s)" % ex)

if export:
    core.export_stl('core.stl')
