import inspect
from typing import Callable
from build123d import *

# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number


# Some common utilities

def show_all() -> Callable[[], None]:
    import ocp_vscode
    ocp_vscode.set_defaults(reset_camera=ocp_vscode.Camera.CENTER,
                            measure_tools=True, render_joints=True)
    return ocp_vscode.show_all


def caller_file() -> str:
    stack = inspect.stack()
    while 'global' in stack[0].filename:
        print(stack[0].filename)
        stack = stack[1:]
    print(stack[0].filename)
    return stack[0].filename


def export(part: Part) -> None:
    if 'show_object' in globals():  # Needed for CI / cq-editor
        show_object(part)  # type: ignore
    else:
        file_of_caller = caller_file()
        print("Exporting to STL using ref %s" % file_of_caller)
        for i, solid in enumerate(part.solids()):
            solid.export_stl(file_of_caller[:-3] + (f'_{i}.stl' if len(part.solids()) > 1 else '.stl'))


def show_or_export(part: Part) -> None:
    try:
        from ocp_vscode import show
        show(part, render_joints=True)
    except Exception as e:
        print("Error showing part (%s), exporting to STL instead" % e)
        export(part)


def bbox_to_box(bb: BoundBox) -> Box:
    return Box(bb.size.X, bb.size.Y, bb.size.Z, mode=Mode.PRIVATE).translate(bb.center())
