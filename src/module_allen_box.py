# %%

from build123d import *
from yacv_server import *

# %% ================== MODELLING ==================

with BuildPart(Plane.front) as part:
    Box(1, 2, 3)

# %% ================== EXPORT ==================

if __name__ == "__main__":
    module_allen_box = part
    import logging

    logging.basicConfig(level=logging.DEBUG)
    show_all()
    if os.getenv('CI', '') != '':
        export_all(os.path.join(os.path.dirname(__file__), '..', 'export'))
