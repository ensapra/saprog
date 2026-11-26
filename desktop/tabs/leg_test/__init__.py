"""Single-leg URDF calibration, Cartesian joystick control and motion previews."""

from .urdf_tab import SingleLegTestTab

TAB_TITLE = "Single Leg Test"
TAB_ORDER = 55


def create_tab(ros_manager=None):
    return SingleLegTestTab(ros_manager)
