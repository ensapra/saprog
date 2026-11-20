"""Static Animator tab package."""

from widgets.player import StaticAnimatorWidget

TAB_TITLE = "Static Animator"
TAB_ORDER = 30


def create_tab(ros_manager=None):
    """Build the Static Animator snapshot browser and player."""
    return StaticAnimatorWidget(ros_manager=ros_manager)
