"""Static Animator tab package."""

from widgets.player import StaticAnimatorWidget

TAB_TITLE = "Static Animator"
TAB_ORDER = 30


def create_tab():
    """Build the Static Animator snapshot browser and player."""
    return StaticAnimatorWidget()
