"""Discover and load individual desktop tab modules."""

from importlib import import_module
from inspect import signature
from pkgutil import iter_modules


def discover_tabs(ros_manager=None):
    """Load each tab module and return its ``(title, widget)`` pair.

    Each tab package provides ``TAB_TITLE`` and ``create_tab()`` in its
    ``__init__.py``. New tab folders are discovered automatically; ordinary
    modules and underscore-prefixed packages are treated as helpers and skipped.
    """
    discovered = []
    for module_info in sorted(iter_modules(__path__), key=lambda item: item.name):
        if module_info.name.startswith("_") or not module_info.ispkg:
            continue
        module = import_module(f"{__name__}.{module_info.name}")
        title = getattr(module, "TAB_TITLE", None)
        create_tab = getattr(module, "create_tab", None)
        if not isinstance(title, str) or not title.strip():
            raise ValueError(f"Tab module {module.__name__} must define a non-empty TAB_TITLE")
        if not callable(create_tab):
            raise ValueError(f"Tab module {module.__name__} must define create_tab()")
        order = getattr(module, "TAB_ORDER", 100)
        if "ros_manager" in signature(create_tab).parameters:
            widget = create_tab(ros_manager=ros_manager)
        else:
            widget = create_tab()
        discovered.append((order, title, widget))
    ordered = sorted(discovered, key=lambda item: (item[0], item[1].casefold()))
    return [(title, widget) for _order, title, widget in ordered]
