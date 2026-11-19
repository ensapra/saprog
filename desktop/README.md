# Saprog Desktop

This is the ROS-independent Qt for Python desktop application. Install its
dependency with:

```sh
python -m pip install -r desktop/requirements.txt
```

Then launch it from the repository root with:

```sh
python desktop
```

The Home window is intentionally empty while existing panels are migrated out
of the ROS package.

## Adding a tab

Add a folder under `tabs/` with an `__init__.py` that defines `TAB_TITLE` and a
`create_tab()` function returning a `QWidget`. Keep that tab's supporting files
in its folder. The app discovers and adds tab packages at startup. Optionally
define `TAB_ORDER` to choose its position; unnumbered tabs follow the built-in
ones alphabetically. Ordinary modules and underscore-prefixed folders are
ignored and can hold shared helpers.
