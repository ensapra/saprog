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

The Statistics tab contains standalone IMU and servo temperature widgets.
They expose plain-data update methods and are not connected to ROS yet.

The tabs use a standard Qt tab bar. Drag a tab within the bar to reorder it;
double-click a tab to open it in a separate window. Closing that window returns
the tab to the main window.

## Adding a tab

Add a folder under `tabs/` with an `__init__.py` that defines `TAB_TITLE` and a
`create_tab()` function returning a `QWidget`. Keep the tab's code and assets
inside its folder. Reusable widgets live under `widgets/`, each in its own
folder with an `__init__.py`; keep supporting code and assets in that widget
folder. Tabs import widgets from this shared package so they can be reused by
other tabs. The app discovers and adds tab packages at startup.
Optionally define `TAB_ORDER` to choose a tab's position; unnumbered tabs follow
the built-in ones alphabetically. Ordinary modules and underscore-prefixed
folders are ignored by tab discovery and can hold shared helpers.
