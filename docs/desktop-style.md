# Desktop style

The desktop uses PySide6 Qt widgets. The source for shared appearance is
[desktop_style.py](../desktop/desktop_style.py); apply_style() selects Fusion,
sets the QApplication palette and installs the common stylesheet.

## Shared presentation

| Use | Source constant | Current value |
| --- | --- | --- |
| Main background | BACKGROUND | #141b26 |
| Text | FOREGROUND | #e5edf7 |
| Secondary text | MUTED | #a5b5ca |
| Disabled text | DISABLED | #778394 |
| Input/panel background | PANEL | #1d2939 |
| Button background | BUTTON | #26364b |
| Borders | BORDER | #34465d |
| Focus | FOCUS | #498bd1 |
| Accent | ACCENT_LIGHT | #70b5f5 |
| Plot grid | GRID | #2b394d |

Reuse the shared constants when drawing custom widgets. The same module supplies
ROBOT_* colors for the IMU scene. Servo plot colors live in
[history.py](../desktop/widgets/dynamixel_graph/history.py).

The stylesheet uses square input/button corners, visible focus borders and
highlighted selected tabs. Set objectName to title for the large page heading,
compactAction for small action buttons, and compactStatus for muted compact
status labels. There is no shared application font override; do not document
Tk fonts, ttk themes or Tk-specific helper functions as current behavior.

## Layout and interaction

Use Qt layouts and stretch factors rather than fixed page geometry. Existing
Statistics and Static Animator pages use horizontal QSplitters; long controller
mappings and timeline settings use scroll areas. Keep reusable widgets under
[widgets](../desktop/widgets/) and page composition under
[tabs](../desktop/tabs/).

Use labels, tooltips and accessible names for icon-only controls. Update status
labels for asynchronous results. Keep SDL polling on the GUI thread as in
[ControllerMonitor](../desktop/tabs/controller/devices.py); keep ROS process and
protocol work behind the [manager](../desktop/ros_manager/manager.py), with Qt
signals delivering updates to widgets.

Tab detachment is owned by [tab_workspace.py](../desktop/tab_workspace.py).
Pages should participate in the existing close handling rather than constructing
another application or stopping the shared ROS manager themselves.
