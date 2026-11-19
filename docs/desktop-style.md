# Desktop UI style guide

This guide records the current Tk desktop presentation so new tabs, windows,
and UI changes fit the project. Read it before changing the desktop UI. The
shared presentation source is [`desktop_style.py`](../src/saprog/scripts/saprog_desktop/desktop_style.py);
keep shared style decisions there and reuse its helpers rather than creating a
second palette or styling system. Shared slider rows are built by
[`motion_controls.py`](../src/saprog/scripts/saprog_desktop/motion_controls.py).

## Shared appearance

`apply_style(root)` selects the `clam` ttk theme when Tk provides it, chooses
Segoe UI when installed and otherwise DejaVu Sans, and sets the default font to
10 points. It sets the root and standard ttk surfaces to a dark blue-gray base
(`BACKGROUND = #141b26`) with pale text (`FOREGROUND = #e5edf7`). The font
fallback is intentionally limited to those two families; do not assume a font
exists on every host.

The shared palette and drawing colors are named constants in `desktop_style.py`.
Views should import those tokens instead of embedding hex values. The robot
orientation renderer has a small set of `ROBOT_*` face/ground colors there;
temperature graph grid lines use `GRID`. The shared palette is:

| Use | Current color |
| --- | --- |
| Main background | `#141b26` |
| Main text | `#e5edf7` |
| Muted text / inactive tab text | `#a5b5ca` |
| Panel / entry surface | `#1d2939` |
| Button / trough surface | `#26364b` |
| Hover / active surface | `#354c68` |
| Border / scrollbar | `#34465d` |
| Focus / selected accent | `#498bd1` |
| Accent button | `#245c96` |
| Fix / torque-on emphasis | `#86e3ad` |
| Release / torque-off emphasis | `#ff9c96` |

Use the existing ttk styles: `Title.TLabel` is 20-point bold, `Muted.TLabel`
uses muted text, label-frame captions are 11-point bold blue, standard buttons
have `(10, 7)` internal padding, and entries/comboboxes have `5` Tk units of
internal padding.
`Accent.TButton` is for a primary action; `Fix.TButton` and `Release.TButton`
communicate the existing fix/release actions. Do not introduce new style names
for one-off color choices when an existing style communicates the same role.

Interactive mappings are part of the appearance: notebook tabs and buttons
distinguish active, selected, pressed, and disabled states; disabled controls
use muted text and dark surfaces. Focused controls use the blue focus/border
color. Checkbuttons distinguish selected indicators. Combobox popup lists and
Text widgets receive dark surfaces, pale text, and blue selection through Tk's
option database. Prefer ttk controls for new forms so these mappings apply.

`configure_changed(widget, **options)` avoids repeated Tk `configure` calls
when a dynamic state has not changed; use it for controls that are frequently
enabled, disabled, or switched between `normal` and `readonly`.
`status_box(parent, variable, ...)` creates a selectable read-only Text status
display. It preserves selection when the text is unchanged and supports Ctrl+A
and Ctrl+C (copying the selection or the full message). Reuse it for longer,
copyable status/error text. Its `height` and `width` options are Tk Text
character/line units, not pixel measurements.

## Layout, resizing, and status

Current layouts use a 14- or 16-unit outer ttk frame inset, left-aligned title,
and modest grouped spacing: commonly 2–12 units between title/help text,
3–5 between repeated rows, and 8–10 between major groups. These are Tk/ttk
layout units, not a fixed pixel grid. Use `pack` or `grid` consistently within
a container, give the main content `fill="both", expand=True`, and let
resizable content take the available space. Keep labels and action rows at
natural size; wrap explanatory copy where it can otherwise push controls off
screen.

Scrollable lists/forms use a `Canvas` containing a ttk content frame, a ttk
vertical scrollbar, a content `<Configure>` binding to update `scrollregion`,
and a canvas `<Configure>` binding to size the embedded frame to the viewport.
Use that pattern for growing collections. Temperature and IMU use canvases as
drawing surfaces and resize their content with the window. Robot uses a native
Tk frame as the embedded RViz viewport. Keep scrollable content accessible at
the minimum window size.

For status, use a `StringVar`-backed ttk label for short, frequently changing
state. Use `status_box` for longer messages where selection/copy is useful.
Current Home and IMU status areas are labels; Creator, Player, and Timeline use
`status_box`. Creator and Player use a Text width of 64 units, and Timeline
sets 4 lines; these are current widget settings, not pixel widths. Preserve
the distinction between labels and copyable status unless the content needs
change.

## Current windows and embedding

| Surface | Current size / minimum | Layout notes |
| --- | --- | --- |
| [Home](../src/saprog/scripts/saprog_desktop/apps/home.py) | `1440x900`, minimum `900x650` | Outer padding 16; Home, Robot, Static Animator, and Other panels in a notebook. Home divides temperature and IMU panes horizontally. Static Animator has Creator / Player and Timeline tabs; Creator and Player start in an equal, horizontally adjustable pane split. |
| [Temperature](../src/saprog/scripts/saprog_desktop/views/temperature_monitor.py) | `1050x700`, minimum `700x450` when standalone | In Home it is an embedded pane and does not create or size a root. Controls and servo selection sit above an expanding graph canvas. |
| [IMU](../src/saprog/scripts/saprog_desktop/views/imu_monitor.py) | `960x850`, minimum `700x650` when standalone | In Home it is embedded and does not call `apply_style` or set root geometry. Contains readings, orientation drawing canvas, and serial log. |
| [Snapshot Creator](../src/saprog/scripts/saprog_desktop/views/snapshot_window.py) | `940x820` when standalone | Embedded joint cards in a scrollable canvas; shared motion sliders and status box. Status width is 64 Text character units to fit the shared Creator / Player pane. |
| [Snapshot Player](../src/saprog/scripts/saprog_desktop/views/snapshot_player.py) | `620x680` when standalone | Scrollable snapshot list between playback controls and status box. |
| [Timeline](../src/saprog/scripts/saprog_desktop/views/timeline_window.py) | `1200x900` when standalone | Sequence/editor panes, lifecycle/actions, and status box; inherits Player behavior and swaps in the timeline editor layout. |
| [Robot](../src/saprog/scripts/saprog_desktop/views/robot_view.py) | Home's resizable Robot notebook panel | ttk controls/status around an expanding native Tk frame used as the embedded RViz viewport; its rendering is external to Tk. |

Home embeds Temperature, IMU, and Robot components in its one root. The
standalone Creator, Player, Timeline, and IMU launchers start separate
processes/windows; the helper classes also accept a `container` or `embedded`
mode so geometry and root styling are only set by the standalone root owner.
Follow that ownership rule when adding an embedded surface: do not set the
parent root's title, geometry, protocol, or global style from a child pane.
Home and standalone Static Animator share the assembly in
[`static_animator.py`](../src/saprog/scripts/saprog_desktop/apps/static_animator.py). The assembly
retains its child views and delegates close decisions to Timeline; Home and the
standalone launcher tear down their root-owned resources only after Timeline
reports a safe close outcome. Unknown action outcomes keep the root alive.
Launcher scripts in [`tabs/`](../tabs/) show which tools run as separate
processes.

## Native Tk and drawing exceptions

The shared helper styles ttk and sets options for Text and combobox popup
lists; it does not impose a universal theme on every classic Tk widget or
external renderer. Existing canvases set their background explicitly where
needed. Timeline's `tk.Listbox` uses native Tk defaults, and its scrollable
transition-editor `Canvas` has no explicit background. Some IMU canvas text is
drawn with `TkDefaultFont` at 9 points or 10-point bold. These are current
inconsistencies, not a separate styling direction. If modifying one of these
surfaces, preserve its behavior while bringing new UI into the shared style
where practical; avoid claiming that `apply_style` styles all Tk widgets.

File dialogs and message boxes use Tk's native `filedialog`/`messagebox`
interfaces. Their appearance follows the platform/Tk implementation and is not
controlled by the shared ttk palette. Keep using those native dialogs for
ordinary file selection and confirmation rather than building custom dialog
windows solely to apply colors.

## Checklist for future UI changes

- Read this guide and inspect the relevant existing window before changing it.
- Reuse `apply_style`, the shared palette, existing ttk style names,
  `configure_changed`, and `status_box` where they fit.
- Follow the existing spacing, expanding-content, minimum-size, and
  scroll-canvas patterns; check the embedded and standalone ownership paths.
- Treat classic Tk drawing/list controls, native dialogs, and RViz as the
  exceptions described above; style new classic Tk widgets explicitly when
  needed instead of assuming ttk rules apply.
- Update this guide when a shared convention or window geometry changes.
- Do not add a parallel palette, UI framework, or broad redesign as part of a
  local tab/window change. Keep source code as the source of truth and describe
  observed behavior rather than promising cross-platform pixel-identical
  rendering.
