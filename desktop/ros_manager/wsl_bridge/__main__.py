"""Support launching the package directory directly from WSL."""

if __package__:
    from .runtime import main
else:
    # Windows launches `python3 -u /path/to/wsl_bridge --workspace ...`.
    # Import this package without importing ros_manager, which depends on Qt.
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from wsl_bridge.runtime import main


if __name__ == "__main__":
    main()
