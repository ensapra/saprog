import importlib
import sys

implementation = importlib.import_module('saprog_desktop.views.snapshot_window')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    from saprog_desktop.startup import snapshot_creator_main
    raise SystemExit(snapshot_creator_main(entrypoint=__file__))
