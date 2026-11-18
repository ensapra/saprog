import importlib
import sys

implementation = importlib.import_module('saprog_desktop.views.snapshot_player')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    from saprog_desktop.startup import snapshot_player_main
    raise SystemExit(snapshot_player_main(entrypoint=__file__))
