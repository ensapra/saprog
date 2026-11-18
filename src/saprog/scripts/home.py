import importlib
import sys

implementation = importlib.import_module('saprog_desktop.apps.home')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    from saprog_desktop.startup import home_main
    raise SystemExit(home_main(entrypoint=__file__))
