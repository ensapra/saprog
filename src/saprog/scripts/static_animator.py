import importlib
import sys

implementation = importlib.import_module('saprog_desktop.apps.static_animator')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    implementation.main()
