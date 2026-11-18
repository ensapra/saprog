import importlib
import sys

implementation = importlib.import_module('saprog_desktop.views.temperature_monitor')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    from saprog_desktop.startup import temperature_monitor_main
    raise SystemExit(temperature_monitor_main(entrypoint=__file__))
