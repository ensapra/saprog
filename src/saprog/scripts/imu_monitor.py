import importlib
import sys

implementation = importlib.import_module('saprog_desktop.views.imu_monitor')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    from saprog_desktop.startup import imu_monitor_main
    raise SystemExit(imu_monitor_main(entrypoint=__file__))
