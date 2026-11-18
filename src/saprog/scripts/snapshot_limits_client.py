import importlib
import sys

implementation = importlib.import_module('saprog_desktop.ros.snapshot_limits_client')
sys.modules[__name__] = implementation
