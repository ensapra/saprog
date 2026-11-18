import importlib
import sys

implementation = importlib.import_module('saprog_desktop.views.robot_view')
sys.modules[__name__] = implementation
