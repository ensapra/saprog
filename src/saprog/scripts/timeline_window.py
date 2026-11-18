import importlib
import sys

implementation = importlib.import_module('saprog_desktop.views.timeline_window')
sys.modules[__name__] = implementation
