import importlib
import sys

implementation = importlib.import_module('saprog_desktop.compat.timeline_model')
sys.modules[__name__] = implementation
