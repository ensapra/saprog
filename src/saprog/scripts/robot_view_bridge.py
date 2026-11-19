#!/usr/bin/env python3
import importlib
import sys

implementation = importlib.import_module('saprog_desktop.ros.robot_view_bridge')
sys.modules[__name__] = implementation

if __name__ == '__main__':
    implementation.main()
