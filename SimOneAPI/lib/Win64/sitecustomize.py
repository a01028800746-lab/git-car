import os
import sys


API_DIRECTORY = os.path.dirname(os.path.abspath(__file__))

if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(API_DIRECTORY)

if API_DIRECTORY not in sys.path:
    sys.path.insert(0, API_DIRECTORY)
