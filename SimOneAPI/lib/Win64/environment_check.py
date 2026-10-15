import os
import sys


API_DIRECTORY = os.path.dirname(os.path.abspath(__file__))

if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(API_DIRECTORY)

sys.path.insert(0, API_DIRECTORY)

from SimOneEvaluationAPI import (
    SoInitEvaluationServiceWithLocalData,
    SoSaveEvaluationRecord,
)
from SimOneSensorAPI import SoGetGps
from SimOneServiceAPI import SoGetCaseRunStatus, SoInitSimOneAPI


print(f"Python: {sys.executable}")
print(f"API directory: {API_DIRECTORY}")
print(f"Python version: {sys.version.split()[0]}")
print("SimOne Python API imports: OK")
print("Evaluation API exports: OK")
print("DLL search path: OK")
print("Environment check complete. Start NEVC-Simone before running a control algorithm.")
