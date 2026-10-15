from pathlib import Path
import subprocess
import sys


API_DIRECTORY = Path(__file__).resolve().parent
PID_FILE = API_DIRECTORY / ".simone_python_algorithm.pid"
algorithm = API_DIRECTORY / (sys.argv[1] if len(sys.argv) > 1 else "TestDrive.py")

process = subprocess.Popen([sys.executable, str(algorithm)], cwd=str(API_DIRECTORY))
PID_FILE.write_text(str(process.pid), encoding="ascii")

try:
    raise SystemExit(process.wait())
finally:
    if PID_FILE.exists() and PID_FILE.read_text(encoding="ascii").strip() == str(process.pid):
        PID_FILE.unlink()
