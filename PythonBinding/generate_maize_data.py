import os
import subprocess
import sys
from pathlib import Path


# If you moved this script, update path resolution to the EvoEngine root.
file_path = Path(__file__).resolve()
evoengine_directory = file_path.parent.parent

# Preferred build output location.
preferred_exe = evoengine_directory / "out" / "build" / "x64-Release" / "EvoEngine_App" / "MaizeDataGeneratorApp.exe"

# Fallback search for differently named build folders.
fallback_candidates = sorted((evoengine_directory / "out" / "build").glob("**/EvoEngine_App/MaizeDataGeneratorApp.exe"))

if preferred_exe.exists():
    maize_app_exe = preferred_exe
elif fallback_candidates:
    maize_app_exe = fallback_candidates[0]
else:
    print("Could not locate MaizeDataGeneratorApp.exe. Build EvoEngine_App first.")
    sys.exit(1)

print(f"Running: {maize_app_exe}")
completed = subprocess.run([str(maize_app_exe)], cwd=str(maize_app_exe.parent))
if completed.returncode != 0:
    print(f"MaizeDataGeneratorApp exited with code {completed.returncode}")
    sys.exit(completed.returncode)
