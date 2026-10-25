@echo off
setlocal EnableExtensions
cd /d "%~dp0"

echo ========================================
echo  V2V Demo GUI - All-in-one install
echo ========================================
echo.

REM Prefer the Windows "py" launcher, then python.
set "PY="
where py >nul 2>&1 && set "PY=py -3"
if not defined PY (
  where python >nul 2>&1 && set "PY=python"
)
if not defined PY (
  echo [ERROR] Python 3 not found.
  echo Install Python 3.10+ from https://www.python.org/downloads/
  echo and check "Add python.exe to PATH".
  pause
  exit /b 1
)

echo [1/4] Python:
%PY% --version
if errorlevel 1 (
  echo [ERROR] Could not run Python.
  pause
  exit /b 1
)

echo.
echo [2/4] Creating virtual environment (.venv) ...
if not exist ".venv\Scripts\python.exe" (
  %PY% -m venv .venv
  if errorlevel 1 (
    echo [ERROR] Failed to create .venv
    pause
    exit /b 1
  )
) else (
  echo       .venv already exists - reusing
)

set "VENV_PY=%~dp0.venv\Scripts\python.exe"
set "VENV_PIP=%~dp0.venv\Scripts\pip.exe"

echo.
echo [3/4] Installing packages (SUMO + GUI deps) ...
echo       This may take several minutes on first run.
"%VENV_PY%" -m pip install --upgrade pip
if errorlevel 1 goto :pip_fail

REM CPU torch first on Windows makes SB3 installs more reliable.
"%VENV_PY%" -m pip install torch --index-url https://download.pytorch.org/whl/cpu
"%VENV_PIP%" install -r "%~dp0requirements.txt"
if errorlevel 1 (
  echo.
  echo [WARN] Full install failed once. Retrying requirements only...
  "%VENV_PIP%" install -r "%~dp0requirements.txt"
  if errorlevel 1 goto :pip_fail
)

echo.
echo [4/4] Verifying SUMO binary ...
"%VENV_PY%" -c "from sumo_intersection_env import find_sumo_binary; p=find_sumo_binary('sumo'); print('SUMO OK:', p)"
if errorlevel 1 (
  echo [ERROR] SUMO binary not found after install.
  echo Try: .venv\Scripts\pip install eclipse-sumo
  pause
  exit /b 1
)

echo.
echo ========================================
echo  Install complete.
echo  Run:  run_demo.bat
echo  Or:   .venv\Scripts\python demo_app.py
echo ========================================
echo.
echo Models are NOT included. Add .zip files in the GUI after launch.
pause
exit /b 0

:pip_fail
echo.
echo [ERROR] pip install failed.
echo If torch failed, open https://pytorch.org and install a matching wheel,
echo then re-run install.bat
pause
exit /b 1
