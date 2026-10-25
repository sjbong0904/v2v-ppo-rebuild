@echo off
setlocal EnableExtensions
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo [ERROR] .venv not found.
  echo Run install.bat first (one-time setup).
  pause
  exit /b 1
)

".venv\Scripts\python.exe" demo_app.py
if errorlevel 1 (
  echo.
  echo [ERROR] Demo exited with an error.
  pause
  exit /b 1
)
