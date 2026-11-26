@echo off
setlocal
set "SELF=%~f0"

fltmc >nul 2>&1
if not errorlevel 1 goto elevated

echo ==========================================
echo  LG-603 needs administrator rights.
echo  A UAC prompt will appear - choose Yes.
echo ==========================================
echo.
powershell -NoProfile -Command "Start-Process -FilePath $env:SELF -Verb RunAs -ErrorAction Stop"
if errorlevel 1 (
  echo.
  echo [ERROR] Elevation cancelled.
  echo         Right-click this file and choose "Run as administrator".
  pause
)
exit /b 0

:elevated
cd /d "%~dp0pkg\LG-603"
if not exist "LG-603.exe" (
  echo [ERROR] LG-603.exe not found.
  pause
  exit /b 1
)
echo ==========================================
echo  Starting LG-603 (console mode)
echo  Working dir: %CD%
echo ==========================================
echo.
"LG-603.exe" -console
set RC=%ERRORLEVEL%
echo.
echo Exit code: %RC%
echo Screenshot the lines above if it failed.
pause
