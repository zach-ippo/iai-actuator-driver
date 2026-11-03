@echo off
rem Starts the iai-gui web GUI from this folder. Double-click it, or run it from a command prompt.
rem The first run creates .venv here and installs the package into it (needs internet for pyserial).
rem Arguments go to iai-gui, for example:  run-gui.bat --sim-only   (demo: simulators only, no serial port)
setlocal
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto have_venv

set "PY="
py -3 --version >nul 2>&1
if not errorlevel 1 set "PY=py -3"
if defined PY goto have_python
python --version >nul 2>&1
if not errorlevel 1 set "PY=python"
if defined PY goto have_python
echo Python was not found. Install Python 3.11 or newer from https://www.python.org/downloads/
echo (tick "Add python.exe to PATH"), then start run-gui.bat again.
goto fail

:have_python
%PY% -c "import sys; raise SystemExit(sys.version_info < (3, 11))"
if errorlevel 1 (
    echo This needs Python 3.11 or newer. Found:
    %PY% --version
    echo Install a newer Python from https://www.python.org/downloads/ and start run-gui.bat again.
    goto fail
)
echo Creating .venv with:
%PY% --version
%PY% -m venv .venv
if errorlevel 1 (
    echo Could not create .venv. Remove the .venv folder if one was left behind and start run-gui.bat again.
    goto fail
)

:have_venv
".venv\Scripts\python.exe" -c "import sys; raise SystemExit(sys.version_info < (3, 11))"
if errorlevel 1 (
    echo The .venv folder uses a Python older than 3.11 or does not work.
    echo Remove or rename the .venv folder and start run-gui.bat again.
    goto fail
)
if exist ".venv\Scripts\iai-gui.exe" goto start
echo Installing the package into .venv (first run; needs internet for pyserial and setuptools) ...
".venv\Scripts\python.exe" -m pip install --disable-pip-version-check -e ..\..\bindings\python -e .
if errorlevel 1 (
    echo The install failed: see the messages above. Check the internet connection or proxy and start
    echo run-gui.bat again. If it keeps failing, remove the .venv folder and try once more.
    goto fail
)

:start
".venv\Scripts\iai-gui.exe" %*
if errorlevel 1 (
    echo iai-gui stopped with an error: see the messages above.
    goto fail
)
endlocal
exit /b 0

:fail
echo.
pause
endlocal
exit /b 1
