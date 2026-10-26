@echo off
cd /d "%~dp0"
if not exist .venv (
    echo Creating virtual environment and installing packages...
    py -3 -m venv .venv || python -m venv .venv
    call .venv\Scripts\activate.bat
    python -m pip install --upgrade pip
    pip install -r requirements.txt
) else (
    call .venv\Scripts\activate.bat
)
python xbee_teleop_windows.py
pause
