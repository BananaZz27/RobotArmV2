@echo off
cd /d "%~dp0src"
"..\.venv\Scripts\python.exe" app.py
if errorlevel 1 pause
