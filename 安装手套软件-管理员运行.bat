@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 正在以管理员权限启动 MotionVenus 安装程序...
powershell -NoProfile -Command "Start-Process -FilePath (Join-Path '%~dp0' 'MotionVenus_HMax_2025.0.6_Setup.exe') -Verb RunAs"
if errorlevel 1 echo 启动失败，请右键本文件选择“以管理员身份运行”。
pause
