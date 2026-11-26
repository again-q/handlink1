@echo off
chcp 65001 >nul
powershell -NoProfile -Command "try { $r=Invoke-WebRequest 'http://127.0.0.1:8767/toggle' -UseBasicParsing -TimeoutSec 3; $m=($r.Content | ConvertFrom-Json).mode; if($m -eq 'code'){'当前模式：代码模式（手套已隔断，可跑脚本）'}else{'当前模式：数据手套操控'} } catch { '桥接没在跑：先启动 tools\glove_bridge.py' }"
timeout /t 3 >nul
