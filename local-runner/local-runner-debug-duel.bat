set PATH=%PATH%;c:\Program Files\Java\jre1.8.0_65\bin\
start "server" "javaw" -jar "local-runner.jar" local-runner-debug-duel.properties
start "current" "..\CodeRacing\Release\cpp-cgdk_vs12.exe" 127.0.0.1 31001 0000000000000000
start "previous" "..\Release\cpp-cgdk_vs12.exe" 127.0.0.1 31002 0000000000000000
