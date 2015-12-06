set PATH=%PATH%;c:\Program Files\Java\jre1.8.0_65\bin\
start "server" "javaw" -jar "local-runner.jar" local-runner-debug-quad.properties
start "c1" "..\CodeRacing\Release\cpp-cgdk_vs12.exe" 127.0.0.1 31001 0000000000000000
timeout 1
start "c2" "..\ReleaseNoLogging\cpp-cgdk_vs12.exe" 127.0.0.1 31002 0000000000000000
timeout 1
start "c3" "..\ReleaseNoLogging\cpp-cgdk_vs12.exe" 127.0.0.1 31003 0000000000000000
timeout 1
start "c4" "..\ReleaseNoLogging\cpp-cgdk_vs12.exe" 127.0.0.1 31004 0000000000000000
timeout 1
