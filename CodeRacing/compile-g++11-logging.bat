echo off
set PATH=C:\Program Files (x86)\mingw-w64\i686-5.1.0-posix-dwarf-rt_v4-rev0\mingw32\bin;%PATH%
set name=MyStrategy

if not exist %name%.cpp (
    echo Unable to find %name%.cpp > compilation.log
    exit 1
)

del /F /Q %name%.exe

SET FILES=

for %%i in (*.cpp) do (
    call concatenate %%i
)

for %%i in (model\*.cpp) do (
    call concatenate %%i
)

for %%i in (csimplesocket\*.cpp) do (
    call concatenate %%i
)

g++.exe -std=c++11 -static -fno-optimize-sibling-calls -fno-strict-aliasing -DONLINE_JUDGE -DWIN32 -DLOGGING -lm -s -x c++ -Wl,--stack=268435456 -O2 -Wall -Wno-unknown-pragmas -o %name%.exe %FILES% -lws2_32 -lwsock32 2>compilation.log
