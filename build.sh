#!/usr/bin/env bash
set -e
x86_64-w64-mingw32-windres src/resource.rc -O coff -o r64.o
i686-w64-mingw32-windres   src/resource.rc -O coff -o r32.o
# probe: multi-API (D3D9/10/11/12) + psapi (module info) + tlhelp32 (in kernel32, no extra lib)
DLL="-O2 -std=c++17 -shared -static -static-libgcc -static-libstdc++ -ld3d9 -ld3d10 -ld3d11 -ld3d12 -ldxgi -lpsapi -lgdi32 -luser32"
x86_64-w64-mingw32-g++ src/probe.cpp $DLL -o 6DOFProbe.dll
i686-w64-mingw32-g++   src/probe.cpp $DLL -o 6DOFProbe32.dll
GUI="-O2 -std=c++17 -static -static-libgcc -static-libstdc++ -municode -mwindows -ldwmapi -lgdi32"
x86_64-w64-mingw32-g++ src/injectgui.cpp r64.o $GUI -o 6DOFInjectGUI.exe
i686-w64-mingw32-g++   src/injectgui.cpp r32.o $GUI -DPROBE32 -o 6DOFInjectGUI32.exe
x86_64-w64-mingw32-g++ src/inject.cpp -O2 -std=c++17 -static -municode -o 6DOFInject.exe
i686-w64-mingw32-g++   src/inject.cpp -O2 -std=c++17 -static -municode -DPROBE32 -o 6DOFInject32.exe
echo built
