## How to run headless simulation with graphics
1. Use WSL2 because it has EGL Renderer Plugin which uses EGLOpenGLWindow.cpp which is different from Windows Win32OpenGLWindow.cpp.
2. Run run.sh so it uses DX11 rendering using the environmental variables `export DISPLAY="${DISPLAY:-:0}" export EGL_PLATFORM=x11`