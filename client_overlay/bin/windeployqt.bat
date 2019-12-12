
cd win64

@REM Release:
@REM OLD windeployqt.exe --dir qtdata --libdir . --plugindir qtdata/plugins --no-system-d3d-compiler --no-opengl-sw --release  --qmldir res/qml OpenVR-InputEmulatorOverlay.exe
@REM windeployqt.exe --dir qtdata --libdir . --plugindir qtdata/plugins --no-system-d3d-compiler --no-compiler-runtime --no-opengl-sw --release  --qmldir res/qml OpenVR-InputEmulatorOverlay.exe

@REM Debug:
windeployqt.exe --dir qtdata --libdir . --plugindir qtdata/plugins --no-system-d3d-compiler --compiler-runtime --no-opengl-sw --debug  --qmldir res/qml OpenVR-InputEmulatorOverlay.exe
