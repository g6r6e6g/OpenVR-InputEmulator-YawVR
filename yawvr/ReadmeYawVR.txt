### Boost
1. Goto https://sourceforge.net/projects/boost/files/boost-binaries/1.63.0/
2. Download Boost 1.71 Binaries (boost_1_71_0-msvc-14.2-64.exe) for Ms Visual Studio v14.2
3. Install Boost into `OpenVR-InputEmulator/third-party/boost_1_63_0`

NOTE: Take care about too long pathname, otherwise install it into C:\tmp and
adjust boost path into all VS projects, All configurations properties,
C/C++/General/Other include directories to use folder boost_1_71_0
 and Linker/General/Additional lib directories to use folder boost_1_71_0\lib64-msvc-14.2

### Qt
1. Goto https://download.qt.io/official_releases/qt/5.9/5.9.1/
2. Download Qt 5.9.1
3. Run the Qt installer (I installed it to "D:\tmp\Qt")
4. Select msvc2015 64-bit, msvc2017 64-bit, remove Android ARMv7
5. Goto `OpenVR-InputEmulator\client_overlay`
6. Create `client_overlay.vcxproj.user` and paste the following into it:

```
<?xml version="1.0" encoding="utf-8"?>
<Project ToolsVersion="14.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <PropertyGroup>
    <QTDIR>D:\tmp\Qt\Qt5.9.1\5.9.1\msvc2015_64</QTDIR>
  </PropertyGroup>
</Project>
```

NOTE: Adjust the path the `msvc2015_64` folder in Qt to match your installation

## OpenVR
1. Goto https://github.com/ValveSoftware/openvr
2. Download OpenVR SDK 1.0.11 archive (latest 1.7.15 replaced IVRControllerComponent by IVRDriverInput)
3. Unzip it into `OpenVR-InputEmulator-YawVR\openvr`

## Visual Studio Community
1. Goto https://visualstudio.microsoft.com/fr/downloads/
2. Download Visual Studio Community 2019 (v14.2)
3. Install it

## Building
1. Open *'VRInputEmulator.sln'* in Visual Studio 2019 (v14.2).
2. Unload the client_leapmotion project, it is no more used
3. Build Solution


## Qt QML files
1. Add Qt bin path into PATH env. var. (eg. D:\tmp\Qt\Qt5.9.1\5.9.1\msvc2015_64\bin)
2. Open a command console
3. with Visual Studio 2019 community (VC16), to set VCVARS, run
    C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat
4. To deploy Qt files for the project, run OpenVR-InputEmulator-YawVR\client_overlay\bin\windeployqt.bat in a console
5. For instance copy dll files, qldata & qml folders from OpenVR-InputEmulator-YawVR\client_overlay\bin\win64
    into previous install C:\Program Files\OpenVR-InputEmulator
=> for instance copy modified .qml files directly into OpenVR-InputEmulator install folder (eg. C:\Program Files\OpenVR-InputEmulator\res\qml)
=> .qml files are automatically (post-build) copied into C:\Program Files\OpenVR-InputEmulator\res\qml when building VS client_overlay project
https://mega.nz/#!cFUmBQrQ!fM0X6iVB6LbCcjLhrclcwD--raSw-PVevYJ1zErb52s

## Installer
1. Install NSIS 3.04
2. Run MakeNSISW
3. Load script OpenVR-InputEmulator-YawVR\installer\installer.nsi
4. Resulting installer file is OpenVR-InputEmulator-YawVR\installer\OpenVR-InputEmulator.exe

TODO
 why yawvr simulator settings offsets are initialized on 2 digits (0.00) ?
