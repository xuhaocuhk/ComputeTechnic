
set LDVIEW_EXE="D:\Program Files (x86)\LDraw\LDView\LDView.exe"

for %%a in ( "*.dat", "*.ldr" ) do %LDVIEW_EXE% "%%a" "-SaveSnapshot=%%~na.png" -PreferenceSet=render_setting -SaveActualSize=0 -SaveImageType=1 -SaveZoomToFit=1 -SaveWidth=800 -SaveHeight=600 -FOV=20 -DefaultMatrix=0.707107,0,0.707107,0.353553,0.866025,-0.353553,-0.612372,0.5,0.612372 -DefaultZoom=0.90


rem %LDVIEW_EXE% "71472.dat" "-SaveSnapshot=C:\Temp\71472.bmp"       -PreferenceSet=DifferentSettings -SaveActualSize=0 -SaveImageType=2 -SaveZoomToFit=1 -SaveWidth=640 -SaveHeight=480 -DefaultZoom=0.95 -FOV=10 -DefaultMatrix=0.662151,0.052753,0.747509,0.501881,0.709535,-0.494645,-0.556478,0.70269,0.443344

rem %LDVIEW_EXE% "3939p91.dat" "-SaveSnapshot=C:\My Images\3939p91.dat.bmp"       -PreferenceSet=MySettings -SaveActualSize=0 -SaveImageType=2 -SaveZoomToFit=1 -SaveWidth=400 -SaveHeight=400 -DefaultZoom=0.95 -FOV=10 -DefaultMatrix=0.662151,0.052753,0.747509,0.501881,0.709535,-0.494645,-0.556478,0.70269,0.443344