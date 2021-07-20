
set LDVIEW_EXE="D:\Program Files (x86)\LDraw\LDView\LDView.exe"

for %%a in ( "*.dat", "*.ldr" ) do %LDVIEW_EXE% "%%a" "-SaveSnapshot=%%~na.bmp" -PreferenceSet=render_setting -SaveActualSize=0 -SaveImageType=1 -SaveZoomToFit=1 -SaveWidth=1200 -SaveHeight=900 -DefaultZoom=0.95 -FOV=10 -DefaultMatrix=0.387697,-0.90394,-0.1805,-0.507513,-0.0458616,-0.860423,0.769494,0.425191,-0.476544


rem %LDVIEW_EXE% "71472.dat" "-SaveSnapshot=C:\Temp\71472.bmp"       -PreferenceSet=DifferentSettings -SaveActualSize=0 -SaveImageType=2 -SaveZoomToFit=1 -SaveWidth=640 -SaveHeight=480 -DefaultZoom=0.95 -FOV=10 -DefaultMatrix=0.662151,0.052753,0.747509,0.501881,0.709535,-0.494645,-0.556478,0.70269,0.443344

rem %LDVIEW_EXE% "3939p91.dat" "-SaveSnapshot=C:\My Images\3939p91.dat.bmp"       -PreferenceSet=MySettings -SaveActualSize=0 -SaveImageType=2 -SaveZoomToFit=1 -SaveWidth=400 -SaveHeight=400 -DefaultZoom=0.95 -FOV=10 -DefaultMatrix=0.662151,0.052753,0.747509,0.501881,0.709535,-0.494645,-0.556478,0.70269,0.443344