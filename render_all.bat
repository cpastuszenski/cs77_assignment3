@echo off

echo "test01: Animated Transformations"
view.exe -i -t 2.5 scenes/test01.json || goto :error

echo "test02: Skinned Mesh"
view.exe -i -t 2.5 scenes/test02.json || goto :error

echo "test03: Particles"
view.exe -i -t 2.5 scenes/test03.json || goto :error

echo "All completed successfully!"
goto :EOF

:error
echo "An error occurred!"
exit /b %errorlevel%
