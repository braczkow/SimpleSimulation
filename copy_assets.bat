set external_dir=source\external
set assets_dir=assets

echo "external dir: " %external_dir%
echo "assets dir: " %assets_dir%

if not exist %external_dir% (
mkdir %external_dir%
) 

mkdir %external_dir%\Box2D
xcopy %assets_dir%\Box2D-master\Box2D %external_dir%\Box2D /s

mkdir %external_dir%\jsoncpp
xcopy %assets_dir%\jsoncpp-dist %external_dir%\jsoncpp /s

mkdir %external_dir%\freeglut
xcopy %assets_dir%\freeglut %external_dir%\freeglut /s
