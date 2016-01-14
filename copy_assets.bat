set external_dir=./source/external

echo %external_dir%

mkdir %external_dir%/Box2D
xcopy assets\Box2D-master\Box2D %external_dir%/Box2D /s


