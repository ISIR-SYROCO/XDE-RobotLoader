XDE-RobotLoader
===============

Module to create a xde world data structure from a
[urdf][1] description of a robot. See example in `./XDE-RobotLoader/test`

If `prefix` is not defined, install in python USER_BASE directory (`~/.local` by default)

Install:
---------
Install module:

`python setup.py install [--prefix=PREFIX]`

Dev-mode:
----------------
Create a symlink to `./XDE-RobotLoader/src` in `prefix` directory:

`python setup.py develop [--prefix=PREFIX] [--uninstall]`

Build Documentation:
--------------------

`runxde.sh setup.py build_doc [--build-dir=BUILD_DIR] [-b TARGET_BUILD]`


[1]: http://www.ros.org/wiki/urdf
