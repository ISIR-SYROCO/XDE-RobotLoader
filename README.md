XDE-RobotLoader
===============

Creation, destruction, serialization of robot in XDE from
[urdf][1] descriptions. See example in `./XDE-RobotLoader/test`

If `prefix` is not defined, install in python USER_BASE directory (`~/.local` by default)

Install:
---------
Install module:

`python setup.py install [--prefix=PREFIX]`

Dev-mode:
----------------
Create a symlink to `./XDE-RobotLoader/src` in `prefix` directory:

`python setup.py develop [--prefix=PREFIX] [--uninstall]`


[1]: http://www.ros.org/wiki/urdf
