from IsirPythonTools import *

package_name = 'xde_robot_loader'

setup(name='XDE-RobotLoader',
	  version='0.1',
	  description='Robot loader util for xde',
	  author='Soseph',
	  author_email='hak@isir.upmc.fr',
	  package_dir={package_name:'src'},
	  package_data={package_name: ['simple_shapes.dae']},
	  packages=[package_name],
	  cmdclass=cmdclass,

	  script_name=script_name,
	  script_args= script_args
	 )
