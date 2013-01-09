#!/usr/bin/env python

####################################
#                                  #
# Import all modules: configure... #
#                                  #
####################################
import loader
import deploy
deploy.loadTypekitsAndPlugins()
import dsimi.interactive
shell = dsimi.interactive.shell() # a special version of IPython

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

#TODO: to delete
import numpy
numpy.set_printoptions(precision=6, suppress=True)


###################
#                 #
# Begin of Script #
#                 #
###################
print "BEGIN OF SCRIPT..."

TIME_STEP = .01


import common
clock, phy, graph = common.createAllAgents(TIME_STEP)





#######################################################################################################
print "START ALL..."
import desc


#k1world = common.createKukaWorld("k1")
#k2world = common.createKukaWorld("k2", [1,1,0])
#genWorld2 = common.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k2g", [.7,0,.4, 0, 0, 0, 1], 0.5)






#groundWorld = common.createGroundWorld()
groundWorld = common.createWorldFromUrdfFile("resources/urdf/ground.xml", "ground", [0,0,-0.4, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")
common.addWorld(groundWorld)



kukaWorld = common.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k1g", [0,0,-0.0, 0.707,0,  0.707, 0], True, 0.5, 0.01)
common.addContactLaws(kukaWorld)
common.addWorld(kukaWorld)
kuka = phy.s.GVM.Robot("k1g")
kuka.lockJoints()

kuka2World = common.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k2g", [0,0,0.2, 0.707, 0, 0.707, 0], True, 0.5, 0.01)
common.addWorld(kuka2World)
kuka2 = phy.s.GVM.Robot("k2g")
kuka2.lockJoints()



#common.addMarkers(genWorld)

#import physicshelper
#k1dm = physicshelper.createDynamicModel(k1world, "k1")
#k2dm = physicshelper.createDynamicModel(k2world, "k2")



#common.delWorld(kukaWorld)
kuka.unlockJoints()
kuka2.unlockJoints()

kuka.enableContactWithBody("groundground", True)
kuka2.enableContactWithBody("groundground", True)
#kuka.enableContactWithRobot("ground", True)
kuka.enableContactWithRobot("k2g", True)

#for b in ["00", "01", "02", "03", "04", "05", "06", "07"]:
#    kuka.enableContactWithBody(b+"k2g", True)

shell()


