#!/usr/bin/env python

####################################
#                                  #
# Import all modules: configure... #
#                                  #
####################################
import xde_world_manager as xwm

import dsimi.interactive
shell = dsimi.interactive.shell()



###################
#                 #
# Begin of Script #
#                 #
###################
print "BEGIN OF SCRIPT..."

TIME_STEP = .01


import xde_robot_loader as xrl
clock, phy, graph = xwm.createAllAgents(TIME_STEP)



import xde_resources as xr


#######################################################################################################
print "START ALL..."
import desc


groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,-0.4, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")
xwm.addWorld(groundWorld)


kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, "kuka", [0,0,-0.0, 0.707,0,  0.707, 0], True, 0.5, 0.01)
xwm.addMarkers(kukaWorld)
xrl.addContactLaws(kukaWorld)
xwm.addWorld(kukaWorld, True)
kuka = phy.s.GVM.Robot("kuka")


rx90World = xrl.createWorldFromUrdfFile(xr.rx90, "rx90", [-0.5,0,0, 1, 0, 0, 0], True, 0.5, 0.01)
xwm.addWorld(rx90World, True)
rx90 = phy.s.GVM.Robot("rx90")
xwm.addMarkers(rx90World)


#dummyWorld = xrl.createWorldFromUrdfFile("resources/urdf/dummy.xml", "dummy", [0,0,.2, 1,0,0,0], True, 0.5, 0.01)
#xwm.addMarkers(dummyWorld)
#xrl.addContactLaws(dummyWorld)
#xwm.addWorld(dummyWorld, True)

#dummy2World = xrl.createWorldFromUrdfFile("resources/urdf/dummy2.xml", "dummy2", [0,0,0, 1,0,0,0], True, 0.5, 0.01)
#xwm.addMarkers(dummy2World)
#xrl.addContactLaws(dummy2World)
#xwm.addWorld(dummy2World, True)


#dummy22World = xrl.createWorldFromUrdfFile("resources/urdf/dummy2.xml", "dummy22", [0,0,.5, 1,0,0,0], True, 0.5, 0.01)
#xwm.addMarkers(dummy22World)
#xrl.addContactLaws(dummy22World)
#xwm.addWorld(dummy22World, True)



kuka.enableContactWithBody("ground.ground", True)
rx90.enableContactWithBody("ground.ground", True)
#kuka2.enableContactWithBody("groundground", True)
##kuka.enableContactWithRobot("ground", True)
#kuka.enableContactWithRobot("k2g", True)

#for b in ["00", "01", "02", "03", "04", "05", "06", "07"]:
#    kuka.enableContactWithBody(b+"k2g", True)



xwm.addInteraction([("ground.ground", "kuka.04"), ("ground.ground", "kuka.05")])
#xwm.removeAllInteractions()

xwm.startSimulation()


shell()


