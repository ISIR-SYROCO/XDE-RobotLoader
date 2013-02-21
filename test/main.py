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
wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP, phy_name="physic",  create_graphic=True, graph_name = "graphic")

import xde_resources as xr


#######################################################################################################
print "START ALL..."

groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,-0.4, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")
wm.addWorld(groundWorld)


kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, "kuka", [0,0,-0.0, 0.707,0,  0.707, 0], True, 0.5, 0.01)

wm.addMarkers(kukaWorld)
xrl.addContactLaws(kukaWorld)

wm.addWorld(kukaWorld, True)
kuka = wm.phy.s.GVM.Robot("kuka")


rx90World = xrl.createWorldFromUrdfFile(xr.rx90, "rx90", [-0.5,0,0, 1, 0, 0, 0], True, 0.5, 0.01)
wm.addWorld(rx90World, True)
rx90 = wm.phy.s.GVM.Robot("rx90")
wm.addMarkers(rx90World)

dummyWorld = xrl.createWorldFromUrdfFile("resources/urdf/dummy2.xml", "dummy", [0,0,.5, 1, 0, 0, 0], True, 0.5, 0.01)
wm.addWorld(dummyWorld, True)
dummy = wm.phy.s.GVM.Robot("dummy")
wm.addMarkers(dummyWorld)


dummy.enableContactWithBody("ground.ground", True)



kuka.enableContactWithBody("ground.ground", True)
rx90.enableContactWithBody("ground.ground", True)

wm.addInteraction([("ground.ground", "kuka.04"), ("ground.ground", "kuka.05")])
#wm.removeAllInteractions()

wm.startSimulation()


shell()


