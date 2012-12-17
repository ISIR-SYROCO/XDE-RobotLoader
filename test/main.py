#!/usr/bin/env python

####################################
#                                  #
# Import all modules: configure... #
#                                  #
####################################
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)



###################
#                 #
# Begin of Script #
#                 #
###################
print "BEGIN OF SCRIPT..."

TIME_STEP = .01


print "CREATE WORLD..."
#import scene
#world = scene.buildWorld()


import common
clock, phy, graph = common.createAllAgents(TIME_STEP)

#common.addWorld(world)



#######################################################################################################
print "START ALL..."

phy.s.setPeriod(TIME_STEP)
clock.s.setPeriod(TIME_STEP)

graph.s.start()
phy.s.start()
clock.s.start()


#k1world = common.createKukaWorld("k1")
#k2world = common.createKukaWorld("k2", [1,1,0])
#common.addWorld(k1world)
#common.addWorld(k2world)

#k1robot = phy.s.GVM.Robot("k1")
#k2robot = phy.s.GVM.Robot("k2")
#import physicshelper
#k1dm = physicshelper.createDynamicModel(k1world, "k1")
#k2dm = physicshelper.createDynamicModel(k2world, "k2")

genWorld = common.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k1")
common.addWorld(genWorld)

genWorld2 = common.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k2")
common.addWorld(genWorld2)

#common.delWorld(genWorld)


#k1 = phy.s.GVM.Robot("k1")
#k2 = phy.s.GVM.Robot("k2")
s  = phy.s.GVM.Scene("main")


#common.addWorld(genWorld)

#k1.removeRobot("k1")
#k2.removeRobot("k2")

#import time
#time.sleep(1)

#phy.s.GVM.Robot("k1").removeRobot("k1")

# a special version of IPython
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


