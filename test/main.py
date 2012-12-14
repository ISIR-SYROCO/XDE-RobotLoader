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
import scene
world = scene.buildWorld()


print "CREATE CLOCK..."
import clockTask
clock = clockTask.createClock()


print "CREATE GRAPHIC..."
import graphic
graph = graphic.createTask()
scene_name = graphic.init()
graphic.deserializeWorld(world)
graph.s.Connectors.IConnectorBody.new("icf", "body_state_H", scene_name)


print "CREATE PHYSIC..."
import physic
phy = physic.createTask()
physic.init(TIME_STEP)
physic.deserializeWorld(world)
phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")


print "CREATE PORTS..."
phy.addCreateInputPort("clock_trigger", "double")
icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("clock_trigger")
clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))


#######################################################################################################
print "START ALL..."

phy.s.setPeriod(TIME_STEP)
clock.s.setPeriod(TIME_STEP)

graph.s.start()
phy.s.start()
clock.s.start()

import common
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

import time
time.sleep(1)

#phy.s.GVM.Robot("k1").removeRobot("k1")

# a special version of IPython
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


