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





#######################################################################################################
print "START ALL..."
import desc


groundWorld = xrl.createWorldFromUrdfFile("resources/urdf/ground.xml", "ground", [0,0,-0.4, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")
xwm.addWorld(groundWorld)


kukaWorld = xrl.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k1g", [0,0,-0.0, 0.707,0,  0.707, 0], True, 0.5, 0.01)
xrl.addContactLaws(kukaWorld)
xwm.addWorld(kukaWorld)
kuka = phy.s.GVM.Robot("k1g")
kuka.lockJoints()


kuka2World = xrl.createWorldFromUrdfFile("resources/urdf/kuka.xml", "k2g", [0,0,0.2, 0.707, 0, 0.707, 0], True, 0.5, 0.01)
xwm.addWorld(kuka2World)
kuka2 = phy.s.GVM.Robot("k2g")
kuka2.lockJoints()



#rx90World = xrl.createWorldFromUrdfFile("resources/urdf/rx90.xml", "rx90", [-0.5,0,0, 1, 0, 0, 0], True, 0.5, 0.01)
#xwm.addWorld(rx90World)
#rx90 = phy.s.GVM.Robot("rx90")
#xwm.addMarkers(rx90World)




#import physicshelper
#k1dm = physicshelper.createDynamicModel(k1world, "k1")
#k2dm = physicshelper.createDynamicModel(k2world, "k2")



#xrl.delWorld(rx90World)

kuka.unlockJoints()
kuka2.unlockJoints()

kuka.enableContactWithBody("groundground", True)
kuka2.enableContactWithBody("groundground", True)
##kuka.enableContactWithRobot("ground", True)
kuka.enableContactWithRobot("k2g", True)

#for b in ["00", "01", "02", "03", "04", "05", "06", "07"]:
#    kuka.enableContactWithBody(b+"k2g", True)



xwm.addInteraction([("groundground", "04k1g"), ("groundground", "05k1g")])
xwm.removeAllInteractions()

shell()


