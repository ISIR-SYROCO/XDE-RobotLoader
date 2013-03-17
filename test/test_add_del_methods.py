#!/usr/bin/env python

####################################
#                                  #
# Import all modules: configure... #
#                                  #
####################################
import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr

import dsimi.interactive
shell = dsimi.interactive.shell()



###################
#                 #
# Begin of Script #
#                 #
###################
print "BEGIN OF SCRIPT..."

TIME_STEP = .01

wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP, phy_name="physic",  create_graphic=True, graph_name = "graphic")




#######################################################################################################
print "START ALL..."

groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,-0.4, 1, 0, 0, 0], True, 0.1, 0.05)
wm.addWorld(groundWorld)


kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, "kuka", [0,0,-0.0, 0.707,0,  0.707, 0], True, 0.5, 0.01)
wm.addMarkers(kukaWorld)
wm.addWorld(kukaWorld)
kuka = wm.phy.s.GVM.Robot("kuka")

wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.0)

kuka.enableContactWithBody("ground.ground", True)

#for interactions:
wm.addInteraction([("kuka.04", "ground.ground"), ("kuka.05", "ground.ground")])
wm.removeInteraction([("kuka.04", "ground.ground"), ("kuka.05", "ground.ground")])
wm.addInteraction([("kuka.04", "ground.ground"), ("kuka.05", "ground.ground")])
wm.removeAllInteractions()
wm.removeAllInteractions() # test if it doesn't crash if one tryieds to remove in void scene
wm.addInteraction([("kuka.04", "ground.ground"), ("kuka.05", "ground.ground")])
wm.removeInteraction([("kuka.06", "ground.ground")])   #no problem if removing non-added interaction


#for markers:
wm.addMarkers(kukaWorld)
wm.removeMarkers(kukaWorld)
wm.addMarkers(kukaWorld, ["kuka.03", "kuka.04", "kuka.05"], False)
wm.removeMarkers(kukaWorld, ["kuka.05"])
wm.removeMarkers(kukaWorld, ["kuka.05"]) #just a warning if marker does not exist


#wm.removeWorld(kukaWorld) #WARNING: PROBLEM IF DELETE WORLD WITH INTERACTIONS OR MARKERS DISPLAYED
#wm.addWorld(kukaWorld)

wm.startAgents()

shell()



