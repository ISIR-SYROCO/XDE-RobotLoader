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

phy.s.setPeriod(TIME_STEP)
clock.s.setPeriod(TIME_STEP)

graph.s.start()
phy.s.start()
clock.s.start()


k1world = common.createKukaWorld("k1")
#k2world = common.createKukaWorld("k2", [1,1,0])
genWorld = common.createWorldFromUrdfFile("resources/urdf/kuka_without_inertia.xml", "k1g", [.5,0,0, 0, 0, 0, 1], 0.5)

genWorld2 = common.createWorldFromUrdfFile("resources/urdf/kuka2.xml", "k2g", [1,0,0, 0, 0, 0, 1], 0.5)


common.addWorld(k1world)
k1 = phy.s.GVM.Robot("k1")

common.addWorld(genWorld)
k1gen = phy.s.GVM.Robot("k1g")
import numpy as np
initPos = - np.array([[-45, -60, 60, 45, 0,90,90]]).T *np.pi/180.
k1gen.setJointPositions(initPos)

common.addWorld(genWorld2)
k2gen = phy.s.GVM.Robot("k2g")
k2gen.setJointPositions(initPos)


#import physicshelper
#k1dm = physicshelper.createDynamicModel(k1world, "k1")
#k2dm = physicshelper.createDynamicModel(k2world, "k2")




#for b in phy.s.GVM.Scene("main").getBodyNames():
for b in ["00", "01", "02", "03", "04", "05", "06", "07"]:
    print "BODY:", b
    k1rb = phy.s.GVM.RigidBody("k1"+b)
    k1grb = phy.s.GVM.RigidBody("k1g"+b)
#    k1grb = phy.s.GVM.RigidBody("k2g"+b)
    k1Mb = k1rb.getMassMatrix()
    k1gMb = k1grb.getMassMatrix()
    
#    print k1Mb - k1gMb
    
#    H_b_pf = k1rb.getPrincipalInertiaFrame()
#    Mpf = common.transport(Mb, H_b_pf)
#    
##    print b
#    print '<inertia ixx="{0}"  ixy="{1}"  ixz="{2}" iyy="{3}" iyz="{4}" izz="{5}" />'.format(Mpf[0,0], Mpf[0,1], Mpf[0,2], Mpf[1,1], Mpf[1,2], Mpf[2,2])
#    print  k1rb.getPrincipalInertiaFrame().getRotation()
#    print  k1grb.getPrincipalInertiaFrame().getRotation()
#    print Q
    
#    rpy = common.Quaternion2RollPitchYaw(Q)
#    print " {} {} {}".format(*rpy)
#    print rb.getPrincipalMomentsOfInertia()



# a special version of IPython
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


