import agents.physic.core
import agents.physic.builder
import physicshelper

phy = None
ms = None
xcd = None
robot = None
dynmodel = None

def createTask():
    phy = agents.physic.core.createAgent("physic", 0)
    setProxy(phy);
    return phy

def setProxy(_phy):
    global phy
    phy = _phy

def init(_timestep):
    global ms, xcd, phy

    ms = agents.physic.core.createGVMScene(phy, "main", time_step=_timestep, uc_relaxation_factor=.1)
    xcd = agents.physic.core.createXCDScene(phy, "xcd", "LMD", lmd_max=0.01)
    ms.setGeometricalScene(xcd)


def deserializeWorld(world):
    global phy
    global robot, dynmodel
    agents.physic.builder.deserializeWorld(phy, ms, xcd, world)
#    robot = phy.s.GVM.Robot("kuka1")
#    dynmodel = physicshelper.createDynamicModel(world, "kuka1")

#    ocb = phy.s.Connectors.OConnectorBodyStateList("ocb")

#    for b in world.scene.rigid_body_bindings:
#        if len(b.graph_node) and len(b.rigid_body):
#            ocb.addBody(str(b.rigid_body))

#    occ = phy.s.Connectors.OConnectorContactBody("occ")
#    occ.addInteraction(phy.s.GVM.Robot("kuka1").getSegmentRigidBody2("03"), "ground")
#    occ.addInteraction(phy.s.GVM.Robot("kuka1").getSegmentRigidBody2("04"), "ground")
#    occ.addInteraction(phy.s.GVM.Robot("kuka1").getSegmentRigidBody2("05"), "ground")
#    occ.addInteraction(phy.s.GVM.Robot("kuka1").getSegmentRigidBody2("06"), "ground")
#    occ.addInteraction(phy.s.GVM.Robot("kuka1").getSegmentRigidBody2("07"), "ground")

#    phy.s.Connectors.IConnectorRobotJointTorque.new("ict", "kuka1_", "kuka1")
#    phy.s.Connectors.OConnectorRobotState.new("ocpos", "kuka1_", "kuka1")

def startTask():
    phy.s.start()

