
import desc.robot
import desc.simple.physic
import desc.simple.scene
import desc.simple.graphic
import desc.simple.collision
import desc.scene
import desc.material
import desc.graphic
import desc.core
import lgsm
import numpy as np
import math


import time

import clockTask
import physic
import graphic








def createAllAgents(TIME_STEP):
    print "CREATE CLOCK..."
    import clockTask
    clock = clockTask.createClock()


    print "CREATE GRAPHIC..."
    import graphic
    graph = graphic.createTask()
    scene_name = graphic.init()
    graph.s.Connectors.IConnectorBody.new("icf", "body_state_H", scene_name)


    print "CREATE PHYSIC..."
    import physic
    phy = physic.createTask()
    physic.init(TIME_STEP)
    phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")


    print "CREATE PORTS..."
    phy.addCreateInputPort("clock_trigger", "double")
    icps = phy.s.Connectors.IConnectorSynchro.new("icps")
    icps.addEvent("clock_trigger")
    clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

    graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))


    return clock, phy, graph





def delWorld(old_world):
    
    #delete graphical scene
    def deleteNodeInGraphicalTree(node):
        nname = str(node.name)
        print 'treating', nname
        if graphic.graph_scn.SceneInterface.nodeExists(nname):
            graphic.graph_scn.SceneInterface.removeNode(nname)
    desc.core.visitDepthFirst(deleteNodeInGraphicalTree, old_world.scene.graphical_scene.root_node)



    #delete physical scene
    for mechanism in old_world.scene.physical_scene.mechanisms:
        mname = str(mechanism.name)
        physic.phy.s.deleteComponent(mname)
    
    #scene = physic.phy.s.GVM.Scene("main")         #TODO: main ne devrait pas etre scpecifie
    scene = physic.ms
    def removeRigidBodyChildren(node):
        print node.rigid_body.name
        for child in node.children:
            removeRigidBodyChildren(child)
        
        rbname = str(node.rigid_body.name)
        scene.removeRigidBody(rbname)
        physic.phy.s.deleteComponent(rbname)
        physic.phy.s.deleteComponent(str(node.inner_joint.name))

    for node in old_world.scene.physical_scene.nodes:
        removeRigidBodyChildren(node)

#    physic.phy.s.deleteComponent()


def addWorld(new_world):
    """
    """
    phy, graph = physic.phy, graphic.graph
    
#    graph.s.stop()
    phy.s.stop()
    old_T = phy.s.getPeriod()
    phy.s.setPeriod(0)

    physic.deserializeWorld(new_world)
    graphic.deserializeWorld(new_world)


    ocb = phy.s.Connectors.OConnectorBodyStateList("ocb")
    for b in new_world.scene.rigid_body_bindings:
        if len(b.graph_node) and len(b.rigid_body):
            ocb.addBody(str(b.rigid_body))

    phy.s.setPeriod(old_T)
    phy.s.start()
#    graph.s.start()









RESOURCES_PATH = "resources/dae/"

def createKukaWorld(robotName="kuka", H_init=None):
    kukaworld = desc.scene.parseColladaFile(RESOURCES_PATH + "kuka_lwr.dae",
                                            root_node_name=robotName+"_root",
                                            append_label_library = '',
                                            append_label_nodes = robotName,
                                            append_label_graph_meshes = robotName)
    
    root_node = kukaworld.scene.graphical_scene.root_node
    children_nodes = root_node.children
    
    daeToKin = {"kuka_base"+robotName: robotName+"_00",
                "kuka_1"+robotName   : robotName+"_01",
                "kuka_2"+robotName   : robotName+"_02",
                "kuka_3"+robotName   : robotName+"_03",
                "kuka_4"+robotName   : robotName+"_04",
                "kuka_5"+robotName   : robotName+"_05",
                "kuka_6"+robotName   : robotName+"_06",
                "kuka_7"+robotName   : robotName+"_07"}


    def setNodePosition(node, H):
        node.ClearField("position")
        node.position.extend(H.tolist())
    map(lambda node: setNodePosition(node, lgsm.Displacementd()), root_node.children)

    desc.graphic.applyMaterialSet(root_node, material_set=["xde/YellowOpaqueAvatars", "xde/GreenOpaqueAvatars", "xde/RedOpaqueAvatars"])

    kuka_mass = 1.5
    kuka_damping = .5 # TODO!!!!!! change to 1.0 in the original script

    H00 = lgsm.Displacementd(0.0, 0., 0.0, 0, 0., 0., 1.)
#    H00 = lgsm.Displacementd(0.0, 0., 0.0, 1, 0., 0., 0)
    H01 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    H12 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    H23 = lgsm.Displacement(lgsm.vectord(0,0.,0.4) , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    H34 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[-1,0,0],[0,-1,0],[0,0,1]])))
    H45 = lgsm.Displacement(lgsm.vectord(0,0.,0.39), lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    H56 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[0,1,0],[-1,0,0],[0,0,1]])))
    H67 = lgsm.Displacement(lgsm.vectord(0,0.,0.)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))


    kinematic_tree = \
    (daeToKin["kuka_base"+robotName], kuka_mass, H00, [], [
      (daeToKin["kuka_1"+robotName], kuka_mass, H01, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, -45*math.pi/180)], [
       (daeToKin["kuka_2"+robotName], kuka_mass, H12, [('hinge', [0,0,0], [0,1,0], kuka_damping, -320*math.pi/180, 320*math.pi/180, -2*30*math.pi/180)], [
        (daeToKin["kuka_3"+robotName], kuka_mass, H23, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, 60*math.pi/180)], [
         (daeToKin["kuka_4"+robotName], kuka_mass, H34, [('hinge', [0,0,0], [0,1,0], kuka_damping, -320*math.pi/180, 320*math.pi/180, 45*math.pi/180)], [
          (daeToKin["kuka_5"+robotName], kuka_mass, H45, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, 0*math.pi/180)], [
           (daeToKin["kuka_6"+robotName], kuka_mass/2, H56, [('hinge', [0,0,0], [0,1,0], kuka_damping, -320*math.pi/180, 320*math.pi/180, 90*math.pi/180)], [
            (daeToKin["kuka_7"+robotName], kuka_mass/20, H67, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, 90*math.pi/180)], [
            ])
           ])
          ])
         ])
        ])
       ])
      ])
     ])


    if H_init is None:
        H_init = lgsm.Displacementd()
    if isinstance(H_init, list) or isinstance(H_init, tuple):
        H_init=lgsm.Displacementd(*H_init)
    root = desc.robot.addKinematicTree(kukaworld.scene.physical_scene, parent_node=None, tree=kinematic_tree, fixed_base=True, H_init=H_init)

    def setNodeMaterial(node):
        node.rigid_body.contact_material = "material.metal"
    desc.core.visitDepthFirst(setNodeMaterial, root)

    kuka_offset = 0.004

    def createKukaComposite(node_name, composite_name):
        graph_node = desc.core.findInTree(kukaworld.scene.graphical_scene.root_node, node_name)
        composite = desc.collision.addCompositeMesh(kukaworld.scene.collision_scene, composite_name, offset=kuka_offset)
        desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)
        composite.root_node.ClearField("position")
        composite.root_node.position.extend([0,0,0,1,0,0,0])

    for child in children_nodes:
        createKukaComposite(child.name, daeToKin[child.name]+".comp")


    # association between physics, graphics and collision

    def createKukaBinding(node_name, body_name, comp_name):
        graph_node = desc.core.findInTree(kukaworld.scene.graphical_scene.root_node, node_name)
        graph_node.name = body_name # it is suitable to have the same name for both graphics and physics.
        desc.scene.addBinding(kukaworld, body_name, body_name, "", comp_name)

    for child in children_nodes:
        createKukaBinding(child.name, daeToKin[child.name], daeToKin[child.name]+".comp")


    kuka_segments = daeToKin.values()
    kuka_bodies = kuka_segments
    desc.physic.addMechanism(kukaworld.scene.physical_scene, robotName, daeToKin["kuka_base"+robotName], [], kuka_bodies, kuka_segments)
    
    return kukaworld







def createWorldFromUrdfFile(urdfFileName, robotName):
    """
    """
#    kukaworld = desc.scene.parseColladaFile(fileName,
#                                            root_node_name=robotName+"_root",
#                                            append_label_library = '',
#                                            append_label_nodes = robotName,
#                                            append_label_graph_meshes = robotName)
    urdfWorld = desc.scene.createWorld(name=robotName+"root")

    urdfRobot, urdfNodes = parse_urdf(urdfFileName, robotName)
    kin_tree = urdfNodes[robotName+urdfRobot.get_root()]
    
    print kin_tree
    
    is_fixed_base = True #TODO: must be set
    H_init = lgsm.Displacementd()
    
    root = desc.robot.addKinematicTree(urdfWorld.scene.physical_scene, parent_node=None, tree=kin_tree, fixed_base=is_fixed_base, H_init=H_init)


    root_node = urdfWorld.scene.graphical_scene.root_node
    children_nodes = root_node.children                     #TODO: if structure is not a comb, but a tree


    def setNodePosition(node, H):
        node.ClearField("position")
        node.position.extend(H.tolist())
    map(lambda node: setNodePosition(node, lgsm.Displacementd()), children_nodes)
    
    urdf_bodies   = [robotName+v for v in  urdfRobot.links.keys()]
    urdf_segments = urdf_bodies
    desc.physic.addMechanism(urdfWorld.scene.physical_scene, robotName, robotName+urdfRobot.get_root(), [], urdf_bodies, urdf_segments)
    
    return urdfWorld






def parse_urdf(urdfFileName, robotName):
    """
    """
    from urdf import URDF
    robot = URDF.load_xml_file("resources/urdf/kuka.xml")


    def RollPitchYaw2Quaternion(roll, pitch, yaw):
        cph,sph = np.cos(roll), np.sin(roll)
        cth,sth = np.cos(pitch), np.sin(pitch)
        cps,sps = np.cos(yaw), np.sin(yaw)
        R = [[cth*cps              , cth*sps              , -sth   ],
             [sph*sth*cps - cph*sps, sph*sth*sps + cph*cps, cth*sph],
             [cph*sth*cps + sph*sps, cph*sth*sps - sph*cps, cth*cph]]
             
        return lgsm.Rotation3.fromMatrix(np.matrix(R))


    def get_joint_from_child_link(child_name):
        for j_name, j in robot.joints.items():
            if j.child == child_name:
                return j
        return None # if not found

    nodes = {}

    for l_name, link  in robot.links.items():
        #mass, inertias, H_parent_body = val
        mass = link.inertial.mass
        j  = get_joint_from_child_link(l_name)
        if j is not None:
            position = j.origin.position
            rotation = j.origin.rotation
            H_parent_body = lgsm.Displacement(lgsm.vectord(*position) , RollPitchYaw2Quaternion(*rotation))

            nodes[robotName+l_name] = [robotName+l_name, mass, H_parent_body, [], []]
        else:
            nodes[robotName+l_name] = [robotName+l_name, mass, lgsm.Displacementd(), [], []]


    urdf_joint_type_to_xde = {"revolute": "hinge",
                              } #TODO: to complete
    for j_name, joint in robot.joints.items():
        p_name = joint.parent
        c_name = joint.child
#        V_p_joint =  lgsm.vectord(*j.origin.position)
#        joint_axis_in_p = lgsm.vectord(*[float(v) for v in j.axis.split()])
        V_p_joint =  j.origin.position
        joint_axis_in_p = [float(v) for v in j.axis.split()] #PB with parser
        qmin    = j.limits.lower
        qmax    = j.limits.upper
        tau_max = j.limits.effort
        joint_damping = j.dynamics.damping if hasattr(j.dynamics, "damping") else 0
        qinit = 0 #TODO: no mean to give init value of q
        
        if j.joint_type in ["revolute"]: #TODO: to complete
            nodes[robotName+c_name][3].append(  (urdf_joint_type_to_xde[j.joint_type], V_p_joint, joint_axis_in_p, joint_damping, qmin, qmax, qinit)   )
            nodes[robotName+p_name][4].append(nodes[robotName+c_name])
        else:
            raise ValueError
    
    return robot, nodes #, nodes[robot.get_root()]  #[robot.get_root()]






