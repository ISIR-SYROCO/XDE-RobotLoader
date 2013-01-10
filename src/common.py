
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


import os





def createAllAgents(TIME_STEP):
    print "CREATE CLOCK..."
    import clockTask
    clock = clockTask.createClock()

    print "CREATE GRAPHIC..."
    import graphic
    graph = graphic.createTask()
    scene_name = graphic.init()
    graph.s.Connectors.IConnectorBody.new("icb", "body_state_H", scene_name)
    graph.s.Connectors.IConnectorFrame.new("icf", "framePosition", scene_name)

    print "CREATE PHYSIC..."
    phy = physic.createTask()
    physic.init(TIME_STEP)
    phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")

    print "CREATE PORTS..."
    phy.addCreateInputPort("clock_trigger", "double")
    icps = phy.s.Connectors.IConnectorSynchro.new("icps")
    icps.addEvent("clock_trigger")
    clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

    graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))
    graph.getPort("framePosition").connectTo(phy.getPort("body_state_H"))

    phy.s.setPeriod(TIME_STEP)
    clock.s.setPeriod(TIME_STEP)

    graph.s.start()
    phy.s.start()
    clock.s.start()


    return clock, phy, graph





def delWorld(old_world):
    
    print "REMOVE CONNECTION PHY/GRAPH..."
    ocb = physic.phy.s.Connectors.OConnectorBodyStateList("ocb")
    for b in old_world.scene.rigid_body_bindings:
        if len(b.graph_node) and len(b.rigid_body):
            ocb.removeBody(str(b.rigid_body))


    print "REMOVE GRAPHICAL WORLD..."
    #delete graphical scene
    def deleteNodeInGraphicalAgent(node):
        for child in node.children:
            deleteNodeInGraphicalAgent(child)
        nname = str(node.name)
        print 'deleting', nname
        if graphic.graph_scn.SceneInterface.nodeExists(nname):
            graphic.graph_scn.SceneInterface.removeNode(nname)

    deleteNodeInGraphicalAgent(old_world.scene.graphical_scene.root_node)



    print "REMOVE PHYSICAL WORLD..."

    phy = physic.phy
    print "STOP PHYSIC..."
    phy.s.stop()
    old_T = phy.s.getPeriod()
    phy.s.setPeriod(0)
    
    #delete physical scene
    for mechanism in old_world.scene.physical_scene.mechanisms:
        mname = str(mechanism.name)
        physic.phy.s.deleteComponent(mname)

    scene = physic.ms
    def removeRigidBodyChildren(node):
        for child in node.children:
            removeRigidBodyChildren(child)
        print "deleting", node.rigid_body.name
        rbname = str(node.rigid_body.name)

        if rbname in scene.getBodyNames(): #TODO: Body and rigidBody, the same???
            scene.removeRigidBody(rbname)

        for to_del in [rbname, str(rbname+".comp"), str(node.inner_joint.name)]:
            if to_del in physic.phy.s.getComponents():
                physic.phy.s.deleteComponent(to_del)


    for node in old_world.scene.physical_scene.nodes:
        removeRigidBodyChildren(node)


    print "REMOVE UNUSED MATERIAL..."
    scene.removeUnusedContactMaterials()


    print "RESTART PHYSIC..."
    phy.s.setPeriod(old_T)
    phy.s.start()



def addWorld(new_world, deserialize_graphic=True):
    """
    """
    phy = physic.phy
    print "STOP PHYSIC..."
    phy.s.stop()
    old_T = phy.s.getPeriod()
    phy.s.setPeriod(0)

    print "CREATE WORLD..."
    scene = physic.ms
    Lmat = new_world.scene.physical_scene.contact_materials
    for mat in Lmat:
        if mat in scene.getContactMaterials():
            new_world.scene.physical_scene.contact_materials.remove(mat)

    physic.deserializeWorld(new_world)

    while len(new_world.scene.physical_scene.contact_materials):
        new_world.scene.physical_scene.contact_materials.remove(new_world.scene.physical_scene.contact_materials[-1])
    new_world.scene.physical_scene.contact_materials.extend(Lmat)


    print "RESTART PHYSIC..."
    phy.s.setPeriod(old_T)
    phy.s.start()


    if deserialize_graphic is True:
        graphic.deserializeWorld(new_world)

        print "CREATE CONNECTION PHY/GRAPH..."
        ocb = phy.s.Connectors.OConnectorBodyStateList("ocb")
        for b in new_world.scene.rigid_body_bindings:
            if len(b.graph_node) and len(b.rigid_body):
                ocb.addBody(str(b.rigid_body))





def addMarkers(world, bodies_to_display=None):
    #TODO: problem. should investigate to add markers properly; now, there is warning messages
    print "problem. should investigate to add markers properly; now, there is warning messages"
    allNodeNames = []
    def getNodeName(node):
        allNodeNames.append(node.rigid_body.name)

    if bodies_to_display is None:
        for child in world.scene.physical_scene.nodes:
            desc.core.visitDepthFirst(getNodeName, child)
        bodies_to_display = allNodeNames

    ocb = physic.phy.s.Connectors.OConnectorBodyStateList("ocb")
    for body_name in bodies_to_display:
        graphic.graph_scn.MarkersInterface.addMarker(str(body_name), False)
        ocb.addBody(str(body_name))
        

################################################################################
def transport(M, H):
    """Transport (express) the mass matrix into another frame.

    :param M: the mass matrix expressed in the original frame (say, `a`)
    :type M: (6,6)-shaped array
    :param H: homogeneous matrix from the new frame (say `b`) to the
              original one: `H_{ab}`
    :type H: (4,4)-shaped array
    :rtype: (6,6)-shaped array

    **Example:**

    >>> M_a = diag((3., 2., 4., 1., 1., 1.))
    >>> H_ab = Hg.transl(1., 3., 0.)
    >>> M_b = transport(M_a, H_ab)
    >>> allclose(M_b, [[ 12.,  -3.,   0.,   0.,   0.,  -3.],
    ...                [ -3.,   3.,   0.,   0.,   0.,   1.],
    ...                [  0.,   0.,  14.,   3.,  -1.,   0.],
    ...                [  0.,   0.,   3.,   1.,   0.,   0.],
    ...                [  0.,   0.,  -1.,   0.,   1.,   0.],
    ...                [ -3.,   1.,   0.,   0.,   0.,   1.]])
    True
    >>> ismassmatrix(M_b)k1g.comp',
 'kuka_2k1g.comp',
 'kuka_3k1g.comp',
 'kuka_4k1g.comp',
 'kuka_5k1g.comp',
 'kuka_6k1g.comp',
 'kuka_7k1g.comp',
 'kuka_basek1g
    True
    >>> from math import pi
    >>> H_ab = Hg.rotx(pi/4)
    >>> M_b = transport(M_a, H_ab)
    >>> allclose(M_b, [[ 3.,  0.,  0.,  0.,  0.,  0.],
    ...                [ 0.,  3.,  1.,  0.,  0.,  0.],
    ...                [ 0.,  1.,  3.,  0.,  0.,  0.],
    ...                [ 0.,  0.,  0.,  1.,  0.,  0.],
    ...                [ 0.,  0.,  0.,  0.,  1.,  0.],
    ...                [ 0.,  0.,  0.,  0.,  0.,  1.]])
    True
    >>> ismassmatrix(M_b)
    True

    """
    Ad = H.adjoint()
    return np.dot(Ad.T, np.dot(M, Ad))



def principalframe(M):
    """Find the principal frame of inertia of a mass matrix.

    :param M: mass matrix expressed in any frame (say `a`)
    :type M: (6,6)-shaped array
    :rtype: (4,4)-shaped array

    Returns the homogeneous matrix `H_{am}` to the principal inertia
    frame `m`

    **Example:**

    >>> M_a = diag((3.,2.,4.,1.,1.,1.))
    >>> H_ab = Hg.transl(1., 3., 0.)
    >>> M_b = transport(M_a, H_ab)
    >>> H_ba = principalframe(M_b)
    >>> dot(H_ab, H_ba)
    array([[ 1.,  0.,  0.,  0.],
           [ 0.,  1.,  0.,  0.],
           [ 0.,  0.,  1.,  0.],
           [ 0.,  0.,  0.,  1.]])

    """
    from numpy.linalg import eig, det
    m = M[5, 5]
    rx = M[0:3, 3:6]/m

    position = [rx[2, 1], rx[0, 2], rx[1, 0]]
    
    RSR = M[0:3, 0:3] + m*np.dot(rx, rx)
    [S, R] = eig(RSR)
    if det(R)<0.:
        iI = np.array([[0, 0, 1], [0, 1, 0], [1, 0, 0]])
        R = np.dot(R, iI)
        S = np.dot(iI, np.dot(S, iI))
    
    return lgsm.Displacement(  lgsm.vectord(*position)   , lgsm.Rotation3.fromMatrix(R)  )




def vec2SkewMatrix(vec):
    assert(len(vec) == 3)
    skm = np.array([[0      ,-vec[2], vec[1]],
                    [ vec[2], 0     ,-vec[0]],
                    [-vec[1], vec[0], 0     ]])
    return skm



def RollPitchYaw2Quaternion(roll, pitch, yaw):
    """ Give the Quaternion coresponding to the roll,pitch,yaw rotation
    """
    cph,sph = np.cos(roll), np.sin(roll)
    cth,sth = np.cos(pitch), np.sin(pitch)
    cps,sps = np.cos(yaw), np.sin(yaw)
    R = [[cth*cps              , cth*sps              , -sth   ],
         [sph*sth*cps - cph*sps, sph*sth*sps + cph*cps, cth*sph],
         [cph*sth*cps + sph*sps, cph*sth*sps - sph*cps, cth*cph]]
    return lgsm.Rotation3.fromMatrix(np.matrix(R))


def Quaternion2RollPitchYaw(Q):
    """
    """
    q0, q1, q2, q3 = Q
    rpy = [np.arctan2(2*q2*q3 + 2*q0*q1, q3**2 - q2**2 - q1**2 + q0**2),
           -np.arcsin(2*q1*q3 - 2*q0*q2),
           np.arctan2(2*q1*q2 + 2*q0*q3, q1**2 + q0**2 - q3**2 - q2**2)]
    return rpy




def createBinding(world, phy_name, graph_node_name, comp_name):
    """
    """
    graph_node      = desc.core.findInTree(world.scene.graphical_scene.root_node, graph_node_name)
    graph_node.name = phy_name # it is suitable to have the same name for both graphics and physics.
    desc.scene.addBinding(world, phy_name, phy_name, "", comp_name)


def createComposite(world, graph_node_name, composite_name, offset):
    """
    """
    graph_node = desc.core.findInTree(world.scene.graphical_scene.root_node, graph_node_name)
    composite  = desc.collision.addCompositeMesh(world.scene.collision_scene, composite_name, offset=offset)
    desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)
    composite.root_node.ClearField("position")
    composite.root_node.position.extend([0,0,0,1,0,0,0])




def printKinTree(ktree, prefix=""):
    print prefix+ktree[0], ":", ktree[3][0:3] #, ktree[3]   #ktree[1]: just the mass
    for subk in ktree[4]:
        printKinTree(subk, prefix+"   ")




def getDaeToKin(robotName):
    return     {"kuka_base"+robotName: robotName+"00",
                "kuka_1"+robotName   : robotName+"01",
                "kuka_2"+robotName   : robotName+"02",
                "kuka_3"+robotName   : robotName+"03",
                "kuka_4"+robotName   : robotName+"04",
                "kuka_5"+robotName   : robotName+"05",
                "kuka_6"+robotName   : robotName+"06",
                "kuka_7"+robotName   : robotName+"07"}


def getKukaKinematicTree(robotName, daeToKin):
    """
    """
    kuka_mass = 1.5
    kuka_damping = .5 # TODO!!!!!! change to 1.0 in the original script

    H00 = lgsm.Displacementd(0.0, 0., 0.0, 0, 0., 0., 1.)
    H01 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    H12 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    H23 = lgsm.Displacement(lgsm.vectord(0,0.,0.4) , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
    #H34 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[-1,0,0],[0,-1,0],[0,0,1]])))
    H34 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[np.cos(0.707),-np.sin(0.707),0],[np.sin(0.707),np.cos(0.707),0],[0,0,1]])))
    
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
    return kinematic_tree



RESOURCES_PATH = "resources/dae/"






def createGroundWorld():
    groundWorld = desc.simple.scene.parseColladaFile(RESOURCES_PATH+"ground.dae", root_node_name="root_ground")
    phy_ground_world = desc.simple.scene.parseColladaFile(RESOURCES_PATH+"ground_phy_50mm.dae", append_label_library=".phyground")

    desc.simple.collision.addCompositeMesh(groundWorld, phy_ground_world, composite_name="ground.comp", offset=0.05, clean_meshes=False, ignore_library_conflicts=False)

#    graph_node = desc.core.findInTree(groundWorld.scene.graphical_scene.root_node, "node-ground")
#    composite = desc.collision.addCompositeMesh(groundWorld.scene.collision_scene, "ground.comp", offset=0.001)
#    desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)

    desc.simple.physic.addRigidBody(groundWorld, "ground", mass=1, contact_material="material.concrete")
    desc.simple.physic.addFixedJoint(groundWorld, "ground.joint", "ground", lgsm.Displacement(0,0,-0.4))

    createBinding(groundWorld, "ground", "node-ground", "ground.comp")
    return groundWorld








def createKukaWorld(robotName="kuka", H_init=None):
    kukaworld = desc.scene.parseColladaFile(RESOURCES_PATH + "kuka_lwr.dae",
                                            root_node_name=robotName+"_root",
                                            append_label_library = '',
                                            append_label_nodes = robotName,
                                            append_label_graph_meshes = robotName)
    
    root_node = kukaworld.scene.graphical_scene.root_node
    children_nodes = root_node.children


    desc.graphic.applyMaterialSet(root_node, material_set=["xde/YellowOpaqueAvatars", "xde/GreenOpaqueAvatars", "xde/RedOpaqueAvatars"])

    daeToKin = getDaeToKin(robotName)
    kinematic_tree = getKukaKinematicTree(robotName, daeToKin)
#    printKinTree(kinematic_tree)
    
    def setNodePosition(node, H):
        node.ClearField("position")
        node.position.extend(H.tolist())
    map(lambda node: setNodePosition(node, lgsm.Displacementd()), root_node.children)
    
    if H_init is None:
        H_init = lgsm.Displacementd()
    if isinstance(H_init, list) or isinstance(H_init, tuple):
        H_init=lgsm.Displacementd(*H_init)
    root = desc.robot.addKinematicTree(kukaworld.scene.physical_scene, parent_node=None, tree=kinematic_tree, fixed_base=True, H_init=H_init)

    def setNodeMaterial(node):
        node.rigid_body.contact_material = "material.metal"
    desc.core.visitDepthFirst(setNodeMaterial, root)

    # create composite for collision
    kuka_offset = 0.004
    for child in children_nodes:
        createComposite(kukaworld, child.name, daeToKin[child.name]+".comp", kuka_offset)


    # association between physics, graphics and collision
#    desc.core.printTree(kukaworld.scene.graphical_scene.root_node)
    for child in children_nodes:
        createBinding(kukaworld, daeToKin[child.name], child.name, daeToKin[child.name]+".comp")


    kuka_segments = daeToKin.values()
    kuka_bodies = kuka_segments
    desc.physic.addMechanism(kukaworld.scene.physical_scene, robotName, daeToKin["kuka_base"+robotName], [], kuka_bodies, kuka_segments)

    return kukaworld







def createWorldFromUrdfFile(urdfFileName, robotName, H_init=None, is_fixed_base = True, minimal_damping=0.001, composite_offset=0.001): #TODO: delete defined_mat
    """
    """
    urdfWorld = desc.scene.createWorld(name=robotName+"root")

    root_node = urdfWorld.scene.graphical_scene.root_node
    children_nodes = root_node.children                     #TODO: if structure is not a comb, but a tree

    urdfRobot, urdfNodes, urdfGraphNodes, urdfCollNodes, urdfMatNodes = parse_urdf(urdfFileName, robotName, minimal_damping)



#    ####################
#    # Create Materials #
#    ####################
##    for material in urdfRobot.materials:
##        mat = urdfWorld.library.materials.add()
##        mat.name = material.name
##        desc.material.fillColorMaterial(mat, [ float(x) for x in material.color.rgba ]) #TODO: if no color is given???


    #########################################################
    # Create kinematic tree and mechanism in physical scene #
    #########################################################
    print "GET KINEMATIC TREE..."
    kin_tree = urdfNodes[urdfRobot.get_root()+robotName]

    if H_init is None:
        H_init = lgsm.Displacementd()
    if isinstance(H_init, list) or isinstance(H_init, tuple):
        H_init=lgsm.Displacementd(*H_init)
    root = desc.robot.addKinematicTree(urdfWorld.scene.physical_scene, parent_node=None, tree=kin_tree, fixed_base=is_fixed_base, H_init=H_init)

    urdf_bodies   = [str(v+robotName) for v in  urdfRobot.links.keys()]
    urdf_segments = urdf_bodies
    desc.physic.addMechanism(urdfWorld.scene.physical_scene, robotName, urdfRobot.get_root()+robotName, [], urdf_bodies, urdf_segments)


    #######################################################
    # Save meshes for GRAPHICAL scene and create bindings #
    #######################################################
    print "GET GRAPHICAL TREE..."
    binding_phy_graph = {}
    for link_name, mesh_filename in urdfGraphNodes.items():
        if mesh_filename is not None:
            filename, sep, node_in_file = mesh_filename.partition("#")

            tmp_world = desc.scene.parseColladaFile( str(os.path.dirname(urdfFileName)+"/"+filename ),
                                                     append_label_library = robotName,
                                                     append_label_nodes = robotName,
                                                     append_label_graph_meshes = robotName )

            if len(node_in_file) > 0:
                if link_name in urdfMatNodes:
                    mat = urdfWorld.library.materials.add()
                    mat.name = link_name+"_material"
                    desc.material.fillColorMaterial(mat, [ float(x) for x in urdfMatNodes[link_name] ])
                    desc.graphic.applyMaterialSet(tmp_world.scene.graphical_scene.root_node, material_set=[mat.name])
                else:
                    desc.graphic.applyMaterialSet(tmp_world.scene.graphical_scene.root_node, material_set=["xde/YellowOpaqueAvatars", "xde/GreenOpaqueAvatars", "xde/RedOpaqueAvatars"]) #TODO: delete
                desc.simple.graphic.addGraphicalTree(urdfWorld, tmp_world, node_in_file+robotName, src_node_name=node_in_file+robotName, ignore_library_conflicts=False)
            else:
                pass # TODO: treat if the whole file is the body (take the root)
                
            binding_phy_graph[link_name] = node_in_file+robotName
        else:
            binding_phy_graph[link_name] = ""


    #######################################################
    # Save meshes for COLLISION scene and create bindings #
    #######################################################
    print "GET COLLISION TREE..."
    binding_phy_coll = {}
    for link_name, mesh_filename in urdfCollNodes.items():
        if mesh_filename is not None:
            filename, sep, node_in_file = mesh_filename.partition("#")

            tmp_world = desc.scene.parseColladaFile( str(os.path.dirname(urdfFileName)+"/"+filename ),
                                                     append_label_library = robotName+'coll',
                                                     append_label_nodes = robotName,
                                                     append_label_graph_meshes = robotName )
            if len(node_in_file) > 0:
                composite_name = link_name+".comp"
                createComposite(tmp_world, node_in_file+robotName, composite_name, composite_offset)
                desc.simple.collision.addCompositeMesh(urdfWorld, tmp_world, composite_name, src_node_name=node_in_file+robotName, offset=composite_offset)
            else:
                pass # TODO: treat if the whole is the body (take the root)

            binding_phy_coll[link_name] = composite_name
        else:
            binding_phy_coll[link_name] = ""


    ###################
    # Create bindings #
    ###################
    for link_name in urdfGraphNodes:
        createBinding(urdfWorld, link_name, binding_phy_graph[link_name], binding_phy_coll[link_name]) #TODO: put bindings with real collision name.


    ######################################################################################
    # Set proper Inertial properties as defined in URDF file, else from collision meshes #
    ######################################################################################
    def setNodeMomentsOfInertia(node):
        uname = node.rigid_body.name[:-len(robotName)]
        link = urdfRobot.links[uname]
        link_material = link.visual.material.name if link.visual.material is not None else ""
        
        if hasattr(link.inertial.origin, "position"):
            p   = link.inertial.origin.position
            R   = RollPitchYaw2Quaternion(*link.inertial.origin.rotation)
            m   = link.inertial.mass
            Iud = link.inertial.matrix
            Inertia =np.array([[Iud['ixx'], Iud['ixy'], Iud['ixz']],
                               [Iud['ixy'], Iud['iyy'], Iud['iyz']],
                               [Iud['ixz'], Iud['iyz'], Iud['izz']]])
            Mc = np.zeros((6,6))
            Mc[0:3, 0:3] = Inertia
            Mc[3:6, 3:6] = m*np.eye(3)
            
            H_c_pf = principalframe(Mc)
            Mpf    = transport(Mc, H_c_pf)
            H_b_c  = lgsm.Displacementd(lgsm.vectord(p), R.inverse())
            H_b_pf = H_b_c * H_c_pf
            desc.physic.fillRigidBody(node.rigid_body,  mass=m, moments_of_inertia=[Mpf[0,0], Mpf[1,1], Mpf[2,2]], H_inertia_segment=H_b_c, contact_material=link_material)
        else:
            compNode = desc.core.findInList(urdfWorld.scene.collision_scene.meshes, node.rigid_body.name+".comp")
            desc.physic.computeInertiaParameters(node.rigid_body, urdfWorld.library, compNode)
            #TODO: warn if no collision & no inertia are given in the urdf file

    desc.core.visitDepthFirst(setNodeMomentsOfInertia, root)


    #######################
    # Set nodes materials #
    #######################
#    def setNodeMaterial(node):
#        node.rigid_body.contact_material = "material.metal"
#    desc.core.visitDepthFirst(setNodeMaterial, root)


#    def setNodePosition(node, H):
#        node.ClearField("position")
#        node.position.extend(H.tolist())
#    map(lambda node: setNodePosition(node, lgsm.Displacementd()), root_node.children)


    return urdfWorld






def parse_urdf(urdfFileName, robotName, minimal_damping):
    """
    """
    import urdf
    robot = urdf.URDF.load_xml_file(urdfFileName)

#    import dsimi.interactive
#    dsimi.interactive.shell()()

    def get_joint_from_child_link(child_name):
        for j_name, joint in robot.joints.items():
            if joint.child == child_name:
                return joint
        return None # if not found

    phy_nodes   = {}
    graph_nodes = {}
    coll_nodes  = {}
    material_nodes = {}

    for l_name, link  in robot.links.items():
        mass = link.inertial.mass
        j_from_child  = get_joint_from_child_link(l_name)

        robotLinkName =  l_name+robotName

        if j_from_child is not None:
            position = j_from_child.origin.position
            rotation = j_from_child.origin.rotation
            H_parent_body = lgsm.Displacement(lgsm.vectord(*position) , RollPitchYaw2Quaternion(*rotation))

            phy_nodes[robotLinkName] = [robotLinkName, mass, H_parent_body, [], []]
        else:
            phy_nodes[robotLinkName] = [robotLinkName, mass, lgsm.Displacementd(), [], []]
        
        if hasattr(link.visual, "geometry"):
            if isinstance(link.visual.geometry, urdf.Mesh):
                graph_nodes[robotLinkName] = link.visual.geometry.filename
            else:
                graph_nodes[robotLinkName] = None #TODO: complete with other shapes, box, sphere, etc...

        if hasattr(link.visual, "material"):
            if link.visual.material.color is not None:
                material_nodes[robotLinkName] = link.visual.material.color.rgba

        if hasattr(link.collision, "geometry"):
            if isinstance(link.collision.geometry, urdf.Mesh):
                coll_nodes[robotLinkName] = link.collision.geometry.filename
            else:
                coll_nodes[robotLinkName] = None #TODO: complete with other shapes, box, sphere, etc...


    urdf_joint_type_to_xde = {"revolute": "hinge",
                              } #TODO: to complete

    for j_name, joint in robot.joints.items():
        p_name     = joint.parent
        c_name     = joint.child
        V_p_joint  = joint.origin.position  #[0,0,0] #
        A_c_joint  = [float(v) for v in joint.axis.split()] #TODO:PB with parser
        qmin       = joint.limits.lower  if hasattr(joint.limits, "lower") else round(-4*np.pi)
        qmax       = joint.limits.upper  if hasattr(joint.limits, "upper") else  round(4*np.pi)
        tau_max    = joint.limits.effort if hasattr(joint.limits, "effort") else 10000.
        joint_damp = joint.dynamics.damping if hasattr(joint.dynamics, "damping") else minimal_damping
        qinit      = 0 #TODO: no mean to give init value of q
        
        #Warning, urdf gives axis in child frame (in joint frame, which is the same).
        #We need to transform this vector from child to parent frame coordinates to fit XDE requirement
        R_p_c     = RollPitchYaw2Quaternion(*joint.origin.rotation)
        A_p_joint = R_p_c * A_c_joint
        A_p_joint.resize(3)
        A_p_joint = [round(v, 10) for v in A_p_joint]
        
        if joint.joint_type in ["revolute"]: #TODO: to complete
            phy_nodes[c_name+robotName][3].append(  (urdf_joint_type_to_xde[joint.joint_type], V_p_joint, A_p_joint, joint_damp, qmin, qmax, qinit)   )
            phy_nodes[p_name+robotName][4].append(phy_nodes[c_name+robotName])
        else:
            raise ValueError


    return robot, phy_nodes, graph_nodes, coll_nodes, material_nodes








################################################################################
def addContactLaws(world):
    """
    """
#    ms = physic.ms
#    for mat in ["material.concrete", "material.metal"]:
#        if mat not in ms.getContactMaterials():
#            world.scene.physical_scene.contact_materials.append(mat)

#    world.scene.physical_scene.contact_materials.extend(["material.concrete", "material.metal"])
    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.metal"
    cl.material_j = "material.concrete"
    cl.law = cl.COULOMB
    cl.friction = 0.5

    cl = world.scene.physical_scene.contact_laws.add()
    cl.material_i = "material.metal"
    cl.material_j = "material.metal"
    cl.law = cl.COULOMB
    cl.friction = 0.5


def addCollisionPairs(world, component1name, component2name):
    cp = world.scene.collision_pairs.add()
    
#    if   desc.physic.findInPhysicalScene(world.scene.physical_scene, component1name):
#        cp.body_i = component1name
#    elif desc.core.findInList(world.scene.physical_scene.mechanisms, component1name):
#        cp_mechanism_i = component1name
#    else:
#        raise ValueError, "cannot find component "+component1name
#    
#    if   desc.physic.findInPhysicalScene(world.scene.physical_scene, component2name):
#        cp.body_j = component2name
#    elif desc.core.findInList(world.scene.physical_scene.mechanisms, component2name):
#        cp_mechanism_j = component2name
#    else:
#        raise ValueError, "cannot find component "+component2name


    cp.body_i = "ground"
    cp.mechanism_i = "k1g"
#    import dsimi.interactive
#    shell = dsimi.interactive.shell()
#    shell()
#    cp.enabled = True

