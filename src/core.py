
import desc.physic
import desc.scene
import desc.material
import desc.graphic
import desc.core
import desc.simple.scene
import desc.simple.physic
import desc.simple.graphic
import desc.simple.collision

import xde.desc.physic
import physicshelper

import lgsm
import numpy as np

import os


import urdf



################################################################################

def principalframe(M):
    """ Find the principal frame of inertia of a mass matrix.

    :param M: a (6x6) mass matrix expressed in any frame (say `a`)

    :rtype: a lgsm.Displacement representing `H_{am}` from `a` to the principal inertia frame `m`
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

    result = lgsm.Displacement()
    result.setTranslation(lgsm.vectord(*position))
    result.setRotation(lgsm.Rotation3.fromMatrix(R))
    return result




def RollPitchYaw2Quaternion(roll, pitch, yaw):
    """ Give the Quaternion coresponding to the (roll,pitch,yaw) rotation

    :param roll: a double which represents the roll angle
    :param pitch: a double which represents the pitch angle
    :param yaw: a double which represents the yaw angle

    :rtype: return a lgsm.Quaternion representing this rotation
    """
    cph,sph = np.cos(roll/2.),  np.sin(roll/2.)
    cth,sth = np.cos(pitch/2.), np.sin(pitch/2.)
    cps,sps = np.cos(yaw/2.),   np.sin(yaw/2.)

    q0 = cph*cth*cps + sph*sth*sps
    q1 = sph*cth*cps - cph*sth*sps
    q2 = cph*sth*cps + sph*cth*sps
    q3 = cph*cth*sps - sph*sth*cps

    Q = lgsm.Quaternion(q0,q1,q2,q3)
    Q.normalize()
    return Q



def Quaternion2RollPitchYaw(Q):
    """ Give the (roll,pitch,yaw) rotation coresponding to the Quaternion

    :param Q: the lgsm.Quaternion one wants to convert

    :rtype: a 3-list corresponding to the [roll, pitch, yaw] rotation

    :warning: it seems that some conversion problems occur when pitch = +|- pi/2
    """
    # new computation of Quaternion to roll,pitch,yaw: it is based on urdf_dom
    w, x, y, z = Q
    sqw, sqx, sqy, sqz = w**2, x**2, y**2, z**2
    roll = np.arctan2( 2*( y*z + w*x ), sqw - sqx -sqy + sqz )
    sarg = -2 ( x*z - w*y )
    if (sarg <= -1.0):
        pitch = -0.5*np.pi
    else:
        if (sarg >= 1.0):
            pitch = 0.5*np.pi
        else:
            pitch = np.arcsin(sarg)
    yaw = np.arctan2( 2*( x*y + w*z ), sqw + sqx - sqy -sqz)

    return roll, pitch, yaw



def UrdfPose2Displacement(urdfPose):
    """ Convert a pose from urdf file into a displacement

    :param urdfPose: a urdf.Pose

    :rtype: a lgsm.Displacement
    """
    if urdfPose is not None:
        result = lgsm.Displacement()
        result.setTranslation(lgsm.vector(urdfPose.position))
        result.setRotation(RollPitchYaw2Quaternion(*urdfPose.rotation))
        return result
    else:
        return lgsm.Displacement()


def Displacement2UrdfPose(H):
    """ Convert a displacement into a pose from urdf file

    :param urdfPose: a lgsm.Displacement

    :rtype: a urdf.Pose
    """
    return urdf.Pose([H.x, H.y, H.z], Quaternion2RollPitchYaw(H.getRotation()))


def createBinding(world, phy_name, graph_name, comp_name):
    """ Create binding between physical, graphical & collision scenes.

    :param world: a scene_pb2.World where the binding is set
    :param phy_name: the name of the physical node
    :param graph_name: the name of the graphical node
    :param comp_name: the name of the composite (collision) name

    :warning: this method changes the name of the graph node into phy_name
    """
    graph_node      = desc.core.findInTree(world.scene.graphical_scene.root_node, graph_name)
    phy_node        = desc.physic.findInPhysicalScene(world.scene.physical_scene, phy_name)
    graph_node.name = phy_name # it is suitable to have the same name for both graphics and physics.

    phy_node.rigid_body.composite_name=comp_name


def createComposite(world, graph_name, composite_name, offset):
    """ Create a composite node copied from a graphical node

    :param world: a scene_pb2.World where the graph node is, and where the composite will be created
    :param graph_name: the name of the graphical node
    :param composite_name: the name of the new composite node
    :param offset: the thickness dimension which covers the composite mesh
    """
    graph_node = desc.core.findInTree(world.scene.graphical_scene.root_node, graph_name)
    composite  = desc.collision.addCompositeMesh(world.scene.physical_scene.collision_scene, composite_name, offset=offset)
    desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)
    composite.root_node.ClearField("position")
    composite.root_node.position.extend([0,0,0,1,0,0,0])




def getParentNode(root_node, node_name):
    parent_nodes = []
    def getNodeName(node):
        if node_name in [c.name for c in node.children]:
            parent_nodes.append(node)
    desc.core.visitDepthFirst(getNodeName, root_node)

    if len(parent_nodes) == 0:
        return None
    elif len(parent_nodes) == 1:
        return parent_nodes[0]
    else:
        raise RuntimeError, "found many parents in tree. problem..."








############################################################################
#                                                                          #
# Methods that transform dae or urdf files into scene_pb2.World instances, #
# which can be loaded into XDE with the physic.deserializeWorld, or with   #
# the XDE-WorldManager.                                                    #
#                                                                          #
############################################################################

def createWorldFromDae(daeFileName, objectName, H_init=None, is_fixed_base=True, can_collide=True, collFileName=None, composite_offset=0.001, scale=1., mass=1., moments_of_inertia=None, H_inertia_segment=None, material_name="material.metal"):
    """ Create a world from a simple object described in a dae file.

    It does not create a robot (a kinematic tree-structure) but a simple body through
    a physical node, a graphical node and a composite (collision) node.

    :param daeFileName: the dae file path to convert
    :param objectName: the name of the physical node
    :param H_init: the initial pose (lgsm.Displacement) of the object in scene; by  default lgsm.Displacement(0,0,0)
    :param can_collide: True if it collide in the physical scene, or False if it is just a decoration
    :param collFileName: the dae file path if collision mesh is in another dae, else it is the daeFileName
    :param composite_offset: the thickness dimension which covers the composite mesh
    :param scale: the scale of the mesh, can be a float or a 3-list
    :param mass: the mass of the physical object
    :param moments_of_inertia: the moments of inertia of the object
    :param H_inertia_segment: the lgsm.Displacement from the object frame to the inertial principal frame
    :param material_name: the material name, used in the definition of frictional interaction

    :rtype: a scene_pb2.World with one physical/graphical/composite node
    """

    ##### CREATE WORLD & FILL GRAPHICAL TREE
    object_world = desc.simple.scene.parseColladaFile(daeFileName,
                                                        append_label_library = objectName,
                                                        append_label_nodes = objectName,
                                                        append_label_graph_meshes = objectName)

    object_world.scene.graphical_scene.root_node.ClearField("scale")
    if hasattr(scale, "__iter__") and len(scale) == 3:
        object_world.scene.graphical_scene.root_node.scale.extend(scale)
    else:
        object_world.scene.graphical_scene.root_node.scale.extend([scale]*3)

    ##### FILL COLLISION TREE
    compositeName = ""
    if can_collide is True:
        compositeName = objectName+".comp"
        if collFileName is not None:
            coll_object_world = desc.simple.scene.parseColladaFile(collFileName, append_label_library="_"+objectName+".collision")
        else:
            coll_object_world = object_world
        desc.simple.collision.addCompositeMesh(object_world, coll_object_world, composite_name=compositeName, offset=composite_offset, clean_meshes=True)

        object_world.scene.physical_scene.collision_scene.meshes[0].root_node.ClearField("scale")
        if hasattr(scale, "__iter__") and len(scale) == 3:
            object_world.scene.physical_scene.collision_scene.meshes[0].root_node.scale.extend(scale)
        else:
            object_world.scene.physical_scene.collision_scene.meshes[0].root_node.scale.extend([scale]*3)


    ##### FILL PHYSICAL TREE
    node = desc.simple.physic.addRigidBody(object_world, objectName)
    if H_inertia_segment is None:
        H_inertia_segment = lgsm.Displacement()
    desc.physic.fillRigidBody(node.rigid_body,mass=mass, moments_of_inertia=moments_of_inertia, H_inertia_segment=H_inertia_segment, contact_material=material_name)

    if H_init is None:
        H_init = lgsm.Displacementd()
    if isinstance(H_init, list) or isinstance(H_init, tuple):
        H_init=lgsm.Displacementd(*H_init)

    if is_fixed_base is True:
        desc.simple.physic.addFixedJoint(object_world, objectName+".joint", objectName, H_init)
    else:
        desc.simple.physic.addFreeJoint(object_world, objectName+".joint", objectName, H_init)

    ##### CREATE BINDINGS
    createBinding(object_world, objectName, "root"+objectName, compositeName)

    return object_world




def createWorldFromUrdfFile(urdfFileName, robotName, H_init=None, is_fixed_base = True, minimal_damping=0.001, composite_offset=0.001, use_collada_color=True):
    """ Create a world from a kinematic tree-structure described in an urdf file.

    It creates a kinematic tree-structure by parsing an urdf file, which describes
    the whole data about physic, kinematic, graphic and collision.
    For more information, see: `http://www.ros.org/wiki/urdf/XML/model`

    :param urdfFileName: the urdf file path which describes the robot
    :param robotName: the robot name given in this world
    :param H_init: the initial pose (lgsm.Displacement) of the object in scene; by  default lgsm.Displacement(0,0,0)
    :param is_fixed_base: True if the robot is rigidly linked to the ground, False if it has a free-flying root
    :param minimal_damping: the minimal articular damping. Should NOT be null, because it seems to block the simulation
    :param composite_offset: the thickness dimension which covers the robto composite meshes
    :param use_collada_color: If material/color node is empty in the urdf, override color defined in the collada file by default color if set to False, else use the color defined in the collada file

    :rtype: a scene_pb2.World that contains the robot

    In this created world, the robot is named 'robotName', and robot components are prefixed with 'robotName+.'
    For instance, if segments are 'seg01', 'seg02', ... in urdf file, and we create a robot 'johnny5'
    they can be found in the GVM scene with the names: 'johnny5.seg01', 'johnny5.seg02', etc...
    """
    urdfWorld = desc.scene.createWorld(name=robotName)
    root_node = urdfWorld.scene.graphical_scene.root_node

    urdfRobot, urdfNodes, urdfGraphNodes, urdfCollNodes, urdfMatNodes = parse_urdf(urdfFileName, robotName, minimal_damping)



    #######################################################
    # Save meshes for GRAPHICAL scene and create bindings #
    #######################################################
    # We create the graphical tree as a comb.
    # The root node is the "urdfWorld.scene.graphical_scene.root_node".
    # Each child node represents a body(/segment?) in the physical scene.
    # This way, bindings will be created between physical and graphical nodes.
    # With all these graphical nodes, we create the following structure to
    # add the corresponding meshes:
    #
    # root_node
    #  |_________________________________________ _ _ _
    #  |                     |            |     |
    # body1_node            body2_node
    #  |                     |
    # body1_transform_node   .
    #  |                     .
    # body1_mesh_node        .
    #
    # The body_node position will change during simulations, due to the bindings
    # The body_transform_node represents the position/scale defined in the URDF
    # The body_mesh_node finally represents the mesh defined in URDF
    #
    #print "GET GRAPHICAL TREE..."
    binding_phy_graph = {}
    for link_name in urdfRobot.links.keys():
        robotLinkName = robotName+"."+link_name
        binding_phy_graph[robotLinkName] = robotLinkName
        #################
        # add body node #
        #################
        graph_node = desc.graphic.addGraphicalNode(urdfWorld.scene.graphical_scene, name=robotLinkName, parent_node=urdfWorld.scene.graphical_scene.root_node)
        graph_node.position.extend([0,0,0,1,0,0,0])
        graph_node.scale.extend([1,1,1])

        if robotLinkName in urdfGraphNodes:
            mesh_filename, mesh_position, mesh_scale = urdfGraphNodes[robotLinkName]
            filename, sep, node_in_file = mesh_filename.partition("#")

            ###########################
            # add body transform node #
            ###########################
            transform_gn = desc.graphic.addGraphicalNode(urdfWorld.scene.graphical_scene, name=robotLinkName+".mesh_transform", parent_node=graph_node)
            desc.graphic.setNodeScale(transform_gn, mesh_scale)
            desc.graphic.setNodePosition(transform_gn, mesh_position)

            tmp_world = desc.scene.parseColladaFile( filename,
                                                     append_label_library = robotLinkName,
                                                     append_label_nodes = robotLinkName,
                                                     append_label_graph_meshes = robotLinkName )

            if robotLinkName in urdfMatNodes:
                mat = urdfWorld.library.materials.add()
                mat.name = robotLinkName+"_material"
                desc.material.fillColorMaterial(mat, [ float(x) for x in urdfMatNodes[robotLinkName] ])
                desc.graphic.applyMaterialSet(tmp_world.scene.graphical_scene.root_node, material_set=[mat.name])
            elif use_collada_color == False:
                desc.graphic.applyMaterialSet(tmp_world.scene.graphical_scene.root_node, material_set=["xde/YellowOpaqueAvatars", "xde/GreenOpaqueAvatars", "xde/RedOpaqueAvatars"]) #TODO: delete?

            if len(node_in_file) > 0:
                node_to_copy = node_in_file+robotLinkName
            else:
                node_to_copy = "root"+robotLinkName

            ######################
            # add body mesh node #
            ######################
            child_node  = desc.core.findInTree(tmp_world.scene.graphical_scene.root_node, node_to_copy)
            parent_node = getParentNode(tmp_world.scene.graphical_scene.root_node,        node_to_copy)
            if parent_node is not None:
                Hp_c = lgsm.Displacement(parent_node.position[:]).inverse() * lgsm.Displacement(child_node.position[:])
            else:
                Hp_c = lgsm.Displacement()

            child_node.ClearField("position")
            child_node.position.extend(Hp_c.tolist())

            mesh_node = desc.simple.graphic.addGraphicalTree(urdfWorld, tmp_world,
                                                            node_name=robotLinkName+".mesh",                        # name of of mesh in dest world
                                                            dest_parent_node_name=robotLinkName+".mesh_transform",  # parent node of mesh in dest world
                                                            src_node_name=node_to_copy)                             # name of node to copy in src world




    #######################################################
    # Save meshes for COLLISION scene and create bindings #
    #######################################################
    #print "GET COLLISION TREE..."
    binding_phy_coll = {}
    #for link_name, mesh_filename in urdfCollNodes.items():
    for link_name in urdfRobot.links.keys():
        robotLinkName = robotName+"."+link_name
        if robotLinkName in urdfCollNodes:
            mesh_filename, mesh_position, mesh_scale = urdfCollNodes[robotLinkName]
            filename, sep, node_in_file = mesh_filename.partition("#")

            tmp_world = desc.scene.parseColladaFile( filename ,
                                                     append_label_library = robotLinkName,
                                                     append_label_nodes = robotLinkName,
                                                     append_label_graph_meshes = robotLinkName )



            ################ CREATE DUMMY TREE to get coll tree-structure: ################
            dummy_world = desc.scene.createWorld(name=robotLinkName+".coll_mesh")
            transform_gn = desc.graphic.addGraphicalNode(dummy_world.scene.graphical_scene, name=robotLinkName+".coll_mesh_transform") #, parent_node=robotLinkName+"_coll_mesh")
            desc.graphic.setNodeScale(transform_gn, mesh_scale)
            desc.graphic.setNodePosition(transform_gn, mesh_position)


            if len(node_in_file) > 0:
                node_to_copy = node_in_file+robotLinkName
            else:
                node_to_copy = "root"+robotLinkName

            #### add body mesh node ####
            child_node  = desc.core.findInTree(tmp_world.scene.graphical_scene.root_node, node_to_copy)
            parent_node = getParentNode(tmp_world.scene.graphical_scene.root_node,        node_to_copy)
            if parent_node is not None:
                Hp_c = lgsm.Displacement(parent_node.position[:]).inverse() * lgsm.Displacement(child_node.position[:])
            else:
                Hp_c = lgsm.Displacement()

            child_node.ClearField("position")
            child_node.position.extend(Hp_c.tolist())

            mesh_node = desc.simple.graphic.addGraphicalTree(dummy_world, tmp_world,
                                                            node_name=node_to_copy,                           # name of of mesh in dest world
                                                            dest_parent_node_name=robotLinkName+".coll_mesh_transform",     # parent node of mesh in dest world
                                                            src_node_name=node_to_copy)                                     # name of node to copy in src world
            ################ END OF DUMMY TREE ################
            composite_name = robotLinkName+".comp"

            createComposite(dummy_world, robotLinkName+".coll_mesh", composite_name, composite_offset)
            desc.simple.collision.addCompositeMesh(urdfWorld, dummy_world, composite_name, src_node_name=robotLinkName+".coll_mesh", offset=composite_offset, clean_meshes=True)

            binding_phy_coll[robotLinkName] = composite_name

        else:
            binding_phy_coll[robotLinkName] = ""

    #########################################################
    # Create kinematic tree and mechanism in physical scene #
    #########################################################
    #print "GET KINEMATIC TREE..."
    kin_tree = urdfNodes[robotName+"."+urdfRobot.get_root()]

    if H_init is None:
        H_init = lgsm.Displacementd()
    if isinstance(H_init, list) or isinstance(H_init, tuple):
        H_init=lgsm.Displacementd(*H_init)
    desc.physic.fillKinematicTree(urdfWorld.scene.physical_scene.nodes.add(), tree=kin_tree, fixed_base=is_fixed_base, H_init=H_init, composites=binding_phy_coll)


    urdf_bodies   = [str(robotName+"."+v) for v in  urdfRobot.links.keys()]
    urdf_segments = urdf_bodies
    desc.physic.addMechanism(urdfWorld.scene.physical_scene, robotName, robotName+"."+urdfRobot.get_root(), [], urdf_bodies, urdf_segments)

    ######################################################################################
    # Set proper Inertial properties as defined in URDF file, else from collision meshes #
    ######################################################################################
    def setNodeMomentsOfInertia(node):
        uname = node.rigid_body.name[len(robotName+"."):]
        link = urdfRobot.links[uname]
        if hasattr(link.visual, "material") and link.visual.material is not None:
            link_material = link.visual.material.name
        else:
            link_material = ""

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
            Mpf    = H_c_pf.adjoint().transpose() * Mc * H_c_pf.adjoint()  #transport(Mc, H_c_pf)
            H_b_c  = lgsm.Displacementd()
            H_b_c.setTranslation(lgsm.vectord(p))
            H_b_c.setRotation(R)
            H_b_pf = H_b_c * H_c_pf
            desc.physic.fillRigidBody(node.rigid_body, mass=m, moments_of_inertia=[Mpf[0,0], Mpf[1,1], Mpf[2,2]], H_inertia_segment=H_b_pf, contact_material=link_material)
        else:
            compNode = desc.core.findInList(urdfWorld.scene.physical_scene.collision_scene.meshes, node.rigid_body.name+".comp")
            if compNode is not None:
                desc.physic.computeInertiaParameters(node.rigid_body, urdfWorld.library, compNode)
            else:
                print "Warning: robot '"+robotName+"', no inertia set on urdf file for link", uname
            #TODO: warn if no collision & no inertia are given in the urdf file
    root = desc.physic.findInPhysicalScene(urdfWorld.scene.physical_scene, robotName+"."+urdfRobot.get_root())
    desc.core.visitDepthFirst(setNodeMomentsOfInertia, root)


    ###################
    # Create bindings #
    ###################
    #for link_name in urdfGraphNodes:
    for link_name in urdfRobot.links.keys():
        robotLinkName = robotName+"."+link_name
        createBinding(urdfWorld, robotLinkName, binding_phy_graph[robotLinkName], binding_phy_coll[robotLinkName]) #TODO: put bindings with real collision name.


    return urdfWorld



def getDynamicModelFromWorld(world, nnode=0, nmechanism=0):
    """ Translate a robot described in a world into a dynamic model.
    
    :param world: a scene_pb2.World where to find the robot model
    :param int nnode: the index of the node in world.scene.physical_scene.nodes from where to copy the kinematic tree
    :param int nmechanism: the index of the mechanism in world.scene.physical_scene.mechanisms we want to copy
    """
    multiBodyModel = xde.desc.physic.physic_pb2.MultiBodyModel()
    multiBodyModel.kinematic_tree.CopyFrom(world.scene.physical_scene.nodes[ nnode ])
    multiBodyModel.meshes.extend(world.library.meshes)
    multiBodyModel.mechanism.CopyFrom(world.scene.physical_scene.mechanisms[ nmechanism ])
    multiBodyModel.composites.extend(world.scene.physical_scene.collision_scene.meshes)
    dynModel = physicshelper.createDynamicModel(multiBodyModel)
    return dynModel



def get_simple_shapes_dae():
    """ Get the dae file path of the simple shapes, box, sphere, cylinder """
    return os.path.dirname(__file__) + os.sep + "simple_shapes.dae"



def parse_urdf(urdfFileName, robotName, minimal_damping):
    """ Parse a urdf file to obtain usefull information for XDE.

    :param urdfFileName: the urdf file path
    :param robotName: the robot name, mainly to rename urdf components with the convention 'robotName.compName'
    :param minimal_damping: if not given in urdf file, minimal damping. Should NOT be zero to avoid any blocking simulation
    """
    robot = urdf.URDF.load_xml_file(urdfFileName)

    phy_nodes      = {}
    graph_nodes    = {}
    coll_nodes     = {}
    material_nodes = {}


    ##############################################
    # Functions defined for parsing robot easily #
    ##############################################
    def get_visual_and_coll_data(linkName, element, dict_to_fill, simple_shape_def):
        """
        """
        if hasattr(element, "geometry"):
            mesh_origin = UrdfPose2Displacement(element.origin)
            if isinstance(element.geometry, urdf.Mesh):
                mesh_scale = (1,1,1)
                if element.geometry.scale is not None:
                    mesh_scale_str = element.geometry.scale.split()
                    if len(mesh_scale_str) == 1:
                        mesh_scale = [float(mesh_scale_str[0])]*3
                    elif len(mesh_scale_str) == 3:
                        mesh_scale = [float(v) for v in mesh_scale_str]
                    else:
                        raise ValueError, "Cannot interpret scale '"+element.geometry.scale+"' of link '"+linkName+"' correctly"
                dict_to_fill[linkName] =  [str(os.path.dirname(urdfFileName) + os.sep + element.geometry.filename), mesh_origin, mesh_scale ]
            elif isinstance(element.geometry, urdf.Box):
                dict_to_fill[linkName] = [get_simple_shapes_dae() + "#simple_shape_box", mesh_origin, element.geometry.dims ]
            elif isinstance(element.geometry, urdf.Sphere):
                radius = element.geometry.radius
                dict_to_fill[linkName] = [get_simple_shapes_dae() + "#simple_shape_sphere_"+simple_shape_def, mesh_origin, (radius,radius,radius)]
            elif isinstance(element.geometry, urdf.Cylinder):
                length, radius = element.geometry.length, element.geometry.radius
                dict_to_fill[linkName] = [get_simple_shapes_dae() + "#simple_shape_cylinder_"+simple_shape_def, mesh_origin, (radius,radius,length) ]


    ############################################
    # Parsing all links (bodies) in urdf files #
    ############################################
    # The goal is to get the inertial, visual and collision
    # properties which will be used in the physical nodes of
    # the world. We do not care about structure here.
    #
    for l_name, link  in robot.links.items():
        robotLinkName =  robotName+"."+l_name
        mass = link.inertial.mass

        phy_nodes[robotLinkName] = [robotLinkName, mass, lgsm.Displacementd(), [], []] # create a node for the XDE kinematic tree in dict phy_nodes

        get_visual_and_coll_data(robotLinkName, link.visual,    graph_nodes, "high") # fill dict graph_nodes (which bodies has visual)
        get_visual_and_coll_data(robotLinkName, link.collision, coll_nodes,  "low")  # fill dict coll_nodes  (which bodies has collision)

        if (link.visual is not None) and (link.visual.material is not None) and (link.visual.material.color is not None): # save color, if any
                material_nodes[robotLinkName] = link.visual.material.color.rgba


    ####################################
    # Parsing all joints in urdf files #
    ####################################
    # The goal is to get the structure of the robot, the relation
    # between bodies, type of joints, axis, etc...
    # At the end, we fill phy_nodes dict to create the kinematic tree
    # for the world in XDE.
    #
    for j_name, joint in robot.joints.items():
        p_name     = joint.parent
        c_name     = joint.child
        assert(p_name in robot.links), p_name+" is not listed in the urdf links in "+urdfFileName
        assert(c_name in robot.links), c_name+" is not listed in the urdf links in "+urdfFileName

        V_p_joint  = joint.origin.position
        A_c_joint  = [float(v) for v in joint.axis.split()] #TODO:PB with parser, it returns a string and not a tuple
        qmin       = joint.limits.lower     if hasattr(joint.limits, "lower")     else -12.
        qmax       = joint.limits.upper     if hasattr(joint.limits, "upper")     else  12.
        tau_max    = joint.limits.effort    if hasattr(joint.limits, "effort")    else 10000.
        joint_damp = joint.dynamics.damping if hasattr(joint.dynamics, "damping") else minimal_damping
        qinit      = 0 #TODO: no mean to give init value of q

        #Warning, urdf gives axis in child frame (in joint frame, which is the same).
        #We need to transform this vector from child to parent frame coordinates to fit XDE requirement
        R_p_c     = RollPitchYaw2Quaternion(*joint.origin.rotation)
        A_p_joint = R_p_c * A_c_joint
        A_p_joint.resize(3)


        #possible urdf joint type = {"revolute","continuous","prismatic","fixed", "planar", "floating (deprecated)"}
        if joint.joint_type in ["revolute", "prismatic"]:
            jType = "hinge" if joint.joint_type == "revolute" else "prismatic"
            phy_nodes[robotName+"."+c_name][2] = UrdfPose2Displacement(joint.origin)
            phy_nodes[robotName+"."+c_name][3].append(  (jType, V_p_joint, A_p_joint, joint_damp, qmin, qmax, qinit)   )
            phy_nodes[robotName+"."+p_name][4].append(phy_nodes[robotName+"."+c_name])
        elif joint.joint_type == "fixed":
            raise ValueError(" more test needed before using this type of joint")
            phy_nodes[robotName+"."+c_name][2] = UrdfPose2Displacement(joint.origin)
            phy_nodes[robotName+"."+c_name][3].append( [] )
            phy_nodes[robotName+"."+p_name][4].append(phy_nodes[robotName+"."+c_name])
        elif joint.joint_type in ["continuous", "planar", "floating"]:
            raise ValueError("joint type '"+joint.joint_type+"' is in urdf convention, but it is not managed with this loader...")
        else:
            raise ValueError("joint type '"+joint.joint_type+"' is NOT in urdf convention. Invalid URDF.")


    return robot, phy_nodes, graph_nodes, coll_nodes, material_nodes





