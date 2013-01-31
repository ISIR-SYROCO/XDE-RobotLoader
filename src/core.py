
import desc.robot
import desc.scene
import desc.material
import desc.graphic
import desc.core
import desc.simple.graphic
import desc.simple.collision

import lgsm
import numpy as np

import os


import urdf



################################################################################
def transport(M, H):
    """Transport (express) the mass matrix into another frame.

    :param M: the mass matrix expressed in the original frame (say, `a`)
    :type M: (6,6)-shaped array
    :param H: homogeneous matrix from the new frame (say `b`) to the
              original one: `H_{ab}`
    :type H: (4,4)-shaped array
    :rtype: (6,6)-shaped array
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
    """
    """
    assert(len(vec) == 3)
    skm = np.array([[0      ,-vec[2], vec[1]],
                    [ vec[2], 0     ,-vec[0]],
                    [-vec[1], vec[0], 0     ]])
    return skm



def RollPitchYaw2Quaternion(roll, pitch, yaw):
    """ Give the Quaternion coresponding to the roll,pitch,yaw rotation
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
    """
    """
    q0, q1, q2, q3 = Q
    rpy = [ np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)),
            np.arcsin(2*(q0*q2 - q1*q3)),
            np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))]
    return rpy



def UrdfPose2Displacement(urdfPose):
    return lgsm.Displacement(lgsm.vector(urdfPose.position) ,RollPitchYaw2Quaternion(*urdfPose.rotation))


def Displacement2UrdfPose(H):
    return urdf.Pose([H.x, H.y, H.z], Quaternion2RollPitchYaw(H.getRotation()))


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




import desc.simple.scene
import desc.simple.physic
def addObjectFromDae(graphFileName, objectName, H_init=None, is_fixed_base = True, collFileName=None, composite_offset=0.001, mass=1., moments_of_inertia=None, H_inertia_segment=None, material_name="material.concrete"):

    ##### CREATE WORLD & FILL GRAPHICAL TREE
    object_world = desc.simple.scene.parseColladaFile(graphFileName,
                                                        append_label_library = objectName,
                                                        append_label_nodes = objectName,
                                                        append_label_graph_meshes = objectName)

    ##### FILL COLLISION TREE
    compositeName = objectName+".comp"
    if collFileName is not None:
        coll_object_world = desc.simple.scene.parseColladaFile(collFileName, append_label_library="_"+objectName+".collision")
    else:
        coll_object_world = object_world
    desc.simple.collision.addCompositeMesh(object_world, coll_object_world, composite_name=compositeName, offset=composite_offset, clean_meshes=True)

    ##### FILL PHYSICAL TREE
    node = desc.simple.physic.addRigidBody(object_world, objectName)
    if moments_of_inertia is None:
        moments_of_inertia = [0,0,0]
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




def createWorldFromUrdfFile(urdfFileName, robotName, H_init=None, is_fixed_base = True, minimal_damping=0.001, composite_offset=0.001): #TODO: delete defined_mat
    """
    """
    urdfWorld = desc.scene.createWorld(name=robotName+"root")
    root_node = urdfWorld.scene.graphical_scene.root_node

    urdfRobot, urdfNodes, urdfGraphNodes, urdfCollNodes, urdfMatNodes = parse_urdf(urdfFileName, robotName, minimal_damping)


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
    print "GET GRAPHICAL TREE..."
    binding_phy_graph = {}
    for link_name in urdfRobot.links.keys():
        robotLinkName = link_name+robotName
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
            transform_gn = desc.graphic.addGraphicalNode(urdfWorld.scene.graphical_scene, name=robotLinkName+"_mesh_transform", parent_node=graph_node)
            desc.graphic.setNodeScale(transform_gn, mesh_scale)
            desc.graphic.setNodePosition(transform_gn, mesh_position)

            tmp_world = desc.scene.parseColladaFile( filename,
                                                     append_label_library = robotName,
                                                     append_label_nodes = robotName,
                                                     append_label_graph_meshes = robotName )

            if robotLinkName in urdfMatNodes:
                mat = urdfWorld.library.materials.add()
                mat.name = robotLinkName+"_material"
                desc.material.fillColorMaterial(mat, [ float(x) for x in urdfMatNodes[robotLinkName] ])
                desc.graphic.applyMaterialSet(tmp_world.scene.graphical_scene.root_node, material_set=[mat.name])
            else:
                desc.graphic.applyMaterialSet(tmp_world.scene.graphical_scene.root_node, material_set=["xde/YellowOpaqueAvatars", "xde/GreenOpaqueAvatars", "xde/RedOpaqueAvatars"]) #TODO: delete?

            if len(node_in_file) > 0:
                node_to_copy = node_in_file+robotName
            else:
                node_to_copy = "root"+robotName

            ######################
            # add body mesh node #
            ######################
            child_node  = desc.core.findInTree(tmp_world.scene.graphical_scene.root_node, node_to_copy)
            parent_node = getParentNode(tmp_world.scene.graphical_scene.root_node,        node_to_copy)
            if parent_node is not None:
                Hp_c = lgsm.Displacement(parent_node.position[:]).inverse() * lgsm.Displacement(child_node.position[:])
            else:
                Hp_c = lgsm.Displacement()
            
#            ud = Displacement2UrdfPose(Hp_c.inverse())
#            print child_node.name, ": <origin xyz=\"{} {} {}\" rpy=\"{} {} {}\" />".format(ud.position[0], ud.position[1], ud.position[2], ud.rotation[0], ud.rotation[1], ud.rotation[2])
            
            child_node.ClearField("position")
            child_node.position.extend(Hp_c.tolist())
            
            
            mesh_node = desc.simple.graphic.addGraphicalTree(urdfWorld, tmp_world,
                                                            node_name=robotLinkName+"_mesh",                        # name of of mesh in dest world
                                                            dest_parent_node_name=robotLinkName+"_mesh_transform",  # parent node of mesh in dest world
                                                            src_node_name=node_to_copy)                             # name of node to copy in src world
                                                            #ignore_library_conflicts=True)

#    import dsimi.interactive
#    dsimi.interactive.shell()()


    #######################################################
    # Save meshes for COLLISION scene and create bindings #
    #######################################################
    print "GET COLLISION TREE..."
    binding_phy_coll = {}
    #for link_name, mesh_filename in urdfCollNodes.items():
    for link_name in urdfRobot.links.keys():
        robotLinkName = link_name+robotName
        if robotLinkName in urdfCollNodes:
            mesh_filename, mesh_position, mesh_scale = urdfCollNodes[robotLinkName]
            filename, sep, node_in_file = mesh_filename.partition("#")

            tmp_world = desc.scene.parseColladaFile( filename ,
                                                     append_label_library = robotName+'coll',
                                                     append_label_nodes = robotName,
                                                     append_label_graph_meshes = robotName )

            

            ################ CREATE DUMMY TREE to get coll tree-structure: ################
            dummy_world = desc.scene.createWorld(name=robotLinkName+"_coll_mesh")
            transform_gn = desc.graphic.addGraphicalNode(dummy_world.scene.graphical_scene, name=robotLinkName+"_coll_mesh_transform") #, parent_node=robotLinkName+"_coll_mesh")
            desc.graphic.setNodeScale(transform_gn, mesh_scale)
            desc.graphic.setNodePosition(transform_gn, mesh_position)
            
            
            if len(node_in_file) > 0:
                node_to_copy = node_in_file+robotName
            else:
                node_to_copy = "root"+robotName

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
                                                            dest_parent_node_name=robotLinkName+"_coll_mesh_transform",     # parent node of mesh in dest world
                                                            src_node_name=node_to_copy)                                     # name of node to copy in src world
            ################ END OF DUMMY TREE ################


            composite_name = robotLinkName+".comp"
            
                
            createComposite(dummy_world, robotLinkName+"_coll_mesh", composite_name, composite_offset)
            desc.simple.collision.addCompositeMesh(urdfWorld, dummy_world, composite_name, src_node_name=robotLinkName+"_coll_mesh", offset=composite_offset)



            binding_phy_coll[robotLinkName] = composite_name

        else:
            binding_phy_coll[robotLinkName] = ""


    ###################
    # Create bindings #
    ###################
    #for link_name in urdfGraphNodes:
    for link_name in urdfRobot.links.keys():
        robotLinkName = link_name+robotName
        createBinding(urdfWorld, robotLinkName, binding_phy_graph[robotLinkName], binding_phy_coll[robotLinkName]) #TODO: put bindings with real collision name.


    ######################################################################################
    # Set proper Inertial properties as defined in URDF file, else from collision meshes #
    ######################################################################################
    def setNodeMomentsOfInertia(node):
        uname = node.rigid_body.name[:-len(robotName)]
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
            Mpf    = transport(Mc, H_c_pf)
            H_b_c  = lgsm.Displacementd(lgsm.vectord(p), R.inverse())
            H_b_pf = H_b_c * H_c_pf
            desc.physic.fillRigidBody(node.rigid_body,  mass=m, moments_of_inertia=[Mpf[0,0], Mpf[1,1], Mpf[2,2]], H_inertia_segment=H_b_c, contact_material=link_material)
        else:
            compNode = desc.core.findInList(urdfWorld.scene.collision_scene.meshes, node.rigid_body.name+".comp")
            if compNode is not None:
                desc.physic.computeInertiaParameters(node.rigid_body, urdfWorld.library, compNode)
            else:
                print "Warning: no inertia set on urdf file for link", uname
            #TODO: warn if no collision & no inertia are given in the urdf file

    desc.core.visitDepthFirst(setNodeMomentsOfInertia, root)


    return urdfWorld






simple_shapes_dae = os.path.dirname(__file__) + os.sep + "simple_shapes.dae" # Dont really know where to place it

def parse_urdf(urdfFileName, robotName, minimal_damping):
    """
    """
    robot = urdf.URDF.load_xml_file(urdfFileName)

    phy_nodes      = {}
    graph_nodes    = {}
    coll_nodes     = {}
    material_nodes = {}


    ##############################################
    # Functions defined for parsing robot easily #
    ##############################################
    def get_joint_from_child_link(child_name):
        """
        """
        for j_name, joint in robot.joints.items():
            if joint.child == child_name:
                return joint
        return None # if not found

    
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
                dict_to_fill[linkName] = [simple_shapes_dae + "#simple_shape_box", mesh_origin, element.geometry.dims ]
            elif isinstance(element.geometry, urdf.Sphere):
                radius = element.geometry.radius
                dict_to_fill[linkName] = [simple_shapes_dae + "#simple_shape_sphere_"+simple_shape_def, mesh_origin, (radius,radius,radius)]
            elif isinstance(element.geometry, urdf.Cylinder):
                length, radius = element.geometry.length, element.geometry.radius
                dict_to_fill[linkName] = [simple_shapes_dae + "#simple_shape_cylinder_"+simple_shape_def, mesh_origin, (radius,radius,length) ]


    ############################################
    # Parsing all links (bodies) in urdf files #
    ############################################
    # The goal is to get the inertial, visual and collision
    # properties which will be used in the physical nodes of
    # the world. We do not care about structure here.
    #
    for l_name, link  in robot.links.items():
        robotLinkName =  l_name+robotName
        mass = link.inertial.mass

        j_from_child  = get_joint_from_child_link(l_name)
        if j_from_child is not None:
            H_parent_body = UrdfPose2Displacement(j_from_child.origin)
        else:
            H_parent_body = lgsm.Displacementd()

        phy_nodes[robotLinkName] = [robotLinkName, mass, H_parent_body, [], []] # create a node for the XDE kinematic tree in dict phy_nodes

        get_visual_and_coll_data(robotLinkName, link.visual,    graph_nodes, "high") # fill dict graph_nodes (which bodies has visual)
        get_visual_and_coll_data(robotLinkName, link.collision, coll_nodes,  "low")  # fill dict coll_nodes  (which bodies has collision)

        if hasattr(link.visual, "material") and link.visual.material.color is not None: # save color, if any
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
        assert(p_name in robot.links), p_name+" is not listed in the urdf links..."
        assert(c_name in robot.links), c_name+" is not listed in the urdf links..."
        
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
        #A_p_joint = [round(v, 10) for v in A_p_joint] TODO: useless I suppose


        #possible urdf joint type = {"revolute","continuous","prismatic","fixed", "planar", "floating (deprecated)"}
        if joint.joint_type in ["revolute", "prismatic"]:
            jType = "hinge" if joint.joint_type == "revolute" else "prismatic"
            phy_nodes[c_name+robotName][3].append(  (jType, V_p_joint, A_p_joint, joint_damp, qmin, qmax, qinit)   )
            phy_nodes[p_name+robotName][4].append(phy_nodes[c_name+robotName])
        elif joint.joint_type == "fixed":
            raise ValueError(" more test needed before using this type of joint")
            phy_nodes[c_name+robotName][3].append( [] )
            phy_nodes[p_name+robotName][4].append(phy_nodes[c_name+robotName])
        elif joint.joint_type in ["continuous", "planar", "floating"]:
            raise ValueError("joint type '"+joint.joint_type+"' is in urdf convention, but it is not managed with this loader...")
        else:
            raise ValueError("joint type '"+joint.joint_type+"' is NOT in urdf convention. Invalid URDF.")


    return robot, phy_nodes, graph_nodes, coll_nodes, material_nodes











################################################################################
def addContactLaws(world):
    """
    """
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


