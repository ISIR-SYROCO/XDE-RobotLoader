from PySide import QtCore, QtGui
import IPython
import sys
import lgsm

window_list = []

class JointGui(QtGui.QWidget):
    """ Control Panel to set joint positions of a robot
    :param robot: The GVM robot
    :param robot_name: Name of the robot (str)
    :param joint_map: Dictionnary {'joint_name' : joint_rank}
    """
    def __init__(self, robot, robot_name, joint_map):
        super(JointGui, self).__init__()
        self.robot = robot
        dof = self.robot.getJointSpaceDim()
        self.joint_position = lgsm.zeros(dof)

        if joint_map:
            self.joint_map = joint_map
        else:
        #joint_map is empty, use str(joint_rank) as dictionnary key
            self.joint_map = {str(v):k for v, k in [[i,i] for i in range(dof)]}

        #New group of widgets
        self.groupbox = QtGui.QGroupBox(robot_name, self)

        #Sliders group
        self.groupbox_sliders = QtGui.QGroupBox("Joints")

        #Sliders to joint map
        self.joint_signal_mapping = QtCore.QSignalMapper(self)

        self.sliders = []

        self.initGui(robot_name)

    def initGui(self, robot_name):
        #Size of the window x,y, w,h
        self.setGeometry(300, 300, 500, 500)

        #Checkbox to enable gravity
        self.gravity_check = QtGui.QCheckBox("Enable Gravity")

        self.gravity_check.toggle()

        #Setting callback
        self.gravity_check.stateChanged.connect(self.enable_gravity)

        #Stacking checkbox and sliders group
        groupboxlayout = QtGui.QVBoxLayout()
        groupboxlayout.addWidget(self.gravity_check)
        self.addSliders()

        self.joint_signal_mapping.mapped.connect(self.setJoint)
        self.groupbox_sliders.setLayout(self.gridlayout)
        groupboxlayout.addWidget(self.groupbox_sliders)
        self.groupbox.setLayout(groupboxlayout)

        self.setWindowTitle(robot_name)
        self.show()

    #Set joint position callback
    def setJoint(self, id):
        #id is the identificator of the slider/joint
        #joint_signal_mapping.mapping(id) is the slider,
        #Value is int and is in [-500,500] so we divide by 100.0
        self.joint_position[id] = self.joint_signal_mapping.mapping(id).value()/100.0
        self.robot.setJointPositions(self.joint_position)
        self.robot.setJointVelocities(lgsm.zeros(self.robot.getJointSpaceDim()))

    def addSliders(self):
        #Add sliders in a grid: Label|Slider
        self.gridlayout = QtGui.QGridLayout()
        current_line = 0
        for joint_name, joint_rank in self.joint_map.items():
            label = QtGui.QLabel(joint_name+": ")
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)

            #slider.setTickPosition(QtGui.QSlider.TicksBothSides)
            slider.setSingleStep(1)
            slider.setRange(-500, 500)
            self.sliders.append(slider)

            #Connect slider valueChanged signal to the signalMapper joint_signal_mapping
            slider.valueChanged.connect(self.joint_signal_mapping.map)

            #Setting the mapping slider:id where id is the joint_rank
            #The slider can be accessed via joint_signal_mapping.mapping(id)
            self.joint_signal_mapping.setMapping(slider, joint_rank)

            #Add the label and the slider to the grid
            self.gridlayout.addWidget(label, current_line, 0)
            self.gridlayout.addWidget(slider, current_line, 1)
            current_line = current_line +1

    #Enable gravity callback
    def enable_gravity(self, state):
        if state == QtCore.Qt.Checked:
            self.robot.enableGravity(True)
        else:
            self.robot.enableGravity(False)

""" Create the joint control window
:param robot: The GVM Robot
:param robot_name: Name of the window, usually the robot name
:param joint_map: Dictionnary {'joint_name' : joint_rank}
"""
def createJointGui(robot, robot_name="JointGui", joint_map={}):
    app_created = False
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtGui.QApplication(sys.argv)
        app_created = True
    app.references = set()
    wid = JointGui(robot, robot_name, joint_map)
    window_list.append(wid)
    app.references.add(wid)
    wid.show()
    if app_created:
        app.exec_()
    return wid

#Calling qt will by default block IPython, in order not to block IPython
#We must enable qt
def configure(app=None):
    IPython.lib.inputhook.enable_qt4(app)