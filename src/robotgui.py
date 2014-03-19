from PySide import QtCore, QtGui
import IPython
import sys

window_list = []

class JointGui(QtGui.QWidget):
    def __init__(self, robot, robot_name):
        super(JointGui, self).__init__()
        self.robot = robot
        self.initGui(robot_name)


    def initGui(self, robot_name):
        self.setGeometry(300, 300, 250, 250)
        self.gravity_check = QtGui.QCheckBox("Enable Gravity", self)
        self.gravity_check.move(20, 20)
        self.gravity_check.toggle()
        self.gravity_check.stateChanged.connect(self.enable_gravity)
        self.setWindowTitle(robot_name)
        self.show()

    def enable_gravity(self, state):
        if state == QtCore.Qt.Checked:
            self.robot.enableGravity(True)
        else:
            self.robot.enableGravity(False)


def createJointGui(robot, robot_name="JointGui"):
    app_created = False
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtGui.QApplication(sys.argv)
        app_created = True
    app.references = set()
    wid = JointGui(robot, robot_name)
    window_list.append(wid)
    app.references.add(wid)
    wid.show()
    if app_created:
        app.exec_()
    return wid

def configure(app=None):
    IPython.lib.inputhook.enable_qt4(app)



