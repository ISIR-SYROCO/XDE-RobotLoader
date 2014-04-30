import fileinput
import xdefw.rtt
import rtt_interface
import lgsm

class ReplayJointTrajectory(xdefw.rtt.Task):
    def __init__(self, name, robot, loop=False):
        super(ReplayJointTrajectory, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        self.trajectory = []
        self.trigger_port = self.addCreateInputPort("tick", "VectorXd", True)
        self.loop = loop
        self.current_position = 0
        self.robot = robot

    def loadTrajectory(self, filename):
        for line in fileinput.input(filename):
            if line != "\n":
                v = lgsm.vectord([float(val) for val in line.split()])
                if len(v) == len(self.robot.getJointPositions()):
                    self.trajectory.append(v)

    def updateHook(self):
        self.tick, self.tick_ok         = self.trigger_port.read()
        if self.current_position != len(self.trajectory):
            self.robot.setJointPositions(self.trajectory[self.current_position])
            self.current_position = self.current_position + 1
        elif self.loop == True:
            self.current_position = 0

