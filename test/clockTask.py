
import deploy.deployer as ddeployer

# to create basic rtt tasks
import rtt_interface as rtt
import dsimi.rtt

def createClock():
    clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
    return clock
