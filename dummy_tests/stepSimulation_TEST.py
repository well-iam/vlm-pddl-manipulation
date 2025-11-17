import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

sim.setStepping(True)

joint = sim.getObject('/Franka/joint')
print(f'PRIMO: {joint}')
ccw = False
time_elapsed = False

sim.startSimulation()
last_time = sim.getSimulationTime()
while (t := sim.getSimulationTime()) < 60:
    if t-last_time>5:
        time_elapsed = True
        last_time = sim.getSimulationTime()
    else:
        time_elapsed = False

    if time_elapsed and ccw:
        ccw = False
        sim.setJointPosition(joint, 90*3.14/180)
    elif time_elapsed and not ccw:
        ccw = True
        sim.setJointPosition(joint, -90 * 3.14 / 180)
    #s = f'Simulation time: {t:.2f} [s]'
    #print(s)
    sim.step()
sim.stopSimulation()