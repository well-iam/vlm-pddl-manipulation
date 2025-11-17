from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

if __name__ == '__main__':

    client = RemoteAPIClient()
    sim = client.require('sim')

    franka = sim.getObject('/Franka')
    fake_franka = sim.getObject('/FakeFranka')
    #franka2 = sim.getObject('/Franka2')
    floor = sim.getObject('/Floor')
    #cuboid2 = sim.getObject('/Cuboid2')
    cuboid3 = sim.getObject('/Cuboid3')

    robot_collection_handle = sim.createCollection(0)
    sim.addItemToCollection(robot_collection_handle, sim.handle_tree, franka, 0)
    print(sim.getCollectionObjects(robot_collection_handle))

    fake_robot_collection_handle = sim.createCollection(0)
    sim.addItemToCollection(fake_robot_collection_handle, sim.handle_tree, fake_franka, 0)
    print(sim.getCollectionObjects(fake_robot_collection_handle))

    # robot2_collection_handle = sim.createCollection(0)
    # sim.addItemToCollection(robot2_collection_handle, sim.handle_tree, franka2, 0)
    # print(sim.getCollectionObjects(robot2_collection_handle))

    res, collidingObjectHandles = sim.checkCollision(cuboid3, fake_robot_collection_handle)
    print(f'RES: {res}')
    print(f'HANDLES: {collidingObjectHandles}')

    # res, collidingObjectHandles = sim.checkCollision(franka, floor)
    # print(f'FLOOR RES: {res}')
    # print(f'FLOOR HANDLES: {collidingObjectHandles}')
    #
    # res, collidingObjectHandles = sim.checkCollision(cuboid, franka)
    # print(f'CUBOID RES: {res}')
    # print(f'CUBOID HANDLES: {collidingObjectHandles}')

    #sim.stopSimulation()

    print('Program ended')