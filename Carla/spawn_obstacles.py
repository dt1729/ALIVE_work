import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import time
import random
from sensor_msgs.msg import Imu
import carla
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo


def spawn_obstacle(world,x, y, z, orientation, type):
    blueprint = random.choice(world.get_blueprint_library().filter(type))
    spawn_point = carla.Transform()
    spawn_point.location.x = x
    spawn_point.location.y = y
    spawn_point.location.z = z
    spawn_point.rotation.roll = 0.0
    spawn_point.rotation.pitch = 0.0
    spawn_point.rotation.yaw = -1*np.rad2deg(orientation)
    world.spawn_actor(blueprint, spawn_point)

def spawnVehicles7obs(world): #Currently giving issues
    spawn_obstacle(world,3.7, -63.3, 2.0, 0, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world,7.2, -83.4, 2.0, 0, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world,12.2, -84.6, 2.0, math.pi/2, 'vehicle.tesla.model3')
    spawn_obstacle(world,1.1, -104.7, 2.0, 0, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world,39.8, -194.7, 2.0, math.pi/2, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world,104.4, -200.0, 2.0, math.pi/2, 'vehicle.carlamotors.carlacola')
    # spawn_obstacle(world,20.5, -169.6, 2.0, 1.2, 'vehicle.tesla.model3')
    
    # spawn_obstacle(world,7.12, -80.3, 2.0, 1.3, 'vehicle.audi.tt')
    # spawn_obstacle(world,2.3, -28, 2.0, 1.4, 'vehicle.audi.tt')
    # spawn_obstacle(world,16.8, -101, 2.0, 1.35, 'vehicle.audi.tt')
    # spawn_obstacle(world,24.1, -170.4, 2.0, 1.38, 'vehicle.audi.tt')

def spawnVehicles(world):
    spawn_obstacle(world, 3.7, -63.3, 2.0, 0, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world, 7.2, -83.4, 2.0, math.pi/2, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world, 12.9, -182.4, 2.0, math.pi/2, 'vehicle.audi.tt')
    spawn_obstacle(world, 9.5, -123.9, 2.0, math.pi/2, 'vehicle.audi.tt')
    spawn_obstacle(world,39.8, -194.7, 2.0, math.pi/2, 'vehicle.carlamotors.carlacola')
    spawn_obstacle(world,104.4, -200.0, 2.0, 0, 'vehicle.audi.tt')
    spawn_obstacle(world, 123.9, -201.4, 2.0, 0, 'vehicle.audi.tt')
    spawn_obstacle(world, 112.5, -201.8, 2.0, 0, 'vehicle.audi.tt')
    # spawn_obstacle(world, 3.7, -63.3, 2.0, 0, 'vehicle.carlamotors.carlacola')
    # spawn_obstacle(world, 3.7, -63.3, 2.0, 0, 'vehicle.carlamotors.carlacola')



def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    spawnVehicles(world)

if __name__ == '__main__':
    print("Spawning obstacles!")
    main()