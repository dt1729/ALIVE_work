import numpy as np
import time
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import rospy
import carla
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from nav_msgs.msg import Odometry
import math

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
rospy.init_node('Velocity_plot', anonymous=False)
ego_vehicle_info = rospy.wait_for_message("/carla/hero/vehicle_info", CarlaEgoVehicleInfo)

actor = world.get_actor(ego_vehicle_info.id)
xdata, ydata = [], []
def run(niter=2000, doblit=True):
    """
    Display the simulation using matplotlib, optionally using blit for speed
    """

    target_speed = 15 
    fig, ax = plt.subplots(1, 1)
    v = actor.get_velocity()
    #ax.set_aspect('equal')
    # ax.set_xlim(0, 200)
    # ax.set_ylim(0, 100)
    plt.axis([0, 200, 0, 190])
    ax.hold(True)    #rw = randomwalk()
    #x, y = rw.next()
    current_time = carla.Timestamp()
    
    xdata.append(0)
    ydata.append(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))    #x, y = 0, np.hypot(actor.get_velocity().x, actor.get_velocity().y)
    plt.show(False)
    plt.draw()
    if doblit:
        # cache the background
        background = fig.canvas.copy_from_bbox(ax.bbox)
    points, = ax.plot([], [], 'o', markersize = 1)
    tic = time.time()
    for ii in xrange(niter):
        # update the xy data
        #x, y = rw.next()
        v = actor.get_velocity()
        xdata.append(ii+1)
        # ydata.append((18/5)*np.hypot(actor.get_velocity().x, actor.get_velocity().y))
        ydata.append((3.6) * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        print(ydata[ii])
        #x, y = ii+1, np.hypot(actor.get_velocity().x, actor.get_velocity().y)
        plt.axis([0, ii + 50, 0, 190])
        points.set_data(xdata, ydata)
        if doblit:
            # restore background
            #fig.canvas.restore_region(background)
            # redraw just the points
            ax.draw_artist(points)
            # fill in the axes rectangle
            fig.canvas.blit(ax.bbox)
        else:
            # redraw everything
            fig.canvas.draw()
            plt.yticks(np.arange(0,200,10))
            plt.xlabel('timestep')
            plt.ylabel('velocity(km/h)')
            plt.grid()
    plt.close(fig)
if __name__ == '__main__':
    run(doblit=False)
    #run(doblit=True)