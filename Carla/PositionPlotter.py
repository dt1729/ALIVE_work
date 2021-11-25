carla_sim = False

import numpy as np
import time
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import rospy
if carla_sim:
    import carla
    from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

xdata, ydata = [], []
if not carla_sim:
    xref, yref = [0.04242, -43.24427], [0.2279,-28.1193]
xref, yref = [19.4,83], [-210.4,-90]
rospy.init_node('Position_plotter', anonymous=False)
if carla_sim:
    # client = carla.Client('localhost', 2000)
    # client.set_timeout(2.0)
    # world = client.get_world()
    # ego_vehicle_info = rospy.wait_for_message("/carla/hero/vehicle_info", CarlaEgoVehicleInfo)
    # actor = world.get_actor(ego_vehicle_info.id)
    pose = rospy.wait_for_message("/carla/hero/odometry", Odometry)
    xdata.append(pose.pose.pose.position.x)
    ydata.append(pose.pose.pose.position.y)
else:
    loc_msg = rospy.wait_for_message("/mavros/local_position/pose", PoseStamped, timeout=10)
    xdata.append(loc_msg.pose.position.x)
    ydata.append(loc_msg.pose.position.y)
    print(loc_msg)

def run(niter=100, doblit=False):
    """
    Display the simulation using matplotlib, optionally using blit for speed
    """
    fig, ax = plt.subplots(1, 1)
    if carla_sim:
        pose = rospy.wait_for_message("/carla/hero/odometry", Odometry, timeout=10)
        xdata.append(pose.pose.pose.position.x)
        ydata.append(pose.pose.pose.position.y)
    else:
        loc_msg = rospy.wait_for_message("/mavros/local_position/pose", PoseStamped,timeout=10)
        xdata.append(loc_msg.pose.position.x)
        ydata.append(loc_msg.pose.position.y)
    if doblit:
        # cache the background
        background = fig.canvas.copy_from_bbox(ax.bbox)
    for ii in xrange(niter):
        # update the xy data
        #x, y = rw.next()
        if carla_sim:
            pose = rospy.wait_for_message("/carla/hero/odometry", Odometry)
            xdata.append(pose.pose.pose.position.x)
            ydata.append(pose.pose.pose.position.y)
        else:
            loc_msg = rospy.wait_for_message("/mavros/local_position/pose", PoseStamped)
            xdata.append(loc_msg.pose.position.x)
            ydata.append(loc_msg.pose.position.y)

        print(ydata[ii])
        # points.set_data(xdata, ydata)
        if doblit:
            # restore background
            #fig.canvas.restore_region(background)
            # redraw just the points
            ax.draw_artist(points)
            # fill in the axes rectangle
            fig.canvas.blit(ax.bbox)
        else:
            # redraw everything
            plt.cla()
            plt.axis([xdata[len(xdata)-1] - 60, xdata[len(xdata)-1] + 60, ydata[len(ydata)-1] - 60, ydata[len(ydata)-1] + 60])
            fig.canvas.draw()
            plt.xticks(np.arange(min(xdata), max(xdata),((max(xdata) - min(xdata))/(len(xdata))  +1)))
            plt.yticks(np.arange(min(ydata), max(ydata),((max(ydata) - min(ydata))/(len(ydata))  +1)))
            plt.xlabel('x(m)')
            plt.ylabel('y(m)')
            plt.grid(True)
            fig.canvas.draw()
            ax.plot(xdata,ydata, 'o', color = 'tab:blue', markersize = 4)
            ax.plot(xref,yref, '*', color = 'tab:red', markersize = 4)
            ax.hold(True)
            plt.draw()
            plt.pause(0.001)
    plt.close(fig)
if __name__ == '__main__':
    run(niter = 1000,doblit=False)
    # run(doblit=True)
