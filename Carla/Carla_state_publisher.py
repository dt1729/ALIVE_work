#TODO the map has been zoomed so divide by 10 to get the correct initial points and trajectory

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import carla
from carla_msgs.msg import CarlaEgoVehicleInfo
import math



def main():

    rospy.init_node('car_heading', anonymous=False)
    rate = rospy.Rate(50)
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    try:
        ego_vehicle_info = rospy.wait_for_message("/carla/hero/vehicle_info", CarlaEgoVehicleInfo, timeout=3)
        sensor_msg_for_time = rospy.wait_for_message("/carla/hero/odometry", Odometry, timeout=3)
    except:
        print("not working")
    print(ego_vehicle_info.id)
    actor = world.get_actor(ego_vehicle_info.id)
    print(actor)
    yaw_publisher = rospy.Publisher('/car_heading', Pose, queue_size=1)
    # odom_publisher = rospy.Publisher('/car_odometry', Pose, queue_size=1)
    yaw_msg = Pose()

    # odom_msg = Pose()

    while not rospy.is_shutdown():
        try:
            sensor_msg_for_time = rospy.wait_for_message("/carla/hero/odometry", Odometry, timeout=0.1)
        except:
            pass

        v = actor.get_velocity()

        yaw_msg.position.x = actor.get_transform().location.x 
        yaw_msg.position.y = actor.get_transform().location.y 
        yaw_msg.position.z = actor.get_transform().location.z

        # yaw_msg.orientation.x = np.deg2rad(actor.get_transform().rotation.roll)
        # yaw_msg.orientation.y = np.deg2rad(actor.get_transform().rotation.pitch)
        yaw_msg.orientation.x = sensor_msg_for_time.header.stamp.secs
        yaw_msg.orientation.y = sensor_msg_for_time.header.stamp.nsecs
        yaw_msg.orientation.z = np.deg2rad(actor.get_transform().rotation.yaw)
        yaw_msg.orientation.w = (3.6)*math.sqrt(v.x**2 + v.y**2 + v.z**2)
        yaw_publisher.publish(yaw_msg)

        print("Now publishing Pose; Currently sending Carla Time instead of roll and pitch")
        rate.sleep()





if __name__ == '__main__':
    main()



