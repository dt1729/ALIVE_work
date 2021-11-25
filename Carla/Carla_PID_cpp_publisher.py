import rospy
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from std_msgs.msg import Float64
import sys

class TwistToVehicleControl(object):  # pylint: disable=too-few-public-methods
    """
    receive geometry_nav_msgs::Twist and publish carla_msgs::CarlaEgoVehicleControl

    use max wheel steer angle
    """

    # TODO change acc to max speed 37
    MAX_LON_ACCELERATION = 10

    def __init__(self, role_name, vehicle_info):
        """
        Constructor
        """
        if not vehicle_info.wheels:  # pylint: disable=no-member
            rospy.logerr(
                "Cannot determine max steering angle: Vehicle has no wheels.")
            sys.exit(1)

        self.max_steering_angle = vehicle_info.wheels[0].max_steer_angle  # pylint: disable=no-member
        if not self.max_steering_angle:
            rospy.logerr("Cannot determine max steering angle: Value is %s",
                         self.max_steering_angle)
            sys.exit(1)
        # rospy.loginfo("Vehicle info received. Max steering angle=%s",
        #               self.max_steering_angle)

        # rospy.Subscriber("/carla/{}/twist".format(role_name), Twist, self.twist_received)

        self.pub = rospy.Publisher("carla/hero/vehicle_control_cmd",
                                   CarlaEgoVehicleControl, tcp_nodelay=True, queue_size=1)

    def twist_received(self, linear_acc, angle):
        """
        receive twist and convert to carla vehicle control
        """
        control = CarlaEgoVehicleControl()
        control.throttle = min(TwistToVehicleControl.MAX_LON_ACCELERATION,
                                   linear_acc) / TwistToVehicleControl.MAX_LON_ACCELERATION

        if control.throttle < 0:
            control.throttle = 0

        if angle > 0:  # TODO convert to angle.
            control.steer = min(self.max_steering_angle, angle) / \
                self.max_steering_angle
        else:
            control.steer = max(-self.max_steering_angle, angle) / \
                self.max_steering_angle
        self.pub.publish(control)
        print(control)



rospy.init_node('Message_transfer_carla', anonymous=False)
ego_vehicle_info = rospy.wait_for_message("/carla/hero/vehicle_info", CarlaEgoVehicleInfo, timeout=0.4)
throttle_cmd = TwistToVehicleControl('hero',ego_vehicle_info)
while True: 
    control1 = rospy.wait_for_message("/PID_throttle",Float64,1)
    control2 = rospy.wait_for_message("/PID_steer",Float64,1)
    throttle_cmd.twist_received(control1.data, control2.data)
    print("Publishing now")



