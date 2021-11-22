import math
import numpy as np
import matplotlib.pyplot as plt
import carla
from carla_msgs.msg import CarlaEgoVehicleInfo


###############################
# PID CLASS
# STATE 
# VEHICLE INFO
###############################

traj = []
target_speed = 10
curr_speed = 0
deltaT = 0.05
max_acceleration = 10.2
max_steering_angle = 1.22173035145

class state:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.t = 0
        self.yaw = 0
        self.v = 0
    def __init__(self,x_i,y_i,t_i,yaw_i,v_i):
        self.x = x_i
        self.y = y_i
        self.t = t_i
        self.yaw = yaw_i
        self.v = v_i

class PID_output:
    def __init__(self):
        self.error_sum = 0
        self.error = 0
        self.output = 0

def testing_callback(traj, ts):
    return_traj = []
    for i in (traj):
        state1 = state()
        state1.x = i[0]
        state1.y = i[1]
        state1.t = ts
        return_traj.append(state1)
    return_traj = time_association_wstate(return_traj, ts)
    return return_traj

def time_association_wstate(traj_state, time_s):
    traj_state[0].t = time_s
    interp_traj = [traj_state[0]]
    i = 0
    while i < len(traj_state):
        interp_step = 0.1
        interp = interp_step
        while interp <= 1:
            temp = state()
            dx = interp*traj_state[i].x + (1-interp)*traj_state[i-1].x
            dy = interp*traj_state[i].y + (1-interp)*traj_state[i-1].y
            temp.x,temp.y = dx,dy
            max_dist = np.hypot(traj_state[0].x - traj_state[len(traj_state)-1].x, traj_state[0].y - traj_state[len(traj_state)-1].y)
            normalised_frac = np.hypot(temp.x - traj_state[0].x, temp.y - traj_state[0].y)/max_dist
            temp.v = target_speed*normalised_frac
            interp_traj.append(temp)
            eudist = np.hypot(temp.x - interp_traj[len(interp_traj)-2].x, temp.y - interp_traj[len(interp_traj)-2].y)
            if curr_speed is not 0:
                if len(interp_traj) < 2:
                    interp_traj[len(interp_traj) - 1].t = time_s + ((eudist)/(curr_speed/3.6))
                else:
                    interp_traj[len(interp_traj) - 1].t = interp_traj[interp_traj.size() - 2].t + ((eudist)/(curr_speed/3.6))
            else:
                if len(interp_traj) < 2:
                    interp_traj[len(interp_traj) - 1].t = time_s + ((eudist)/(curr_speed/3.6))
                else:
                    interp_traj[len(interp_traj) - 1].t = interp_traj[len(interp_traj) - 2].t + ((eudist)/(target_speed/3.6))
            interp += interp_step
        i += 1 

    return interp_traj

def search_idx(trajectory, time_s):
    count = 0
    for i in trajectory:
        if i.t < time_s:
            return count
        count += 1
    return count-1

def reference_model(prev_state, vref, deltaT, theta_m, iniVel):
    p = 1
    ans = state()
    ans.v = prev_state.v*(math.exp(-1*deltaT*p)) + vref*(1-math.exp(-1*deltaT*p))
    ans.x = ans.x + (prev_state.v/3.6)*deltaT*math.cos(theta_m)
    ans.y = ans.y + (prev_state.v/3.6)*deltaT*math.sin(theta_m)
    return ans

def PID_controller(Kp, Ki, Kd, setPoint, stateValue, prev_error_sum, prev_error, saturation):
    error = setPoint - stateValue
    error_sum = prev_error_sum + error*deltaT
    error_der = float(error - prev_error)/deltaT

    a = Kp*error + Ki*error_sum + Kd*error_der
    if(a>saturation):
        error_sum = prev_error_sum
    a = Kp*error + Ki*error_sum + Kd*error_der
    if a < 0:
        a = 0
    else:
        a = min(saturation,a)
    
    ans = PID_output()
    ans.output = a
    ans.error = error
    ans.error_sum = error_sum
    return ans

def PID_controller_yaw(Kp, Ki, Kd, setPoint, stateValue, prev_error_sum, prev_error, saturation):

    error = angle_wrap(setPoint - stateValue)
    error_sum = prev_error_sum + error
    error_der = float(prev_error - error)
    a = Kp*error + Ki*error_sum + Kd*error_der

    if a>saturation or abs(error) < 0.1:
        error_sum = 0

    a = Kp*error + Ki*error_sum + Kd*error_der
    if a < 0:
        a = max(a,-1*saturation)/saturation
    else:
        a = min(a,saturation)/saturation
    
    ans = PID_output()
    ans.output = a
    ans.error = error
    ans.error_sum = error_sum
    return ans

def angle_wrap(theta):
    if theta > math.pi:
        return -1*(2*math.pi - theta)
    elif theta < -1*math.pi:
        return -1*(-2*math.pi - theta)
    return theta

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    actor = world.get_actor("hero")
    new_settings = world.get_settings()
    new_settings.synchronous_mode = True
    new_settings.fixed_delta_seconds = deltaT
    world.apply_settings(new_settings) 
    world.tick()
    v = actor.get_velocity()
    x = actor.get_transform().location.x  
    y = actor.get_transform().location.y 
    z = actor.get_transform().location.z
    yaw = actor.get_transform().orientation.z
    world.tick()
    vehicle_state = state()
    prev_state = state()
    PID_velocity = PID_output()
    PID_throttle = PID_output()
    PID_yaw = PID_output()

    #   ATTACH TIME STAMPS TO THE REFERENCE TRAJECTORY

    time_stamped_traj  = testing_callback(traj ,new_settings.fixed_delta_seconds)

    Kp_vel_cascade = 1.5
    Ki_vel_cascade = 0.2
    Kd_vel_cascade = 0
    Kp_throttle = 2
    Ki_throttle = 0.0
    Kd_throttle = 0.0
    Kp_yaw = 1.2
    Ki_yaw = 0.0001
    Kd_yaw = 0.0

    loop_count = 0
    ###################
    # Controller time initialisation
    # loop init
        # Poll position data
        # trajectory point choosing
        # track theta
        # time difference calculation
        # generate reference model with time
        # Velocity level PID
        # Throttle level PID
        # Yaw PID
        # Publish throttle and steer cmds
    ###################
    while True:
        curr_sim_time = new_settings.fixed_delta_seconds*loop_count
        vehicle_state = state(actor.get_transform().location.x ,actor.get_transform().location.y, curr_sim_time,
                        actor.get_transform().orientation.z,actor.get_velocity())
        
        idx = search_idx(time_stamped_traj, curr_sim_time)
        thetaM = math.atan2(time_stamped_traj[idx].y - time_stamped_traj[idx-1].y, time_stamped_traj[idx].x - time_stamped_traj[idx-1].x)
        ref_state = reference_model(prev_state, target_speed, new_settings.fixed_delta_seconds, thetaM, v)
        
        err_x = ref_state.x - vehicle_state.x
        err_y = ref_state.y - vehicle_state.y
        lookahead = 2
        error_func = np.hypot(err_y,err_x) - lookahead
        PID_velocity = PID_controller(Kp_vel_cascade, Ki_vel_cascade, Kd_vel_cascade, error_func, vehicle_state.v, 
                                    PID_velocity.error_sum, PID_velocity.error, max_acceleration)
        PID_throttle = PID_controller(Kp_throttle, Ki_throttle, Kd_throttle, PID_velocity.output, vehicle_state.v, 
                                    PID_throttle.error_sum, PID_throttle.error, max_acceleration)
        target_heading = math.atan2((time_stamped_traj[idx].y) - vehicle_state.y,(time_stamped_traj[idx].x) - vehicle_state.x)



        PID_yaw = PID_controller_yaw(Kp_yaw, Ki_yaw, Kd_yaw, target_heading, vehicle_state.yaw, 
                                    PID_yaw.error_sum, PID_yaw.error, max_steering_angle)

        steer = PID_yaw.output

        #############################
        # Apply PID values to the car
        # Log data and generate previous state variable for ref_state gen
        #############################

        actor.apply_control(carla.VehicleControl(throttle=PID_throttle.output, steer=steer))
        prev_state = vehicle_state
        
        loop_count += 1
        world.tick()


if __name__ == '__main__':
    main()