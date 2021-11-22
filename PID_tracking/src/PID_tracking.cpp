#include <iostream>
// #include <stdlib.h>
#include <vector>
#include <string>
#include <fstream>
#include <string>
// #include <stdlib.h>
#include <ctime>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <chrono>
#include <cmath>
#define PI 3.141592

// Global Variables
// #TESTING : UNCOMMENT THESE LINES FOR TESTING
// #DEMO : UNCOMMENT THESE LINES FOR DEMO
// Prue Pursuit Parameters
double dt = 0.05; //[s] carla simulation time step
float target_speed = 20;
float ts = 0.0;
double ts_carla = 0.0;
float WB = 3.0046389109377785;
float grid_cell_size = 0.1;
// state callback updates
float velocity = 0;
float curr_x = 0;
float curr_y = 0;
float curr_yaw = 0;
int counter = 0;
bool testing = false; // #TESTING

std::vector<std::vector<float> > traj; // traj global variable that stores ros updates
std::vector<std::pair<float,float>> errors;
std::vector<std::pair<float,float>> error_pos;
std::vector<std::pair<float,float>> sps;
std::vector<std::pair<float,float>> data_log;
std::vector<std::pair<float,float>> pos;
std::vector<std::pair<float,float>> ref_model;
std::vector<std::pair<float,float>> time_stamps;
std::vector<std::pair<float,float>> cascade_inputs;
 
// std::vector<std::vector<float> > test = {{394.78558247,-345.38634753},
//  {391.23659525,-343.72815595},
//  {387.70622925,-342.05766194},
//  {385.72655447,-341.18204983},
//  {385.53863713,-341.10570801},
//  {385.11994773,-340.91837292},
//  {384.59974594,-340.72708375},
//  {384.3585656 ,-340.61665421},
//  {383.85084263,-340.422195  },
//  {383.57170471,-340.33247951},
//  {383.13920904,-340.16054302},
//  {382.65382413,-339.69737146},
//  {382.69059084,-339.066},
//  {382.7020888 ,-338.29980789},
//  {382.74317357,-337.91036392},
//  {382.76501557,-337.21687995},
//  {382.80239063,-336.40163842},
//  {382.80212624,-335.66804712},
//  {382.84753956,-334.76768282},
//  {382.87535116,-334.46645446},
//  {382.89807593,-334.14180124},
//  {382.90583839,-333.71785824},
//  {382.92143183,-332.99349619},
//  {382.96421114,-332.44492644},
//  {383.01193585,-331.92619948},
//  {383.02001976,-331.52309267},
//  {382.98528725,-330.99981098},
//  {382.99326534,-330.58573882},
//  {310,-270},
//  {236.5,-65.9}};

// std::vector<std::vector<float> > test= {{19.4,-210.4},{83,-90},{101.5,-265.2}}; /*Right U turn*/
std::vector<std::vector<float> > test= {{19.4,-210.4},{83,-90}}; /*Left U turn  ,{-10.5,-70.2}        */

/*          Hardware Waypoints for testing*/
// std::vector<std::vector<float> > test= {{0.04242,0.2279}, {-43.2442,-28.1193}}; /*Straight Line 1 turn*/

// Struct definition
struct PID_output{
    public:
    float output = 0;
    float error_sum = 0;
    float error = 0;
};


// Classes definition
class state{
    public:
    float x;
    float y;
    float yaw;
    float v;
    float t;
    /*      VALUE ASSIGNMENT CONSTRUCTOR      */
    state(float a, float b, float c, float d, float e){
        x   =   a;
        y   =   b;
        yaw =   c;
        v   =   d;
        t   =   e;
    }
    /*      INITIALISATION CONSTRUCTOR        */
    state(){
        x   =   0;
        y   =   0;
        yaw =   0;
        v   =   0;
        t   =   0;
    }
};

std::vector<state> traj1;


class TwistToVehicleControl{
    public:
    float max_steering_angle = 1.22173035145;
    // float max_steering_angle = (90) * PI / 180.0; /*       Hardware values         */
    float max_acceleration = 10.0; /*            10 FOR CARLA        */
    // void messagePublisher(float velocity_error,float linear_acc, float angle, ros::NodeHandle nodeh);
};

// void TwistToVehicleControl::messagePublisher(float velocity_error, float linear_acc, float angle, ros::NodeHandle nodeh){
//     carla_msgs::CarlaEgoVehicleControl control;
//     float throttle_val = std::min(max_acceleration, linear_acc)/max_acceleration;
//     // if(velocity_error>0){
//     if(throttle_val < 0){
//         throttle_val = 0;
//     }
//     control.throttle = throttle_val;

//     if(angle >0){
//         control.steer = std::min(max_steering_angle, angle)/max_steering_angle;
//     }
//     else{
//         control.steer = std::max(-1*max_steering_angle, angle)/max_steering_angle;
//     }
//     ros::Publisher carla_command;
//     carla_command = nodeh.advertise<carla_msgs::CarlaEgoVehicleControl>("carla/hero/vehicle_control_cmd", 1);

//     while (carla_command.getNumSubscribers() < 1) {
//         ros::WallDuration sleep_t(1);    
//         sleep_t.sleep();
//     }
//     // ros::WallDuration sleep_t(0.01);    
//     // sleep_t.sleep();
//     if(carla_command.getNumSubscribers() >= 1){
//         control.header.seq = ::counter;
//         ::counter++;
//         carla_command.publish(control);
//         std::cout << "\n" << carla_command.getNumSubscribers() << std::endl;
//     }
//     // else {
//     //     ros::WallDuration sleep_t(1);
//     //     sleep_t.sleep();
//     // }
// }

// Function declaration
int search_index(std::vector<state> trajectory, float ts);
void state_callback(const geometry_msgs::Pose::ConstPtr& msg);
void callback(const geometry_msgs::PoseArray::ConstPtr& msg);
void coordinateTransform(std::vector<std::vector<float> > grid_trajectory, float orientation_car, std::vector<float> position_car, float grid_size);
void write_csv(std::vector<std::pair<float,float>> dataset, std::string filename);
double angle_unwrap(double theta);
double angle_wrap(double theta);
PID_output PID_controller(float Kp, float Ki, float Kd, float setPoint, float stateValue, float prev_error_sum, float prev_error, float saturation);
PID_output PID_controller_yaw(float Kp, float Ki, float Kd, float setPoint, float stateValue, float prev_error_sum, float prev_error, float saturation);
PID_output Cascade_PID_controller(float Kp, float Ki, float Kd, float error, float stateValue, float prev_error_sum, float prev_error, float saturation);
std::vector<state> testingCallback(std::vector<std::vector<float> > trajectory, float ts);
std::vector<state> time_association_wstate(std::vector<state> trajectoryRRTstar, double timestep);
state reference_model(state prev_state, float vref, float deltaT, float thetaM, float ini_vel);
// test main
// int main(int argc, char** argv){
//     ros::init(argc, argv, "PID_tracking_node");
//     ros::NodeHandle n;
// 	// ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
//     // Two subscribers 
//     ros::Subscriber car_state =  n.subscribe("/car_heading", 1, state_callback);
//     ros::Subscriber trajectory_sub = n.subscribe("/PlannerTraj", 1, callback);
//     ros::Rate rate(10); 
//     ros::Publisher carla_command;
//     carla_msgs::CarlaEgoVehicleControl control;
//     // control.header.frame_id = "PID_tracking_node";
//     control.throttle = 0.4;
//     control.steer = 0.1;
//     carla_command = n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 1);
//     // world.tick
//     while (carla_command.getNumSubscribers() < 1) {
//         ros::WallDuration sleep_t(0.5);    
//         sleep_t.sleep();
//     }
//     carla_command.publish(control);     
//     while(::counter < 100){
//         if(carla_command.getNumSubscribers() > 0){
//             std::cout << carla_command.getTopic() << std::endl;
//             ros::spinOnce(); 
//             control.header.seq = ::counter;
//             rate.sleep();
//             // if(::traj1.size() > 0)
//             std::cout << ::traj1.size() << std::endl;
//             std::cout << control << std::endl;
//             ::traj1.clear();
//             ::traj.clear();
//             ::counter++;
//         }
//     }   
//     return 0;
// }


// Main
// Main
int main(int argc, char** argv){
    ros::init(argc, argv, "PID_tracking_node");
    ros::NodeHandle n;
    bool testing = false; // #TESTING
    std::cout << "hellp";
    // timer init
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
    float check =::curr_x;  
    unsigned int ini_time = 0;
    ros::Rate r(20);
    ros::Subscriber car_state =  n.subscribe("/car_heading", 1, state_callback);

    while(check == ::curr_x){
        ros::spinOnce();
        // std::cout << "here\n";
        ros::Time::now();
        ::ts = ros::Time::now().toSec();
        ini_time = ::ts;
        // std::cout << ::ts;
        // std::cout << ::curr_x << std::endl;
        r.sleep();
    }
    

    //Car state subscriber
    // if(!testing)
    ros::Subscriber trajectory_sub = n.subscribe("/PlannerTraj", 1, callback); // #DEMO
    if(!testing){
        // RRT* trajectory subscriber
        // spin once to scan for first traj
        std::cout << ::traj1.size();
        while(::traj1.size() == 0){
            ros::spinOnce();
        }
    }
    else{
        ::traj1 = testingCallback(::test, ::ts);
        std::cout << "testing traj";
    }
    // 
    PID_output PID_velocity;
    PID_velocity.error = 0;
    PID_velocity.error_sum = 0;
    PID_output PID_throttle;
    PID_throttle.error = 0;
    PID_throttle.error_sum = 0;
    PID_output PID_yaw;
    PID_yaw.error = 0;
    PID_yaw.error_sum = 0;

    ros::Publisher PID1;
    ros::Publisher PID2;
    std_msgs::Float64 PID_1, PID_2;
    PID1 = n.advertise<std_msgs::Float64>("/PID_throttle", 1);
    PID2 = n.advertise<std_msgs::Float64>("/PID_steer", 1);
    
    // controller start time 
    state prev_state = state(::traj1[0].x, ::traj1[0].y, ::curr_yaw, ::velocity, ::ts_carla);
    double prev_loop_time = ::ts_carla;
    ini_time = prev_loop_time;
    double prev_time = prev_loop_time;
    double ini_vel = ::velocity;
    double prev_dt = 0.05;
    // && ::ts_carla - ini_time < 20 // #TESTING
    while(std::hypot(::traj1[::traj1.size() - 1].x - curr_x, ::traj1[::traj1.size() - 1].y - curr_y) > 0.1){
        ::ts = ros::Time::now().toSec();
        auto t5 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int ind = search_index(::traj1, ::ts);
        /*                                      REFERENCE POSITION AND VELOCITY CODE                                                */
        float thetaM = std::atan2((::traj1[1].y) - (::traj1[0].y),(::traj1[1].x) - (::traj1[0].x));
        std::cout << "::ts_carla " << prev_state.x << " " << prev_loop_time << std::endl;

        /*                                     DEFINING DT EVERYTIME YOU POLL SENSOR DATA                                           */    
        ::dt = (::ts_carla - prev_loop_time); // #CASCADED
        if(::dt == 0 ) ::dt = prev_dt; // #CASCADED
        else prev_dt = ::dt; // #CASCADED
        
        
        state ref_state = reference_model(prev_state, ::target_speed, ::ts_carla - ini_time, thetaM, ini_vel);
        TwistToVehicleControl throttle_cmd;
        // float Kp_vel_cascade = 0.1277; 1.3 , 0.2
        float Kp_vel_cascade = 1.5;
        float Ki_vel_cascade = 0.2;
        float Kd_vel_cascade = 0;
        float Kp_throttle = 2;
        float Ki_throttle = 0.0; //05
        float Kd_throttle = 0.0;
        // constant velocity    tracking/
        // Cascade controller for 
        float err_x = (ref_state.x) - ::curr_x;
        float err_y = (ref_state.y) - ::curr_y;
        float err_netx = (::traj1[::traj1.size()-1].x - ::traj1[0].x);
        float err_nety = (::traj1[::traj1.size()-1].y - ::traj1[0].y);
        float lookahead = 2;
        float error_func = (std::sqrt(std::pow(err_y,2) + std::pow(err_x,2)))  - lookahead;
        PID_velocity = Cascade_PID_controller(Kp_vel_cascade, Ki_vel_cascade, Kd_vel_cascade, error_func, ::velocity, PID_velocity.error_sum, PID_velocity.error,throttle_cmd.max_acceleration); 
        // std::cout << PID_throttle.output << std::endl;

        //variable velocity       tracking/
        // std::cout << "Ref vel " << ref_state.v << std::endl;
        PID_throttle = PID_controller(Kp_throttle, Ki_throttle, Kd_throttle, ::target_speed, ::velocity, PID_throttle.error_sum, PID_throttle.error, throttle_cmd.max_acceleration); // #TESTING replace ::target_speed with PID_velocity.output

        float target_heading = std::atan2((::traj1[ind].y) - ::curr_y,(::traj1[ind].x) - ::curr_x);  // #DEMO
        // float target_heading = std::atan2((ref_state.y) - ::curr_y,(ref_state.x) - ::curr_x);    // #TESTING CASCADED CONTROLLER
        // std::cout << target_heading - ::curr_yaw << std::endl; 5,0,0.0000099
     	// float Kp_yaw = 0.5;//2
        // float Ki_yaw = 0.000000;
        // float Kd_yaw = 0.00000;

        float Kp_yaw = 1.2;
        float Ki_yaw = 0.0001;
        float Kd_yaw = 0.0;


        PID_yaw = PID_controller_yaw(Kp_yaw, Ki_yaw, Kd_yaw, target_heading, ::curr_yaw, PID_yaw.error_sum, PID_yaw.error, throttle_cmd.max_steering_angle);

        // float steer = std::atan2(::WB * PID_yaw.output,::velocity/3.6); //TODO
        float steer = PID_yaw.output;
        // steer = 0;
        // PID_throttle.output = 0;
        // for(int p = 0; p < ::traj1.size(); p++){
        //     std::cout << "y: " << ::traj1[p].y << " x: " << ::traj1[p].x << " t: " << ::traj1[p].t << std::endl;
        // }

        errors.push_back(std::pair<float,float>{PID_velocity.output - ::velocity, target_heading - ::curr_yaw});
        error_pos.push_back(std::pair<float,float>{ref_state.x - ::curr_x, ref_state.y - ::curr_y});
        // sps.push_back(std::pair<float,float>{::target_speed,target_heading}); //TODO add log for control actions
        sps.push_back(std::pair<float,float>{PID_velocity.output, target_heading});
        data_log.push_back(std::pair<float,float>{::velocity, ::curr_yaw});
        ref_model.push_back(std::pair<float,float>{ref_state.x, ref_state.y});
        time_stamps.push_back(std::pair<float,float>{::ts_carla-ini_time, ref_state.v});
        cascade_inputs.push_back(std::pair<float,float>{PID_velocity.error_sum,PID_velocity.output});

        PID_1.data = PID_throttle.output;
        PID_2.data = steer;
        PID1.publish(PID_1);
        PID2.publish(PID_2);
        pos.push_back(std::pair<float, float>{::curr_x,::curr_y});
        // throttle_cmd.messagePublisher(PID_throttle.error, PID_throttle.output, steer, n);

        geometry_msgs::PoseArray solutionPath;
        // TRAJECTORY UPDATE FOR #DEMO PURPOSE
        if(!testing){
            // if(std::hypot(::traj1[::traj1.size() - 1].x - curr_x, ::traj1[::traj1.size() - 1].y - curr_y) > 2.5){
            if(ros::Time::now().toSec() - prev_time > 0.5){
                ROS_INFO("Old Waypoints ended");
                std::vector<state> temp_traj = ::traj1; 
                // updating the trajectory
                    ::traj1.clear();
                    ::traj.clear();
                    float temp_t = ros::Time::now().toSec();
                    while(::traj1.size() == 0){
                        if(ros::Time::now().toSec() - temp_t > 0.3){
                            ::traj1 = temp_traj;
                            // std::cout <<"print";
                            break;
                        }
                        ros::spinOnce();
                        // rate.sleep();
                    }
                    ROS_INFO("New Waypoints recieved");
                    // std::cout << ::traj1.size() << std::endl;
                    prev_time = ros::Time::now().toSec();
            }   
        }
        // updating current state vector while keeping the trajectory same
        std::vector<state> temp = ::traj1; 

        // for (std::size_t i = 0 ; i < ::traj1.size(); ++i){
        //     const int w = ::traj1[i].x;
        //     const int h = ::traj1[i].y;
        //     geometry_msgs::Pose temp;
        //     temp.position.x = w;
        //     temp.position.y = h;
        //     temp.position.z = 0;

        //     temp.orientation.x = 0;
        //     temp.orientation.y = 0;
        //     temp.orientation.z = ::traj1[i].yaw;
        //     temp.orientation.w = 0;
        //     solutionPath.poses.push_back(temp);
        // }



        // ros::Publisher PID_traj;
        // PID_traj = n.advertise<geometry_msgs::PoseArray>("/PlannerTraj_PID", 1);
        // PID_traj.publish(solutionPath);

        if(!testing){
            ::traj.clear();
            ::traj1.clear();
        }
        prev_loop_time = ::ts_carla;
        prev_state = ref_state;
        ros::spinOnce(); 
        r.sleep();
        if(!testing){
            ::traj1 = temp;
        }        
        auto t6 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // std::cout <<"single while loop time in milliseconds "<< t6 - t5 << std::endl;
    }
    PID_1.data = 0;
    PID_2.data = 0;
    PID1.publish(PID_1);
    PID2.publish(PID_2);
    std::string f1 = "errors.csv";
    std::string f2 = "set_points.csv";
    std::string f3 = "state_data.csv";
    std::string f4 = "car_position.csv";
    std::string f5 = "reference_model.csv";
    std::string f6 = "time_stamps.csv";
    std::string f7 = "errors_pos.csv";
    std::string f8 = "cascade_inputs.csv";
    write_csv(errors, f1);
    write_csv(sps,f2);
    write_csv(data_log,f3);
    write_csv(pos,f4);
    write_csv(ref_model,f5);
    write_csv(time_stamps,f6);
    write_csv(error_pos,f7);
    write_csv(cascade_inputs,f8);
    ros::spinOnce();


    return 0;
}

void write_csv(std::vector<std::pair<float,float>> dataset, std::string filename){
    std::ofstream myfile(filename);
    for(int i = 0; i < dataset.size(); ++i)
    {
        myfile << dataset.at(i).first << "," << dataset.at(i).second<<std::endl;
        // myFile << "\n";
    }

}


void state_callback(const geometry_msgs::Pose::ConstPtr& msg){
    ::curr_x = msg->position.x;
    ::curr_y = msg->position.y;
    ::curr_yaw = msg->orientation.z;
    ::velocity = msg->orientation.w;
    ::ts_carla = msg->orientation.x + ((msg->orientation.y)/(std::pow(10,9)));
}

int search_index(std::vector<state> trajectory, float currTime){
    if(trajectory.size() == NULL){
        // std::cout << "error at time: " << ts << " " << trajectory.size() << std::endl; 
    }
    // update desired set point based on time
    for(int i = 0; i < trajectory.size()-1; i++){
        if(trajectory[i].t - currTime > 0){
            // std::cout << "in " << trajectory.size() << std::endl; 
            return i+1;
        }
    }
    return trajectory.size() - 1;
}

std::vector<state> time_association_wstate(std::vector<state> trajectoryRRTstar, double timestep){
    trajectoryRRTstar[0].t = timestep;
    int i = 1;
    std::vector<state> return_val;
    return_val.push_back(trajectoryRRTstar[0]);

    while(i < int(trajectoryRRTstar.size())){
        float interp_step = 1;
        if(::testing) interp_step = 1;
        float interp = interp_step;
        // updating the x_val as the first point will be taken as it is.
        while(interp <= 1){
            state temp1;
			float dx = interp*trajectoryRRTstar[i].x + (1-interp)*trajectoryRRTstar[i-1].x;
			float dy = interp*trajectoryRRTstar[i].y + (1-interp)*trajectoryRRTstar[i-1].y;
            temp1.x = dx;
            temp1.y = dy;
            float max_dist = std::hypot(trajectoryRRTstar[0].x - trajectoryRRTstar[i+1].x, trajectoryRRTstar[0].y - trajectoryRRTstar[i+1].y);
            float normalised_frac = (std::hypot(temp1.x - trajectoryRRTstar[i+1].x, temp1.y - trajectoryRRTstar[i+1].y))/(max_dist);
            temp1.v = ::target_speed*(normalised_frac);
            temp1.t = ros::Time::now().toSec();
            // temp1.t = 0;
            return_val.push_back(temp1);
            float eudist = (std::hypot(temp1.x - return_val[return_val.size() - 2].x,temp1.y - return_val[return_val.size() - 2].y));
            if(int(::velocity) != 0){
                if(return_val.size() < 2){
                    return_val[return_val.size() - 1].t = timestep + ((eudist)/(::velocity/3.6));
                }
                else{
                    return_val[return_val.size() - 1].t = return_val[return_val.size() - 2].t + ((eudist)/(::velocity/3.6));
                }
            }
            else{
                if(return_val.size() < 2){
                    return_val[return_val.size() - 1].t = timestep + ((eudist)/(::target_speed/3.6));
                }
                else{
                    return_val[return_val.size() - 1].t = return_val[return_val.size() - 2].t + ((eudist)/(::target_speed/3.6));
                }
            }
            std::cout << "\n x: "  << return_val[return_val.size() - 1].x << " t: " << return_val[return_val.size() - 1].t << std::endl;            
            interp += interp_step;
        }
        i++;
    }
    // std::cout << ::curr_x << std::endl;
    return return_val;
}

PID_output PID_controller(float Kp, float Ki, float Kd, float setPoint, float stateValue, float prev_error_sum, float prev_error, float saturation){

    float error = setPoint - stateValue;
    float error_sum = prev_error_sum + error*::dt; //TODO
    float error_der = float((error - prev_error)/::dt);

    float a = Kp*error + Ki*error_sum + Kd*error_der;
    if(a > saturation){
        error_sum = prev_error_sum;
    }

    a = Kp*error + Ki*error_sum + Kd*error_der;
    if(a < 0){
        a = 0;
    }
    else{
        a = std::min(a,saturation);                     /*For car hardware send velocity*/
        // a = std::min(a,saturation)/saturation;          /*For Carla simulations*/
    }

    PID_output ans;
    ans.output = a;
    ans.error = error;
    ans.error_sum = error_sum;

    return ans;
}

PID_output PID_controller_yaw(float Kp, float Ki, float Kd, float setPoint, float stateValue, float prev_error_sum, float prev_error, float saturation){

    float error = angle_wrap(setPoint - stateValue);
    // float error = setPoint - stateValue;
    float error_sum = prev_error_sum + error;
    float error_der = float((error - prev_error));

    float a = Kp*error + Ki*error_sum + Kd*error_der;
    // std::abs(error) < 0.1 || 
    if(std::abs(a) > saturation || std::abs(error) < 0.1){
        error_sum = 0;
    }
    a = Kp*error + Ki*error_sum + Kd*error_der;
    //a = (90-::rad2deg(a)) //Changes during testing
    if(a < 0){
        a = std::max(a,-1*saturation)/saturation;       /*For Carla simulation*/   
    }
    else{
        a = std::min(a,saturation)/saturation;
    }
    PID_output ans;
    ans.output = a;
    ans.error = error;
    ans.error_sum = error_sum;

    return ans;
}

PID_output Cascade_PID_controller(float Kp, float Ki, float Kd, float error, float stateValue, float prev_error_sum, float prev_error, float saturation){

    float error_sum = prev_error_sum + error*::dt;
    float error_der = float((error - prev_error)/dt);

    float a = Kp*error + Ki*error_sum + Kd*error_der;
    if(a > saturation){
        error_sum = prev_error_sum;
    }

    a = Kp*error + Ki*error_sum + Kd*error_der;
    if(a < 0){
        a = 0;
    }
    else{
        // a = std::min(a,saturation);                     /*For car hardware send velocity*/
        // a = std::min(a,saturation)/saturation;          /*For Carla simulations*/
    }
    PID_output ans;
    ans.output = a;
    ans.error = error;
    ans.error_sum = error_sum;

    return ans;
}

void callback(const geometry_msgs::PoseArray::ConstPtr& msg){

    float car_pos_x = msg->poses[0].position.x;
    float car_pos_y = msg->poses[0].position.y;
    float car_orientation = msg->poses[0].orientation.z;
    std::vector<float> initial_pos = {car_pos_x, car_pos_y};

    for(int i = 1; i < msg->poses.size(); i++){
        std::vector<float> temp = {msg->poses[i].position.x, msg->poses[i].position.y};
        traj.push_back(temp);
    }
    state temp;
    temp.x = ::curr_x;
    temp.y = ::curr_y;
    ::traj1.push_back(temp);
    try{
        //converts to global coord  inates and puts the values in traj1
        coordinateTransform(traj, car_orientation, initial_pos, 0.1);
    }
    catch(...){
        std::cout << "co-ordinate transform failed" << std::endl;
    }
    ::traj1 = time_association_wstate(::traj1, ::ts);
}


std::vector<state> testingCallback(std::vector<std::vector<float> > trajectory, float ts){
    std::vector<state> return_traj; 
    for(int i = 0; i < trajectory.size(); i++){
        state temp;
        temp.x = trajectory[i][0];
        temp.y = trajectory[i][1];
        temp.t = ts;
        return_traj.push_back(temp);
    }

    return_traj = time_association_wstate(return_traj, ts);
    return return_traj;
}


void coordinateTransform(std::vector<std::vector<float> > grid_trajectory, float orientation_car, std::vector<float> position_car, float grid_size){
    // std::cout << orientation_car << std::endl;
    // orientation_car = angle_unwrap(orientation_car);
    orientation_car = (orientation_car) - float(3.1412/2);
    
    for(int i = 0; i < grid_trajectory.size(); i++){
        grid_trajectory[i][0] = grid_trajectory[i][0]*::grid_cell_size;
        grid_trajectory[i][1] = grid_trajectory[i][1]*::grid_cell_size;

        // translating trajectory of car back to grid's origin
        grid_trajectory[i][0] = grid_trajectory[i][0] - 25.1;
        grid_trajectory[i][1] = grid_trajectory[i][1] - 25.1;

        // std::swap(grid_trajectory[i][0], grid_trajectory[i][1]);

        float x = grid_trajectory[i][0];
        float y = grid_trajectory[i][1];

        // Invertung the grid's yaw(is oriented according to the car) back to carla'a co-ordinate system

        grid_trajectory[i][0] = x*(cos(orientation_car)) - y*(sin(orientation_car));
        grid_trajectory[i][1] = x*(sin(orientation_car)) + y*(cos(orientation_car));


        // As both Grid's and carla's coordinate systems are now located at 0,0 of carla, we will position the grid's coordinate system back to carla's ego vehicle.
        // positioning grid's coordinate system at carla ego vehicle w.r.t. carla'a origin 

        grid_trajectory[i][0] = grid_trajectory[i][0] + position_car[0];
        grid_trajectory[i][1] = grid_trajectory[i][1] + position_car[1];
        // #TODO HERE ADD TRANSLATION CORRESPONDING TO LIDAR
        // std::cout << "x: " << grid_trajectory[i][0] << " y: " << grid_trajectory[i][1] << std::endl;
    }
    // assigning variable to global trajectory
    for(int i = 0; i < grid_trajectory.size(); i++){
        state temp;
        temp.x = grid_trajectory[i][0];
        temp.y = grid_trajectory[i][1];
        ::traj1.push_back(temp);
    }
}


double angle_unwrap(double theta){
    /*
    going from (-180,180) to (0,360)
    */
    if(theta < 0)
    {
        theta = 2*PI + theta;
    }
    return theta;
}

double angle_wrap(double theta){
    /*
    going from (-360,360) to (-180,180)
    */
   if(theta > PI){
       return -1*(2*PI-theta);
   }
   else if(theta < -PI){
       return -1*(-2*PI-theta);
   }
   return theta;
}


state reference_model(state prev_state, float vref, float deltaT, float thetaM, float ini_vel){
    state ans;
    float p = 1;
    ans.v = prev_state.v*(std::exp(-1*::dt*p)) + (vref)*(1-std::exp(-1*::dt*p));
    std::cout << "::dt  " << ::dt << std::endl;
    ans.x = prev_state.x    +   (prev_state.v/3.6)*(::dt)*(std::cos(thetaM));
    ans.y = prev_state.y    +   (prev_state.v/3.6)*(::dt)*(std::sin(thetaM));
    // std::cout << "reference x: " << ans.x << " reference y: " << ans.y << std::endl;
    return ans;
}