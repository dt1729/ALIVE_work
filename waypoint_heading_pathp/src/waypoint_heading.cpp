#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <iostream>
#include <vector> 
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdafx.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <interpolation.h> // alglib library
#include <string>
#define PI 3.141592
using namespace std;
using namespace alglib;
ros::Publisher pub_des_vel; // Desired Velocity publisher subs - pixhawkctrlNode.cc
ros::Publisher pub_heading; // car heading pub subs - matrixnode.cpp
ros::Publisher pub_OmplStatus; // reads trajectory file and sets the OmplStatus accordingly
std_msgs::Float64 heading_msg;

int previous_vel = -1000; // arbitrarily large negative value
double prev_dist = INFINITY;
double max_vel = 15; // max velocity that the car can travel
double min_vel = 5;
double min_dist_bearing = 0.1; // min bearing update distance
double min_dist_goal = 15; // min distance to between waypoint and car to consider it as reached goal
double current_gps_bearing = 0.0; 
double goal_gps_bearng = 0.0; // bearing between goal and current pos
double current_position[2] = {0,0};
double prev_position[2] = {0,0};
double prev_prev_position[2] = {0,0};
double current_goal[2] = {0,0};
double heading = 0.0;
int flag = 0; // flag for initialising first time in gps_callback 
bool check = false; // initialising spline based waypoint for the first time.
// vector<double> waypoints[2]; // waypoints vector [0] contains x and [1] contains y coordinates
int curr_goal_index = 0;  // current goal index in the waypoints file
int navigation_status = 0; // not navigating

double curr_time; // to store the time at the event of calling e2o_callback 
double last_time; // last time e2o_callback was called
double set_distance = 0;  // distance between the obstacle and the car in order to stop. Upates every timestep
double dist_obst = 200; // distance set initially
int mode = 0; // (Yet to implement). 0 - contant vel mode. 1 - obstacle mode. 2 - turning mode. (0 & 2 are merged now)
int desired_velocity = max_vel; 
double dist_obs_car = 200; //?
double curr_vel = 1; // Current vel from e2o callback
double deceleration = -0.3; // Initial deceleration for stopping
int smooth_size = 5;  // Heading difference between car and goal smooth window size
vector<double> heading_vect;  // Heading vector storing prev heading differences
vector<double> goal = {0,0};
vector<vector<double> > spline_print = {};
double sum_threshold = 40; // Threshold to detect turn
double sum_delta = 0; // unused
double ratio = 0; // unused
int scale_factor = 2; // unuased


std::vector<std::vector<double> > waypoints;
std::vector<std::pair<float,float> > waypoints_chosen;
// TODO
void gps_callback(const geometry_msgs::Pose::ConstPtr& msg); // use nav_msgs::Odometry for local
void write_csv(std::vector<std::pair<float,float>> dataset, std::string filename);
double heading_shift(double current_gps_bearing, double goal_gps_bearng);
double angle_unwrap(double theta);
double angle_wrap(double theta);
double distance(double pos1_x, double pos1_y, double pos2_x, double pos2_y);
double bearing(double pos1_x, double pos1_y, double pos2_x, double pos2_y);
double angle_circle_wrap(double angle);
double rad2deg(double angle_in_rad);
int index_update(int prev_index, double x, double y);
string vect2string(vector<vector<double> > waypoint, int coordinate);
vector<vector<double> > forward_transform(vector<vector<double> > waypoint, double car_yaw, vector<vector<double> > car_pos);
vector<double> inverse_transform(vector<double> point, double car_yaw, vector<vector<double> > car_pos);
vector<double> spline_fit_goal(int index,const geometry_msgs::Pose::ConstPtr& msg, vector<vector<double> > waypoint, double current_position[2]);
vector<vector<double> > print_inverse_transform(vector<vector<double> > point, double car_yaw, double current_position[2]);
double points_in_lookahead(vector<vector<double> > gps_coords, double look_ahead_distance);

int main(int argc, char** argv){
<<<<<<< Updated upstream
  std::ifstream file_x("../../waypoints/carla/waypoints_2_x.txt");
  std::ifstream file_y("../../waypoints/carla/waypoints_2_y.txt");

  std::string data_x;
  std::string data_y;
=======
  // std::ifstream file_x("/home/komalb/Downloads/ControlFiles/waypoints_testing/waypoints_1_x.txt");
  // std::ifstream file_y("/home/komalb/Downloads/ControlFiles/waypoints_testing/waypoints_1_y.txt");
  std::ifstream file_x("/home/komalb/Downloads/alive/waypoints/carla/waypoints_2_x.txt");
  std::ifstream file_y("/home/komalb/Downloads/alive/waypoints/carla/waypoints_2_y.txt");
  
  std::string data;
>>>>>>> Stashed changes
  double val;
  waypoints.clear();
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  
  while(getline(file_x, data_x) && getline(file_y, data_y))
  {
    val = std::stod(data_x);
    waypoints_x.emplace_back(val);
    val = std::stod(data_y);
    waypoints_y.emplace_back(val);
  }
  file_x.close();
  file_y.close();

  waypoints.emplace_back(waypoints_x);
  waypoints.emplace_back(waypoints_y);

  ros::init(argc, argv, "waypoint_heading");   
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  ros::Subscriber gps_sub = n.subscribe<geometry_msgs::Pose>("/car_heading", 1, gps_callback, noDelay); //#TODO for nuc replace this
  std::cout << "size" << waypoints[0].size() << std::endl;
  pub_heading = n.advertise<std_msgs::Float64>("/car_heading_d", 1);
  // ros::Rate rate(5);
  double ts = ros::Time::now().toSec();
  while(ros::ok()){

      ros::spinOnce();
      // ROS_INFO("Publishing Waypoints");
      // rate.sleep();
      // if(ros::Time::now().toSec() - ts > 100){
      //   break;
      // }
  }
  std::string a = "waypoints_chosen.csv";
  write_csv(::waypoints_chosen, a);
  return 0;
}
/*
Returns waypoint in the form [x,x1...xn]
*/
std::string vect2string(vector<vector<double> > waypoint, int coordinate){
  vector<double> temp = waypoint[coordinate];
  std::string final_val = "[";
  for(int i = 0; i<temp.size(); i++){
    final_val += ((std::to_string(temp[i])) + ",");
  }
  final_val.replace(final_val.length()-1, final_val.length()-1, "]");
  return final_val; 
}

/*
Spline fitting using Alglib
index used to indicate the points to be skipped closest to car
*/
vector<double> spline_fit_goal(int index, const geometry_msgs::Pose::ConstPtr& msg, vector<vector<double> > waypoint, double current_position[2]){
  // Points selection
  // Distance based points extraction
  // linear scan to remove dec. x points.
  vector<double> t1 = {msg->position.x};
  t1.insert(std::end(t1), std::begin(waypoint[0]) + index, std::end(waypoint[0])); // INSERT POINTS TO THE CURRENT ARRAY FROM CURRENT GOAL INDEX TO END
  vector<double> t2 = {msg->position.y};
  t2.insert(std::end(t2), std::begin(waypoint[1]) + index, std::end(waypoint[1])); // INSERT POINTS TO THE CURRENT ARRAY FROM CURRENT GOAL INDEX TO END
  vector<vector<double> > temp0 = {t1,t2};
  std::cout << t1.size()<< std::endl;
  std::cout << "temp0: " << temp0.size() << std::endl;
  // transform incoming waypoints into car's frame
  vector<vector<double> > car_pos = {{current_position[0]},{current_position[1]}};
  vector<vector<double> > temp = forward_transform(temp0, msg->orientation.z, car_pos);

  std::string a = vect2string(temp,0);
  std::string b = vect2string(temp,1);
  std::string p1 = vect2string(waypoints,0); // ONLY TO DEBUG AND PRINT
  std::string p2 = vect2string(waypoints,1); // ONLY TO DEBUG AND PRINT
  
  char *x_pos = &a[0];
  char *y_pos = &b[0];
  
  alglib::real_1d_array x_coord = x_pos;
  alglib::real_1d_array y_coord = y_pos;
  
    // std::cout << "a: " << a << std::endl;
    // std::cout << "b: " << b << std::endl;
    // std::cout << "------------------" << endl;
    // std::cout << "x: " << p1 << std::endl;
    // std::cout << "y: " << p2 << std::endl;
  // INITIALISERS TO USE ALGLIB polynomialfit FUNCTION
  alglib::ae_int_t info;
  alglib::spline1dinterpolant s;
  alglib::barycentricinterpolant bary;
  alglib::spline1dfitreport rep;
  alglib::polynomialfitreport polrep;
  
<<<<<<< Updated upstream
  double rho = 0; //unused
  // braking distance or 25
  double look_ahead_distance = std::max(((msg->orientation.w*1.5)/3.6) + ((std::pow((msg->orientation.w/3.6),2))/(1.4*9.8)), 25.0);
=======
  double rho = 0;
  // double look_ahead_distance = 15+(msg->orientation.w/3.6) * (3);  // speed in m/s -> kmph / 3.6 total time in mapping, planning, control + actuation is assumed 4s.
  double look_ahead_distance = std::max(((msg->orientation.w*1.5)/3.6) + ((std::pow((msg->orientation.w/3.6),2))/(1.4*9.8)), 25.0);//#TODO SPARSNESS OF WAYPOINTS; LOOK INTO 25 M HARD LIMIT
  // double look_ahead_distance = std::pow((msg->orientation.w/10),2) + ((msg->orientation.w/10)*3);
  // std::cout << "speed" << msg->orientation.w << std::endl;
>>>>>>> Stashed changes
  if(msg->orientation.w == 0){
    look_ahead_distance = 25;
  }
  // look_ahead_distance = 15;

  
  // alglib::ae_int_t number_of_spline_pts = 5;
    // number_of_spline_pts = 5; rho = 0;
  try{ //unused
    // std::cout << number_of_spline_pts << endl;
    // number_of_spline_pts = std::min(double(number_of_spline_pts),double(waypoints[0].size() - index));
    // spline1dfitpenalized(x_coord, y_coord, number_of_spline_pts, 5, rho, info, s, rep);
    try{
      std::cout << "lookahead distance " << look_ahead_distance << std::endl;
      polynomialfit(x_coord, y_coord, points_in_lookahead(temp,look_ahead_distance), 2, info, bary, polrep);
      // polynomialfit(x_coord, y_coord, 4, 2, info, bary, polrep);

    }
    catch(...){
      std::cout << "poly not working " << points_in_lookahead(temp,look_ahead_distance) << std::endl;
    }
    std::cout << "here" << endl;  
  }catch(...){
    std::cout << "error at: "<< index << std::endl;
    // PRINT FITTED SPLINE/POLYNOMIAL THIS WAY
    // std::cout << to_string(info) << endl;
    // std::cout << "[";
    // for(int i = 0; i < spline_print.size(); i++){
    //   std::cout << "[" << spline_print[i][0] << "," << spline_print[i][1] << "];"<< std::endl;
    // }
    // std::cout << "];" << std::endl;
  }
  //#Dummy value initialised
  vector<double> goal_temp = {0,0};

  vector<vector<double> > position_car_coordinates = forward_transform(car_pos,msg->orientation.z,car_pos);

  goal_temp[0] = (temp[0][points_in_lookahead(temp0,look_ahead_distance)]+position_car_coordinates[0][0])/2;
  // goal_temp[1] = alglib::spline1dcalc(s,goal_temp[0]);
  goal_temp[1] = alglib::barycentriccalc(bary, goal_temp[0]); // MID POINT CALCULATION ON THE LINE
  
  goal_temp = inverse_transform(goal_temp, msg->orientation.z,car_pos);  // BRING BACK TO THE CARLA GLOBAL FRAME TO USE AS A WAYPOINT TO FIND HEADING ETC.
  std::cout << "goal x: "<< goal_temp[0] << "temp0 y: " << goal_temp[1] << std::endl;
  ::waypoints_chosen.push_back(std::pair<double,double>{goal_temp[0],goal_temp[1]});
  return goal_temp;
}

double points_in_lookahead(vector<vector<double> > gps_coords, double look_ahead_distance){
  vector<vector<double> > solution;
  int count = 0;
  for(int i = 0; i < gps_coords[0].size(); i++){
    if(distance(gps_coords[0][0],gps_coords[1][0],gps_coords[0][i],gps_coords[1][i]) < look_ahead_distance){
      count++;
    }
  }
  std::cout << "count" << count << std::endl; 
  return count;
  // sort the array in x;
}

vector<vector<double> > forward_transform(vector<vector<double> >waypoint, double car_yaw, vector<vector<double> > car_pos){
 for(int i =0; i<waypoint[0].size(); i++){
  //translation
      waypoint[0][i] = waypoint[0][i] - car_pos[0][0];
      waypoint[1][i] = waypoint[1][i] - car_pos[1][0];
	//rotation
  double x = waypoint[0][i];
  double y = waypoint[1][i];
  // flipping about y axis
	//  waypoint[0][i] = waypoint[0][i]*cos(PI);
	//  waypoint[1][i] = waypoint[1][i];

  //  rotation
    waypoint[0][i] = (x*cos(car_yaw)) - (y*sin(car_yaw));
    waypoint[1][i] = (x*sin(car_yaw)) + (y*cos(car_yaw));
	 //scaling
	 //waypoint[0][i] *= grid_cell_size;
	 //waypoint[1][i] *= grid_cell_size;

 }
 return waypoint;
}

vector<double> inverse_transform(vector<double> point, double car_yaw, vector<vector<double> > car_pos){

	// car_yaw = -1*(car_yaw - (PI/2));

  vector<double> temp = {point[0],point[1]};
  //rotation
  point[0] = (temp[0]*cos(car_yaw)) + (temp[1]*sin(car_yaw));
  point[1] = (temp[1]*cos(car_yaw)) - (temp[0]*sin(car_yaw));
  // translation
  point[0] = point[0] + car_pos[0][0];
  point[1] = point[1] + car_pos[1][0];

	return point;
}

vector<vector<double> > print_inverse_transform(vector<vector<double> > point, double car_yaw, double current_position[2]){
  for(int i = 0; i<point.size(); i++){
    vector<double> temp = {point[i][0],point[i][1]};
    point[i][0] = (temp[0]*cos(car_yaw)) + (temp[1]*sin(car_yaw));
    point[i][1] = (temp[1]*cos(car_yaw)) - (temp[0]*sin(car_yaw));

    // translation
    
    point[i][0] = point[i][0] + current_position[0];
    point[i][1] = point[i][1] + current_position[1];
  }
  return point;
}

/*
going from (-180,180) to (0,360)
*/
double angle_unwrap(double theta){

    if(theta < 0)
    {
        theta = 360 + theta;
    }
    return theta;
}

/*
going from (0,360) to (-180,180)
*/
double angle_wrap(double angle){
	if(angle > 180)
	{
		angle = angle - 360; 
	}
	else if(angle < -180)
	{
		angle = 360 + angle;
	}
	return angle;
}

double angle_circle_wrap(double angle){
  if(angle < 0){
    angle = 360 - abs(angle);
  }
  return angle;
}

/*
Calculates heading diff between goal and car
*/
double heading_shift(double current_gps_bearing, double goal_gps_bearng){
    // double delta = angle_unwrap(goal_gps_bearng) - angle_unwrap(current_gps_bearing);
    double delta = angle_wrap(angle_circle_wrap(goal_gps_bearng - current_gps_bearing));
    /*
    delta +ve is right 
    delta -ve is left
    */
    double heading_shift = delta;
    double sum=0;
    double avg_heading=0;
    heading_vect.push_back(heading_shift);
    for(int i=0;i<heading_vect.size();i++)
    { 
      sum = sum + heading_vect[i];
    }

    avg_heading = sum/heading_vect.size(); // Smoothes out heading
    if(heading_vect.size() == smooth_size)  
      heading_vect.erase(heading_vect.begin()); // Pops back first element
    // cout<<"current bearing : "<< current_gps_bearing<<"\n";
    // cout<<"goal bearing    : "<< goal_gps_bearng<<"\n";
    // if(delta > 0)
    //     cout<<"right : "<<delta<<"\n";
    // else
    //     cout<<"left  : "<<delta<<"\n";

    // cout << -1*delta << "\n";
    /*
    delta -ve is right
    delta +ve is left
    */

    return -1*delta;
    // return delta;
    // return (-1*avg_heading);
}
/*
Calculates heading diff between goal and car
*/
int index_update(int prev_index, double x, double y){
  int min_ind = prev_index; 
  double prev_min_dist = INFINITY;
  // std::cout << waypoints[0].size();
  for(int i = prev_index; i < waypoints[0].size(); i++){
    double curr_dist = distance(x,y,waypoints[0][i],waypoints[1][i]);
    std::vector<double> temp_goal = {0,0};
    temp_goal[0] = waypoints[0][i];
    
    temp_goal[1] = waypoints[1][i];
    // double temp_goal_bearing = abs(rad2deg(bearing(x, y, temp_goal[0], temp_goal[1])));
    // std::cout << "index" << i << std::endl;
    // std::cout << "heading" << rad2deg(heading) << std::endl;
    // std::cout << "bearing angle" << temp_goal_bearing << std::endl;
    // double bearing_yaw_difference = heading_shift(-1*rad2deg(heading), temp_goal_bearing);
    // std::cout << "bearing_yaw_diff" << bearing_yaw_difference << std::endl;


    float V_min = 0;
    float V_max = 50;
    float V_curr = curr_vel;

    // double cone_thresh = 90;
    // if(abs(bearing_yaw_difference) < cone_thresh){
      if(curr_dist < prev_min_dist){
        // std::cout << "bearing_yaw_diff" << bearing_yaw_difference << std::endl;
        prev_min_dist = curr_dist;
        min_ind = i;
      }
    // }
  }
  return min_ind; // #TODO
}

void gps_callback(const geometry_msgs::Pose::ConstPtr& msg) // use nav_msgs::Odometry for local
{
        // INITIALISE(1st call of this function) PREVIOUS POSITION VARIABLE(TO BE USED IN LATER LINES)
        if(flag == 0)
        {
            prev_position[0] = msg->position.x;
            prev_position[1] = msg->position.y;
            flag = 1;
            return;
        }
        heading = msg->orientation.z;
        curr_vel = msg->orientation.w;
        current_position[0] = msg->position.x;
        current_position[1] = msg->position.y;
        // SPLINE Based Waypoint update to initially(1st call of this function) generate a goal
        if(check){
          current_goal[0] = goal[0];
          current_goal[1] = goal[1];
          // std::cout << "goal_x: " << goal[0] << " goal_y: " << goal[1] << std::endl;
        }
        else{
          // #TODO call spline based waypoint update
          goal = spline_fit_goal(0,msg,waypoints,current_position);
          current_goal[0] = goal[0];
          current_goal[1] = goal[1];
          check = true;
        }
        // current_goal[0] = waypoints[0][::curr_goal_index];
        // current_goal[1] = waypoints[1][::curr_goal_index];

        //check if distance is greater than min distance only then update heading
        double dist = distance(prev_position[0], prev_position[1], current_position[0], current_position[1]);
        // ::prev_prev_position[0] = ::prev_position[0];
        // ::prev_prev_position[1] = ::prev_position[1];

        // cout<<"distance from last bearing calculation : "<<dist<<"\n";
        // Measurement update after distance
        if(dist > min_dist_bearing)
        {
          current_gps_bearing = rad2deg(bearing(prev_position[0], prev_position[1], current_position[0], current_position[1]));
          // std::cout << "current_gps_bearing->" << current_gps_bearing << endl;
          prev_position[0] = msg->position.x;
          prev_position[1] = msg->position.y;

          goal_gps_bearng = rad2deg(bearing(current_position[0], current_position[1], current_goal[0], current_goal[1]));  //# value between -180 -> 180
          
          
          if(navigation_status == 0)
          {
            cout<<"STARTED NAVIGATION";
            navigation_status = 1; // started navigating
          }
        }

        double distance_to_goal = distance(current_position[0], current_position[1], current_goal[0], current_goal[1]);

        if(navigation_status == 1)
        {
          // cout<<"distance from waypoint : "<<distance_to_goal<<"\n";
          
          if( distance_to_goal <= min_dist_goal) // trying to reach waypoint just publish heading shift
          {
            // std::cout << "Current Waypoint: x " << current_goal[0] << "Current Waypoint: y" << current_goal[1] << std::endl;
            heading_msg.data =  heading_shift(current_gps_bearing, goal_gps_bearng);
            // std::cout << "" <<  distance_to_goal << prev_dist << std::endl; 
          }
          else // reached waypoint
          {
            if(curr_goal_index>=0 && curr_goal_index < waypoints[0].size())// switch to next waypoint
            {
              heading_msg.data =  heading_shift(current_gps_bearing, goal_gps_bearng);
              ::curr_goal_index = index_update(::curr_goal_index, current_position[0], current_position[1]);
              goal = spline_fit_goal(::curr_goal_index, msg, waypoints, current_position); // STARTS SEARCH FROM CURRENT GOAL INDEX
              // goal = vector<double>{waypoints[0][::curr_goal_index], waypoints[1][::curr_goal_index]};
              current_goal[0] = goal[0];
              current_goal[1] = goal[1];
              // std::cout << "goal_x: " << goal[0] << " goal_y: " << goal[1] << std::endl;
              // #TODO SPLINE based goal selector.
              
              if(curr_goal_index == waypoints[0].size()-1) // reached final waypoint
              {
                cout<<"REACHED DESTINATION\n";
                navigation_status = 0; // not navigating
                desired_velocity = 0;
                navigation_status = 2;
                exit(1);
              }
            }
          }
          
          pub_heading.publish(heading_msg);
          // cout<<"+++++++++ Max velocity :"<<max_vel<<endl;
          // cout<<"+++++++++ Min velocity :"<<min_vel<<endl;
          // cout<<"+++++++++ Angle Sum  :  "<<sum_delta<<endl;
          // cout<<"+++++++++ Desired velocity :"<<desired_velocity<<endl;
          // cout<<"+++++++++ Ratio:"<<ratio<<endl;
          //  cout<<"-----------------------------\n";            
        } 
}   

double bearing(double pos1_x, double pos1_y, double pos2_x, double pos2_y){
    return std::atan2((pos2_y - pos1_y),(pos2_x - pos1_x));
}

double distance(double pos1_x, double pos1_y, double pos2_x, double pos2_y){
    return std::hypot(pos1_x - pos2_x, pos1_y-pos2_y);
}

double rad2deg(double angle_in_rad){
  return angle_in_rad * (180.0/3.141592653589793238463);
}
void write_csv(std::vector<std::pair<float,float>> dataset, std::string filename){
    std::ofstream myfile(filename);
    for(int i = 0; i < dataset.size(); ++i)
    {
        myfile << dataset.at(i).first << "," << dataset.at(i).second<<std::endl;
        // myFile << "\n";
    }
}
