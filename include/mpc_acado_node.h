#ifndef _MPC_acado_node_H
#define _MPC_acado_node_H


#define _USE_MATH_DEFINES
#include <memory>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <fstream>
//#include "yaml-cpp/yaml.h"

#include "nmpc_common.h"
#include "nmpc_auxiliary_functions.h"
#include <thread>
#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench.hpp"


#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "mpc_acado.h"


/* Some convenient definitions. */





using namespace std;
using namespace Eigen;
using std::placeholders::_1;




// TODO: check if veclocity is in body frame if not transform


#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include <chrono>
#include <iostream>
#include "nav_msgs/msg/odometry.hpp" 
#include "std_msgs/msg/float64_multi_array.hpp"
using namespace std::chrono;
using namespace std::chrono_literals;


//using std::placeholders::_1;

//std::unique_ptr<Mpc> mpc_ptr;
rclcpp::Time start_time;




//Eigen::MatrixXd desired_state;
//Eigen::MatrixXd current_state;
//Eigen::MatrixXd desired_control;

//Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N+1, 9);



double  Phi, Theta, Psi;
//double roll_off, pitch_off, yaw_off;
nmpc_struct_ nmpc_struct;
online_data_struct_ online_data;



std::vector<double> pos_ref;
std::vector<double> current_pos_att;
std::vector<double> point;
std::vector<double> current_vel_rate;
std::vector<double> state_feedback;
std::vector<double> reference; 
std::vector<double> gp_dist;

struct _dist_struct
{
    bool predInit;
    int print_predInit = 1;
    std::vector<double> data;
    std::vector<double> data_zeros;
} dist_Fx, dist_Fy, dist_Fz;



#endif
