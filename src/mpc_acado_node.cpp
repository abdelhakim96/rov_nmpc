



//int N =9;


#include "mpc_acado.cpp"
#include "mpc_acado_node.h"
#include "mpc_acado.h"


class ModelPredictiveControl : public rclcpp::Node
{

      
public:
  NMPC* nmpc_ptr_;
  //ModelPredictiveControl() : Node("model_predictive_control")
  ModelPredictiveControl(NMPC* nmpc_ptr) : Node("model_predictive_control"), nmpc_ptr_(nmpc_ptr)
  {   
        
       


    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);


    mpc_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);


     position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&ModelPredictiveControl::position_cb, this, std::placeholders::_1));

     gp_dist_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gp_pred_mu", qos, std::bind(&ModelPredictiveControl::gp_dist_cb, this, std::placeholders::_1));




    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {

      if (offboard_setpoint_counter_ == 10) {

        // Change to Offboard mode after 10 setpoints
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        this->arm();
      }
        
      publish_control_mode();

      //publish_trajectory_setpoint(); //for setpoint position control *PID*

      publish_control();
            
            //
      // stop the counter after reaching 11
      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
    };
    timer_ = this->create_wall_timer(2ms, timer_callback);
  }

  void arm();
  void disarm();
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;

private:

  void position_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void gp_dist_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;


  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gp_dist_subscriber_;



  rclcpp::Publisher<OffboardControlMode>::SharedPtr mpc_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;




  std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

  uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

  void publish_control_mode();
  void publish_trajectory_setpoint();
  void publish_control();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

};



void ModelPredictiveControl::gp_dist_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

   gp_dist ={msg->data[0],msg->data[1],msg->data[2]};

}


void ModelPredictiveControl::position_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      

    tf2::Quaternion q(
      msg->q[0],
      msg->q[1],
      msg->q[2],
      msg->q[3]);


    tf2::Matrix3x3 m(q);


    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);


    double vx     = msg->velocity[1];
    double vy     = msg->velocity[0];
    double vz     = -msg->velocity[2];
    double Phi    = yaw;
    double Theta  = pitch;
    double Psi    = roll-1.570;


    Psi = -Psi;
  

    Eigen::Matrix3d Rx, Ry, Rz;

    Rx << 1, 0, 0,
          0, cos(Psi), -sin(Psi),
          0, sin(Psi), cos(Psi);

    Ry << cos(Psi), 0, sin(Psi),
          0, 1, 0,
          -sin(Psi), 0, cos(Psi);

    Rz << cos(Psi), -sin(Psi), 0,
          sin(Psi), cos(Psi), 0,
          0, 0, 1;



    //Eigen::Matrix3d R = Rz;

    //Eigen::Matrix3d R = Rx * Ry * Rz;

    Eigen::Matrix3d R = Rz * Ry * Rx;



    //
    R = R.transpose().eval();
    //
    double r_vx = R(0, 0) * vx + R(0, 1) * vy + R(0, 2) * vz;
    double r_vy = R(1, 0) * vx + R(1, 1) * vy + R(1, 2) * vz;
    double r_vz = R(2, 0) * vx + R(2, 1) * vy + R(2, 2) * vz;



    std::cout << "x: " << msg->position[1] << "\n";
    std::cout << "y: " << msg->position[0]<< "\n";
    std::cout << "z: " << -msg->position[2]<< "\n";

    std::cout << "Yaw: " << Psi << "\n";


    

  current_states = { msg->position[1], 
                      msg->position[0],
                      -msg->position[2],
                          r_vx,
                          r_vy,
                          r_vz,
                         yaw,    //proll
                          pitch,  //pitch
                          -Psi};   //yaw



}





void ModelPredictiveControl::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void ModelPredictiveControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void ModelPredictiveControl::publish_control_mode()
{
  OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  mpc_mode_publisher_->publish(msg);
}


void ModelPredictiveControl::publish_trajectory_setpoint()
{
  TrajectorySetpoint msg{};
  msg.position = {0.0, 0.0, -3.0};
  msg.yaw = -3.14; // [-PI:PI]
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}




/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */


void ModelPredictiveControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}





void ModelPredictiveControl::publish_control()
{
  auto control_cmd = nmpc_ptr_->nmpc_cmd_struct; // Accessing nmpc_ptr_
      
  VehicleRatesSetpoint msg{};
  
  msg.thrust_body[0] = 0.0;
  msg.thrust_body[1] = 0.0;      
  double scale = 1.2 * 9.81 * 2;
  

  msg.thrust_body[2] = -control_cmd.control_thrust_vec[2]/scale;   //max thrust =30;


  msg.roll = control_cmd.control_attitude_vec[0];


  msg.pitch = -control_cmd.control_attitude_vec[1];

  msg.yaw = -control_cmd.control_attitude_vec[2];

   
  ModelPredictiveControl::vehicle_rates_setpoint_publisher_->publish(msg);



}


int main(int argc, char *argv[])
{

  
  ref_trajectory = { 0.0,     // px
                      0.0,    // py
                      2.0,    // pz
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // phi
                      0.0,    // theta
                      0.0     // psi
                      };

  gp_dist = {0.0,
             0.0,
            0.0};


  nmpc_struct.U_ref.resize(NMPC_NU);
  nmpc_struct.W.resize(NMPC_NY);

  cout << NMPC_NX ;

  nmpc_struct.W(0) = 30.0;
  nmpc_struct.W(1) = 30.0;
  nmpc_struct.W(2) = 30.0;
  nmpc_struct.W(3) = 1.0;
  nmpc_struct.W(4) = 1.0;
  nmpc_struct.W(5) = 1.0;
  nmpc_struct.W(6) = 30.0;
  nmpc_struct.W(7) = 30.0;
  //nmpc_struct.W(8) = 1.0;
  nmpc_struct.W(8) = 2.0;

  nmpc_struct.W(9) = 0.2;
  nmpc_struct.W(10) = 1.0;
  nmpc_struct.W(11) = 1.0;
  //nmpc_struct.W(12) = 0.01;
  nmpc_struct.W(12) = 3.0;


  nmpc_struct.min_Fz_scale = 0.5;
  nmpc_struct.max_Fz_scale = 10.0;
  nmpc_struct.W_Wn_factor = 0.5;
  
  double u_ref = 9.81*1.2*1.49;
  nmpc_struct.U_ref(0) = u_ref;   //m*g
  nmpc_struct.U_ref(1) = 0.0;
  nmpc_struct.U_ref(2) = 0.0;
  nmpc_struct.U_ref(3) = 0.0;

  

  current_states = { 0.0,     // px
                        0.0,     // py
                       0.0,   // pz
                         0.0,    // u
                         0.0,    // v
                        0.0,     // w
                        0.0,     // phi
                        0.0,     // theta
                        0.0};    // psi


  std::cout << "MPC node..." << std::endl;
  
  std::cout << "Starting model predictive control node..." << std::endl;
   


  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  rclcpp::init(argc, argv);
  

    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    dist_Fx.data = dist_Fx.data_zeros;
    dist_Fy.data = dist_Fy.data_zeros;
    dist_Fz.data = dist_Fz.data_zeros;

    online_data.distFx = dist_Fx.data;
    online_data.distFy = dist_Fy.data;
    online_data.distFz = dist_Fz.data;




  NMPC* nmpc = new NMPC(nmpc_struct);
  

  pos_ref = current_states;

  if (!nmpc->return_control_init_value())
      {  

          nmpc->nmpc_init(pos_ref, nmpc->nmpc_struct);


          nmpc_struct.verbose = 1;

          if (nmpc_struct.verbose && nmpc->return_control_init_value())
          {
              std::cout << "***********************************\n";
              std::cout << "NMPC: initialized correctly\n";
              std::cout << "***********************************\n";
          }
      }



    auto mpc_node = std::make_shared<ModelPredictiveControl>(nmpc);

   
    double t = 0.0;  // Initialize time
     
     while (rclcpp::ok()){ 
  online_data.distFx = {gp_dist.at(0)};
  online_data.distFy = {gp_dist.at(1)};
  online_data.distFz = {gp_dist.at(2)};
  ref_trajectory = { 2.0,     // px
                      2.0,    // py
                      2.0,    // pz
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // phi
                      0.0,    // theta
                      0.4     // psi  -psi
                      };
            
            t += 0.01;  // Adjust `time_step` according to your simulation's time step
         nmpc->nmpc_core(nmpc_struct,
                      nmpc->nmpc_struct,
                               nmpc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);
          
         //std::cout<<"online_data " << online_data.distFx[0];
          rclcpp::spin_some(mpc_node); // Spin only for the current node
        
  }

  
   
  rclcpp::shutdown();


  

 
  
 
  
  return 0;
  }
