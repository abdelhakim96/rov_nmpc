



//int N =9;


#include "mpc_acado.cpp"
#include "mpc_acado_node.h"
#include "mpc_acado.h"


class ModelPredictiveControl : public rclcpp::Node
{

      
public:
  NMPC* nmpc_ptr_;
  ModelPredictiveControl(NMPC* nmpc_ptr) : Node("model_predictive_control"), nmpc_ptr_(nmpc_ptr)
  {   
 
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    // create publishers: 
    control_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/mobula/rov/wrench", 10);
    
    // create subscribers:

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&ModelPredictiveControl::odom_cb, this, std::placeholders::_1));

    gp_dist_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gp_pred_mu", qos, std::bind(&ModelPredictiveControl::gp_dist_cb, this, std::placeholders::_1));

    auto timer_callback = [this]() -> void {

      publish_control();
            
    
    };
    timer_ = this->create_wall_timer(2ms, timer_callback);
  }
            
    
  


private:

  //Initialize callbacks: 

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);    //odometry callback
  void gp_dist_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg); //gp estimation callback


  // Initiliaze subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gp_dist_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;


  // Initiliaze publishers
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr control_wrench_publisher_;


  void publish_control();

};


void ModelPredictiveControl::gp_dist_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

   gp_dist ={msg->data[0],msg->data[1],msg->data[2]};

}


void ModelPredictiveControl::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
      

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);


    tf2::Matrix3x3(q).getRPY(Phi, Theta, Psi);

   /*
  state_feedback = { msg->pose.pose.position.x, 
                      msg->pose.pose.position.y,
                      -msg->pose.pose.position.z,
                         Phi,    
                          Theta,  
                          Psi,
                          msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z,
                          msg->twist.twist.angular.x,
                          msg->twist.twist.angular.y,
                          msg->twist.twist.angular.z
                            };
*/
}



void ModelPredictiveControl::publish_control()
{
  auto control_cmd = nmpc_ptr_->nmpc_cmd_struct; // Accessing nmpc_ptr_
      
  geometry_msgs::msg::Wrench msg{};
    
  msg.force.x = control_cmd.force[0];  
  msg.force.y = control_cmd.force[1];
  msg.force.z = control_cmd.force[2];
  msg.torque.x = control_cmd.torque[0];
  msg.torque.y = control_cmd.torque[1];
  msg.torque.z = control_cmd.torque[2];

   
  ModelPredictiveControl::control_wrench_publisher_->publish(msg);
  cout << "force x" << msg.force.x  << std::endl;
  cout << "force y" << msg.force.y  << std::endl;


}


int main(int argc, char *argv[])
{



  state_feedback = { 0.0,     // px
                     0.0,     // px
                      0.0,     // px
                      0.0,    // phi
                      0.0,    // theta
                      0.0,    // psi
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // p
                      0.0,    // q
                      0.0     // r
                    };

  reference = {       0.0,     // px
                      0.0,     // px
                      0.0,     // px
                      0.0,    // phi
                      0.0,    // theta
                      0.0,    // psi
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // p
                      0.0,    // q
                      0.0     // r
                    };



 


  nmpc_struct.U_ref.resize(NMPC_NU);
  nmpc_struct.W.resize(NMPC_NY);

  

  // Weights of Cost function

  // Weights on state-feedback
  nmpc_struct.W(0) = 30.0;   //W_p_x
  nmpc_struct.W(1) = 30.0;   //W_p_y
  nmpc_struct.W(2) = 30.0;   //W_p_z
  nmpc_struct.W(3) = 1.0;    //W_roll
  nmpc_struct.W(4) = 1.0;     //W_pitch
  nmpc_struct.W(5) = 1.0;     //W_yaw
  nmpc_struct.W(6) = 1.0;     //W_u
  nmpc_struct.W(7) = 1.0;     //W_v
  nmpc_struct.W(8) = 1.0;     //W_w
  nmpc_struct.W(9) = 0.2;     //W_p
  nmpc_struct.W(10) = 1.0;     //W_q
  nmpc_struct.W(11) = 1.0;     //W_r

  // Weights on control
  nmpc_struct.W(12) = 3.0;     //W_X
  nmpc_struct.W(13) = 3.0;     //W_Y
  nmpc_struct.W(14) = 3.0;     //W_Z
  nmpc_struct.W(15) = 3.0;     //W_M_x
  nmpc_struct.W(16) = 3.0;     //W_M_y
  nmpc_struct.W(17) = 3.0;     //W_M_z
  
  // Terminal weights (for convergence and stability)
  nmpc_struct.W_Wn_factor = 1.0;
  

  // set reference for controls
  nmpc_struct.U_ref(0) = 0.0;   
  nmpc_struct.U_ref(1) = 0.0;
  nmpc_struct.U_ref(2) = 0.0;
  nmpc_struct.U_ref(3) = 0.0;
  nmpc_struct.U_ref(4) = 0.0;
  nmpc_struct.U_ref(5) = 0.0;

  // initialize GP estimation
  //online_data.distFx = 0.0;

   dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    online_data.distFx = dist_Fx.data_zeros;
    online_data.distFy = dist_Fx.data_zeros;
    online_data.distFz = dist_Fx.data_zeros;
    online_data.distMx = dist_Fx.data_zeros;
    online_data.distMy = dist_Fx.data_zeros;
    online_data.distMz = dist_Fx.data_zeros;






  std::cout << "MPC node..." << std::endl;
  
  std::cout << "Starting model predictive control node..." << std::endl;
   


  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  rclcpp::init(argc, argv);
  
    /*
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    dist_Fx.data = dist_Fx.data_zeros;
    dist_Fy.data = dist_Fy.data_zeros;
    dist_Fz.data = dist_Fz.data_zeros;
     */
 




  NMPC* nmpc = new NMPC(nmpc_struct);
  

  pos_ref = state_feedback;

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

   
     
    while (rclcpp::ok()){ 
      //online_data.distFx = {gp_dist.at(0)};
    //online_data.distFy = {gp_dist.at(1)};
     //online_data.distFz = {gp_dist.at(2)};
    online_data.distFx = dist_Fx.data_zeros;
    online_data.distFy = dist_Fx.data_zeros;
    online_data.distFz = dist_Fx.data_zeros;
    online_data.distMx = dist_Fx.data_zeros;
    online_data.distMy = dist_Fx.data_zeros;
    online_data.distMz = dist_Fx.data_zeros;

  state_feedback = { 0.0,     // px
                     0.0,     // px
                      0.0,     // px
                      0.0,    // phi
                      0.0,    // theta
                      0.0,    // psi
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // p
                      0.0,    // q
                      0.0     // r
                    };

  reference = {       2.0,     // px
                      0.0,     // px
                      0.0,     // px
                      0.0,    // phi
                      0.0,    // theta
                      0.0,    // psi
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // p
                      0.0,    // q
                      0.0     // r
                    };
            
         nmpc->nmpc_core(nmpc_struct,
                      nmpc->nmpc_struct,
                               nmpc->nmpc_cmd_struct,
                               reference,
                               online_data,
                               state_feedback);
          
         //std::cout<<"online_data " << online_data.distFx[0];
          rclcpp::spin_some(mpc_node); // Spin only for the current node
        
  }

  
   
  rclcpp::shutdown();


  

 
  
 
  
  return 0;
  }
