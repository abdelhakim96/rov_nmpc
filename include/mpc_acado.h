#ifndef _MPC_acado_H
#define _MPC_acado_H

#include <math.h>
#include <Eigen/Dense>
#include <time.h>
#include "nmpc_common.h"
#include "nmpc_auxiliary_functions.h"
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <vector>
#include <iostream>
extern NMPCworkspace nmpcWorkspace;
extern NMPCvariables nmpcVariables;

using namespace Eigen;
//using namespace std;


struct nmpc_struct_
{
    bool verbose;
    bool yaw_control;
    double min_Fz_scale;
    double max_Fz_scale;
    double W_Wn_factor;

    Eigen::VectorXd U_ref;
    Eigen::VectorXd W;
};

struct online_data_struct_
{
    std::vector<double> distFx;
    std::vector<double> distFy;
   std::vector<double> distFz;
    std::vector<double> distMx;
    std::vector<double> distMy;
   std::vector<double> distMz;
};

class NMPC
{
private:
    //    double m;
    //    double g;

    bool is_control_init;

    nmpc_struct_ nmpc_inp_struct_0, nmpc_inp_struct;
    Eigen::VectorXd WN;

public:
    int acado_feedbackStep_fb;

    struct acado_struct
    {
        boost::function<int(void)> initializeSolver;
        boost::function<int(void)> preparationStep;
        boost::function<int(void)> feedbackStep;
        boost::function<real_t(void)> getKKT;
        boost::function<real_t(void)> getObjective;
        boost::function<void(void)> printDifferentialVariables;
        boost::function<void(void)> printControlVariables;

        int acado_N;
        int acado_NX;
        int acado_NY;
        int acado_NYN;
        int acado_NU;
        int acado_NOD;

        real_t* x0;
        real_t* u;
        real_t* x;
        real_t* od;
        real_t* y;
        real_t* yN;
        real_t* W;
        real_t* WN;
    } nmpc_struct;

    struct command_struct
    {
        std::vector<double> force;
        std::vector<double> torque;
        double exe_time;
        double kkt_tol;
        double obj_val;

    } nmpc_cmd_struct;

    NMPC(struct nmpc_struct_& _nmpc_inp_struct);
    ~NMPC();

    bool return_control_init_value();

    void nmpc_init(std::vector<double> posref, struct acado_struct& acadostruct);

    void nmpc_core(struct nmpc_struct_& _nmpc_inp_struct,
                   struct acado_struct& acadostruct,
                   struct command_struct& commandstruct,
                   std::vector<double>& reftrajectory,
                   struct online_data_struct_& online_data,   //online data
                   std::vector<double>& statesmeas);

    void publish_rpyFz(struct command_struct& commandstruct);

protected:
    void set_measurements(struct acado_struct& acadostruct,
                          struct online_data_struct_& online_data,
                          std::vector<double>& statesmeas);

    void set_reftrajectory(struct acado_struct& acadostruct, std::vector<double>& reftrajectory);

    void nan_check_for_dist_estimates(struct online_data_struct_& online_data);
};

#endif