
#include "mpc_acado.h"


NMPCworkspace nmpcWorkspace;
NMPCvariables nmpcVariables;

NMPC::NMPC(struct nmpc_struct_& _nmpc_inp_struct)
{
    is_control_init = false;
    
    nmpc_inp_struct = _nmpc_inp_struct;
    nmpc_inp_struct_0 = _nmpc_inp_struct;
    nmpc_inp_struct.verbose =1;
    WN.resize(NMPC_NYN);
    for (int i = 0; i < NMPC_NYN; ++i)
        WN[i] = nmpc_inp_struct.W_Wn_factor * nmpc_inp_struct.W[i];

    // --------------------
    // ACADO NMPC CONTROLLER
    // --------------------

    nmpc_struct.initializeSolver = boost::bind(nmpc_initializeSolver);
    nmpc_struct.preparationStep = boost::bind(nmpc_preparationStep);
    nmpc_struct.feedbackStep = boost::bind(nmpc_feedbackStep);
    nmpc_struct.getKKT = boost::bind(nmpc_getKKT);
    nmpc_struct.getObjective = boost::bind(nmpc_getObjective);
    nmpc_struct.printDifferentialVariables = boost::bind(nmpc_printDifferentialVariables);
    nmpc_struct.printControlVariables = boost::bind(nmpc_printControlVariables);

    nmpc_struct.acado_N = NMPC_N;
    nmpc_struct.acado_NX = NMPC_NX;
    nmpc_struct.acado_NY = NMPC_NY;
    nmpc_struct.acado_NYN = NMPC_NYN;
    nmpc_struct.acado_NU = NMPC_NU;
    nmpc_struct.acado_NOD = NMPC_NOD;

    nmpc_struct.x0 = &nmpcVariables.x0[0];
    nmpc_struct.x = &nmpcVariables.x[0];
    nmpc_struct.od = &nmpcVariables.od[0];  //online data
    nmpc_struct.y = &nmpcVariables.y[0];
    nmpc_struct.yN = &nmpcVariables.yN[0];
    nmpc_struct.u = &nmpcVariables.u[0];
    nmpc_struct.W = &nmpcVariables.W[0];
    nmpc_struct.WN = &nmpcVariables.WN[0];

    nmpc_cmd_struct.force.resize(3, 0.0);
    nmpc_cmd_struct.torque.resize(3, 0.0);
    //nmpc_cmd_struct.control_thrust_vec = {nmpc_inp_struct.U_ref(3),
    //                                      ((1 - 0) / (nmpc_inp_struct.max_Fz_scale - nmpc_inp_struct.min_Fz_scale)) *
    //                                          (nmpc_inp_struct.U_ref(3) - nmpc_inp_struct.min_Fz_scale)};




    nmpc_cmd_struct.exe_time = 0.0;
    nmpc_cmd_struct.kkt_tol = 0.0;
    nmpc_cmd_struct.obj_val = 0.0;

    if (nmpc_inp_struct.verbose)
    {
        std::cout << "***********************************\n";
        std::cout << "Constructor of the class NMPC is created\n";
        std::cout << "***********************************\n";
    }
}

NMPC::~NMPC()
{
    if (nmpc_inp_struct.verbose)
        std::cout << "Destructor of the class NMPC\n";
}

bool NMPC::return_control_init_value()
{
    return NMPC::is_control_init;
}

void NMPC::nmpc_init(std::vector<double> posref, struct acado_struct& acadostruct)
{
    if (nmpc_inp_struct.verbose)
    {
        std::cout << "***********************************\n";
        std::cout << "outer_nmpc_initController - start\n";
    }

    // Initialize the solver
    // ---------------------
    acadostruct.initializeSolver();

    // NMPC: initialize/set the states
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NX * (acadostruct.acado_N + 1); ++i)
    {
        acadostruct.x[i] = 0.0;
    }

    // NMPC: initialize/set the controls
    // ---------------------
    for (int i = 0; i < acadostruct.acado_N; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NU; ++j)
            acadostruct.u[(i * acadostruct.acado_NU) + j] = nmpc_inp_struct.U_ref(j);
    }

    // NMPC: initialize/set the online data
    // ---------------------
    for (int i = 0; i < acadostruct.acado_N + 1; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NOD; ++j)
        {
            //if (j < acadostruct.acado_NOD - 3)
            //    acadostruct.od[(i * acadostruct.acado_NOD) + j] = 0.0;
            //else
                acadostruct.od[(i * acadostruct.acado_NOD) + j] = 0.0;
        }
    }

    // NMPC: initialize the measurements/reference
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NY * acadostruct.acado_N; ++i)
    {
        acadostruct.y[i] = 0.0;
    }
    for (int i = 0; i < acadostruct.acado_NYN; ++i)
    {
        acadostruct.yN[i] = 0.0;
    }

    // NMPC: initialize the current state feedback
    // ---------------------
#if ACADO_INITIAL_STATE_FIXED
    for (int i = 0; i < acadostruct.acado_NX; ++i)
    {
        if (i < 3)
        {
            acadostruct.x0[i] = posref[i];
        }
        else
            acadostruct.x0[i] = 0;
    }
#endif

    // NMPC: initialize the weight matrices
    // ------------------------------------
    for (int i = 0; i < acadostruct.acado_NY; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NY; ++j)
        {
            if (i == j)
                acadostruct.W[(i * acadostruct.acado_NY) + j] = nmpc_inp_struct.W[i];

            else
                acadostruct.W[(i * acadostruct.acado_NY) + j] = 0.0;
        }
    }

     std::cout<<"W_0 = "<<nmpc_inp_struct.W<<"\n";

    for (int i = 0; i < acadostruct.acado_NYN; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NYN; ++j)
        {
            if (i == j)
                acadostruct.WN[(i * acadostruct.acado_NYN) + j] = WN[i];
            else
                acadostruct.WN[(i * acadostruct.acado_NYN) + j] = 0.0;
        }
    }

    //  std::cout << "WN_0 = "<<WN<<"\n";

    // Prepare first step
    // ------------------
    acadostruct.preparationStep();

    if (nmpc_inp_struct.verbose)
    {
        std::cout << "Outer NMPC: initialized correctly\n";
        std::cout << "***********************************\n";
    }
    is_control_init = true;
}

void NMPC::nmpc_core(struct nmpc_struct_& _nmpc_inp_struct,
                        struct acado_struct& acadostruct,
                        struct command_struct& commandstruct,
                        std::vector<double>& reftrajectory,
                        struct online_data_struct_& online_data,
                        std::vector<double>& statesmeas)
{
    nmpc_inp_struct = _nmpc_inp_struct;

    // To avoid sending nan values to optimization problem
    nan_check_for_dist_estimates(online_data);
    

    //std::cout << "online_data: " << online_data << "\n";
    // set the current state feedback
    set_measurements(acadostruct, online_data, statesmeas);

    // set the reference path
    set_reftrajectory(acadostruct, reftrajectory);

    // NMPC: calc and apply control and prepare optimization for the next step
    // ----------------------------------------------------------------------
     //std::cout << "NMPC core is running" << "\n";
    // Execute Calculation (Optimization)
    clock_t stopwatch;
    stopwatch = clock();
    acado_feedbackStep_fb = acadostruct.feedbackStep();
    
    if (nmpc_inp_struct.verbose && acado_feedbackStep_fb != 0)
    {
        std::cout << "ACADO ERROR: " << acado_feedbackStep_fb << "\n";
        std::cout << "acado outer nmpc controller states: x, y, z, u, v, w, roll, pitch, yaw = " << acadostruct.x0[0] << ", "
                  << acadostruct.x0[1] << ", " << acadostruct.x0[2] << ", " << acadostruct.x0[3] << ", "
                  << acadostruct.x0[4] << ", " << acadostruct.x0[5] << ", " << acadostruct.x0[6] << ", "  
                  << acadostruct.x0[7] << ", " << acadostruct.x0[8] << " \n";
    }

    // Apply the new control immediately to the process, first NU components.
    
    //std::cout << "thrust debug: " << acadostruct.u[0] << "\n";
    //commandstruct.control_thrust_vec = {acadostruct.u[0],
      //                                  ((1 - 0) / (nmpc_inp_struct.max_Fz_scale - nmpc_inp_struct.min_Fz_scale)) *
      //                                      (acadostruct.u[0] - nmpc_inp_struct.min_Fz_scale)};

    commandstruct.force[0] = acadostruct.u[0];
    commandstruct.force[1] = acadostruct.u[1];
    commandstruct.force[2] = acadostruct.u[2];

    commandstruct.torque[0] = acadostruct.u[3];
    commandstruct.torque[1] = acadostruct.u[4];
    commandstruct.torque[2] = acadostruct.u[5];

    commandstruct.kkt_tol = acadostruct.getKKT();
    commandstruct.obj_val = acadostruct.getObjective();

    // Settings for the next iteration
    acadostruct.preparationStep();

    // Calculate the entire execution time!
    commandstruct.exe_time = ((double)(clock() - stopwatch)) / CLOCKS_PER_SEC;

    //    ROS_INFO_STREAM("Stoptime outer NMPC: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");

    /* ------ NMPC_DEBUG ------*/
      //acadostruct.printDifferentialVariables();
      //acadostruct.printControlVariables();
}

void NMPC::set_measurements(struct acado_struct& acadostruct,
                               struct online_data_struct_& online_data,
                               std::vector<double>& statesmeas)
{
    for (int i = 0; i < acadostruct.acado_NX; i++)
    {
        acadostruct.x0[i] = statesmeas.at(i);
    }
    for (int i = 0; i < acadostruct.acado_N + 1; ++i)
    {
        int ref_idx = 0;
        //for (int idx = 0; idx < 3; idx++)
        //{
        //    acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = statesmeas.at(acadostruct.acado_NX + idx);
        //}
       // acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distFx[i];
       // acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distFy[i];
       // acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distFz[i];
        acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distFx[0];
        acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distFy[0];
        acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distFz[0];
        acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distMx[0];
        acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distMy[0];
        acadostruct.od[(i * acadostruct.acado_NOD) + ref_idx++] = online_data.distMz[0];
        
    }


    // Recompute U_ref based on new disturbance estimates
    // phi_ref = asin(Fy_dist/Fz)
   //  nmpc_inp_struct.U_ref(0) = 0.0;
    // theta_ref = asin(-Fx_dist/Fz)
   //  nmpc_inp_struct.U_ref(1) = 0.0;
   //  nmpc_inp_struct.U_ref(3) = 0.0;  // works for GP
    // nmpc_inp_struct.U_ref(3) = (nmpc_inp_struct_0.U_ref(3) - online_data.distFz[0]*2.0);  // works for NMHE

}

void NMPC::set_reftrajectory(struct acado_struct& acadostruct, std::vector<double>& reftrajectory)
{
    // References for objectives
    for (int i = 0; i < acadostruct.acado_NYN; i++)
    {
        acadostruct.yN[i] = reftrajectory[i];
    }
    


    for (int i = 0; i < acadostruct.acado_N; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NY; ++j)
        {
            if (j < acadostruct.acado_NYN)
                // Set References for states

                acadostruct.y[(i * acadostruct.acado_NY) + j] = acadostruct.yN[j];
            else

                // Set References for control

                acadostruct.y[(i * acadostruct.acado_NY) + j] = nmpc_inp_struct.U_ref(j - acadostruct.acado_NYN);
        }
    }


}

void NMPC::nan_check_for_dist_estimates(struct online_data_struct_& online_data)
{
    if (std::isnan(online_data.distFx[0]) == true || std::isnan(online_data.distFx[NMPC_N - 1]) == true)
    {
        std::cout << "NAN received for Fx estimates! \n";
        std::cout << "Zero values are enforced! \n";
        std::fill(online_data.distFx.begin(), online_data.distFx.end(), 0);
    }
    if (std::isnan(online_data.distFy[0]) == true || std::isnan(online_data.distFy[NMPC_N - 1]) == true)
    {
        std::cout << "NAN received for Fy estimates! \n";
        std::cout << "Zero values are enforced! \n";
        std::fill(online_data.distFy.begin(), online_data.distFy.end(), 0);
    }
    if (std::isnan(online_data.distFz[0]) == true || std::isnan(online_data.distFz[NMPC_N - 1]) == true)
    {
        std::cout << "NAN received for Fz estimates! \n";
        std::cout << "Zero values are enforced! \n";
        std::fill(online_data.distFz.begin(), online_data.distFz.end(), 0);
    }
}