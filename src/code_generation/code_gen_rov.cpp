#include <acado_code_generation.hpp>
#include <iostream>

using namespace std;



// BlueROV2 Model Parameters 
//const double F_bouy = 114.8; // Bouyancy force (N)


const double m = 11.4;    // BlueROV2 mass (kg)  
const double g = 9.82;  // gravitational field strength (m/s^2)

const double F_bouy = 1026 * 0.0115 * g; // Bouyancy force (N)
const double eps = 0.00001;
//const double F_bouy = 114.8; // Buoyancy force (N)

const double X_ud = -2.6; // Added mass in x direction (kg)
const double Y_vd = -18.5; // Added mass in y direction (kg)
const double Z_wd = -13.3; // Added mass in z direction (kg)


const double  K_pd = -0.054; // Added mass for rotation about x direction (kg)
const double  M_qd = -0.0173; // Added mass for rotation about y direction (kg)
const double N_rd = -0.28; // Added mass for rotation about z direction (kg)

const double I_xx = 0.21; // Moment of inertia (kg.m^2)
const double I_yy = 0.245; // Moment of inertia (kg.m^2)
const double I_zz = 0.245; // Moment of inertia (kg.m^2)

const double X_u = -0.09; // Linear damping coefficient in x direction (N.s/m)
const double Y_v = -0.26; // Linear damping coefficient  in y direction (N.s/m)
const double Z_w = -0.19; // Linear damping coefficient  in z direction (N.s/m)

const double K_p = -0.895;  // Linear damping coefficient for rotation about z direction (N.s/rad)
const double M_q = -0.287;  // Linear damping coefficient for rotation about z direction (N.s/rad)
const double N_r = -4.64;  // Linear damping coefficient for rotation about z direction (N.s/rad)

const double X_uc = -34.96; // quadratic damping coefficient in x direction (N.s^2/m^2)
const double Y_vc = -103.25; // quadratic damping coefficient  in y direction (N.s^2/m^2)
const double Z_wc = -74.23; // quadratic damping coefficient  in z direction (N.s^2/m^2)

const double K_pc = -0.084; // quadratic damping coefficient for rotation about x direction (N.s^2/rad^2)
const double M_qc = -0.028; // quadratic damping coefficient for rotation about y direction (N.s^2/rad^2)
const double N_rc = -0.43; // quadratic damping coefficient for rotation about z direction (N.s^2/rad^2)

const double z_b = 0.1; //Distance between cb and cg along the z-axis.
const double r_b = 0.01; //The distance between co and cg.

const double lx_1 = 0.01; //Moment arms  about x from cg to each thruster i.
const double lx_2 = 0.01;
const double lx_3 = 0.01;
const double lx_4 = 0.01;
const double lx_5 = 0.01;
const double lx_6 = 0.01;
const double lx_7 = 0.01;
const double lx_8 = 0.01;

const double ly_1 = 0.01; //Moment arms  about y from cg to each thruster i.
const double ly_2 = 0.01;
const double ly_3 = 0.01;
const double ly_4 = 0.01;
const double ly_5 = 0.01;
const double ly_6 = 0.01;
const double ly_7 = 0.01;
const double ly_8 = 0.01;

const double lz_1 = 0.01; //Moment arms  about z from cg to each thruster i.
const double lz_2 = 0.01;
const double lz_3 = 0.01;
const double lz_4 = 0.01;
const double lz_5 = 0.01;
const double lz_6 = 0.01;
const double lz_7 = 0.01;
const double lz_8 = 0.01;




int main( )
{
	USING_NAMESPACE_ACADO
	string path ="/home/hakim/home/ros2_ws/src/nmpc_rov";
	// Variables:
   ///home/hakim/Desktop/Phd/ros2_ws/src/mpc_acado/externals/ACADOtoolkit/acado

    //State Variables:
    DifferentialState p_x;  // the body position w.r.t X_I
    DifferentialState p_y;  // the body position w.r.t Y_I
    DifferentialState p_z;  // the body position w.r.t Z_I


    DifferentialState phi;  // roll angle 
    DifferentialState theta;  // pitch angle 
    DifferentialState psi;  // yaw angle 



    DifferentialState u;  // the translation velocity along X_B
    DifferentialState v;  // the translation velocity along Y_B
    DifferentialState w;  // the translation velocity along Z_B



    DifferentialState p;   // roll rate
    DifferentialState q;   // pitch rate 
    DifferentialState r;   // yaw rate

    OnlineData Fx_dist;  // the external disturbance force along X_B
    OnlineData Fy_dist;  // the external disturbance force along Y_B
    OnlineData Fz_dist;  // the external disturbance force along Z_B
    OnlineData Mx_dist;  // the external disturbance force along X_B
    OnlineData My_dist;  // the external disturbance force along Y_B
    OnlineData Mz_dist;  // the external disturbance force along Z_B

    // MPC control input 
    Control X;  // Force along X_B
    Control Y;  // Force along Y_B
    Control Z;  // Force along Z_B

    Control M_x;  // Torque about X_B (Rolling moment)
    Control M_y;  // Torque about Y_B (Pitching moment)
    Control M_z;  // Torque about Z_B (Yawing moment)


    // Model equations: 2-D model, assuming no roll or pitch
    DifferentialEquation f;

    IntermediateState g_x = (F_bouy - m * g) * sin(theta);
    IntermediateState g_y = -(F_bouy - m * g) * cos(theta) * sin(phi);
    IntermediateState g_z = -(F_bouy - m * g) * cos(theta) * cos(phi);
    IntermediateState g_phi = -z_b * F_bouy * cos(theta) * sin(phi);
    IntermediateState g_theta = -z_b * F_bouy * sin(theta);
    IntermediateState g_psi = 0.0;


    // Equations of motion:

    // a)  Transformation body frame (F_B) to Inertial frame (F_I)
    f << dot(p_x) == (cos(theta) * cos(psi)) * u + (sin(phi) * sin(theta) * cos(psi) - sin(psi) * cos(phi)) * v +
        (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;

    f << dot(p_y) == (cos(theta) * sin(psi)) * u + (sin(phi) * sin(theta) * sin(psi) + cos(psi) * cos(phi)) * v +
        (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;

    f << dot(p_z) == (-sin(theta)) * u + (sin(phi) * cos(theta)) * v + (cos(phi) * cos(theta)) * w;

    f << dot(phi) == p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;

    f << dot(theta) == cos(phi) * q - sin(phi) * r;

    f << dot(psi) == sin(phi) / cos(theta) * q + cos(phi) / cos(theta) * r;



    // b)  Newton-Euler equations of motions in body frame: 

    f << dot(u) == (X + m * (r * v - q * w) - Y_vd * r * v + Z_wd * q * w
        + (X_u + X_uc * sqrt(u * u + eps)) * u + g_x) / (m - X_ud) + Fx_dist;        // x-direction   (x_b)

    f << dot(v) == (Y + m * (p * w - r * u) + X_ud * r * u - Z_wd * p * w
        + (Y_v + Y_vc * sqrt(v * v + eps)) * v + g_y) / (m - Y_vd) + Fy_dist;              // y-direction  (y_b)

    f << dot(w) == (Z + m * (q * u - p * v) - X_ud * q * u + Y_vd * p * v
        + (Z_w + Z_wc * sqrt(w * w + eps)) * w + g_z) / (m - Z_wd) + Fz_dist;               // z-direction   (z_b)

    f << dot(p) == (M_x - M_qd * q * r + N_rd * q * r + q * r(I_yy - I_zz)                     // rotation about x_b 
        - Y_vd * v * w + Z_wc * v * w
        + (K_p + K_pc * sqrt(p * p + eps)) * p + g_phi) / (I_xx - K_pd);

    f << dot(q) == (M_y + K_pd * p * r - N_rd * p * r                                          // rotation about y_b 
        + p * r(I_zz - I_xx) - Z_wd * u * w + X_ud * u * w
        + (M_q + M_qc * sqrt(q * q + eps)) * q + g_theta) / (I_yy - M_qd);

    f << dot(r) == (M_z - K_pd * p * q + M_qd * p * q + p * q(I_xx - I_yy)                    // rotation about z_b 
        - X_ud * u * v + Y_vd * u * v
        + (N_r + N_rc * sqrt(r * r + eps)) * r) / (I_zz - N_rd);


    // Reference functions and weighting matrices:
    Function h, hN;
    h << p_x << p_y << p_z << u << v << w << phi << theta << psi << r << X << Y << Z << M_x << M_y << M_z;
    hN << p_x << p_y << p_z << u << v << w << phi << theta << psi << r;

    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    //
    // Optimal Control Problem
    //

    // Prediction Horizon = N * Ts
    double N = 50;     // Number of prediction steps  
    double Ts = 0.01;  // time step 
    OCP ocp(0.0, N * Ts, N);

    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);


    // Constraints on Inputs ( based on max thrust)
    ocp.subjectTo(-80 <= X <= 80);
    ocp.subjectTo(-80 <= Y <= 80);
    ocp.subjectTo(-160 <= Z <= 160);
    ocp.subjectTo(-160 <= M_x <= 160);    //in Nm
    ocp.subjectTo(-160 <= M_y <= 160);    //in Nm
    ocp.subjectTo(-160 <= M_z <= 160);    //in Nm
  

	
	// Export the code:
	OCPexport mpc( ocp );
   
	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        2 * N );


	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	//mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX,         YES);
	mpc.set( QP_SOLVER,QP_QPOASES);
	mpc.set(MAX_NUM_QP_ITERATIONS, 1000);
 	mpc.set( HOTSTART_QP,                 YES             );

    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, YES);  // Possible to Change Constraints Afterwards (only with qpOASES)

	mpc.set( GENERATE_TEST_FILE,          NO             );
	mpc.set( GENERATE_MAKE_FILE,          NO             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );
	
    

    // Optionally set custom module name:
    mpc.set(CG_MODULE_NAME, "nmpc");
    mpc.set(CG_MODULE_PREFIX, "NMPC");
    
	if (mpc.exportCode( path + "/model/codegen" ) != SUCCESSFUL_RETURN)
	//  if (mpc.exportCode( "acado_threading" ) != SUCCESSFUL_RETURN)
  		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );


	return EXIT_SUCCESS;
}
