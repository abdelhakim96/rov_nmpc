#include <acado_code_generation.hpp>
#include <iostream>

using namespace std;
int main( )
{
	USING_NAMESPACE_ACADO
	string path ="/home/hakim/Desktop/Phd/ros2_ws/src/mpc_acado";
	// Variables:
   ///home/hakim/Desktop/Phd/ros2_ws/src/mpc_acado/externals/ACADOtoolkit/acado


		//string path ="home/hakim/Desktop/Phd/ros2_ws/src/mpc_acado/externals/ACADOtoolkit/acado";

    const float m = 1.2;
    const float g = 9.8;

    //string path ="/home/hakim/Desktop/Phd/ros2_ws/build/mpc_acado";

	// Variables:
	DifferentialState   p_x    ;  // pos
	DifferentialState   p_y    ;  // pos 
    DifferentialState   p_z    ;  // pos 

	DifferentialState   u    ;  
	DifferentialState   v    ;   
    DifferentialState   w    ;  

	DifferentialState   phi  ;  
	DifferentialState 	theta; 		
	DifferentialState 	psi;

    OnlineData Fx_dist;   
    OnlineData Fy_dist;   
    OnlineData Fz_dist;   

	Control             F_zb    ;  // thrust
	Control 			p	;	// roll rate
    Control 			q	;	// pitch rate
	Control 			r	;	// yaw rate

			
	// Model equations:
	DifferentialEquation f; 


	
    f << dot(p_x) == (cos(theta) * cos(psi)) * u + (sin(phi) * sin(theta) * cos(psi) - sin(psi) * cos(phi)) * v +
                       (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;
    f << dot(p_y) == (cos(theta) * sin(psi)) * u + (sin(phi) * sin(theta) * sin(psi) + cos(psi) * cos(phi)) * v +
                       (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;
    f << dot(p_z) == (-sin(theta)) * u + (sin(phi) * cos(theta)) * v + (cos(phi) * cos(theta)) * w;

    f << dot(u) == r * v - q* w + g * sin(theta) + Fx_dist;
    f << dot(v) == p * w - r * u - g * sin(phi) * cos(theta) + Fy_dist ;
    f << dot(w) == q * u - p * v - g * cos(phi) * cos(theta) + (1 / m) * (F_zb) + Fz_dist;

    f << dot(phi) == p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
    f << dot(theta) == cos(phi) * q - sin(phi) * r;
    f << dot(psi) == sin(phi) / cos(theta) * q + cos(phi) / cos(theta) * r;


	// Reference functions and weighting matrices:
	Function h, hN;

	h << p_x << p_y << p_z << u << v << w << phi << theta << psi << F_zb << p << q << r;				
	hN << p_x << p_y << p_z << u << v << w << phi << theta << psi ;

	// Provide defined weighting matrices:
	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );
		

    double N = 30;
    double Ts = 0.01;
    OCP ocp(0.0, N * Ts, N);

	ocp.subjectTo( f );
	

	
	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( -0.3 * m * g <= F_zb <= m * g * 2 );
	ocp.subjectTo(-3.0 <=  p <= 3.0);
	ocp.subjectTo(-3.0 <=  q <= 3.0);
    ocp.subjectTo(-3.0 <=  r <= 3.0);
  

	
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
