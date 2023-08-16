#include <two_wheel_control/diff_drive_MPC.hpp>
#include <Eigen/Core>
#include <ct/core/plot/plot.h>

int main(int argc, char** argv){

    #ifdef HPIMP
        std::cout << "HPIMP not found" << std::endl;
    #endif

    two_wheel_controller::DiffDriveMPCController::StateVector x, x_dot;
    two_wheel_controller::DiffDriveMPCController::ControlVector u;
    if(argc < 6){
        x << 1,0,0,0,0;
    }else{
        for(int i = 1; i < argc && i < 6; i++){
            // double tmp;
            // std::cout << std::string(argv[i]) << std::endl;
            
            x(i-1) = std::stof(std::string(argv[i]));
        }
    }
    std::cout << x << std::endl;
    ////// TEST CONSTRAINT /////
    // // valid
    // two_wheel_controller::DiffDriveConstraint diff_constr(-10,10,0.1);
    // u << 0,0;
    // std::cout << "Constaint Test: \n" << diff_constr.evaluate(x,u,0) << std::endl;

    // // not valid (low)
    // u << -201,0;
    // std::cout << "Constaint Test: \n" << diff_constr.evaluate(x,u,0) << std::endl;

    // // not valid (low)
    // u << -100, -101;
    // std::cout << "Constaint Test: \n" << diff_constr.evaluate(x,u,0) << std::endl;

    // // valid 
    // u << 100, 50;
    // std::cout << "Constaint Test: \n" << diff_constr.evaluate(x,u,0) << std::endl;

    // // not valid (high)
    // u << 201,0;
    // std::cout << "Constaint Test: \n" << diff_constr.evaluate(x,u,0) << std::endl;

    // // not valid (high)
    // u << 100, 101;
    // std::cout << "Constaint Test: \n" << diff_constr.evaluate(x,u,0) << std::endl;

    // std::cout << "ubd: \n" << diff_constr.getLowerBound() << std::endl;
    // std::cout << "lbd: \n" << diff_constr.getUpperBound() << std::endl;

    // ////// TEST SYSTEM //////
    // two_wheel_controller::DiffDriveSystem sys;
    // sys.computeControlledDynamics(x,0,u,x_dot);

    // std::cout << "derivatives\n" << x_dot << std::endl;


    u << 0,0;
    two_wheel_controller::DiffDriveMPCController::diff_drive_param param;
    Eigen::Array<double,5,1> Q_diag;
    Q_diag << 1, 1, 0.1, 0, 0;
    Eigen::Array<double,5,1> P_diag;
    P_diag << 100, 100, 1, 0, 0;
    Eigen::Array<double,2,1> R_diag;
    R_diag << 0.1,0.1;
    param.Q.diagonal() = Q_diag;
    param.R.diagonal() = R_diag;
    param.P.diagonal() = P_diag;

    param.time_horizon = 5;
    param.frequency = 10;
    param.mpc_iterations = 10000;

    param.mpc_verbose = true;

    param.u_lb = -1;
    param.u_ub = 1;
    param.rw = 0.1;

    two_wheel_controller::DiffDriveMPCController mpc(param);
    

    u = mpc.update(x);
    std::cout << u << std::endl;

    std::shared_ptr<two_wheel_controller::DiffDriveSystem> system(new two_wheel_controller::DiffDriveSystem());
    Eigen::Matrix<double, 2, -1> control_traj = mpc.getControlTrajectory();

    // double dt = 0.1;
    // ct::core::Time t0 = 0;
    size_t steps = 50;
    Eigen::Matrix<double, 5, -1> sim_traj = Eigen::Matrix<double, 5, -1>::Zero(5,steps);
    // sim_traj.col(0) = x;
    // ct::core::StateVector<5> der;
    // for(size_t i = 1; i < steps; i++){
    //     system->computeControlledDynamics(sim_traj.col(i-1), 0, control_traj.col(i-1), der);
    //     sim_traj.col(i) = sim_traj.col(i-1) + der*dt;
    // }
    // for(size_t i = 0; i < 50; i++){
    //     auto x_tmp = x; 
    //     mpc.getMPC()->doForwardIntegration(0, (double)i*0.1, x_tmp);
    //     sim_traj.col(i) = x_tmp;
    // }
    // ct::core::plot::ion();
    // ct::core::plot::figure("Simulated State");
    // ct::core::plot::plot(sim_traj.row(0));
    // ct::core::plot::plot(sim_traj.row(1));
    // ct::core::plot::plot(sim_traj.row(2));
    // ct::core::plot::plot(sim_traj.row(3));
    // ct::core::plot::plot(sim_traj.row(4));
    // ct::core::plot::grid(true);

    Eigen::Matrix<double,5,-1> state_traj = mpc.getStateTrajectory();
    ct::core::plot::figure("State Trajectory");
    std::cout << "Trajectory = [" << state_traj << "]" << std::endl;
    ct::core::plot::plot(state_traj.row(0));
    ct::core::plot::plot(state_traj.row(1));
    ct::core::plot::plot(state_traj.row(2));
    ct::core::plot::plot(state_traj.row(3));
    ct::core::plot::plot(state_traj.row(4));
    ct::core::plot::grid(true);

    std::cout << "Control_action = [" << control_traj << "];" << std::endl;
    ct::core::plot::figure("Control Trajectory");
    ct::core::plot::plot(control_traj.row(0));
    ct::core::plot::plot(control_traj.row(1));
    ct::core::plot::grid(true);
    ct::core::plot::show();

    

    return 0;
}