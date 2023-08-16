// #include <blasfeo_common.h>
#include <hpipm_common.h>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

namespace two_wheel_controller{

class DiffDriveSystem : public ct::core::ControlledSystem<5, 2, double>{
public:
    static const size_t STATE_DIM = 5;
    static const size_t CONTROL_DIM = 2;

    typedef ControlledSystem<STATE_DIM,CONTROL_DIM,double> Base;
    typedef typename Base::time_t time_t;

    /*!
    */
    DiffDriveSystem(std::shared_ptr<ct::core::Controller<STATE_DIM,2,double>> controller = nullptr):
        ControlledSystem<STATE_DIM,CONTROL_DIM,double>(controller, ct::core::SYSTEM_TYPE::GENERAL){
    }

    /*!
    * @brief Copy constructor
    */
    DiffDriveSystem(const DiffDriveSystem& arg) : 
        ControlledSystem<STATE_DIM,CONTROL_DIM,double>(arg){

    }

    /*!
    * @brief Deep Cloning
    */
    DiffDriveSystem* clone() const override{
        return new DiffDriveSystem(*this);
    }

    /*!
    * @brief Deconstructor
    */
    ~DiffDriveSystem(){

    }

    /*!
    * @brief Compute the state derivatives
    * @param state The current state variable (x,y,yaw,speed,yaw angular velocity)
    * @param t Time instant at the computation (for time invariant system can be 0)
    * @param control The control command vector
    * @param derivative The derivative vector
    * 
    * @return Nothing, the raturn value is contained into derivative
    */
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM,double>& state,
        const time_t& t, 
        const ct::core::ControlVector<CONTROL_DIM, double>& control,
        ct::core::StateVector<STATE_DIM, double>& derivative){
        derivative << state(3) * std::cos(state(2)),
            state(3) * std::sin(state(2)),
            state(4),
            control(0),
            control(1);
    }
};

class DiffDriveErrorSystem : public ct::core::ControlledSystem<5, 2, double>{
public:
    static const size_t STATE_DIM = 5;
    static const size_t CONTROL_DIM = 2;
    static const size_t TRAJECTORY_DIM = 3;

    typedef ControlledSystem<STATE_DIM,CONTROL_DIM,double> Base;
    typedef typename Base::time_t time_t;

    /*!
    * @brief Standard constructor
    */
    DiffDriveErrorSystem(std::shared_ptr<ct::core::Controller<STATE_DIM,2,double>> controller = nullptr):
        ControlledSystem<STATE_DIM,CONTROL_DIM,double>(controller, ct::core::SYSTEM_TYPE::GENERAL){
        
        a.push_back(0);
        b.push_back(0);
        c.push_back(0);
    }

    /*!
    * @brief Copy constructor
    */
    DiffDriveErrorSystem(const DiffDriveSystem& arg) : 
        ControlledSystem<STATE_DIM,CONTROL_DIM,double>(arg){

    }

    /*!
    * @brief Deep Cloning
    */
    DiffDriveErrorSystem* clone() const override{
        return new DiffDriveErrorSystem(*this);
    }

    /*!
    * @brief Deconstructor
    */
    ~DiffDriveErrorSystem(){
        
    }

    /*!
    * @brief Compute the state derivatives
    * @param state The current state variable (x,y,yaw,speed,yaw angular velocity)
    * @param t Time instant at the computation (for time invariant system can be 0)
    * @param control The control command vector
    * @param derivative The derivative vector
    * 
    * @return Nothing, the raturn value is contained into derivative
    */
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM,double>& state,
        const time_t& t, 
        const ct::core::ControlVector<CONTROL_DIM, double>& control,
        ct::core::StateVector<STATE_DIM, double>& derivative){
        
        // Compute the derivative of the trajectory at t
        // The first element is ignored since we take the derivative
        trajectory(0) = 0;
        for(coeff_index = 1; coeff_index < a.size(); coeff_index++){
            trajectory(0) += (double)coeff_index * a[coeff_index] * std::pow((double)t, (coeff_index-1));
        }
        trajectory(1) = 0;
        for(coeff_index = 1; coeff_index < b.size(); coeff_index++){
            trajectory(1) += (double)coeff_index * b[coeff_index] * std::pow((double)t, (coeff_index-1));
        }
        trajectory(2) = 0;
        for(coeff_index = 1; coeff_index < c.size(); coeff_index++){
            trajectory(2) += (double)coeff_index * c[coeff_index] * std::pow((double)t, (coeff_index-1));
        }

        derivative << trajectory(0) - state(3) * std::cos(state(2)),
            trajectory(1) - state(3) * std::sin(state(2)),
            trajectory(2) - state(4),
            -control(0),
            -control(1);
    }

    /*!
    * @brief Set the coefficient of the polynomial trajectory in function of the time: x(t) y(t) yaw(t)
    */
    // TODO: check how MPC handles time, maybe we can set it back to zero each time we start an iteration
    void setTrajectoryCoefficients(const std::vector<double>& x,const std::vector<double>& y,const std::vector<double>& yaw){
        a = x;
        b = y;
        c = yaw;
    }


private:
    

    /*!
    * @brief Coefficients of the polynomial trajectory, a for x, b for y, c for theta.
    *   The coefficients start from 0 (coeff of order 0) to n (coeff of order n)
    */
    int coeff_index;
    std::vector<double> a,b,c; 
    ct::core::StateVector<TRAJECTORY_DIM, double> trajectory;
};

class DiffDriveConstraint : public ct::optcon::ConstraintBase<5,2,double>{
public:
    typedef ct::optcon::ConstraintBase<5,2,double> Base;
    typedef ct::core::ControlVector<2,double> control_vector_t;
    typedef ct::core::StateVector<5,double> state_vector_t;
    typedef Eigen::Matrix<double, -1, 1> constraint_t;
    typedef Eigen::Matrix<double, -1, -1> Jacobian_t;

    DiffDriveConstraint(double u_lb, double u_ub, double rw): _u_lb(u_lb), _u_ub(u_ub), _rw(rw){
        Base::lb_.resize(2);
        Base::ub_.resize(2);
        Base::lb_ << 2 / rw * _u_lb * constraint_t::Ones(2,1);
        Base::ub_ << 2 / rw * _u_ub * constraint_t::Ones(2,1);

        _jacobian_input = Jacobian_t::Zero(2,2);
        _jacobian_state = Jacobian_t::Zero(2,5);
    }

    DiffDriveConstraint(const DiffDriveConstraint& other){
        this->_u_lb = other._u_lb;
        this->_u_ub = other._u_ub;
        this->_rw = other._rw;
        Base::lb_.resize(2);
        Base::ub_.resize(2);
        Base::lb_ << 2 / _rw * _u_lb * Eigen::Matrix<double, -1, 1>::Ones(2,1);
        Base::ub_ << 2 / _rw * _u_ub * Eigen::Matrix<double, -1, 1>::Ones(2,1);

        _jacobian_input = Jacobian_t::Zero(2,2);
        _jacobian_state = Jacobian_t::Zero(2,5);
        _jacobian_input << 1, 1,
                            1, -1;
    }

    virtual DiffDriveConstraint* clone() const override{
        return new DiffDriveConstraint(*this);
    }

    virtual size_t getConstraintSize() const override{
        return 2;
    }

    virtual constraint_t evaluate(const state_vector_t& x, const control_vector_t& u, const double t){
        Eigen::Matrix <double,2,1> ret;
        ret << u(0) + u(1),
            u(0) - u(1);

        return ret;
    }

    virtual Jacobian_t jacobianState(const state_vector_t& x, const control_vector_t& u, double t) override{
        // The matrix is initialized as all zeros 
        return _jacobian_state;
    }

    virtual Jacobian_t jacobianInput(const state_vector_t& x, const control_vector_t& u, double t) override{
        // The matrix is constant
        return _jacobian_input;
    }

private: 
    double _u_lb, _u_ub, _rw;
    Jacobian_t _jacobian_state, _jacobian_input;
};

class DiffDriveMPCController{
public:

    typedef struct{
        Eigen::Matrix<double,2,2> R;
        Eigen::Matrix<double,5,5> Q;
        Eigen::Matrix<double,5,5> P;

        ct::core::Time time_horizon;
        double frequency;

        int mpc_iterations;
        bool mpc_verbose;

        double u_ub,u_lb;
        double rw;

    } diff_drive_param;

    typedef ct::core::StateVector<DiffDriveSystem::STATE_DIM, double> StateVector;
    typedef ct::core::ControlVector<DiffDriveSystem::CONTROL_DIM, double> ControlVector;
    typedef ct::optcon::NLOptConSolver<DiffDriveSystem::STATE_DIM, DiffDriveSystem::CONTROL_DIM>::Policy_t ControllerPolicy_t;
    typedef ct::optcon::MPC<ct::optcon::NLOptConSolver<DiffDriveSystem::STATE_DIM, DiffDriveSystem::CONTROL_DIM>> mpc_t;

    DiffDriveMPCController(diff_drive_param param){
        
        _param = param;

        const size_t state_dim = DiffDriveSystem::STATE_DIM;
        const size_t control_dim = DiffDriveSystem::CONTROL_DIM;

        std::shared_ptr<
            ct::core::ControlledSystem<
                state_dim,
                control_dim
                >
            > system_dynamics(new DiffDriveSystem());

        std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> linearizer(new ct::core::SystemLinearizer<state_dim, control_dim>(system_dynamics));


        std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediate_term(new ct::optcon::TermQuadratic<state_dim, control_dim>(param.Q,param.R));
        std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> final_term(new ct::optcon::TermQuadratic<state_dim, control_dim>(param.P,param.R)); 

        std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> cost_fun(new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
        cost_fun->addIntermediateTerm(intermediate_term);
        cost_fun->addFinalTerm(final_term);

        ct::core::Time time_horizon = param.time_horizon;
        ct::core::StateVector<state_dim, double> x0 = ct::core::StateVector<state_dim, double>::Zero();
        ct::optcon::ContinuousOptConProblem<state_dim, control_dim, double> problem(time_horizon, x0, system_dynamics, cost_fun, nullptr);
        
        // Constraints
        std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> constraint_container(new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
        
        // Differential Drive specific constraint
        std::shared_ptr<DiffDriveConstraint> model_constraint(new DiffDriveConstraint(param.u_lb, param.u_ub, param.rw));
        
        // Target constraint
        std::shared_ptr<ct::optcon::StateConstraint<state_dim, control_dim>> target_constraint(new ct::optcon::StateConstraint<state_dim, control_dim>(-1e-2*Eigen::Matrix<double,5,1>::Ones(), 1e-2*Eigen::Matrix<double,5,1>::Ones()) );

        /////// JUST TRYING
        std::shared_ptr<ct::optcon::StateConstraint<state_dim, control_dim>> bound_constraint(new ct::optcon::StateConstraint<state_dim, control_dim>(-1.5*Eigen::Matrix<double,5,1>::Ones(), 1.5*Eigen::Matrix<double,5,1>::Ones()) );
    
        constraint_container->addIntermediateConstraint(model_constraint, true);
        // constraint_container->addTerminalConstraint(target_constraint, true);

        // constraint_container->addIntermediateConstraint(bound_constraint, true);

        problem.setGeneralConstraints(constraint_container);


        // NLOPTCON problem
        ct::optcon::NLOptConSettings solver_settings;
        solver_settings.dt = 1.0/param.frequency;
        solver_settings.integrator = ct::core::IntegrationType::RK4;
        solver_settings.discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        solver_settings.max_iterations = param.mpc_iterations;
        solver_settings.nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
        solver_settings.lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;
        solver_settings.printSummary = param.mpc_verbose;


        ct::optcon::mpc_settings controller_settings;
        controller_settings.stateForwardIntegration_ = true;
        controller_settings.postTruncation_ = false;
        controller_settings.measureDelay_ = false;
        controller_settings.delayMeasurementMultiplier_ = 1.0;
        controller_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
        controller_settings.coldStart_ = true;
        int K = solver_settings.computeK(time_horizon);

        // FOR NLOPTCON
        ct::core::FeedbackArray<state_dim, control_dim> u0_fb(K, ct::core::FeedbackMatrix<state_dim, control_dim, double>::Zero());
        ct::core::ControlVectorArray<control_dim> u0_ff(K, ct::core::ControlVector<control_dim, double>::Zero());
        ct::core::StateVectorArray<state_dim> x_ref_init(K+1, x0);
        ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t controller_policy(x_ref_init, u0_ff, u0_fb, solver_settings.dt);

        // ct::optcon::NLOptConSolver<state_dim, control_dim> problem_solver(problem, solver_settings);

        mpc_solver = std::shared_ptr<ct::optcon::MPC<ct::optcon::NLOptConSolver<state_dim, control_dim>>>(new ct::optcon::MPC<ct::optcon::NLOptConSolver<state_dim, control_dim>>(problem, solver_settings, controller_settings));
        mpc_solver->getSolver().setInitialGuess(controller_policy);

        mpc_solver->setInitialGuess(controller_policy);
    }

    ControlVector update(StateVector state){

        mpc_solver->prepareIteration(0);
        mpc_solver->finishIteration(state, 0, control_policy, policy_time);

        // control_policy->computeControl(state, 0, control_action);
        // use compute control action in order to take the input?
        //update the control function

        control_action = mpc_solver->getSolver().getControlTrajectory().front();
        // Eigen::Matrix<double,5,-1> state_traj = Eigen::Matrix<double,5,-1>::Zero(5,mpc_solver->getSolver().getStateTrajectory().size());
        // for(int i = 0; i < mpc_solver->getSolver().getStateTrajectory().size(); i++){
        //     state_traj.block(0,i,5,1) = mpc_solver->getSolver().getStateTrajectory()[i];
        // }

        // std::cout << "Trajectory= [\n" << state_traj << "];" << std::endl;

        // Eigen::Matrix<double,2,-1> control_traj = Eigen::Matrix<double,2,-1>::Zero(2,mpc_solver->getSolver().getControlTrajectory().size());
        // for(int i = 0; i < mpc_solver->getSolver().getControlTrajectory().size(); i++){
        //     control_traj.block(0,i,2,1) = mpc_solver->getSolver().getControlTrajectory()[i];
        // }
        // std::cout << "control=[\n" << control_traj << "];" << std::endl;
        return control_action;
    }

    Eigen::Matrix<double, 5, -1> getStateTrajectory(){
        Eigen::Matrix<double,5,-1> state_traj = Eigen::Matrix<double,5,-1>::Zero(5,mpc_solver->getSolver().getStateTrajectory().size());
        for(int i = 0; i < mpc_solver->getSolver().getStateTrajectory().size(); i++){
            state_traj.block(0,i,5,1) = mpc_solver->getSolver().getStateTrajectory()[i];
        }
        return state_traj;
    }

    Eigen::Matrix<double, 2, -1> getControlTrajectory(){
        Eigen::Matrix<double,2,-1> control_traj = Eigen::Matrix<double,2,-1>::Zero(2,mpc_solver->getSolver().getControlTrajectory().size());
        for(int i = 0; i < mpc_solver->getSolver().getControlTrajectory().size(); i++){
            control_traj.block(0,i,2,1) = mpc_solver->getSolver().getControlTrajectory()[i];
        }
        return control_traj;
    }

    ControllerPolicy_t& getPolicy(){
        return control_policy;
    }

    std::shared_ptr<mpc_t> getMPC(){
        return mpc_solver;
    }


private:

    std::shared_ptr<mpc_t> mpc_solver;
    diff_drive_param _param;

    ControllerPolicy_t control_policy;
    double policy_time;
    ControlVector control_action;
};

class DiffDriveIPOPTProblem : public ct::optcon::tpl::DiscreteCostEvaluatorBase<double>{
public:
    DiffDriveIPOPTProblem(std::shared_ptr<ct::optcon::tpl::OptVector<double>> optVector) : {

    }


private:

    ct::opcton::tpl::OptVector<double> variables;

};

class DiffDriveIPOPTController{
public:

    typedef struct{
        Eigen::Matrix<double,2,2> R;
        Eigen::Matrix<double,5,5> Q;
        Eigen::Matrix<double,5,5> P;

        ct::core::Time time_horizon;
        double frequency;

        int mpc_iterations;
        bool mpc_verbose;

        double u_ub,u_lb;
        double rw;

    } diff_drive_param;

    typedef ct::core::StateVector<DiffDriveSystem::STATE_DIM, double> StateVector;
    typedef ct::core::ControlVector<DiffDriveSystem::CONTROL_DIM, double> ControlVector;
    typedef ct::optcon::NLOptConSolver<DiffDriveSystem::STATE_DIM, DiffDriveSystem::CONTROL_DIM>::Policy_t ControllerPolicy_t;
    typedef ct::optcon::MPC<ct::optcon::NLOptConSolver<DiffDriveSystem::STATE_DIM, DiffDriveSystem::CONTROL_DIM>> mpc_t;

    DiffDriveIPOPTController(diff_drive_param param){
        
        _param = param;

    }

    ControlVector update(StateVector state){

        mpc_solver->prepareIteration(0);
        mpc_solver->finishIteration(state, 0, control_policy, policy_time);

        // control_policy->computeControl(state, 0, control_action);
        // use compute control action in order to take the input?
        //update the control function

        control_action = mpc_solver->getSolver().getControlTrajectory().front();
        // Eigen::Matrix<double,5,-1> state_traj = Eigen::Matrix<double,5,-1>::Zero(5,mpc_solver->getSolver().getStateTrajectory().size());
        // for(int i = 0; i < mpc_solver->getSolver().getStateTrajectory().size(); i++){
        //     state_traj.block(0,i,5,1) = mpc_solver->getSolver().getStateTrajectory()[i];
        // }

        // std::cout << "Trajectory= [\n" << state_traj << "];" << std::endl;

        // Eigen::Matrix<double,2,-1> control_traj = Eigen::Matrix<double,2,-1>::Zero(2,mpc_solver->getSolver().getControlTrajectory().size());
        // for(int i = 0; i < mpc_solver->getSolver().getControlTrajectory().size(); i++){
        //     control_traj.block(0,i,2,1) = mpc_solver->getSolver().getControlTrajectory()[i];
        // }
        // std::cout << "control=[\n" << control_traj << "];" << std::endl;
        return control_action;
    }

    Eigen::Matrix<double, 5, -1> getStateTrajectory(){
        Eigen::Matrix<double,5,-1> state_traj = Eigen::Matrix<double,5,-1>::Zero(5,mpc_solver->getSolver().getStateTrajectory().size());
        for(int i = 0; i < mpc_solver->getSolver().getStateTrajectory().size(); i++){
            state_traj.block(0,i,5,1) = mpc_solver->getSolver().getStateTrajectory()[i];
        }
        return state_traj;
    }

    Eigen::Matrix<double, 2, -1> getControlTrajectory(){
        Eigen::Matrix<double,2,-1> control_traj = Eigen::Matrix<double,2,-1>::Zero(2,mpc_solver->getSolver().getControlTrajectory().size());
        for(int i = 0; i < mpc_solver->getSolver().getControlTrajectory().size(); i++){
            control_traj.block(0,i,2,1) = mpc_solver->getSolver().getControlTrajectory()[i];
        }
        return control_traj;
    }

    ControllerPolicy_t& getPolicy(){
        return control_policy;
    }

    std::shared_ptr<mpc_t> getMPC(){
        return mpc_solver;
    }


private:

    std::shared_ptr<mpc_t> mpc_solver;
    diff_drive_param _param;

    ControllerPolicy_t control_policy;
    double policy_time;
    ControlVector control_action;
};




}