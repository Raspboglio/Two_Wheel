#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <two_wheel_control_msgs/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Dense>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

namespace two_wheel_controller{

    class TwoWheelMPCController : public controller_interface::ControllerInterface{
    
    public:
    
        TwoWheelMPCController();
        ~TwoWheelMPCController();
        
        
        controller_interface::return_type init(const std::string &controller_name) override;
        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::return_type update() override;


    private:
    
        Eigen::Matrix<double, 4, 4> A_gen,A;
        Eigen::Matrix<double, 4, 4> T;
        Eigen::Matrix<double, 4, 2> B;
        Eigen::Matrix<double, 4, 4> C; 
        Eigen::Matrix<double, 4, 2> D;
        Eigen::Matrix<double, 4, 4> Q;
        Eigen::Matrix<double, 2, 2> R;
        Eigen::Matrix<double, 3, 1> pos, last_pos;
        Eigen::Matrix<double, 4, 1> state, goal, error;
        ct::core::ControlVector<2,double> control;

        std::unique_ptr<ct::optcon::MPC<ct::optcon::NLOptConSolver<4, 2>>> _mpc;
        ct::core::StateFeedbackController<4, 2> new_policy;
        ct::core::Time ts_new_policy;
        bool success;
        ct::core::Time start_time, cur_time;
        // TODO: decide what to limit as inequalities
        
        
        double max_angle, l, update_rate, w, x, y, z;

        std::unique_ptr<rclcpp::Duration> period;


        rclcpp::Publisher<two_wheel_control_msgs::msg::State>::SharedPtr _state_pub;
        two_wheel_control_msgs::msg::State _pub_msg;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _sub;
        void subCallback(const geometry_msgs::msg::Twist::SharedPtr msg);


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
        geometry_msgs::msg::Pose _last_pose;
        nav_msgs::msg::Odometry _odom_msg;
        void updateOdom();

    };

    
    // states are:    thetay, wr, wl, omegay
    class TwoWheelDynamic : public ct::core::ControlledSystem<4, 2>{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            static const size_t STATE_DIM = 4;
            static const size_t CON_DIM = 2;

            typedef ControlledSystem<STATE_DIM, CON_DIM> Base;
            typedef typename Base::time_t time_t;   

            TwoWheelDynamic() = delete;

            TwoWheelDynamic(std::vector<double> I_b,
                std::vector<double> I_w, 
                double m_b,
                double m_w, 
                double l, 
                double h,
                double r,
                double d, 
                std::shared_ptr<ct::core::Controller<STATE_DIM, CON_DIM>> controller = nullptr) : ControlledSystem<STATE_DIM, CON_DIM>(controller){
                
                
                this->I_b = I_b;
                this->I_w = I_w;
                this->m_b = m_b;
                this->m_w = m_w;
                this->l = l;
                this->h = h;
                this->r = r;
                this->d = d;

                double Gamma_w_yy = I_w[1];
                double Gamma_w_zz = I_w[2];
                double Gamma_b_yy = I_b[1];
                double Gamma_b_zz = I_b[2];

                // B <<
                //     0, I_w[1] + 3 * m_w, -l * m_w, 0,
                //     0, 0, 0, I_b[2] + I_w[2] * 2 + d * d * m_w/2 + I_w[1] * d * d/(2 * r * r);

                // B_inv <<
                //     0, 0,
                //     (m_w * l * l + I_b[1])/(2 * l * l * m_w * m_w + I_w[1] * l * l * m_w + 3 * I_b[1] * m_w + I_b[1] * I_w[1]), 0,
                //     (l * m_w)/( 2 * l * l * m_w * m_w + I_w[1] * l * l * m_w + 3 * I_b[1] * m_w + I_b[1] * I_w[1]), 0,
                //     0,  1/(I_b[2] + 2 * I_b[2] + (d * d * m_w) / 2 + (I_w[1] * d * d)/(2 * r * r));

                B << 
                    0, 2 * m_w + m_b + 2 * I_w[1] / (r*r), -l * m_b, 0,
                    0, 0, 0, I_b[2] + 2 * I_w[2] + d * d * m_w / 2 + I_w[2] * d * d / (2 * r * r);

                B_inv << 
                    0, 0,
                    r * r * ( m_b * l * l + I_b[1]) / (2*I_b[1] * I_w[1] + 2 * I_w[1] * l * l * m_b + I_b[1] * m_b * r * r + 2 * I_b[1] * m_w * r * r + 2 * l * l * m_b * m_w * r * r), 0,
                    ( l * m_b * r * r) / (2 * I_b[1] * I_w[1] + 2 * I_w[1] * l * l * m_b + I_b[1] * m_b * r * r + 2 * I_b[1] * m_w * r * r + 2 * l * l * m_b *m_w *r * r), 0,
                    0, ( 2 * r * r)/(I_w[1] * d * d + 2 * I_b[1] * r * r + 4 * I_w[1] * r * r + d * d * m_w * r * r);

                T << 
                    1, 0, 0, 0,
                    0, 1/r, -1, d/(2*r),
                    0, 1/r, -1, -d/(2*r),
                    0, 0, 1, 0;

               T_tau_inv <<
                    (m_b+m_w) * r/(2 * I_w[1]), (m_b + m_w) * r/(2 * I_w[1]),
                    (I_b[2] + 2 * I_w[2] + ( d * d * m_w)/2 + (I_w[2] * d * d)/(2 * r * r))/(I_w[2]*d*r), -(I_b[2] + 2 * I_w[2] + ( d * d * m_w)/2 + (I_w[2] * d * d)/(2 * r * r))/(I_w[2]*d*r); 

                A <<
                    0, 0, 0, 1,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0;

                B_real <<
                    0, I_w[1] + I_b[2] * d * d / 4  + I_w[2] * d * d / 2 + m_b * r * r / 4 + m_w * r * r, m_b * r * r / 4 - I_w[2] * d * d / 2 - I_b[2] * d * d / 4, m_b * r * (l - r) / 2 - m_w * r * r - I_w[1],
                    0, m_b * r * r / 4 - I_w[2] * d * d / 2 - I_b[2] * d * d / 4, I_w[1] + I_b[2] * d * d / 4  + I_w[2] * d * d / 2 + m_b * r * r / 4 + m_w * r * r, m_b * r * (l - r) / 2 - m_w * r * r - I_w[1];

                B_inv_real <<
                    0, 0,
                    (4*Gamma_w_yy * Gamma_w_yy + 4*Gamma_b_yy*Gamma_w_yy + 4*m_w * m_w * std::pow(r,4) + 2 * m_b * m_w * std::pow(r,4) + Gamma_b_yy*Gamma_b_zz*d*d + 2*Gamma_b_yy*Gamma_w_zz*d*d + 2*Gamma_b_zz*Gamma_w_yy*d*d + 4*Gamma_w_yy*Gamma_w_zz*d*d + 4*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_w_yy*m_b*r*r + 4*Gamma_b_yy*m_w*r*r + 8*Gamma_w_yy*m_w*r*r - 4*l*m_b*m_w*std::pow(r,3) + Gamma_b_zz*d*d*l*l*m_b + 2*Gamma_w_zz*d*d*l*l*m_b + Gamma_b_zz*d*d*m_b*r*r + 2*Gamma_w_zz*d*d*m_b*r*r + 2*Gamma_b_zz*d*d*m_w*r*r + 4*Gamma_w_zz*d*d*m_w*r*r + 4*l*l*m_b*m_w*r*r - 4*Gamma_w_yy*l*m_b*r - 2*Gamma_b_zz*d*d*l*m_b*r - 4*Gamma_w_zz*d*d*l*m_b*r)/((2*Gamma_w_yy + Gamma_b_zz*d*d + 2*Gamma_w_zz*d*d + 2*m_w*r*r)*(2*Gamma_b_yy*Gamma_w_yy + 2*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_b_yy*m_w*r*r + 2*l*l*m_b*m_w*r*r)), (4*Gamma_w_yy*Gamma_w_yy + 4*m_w*m_w*std::pow(r,4) + 2*m_b*m_w*std::pow(r,4) + Gamma_b_yy*Gamma_b_zz*d*d + 2*Gamma_b_yy*Gamma_w_zz*d*d + 2*Gamma_b_zz*Gamma_w_yy*d*d + 4*Gamma_w_yy*Gamma_w_zz*d*d - Gamma_b_yy*m_b*r*r + 2*Gamma_w_yy*m_b*r*r + 8*Gamma_w_yy*m_w*r*r - 4*l*m_b*m_w*std::pow(r,3) + Gamma_b_zz*d*d*l*l*m_b + 2*Gamma_w_zz*d*d*l*l*m_b + Gamma_b_zz*d*d*m_b*r*r + 2*Gamma_w_zz*d*d*m_b*r*r + 2*Gamma_b_zz*d*d*m_w*r*r + 4*Gamma_w_zz*d*d*m_w*r*r - 4*Gamma_w_yy*l*m_b*r - 2*Gamma_b_zz*d*d*l*m_b*r - 4*Gamma_w_zz*d*d*l*m_b*r)/((2*Gamma_w_yy + Gamma_b_zz*d*d + 2*Gamma_w_zz*d*d + 2*m_w*r*r)*(2*Gamma_b_yy*Gamma_w_yy + 2*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_b_yy*m_w*r*r + 2*l*l*m_b*m_w*r*r)),
                    (4*Gamma_w_yy*Gamma_w_yy + 4*m_w*m_w*std::pow(r,4) + 2*m_b*m_w*std::pow(r,4) + Gamma_b_yy*Gamma_b_zz*d*d + 2*Gamma_b_yy*Gamma_w_zz*d*d + 2*Gamma_b_zz*Gamma_w_yy*d*d + 4*Gamma_w_yy*Gamma_w_zz*d*d - Gamma_b_yy*m_b*r*r + 2*Gamma_w_yy*m_b*r*r + 8*Gamma_w_yy*m_w*r*r - 4*l*m_b*m_w*std::pow(r,3) + Gamma_b_zz*d*d*l*l*m_b + 2*Gamma_w_zz*d*d*l*l*m_b + Gamma_b_zz*d*d*m_b*r*r + 2*Gamma_w_zz*d*d*m_b*r*r + 2*Gamma_b_zz*d*d*m_w*r*r + 4*Gamma_w_zz*d*d*m_w*r*r - 4*Gamma_w_yy*l*m_b*r - 2*Gamma_b_zz*d*d*l*m_b*r - 4*Gamma_w_zz*d*d*l*m_b*r)/((2*Gamma_w_yy + Gamma_b_zz*d*d + 2*Gamma_w_zz*d*d + 2*m_w*r*r)*(2*Gamma_b_yy*Gamma_w_yy + 2*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_b_yy*m_w*r*r + 2*l*l*m_b*m_w*r*r)),(4*Gamma_w_yy * Gamma_w_yy + 4*Gamma_b_yy*Gamma_w_yy + 4*m_w * m_w * std::pow(r,4) + 2 * m_b * m_w * std::pow(r,4) + Gamma_b_yy*Gamma_b_zz*d*d + 2*Gamma_b_yy*Gamma_w_zz*d*d + 2*Gamma_b_zz*Gamma_w_yy*d*d + 4*Gamma_w_yy*Gamma_w_zz*d*d + 4*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_w_yy*m_b*r*r + 4*Gamma_b_yy*m_w*r*r + 8*Gamma_w_yy*m_w*r*r - 4*l*m_b*m_w*std::pow(r,3) + Gamma_b_zz*d*d*l*l*m_b + 2*Gamma_w_zz*d*d*l*l*m_b + Gamma_b_zz*d*d*m_b*r*r + 2*Gamma_w_zz*d*d*m_b*r*r + 2*Gamma_b_zz*d*d*m_w*r*r + 4*Gamma_w_zz*d*d*m_w*r*r + 4*l*l*m_b*m_w*r*r - 4*Gamma_w_yy*l*m_b*r - 2*Gamma_b_zz*d*d*l*m_b*r - 4*Gamma_w_zz*d*d*l*m_b*r)/((2*Gamma_w_yy + Gamma_b_zz*d*d + 2*Gamma_w_zz*d*d + 2*m_w*r*r)*(2*Gamma_b_yy*Gamma_w_yy + 2*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_b_yy*m_w*r*r + 2*l*l*m_b*m_w*r*r)),
                    (2*Gamma_w_yy + m_b*r*r + 2*m_w*r*r - l*m_b*r)/(2*Gamma_b_yy*Gamma_w_yy + 2*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_b_yy*m_w*r*r + 2*l*l*m_b*m_w*r*r), (2*Gamma_w_yy + m_b*r*r + 2*m_w*r*r - l*m_b*r)/(2*Gamma_b_yy*Gamma_w_yy + 2*Gamma_w_yy*l*l*m_b + Gamma_b_yy*m_b*r*r + 2*Gamma_b_yy*m_w*r*r + 2*l*l*m_b*m_w*r*r);


                std::cout << "B\n" << B << "\nB_inv\n" << B_inv << "\nT\n" << T << "\nT_tau_inv\n" << T_tau_inv << std::endl;
                std::cout << "T B_inv\n" << T*B_inv << "\nT B_inv T_tau_inv\n" << T * B_inv * T_tau_inv << std::endl;
                std::cout << "B T_inv\n" << B*T.inverse() << std::endl;
                std::cout << "B_real\n" << B_real << "\nB_real_inv\n" << B_inv_real << std::endl;
            
            }

            TwoWheelDynamic(const TwoWheelDynamic &other){
                this->I_b = other.I_b;
                this->I_w = other.I_w;
                this->m_b = other.m_b;
                this->m_w = other.m_w;
                this->l = other.l;
                this->h = other.h;
                this->r = other.r;
                this->d = other.d;
                this->A = other.A;
                this->B = other.B;
                this->B_inv = other.B_inv;
                this->T = other.T;
                this->T_tau_inv = other.T_tau_inv;
                this->B_real = other.B_real;
                this->B_inv_real = other.B_inv_real;
            }

            ~TwoWheelDynamic() = default;

            TwoWheelDynamic* clone() const override{
                return new TwoWheelDynamic(*this);
            }


            // TODO: now it's linearized around theta_y = 0, make it not linear
            void computeControlledDynamics(const ct::core::StateVector<STATE_DIM> &x, const ct::core::Time &t, const ct::core::ControlVector<CON_DIM> &control , ct::core::StateVector<STATE_DIM>& derivative) override{
                
                // derivative(0) = x(3);
                // derivative(1) = control(0) / _I_wy; // control(0) / I_wy - Ib/Iw * derivative(3);
                // derivative(2) = control(1) / _I_wy; // control(1) / I_wy - Ib/Iw * derivative(3);
                // derivative(3) = - (control(0) + control(1)) / _I_wy * _r / 2 * (2 * _m_w + _m_b) * _l / _I_by + _I_by * _h * _m_b * 9.81 * std::sin(x(0)); //(x(4) + x(5)) / r * I_by/m_tot/l*2 + I_by * h * m_b * 9.81 * std::sin(x(0)) + control(0)/I_by_transl + control(1)/I_by_transl + Iw/I_by_transl*x(4) + Iw/I_by_transl*x(5) 
                // derivative(3) = - (control(0) + control(1)) * 1e-10 + _I_by * _h * _m_b * 9.81 * std::sin(x(0));
                
                // derivative = A * x + B_inv_real * control;    
                // derivative(3) +=  h * m_b * 9.81 * std::sin(x(0)) / I_b[1];
                
                // Computed in matlab
                // derivative(0)=x(3);
                // derivative(1)=(-(4000000*((1692209*std::cos(x(0)))/20000000 - (10201*std::pow(std::cos(x(0)),2))/10000 + 7010212923/4000000000))/(170241*(std::cos(x(0))/2000 + std::pow(std::cos(x(0)),2) - 567/400)))*control(0) +  (-(4000000*((1712611*std::cos(x(0)))/20000000 + (10201*std::pow(std::cos(x(0)),2))/10000 - 4557721077/4000000000))/(170241*(std::cos(x(0))/2000 + std::pow(std::cos(x(0)),2) - 567/400)))*control(1);
                // derivative(2)=(-(4000000*((1712611*std::cos(x(0)))/20000000 + (10201*std::pow(std::cos(x(0)),2))/10000 - 4557721077/4000000000))/(170241*(std::cos(x(0))/2000 + std::pow(std::cos(x(0)),2) - 567/400)))*control(0) + (-(4000000*((1692209*std::cos(x(0)))/20000000 - (10201*std::pow(std::cos(x(0)),2)/10000 + 7010212923/4000000000))/(170241*(std::cos(x(0))/2000 + std::pow(std::cos(x(0)),2) - 567/400))))*control(1);
                // derivative(3)=(std::cos(x(0)) + 81/400)/(std::cos(x(0))/2000 + std::pow(std::cos(x(0)),2) - 567/400)*(control(0) + control(1));


                // linearized
                derivative(0)=x(3);
                derivative(1)=46.0379*control(0) - 1.8988*control(1);
                derivative(2)=-1.8988*control(0) + 46.0379*control(1);
                derivative(3)=-2.8837*(control(0)+control(1)) + h * m_b * 9.81 * std::sin(x(0)) / I_b[1];

            }


        private:

            Eigen::Matrix<double, 4, 4> A;
            Eigen::Matrix<double, 2, 4> B;
            Eigen::Matrix<double, 4, 2> B_inv, B_inv_real;
            Eigen::Matrix<double, 4, 4> T;
            Eigen::Matrix<double, 2, 2> T_tau_inv;

            Eigen::Matrix<double, 2, 4> B_real;
            std::vector<double> I_b, I_w;
            double l, d, h, r, m_w, m_b;

            
    };
    
}