#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>


#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "utils.h"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class DetectionAndControl : public rclcpp::Node
{
    public:
        DetectionAndControl()
        : Node("ros2_kdl_vision_control"), 
        node_handle_(std::shared_ptr<DetectionAndControl>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "velocity"); // defaults to "velocity"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }


            declare_parameter("task", "positioning"); // defaults to "positioning"
            get_parameter("task", task_);
            RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());

            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }



            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            aruco_state_available_ = false;

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            qi_.resize(nj);
            ex_joint_velocities_.resize(nj);
     
            joint_efforts_.resize(nj);

            joint_velocities_old_.resize(nj); 
            joint_accelerations_d_.resize(nj);
     
            des_joint_positions_.resize(nj);
            des_joint_velocities_.resize(nj);
            des_joint_accelerations_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&DetectionAndControl::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Subscriber for marker
            aruco_marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&DetectionAndControl::imageCallback2, this, std::placeholders::_1));

            // Wait for the aruco_state topic
            while(!aruco_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data (aruco) received yet! ...");
                rclcpp::spin_some(node_handle_);
            }


            // Update KDLrobot object
            
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));


            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            
            // robot_->getInverseKinematics(init_cart_pose_, qi_);
            qi_=robot_->getJntValues();


            // Initialize controller
            KDLController controller_(*robot_);
            if(task_ == "positioning"){
                // EE's trajectory initial position (just an offset)
                Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.01));

                double offset_camera = 1; // Distanza desiderata dal marker

                Eigen::Vector3d end_position;
                end_position << marker_frame_.p.data[0] + offset_camera, marker_frame_.p.data[1], marker_frame_.p.data[2];

                std::cout << "END position: " << end_position[0]<<" "<< end_position[1]<<" "<<end_position[2] << "\n";


                // Plan trajectory
                double traj_duration = 1.5, acc_duration = 0.5;
                double t= 0.0;
                //std::cout << "DEBUG: Inizio dello switch, traj_chosen = " << traj_chosen << std::endl;

                // Trajectory rectilinear cubic

                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using cubic_polynomial for rectiliniar path
                p = planner_.compute_trajectory(t);

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            }
            else if(cmd_interface_ == "effort"){
                //LOOK AT POINT

                // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

                // Plan trajectory
            double traj_duration = 1.5*3, acc_duration = 0.5*3, t = 0.0;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
            
            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t);
            

            }

            if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&DetectionAndControl::cmd_publisher, this));

                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else{   // effort
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                            std::bind(&DetectionAndControl::cmd_publisher, this));
            
                // Send joint effort commands
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            KDLController controller_(*robot_);
            iteration_ = iteration_ + 1;
            double total_time;
            double dt;

            if(task_=="positioning"){
            
            total_time = 1.5; // 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            dt = 1.0 / loop_rate;
            t_+=dt;
            }else{

            // define trajectory
            total_time = 1.5*3; // 
            int trajectory_len = 150*3; // 
            int loop_rate = trajectory_len / total_time;
            dt = 1.0 / loop_rate;
            t_+=dt;
            }

            if (t_ < total_time)        // until the trajectory hasn't finished
            {
                if(task_ == "positioning") {
                    if(cmd_interface_ == "velocity"){
                        if(t_ <= total_time) {
                            p = planner_.compute_trajectory(t_);
                            if(t_ >= total_time - dt) {                     // last dt before the end of the trajectory
                                p = planner_.compute_trajectory(t_);
                                final_pos = p;
                            }
                        }
                        else {
                            // std::cout << "tempo attuale" << t_;
                            p.pos = final_pos.pos;
                            p.vel = Eigen::Vector3d::Zero();
                            p.acc = Eigen::Vector3d::Zero();
                        }

                        // Compute EE frame
                        KDL::Frame cartpos = robot_->getEEFrame();           

                        KDL::Rotation y_rotation = KDL::Rotation::RotY(M_PI);
                        KDL::Rotation marker_frame_rotated;
                        marker_frame_rotated = marker_frame_.M * y_rotation;



                        // compute errors
                        Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                        Eigen::Vector3d o_error = computeOrientationError(toEigen(marker_frame_rotated), toEigen(cartpos.M));
                        std::cout << "The error norm is : " << error.norm() << std::endl;

                        // Compute differential IK
                        Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                        joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    }
                } else if(task_ == "look-at-point"){
                    //LOOK AT POINT
                    Eigen::Vector3d sd(0, 0, 1);
                    //Calcolo vettore normalizzato 
                    Eigen::Vector3d s(marker_frame_.p.x(), marker_frame_.p.y(), marker_frame_.p.z());
                    



                    // Calcola la norma di P0
                    double norm_s = s.norm();
                    s.normalize();
                    //Calcolo matrice S
                    Eigen::Matrix3d S = skew(s);

                    //Calcolo R
                    KDL::Frame cartpos = robot_->getEEFrame();
                    Eigen::Matrix3d Rc = toEigen(marker_frame_.M);
                   
                    Matrix6d R = Matrix6d::Zero();

                    R.block(0,0,3,3)=Rc;
                    R.block(3,3,3,3)=Rc;
                

                    
                    std::cout << "Matrice 6x6 R:\n" << R << std::endl;

                    //Calcolo di L(s)

                    // Calcola I - ss^T
                    Eigen::Matrix3d ssT = s * s.transpose();
                    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
                    Eigen::Matrix3d I_minus_ssT = I3 - ssT;
                 
                    Eigen::Matrix3d scaled_matrix = (-1.0 / norm_s) * I_minus_ssT;

                    Eigen::MatrixXd L  = Eigen::Matrix<double,3,6>::Zero();
                    L.block(0,0,3,3)=scaled_matrix;
                    L.block(0,3,3,3)=S;
                    L=L*R.transpose();

                        
                    Eigen::MatrixXd Jc = robot_->getEEJacobian().data;

                    Eigen::MatrixXd LJ_PseudoInv = pseudoinverse(L*Jc);
                 
                    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(robot_->getNrJnts(), robot_->getNrJnts()) - pseudoinverse(L*Jc) * L*Jc;

                    Eigen::Vector3d error_s = sd - s;

                    
                    Eigen::VectorXd q=robot_->getJntValues();
                    if(cmd_interface_ == "velocity"){
                        //Control law 
                        joint_velocities_.data = 5*LJ_PseudoInv*sd + N* (qi_- q);
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*0.02;
                    }
                    else{ //effort
  
                            KDL::Frame end_effector_frame;
   
                            KDL::Rotation rotation_cam_ee = KDL::Rotation::RPY(0, -1.57, 3.14);
                           

                            Eigen::Vector3d s_axis = sd.cross(s); 
                            double s_angle =std::acos(sd.dot(s)); 

                            KDL::Frame Marker_frame;
                            Marker_frame.M =(robot_->getEEFrame()).M*rotation_cam_ee*(KDL::Rotation::Rot(toKDL(s_axis), s_angle)); 

   
                            KDL::Rotation marker_frame_rotated;
                            marker_frame_rotated = end_effector_frame.M ;//* y_rotation;

                        


                        // Compute EE frame
                        KDL::Frame cartpos = robot_->getEEFrame();  
                        trajectory_point p1= planner_.compute_trajectory(t_);        

                        // compute errors
                        Eigen::Vector3d error = computeLinearError(p1.pos, Eigen::Vector3d(cartpos.p.data));
                        Eigen::Vector3d o_error = computeOrientationError(toEigen(Marker_frame.M), toEigen(cartpos.M));
                        


                        //JOINT SPACE INVERSE DYNAMICS CONTROL

                        // Compute differential IK

                        joint_velocities_old_.data = joint_velocities_.data;

                        Vector6d cartvel; cartvel << p.vel + 5*error, 20*o_error;
                        joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                        joint_accelerations_d_.data = (joint_velocities_.data - joint_velocities_old_.data)/dt;

                        
                        KDLController controller_(*robot_);
                        joint_efforts_.data = controller_.idCntr(joint_positions_, joint_velocities_, joint_accelerations_d_, 50.0, 5.0);

                        KDL::Frame frame_final = robot_->getEEFrame();
                        KDL::Twist velocities_final; velocities_final.vel=KDL::Vector::Zero(); velocities_final.rot=KDL::Vector::Zero();
                        KDL::Twist acceleration_final; acceleration_final.vel=KDL::Vector::Zero(); acceleration_final.rot=KDL::Vector::Zero();


                    }
                    

                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "velocity") {
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else{   // effort
                    // Send joint effort commands
                    for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                if(cmd_interface_ =="velocity"){
                    //Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                    if(task_ == "look-at-point"){
                        t_=0;
                    }
                }

                if(cmd_interface_ =="effort")
                {   
                    KDLController controller_(*robot_);
                    des_joint_velocities_.data=Eigen::VectorXd::Zero(7,1);
                    des_joint_accelerations_.data=Eigen::VectorXd::Zero(7,1);
                    
                    joint_efforts_.data=controller_.KDLController::idCntr(joint_positions_,des_joint_velocities_, des_joint_accelerations_, 50.0, 5.0); 

                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                    
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_.data[i];             
                    }
                }   
            // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

        
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
                joint_efforts_.data[i] = sensor_msg.effort[i];
            }
        }

        void imageCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr p_msg) {

            

            KDL::Vector marker_position;
            Eigen::VectorXd marker_quaternion;
            marker_quaternion.resize(4);

            marker_position[0] = p_msg->pose.position.x;
            marker_position[1] = p_msg->pose.position.y;
            marker_position[2] = p_msg->pose.position.z;

            marker_quaternion[0] = p_msg->pose.orientation.x;
            marker_quaternion[1] = p_msg->pose.orientation.y;
            marker_quaternion[2] = p_msg->pose.orientation.z;
            marker_quaternion[3] = p_msg->pose.orientation.w;

            KDL::Frame marker_frame;
            marker_frame.M = KDL::Rotation::Quaternion(marker_quaternion[0],marker_quaternion[1],marker_quaternion[2],marker_quaternion[3]);


            marker_frame.M = marker_frame.M ;//* y_rotation;
            marker_frame.p = marker_position;

            // marker_frame_ =robot_->getEEFrame() * marker_frame;
            marker_frame_ = marker_frame;

            // std::cout<<marker_position[2]<<"\n";
            std::cout << "X position: " << marker_position[0] << "\n";
            std::cout << "Y position: " << marker_position[1] << "\n";
            std::cout << "Z position: " << marker_position[2] << "\n";

            aruco_state_available_ = true;
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_marker_pose_sub_;

        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_efforts_;

        KDL::Frame marker_frame_;

        trajectory_point p;
        trajectory_point final_pos;


        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        int iteration_;
        bool joint_state_available_;

        bool aruco_state_available_;

        double t_;
        std::string cmd_interface_;
        std::string task_;
        KDL::Frame init_cart_pose_;
        Eigen::VectorXd qi_;

        KDL::JntArray joint_accelerations_d_;
        KDL::JntArray joint_velocities_old_;
        KDL::JntArray des_joint_positions_;
        KDL::JntArray des_joint_velocities_;
        KDL::JntArray des_joint_accelerations_;
        KDL::JntArray ex_joint_velocities_;
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionAndControl>());
    rclcpp::shutdown();
    return 1;
}


