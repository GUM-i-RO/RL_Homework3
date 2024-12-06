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
     
            joint_efforts_.resize(nj);

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
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;


            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.01));

            // EE's trajectory end position (just opposite y)
            //Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];
            // CORREGGERE END POSITION
            double z_target = 1; // Distanza desiderata dal marker

            Eigen::Vector3d end_position;
            end_position << marker_frame_.p.data[0] + z_target, marker_frame_.p.data[1], marker_frame_.p.data[2];

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

            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&DetectionAndControl::cmd_publisher, this));

            // Send joint velocity commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){


            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; // 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;

            if (t_ < total_time)        // until the trajectory hasn't finished
            {
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

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(marker_frame_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                // Compute differential IK
                Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
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
                }
            // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
                joint_efforts_.data[i] = sensor_msg.effort[i];
            }
        }

        void imageCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr p_msg) {

            KDL::Rotation y_rotation = KDL::Rotation::RotY(M_PI);
            KDL::Rotation x_rotation = KDL::Rotation::RotX(M_PI);
            KDL::Rotation z_rotation = KDL::Rotation::RotZ(M_PI);

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

            // KDL::Frame aruco_world;

            marker_frame.M = marker_frame.M*y_rotation;
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
        KDL::Frame init_cart_pose_;
};

int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionAndControl>());
    rclcpp::shutdown();
    return 1;
}