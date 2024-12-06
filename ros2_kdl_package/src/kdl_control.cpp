#include "kdl_control.h"

KDLController::KDLController()
{
  robot_=nullptr;
}


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis(); //+ robot_->getGravity() /*friction compensation?*/;   // gravity has been set to 0 in the .world
}




Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // read current state
    KDL::Frame x = robot_->getEEFrame();
    KDL::Twist dx = robot_->getEEVelocity();

    Vector6d x_tilde, dot_x_tilde;

    Eigen::Matrix<double,6,6> Kp;
    Eigen::Matrix<double,6,6> Kd;
    Kp.setZero();
    Kd.setZero();

    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

    computeErrors(_desPos,x,_desVel,dx,x_tilde,dot_x_tilde);

    for (unsigned int i=0;i<3;i++){x_tilde(i)=_Kpp*x_tilde(i);dot_x_tilde(i)=_Kdp*dot_x_tilde(i);}
    for (unsigned int i=3;i<6;i++){x_tilde(i)=_Kpo*x_tilde(i);dot_x_tilde(i)=_Kdo*dot_x_tilde(i);}
    
    Eigen::Matrix<double,7,1> y;
    y = pseudoinverse(robot_->getEEJacobian().data)*(toEigen(_desAcc)+x_tilde+dot_x_tilde-robot_->getEEJacDotqDot());
    
    return  robot_->getJsim()*y + robot_->getCoriolis(); //+ robot_->getGravity();      // gravity has been set to 0 in the .world
}
