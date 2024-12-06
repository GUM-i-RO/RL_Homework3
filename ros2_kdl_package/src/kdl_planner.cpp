// #include "kdl_planner.h"


// KDLPlanner::KDLPlanner(){}


// KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
// {
//     velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
// }


// KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
// {
//     trajDuration_ = _trajDuration;
//     trajInit_ = _trajInit;
//     trajEnd_ = _trajEnd;
//     trajRadius_ = 0.0;
// }

// KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
// {
//     trajDuration_ = _trajDuration;
//     trajInit_ = _trajInit;
//     trajRadius_ = _trajRadius;
//     trajEnd_ = Eigen::Vector3d::Zero();
// }

// void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
//                                             double _radius, double _eqRadius
//                                             )
// {
//     path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

//     for (unsigned int i = 0; i < _frames.size(); i++)
//     {
//         path_->Add(_frames[i]);
//     }
//     path_->Finish();

//     velpref_->SetProfile(0,path_->PathLength());
//     traject_ = new KDL::Trajectory_Segment(path_, velpref_);
// }

// void KDLPlanner::createCircPath(KDL::Frame &_F_start,
//                                 KDL::Vector &_V_centre,
//                                 KDL::Vector& _V_base_p,
//                                 KDL::Rotation& _R_base_end,
//                                 double alpha,
//                                 double eqradius
//                                 )
// {
//     KDL::RotationalInterpolation_SingleAxis* otraj;
//     otraj = new KDL::RotationalInterpolation_SingleAxis();
//     otraj->SetStartEnd(_F_start.M,_R_base_end);
//     path_circle_ = new KDL::Path_Circle(_F_start,
//                                         _V_centre,
//                                         _V_base_p,
//                                         _R_base_end,
//                                         alpha,
//                                         otraj,
//                                         eqradius);
//     velpref_->SetProfile(0,path_circle_->PathLength());
//     traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
// }

// KDL::Trajectory* KDLPlanner::getTrajectory()
// {
// 	return traject_;
// }

// trajectory_point KDLPlanner::compute_trajectory(double time, double accDuration) //for rectilinear path and trapezoidal
// {
//   double s,s_dot,s_ddot;
//   this->trapezoidal_vel(time, accDuration, s, s_dot, s_ddot);
//   trajectory_point traj;
  

//   traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
//   traj.vel = s_dot*(trajEnd_-trajInit_);
//   traj.acc = s_ddot*(trajEnd_-trajInit_);
//   return traj;

// }

// trajectory_point KDLPlanner::compute_trajectory(double time)  //for rectilinear path and cubic_polynomial
// {
//   double s,s_dot,s_ddot;
//   this->cubic_polynomial(time, s, s_dot, s_ddot);
//   trajectory_point traj;
  

//   traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
//   traj.vel = s_dot*(trajEnd_-trajInit_);
//   traj.acc = s_ddot*(trajEnd_-trajInit_);
//   return traj;

// }

// trajectory_point KDLPlanner::compute_trajectory_circ(double time, double accDuration)  //for circular path and trapezoidal
// {
//   double s,s_dot,s_ddot;
//   this->trapezoidal_vel(time, accDuration, s, s_dot, s_ddot);
//   trajectory_point traj;
  
  

//   traj.pos.x() = trajInit_.x();
//   traj.pos.y() = trajInit_.y() - trajRadius_*cos(2*M_PI*s);
//   traj.pos.z() = trajInit_.z() - trajRadius_*sin(2*M_PI*s);
//   traj.vel.x() = 0;  //s_dot*(trajEnd_-trajInit_);
//   traj.vel.y() = trajRadius_*sin(2*M_PI*s)*2*M_PI*s_dot;
//   traj.vel.z() = -trajRadius_*cos(2*M_PI*s)*2*M_PI*s_dot;
//   traj.acc.x() = 0;
//   traj.acc.y() = -trajRadius_*2*M_PI*(-2*M_PI*sin(2*M_PI*s))*pow(s_dot,2)+cos(2*M_PI*s)*s_ddot;
//   traj.acc.z() = -trajRadius_*2*M_PI*(-trajRadius_*sin(2*M_PI*s)*pow(s_dot,2)+cos(2*M_PI*s)*s_ddot);

//   //traj.acc = s_ddot*(trajEnd_-trajInit_);
//   return traj;

// }

// trajectory_point KDLPlanner::compute_trajectory_circ(double time)  //for circular path and trapezoidal
// {
//   double s,s_dot,s_ddot;
//   this->cubic_polynomial(time, s, s_dot, s_ddot);
//   trajectory_point traj;
  
  

//   traj.pos.x() = trajInit_.x();
//   traj.pos.y() = trajInit_.y() - trajRadius_*cos(2*M_PI*s);
//   traj.pos.z() = trajInit_.z() - trajRadius_*sin(2*M_PI*s);
//   traj.vel.x() = 0;  //s_dot*(trajEnd_-trajInit_);
//   traj.vel.y() = trajRadius_*sin(2*M_PI*s)*2*M_PI*s_dot;
//   traj.vel.z() = -trajRadius_*cos(2*M_PI*s)*2*M_PI*s_dot;
//   traj.acc.x() = 0;
//   traj.acc.y() = -trajRadius_*2*M_PI*(-2*M_PI*sin(2*M_PI*s))*pow(s_dot,2)+cos(2*M_PI*s)*s_ddot;
//   traj.acc.z() = -trajRadius_*2*M_PI*(-trajRadius_*sin(2*M_PI*s)*pow(s_dot,2)+cos(2*M_PI*s)*s_ddot);

//   //traj.acc = s_ddot*(trajEnd_-trajInit_);
//   return traj;

// }



// void KDLPlanner::trapezoidal_vel(double time, double accDuration, double& s, double& s_dot, double& s_ddot){
//   /* trapezoidal velocity profile with accDuration acceleration time period and trajDuration_ total duration.
//      time = current time
//      trajDuration_  = final time
//      accDuration   = acceleration time
//      trajInit_ = trajectory initial point
//      trajEnd_  = trajectory final point */  
  
//  // Eigen::Vector3d sc_ddot = -1.0/(std::pow(accDuration,2)-trajDuration_*accDuration)*(trajEnd_-trajInit_);
//   double sc_ddot=-1.0/(std::pow(accDuration,2)-trajDuration_*accDuration);
//   if(time <= accDuration)
//   {
//     s = 0.5*sc_ddot*std::pow(time,2);
//     s_dot = sc_ddot*time;
//     s_ddot = sc_ddot;
//   }
//   else if(time <= trajDuration_-accDuration)
//   {
//     s = sc_ddot*accDuration*(time-(accDuration/2));
//     s_dot = sc_ddot*accDuration;
//     s_ddot = 0.0;
//   }
//   else
//   {
//     s = 1 - 0.5*sc_ddot*std::pow(trajDuration_-time,2);
//     s_dot = sc_ddot*(trajDuration_-time);
//     s_ddot = -sc_ddot;
//   }
// }

// void KDLPlanner::cubic_polynomial(double t, double& s, double& s_dot, double& s_ddot){
//   double a0, a1, a2, a3, s0, sDot0, sDotf, sf; 

//   s0 = 0;
//   sDot0 = 0;
//   sDotf = 0;
//   sf = 1;

//   a0 = s0;
//   a1 = sDot0;
//   a2 = (3 * (sf - s0) / (trajDuration_ * trajDuration_)) - ((2 * sDot0 + sDotf) / trajDuration_);
//   a3 = (-2 * (sf - s0) / (trajDuration_ * trajDuration_ * trajDuration_)) + ((sDot0 + sDotf) / (trajDuration_ * trajDuration_));


//   s = a3 * t * t * t + a2 * t * t + a1 * t + a0;
//   s_dot = 3 * a3 * t * t + 2 * a2 * t + a1;
//   s_ddot = 6 * a3 * t + 2 * a2;
// }

#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}



