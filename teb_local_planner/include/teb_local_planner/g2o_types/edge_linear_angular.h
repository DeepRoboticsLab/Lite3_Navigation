
#pragma once

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include <math.h>

#include <iostream>

namespace teb_local_planner
{

class EdgeLinearAngular : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */	      
  EdgeLinearAngular()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    
    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();
    
    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    double vel = dist / deltaT->estimate();
    
//     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction
    
    const double omega = angle_diff / deltaT->estimate();
  
    _error[0] = pow(vel, 2 * fabs(cfg_->optim.weight_linear_angular_times)) * pow(omega, 2 * fabs(cfg_->optim.weight_linear_angular_times));

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeLinearAngular::computeError() _error[0]=%f\n",_error[0]);
  }

public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace
