#include "KeyframeTracker.h"
#include <Eigen/Dense>
#include <array>
#include <spdlog/spdlog.h>

covo::KeyframeTracker::KeyframeTracker() {}

void covo::KeyframeTracker::calculateRelativeTransformation()
{
  Eigen::Vector3d translation = prevQ.conjugate() * (currentP - prevP);
  Eigen::Quaterniond rotation = prevQ.conjugate() * currentQ;
  std::array<double, 7> transformation;
  transformation[0] = translation[0];
  transformation[1] = translation[1];
  transformation[2] = translation[2];
  transformation[3] = rotation.vec()[0];
  transformation[4] = rotation.vec()[1];
  transformation[5] = rotation.vec()[2];
  transformation[6] = rotation.w();
  spdlog::get("console")->info("Relative Transformation: \n"
      "{} {} {} {} {} {} {}", 
      transformation[0],
      transformation[1],
      transformation[2],
      transformation[3],
      transformation[4],
      transformation[5],
      transformation[6]);
}

void covo::KeyframeTracker::setPreviousTransformation(Eigen::Vector3d p, Eigen::Quaterniond q)
{
  prevP = p;
  prevQ = q;
}

void covo::KeyframeTracker::setCurrentTransformation(Eigen::Vector3d p, Eigen::Quaterniond q)
{
  currentP = p;
  currentQ = q;
}

std::array<double, 7> covo::KeyframeTracker::getRelativeTransformation() const
{
  return relTrans;
}
