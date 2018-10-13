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
  relTrans[0] = translation[0];
  relTrans[1] = translation[1];
  relTrans[2] = translation[2];
  relTrans[3] = rotation.vec()[0];
  relTrans[4] = rotation.vec()[1];
  relTrans[5] = rotation.vec()[2];
  relTrans[6] = rotation.w();
  spdlog::get("console")->info("Relative Transformation: \n"
      "{} {} {} {} {} {} {}", 
      relTrans[0],
      relTrans[1],
      relTrans[2],
      relTrans[3],
      relTrans[4],
      relTrans[5],
      relTrans[6]);
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
