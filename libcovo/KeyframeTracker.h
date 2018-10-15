#pragma once

#include <array>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

namespace covo
{
  class KeyframeTracker
  {
    public:
      KeyframeTracker();
      // TODO find a way to make const ref
      void setPreviousTransformation(Eigen::Vector3d, Eigen::Quaterniond);
      void setCurrentTransformation(Eigen::Vector3d, Eigen::Quaterniond);
      std::array<double, 7> getRelativeTransformation() const; 
      void calculateRelativeTransformation();
      Eigen::Vector3d getTranslation() const;
      Eigen::Quaterniond getRotation() const;

    private:
     Eigen::Vector3d prevP;
     Eigen::Quaterniond prevQ;
     Eigen::Vector3d currentP;
     Eigen::Quaterniond currentQ;
     std::array<double, 7> relTrans;
     Eigen::Vector3d translation;
     Eigen::Quaterniond rotation;

  };
}
