#include "Matcher.h"
#include "Uncertainty.h"
#include "CeresEuclideanModel.h"
#include <array>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

namespace covo
{
  class Optimizer
  {
    public:
      Optimizer(
          const nlohmann::json&, 
          const covo::Matcher&, 
          const covo::Uncertainty&);
      bool optimize();
      std::array<double, 7> getTransformation() const;
      std::array<double, 21> getCovariance() const;
      Eigen::Vector3d getTranslation() const;
      Eigen::Quaterniond getRotation() const;

    private:
      const nlohmann::json& COVO_SETTINGS;
      const covo::Matcher& matches;
      const covo::Uncertainty& uncertainty;
      Eigen::Vector3d translation;
      Eigen::Quaterniond rotation;
      std::array<double, 7> transformation;
      std::array<double, 21> covariance;
  };


}


