#pragma once

#include "Matcher.h"
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::Point3f> PointMatrixXYZ;
typedef std::vector<cv::Point3f> PointMatrixUVD;
typedef std::vector<Eigen::Matrix<double, 3, 3>> Covariances;
typedef std::vector<Eigen::Matrix<double, 3, 3>> Informations;

namespace covo
{
  class Uncertainty 
  {

    public:
      Uncertainty(const nlohmann::json&, const covo::Matcher&);
      Covariances getCovariances() const;
      Informations getSqrtInformations() const;
      //static Informations combineInformations(const Uncertainty&, const Uncertainty&);


    private:
      void calculateCovariance();
      void calculateNguyenQuadraticOnly(const PointMatrixUVD&, const PointMatrixUVD&);
      std::string exec(std::string);

    private:
      const nlohmann::json& COVO_SETTINGS;
      const covo::Matcher& matches;
      Covariances covariances1;
      Covariances covariances2;
      Covariances sumCovariances;
      Informations sqrtInformations1;
      Informations sqrtInformations2;
      Informations sumSqrtInformations;
  };
}

