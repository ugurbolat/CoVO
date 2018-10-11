#include "Uncertainty.h"
#include <Eigen/Dense>
//#include <unsupported/Eigen/MatrixFunctions>
#include <spdlog/spdlog.h>

using json = nlohmann::json;


covo::Uncertainty::Uncertainty(const json& _COVO_SETTINGS, const covo::Matcher& _matches) :
  COVO_SETTINGS(_COVO_SETTINGS), matches(_matches) 
{
  covo::Uncertainty::calculateCovariance();
}


//covo::Uncertainty::Uncertainty(const json& _COVO_SETTINGS, const PointMatrixUVD& pointClouds) : COVO_SETTINGS(_COVO_SETTINGS)
//{
//  covo::Uncertainty::calculateCovariance(pointClouds);
//}

void covo::Uncertainty::calculateCovariance()
{
    const std::string depthType = COVO_SETTINGS["covo_settings"]["uncertainty_depth_model"];

  if (depthType== "NGUYEN_QUADRATIC_ONLY")
  {
    covo::Uncertainty::calculateNguyenQuadraticOnly(matches.getUvd1(), matches.getUvd2());
  }
  else
  {
    spdlog::get("console")->warn("Invalid uncertainty depth model. Default set to NGUYEN_QUADRATIC_ONLY");
    covo::Uncertainty::calculateNguyenQuadraticOnly(matches.getUvd1(), matches.getUvd2());
  }
}

void covo::Uncertainty::calculateNguyenQuadraticOnly(const PointMatrixUVD& uvd1, const PointMatrixUVD& uvd2)
{
  const float& std_u = COVO_SETTINGS["covo_settings"]["uncertainty_u_in_pixels"];
  const float& std_v = COVO_SETTINGS["covo_settings"]["uncertainty_v_in_pixels"];
  auto& fx = COVO_SETTINGS["camera_settings"]["fx"];
  auto& fy = COVO_SETTINGS["camera_settings"]["fy"];
  auto& cx = COVO_SETTINGS["camera_settings"]["cx"];
  auto& cy = COVO_SETTINGS["camera_settings"]["cy"];

  // TODO try to take advantage of matrix multiplications
  for (int i = 0; i < uvd1.size(); i++)
  {
    // calculate covariance for 1. match
    const float& std_d1 = (0.0012 + 0.0019 * (uvd1[i].z-0.4)*(uvd1[i].z-0.4))*10;
    Eigen::Matrix<double, 3, 3> ck;
    ck << std_u, 0, 0,
          0, std_v, 0,
          0, 0, std_d1;
    Eigen::Matrix<double, 3, 3> jp;
    jp << uvd1[i].z/float(fx), 0, (uvd1[i].x/float(fx) - float(cx)/float(fx)),
          0, uvd1[i].z/float(fy), (uvd1[i].y/float(fy) - float(cy)/float(fy)),
          0, 0, 1;

    Eigen::Matrix<double, 3, 3> cp1 = jp * ck * jp.transpose();
    spdlog::get("console")->trace("cp1:"
        "\n{: f}    {: f}    {: f}\n{: f}    {: f}    {: f}\n{: f}    {: f}    {: f}", 
        cp1(0,0), cp1(0,1), cp1(0,2), 
        cp1(1,0), cp1(1,1), cp1(1,2), 
        cp1(2,0), cp1(2,1), cp1(2,2));
    covariances1.push_back(cp1);

    // calculate covariance for 2. match
    const float& std_d2 = (0.0012 + 0.0019 * (uvd2[i].z-0.4)*(uvd2[i].z-0.4))*10;
    ck << std_u, 0, 0,
          0, std_v, 0,
          0, 0, std_d2;
    jp << uvd2[i].z/float(fx), 0, (uvd2[i].x/float(fx) - float(cx)/float(fx)),
          0, uvd2[i].z/float(fy), (uvd2[i].y/float(fy) - float(cy)/float(fy)),
          0, 0, 1;

    Eigen::Matrix<double, 3, 3> cp2 = jp * ck * jp.transpose();
    spdlog::get("console")->trace("cp2:"
        "\n{: f}    {: f}    {: f}\n{: f}    {: f}    {: f}\n{: f}    {: f}    {: f}", 
        cp2(0,0), cp2(0,1), cp2(0,2), 
        cp2(1,0), cp2(1,1), cp2(1,2), 
        cp2(2,0), cp2(2,1), cp2(2,2));
    covariances2.push_back(cp2);

    sumCovariances.push_back(cp1+cp2);

    // calculating information matrix
    Eigen::Matrix<double, 3, 3> sqrtInfo1 = cp1.inverse().llt().matrixL();
    spdlog::get("console")->trace("sqrtInfo1:"
        "\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}", 
        sqrtInfo1(0,0), sqrtInfo1(0,1), sqrtInfo1(0,2), 
        sqrtInfo1(1,0), sqrtInfo1(1,1), sqrtInfo1(1,2), 
        sqrtInfo1(2,0), sqrtInfo1(2,1), sqrtInfo1(2,2));
    sqrtInformations1.push_back(sqrtInfo1);

    Eigen::Matrix<double, 3, 3> sqrtInfo2 = cp2.inverse().llt().matrixL();
    spdlog::get("console")->trace("sqrtInfo2:"
        "\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}", 
        sqrtInfo2(0,0), sqrtInfo2(0,1), sqrtInfo2(0,2), 
        sqrtInfo2(1,0), sqrtInfo2(1,1), sqrtInfo2(1,2), 
        sqrtInfo2(2,0), sqrtInfo2(2,1), sqrtInfo2(2,2));
    sqrtInformations2.push_back(sqrtInfo2);

    sumSqrtInformations.push_back(sqrtInfo1+sqrtInfo2);
  }
}


Covariances covo::Uncertainty::getCovariances() const
{
  return sumCovariances;
}

Informations covo::Uncertainty::getSqrtInformations() const
{
  return sumSqrtInformations;
}
