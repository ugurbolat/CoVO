#include "Uncertainty.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <spdlog/spdlog.h>

using json = nlohmann::json;


covo::Uncertainty::Uncertainty(const json& _COVO_SETTINGS, const covo::Matcher& _matches) :
  COVO_SETTINGS(_COVO_SETTINGS), matches(_matches)
{
  covo::Uncertainty::calculateCovariance();
}


std::string covo::Uncertainty::exec(std::string cmd)
{
  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
    if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
  }
  return data;
}

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
  const float& fx = COVO_SETTINGS["camera_settings"]["fx"];
  const float& fy = COVO_SETTINGS["camera_settings"]["fy"];
  const float& cx = COVO_SETTINGS["camera_settings"]["cx"];
  const float& cy = COVO_SETTINGS["camera_settings"]["cy"];

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
    covariances2.push_back(cp2);

    // TODO not so mathetically correct thing to do :D
    sumCovariances.push_back((cp1+cp2)/2);

    // using unsupported eigen library to take sqrt()
    Eigen::Matrix<double, 3, 3> unsupportedSqrtInfo = sumCovariances.at(i).inverse().sqrt();
    spdlog::get("console")->trace("unsupported sqrt:"
        "\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}",
        unsupportedSqrtInfo(0,0), unsupportedSqrtInfo(0,1), unsupportedSqrtInfo(0,2),
        unsupportedSqrtInfo(1,0), unsupportedSqrtInfo(1,1), unsupportedSqrtInfo(1,2),
        unsupportedSqrtInfo(2,0), unsupportedSqrtInfo(2,1), unsupportedSqrtInfo(2,2));
    sumSqrtInformations.push_back(unsupportedSqrtInfo);

//    // llt method to sqrt info. fast but not accurate.
//    // TODO why?
//    Eigen::Matrix<double, 3, 3> lltSqrtInfo1 = cp1.inverse().llt().matrixL();
//    spdlog::get("console")->trace("llt l:"
//        "\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}",
//        lltSqrtInfo1(0,0), lltSqrtInfo1(0,1), lltSqrtInfo1(0,2),
//        lltSqrtInfo1(1,0), lltSqrtInfo1(1,1), lltSqrtInfo1(1,2),
//        lltSqrtInfo1(2,0), lltSqrtInfo1(2,1), lltSqrtInfo1(2,2));
//    Eigen::Matrix<double, 3, 3> lltSqrtInfo2 = cp2.inverse().llt().matrixL();
//    spdlog::get("console")->trace("llt l:"
//        "\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}\n{:^5.1f}    {:^5.1f}    {:^5.1f}",
//        lltSqrtInfo2(0,0), lltSqrtInfo2(0,1), lltSqrtInfo2(0,2),
//        lltSqrtInfo2(1,0), lltSqrtInfo2(1,1), lltSqrtInfo2(1,2),
//        lltSqrtInfo2(2,0), lltSqrtInfo2(2,1), lltSqrtInfo2(2,2));
//    sumSqrtInformations.push_back((lltSqrtInfo1 + lltSqrtInfo2)/2);

//    // calling python tested script to validate sqrting
//    std::ostringstream cmd;
//    cmd <<
//      "/home/bolatu/dev/thesis/testbenches/covo_covariance/venv/bin/python "
//      "/home/bolatu/dev/thesis/testbenches/covo_covariance/cov.py "
//      "517.3 516.5 318.6 255.3 10 10 " <<
//      uvd1[i].x << " " << uvd1[i].y << " " << uvd1[i].z << " " << uvd2[i].x << " " << uvd2[i].y << " " << uvd2[i].z << " "
//      << std_d1 <<  " " << std_d2;
//    std::string res = exec(cmd.str());
//    std::cout << res << std::endl;
//
//    std::string buff;
//    std::stringstream ss(res);
//
//    std::vector<double> tokens;
//    while(ss>>buff)
//      tokens.push_back(std::stod(buff));
//    Eigen::Matrix<double, 3, 3> pySqrtInfo;
//    for (int i = 0; i < 3; i++)
//    {
//      for (int j = 0; j < 3; j++)
//      {
//        pySqrtInfo(j,i) = tokens.at(3*i+j);
//      }
//    }
//    spdlog::get("console")->trace("pySqrtInfo:"
//        "\n{: f}    {: f}    {: f}\n{: f}    {: f}    {: f}\n{: f}    {: f}    {: f}",
//        pySqrtInfo(0,0), pySqrtInfo(0,1), pySqrtInfo(0,2),
//        pySqrtInfo(1,0), pySqrtInfo(1,1), pySqrtInfo(1,2),
//        pySqrtInfo(2,0), pySqrtInfo(2,1), pySqrtInfo(2,2));
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
