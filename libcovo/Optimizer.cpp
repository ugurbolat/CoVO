#include "Optimizer.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

covo::Optimizer::Optimizer(
    const json& _COVO_SETTINGS, 
    const covo::Matcher& _matches,
    const covo::Uncertainty& _uncertainty) : 
  COVO_SETTINGS(_COVO_SETTINGS), matches(_matches), uncertainty(_uncertainty) {}


bool covo::Optimizer::optimize()
{
  // getting point clouds' coordinates and uncertainties
  const auto& pointClouds1 = matches.getXyz1();
  const auto& pointClouds2 = matches.getXyz2();
  const auto& info = uncertainty.getSqrtInformations();

  // initial guess
  Eigen::Vector3d p(0, 0, 0);
  Eigen::Quaterniond q(1, 0, 0, 0);

  ceres::Problem problem;
  // TODO smart pointer?
  ceres::LocalParameterization *lpQuaternion = new ceres::EigenQuaternionParameterization;

  for (int i = 0; i < pointClouds1.size(); i++) 
  {
    ceres::CostFunction* cost_function = EuclideanErrorResidual::Create(
            pointClouds1.at(i).x, pointClouds1.at(i).y, pointClouds1.at(i).z,
            pointClouds2.at(i).x, pointClouds2.at(i).y, pointClouds2.at(i).z,
            info.at(i));

    // ceres::LossFunction* loss_function = new ceres::HuberLoss(5.991);
    ceres::LossFunction* loss_function = NULL;
    problem.AddResidualBlock(
        cost_function, loss_function, 
        p.data(), 
        q.coeffs().data());

    // parameterize rotation for quaternion
    problem.SetParameterization(q.coeffs().data(), lpQuaternion);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  if (COVO_SETTINGS["log_level"] != "warn" && 
      COVO_SETTINGS["log_level"] != "error" && 
      COVO_SETTINGS["log_level"] != "critical")
    options.minimizer_progress_to_stdout = true;
  else
    options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 10;
  ceres::Solver::Summary summary;

  spdlog::get("console")->info("Optimization started...");

  ceres::Solve(options, &problem, &summary);
//    options.function_tolerance = 1.0e-10;
//    options.parameter_tolerance = 1.0e-10;
//    options.gradient_tolerance = 1.0e-10;
//    ceres::Solver::Summary summary;
//    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.FullReport() << "\n";

  // save optimized transformation result
  transformation[0] = p.data()[0];
  transformation[1] = p.data()[1];
  transformation[2] = p.data()[2];
  transformation[3] = q.vec()[0];
  transformation[4] = q.vec()[1];
  transformation[5] = q.vec()[2];
  transformation[6] = q.w();

  spdlog::get("console")->info("Optimized Transformation: \n"
      "{0} {1} {2} {3} {4} {5} {6}", 
      transformation[0],
      transformation[1],
      transformation[2],
      transformation[3],
      transformation[4],
      transformation[5],
      transformation[6]);


  // calc covariance
  ceres::Covariance::Options cov_options;
  cov_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance ceresCov(cov_options);

  std::vector<std::pair<const double*, const double*>> covBlocks;
  covBlocks.push_back(std::make_pair(p.data(), p.data()));
  covBlocks.push_back(std::make_pair(p.data(), q.coeffs().data()));
  covBlocks.push_back(std::make_pair(q.coeffs().data(), q.coeffs().data()));

  CHECK(ceresCov.Compute(covBlocks, &problem));

  double covariance_pp[3*3];
  double covariance_pq[3*3];
  double covariance_qq[3*3];

  ceresCov.GetCovarianceBlock(p.data(), p.data(), covariance_pp);
  ceresCov.GetCovarianceBlockInTangentSpace(p.data(), q.coeffs().data(), covariance_pq);
  ceresCov.GetCovarianceBlockInTangentSpace(q.coeffs().data(), q.coeffs().data(), covariance_qq);

  covariance[0] = covariance_pp[0];
  covariance[1] = covariance_pp[1];
  covariance[2] = covariance_pp[2];
  covariance[3] = covariance_pq[0];
  covariance[4] = covariance_pq[1];
  covariance[5] = covariance_pq[2];

  covariance[6] = covariance_pp[4];
  covariance[7] = covariance_pp[5];
  covariance[8] = covariance_pq[3];
  covariance[9] = covariance_pq[4];
  covariance[10] = covariance_pq[5];

  covariance[11] = covariance_pp[8];
  covariance[12] = covariance_pq[6];
  covariance[13] = covariance_pq[7];
  covariance[14] = covariance_pq[8];

  covariance[15] = covariance_qq[0];
  covariance[16] = covariance_qq[1];
  covariance[17] = covariance_qq[2];

  covariance[18] = covariance_qq[4];
  covariance[19] = covariance_qq[5];

  covariance[20] = covariance_qq[8];

  spdlog::get("console")->info("Optimized Covariance: \n"
      "{: .5e}    {: .5e}    {: .5e}    {: .5e}    {: .5e}    {: .5e}\n"
      "{:>12}    {: .5e}    {: .5e}    {: .5e}    {: .5e}    {: .5e}\n"
      "{:>12}    {:>12}    {: .5e}    {: .5e}    {: .5e}    {: .5e}\n"
      "{:>12}    {:>12}    {:>12}    {: .5e}    {: .5e}    {: .5e}\n"
      "{:>12}    {:>12}    {:>12}    {:>12}    {: .5e}    {: .5e}\n"
      "{:>12}    {:>12}    {:>12}    {:>12}    {:>12}    {: .5e}",
      covariance_pp[0], covariance_pp[1], covariance_pp[2], covariance_pq[0], covariance_pq[1], covariance_pq[2],
      "",covariance_pp[4], covariance_pp[5], covariance_pq[3], covariance_pq[4], covariance_pq[5],
      "","",covariance_pp[8], covariance_pq[6], covariance_pq[7], covariance_pq[8],
      "","","",covariance_qq[0], covariance_qq[1], covariance_qq[2],
      "","","","",covariance_qq[4], covariance_qq[5],
      "","","","","",covariance_qq[8]
      );
}


std::array<double, 7> covo::Optimizer::getTransformation() const
{
  return transformation;
}

std::array<double, 21> covo::Optimizer::getCovariance() const
{
  return covariance;
}
