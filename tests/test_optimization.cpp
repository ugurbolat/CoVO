#include "ReadArgs.h"
#include "Feature.h"
#include "Matcher.h"
#include "Uncertainty.h"
#include "Optimizer.h"
#include "KeyframeTracker.h"
#include <nlohmann/json.hpp>
#include <arg/args.hxx>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Dense>


using json = nlohmann::json;


int main(int argc, char** argv)
{
  json covo_settings;
  std::vector<std::array<std::string, 2>> rgbdImgFiles;
  std::vector<std::array<double, 2>> rgbdImgTimestamps;

  // Reading user arguments
  if(!read_args(argc, argv, &covo_settings, &rgbdImgFiles, &rgbdImgTimestamps))
  {
    spdlog::get("console")->critical("Reading args FAILED!");
    return 0;
  }

  // Adding const-ness to global settings
  const json& COVO_SETTINGS = covo_settings;

  // initing keyframe tracker
  covo::KeyframeTracker keyframeTracker;
  Eigen::Vector3d initP(0, 0, 0);
  Eigen::Quaterniond initQ(1, 0, 0, 0);
  std::array<double, 21> initCov = {0};
  keyframeTracker.setPreviousTransformation(initP, initQ);

  // output file for relative transformation results
  std::stringstream ssOutputDir;
  ssOutputDir << COVO_SETTINGS["output_dir"].get<std::string>();
  std::ofstream ofRelTrans(ssOutputDir.str() + "covo_rel_trans.txt", std::ios_base::trunc);
  ofRelTrans.setf(std::ios_base::fixed);
  if (ofRelTrans.fail())
  {
    spdlog::get("console")->error("Cannot open covo_rel_trans.txt to write!");
  }
  ofRelTrans << "# CoVO Relative Transformations Results [px,py,pz,qx,qy,qz,qw]\n";

  std::ofstream ofRelCov(ssOutputDir.str() + "covo_rel_cov.txt", std::ios_base::trunc);
  ofRelCov.setf(std::ios_base::fixed);
  if (ofRelCov.fail())
  {
    spdlog::get("console")->error("Cannot open covo_rel_cov.txt to write!");
  }
  // TODO fill the formating
  ofRelCov << "# CoVO Relative Covariance Results. Only upper triangluar matrix is given\n";

  std::ofstream ofTrajTrans(ssOutputDir.str() + "covo_trajectory.txt", std::ios_base::trunc);
  ofTrajTrans.setf(std::ios_base::fixed);
  if (ofTrajTrans.fail())
  {
    spdlog::get("console")->error("Cannot open covo_trajectory.txt to write!");
  }
  ofTrajTrans << "# CoVO Concatenated Transformations Results [px,py,pz,qx,qy,qz,qw]\n";

  // Concatenate relative transformations
  Eigen::Vector3d trajectoryP;
  trajectoryP <<
    COVO_SETTINGS["initial_pose"][0],
    COVO_SETTINGS["initial_pose"][1],
    COVO_SETTINGS["initial_pose"][2];
  Eigen::Quaterniond trajectoryQ;
  trajectoryQ.w() = COVO_SETTINGS["initial_pose"][6];
  trajectoryQ.vec() <<
    COVO_SETTINGS["initial_pose"][3],
    COVO_SETTINGS["initial_pose"][4],
    COVO_SETTINGS["initial_pose"][5];
  ofTrajTrans << std::setprecision(6) << rgbdImgTimestamps.at(0)[0] << " ";
  ofTrajTrans
    << trajectoryP[0] << " "
    << trajectoryP[1] << " "
    << trajectoryP[2] << " "
    << trajectoryQ.vec()[0] << " "
    << trajectoryQ.vec()[1] << " "
    << trajectoryQ.vec()[2] << " "
    << trajectoryQ.w() << "\n";

  // Loop related stuff
  int noImgPairsToProcess;
  if (COVO_SETTINGS["no_img_pairs_to_process"] == -1)
  {
    noImgPairsToProcess = rgbdImgFiles.size();
  }
  else
  {
    noImgPairsToProcess = COVO_SETTINGS["no_img_pairs_to_process"];
  }
  int kfIdx = 0;
  for (int i = 0; i < noImgPairsToProcess-1; i++)
  {
    // read and extract features from img 1
    cv::String rgbImg1(rgbdImgFiles.at(kfIdx)[0]);
    cv::String depthImg1(rgbdImgFiles.at(kfIdx)[1]);
    cv::Mat rgbImgMat1, depthImgMat1;
    depthImgMat1 = cv::imread(depthImg1, cv::IMREAD_ANYDEPTH);
    rgbImgMat1 = cv::imread(rgbImg1, cv::IMREAD_GRAYSCALE);
    if (rgbImg1.empty())
    {
      spdlog::get("console")->critical("Could not open or find the rgb image!");
      return 0;
    }
    spdlog::get("console")->info("Detecting features in {:.6f} RGB image...", rgbdImgTimestamps.at(kfIdx)[0]);
    covo::Feature feature1(COVO_SETTINGS, rgbImgMat1);
    feature1.detectFeatures();

    // read and extract features from img 2
    cv::String rgbImg2(rgbdImgFiles.at(i+1)[0]);
    cv::String depthImg2(rgbdImgFiles.at(i+1)[1]);
    cv::Mat rgbImgMat2, depthImgMat2;
    depthImgMat2 = cv::imread(depthImg2, cv::IMREAD_ANYDEPTH);
    rgbImgMat2 = cv::imread(rgbImg2, cv::IMREAD_GRAYSCALE);
    if (rgbImg2.empty())
    {
      spdlog::get("console")->critical("Could not open or find the rgb image!");
      return 0;
    }
    spdlog::get("console")->info("Detecting features in {:6f} RGB image...", rgbdImgTimestamps.at(i+1)[0]);
    covo::Feature feature2(COVO_SETTINGS, rgbImgMat2);
    feature2.detectFeatures();

    // matching features
    spdlog::get("console")->info("Matching features...");
    covo::Matcher matcher(COVO_SETTINGS,
        rgbImgMat1, depthImgMat1,
        rgbImgMat2, depthImgMat2,
        feature1, feature2);
    matcher.findMatches();
    matcher.filterOutliers();
    if (COVO_SETTINGS["draw_matches"])
    {
      //matcher.drawRawMatches();
      matcher.drawInlierMatches();
      //matcher.drawOutlierMatches();
    }
    // calculate uncertainty for point clouds
    spdlog::get("console")->info("Calculating uncertainty for point clouds...");
    covo::Uncertainty uncertainty(COVO_SETTINGS, matcher);

    // optimize!
    spdlog::get("console")->info("Optimizing...");
    covo::Optimizer optimizer(COVO_SETTINGS, matcher, uncertainty);
    optimizer.optimize();

    // calculate frame to frame relative transformation from keyframe
    Eigen::Vector3d currentP = optimizer.getTranslation();
    Eigen::Quaterniond currentQ = optimizer.getRotation();
    // saving current transformation result to keyframe tracker
    keyframeTracker.setCurrentTransformation(currentP, currentQ);
    // now getting relative transformation btw prev and current transformation
    keyframeTracker.calculateRelativeTransformation();
    std::array<double, 7> relTrans = keyframeTracker.getRelativeTransformation();

    // write out the results
    ofRelTrans << std::setprecision(6) << rgbdImgTimestamps.at(i+1)[0] << " ";
    for (auto res : relTrans)
    {
      ofRelTrans << res << " ";
    }
    ofRelTrans << "\n";
    ofRelCov << std::setprecision(10) << rgbdImgTimestamps.at(i+1)[0] << " ";
    std::array<double, 21> relCov = optimizer.getCovariance();
    for (auto res : relCov)
    {
      ofRelCov << res << " ";
    }
    ofRelCov << "\n";
    if (COVO_SETTINGS["trajectory"])
    {
      trajectoryP = trajectoryP + trajectoryQ * keyframeTracker.getTranslation();
      trajectoryQ = trajectoryQ * keyframeTracker.getRotation();
      ofTrajTrans << std::setprecision(6) << rgbdImgTimestamps.at(i+1)[0] << " ";
      ofTrajTrans
        << trajectoryP[0] << " "
        << trajectoryP[1] << " "
        << trajectoryP[2] << " "
        << trajectoryQ.vec()[0] << " "
        << trajectoryQ.vec()[1] << " "
        << trajectoryQ.vec()[2] << " "
        << trajectoryQ.w() << "\n";
    }

    // preparing keyframe tracker for the next iter
    keyframeTracker.setPreviousTransformation(currentP, currentQ);

    // check for sufficient matching
    if (!matcher.isSufficientNoMatches())
    {
      // TODO you might want to do matching again with updated keyframe?
      spdlog::get("console")->warn("Insufficient no of matches. Changing ref keyframe!");
      kfIdx = i;
      keyframeTracker.setPreviousTransformation(initP, initQ);
    }
  }

  ofRelTrans.close();
  ofTrajTrans.close();
  ofRelCov.close();

  return 1;
}
