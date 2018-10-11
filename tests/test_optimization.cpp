#include "ReadArgs.h"
#include "Feature.h"
#include "Matcher.h"
#include "Uncertainty.h"
#include "Optimizer.h"
#include "KeyframeTracker.h"
#include <nlohmann/json.hpp>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

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
  if(read_args(argc, argv, &covo_settings, &rgbdImgFiles, &rgbdImgTimestamps))
    spdlog::get("console")->info("Reading args...");
  else
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
  keyframeTracker.setPreviousTransformation(initP, initQ);

  // Extracting features from RGB images
  spdlog::get("console")->info("Reading rgb&depth images from dataset...");
  int kfIdx = 0;
  for (int i = 0; i < 500; i++)
  {
    // read and extract features from img 1
    cv::String rgbImg1(rgbdImgFiles.at(kfIdx)[0]);
    cv::String depthImg1(rgbdImgFiles.at(kfIdx)[1]);
    cv::Mat rgbImgMat1, depthImgMat1;
    depthImgMat1 = cv::imread(depthImg1, cv::IMREAD_ANYDEPTH);
    //std::cout << cv::format(depthImgMat1, cv::Formatter::FMT_CSV) << std::endl;
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
    matcher.drawRawMatches();
    // TODO find a way to make const ref
    matcher.filterOutliers();
    matcher.drawInlierMatches();
    matcher.drawOutlierMatches();

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
    std::array<double, 7> relTrans = keyframeTracker.getRelativeTransformation();

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

  return 1;
}
